/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (STM32F207ZG, lockstep audio pipeline)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * Overview
  * --------
  * This project implements a "lockstep" audio pipeline driven by TIM2 @ 44.1 kHz.
  *
  *  - Input side (DMA P2M, circular):
  *      TIM2 update event triggers DMA to copy a "sample register" (TIM2->CCR1)
  *      into sig_dma_buf[] (int16) at SIG_FS_HZ. Half-transfer (HT) and
  *      transfer-complete (TC) interrupts increment counters:
  *        sig_dma_ht_count, sig_dma_tc_count
  *
  *  - CPU side (block-based):
  *      In the main loop we poll the HT/TC counters. When a half-buffer is ready,
  *      we copy that half into cpu_block[], optionally process into
  *      cpu_block_processed[], and then feed the output side.
  *
  *  - Output side (DAC + DMA M2P, circular):
  *      TIM2 TRGO triggers DAC updates, and the DAC requests DMA to fetch samples
  *      from dac_dma_buf[] (uint16, 12-bit right-aligned) in circular mode.
  *
  * Design notes
  * ------------
  *  - No heavy DSP work is performed inside interrupts. Interrupts only set flags
  *    (via counters). Block processing is done in the main loop.
  *  - This lockstep approach is deterministic: each input half-buffer maps to an
  *    output half-buffer.
  *  - In later steps, cpu_block_processed[] will contain the DSP output.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "signal_gen.h"
#include "arm_math.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/**
 * @brief Half-buffer length in samples.
 *
 * sig_dma_buf[] and dac_dma_buf[] are sized SIG_DMA_BUF_LEN and used as a
 * double buffer:
 *   - first half:  [0 .. SIG_HALF_LEN-1]
 *   - second half: [SIG_HALF_LEN .. SIG_DMA_BUF_LEN-1]
 *
 * The DMA driver raises two interrupts:
 *   - HT (half transfer): first half stable, second half currently filling
 *   - TC (transfer complete): second half stable, first half currently filling
 */
#define SIG_HALF_LEN   (SIG_DMA_BUF_LEN / 2u)

/* ===== FFT configuration (block FFT) =====
 * We run a complex FFT over one "stable half" of the DMA buffer.
 * With SIG_DMA_BUF_LEN = 512, SIG_HALF_LEN = 256 → FFT_N = 256.
 */
#define FFT_N           SIG_HALF_LEN
#define FFT_BINS        (FFT_N / 2u)          /* unique bins for real input */
#define FFT_MAG_COUNT   (FFT_BINS + 1u)       /* DC..Nyquist (inclusive) */

/* ===== UART streaming configuration =====
 * We transmit FFT magnitudes to the host at a low rate (10 Hz).
 * The FFT itself may run per half-buffer; the UART rate is throttled.
 */
#define FFT_TX_HZ           10u
#define FFT_TX_PERIOD_MS    (1000u / FFT_TX_HZ)

/* Simple binary framing to allow robust re-synchronization on the host side. */
#define UART_SYNC0          0xA5u
#define UART_SYNC1          0x5Au

/* Frame format (little-endian):
 *   [0]   0xA5
 *   [1]   0x5A
 *   [2..5]  uint32 frame_counter
 *   [6..7]  uint16 count (= FFT_MAG_COUNT)
 *   [8..]   payload: count * int16 magnitudes (little-endian)
 *   [end-2..end-1] uint16 checksum (sum of payload bytes modulo 65536)
 *
 * Total bytes = 2 + 4 + 2 + 2*count + 2
 */
#define FFT_TX_MAX_BYTES (2u + 4u + 2u + (2u * FFT_MAG_COUNT) + 2u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ETH_HandleTypeDef heth;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* =============================================================================
 * CPU working buffers
 * =============================================================================
 * cpu_block[] is a stable snapshot of one half of sig_dma_buf[].
 * cpu_block_processed[] is currently pass-through, reserved for later DSP.
 */
static int16_t cpu_block[SIG_HALF_LEN];
static int16_t cpu_block_processed[SIG_HALF_LEN];

/* =============================================================================
 * FFT state & debug buffers
 * =============================================================================
 * CMSIS-DSP radix-4 complex FFT in Q15.
 *
 * Input is real-valued audio (int16), mapped to complex by setting Im = 0.
 * Output is complex bins; we compute magnitudes (Q15).
 *
 * NOTE ON SCALING:
 * - CMSIS Q15 FFT implementations typically apply internal scaling to avoid
 *   overflow. Magnitudes are "relative" and are mainly useful for visualization
 *   unless you calibrate a scale factor.
 */
static arm_cfft_radix4_instance_q15 s_fft;

/* Interleaved complex buffer: Re0, Im0, Re1, Im1, ... (in-place FFT). */
volatile q15_t g_fft_buf[2u * FFT_N];

/* Magnitude spectrum: DC..Nyquist (FFT_MAG_COUNT samples). */
volatile q15_t g_fft_mag[FFT_MAG_COUNT];

/* Frame counter incremented whenever we compute a new spectrum. */
volatile uint32_t g_fft_frame = 0;

/* =============================================================================
 * UART streaming buffers/state
 * =============================================================================
 * We build a binary frame in g_fft_tx_buf and send it periodically.
 * The host can resynchronize using the two-byte sync marker.
 */
static uint8_t  g_fft_tx_buf[FFT_TX_MAX_BYTES];
static uint32_t g_fft_last_tx_ms = 0;

/* Debug counter: number of half-blocks processed. */
static volatile uint32_t cpu_blocks = 0;

/* =============================================================================
 * DAC helper
 * =============================================================================
 * Converts signed 16-bit audio samples to 12-bit unsigned DAC values.
 * This is only used when SIG_ENABLE_DAC_OUT is enabled.
 */
static inline uint16_t s16_to_dac_u12(int16_t s)
{
  int32_t v = ((int32_t)s >> 4) + 2048;  /* 16->12 bit + midscale offset */
  if (v < 0)    v = 0;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}

/**
 * @brief Simple checksum: sum of bytes modulo 65536.
 *
 * This is not cryptographic; it is only meant to catch obvious corruption and
 * allow the host to drop bad frames. For USB-UART links you may omit it.
 */
static uint16_t checksum16(const uint8_t *p, uint32_t n)
{
  uint32_t s = 0;
  for (uint32_t i = 0; i < n; i++) {
    s += p[i];
  }
  return (uint16_t)(s & 0xFFFFu);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* static void MX_ETH_Init(void); */  /* Not used */
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */

/**
 * @brief Compute FFT magnitude spectrum for a real input block.
 *
 * @param[in]  in_s16   Pointer to FFT_N signed 16-bit samples.
 *
 * This function:
 *  1) Copies real input into an interleaved complex buffer (Im = 0)
 *  2) Executes an in-place complex FFT (radix-4)
 *  3) Computes magnitudes for bins 0..N/2 (inclusive)
 *  4) Increments g_fft_frame
 *
 * Notes:
 *  - arm_cmplx_mag_q15 expects complex interleaved input length = numSamples.
 *    Here, we pass FFT_MAG_COUNT which corresponds to the first N/2+1 bins.
 */
static void FFT_ProcessBlock_Q15(const int16_t *in_s16)
{
  /* Real -> complex (Q15) */
  for (uint32_t i = 0; i < FFT_N; i++) {
    g_fft_buf[2u * i + 0u] = (q15_t)in_s16[i]; /* Re */
    g_fft_buf[2u * i + 1u] = 0;               /* Im */
  }

  /* In-place complex FFT */
  arm_cfft_radix4_q15(&s_fft, (q15_t *)g_fft_buf);

  /* Magnitude for DC..Nyquist */
  arm_cmplx_mag_q15((q15_t *)g_fft_buf, (q15_t *)g_fft_mag, FFT_MAG_COUNT);

  g_fft_frame++;
}

/**
 * @brief Send the most recent FFT magnitude spectrum to the host via UART.
 *
 * The payload is the raw q15_t magnitudes (int16) in little-endian order.
 * The host script can rescale by dividing by 32768.0.
 *
 * This function is intentionally blocking and low-rate (10 Hz). For higher
 * throughput you would switch to interrupt or DMA-based UART TX.
 */
static void FFT_SendSpectrum_UART10Hz(void)
{
  const uint32_t now = HAL_GetTick();
  if ((now - g_fft_last_tx_ms) < FFT_TX_PERIOD_MS) {
    return;
  }
  g_fft_last_tx_ms = now;

  uint32_t idx = 0;

  /* Sync */
  g_fft_tx_buf[idx++] = UART_SYNC0;
  g_fft_tx_buf[idx++] = UART_SYNC1;

  /* Frame counter (little-endian) */
  const uint32_t fc = g_fft_frame;
  g_fft_tx_buf[idx++] = (uint8_t)(fc & 0xFFu);
  g_fft_tx_buf[idx++] = (uint8_t)((fc >> 8) & 0xFFu);
  g_fft_tx_buf[idx++] = (uint8_t)((fc >> 16) & 0xFFu);
  g_fft_tx_buf[idx++] = (uint8_t)((fc >> 24) & 0xFFu);

  /* Count (little-endian) */
  const uint16_t count = (uint16_t)FFT_MAG_COUNT;
  g_fft_tx_buf[idx++] = (uint8_t)(count & 0xFFu);
  g_fft_tx_buf[idx++] = (uint8_t)((count >> 8) & 0xFFu);

  /* Payload: count * int16 magnitudes (little-endian) */
  for (uint32_t k = 0; k < FFT_MAG_COUNT; k++) {
    const int16_t v = (int16_t)g_fft_mag[k];
    g_fft_tx_buf[idx++] = (uint8_t)(v & 0xFF);
    g_fft_tx_buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
  }

  /* Checksum over payload bytes only */
  const uint32_t payload_len = 2u * FFT_MAG_COUNT;
  const uint16_t cs = checksum16(&g_fft_tx_buf[2u + 4u + 2u], payload_len);
  g_fft_tx_buf[idx++] = (uint8_t)(cs & 0xFFu);
  g_fft_tx_buf[idx++] = (uint8_t)((cs >> 8) & 0xFFu);

  (void)HAL_UART_Transmit(&huart3, g_fft_tx_buf, (uint16_t)idx, 50);
}

/* USER CODE END PFP */

int main(void)
{
  /* Reset peripherals, init Flash interface and SysTick. */
  HAL_Init();

  /* Configure system clock. */
  SystemClock_Config();

  /* Initialize peripherals configured by CubeMX. */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  /* USER CODE BEGIN 2 */

  /* Initialize CMSIS-DSP FFT instance:
   * - FFT length: FFT_N
   * - ifftFlag: 0 (forward FFT)
   * - bitReverseFlag: 1 (bit reversal enabled)
   */
  arm_cfft_radix4_init_q15(&s_fft, FFT_N, 0, 1);

#if SIG_USE_TIM2_DMA_TEST
  /* Start lockstep DMA-based audio pipeline (input + optional output). */
  SigDma_TestInit();
  SigDma_TestStart();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
#if SIG_USE_TIM2_DMA_TEST

    /* We poll HT/TC counters updated by DMA IRQs. */
    static uint32_t last_ht = 0;
    static uint32_t last_tc = 0;

    /* -----------------------------------------------------------------------
     * Half-transfer: first half of sig_dma_buf[] is stable
     * --------------------------------------------------------------------- */
    if (sig_dma_ht_count != last_ht) {
      last_ht = sig_dma_ht_count;

      /* Snapshot stable input half into CPU buffer */
      memcpy(cpu_block, &sig_dma_buf[0], SIG_HALF_LEN * sizeof(int16_t));

      /* Compute FFT spectrum for this block */
      FFT_ProcessBlock_Q15(cpu_block);

      /* Placeholder DSP: currently pass-through */
      memcpy(cpu_block_processed, cpu_block, SIG_HALF_LEN * sizeof(int16_t));

#if SIG_ENABLE_DAC_OUT
      /* Update DAC DMA first half */
      for (uint32_t i = 0; i < SIG_HALF_LEN; i++) {
        dac_dma_buf[i] = s16_to_dac_u12(cpu_block_processed[i]);
      }
#endif

      cpu_blocks++;
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    }

    /* -----------------------------------------------------------------------
     * Transfer-complete: second half of sig_dma_buf[] is stable
     * --------------------------------------------------------------------- */
    if (sig_dma_tc_count != last_tc) {
      last_tc = sig_dma_tc_count;

      /* Snapshot stable input half into CPU buffer */
      memcpy(cpu_block, &sig_dma_buf[SIG_HALF_LEN], SIG_HALF_LEN * sizeof(int16_t));

      /* Compute FFT spectrum for this block */
      FFT_ProcessBlock_Q15(cpu_block);

      /* Placeholder DSP: currently pass-through */
      memcpy(cpu_block_processed, cpu_block, SIG_HALF_LEN * sizeof(int16_t));

#if SIG_ENABLE_DAC_OUT
      /* Update DAC DMA second half */
      for (uint32_t i = 0; i < SIG_HALF_LEN; i++) {
        dac_dma_buf[SIG_HALF_LEN + i] = s16_to_dac_u12(cpu_block_processed[i]);
      }
#endif

      cpu_blocks++;
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    }

    /* Throttled spectrum streaming to PC (10 Hz) */
    FFT_SendSpectrum_UART10Hz();

#else
    /* Alternative mode (not used) */
#endif
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * Note:
  *   CubeMX-generated. Timing assumptions in signal_gen.c depend on these settings.
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator config */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Bus clocks config */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  *
  * Note:
  *   CubeMX-generated. We use LD2 for debug visibility.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /* USER button */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /* LEDs */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USB power switch */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* USB overcurrent */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */