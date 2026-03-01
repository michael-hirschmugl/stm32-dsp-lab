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
 */
#define SIG_HALF_LEN   (SIG_DMA_BUF_LEN / 2u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ETH_HandleTypeDef heth;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/**
 * @brief CPU working buffer (one half-buffer of audio samples).
 *
 * cpu_block[]:
 *   receives a stable snapshot of the DMA input buffer half (sig_dma_buf[]).
 *
 * cpu_block_processed[]:
 *   will later carry processed samples. For now it's copy-through.
 */
static int16_t cpu_block[SIG_HALF_LEN];
static int16_t cpu_block_processed[SIG_HALF_LEN];

/** @brief Counter of processed half-blocks (for debugging/telemetry). */
static volatile uint32_t cpu_blocks = 0;

/**
 * @brief Convert signed 16-bit audio sample to unsigned 12-bit DAC value.
 *
 * Input range:  int16 [-32768 .. 32767]
 * DAC expects:  12-bit unsigned [0 .. 4095] (right-aligned in DHR12R1)
 *
 * Mapping:
 *   - scale down 16->12 bits by shifting (>>4)
 *   - add midscale offset (2048) to center around Vref/2
 *   - clamp to [0, 4095]
 *
 * Note:
 *   - This is a simple truncating conversion. For better quality you can
 *     apply dithering or rounding later.
 */
static inline uint16_t s16_to_dac_u12(int16_t s)
{
  int32_t v = ((int32_t)s >> 4) + 2048;  // 16->12 bit + midscale offset
  if (v < 0)    v = 0;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

int main(void)
{
  /* Reset of all peripherals, init Flash interface and SysTick. */
  HAL_Init();

  /* Configure the system clock (PLL, bus prescalers, etc.). */
  SystemClock_Config();

  /* Initialize peripherals configured by CubeMX. */
  MX_GPIO_Init();
  //MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  /* USER CODE BEGIN 2 */
#if SIG_USE_TIM2_DMA_TEST
  /**
   * Start the audio pipeline:
   *  - config TIM2 @ SIG_FS_HZ, TRGO=Update
   *  - start input DMA (P2M) into sig_dma_buf[]
   *  - start DAC + output DMA (M2P) from dac_dma_buf[] (if enabled)
   */
  SigDma_TestInit();
  SigDma_TestStart();
#else
  // Alternative mode (not used currently)
  //SigGen_Init();
  //SigGen_Start();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
#if SIG_USE_TIM2_DMA_TEST
    /**
     * We poll the HT/TC counters which are incremented from DMA IRQ handlers.
     * This avoids doing any heavy work in interrupt context.
     *
     * Lockstep rule:
     *  - When input HT happens: first half of sig_dma_buf[] is stable and can be read.
     *    DMA is currently filling the second half.
     *  - When input TC happens: second half is stable and can be read.
     *    DMA is currently filling the first half again.
     *
     * We write to the corresponding half of dac_dma_buf[]. Because output DMA is
     * circular as well, writing the "inactive half" is safe.
     */
    static uint32_t last_ht = 0;
    static uint32_t last_tc = 0;

    /* ---- Half-transfer: consume first half, produce first half ---- */
    if (sig_dma_ht_count != last_ht) {
      last_ht = sig_dma_ht_count;

      /* 1) Snapshot stable input half into CPU buffer */
      memcpy(cpu_block, &sig_dma_buf[0], SIG_HALF_LEN * sizeof(int16_t));

      /* 2) DSP placeholder: copy-through (replace with real processing later) */
      memcpy(cpu_block_processed, cpu_block, SIG_HALF_LEN * sizeof(int16_t));

#if SIG_ENABLE_DAC_OUT
      /* 3) Fill output buffer half for DAC DMA (first half) */
      for (uint32_t i = 0; i < SIG_HALF_LEN; i++) {
        dac_dma_buf[i] = s16_to_dac_u12(cpu_block_processed[i]);
      }
#endif

      cpu_blocks++;
      /* Debug visibility: toggle LED on each processed half-block */
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    }

    /* ---- Transfer-complete: consume second half, produce second half ---- */
    if (sig_dma_tc_count != last_tc) {
      last_tc = sig_dma_tc_count;

      /* 1) Snapshot stable input half into CPU buffer */
      memcpy(cpu_block, &sig_dma_buf[SIG_HALF_LEN], SIG_HALF_LEN * sizeof(int16_t));

      /* 2) DSP placeholder: copy-through (replace with real processing later) */
      memcpy(cpu_block_processed, cpu_block, SIG_HALF_LEN * sizeof(int16_t));

#if SIG_ENABLE_DAC_OUT
      /* 3) Fill output buffer half for DAC DMA (second half) */
      for (uint32_t i = 0; i < SIG_HALF_LEN; i++) {
        dac_dma_buf[SIG_HALF_LEN + i] = s16_to_dac_u12(cpu_block_processed[i]);
      }
#endif

      cpu_blocks++;
      /* Debug visibility: toggle LED on each processed half-block */
      HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    }

    /* Optional breakpoint after N blocks (useful for dumping buffers) */
    //if (cpu_blocks == 500) { __BKPT(0); }

#else
    /* Alternative mode (not used currently) */
#endif
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * Note:
  *   This is CubeMX-generated. It configures the PLL and bus prescalers.
  *   The timer clock assumptions used in signal_gen.c depend on these settings.
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
  *   This is CubeMX-generated. We use LD2 for debug visibility.
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