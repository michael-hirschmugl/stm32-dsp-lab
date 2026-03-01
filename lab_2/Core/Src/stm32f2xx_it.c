/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @brief   Interrupt Service Routines (ISRs)
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
  * Project-specific ISR overview
  * ----------------------------
  * This application uses a timer-driven audio pipeline:
  *
  *   - TIM2 runs at SIG_FS_HZ (e.g., 44.1 kHz).
  *   - TIM2 update events are used in two ways:
  *       (1) Generate the next sample in TIM2_IRQHandler (SigGen_OnTick()).
  *       (2) Trigger DMA requests (UDE) to pull the sample register into a
  *           circular buffer (P2M input path).
  *   - DAC output is also triggered from TIM2_TRGO, and the DAC requests DMA
  *     (M2P output path) from a circular output buffer.
  *
  * Interrupt policy
  * ----------------
  * ISRs should do the minimum work necessary:
  *   - Clear flags
  *   - Update counters / flags
  *   - Call small, bounded handlers
  *
  * Heavy DSP work is intentionally done in the main loop on half-buffer blocks.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "signal_gen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */

  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
    /* NMI: typically used for clock failure or critical faults. */
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */

  while (1) {
    /* HardFault: inspect SCB->HFSR/CFSR/MMFAR/BFAR in debugger. */
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */

  while (1) {
    /* MemManage: typically invalid memory access, MPU violation, etc. */
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */

  while (1) {
    /* BusFault: typically invalid address on bus, alignment, etc. */
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */

  while (1) {
    /* UsageFault: undefined instruction, divide-by-zero (if enabled), etc. */
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */

  HAL_IncTick();

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F2xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/
/* USER CODE BEGIN 1 */

/**
 * @brief DMA1 Stream1 IRQ handler (Input path: TIM2_UP -> DMA P2M -> sig_dma_buf)
 *
 * The input DMA stream runs in circular mode. Its HT/TC events indicate:
 *  - HT: first half of sig_dma_buf[] completed (second half currently being filled)
 *  - TC: second half completed (first half currently being filled)
 *
 * The actual counter updates and flag clears are implemented in SigDma_TestIRQHandler().
 */
void DMA1_Stream1_IRQHandler(void)
{
  SigDma_TestIRQHandler();
}

/**
 * @brief TIM2 global IRQ handler
 *
 * TIM2 is the master sample clock. We use the Update Interrupt (UIF) to run the
 * sample generator exactly once per sample period.
 *
 * Important:
 *  - Always clear UIF to avoid re-entering the ISR immediately.
 *  - Keep this ISR short: generate one sample and store it to the "DMA source"
 *    register (TIM2->CCR1) inside SigGen_OnTick().
 *
 * Note:
 *  - On STM32 timers, clearing UIF is done by writing 0 to the UIF bit.
 *    TIM2->SR &= ~TIM_SR_UIF is the common, readable pattern.
 */
void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF;   // clear update interrupt flag
    SigGen_OnTick();           // generate next sample (writes TIM2->CCR1)
  }
}

/**
 * @brief DMA1 Stream5 IRQ handler (Output path: DAC DMA M2P -> DAC->DHR12R1)
 *
 * The output DMA stream runs in circular mode and feeds the DAC. HT/TC can be
 * used for instrumentation or for "producer" pacing if desired, but the main
 * application currently uses input HT/TC for lockstep processing.
 *
 * The flag handling and optional counters are implemented in SigDac_OutDmaIRQHandler().
 */
void DMA1_Stream5_IRQHandler(void)
{
  SigDac_OutDmaIRQHandler();
}

/* USER CODE END 1 */