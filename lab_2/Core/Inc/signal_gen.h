#pragma once
/**
 * @file    signal_gen.h
 * @brief   Timer-driven signal generator + DMA capture + optional DAC output
 *
 * This module provides a small, deterministic “audio engine” built around
 * TIM2 running at SIG_FS_HZ (e.g. 44.1 kHz).
 *
 * High-level dataflow
 * -------------------
 *   TIM2 @ SIG_FS_HZ
 *     ├─ (IRQ)  TIM2_IRQHandler -> SigGen_OnTick()
 *     │          Generates exactly one sample per tick and writes it into a
 *     │          “DMA source register” (TIM2->CCR1) as a hacky but effective
 *     │          peripheral register source for DMA P2M.
 *     │
 *     ├─ (DMA P2M) TIM2 update DMA request -> sig_dma_buf[] (int16, circular)
 *     │            HT/TC interrupts increment counters so the CPU can safely
 *     │            consume half-buffers in lockstep.
 *     │
 *     └─ (optional) TIM2_TRGO -> DAC trigger
 *                DAC requests DMA (M2P) -> dac_dma_buf[] (u12 values, circular)
 *
 * Notes
 * -----
 * - Heavy processing is intended to happen in the main loop on half-buffer blocks.
 * - Interrupts should remain short and bounded.
 * - Buffer lengths are chosen to support double-buffering (half-transfer/transfer-complete).
 */

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * Configuration
 * ============================================================================
 */

/**
 * @brief Sampling frequency in Hz.
 *
 * Must match TIM2 configuration in signal_gen.c.
 */
#ifndef SIG_FS_HZ
#define SIG_FS_HZ 44100u
#endif

/* ---- Generator enable switches (compile-time) ----
 *
 * These allow you to include/exclude oscillators and noise at build time.
 * Set to 1 to enable the component, 0 to disable.
 */
#ifndef SIG_ENABLE_TONE1
#define SIG_ENABLE_TONE1 0
#endif
#ifndef SIG_ENABLE_TONE2
#define SIG_ENABLE_TONE2 0
#endif
#ifndef SIG_ENABLE_TONE3
#define SIG_ENABLE_TONE3 1
#endif
#ifndef SIG_ENABLE_NOISE
#define SIG_ENABLE_NOISE 1
#endif

/* ---- Generator parameters ----
 *
 * Tone frequencies in Hz, amplitudes are linear scale (roughly 0..1).
 * Keep summed amplitudes <= 1 to avoid clipping. If you add multiple tones
 * and noise, reduce amplitudes accordingly.
 */
#ifndef SIG_TONE1_FREQ_HZ
#define SIG_TONE1_FREQ_HZ 440.0f
#endif
#ifndef SIG_TONE1_AMP
#define SIG_TONE1_AMP     0.55f
#endif

#ifndef SIG_TONE2_FREQ_HZ
#define SIG_TONE2_FREQ_HZ 880.0f
#endif
#ifndef SIG_TONE2_AMP
#define SIG_TONE2_AMP     0.25f
#endif

#ifndef SIG_TONE3_FREQ_HZ
#define SIG_TONE3_FREQ_HZ 1760.0f
#endif
#ifndef SIG_TONE3_AMP
#define SIG_TONE3_AMP     0.10f
#endif

#ifndef SIG_NOISE_AMP
#define SIG_NOISE_AMP     0.03f
#endif

/**
 * @brief Enable analog output via DAC (Channel 1 on PA4).
 *
 * If enabled:
 *  - TIM2_TRGO triggers the DAC at SIG_FS_HZ
 *  - DAC requests DMA transfers (M2P) from dac_dma_buf[]
 *
 * If disabled, output-related APIs and globals are not declared.
 */
#ifndef SIG_ENABLE_DAC_OUT
#define SIG_ENABLE_DAC_OUT 1
#endif

/**
 * @brief Enable the “TIM2 -> DMA (P2M) capture/test” path.
 *
 * If enabled:
 *  - Input DMA is configured in circular mode into sig_dma_buf[]
 *  - HT/TC interrupts are used for double-buffer block availability
 */
#ifndef SIG_USE_TIM2_DMA_TEST
#define SIG_USE_TIM2_DMA_TEST 1
#endif

/**
 * @brief Length of the circular DMA buffers in samples.
 *
 * Requirements:
 *  - Must be <= 65535 (DMA NDTR is 16-bit on STM32F2)
 *  - Should be even, because we use half-buffer processing (HT/TC).
 */
#ifndef SIG_DMA_BUF_LEN
#define SIG_DMA_BUF_LEN 512u
#endif

/** @brief Convenience: half-buffer size in samples. */
#define SIG_DMA_HALF_LEN (SIG_DMA_BUF_LEN / 2u)

/* ============================================================================
 * Public API
 * ============================================================================
 */

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Signal generator (runs at SIG_FS_HZ)
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize lookup tables/state used by the signal generator.
 *
 * Typically called from SigDma_TestInit() as part of the pipeline init.
 */
void SigGen_Init(void);

/**
 * @brief Generate one sample (called from TIM2 update ISR).
 *
 * This function is designed to be fast and deterministic.
 *
 * In this project, the generated sample is written into TIM2->CCR1 so that
 * the DMA input stream can read a “real, changing” peripheral register value.
 */
void SigGen_OnTick(void);

/* --------------------------------------------------------------------------
 * Input DMA capture/test (Peripheral-to-Memory, circular)
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialize TIM2 + DMA input stream (P2M) and related interrupts.
 *
 * This sets up:
 *  - TIM2 @ SIG_FS_HZ
 *  - TIM2 update DMA request (UDE) to the selected DMA stream/channel
 *  - sig_dma_buf[] circular buffer (int16)
 *  - HT/TC interrupts for block availability
 *
 * Note:
 *  SigDma_TestStart() must be called to actually enable the stream/timer.
 */
void SigDma_TestInit(void);

/**
 * @brief Start the input DMA stream and TIM2 counter.
 *
 * Typical sequence:
 *  - Enable DMA streams first
 *  - Then start TIM2 (so early requests aren’t missed)
 *
 * If DAC output is enabled, the implementation may also start the DAC DMA
 * before enabling TIM2.
 */
void SigDma_TestStart(void);

/**
 * @brief ISR hook for the input DMA stream.
 *
 * Clears HT/TC/TE flags and increments counters.
 * Must be called from DMA1_StreamX_IRQHandler().
 */
void SigDma_TestIRQHandler(void);

/**
 * @brief Input DMA half-transfer counter.
 *
 * Incremented when the first half of sig_dma_buf[] is complete.
 * The CPU can safely read sig_dma_buf[0 .. SIG_DMA_HALF_LEN-1] when this changes.
 */
extern volatile uint32_t sig_dma_ht_count;

/**
 * @brief Input DMA transfer-complete counter.
 *
 * Incremented when the second half of sig_dma_buf[] is complete.
 * The CPU can safely read sig_dma_buf[SIG_DMA_HALF_LEN .. SIG_DMA_BUF_LEN-1]
 * when this changes.
 */
extern volatile uint32_t sig_dma_tc_count;

/**
 * @brief Indicates which event occurred last (1=HT, 0=TC).
 *
 * Optional debug/telemetry convenience.
 */
extern volatile uint32_t sig_dma_last_is_ht;

/**
 * @brief Input circular buffer filled by DMA (P2M).
 *
 * Format: signed 16-bit samples (Q15-ish audio range).
 */
extern int16_t sig_dma_buf[SIG_DMA_BUF_LEN];

/* --------------------------------------------------------------------------
 * Optional DAC output (Memory-to-Peripheral, circular)
 * -------------------------------------------------------------------------- */
#if SIG_ENABLE_DAC_OUT

/**
 * @brief Initialize DAC Channel 1 output (PA4) and its DMA stream (M2P).
 *
 * Typical configuration:
 *  - PA4 analog mode
 *  - DAC ch1 enabled, triggered by TIM2_TRGO
 *  - DAC DMA enabled
 *  - DMA configured to write dac_dma_buf[] into DAC->DHR12R1 in circular mode
 */
void SigDac_Init(void);

/**
 * @brief Start DAC output DMA stream and enable DAC triggering.
 *
 * Often called from SigDma_TestStart() to ensure correct start ordering.
 */
void SigDac_Start(void);

/**
 * @brief ISR hook for the DAC output DMA stream (e.g. DMA1 Stream5).
 *
 * Clears HT/TC/TE flags and increments counters.
 */
void SigDac_OutDmaIRQHandler(void);

/** @brief DAC DMA half-transfer counter (optional instrumentation). */
extern volatile uint32_t dac_dma_ht_count;

/** @brief DAC DMA transfer-complete counter (optional instrumentation). */
extern volatile uint32_t dac_dma_tc_count;

/**
 * @brief Output circular buffer consumed by DAC DMA (M2P).
 *
 * Format: unsigned 12-bit right-aligned samples in uint16_t range [0..4095].
 * The DAC will output (value / 4095) * Vref (with typical Vref = 3.3V or VDDA).
 */
extern uint16_t dac_dma_buf[SIG_DMA_BUF_LEN];

#endif /* SIG_ENABLE_DAC_OUT */

#ifdef __cplusplus
}
#endif