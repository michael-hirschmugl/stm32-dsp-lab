#pragma once
#include <stdint.h>
#include <stddef.h>

#ifndef SIG_FS_HZ
#define SIG_FS_HZ            44100u
#endif

// ---- Schalter ----
#define SIG_ENABLE_TONE1      0
#define SIG_ENABLE_TONE2      0
#define SIG_ENABLE_TONE3      1
#define SIG_ENABLE_NOISE      1

// ---- Parameter ----
#define SIG_TONE1_FREQ_HZ     440.0f
#define SIG_TONE1_AMP         0.55f

#define SIG_TONE2_FREQ_HZ     880.0f
#define SIG_TONE2_AMP         0.25f

#define SIG_TONE3_FREQ_HZ     1760.0f
#define SIG_TONE3_AMP         0.10f

#define SIG_NOISE_AMP         0.03f

// ---------------------------------------------------------------------------
// Enable DAC output on PA4 (DAC Channel 1), triggered by TIM2_TRGO,
// with DMA circular output buffer.
// ---------------------------------------------------------------------------
#ifndef SIG_ENABLE_DAC_OUT
#define SIG_ENABLE_DAC_OUT    1
#endif

#ifndef SIG_USE_TIM2_DMA_TEST
#define SIG_USE_TIM2_DMA_TEST  1
#endif

#ifndef SIG_DMA_BUF_LEN
#define SIG_DMA_BUF_LEN        512u
#define SIG_DMA_HALF_LEN       (SIG_DMA_BUF_LEN/2)
#endif

#ifdef __cplusplus
extern "C" {
#endif

void     SigGen_Init(void);
void     SigGen_OnTick(void);

// Input DMA (P2M) test/capture
void     SigDma_TestInit(void);
void     SigDma_TestStart(void);
void     SigDma_TestIRQHandler(void);

extern volatile uint32_t sig_dma_ht_count;
extern volatile uint32_t sig_dma_tc_count;
extern volatile uint32_t sig_dma_last_is_ht;
extern int16_t           sig_dma_buf[SIG_DMA_BUF_LEN];

// DAC output (M2P)
#if SIG_ENABLE_DAC_OUT
void     SigDac_Init(void);
void     SigDac_Start(void);
void     SigDac_OutDmaIRQHandler(void);

extern volatile uint32_t dac_dma_ht_count;
extern volatile uint32_t dac_dma_tc_count;
extern uint16_t          dac_dma_buf[SIG_DMA_BUF_LEN];
#endif

#ifdef __cplusplus
}
#endif