#pragma once
#include <stdint.h>
#include <stddef.h>

#ifndef SIG_FS_HZ
#define SIG_FS_HZ            44100u
#endif

//#ifndef SIG_BLOCK_SIZE
//#define SIG_BLOCK_SIZE       128u
//#endif

// Ringbuffer-Länge (muss Power-of-Two sein!)
//#ifndef SIG_RING_LEN
//#define SIG_RING_LEN         1024u
//#endif

// ---- Schalter ----
#define SIG_ENABLE_TONE1      1
#define SIG_ENABLE_TONE2      1
#define SIG_ENABLE_TONE3      0
#define SIG_ENABLE_NOISE      0

// ---- Parameter ----
#define SIG_TONE1_FREQ_HZ     440.0f
#define SIG_TONE1_AMP         0.55f

#define SIG_TONE2_FREQ_HZ     880.0f
#define SIG_TONE2_AMP         0.25f

#define SIG_TONE3_FREQ_HZ     1760.0f
#define SIG_TONE3_AMP         0.10f

#define SIG_NOISE_AMP         0.03f   // “leichtes” Rauschen

// Optional: echte Analog-Ausgabe über DAC (PA4), später aktivieren
//#define SIG_ENABLE_DAC_OUT    0

// ---------------------------------------------------------------------------
// TIM2 -> DMA (Test / Capture)
//
// Ziel: TIM2 erzeugt bei jedem Update-Event einen DMA-Request. Der DMA liest
// ein (quasi konstantes) Wort aus einem Peripherie-Register und schreibt es in
// einen Circular Buffer. Bei N/2 (HT) und N (TC) gibt's IRQs.
//
// Hinweis: Auf STM32F207 (DMA ohne DMAMUX) ist ein "Timer -> DMA -> RAM"-Test
// am einfachsten als Peripheral-to-Memory Transfer. Hier lesen wir TIM2->CNT.
// Bei Update-Events ist CNT typischerweise 0 oder nahe 0 -> gut als "konstanter"
// Testwert.
// ---------------------------------------------------------------------------
#ifndef SIG_USE_TIM2_DMA_TEST
#define SIG_USE_TIM2_DMA_TEST  1
#endif

#ifndef SIG_DMA_BUF_LEN
#define SIG_DMA_BUF_LEN        512u  // N (<= 65535)
#endif

#ifdef __cplusplus
extern "C" {
#endif

void     SigGen_Init(void);
void     SigGen_Start(void);

// “ADC-like” API:
//size_t   SigGen_Available(void);
//size_t   SigGen_ReadBlock(int16_t* dst, size_t n);

// ISR hook (wird vom TIM2_IRQHandler aufgerufen)
void     SigGen_OnTick(void);

// DMA-Test API
void     SigDma_TestInit(void);
void     SigDma_TestStart(void);
void     SigDma_TestIRQHandler(void);

extern volatile uint32_t sig_dma_ht_count;
extern volatile uint32_t sig_dma_tc_count;
extern volatile uint32_t sig_dma_last_is_ht;
extern int16_t           sig_dma_buf[SIG_DMA_BUF_LEN];

#ifdef __cplusplus
}
#endif