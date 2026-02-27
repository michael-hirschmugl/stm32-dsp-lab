#include "signal_gen.h"
#include "stm32f207xx.h"
#include <math.h>

// ===================== INPUT DMA (P2M): TIM2_UP -> DMA1 Stream1 Ch3 =====================

volatile uint32_t sig_dma_ht_count = 0;
volatile uint32_t sig_dma_tc_count = 0;
volatile uint32_t sig_dma_last_is_ht = 0;
int16_t           sig_dma_buf[SIG_DMA_BUF_LEN];
uint32_t          val = 0xBEAF;

#define SIG_DMAx              DMA1
#define SIG_DMA_STREAM        DMA1_Stream1
#define SIG_DMA_STREAM_IRQn   DMA1_Stream1_IRQn
#define SIG_DMA_CHSEL         (3u)   // TIM2_UP mapping

// ===================== DAC OUTPUT DMA (M2P): DAC1 -> DMA1 Stream5 Ch7 =====================

#if SIG_ENABLE_DAC_OUT
volatile uint32_t dac_dma_ht_count = 0;
volatile uint32_t dac_dma_tc_count = 0;
uint16_t          dac_dma_buf[SIG_DMA_BUF_LEN];

#define DAC_DMAx              DMA1
#define DAC_DMA_STREAM        DMA1_Stream5
#define DAC_DMA_STREAM_IRQn   DMA1_Stream5_IRQn
#define DAC_DMA_CHSEL         (7u)   // DAC channel mapping
#endif

// ===================== Fast Cos LUT + DDS =====================

#define COS_LUT_BITS   10u
#define COS_LUT_SIZE   (1u << COS_LUT_BITS)
static float cos_lut[COS_LUT_SIZE];

static inline float fast_cos_u32(uint32_t phase) {
    uint32_t idx = phase >> (32u - COS_LUT_BITS);
    return cos_lut[idx];
}

typedef struct {
    uint32_t phase;
    uint32_t inc;
    float    amp;
    uint8_t  en;
} dds_t;

static dds_t t1, t2, t3;

static uint32_t rng = 0x12345678u;
static inline uint32_t xorshift32(void) {
    uint32_t x = rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng = x;
    return x;
}

static inline float white_noise_uniform(void) {
    uint32_t r = xorshift32();
    float u = (float)(r & 0x00FFFFFFu) * (1.0f / 16777215.0f);
    return 2.0f * u - 1.0f;
}

static inline uint32_t phase_inc_from_freq(float f_hz) {
    double inc = (double)f_hz * (4294967296.0 / (double)SIG_FS_HZ);
    if (inc < 0) inc = 0;
    if (inc > 4294967295.0) inc = 4294967295.0;
    return (uint32_t)inc;
}

static inline int16_t float_to_q15_clip(float x) {
    if (x > 0.999969f) x = 0.999969f;
    if (x < -1.0f)     x = -1.0f;
    int32_t v = (int32_t)lrintf(x * 32768.0f);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

// Map int16 audio sample [-32768..32767] to DAC 12-bit unsigned [0..4095]
#if SIG_ENABLE_DAC_OUT
static inline uint16_t s16_to_dac_u12(int16_t s) {
    // scale down to 12-bit, add midscale offset
    int32_t v = ((int32_t)s >> 4) + 2048;   // 16->12 bit + offset
    if (v < 0) v = 0;
    if (v > 4095) v = 4095;
    return (uint16_t)v;
}
#endif

// ===================== TIM2 @ Fs, TRGO = Update =====================

static void tim2_init_fs(void) {
    const uint32_t tim_clk_hz = 60000000u; // per your clock notes

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;
    TIM2->PSC = 0;

    const uint32_t arr = (uint32_t)((tim_clk_hz + (SIG_FS_HZ / 2u)) / SIG_FS_HZ);
    TIM2->ARR = (arr > 0u) ? (arr - 1u) : 0u;

    TIM2->EGR = TIM_EGR_UG;

    // TRGO = Update event (MMS=010)
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | (2u << TIM_CR2_MMS_Pos);
}

static void tim2_disable_update_irq(void) {
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->SR   &= ~TIM_SR_UIF;
}

// ===================== DAC init: PA4, trigger TIM2_TRGO =====================

#if SIG_ENABLE_DAC_OUT
void SigDac_Init(void) {
    // GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA4 analog
    GPIOA->MODER |= (3u << (4u * 2u));

    // DAC clock
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Disable ch1 while configuring
    DAC->CR &= ~DAC_CR_EN1;

    // Clear relevant bits for ch1: TSEL1[5:3], TEN1, DMAEN1, WAVE1[7:6], MAMP1[11:8]
    DAC->CR &= ~((7u << 3) | DAC_CR_TEN1 | DAC_CR_DMAEN1 | (3u << 6) | (0xFu << 8));

    // Select trigger TIM2_TRGO:
    // On STM32F2: TSEL=0b100 corresponds to TIM2_TRGO
    DAC->CR |= (4u << 3);

    // Enable trigger + DMA for channel 1
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_DMAEN1;

    // Enable DAC channel 1
    DAC->CR |= DAC_CR_EN1;

    // Pre-fill output buffer with midscale (silence)
    for (uint32_t i = 0; i < SIG_DMA_BUF_LEN; i++) {
        dac_dma_buf[i] = 2048u;
    }

    dac_dma_ht_count = 0;
    dac_dma_tc_count = 0;
}
#endif

// ===================== DAC DMA init/start =====================

#if SIG_ENABLE_DAC_OUT
static void dac_dma_init_stream(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Stop stream if running
    if (DAC_DMA_STREAM->CR & DMA_SxCR_EN) {
        DAC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        while (DAC_DMA_STREAM->CR & DMA_SxCR_EN) { /* wait */ }
    }

    // Clear flags for Stream5 (HISR/HIFCR for streams 4..7)
    // Stream5 flags are in DMA1->HISR / DMA1->HIFCR.
    DAC_DMAx->HIFCR = DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 |
                      DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5;

    DAC_DMA_STREAM->CR = 0;
    DAC_DMA_STREAM->NDTR = SIG_DMA_BUF_LEN;

    // Peripheral: DAC channel 1 12-bit right-aligned holding register
    DAC_DMA_STREAM->PAR  = (uint32_t)&DAC->DHR12R1;
    DAC_DMA_STREAM->M0AR = (uint32_t)&dac_dma_buf[0];

    // Memory-to-peripheral (DIR=01)
    // 16-bit transfers (halfword) are fine for DHR12R1
    DAC_DMA_STREAM->CR =
        (DAC_DMA_CHSEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0 |      // M2P
        DMA_SxCR_MINC |
        DMA_SxCR_CIRC |
        DMA_SxCR_PSIZE_0 |    // 16-bit peripheral
        DMA_SxCR_MSIZE_0 |    // 16-bit memory
        DMA_SxCR_HTIE |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        (2u << DMA_SxCR_PL_Pos);

    DAC_DMA_STREAM->FCR = 0;

    NVIC_SetPriority(DAC_DMA_STREAM_IRQn, 7);
    NVIC_EnableIRQ(DAC_DMA_STREAM_IRQn);
}

void SigDac_Start(void) {
    dac_dma_init_stream();

    // Enable DMA stream first
    DAC_DMA_STREAM->CR |= DMA_SxCR_EN;
}
#endif

void SigDac_OutDmaIRQHandler(void) {
#if SIG_ENABLE_DAC_OUT
    const uint32_t hisr = DAC_DMAx->HISR;

    if (hisr & DMA_HISR_HTIF5) {
        DAC_DMAx->HIFCR = DMA_HIFCR_CHTIF5;
        dac_dma_ht_count++;
    }
    if (hisr & DMA_HISR_TCIF5) {
        DAC_DMAx->HIFCR = DMA_HIFCR_CTCIF5;
        dac_dma_tc_count++;
    }
    if (hisr & DMA_HISR_TEIF5) {
        DAC_DMAx->HIFCR = DMA_HIFCR_CTEIF5;
        DAC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    }
#else
    (void)0;
#endif
}

// ===================== INPUT DMA init/start (unchanged core) =====================

void SigDma_TestInit(void) {
    SigGen_Init();

    tim2_init_fs();
    tim2_disable_update_irq();

#if SIG_ENABLE_DAC_OUT
    SigDac_Init();
#endif

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    if (SIG_DMA_STREAM->CR & DMA_SxCR_EN) {
        SIG_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        while (SIG_DMA_STREAM->CR & DMA_SxCR_EN) { /* wait */ }
    }

    SIG_DMAx->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 |
                      DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;

    SIG_DMA_STREAM->CR = 0;
    SIG_DMA_STREAM->NDTR = SIG_DMA_BUF_LEN;

    TIM2->CCR1 = val;
    SIG_DMA_STREAM->PAR  = (uint32_t)&TIM2->CCR1;
    SIG_DMA_STREAM->M0AR = (uint32_t)&sig_dma_buf[0];

    SIG_DMA_STREAM->CR =
        (SIG_DMA_CHSEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC |
        DMA_SxCR_CIRC |
        DMA_SxCR_PSIZE_0 |
        DMA_SxCR_MSIZE_0 |
        DMA_SxCR_HTIE |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        (2u << DMA_SxCR_PL_Pos);

    SIG_DMA_STREAM->FCR = 0;

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;

    TIM2->DIER |= TIM_DIER_UDE;

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);

    NVIC_SetPriority(SIG_DMA_STREAM_IRQn, 6);
    NVIC_EnableIRQ(SIG_DMA_STREAM_IRQn);

    sig_dma_ht_count = 0;
    sig_dma_tc_count = 0;
    sig_dma_last_is_ht = 0;
}

void SigDma_TestStart(void) {
    // Start output DMA first (so DAC has data once timer starts)
#if SIG_ENABLE_DAC_OUT
    SigDac_Start();
#endif

    // Start input DMA
    SIG_DMA_STREAM->CR |= DMA_SxCR_EN;

    // Start TIM2 (master clock)
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void SigDma_TestIRQHandler(void) {
    const uint32_t lisr = SIG_DMAx->LISR;

    if (lisr & DMA_LISR_HTIF1) {
        SIG_DMAx->LIFCR = DMA_LIFCR_CHTIF1;
        sig_dma_ht_count++;
        sig_dma_last_is_ht = 1;
    }
    if (lisr & DMA_LISR_TCIF1) {
        SIG_DMAx->LIFCR = DMA_LIFCR_CTCIF1;
        sig_dma_tc_count++;
        sig_dma_last_is_ht = 0;
    }
    if (lisr & DMA_LISR_TEIF1) {
        SIG_DMAx->LIFCR = DMA_LIFCR_CTEIF1;
        SIG_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    }
}

// ===================== Generator init/tick =====================

void SigGen_Init(void) {
    for (uint32_t i = 0; i < COS_LUT_SIZE; i++) {
        float a = 2.0f * (float)M_PI * (float)i / (float)COS_LUT_SIZE;
        cos_lut[i] = cosf(a);
    }

    t1.phase = 0; t1.inc = phase_inc_from_freq(SIG_TONE1_FREQ_HZ); t1.amp = SIG_TONE1_AMP; t1.en = SIG_ENABLE_TONE1;
    t2.phase = 0; t2.inc = phase_inc_from_freq(SIG_TONE2_FREQ_HZ); t2.amp = SIG_TONE2_AMP; t2.en = SIG_ENABLE_TONE2;
    t3.phase = 0; t3.inc = phase_inc_from_freq(SIG_TONE3_FREQ_HZ); t3.amp = SIG_TONE3_AMP; t3.en = SIG_ENABLE_TONE3;
}

void SigGen_OnTick(void) {
    float s = 0.0f;

    if (t1.en) { s += t1.amp * fast_cos_u32(t1.phase); t1.phase += t1.inc; }
    if (t2.en) { s += t2.amp * fast_cos_u32(t2.phase); t2.phase += t2.inc; }
    if (t3.en) { s += t3.amp * fast_cos_u32(t3.phase); t3.phase += t3.inc; }

#if SIG_ENABLE_NOISE
    s += SIG_NOISE_AMP * white_noise_uniform();
#endif

    int16_t q = float_to_q15_clip(s);
    TIM2->CCR1 = (uint16_t)q;   // P2M hack source
}