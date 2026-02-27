/**
 * @file    signal_gen.c
 * @brief   Timer-driven signal generator + DMA capture + optional DAC output
 *
 * This module implements a deterministic, timer-driven “audio engine” on STM32F207.
 *
 * System concept
 * --------------
 * A single master timer (TIM2) runs at SIG_FS_HZ (typically 44.1 kHz). Each TIM2
 * update event is used to drive both input and output data movement via DMA and
 * also to tick the sample generator:
 *
 *   - (IRQ) TIM2 update interrupt calls SigGen_OnTick(), which generates one
 *     sample and stores it in TIM2->CCR1. This is a deliberate "hack" to provide
 *     a peripheral register with continuously changing data.
 *
 *   - (Input DMA, P2M) TIM2 update DMA request transfers TIM2->CCR1 into
 *     sig_dma_buf[] in circular mode. Half-transfer (HT) and transfer-complete (TC)
 *     events generate interrupts so the CPU can safely consume half-buffers.
 *
 *   - (Optional output) TIM2 TRGO is configured as "update event". The DAC is
 *     configured to trigger on TIM2_TRGO and to request DMA transfers (M2P) from
 *     dac_dma_buf[] into DAC->DHR12R1. This yields a continuous analog output
 *     synchronized with the same master sample clock.
 *
 * Design rules
 * ------------
 * - ISRs must remain short. Heavy processing is intended for the main loop.
 * - All circular buffers are split into two halves for safe lockstep processing.
 * - No dynamic allocation; everything is static and deterministic.
 */

#include "signal_gen.h"
#include "stm32f207xx.h"
#include <math.h>

/* ============================================================================
 * Input DMA (P2M): TIM2_UP -> DMA1 Stream1, Channel 3
 * ============================================================================
 *
 * The mapping TIM2_UP -> DMA1 Stream1 Channel 3 is per RM0033 DMA request mapping.
 * This stream performs peripheral-to-memory transfers from TIM2->CCR1 into
 * sig_dma_buf[] at the timer update rate.
 */

volatile uint32_t sig_dma_ht_count      = 0;
volatile uint32_t sig_dma_tc_count      = 0;
volatile uint32_t sig_dma_last_is_ht    = 0;
int16_t           sig_dma_buf[SIG_DMA_BUF_LEN];

/* Initial seed value for CCR1 before the generator starts ticking. */
static uint32_t   s_seed_val = 0xBEAFu;

#define SIG_DMAx              DMA1
#define SIG_DMA_STREAM        DMA1_Stream1
#define SIG_DMA_STREAM_IRQn   DMA1_Stream1_IRQn
#define SIG_DMA_CHSEL         (3u)      /* TIM2_UP mapping */

/* ============================================================================
 * DAC Output DMA (M2P): DAC1 -> DMA1 Stream5, Channel 7
 * ============================================================================
 *
 * When enabled, the DAC is triggered by TIM2_TRGO. Each trigger causes the DAC
 * to request a DMA transfer which writes the next sample into DAC->DHR12R1.
 *
 * The stream is configured as:
 *  - Memory-to-peripheral (M2P)
 *  - Circular mode
 *  - Halfword transfers (16-bit)
 *  - HT/TC/TE interrupts for instrumentation (optional)
 */

#if SIG_ENABLE_DAC_OUT

volatile uint32_t dac_dma_ht_count = 0;
volatile uint32_t dac_dma_tc_count = 0;

/* Output buffer consumed by the DAC DMA stream (12-bit right-aligned in u16). */
uint16_t          dac_dma_buf[SIG_DMA_BUF_LEN];

#define DAC_DMAx              DMA1
#define DAC_DMA_STREAM        DMA1_Stream5
#define DAC_DMA_STREAM_IRQn   DMA1_Stream5_IRQn
#define DAC_DMA_CHSEL         (7u)      /* DAC channel mapping */

#endif /* SIG_ENABLE_DAC_OUT */

/* ============================================================================
 * Fast cosine LUT + DDS oscillator bank
 * ============================================================================
 *
 * We generate simple test signals using a DDS (direct digital synthesis) approach.
 * Each tone maintains a 32-bit phase accumulator. The phase increment is derived
 * from frequency and sample rate.
 *
 * fast_cos_u32() uses a small cosine lookup table for speed and determinism.
 */

#define COS_LUT_BITS   10u
#define COS_LUT_SIZE   (1u << COS_LUT_BITS)
static float cos_lut[COS_LUT_SIZE];

static inline float fast_cos_u32(uint32_t phase)
{
    /* phase: 0..2^32-1 maps to 0..COS_LUT_SIZE-1 */
    const uint32_t idx = phase >> (32u - COS_LUT_BITS);
    return cos_lut[idx];
}

typedef struct {
    uint32_t phase;   /* phase accumulator */
    uint32_t inc;     /* phase increment per sample */
    float    amp;     /* linear amplitude */
    uint8_t  en;      /* enable switch */
} dds_t;

static dds_t t1, t2, t3;

/* ============================================================================
 * Noise generator (xorshift32)
 * ============================================================================
 *
 * Lightweight PRNG, suitable for basic white noise. The output is uniform in
 * [-1, +1]. If you need better spectral properties later, you can replace this.
 */

static uint32_t rng = 0x12345678u;

static inline uint32_t xorshift32(void)
{
    uint32_t x = rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng = x;
    return x;
}

static inline float white_noise_uniform(void)
{
    /* Uniform in [-1, +1]. Use 24 bits to keep float quantization reasonable. */
    const uint32_t r = xorshift32();
    const float u = (float)(r & 0x00FFFFFFu) * (1.0f / 16777215.0f);
    return 2.0f * u - 1.0f;
}

/* ============================================================================
 * Helper conversions
 * ============================================================================
 */

static inline uint32_t phase_inc_from_freq(float f_hz)
{
    /* inc = f * 2^32 / Fs */
    double inc = (double)f_hz * (4294967296.0 / (double)SIG_FS_HZ);
    if (inc < 0.0) inc = 0.0;
    if (inc > 4294967295.0) inc = 4294967295.0;
    return (uint32_t)inc;
}

static inline int16_t float_to_q15_clip(float x)
{
    /* Clip and convert float in [-1, +1) to signed 16-bit. */
    if (x > 0.999969f) x = 0.999969f;
    if (x < -1.0f)     x = -1.0f;

    int32_t v = (int32_t)lrintf(x * 32768.0f);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

/* Map int16 audio sample [-32768..32767] to DAC 12-bit unsigned [0..4095]. */
#if SIG_ENABLE_DAC_OUT
static inline uint16_t s16_to_dac_u12(int16_t s)
{
    int32_t v = ((int32_t)s >> 4) + 2048;  /* 16->12 bit + midscale offset */
    if (v < 0)    v = 0;
    if (v > 4095) v = 4095;
    return (uint16_t)v;
}
#endif

/* ============================================================================
 * TIM2 @ Fs, TRGO = Update
 * ============================================================================
 *
 * TIM2 is the master sample clock. TRGO is configured to update event so that
 * the DAC can use TIM2_TRGO as its trigger.
 *
 * Clock assumptions:
 *  - With your SystemClock_Config, APB1 prescaler != 1, so timer clock on APB1
 *    is 2*PCLK1.
 *  - In your setup: HCLK=120MHz, APB1=HCLK/4=30MHz, TIM2CLK=60MHz.
 *
 * If you change clock tree settings, adjust tim_clk_hz accordingly.
 */

static void tim2_init_fs(void)
{
    const uint32_t tim_clk_hz = 60000000u;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;
    TIM2->PSC = 0;

    /* ARR chosen to approximate Fs. Round-to-nearest to minimize error. */
    const uint32_t arr = (uint32_t)((tim_clk_hz + (SIG_FS_HZ / 2u)) / SIG_FS_HZ);
    TIM2->ARR = (arr > 0u) ? (arr - 1u) : 0u;

    /* Force update to load prescaler/ARR immediately. */
    TIM2->EGR = TIM_EGR_UG;

    /* TRGO = Update event (MMS=010). Used by DAC trigger. */
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | (2u << TIM_CR2_MMS_Pos);
}

static void tim2_disable_update_irq(void)
{
    /* Disable update interrupt and clear UIF. */
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->SR   &= ~TIM_SR_UIF;
}

/* ============================================================================
 * DAC init: PA4 (DAC_OUT1), trigger TIM2_TRGO
 * ============================================================================
 */

#if SIG_ENABLE_DAC_OUT

void SigDac_Init(void)
{
    /* Enable GPIOA clock. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* Configure PA4 as analog mode (MODER4 = 11). */
    GPIOA->MODER |= (3u << (4u * 2u));

    /* Enable DAC peripheral clock. */
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* Disable channel 1 while configuring. */
    DAC->CR &= ~DAC_CR_EN1;

    /* Clear relevant configuration fields for channel 1:
     *  - TSEL1[5:3] trigger selection
     *  - TEN1 trigger enable
     *  - DMAEN1 DMA enable
     *  - WAVE1[7:6], MAMP1[11:8] (ensure no waveform generator is active)
     */
    DAC->CR &= ~((7u << 3) | DAC_CR_TEN1 | DAC_CR_DMAEN1 | (3u << 6) | (0xFu << 8));

    /* Select TIM2_TRGO as trigger source.
     * On STM32F2, TSEL1=0b100 corresponds to TIM2_TRGO.
     */
    DAC->CR |= (4u << 3);

    /* Enable trigger and DMA request generation for ch1. */
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_DMAEN1;

    /* Enable DAC channel 1. */
    DAC->CR |= DAC_CR_EN1;

    /* Initialize output buffer to midscale (silence). */
    for (uint32_t i = 0; i < SIG_DMA_BUF_LEN; i++) {
        dac_dma_buf[i] = 2048u;
    }

    dac_dma_ht_count = 0;
    dac_dma_tc_count = 0;
}

#endif /* SIG_ENABLE_DAC_OUT */

/* ============================================================================
 * DAC DMA init/start
 * ============================================================================
 */

#if SIG_ENABLE_DAC_OUT

static void dac_dma_init_stream(void)
{
    /* Enable DMA1 clock. */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* Stop stream if currently enabled. */
    if (DAC_DMA_STREAM->CR & DMA_SxCR_EN) {
        DAC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        while (DAC_DMA_STREAM->CR & DMA_SxCR_EN) { /* wait */ }
    }

    /* Clear pending flags for Stream5 (in HISR/HIFCR). */
    DAC_DMAx->HIFCR = DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 |
                      DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5;

    /* Configure stream registers. */
    DAC_DMA_STREAM->CR   = 0;
    DAC_DMA_STREAM->NDTR = SIG_DMA_BUF_LEN;

    /* Peripheral address: DAC 12-bit right-aligned holding register (ch1). */
    DAC_DMA_STREAM->PAR  = (uint32_t)&DAC->DHR12R1;

    /* Memory address: output ring buffer. */
    DAC_DMA_STREAM->M0AR = (uint32_t)&dac_dma_buf[0];

    /* Stream configuration:
     *  - Channel selection: DAC request
     *  - DIR = 01 (memory-to-peripheral)
     *  - MINC = 1 (increment memory address)
     *  - CIRC = 1 (circular mode)
     *  - PSIZE/MSIZE = 16-bit (halfword)
     *  - HT/TC/TE interrupts enabled for instrumentation/debug
     *  - Priority high
     */
    DAC_DMA_STREAM->CR =
        (DAC_DMA_CHSEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0 |        /* M2P */
        DMA_SxCR_MINC |
        DMA_SxCR_CIRC |
        DMA_SxCR_PSIZE_0 |      /* 16-bit peripheral */
        DMA_SxCR_MSIZE_0 |      /* 16-bit memory */
        DMA_SxCR_HTIE |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        (2u << DMA_SxCR_PL_Pos);

    /* Direct mode (FIFO disabled) is fine for this use case. */
    DAC_DMA_STREAM->FCR = 0;

    /* Enable NVIC for DAC output DMA stream. */
    NVIC_SetPriority(DAC_DMA_STREAM_IRQn, 7);
    NVIC_EnableIRQ(DAC_DMA_STREAM_IRQn);
}

void SigDac_Start(void)
{
    /* Configure stream and enable it before starting the master timer. */
    dac_dma_init_stream();
    DAC_DMA_STREAM->CR |= DMA_SxCR_EN;
}

#endif /* SIG_ENABLE_DAC_OUT */

/**
 * @brief DAC output DMA ISR hook.
 *
 * Clears interrupt flags and updates counters. Keep it short; do not do DSP here.
 * Called from DMA1_Stream5_IRQHandler().
 */
void SigDac_OutDmaIRQHandler(void)
{
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
        /* Transfer error: clear and stop stream (optional safety). */
        DAC_DMAx->HIFCR = DMA_HIFCR_CTEIF5;
        DAC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    }
#endif
}

/* ============================================================================
 * Input DMA init/start
 * ============================================================================
 *
 * Input DMA uses TIM2 update DMA request and reads TIM2->CCR1 as peripheral source.
 * TIM2->CCR1 is updated once per sample in SigGen_OnTick() (called from TIM2 IRQ).
 */

void SigDma_TestInit(void)
{
    /* Initialize generator state (LUT + DDS params). */
    SigGen_Init();

    /* Configure TIM2 for sample rate and TRGO (used by DAC if enabled). */
    tim2_init_fs();
    tim2_disable_update_irq();

#if SIG_ENABLE_DAC_OUT
    /* Configure DAC (trigger + DMA requests). */
    SigDac_Init();
#endif

    /* Enable DMA1 clock (input stream uses DMA1). */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* Stop the input stream if running. */
    if (SIG_DMA_STREAM->CR & DMA_SxCR_EN) {
        SIG_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        while (SIG_DMA_STREAM->CR & DMA_SxCR_EN) { /* wait */ }
    }

    /* Clear pending flags for Stream1 (in LISR/LIFCR). */
    SIG_DMAx->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 |
                      DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;

    /* Configure stream registers. */
    SIG_DMA_STREAM->CR   = 0;
    SIG_DMA_STREAM->NDTR = SIG_DMA_BUF_LEN;

    /* Seed CCR1 before starting (ensures deterministic first DMA reads). */
    TIM2->CCR1 = s_seed_val;

    /* Peripheral address: TIM2->CCR1 (hack source register). */
    SIG_DMA_STREAM->PAR  = (uint32_t)&TIM2->CCR1;

    /* Memory address: input ring buffer. */
    SIG_DMA_STREAM->M0AR = (uint32_t)&sig_dma_buf[0];

    /* Stream configuration:
     *  - Channel selection: TIM2_UP request
     *  - DIR = 00 (peripheral-to-memory)
     *  - MINC = 1, PINC = 0
     *  - CIRC = 1
     *  - PSIZE/MSIZE = 16-bit
     *  - HT/TC/TE interrupts enabled
     *  - Priority high
     */
    SIG_DMA_STREAM->CR =
        (SIG_DMA_CHSEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC |
        DMA_SxCR_CIRC |
        DMA_SxCR_PSIZE_0 |      /* 16-bit peripheral */
        DMA_SxCR_MSIZE_0 |      /* 16-bit memory */
        DMA_SxCR_HTIE |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        (2u << DMA_SxCR_PL_Pos);

    SIG_DMA_STREAM->FCR = 0;

    /* Freeze TIM2 when debugging (optional but extremely useful). */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;

    /* Enable DMA request on TIM2 update events (UDE). */
    TIM2->DIER |= TIM_DIER_UDE;

    /* Enable TIM2 update interrupt to run the generator tick. */
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable NVIC for input DMA stream. */
    NVIC_SetPriority(SIG_DMA_STREAM_IRQn, 6);
    NVIC_EnableIRQ(SIG_DMA_STREAM_IRQn);

    /* Reset counters. */
    sig_dma_ht_count   = 0;
    sig_dma_tc_count   = 0;
    sig_dma_last_is_ht = 0;
}

void SigDma_TestStart(void)
{
    /* Start output DMA first (so DAC has data once the timer starts). */
#if SIG_ENABLE_DAC_OUT
    SigDac_Start();
#endif

    /* Start input DMA stream. */
    SIG_DMA_STREAM->CR |= DMA_SxCR_EN;

    /* Start TIM2 (master clock). */
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Input DMA ISR hook.
 *
 * Clears interrupt flags and updates HT/TC counters.
 * Called from DMA1_Stream1_IRQHandler().
 */
void SigDma_TestIRQHandler(void)
{
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
        /* Transfer error: clear and stop stream (optional safety). */
        SIG_DMAx->LIFCR = DMA_LIFCR_CTEIF1;
        SIG_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    }
}

/* ============================================================================
 * Generator init/tick
 * ============================================================================
 */

void SigGen_Init(void)
{
    /* Build cosine lookup table once at startup. */
    for (uint32_t i = 0; i < COS_LUT_SIZE; i++) {
        const float a = 2.0f * (float)M_PI * (float)i / (float)COS_LUT_SIZE;
        cos_lut[i] = cosf(a);
    }

    /* Configure DDS oscillators from compile-time parameters. */
    t1.phase = 0;
    t1.inc   = phase_inc_from_freq(SIG_TONE1_FREQ_HZ);
    t1.amp   = SIG_TONE1_AMP;
    t1.en    = SIG_ENABLE_TONE1;

    t2.phase = 0;
    t2.inc   = phase_inc_from_freq(SIG_TONE2_FREQ_HZ);
    t2.amp   = SIG_TONE2_AMP;
    t2.en    = SIG_ENABLE_TONE2;

    t3.phase = 0;
    t3.inc   = phase_inc_from_freq(SIG_TONE3_FREQ_HZ);
    t3.amp   = SIG_TONE3_AMP;
    t3.en    = SIG_ENABLE_TONE3;
}

/**
 * @brief Generate one new sample at SIG_FS_HZ.
 *
 * Called from TIM2_IRQHandler() once per TIM2 update event.
 *
 * Important:
 *  - Keep this function short and deterministic.
 *  - The generated sample is written to TIM2->CCR1 so the input DMA can read it.
 *    This introduces a one-sample pipeline (DMA reads the register value at the
 *    time of request); this is acceptable and deterministic.
 */
void SigGen_OnTick(void)
{
    float s = 0.0f;

    if (t1.en) { s += t1.amp * fast_cos_u32(t1.phase); t1.phase += t1.inc; }
    if (t2.en) { s += t2.amp * fast_cos_u32(t2.phase); t2.phase += t2.inc; }
    if (t3.en) { s += t3.amp * fast_cos_u32(t3.phase); t3.phase += t3.inc; }

#if SIG_ENABLE_NOISE
    s += SIG_NOISE_AMP * white_noise_uniform();
#endif

    /* Convert to signed 16-bit audio sample. */
    const int16_t q = float_to_q15_clip(s);

    /* Hack source register for input DMA (P2M). */
    TIM2->CCR1 = (uint16_t)q;

#if SIG_ENABLE_DAC_OUT
    /* Optional: if you ever want to directly feed the DAC buffer from here,
     * do NOT do it in lockstep mode; keep block processing in main instead.
     * The DAC output is filled by the CPU block code in main.c currently.
     */
    (void)s16_to_dac_u12;
#endif
}