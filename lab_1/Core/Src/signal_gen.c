#include "signal_gen.h"
#include "stm32f207xx.h"
#include <math.h>

// ---------------------------------------------------------------------------
// TIM2 -> DMA Test (Peripheral-to-Memory, Circular, HT/TC IRQ)
//
// Implementation detail for STM32F207:
// TIM2 Update DMA request mapping (DMA1) is documented in RM0033, Table 22.
// One valid choice is DMA1 Stream1, Channel 3 for TIM2_UP. citeturn0search12turn0search17
// ---------------------------------------------------------------------------

volatile uint32_t sig_dma_ht_count = 0;
volatile uint32_t sig_dma_tc_count = 0;
volatile uint32_t sig_dma_last_is_ht = 0;
int16_t           sig_dma_buf[SIG_DMA_BUF_LEN];
uint32_t          val = 0xBEAF;

#define SIG_DMAx              DMA1
#define SIG_DMA_STREAM        DMA1_Stream1
#define SIG_DMA_STREAM_IRQn   DMA1_Stream1_IRQn
// CHSEL = 3 -> "Channel 3" in RM tables.
#define SIG_DMA_CHSEL         (3u)

// ---------------- Ringbuffer ----------------
//static volatile uint32_t wr = 0;
//static volatile uint32_t rd = 0;
//static int16_t ring[SIG_RING_LEN];
//
//static inline uint32_t ring_mask(void) { return (SIG_RING_LEN - 1u); }
//
//size_t SigGen_Available(void) {
//    return (size_t)((wr - rd) & 0xFFFFFFFFu);
//}
//
//size_t SigGen_ReadBlock(int16_t* dst, size_t n) {
//    size_t avail = SigGen_Available();
//    if (n > avail) n = avail;
//
//    for (size_t i = 0; i < n; i++) {
//        dst[i] = ring[rd & ring_mask()];
//        rd++;
//    }
//    return n;
//}

// ---------------- Fast Cos via LUT ----------------
// 1024er Cos-Tabelle (Q15-ish wäre auch möglich; hier float für Einfachheit)
#define COS_LUT_BITS   10u
#define COS_LUT_SIZE   (1u << COS_LUT_BITS)
static float cos_lut[COS_LUT_SIZE];

static inline float fast_cos_u32(uint32_t phase) {
    // phase: 0..2^32-1 -> Index 0..COS_LUT_SIZE-1
    uint32_t idx = phase >> (32u - COS_LUT_BITS);
    return cos_lut[idx];
}

// ---------------- DDS ----------------
typedef struct {
    uint32_t phase;
    uint32_t inc;
    float    amp;
    uint8_t  en;
} dds_t;

static dds_t t1, t2, t3;

// ---------------- Noise (xorshift32) ----------------
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
    // uniform in [-1, +1]
    uint32_t r = xorshift32();
    // 24-bit mantissa-ish
    float u = (float)(r & 0x00FFFFFFu) * (1.0f / 16777215.0f);
    return 2.0f * u - 1.0f;
}

static inline uint32_t phase_inc_from_freq(float f_hz) {
    // inc = f * 2^32 / Fs
    // Achtung: float -> uint32, rounding ok
    double inc = (double)f_hz * (4294967296.0 / (double)SIG_FS_HZ);
    if (inc < 0) inc = 0;
    if (inc > 4294967295.0) inc = 4294967295.0;
    return (uint32_t)inc;
}

// ---------------- Optional DAC (PA4 -> DAC1) ----------------
//#if SIG_ENABLE_DAC_OUT
//static inline void dac_init_pa4(void) {
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//    // PA4 analog
//    GPIOA->MODER |= (3u << (4u * 2u));
//
//    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
//    // enable DAC channel 1
//    DAC->CR |= DAC_CR_EN1;
//}
//static inline void dac_write_u12(uint16_t v) {
//    // 12-bit right aligned
//    DAC->DHR12R1 = (uint32_t)(v & 0x0FFFu);
//}
//#endif

// ---------------- TIM2 @ Fs ----------------
static void tim2_init_fs(void) {
    // TIM2 clock: APB1 timer clock = 2*PCLK1 when APB1 prescaler != 1
    // In deinem Clock-Setup: HCLK=120MHz, APB1=HCLK/4=30MHz, Timerclk=60MHz
    const uint32_t tim_clk_hz = 60000000u;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;
    TIM2->PSC = 0; // prescaler
    // round to nearest for minimal Fs error
    const uint32_t arr = (uint32_t)((tim_clk_hz + (SIG_FS_HZ / 2u)) / SIG_FS_HZ);
    TIM2->ARR = (arr > 0u) ? (arr - 1u) : 0u; // auto-reload
    TIM2->EGR = TIM_EGR_UG; // update event to load regs

    // (UIE/UIF handling is used by the "old" SigGen_OnTick path.
    // For the DMA test we enable UDE instead in SigDma_TestInit().)
}

static void tim2_disable_update_irq(void) {
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->SR   &= ~TIM_SR_UIF;
}

void SigDma_TestInit(void) {
    SigGen_Init();
    // TIM2 @ SIG_FS_HZ (default SIG_FS_HZ can be overridden to 44100 in signal_gen.h)
    tim2_init_fs();
    tim2_disable_update_irq();

    // --- DMA clock ---
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // --- Stop + reset stream ---
    if (SIG_DMA_STREAM->CR & DMA_SxCR_EN) {
        SIG_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        while (SIG_DMA_STREAM->CR & DMA_SxCR_EN) { /* wait */ }
    }

    // Clear pending flags for Stream1 (LISR bits)
    // Stream1 flags are in DMA1->LISR / DMA1->LIFCR.
    SIG_DMAx->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 |
                      DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;

    // --- Configure stream ---
    SIG_DMA_STREAM->CR = 0;
    SIG_DMA_STREAM->NDTR = SIG_DMA_BUF_LEN;
    // Peripheral address: TIM2->ARR (konstanter Wert) -> ideal zum Testen
    //SIG_DMA_STREAM->PAR  = (uint32_t)&TIM2->ARR;
    TIM2->CCR1 = val;  // This is a hack in order to read real values with DMA peripheral-to-mem. // initial “seed”
    SIG_DMA_STREAM->PAR = (uint32_t)&TIM2->CCR1;
    //SIG_DMA_STREAM->PAR  = (uint32_t)&val;
    // Memory address
    SIG_DMA_STREAM->M0AR = (uint32_t)&sig_dma_buf[0];

    // Direction: Peripheral-to-memory (DIR=00)
    // Peripheral size: 32-bit (PSIZE=10), Memory size: 32-bit (MSIZE=10)
    // Memory increment: ON (MINC), Peripheral increment: OFF
    // Circular: ON
    // Interrupts: HT + TC + TE
    SIG_DMA_STREAM->CR =
        (SIG_DMA_CHSEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC |
        DMA_SxCR_CIRC |
        DMA_SxCR_PSIZE_0 |  // 16-bit peripheral
        DMA_SxCR_MSIZE_0 |  // 16-bit memory
        DMA_SxCR_HTIE |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        (2u << DMA_SxCR_PL_Pos); // priority high

    // FIFO disabled (direct mode) is fine here
    SIG_DMA_STREAM->FCR = 0;

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;

    // Enable DMA request on TIM2 update event
    TIM2->DIER |= TIM_DIER_UDE;

    // Generator tick @ Fs:
    TIM2->DIER |= TIM_DIER_UIE;        // Update Interrupt enable
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);

    // NVIC for DMA stream
    NVIC_SetPriority(SIG_DMA_STREAM_IRQn, 6);
    NVIC_EnableIRQ(SIG_DMA_STREAM_IRQn);

    // Reset counters
    sig_dma_ht_count = 0;
    sig_dma_tc_count = 0;
    sig_dma_last_is_ht = 0;
}

void SigDma_TestStart(void) {
    // Start DMA stream first (so we don't miss early requests)
    SIG_DMA_STREAM->CR |= DMA_SxCR_EN;
    // Start TIM2
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void SigDma_TestIRQHandler(void) {
    // Stream1 interrupts live in DMA1->LISR
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
        // Transfer error: clear and stop (optional)
        SIG_DMAx->LIFCR = DMA_LIFCR_CTEIF1;
        SIG_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    }
}

void SigGen_Init(void) {
    // LUT init
    for (uint32_t i = 0; i < COS_LUT_SIZE; i++) {
        float a = 2.0f * (float)M_PI * (float)i / (float)COS_LUT_SIZE;
        cos_lut[i] = cosf(a);
    }

    // DDS setup
    t1.phase = 0; t1.inc = phase_inc_from_freq(SIG_TONE1_FREQ_HZ); t1.amp = SIG_TONE1_AMP; t1.en = SIG_ENABLE_TONE1;
    t2.phase = 0; t2.inc = phase_inc_from_freq(SIG_TONE2_FREQ_HZ); t2.amp = SIG_TONE2_AMP; t2.en = SIG_ENABLE_TONE2;
    t3.phase = 0; t3.inc = phase_inc_from_freq(SIG_TONE3_FREQ_HZ); t3.amp = SIG_TONE3_AMP; t3.en = SIG_ENABLE_TONE3;

    //wr = rd = 0;

#if SIG_ENABLE_DAC_OUT
    dac_init_pa4();
#endif

    tim2_init_fs();
}

void SigGen_Start(void) {
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static inline int16_t float_to_q15_clip(float x) {
    if (x > 0.999969f) x = 0.999969f;
    if (x < -1.0f)     x = -1.0f;
    int32_t v = (int32_t)lrintf(x * 32768.0f);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
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

    // Wichtig: CCR1 vor dem *nächsten* DMA-Request setzen (Pipeline um 1 Sample ist ok)
    TIM2->CCR1 = (uint16_t)q;   // Bits bleiben identisch, DMA liest die 16 Bit roh.
}