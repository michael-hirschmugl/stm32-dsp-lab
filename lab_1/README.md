# Lab 1 — Deterministic Audio Stream (TIM2 + DMA Ping‑Pong + DAC) on STM32F207

Lab 1 builds the **hardware‑timed streaming backbone** for later DSP labs.

Target hardware: **STM32F207 (Nucleo‑F207ZG)**

This lab implements:

- **TIM2** as a single master sample clock (e.g. **44.1 kHz**)
- **DMA in circular mode** with **Half Transfer / Transfer Complete** signaling (ping‑pong)
- A **DDS‑based multi‑tone signal generator** (optional white noise)
- **DAC output (PA4 / DAC_OUT1)** triggered by **TIM2_TRGO** and fed by **DMA (memory → peripheral)**
- Clean separation between **ISR duties** (minimal) and **CPU block work** (main loop)

**No DSP is performed in Lab 1.**  
Lab 2+ will insert processing into the same ping‑pong work window.

---

## 1) Goal

Create a deterministic, hardware‑driven streaming system that:

- generates **one sample per tick** (software generator)
- moves samples with **DMA** (hardware transport)
- exposes deterministic **half‑buffer work windows** (ping‑pong)
- optionally outputs the stream via **DAC** using the **same master clock**

---

## 2) System Architecture (High Level)

A single clock domain (TIM2) drives everything:

```text
                     +----------------------+
                     |      TIM2 @ Fs       |
                     |  (Update Event @ Fs) |
                     +----------+-----------+
                                |
             +------------------+------------------+
             |                                     |
             v                                     v
  (IRQ) TIM2_IRQHandler                      TIM2_TRGO = Update
  SigGen_OnTick()                                 |
  writes next sample                               v
  into TIM2->CCR1                             +----------+
                                             |   DAC    |
                                             | trigger  |
                                             +----+-----+
                                                  |
                                      (DMA request from DAC)
                                                  |
                                                  v
                                              DMA (M2P)
                                             dac_dma_buf[]
                                           -> DAC->DHR12R1
                                                  |
                                                  v
                                             PA4 (DAC_OUT1)
```

In parallel, the TIM2 update also triggers the **input DMA** which captures the generated sample:

```text
TIM2 Update DMA Request  -->  DMA (P2M)  --> sig_dma_buf[] (int16, circular)
                                   |--> HT interrupt (first half ready)
                                   |--> TC interrupt (second half ready)
```

---

## 3) Data Path Details

### 3.1 Signal Generator (software DDS)

The generator is implemented as a small DDS bank:

- up to **3 cosine oscillators** (compile‑time enable)
- optional **white noise** (xorshift32)
- output is produced **sample‑by‑sample** in `TIM2_IRQHandler`

Configuration lives in `signal_gen.h`:

- `SIG_ENABLE_TONE1/2/3`
- `SIG_TONE*_FREQ_HZ`, `SIG_TONE*_AMP`
- `SIG_ENABLE_NOISE`, `SIG_NOISE_AMP`
- `SIG_FS_HZ`

The generator output is `int16_t` samples (audio‑style signed).

**Important implementation detail (intentional “hack”):**  
The sample is written into **TIM2->CCR1** so that the **P2M DMA** can read a real
peripheral register that changes every sample.

---

### 3.2 TIM2: Master Sample Clock

TIM2 is configured for `SIG_FS_HZ`:

- Default: **44100 Hz**
- Timer clock assumption (from current clock tree): **60 MHz**
- `ARR` is computed to approximate Fs

TIM2 provides:

- **Update interrupt (UIE)** → runs `SigGen_OnTick()`
- **Update DMA request (UDE)** → triggers the input DMA (P2M)
- **TRGO = Update event** (`MMS=010`) → triggers DAC conversions

This keeps the whole system **hardware‑timed** and **deterministic**.

---

### 3.3 Input DMA (Peripheral → Memory): Ping‑Pong Buffer

**Purpose:** capture one generated sample per tick into a circular buffer.

- Source: `TIM2->CCR1`
- Destination: `sig_dma_buf[]` (`int16_t`)
- Mode: **circular**
- Interrupts: **HT**, **TC**, **TE**

Ping‑pong layout:

```text
sig_dma_buf (N samples):
+-----------------------+-----------------------+
|  first half (N/2)     |  second half (N/2)    |
+-----------------------+-----------------------+
        HT IRQ                     TC IRQ
```

**Safety rule:**

- On **HT**: DMA is writing the **second half**, so the CPU may safely read the **first half**
- On **TC**: DMA is writing the **first half**, so the CPU may safely read the **second half**

The ISR updates counters:

- `sig_dma_ht_count`
- `sig_dma_tc_count`
- `sig_dma_last_is_ht` (debug convenience)

The main loop watches these counters and consumes each half exactly once.

---

### 3.4 CPU Block Stage (no DSP yet)

Lab 1 does not modify audio. It only demonstrates safe block acquisition and a stable work window.

- `cpu_block[]` holds the copied input half‑buffer
- `cpu_block_processed[]` is currently **copy‑through** (placeholder)

This is where Lab 2 inserts DSP.

---

### 3.5 DAC Output (Memory → Peripheral), TIM2‑Triggered

**Purpose:** produce an analog signal synchronized to the same sample clock.

- Output pin: **PA4 (DAC_OUT1)**
- Trigger: **TIM2_TRGO** (update event)
- DAC DMA stream: circular **M2P** transfers
  - Source: `dac_dma_buf[]` (`uint16_t`, holds 12‑bit right‑aligned samples)
  - Destination: `DAC->DHR12R1`

In the main loop, when HT/TC indicates a new input half‑block, the firmware fills the corresponding half of `dac_dma_buf[]`:

- `int16_t` audio range `[-32768..32767]`
- mapped to DAC unsigned 12‑bit `[0..4095]` with midscale offset (2048)

This forms a lockstep “capture → optional processing → output” flow at block granularity.

---

## 4) Code Structure

### `signal_gen.h`
- compile‑time configuration (Fs, tones, noise, buffer sizes, feature flags)
- public API for generator, input DMA, DAC output
- exported counters and buffers

### `signal_gen.c`
- TIM2 configuration (Fs + TRGO)
- DDS + noise generator
- input DMA (P2M) configuration and ISR hook
- DAC configuration (trigger + DMA enable)
- DAC DMA (M2P) configuration and ISR hook

### `main.c`
- system init (HAL / Cube)
- starts the pipeline (`SigDma_TestInit()`, `SigDma_TestStart()`)
- polls HT/TC counters
- copies stable half‑buffer into CPU workspace
- copy‑through to processed buffer (placeholder for DSP)
- fills DAC output half‑buffer
- toggles an LED for activity visibility

### `stm32f2xx_it.c`
- minimal ISR glue only:
  - `TIM2_IRQHandler`: clear UIF + `SigGen_OnTick()`
  - `DMA1_Stream1_IRQHandler`: `SigDma_TestIRQHandler()`
  - `DMA1_Stream5_IRQHandler`: `SigDac_OutDmaIRQHandler()` (optional instrumentation)

---

## 5) What You Should Observe

- `sig_dma_ht_count` and `sig_dma_tc_count` increment steadily
- `cpu_blocks` increments once per half‑buffer consumed
- LED toggles at a stable cadence (block rate)
- analog output on **PA4 (DAC_OUT1)** when routed to a suitable load/amplifier

---

## 6) Hardware Notes (Nucleo‑F207ZG)

- DAC output is **PA4 / DAC_OUT1** (DAC channel 1).
- On Nucleo boards, “Arduino A* pins” are not guaranteed to match DAC pins.
  Use the board’s pinout to locate **PA4** on the headers.

**Electrical reality check:**
- The DAC pin is **not** a headphone driver.
- Prefer an amplifier / powered speaker / line‑in.
- If you must test with headphones, use proper coupling and protection (series resistor, capacitor),
  and keep volume expectations realistic.

---

## 7) Why This Matters

This is the core architecture of many embedded audio systems:

- hardware‑timed sample clock
- DMA transport to avoid jitter
- ping‑pong buffering for deterministic processing windows
- output synchronized to the same clock to prevent drift

Lab 1 answers the crucial pre‑DSP question:

> “Is my real‑time pipeline stable, deterministic, and block‑safe?”

---

## 8) Lab 2 Preview

Lab 2 will replace “copy‑through” with real DSP:

- process in the HT/TC work windows
- analyze CPU load and headroom
- explore latency vs. block size
- add first effects/filters (gain, clipping, FIR/IIR, delay, etc.)

Lab 1 is complete once the streaming backbone is stable and observable.
