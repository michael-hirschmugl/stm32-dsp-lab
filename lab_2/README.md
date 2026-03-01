# Lab 2 -- Lockstep Audio Pipeline with FFT (STM32F207ZG)

## Overview

This lab implements a deterministic, lockstep audio processing pipeline
on an STM32F207ZG microcontroller.\
The system captures samples at 44.1 kHz using a timer-driven DMA
mechanism, performs block-based FFT analysis using CMSIS-DSP (Q15), and
streams the magnitude spectrum to a host PC via UART for live
visualization.

The design emphasizes: - Deterministic timing - Clean separation between
interrupt context and processing context - Efficient fixed-point DSP
using CMSIS-DSP - Structured binary communication protocol over UART -
Cross-platform live visualization with Python and Matplotlib

------------------------------------------------------------------------

## System Architecture

### High-Level Signal Flow

               TIM2 @ 44.1 kHz
                     │
                     ▼
             DMA (P2M, circular)
                     │
             sig_dma_buf[] (double buffer)
                     │
              Main loop (poll HT/TC)
                     │
              cpu_block[]
                     │
              CMSIS-DSP FFT (Q15)
                     │
             Magnitude spectrum
                     │
                UART (binary)
                     │
               Python Live Plot

------------------------------------------------------------------------

## Embedded Side (STM32)

### Core Concepts

#### 1. Lockstep Processing

The DMA operates in circular mode with half-transfer (HT) and
transfer-complete (TC) interrupts.

-   HT → first half-buffer ready
-   TC → second half-buffer ready

The main loop polls counters incremented by the DMA ISR.\
No DSP is performed inside interrupts.

This guarantees: - Predictable execution - No ISR jitter from heavy
computation - Deterministic mapping of input block → output block

------------------------------------------------------------------------

#### 2. Buffer Structure

    sig_dma_buf[0 … N/2-1]       → first half
    sig_dma_buf[N/2 … N-1]       → second half

At each half event: 1. Copy stable half to `cpu_block[]` 2. Run FFT 3.
Compute magnitude 4. Optionally forward to DAC 5. Stream spectrum over
UART

------------------------------------------------------------------------

#### 3. FFT Configuration

-   FFT length: 256
-   Sampling rate: 44.1 kHz
-   FFT bins transmitted: N/2 + 1 = 129
-   Format: Q15 complex input
-   Library: CMSIS-DSP

The frequency resolution is:

    Δf = Fs / N = 44100 / 256 ≈ 172.27 Hz

------------------------------------------------------------------------

## UART Protocol

Each transmitted frame has the following format (little-endian):

  Field           Size   Description
  --------------- ------ ------------------
  SYNC            2 B    0xA5 0x5A
  Frame Counter   4 B    uint32
  Count           2 B    uint16
  Payload         2×N    int16 magnitudes
  Checksum        2 B    Optional

Payload contains `count` signed 16-bit magnitude values.

------------------------------------------------------------------------

## Host Side (Python)

### plot_fft_uart.py

Responsibilities:

-   Robust stream parsing
-   Sync detection
-   Frame validation
-   Conversion Q15 → float
-   Live Matplotlib visualization

The frequency axis is reconstructed using:

    f[k] = k * Fs / N

### Requirements

Python 3.10+ recommended.

Install dependencies:

``` bash
pip install pyserial numpy matplotlib
```

Run:

``` bash
python3 plot_fft_uart.py
```

------------------------------------------------------------------------

## Build System

The project uses a Makefile-based GCC ARM toolchain build.

Generated artifacts are placed in:

    build/

Important outputs:

-   `.elf` → Debug image
-   `.hex` → Flashable image
-   `.map` → Linker memory map
-   `.bin` → Raw binary

------------------------------------------------------------------------

## Flashing

Using OpenOCD:

``` bash
./scripts/openocd.sh
```

Or manual:

``` bash
openocd -f interface/stlink.cfg -f target/stm32f2x.cfg
```

Then in another terminal:

``` bash
arm-none-eabi-gdb build/stm32-lab-1.elf
```

------------------------------------------------------------------------

## Debugging Strategy

### LED Heartbeat

LD2 toggles on each processed half-block.

### Frame Rate Monitoring

Python prints:

    fps ~ XX.X, buffered=YY

This verifies: - UART throughput - Parser stability - Frame continuity

------------------------------------------------------------------------

## Performance Considerations

### Determinism

-   Timer-triggered DMA ensures fixed sample rate
-   No blocking inside ISR
-   UART transmission at 10 Hz avoids bandwidth saturation

### Memory Footprint

-   Q15 FFT avoids floating-point overhead
-   Double buffering ensures safe concurrent access
-   CMSIS-DSP optimized for Cortex-M3

------------------------------------------------------------------------

## Possible Extensions

-   Windowing (Hann / Hamming)
-   Log-magnitude display (dB scale)
-   Real audio ADC input
-   FIR filtering before FFT
-   Real-time spectral peak detection
-   USB CDC instead of UART
-   DMA-based UART transmission

------------------------------------------------------------------------

## Learning Objectives

By completing this lab, you should understand:

-   Timer-triggered DMA on STM32
-   Circular double buffering
-   Half-transfer interrupt strategy
-   Fixed-point DSP using CMSIS
-   Real-time constraints in embedded systems
-   Binary protocol design
-   Host-side visualization pipelines

------------------------------------------------------------------------

## Directory Structure

    lab_2/
     ├── Core/
     ├── Drivers/
     ├── build/
     ├── python/
     ├── scripts/
     ├── Makefile
     └── STM32F207XX_FLASH.ld

------------------------------------------------------------------------

## Conclusion

This lab demonstrates a complete real-time DSP pipeline:

Deterministic sampling → Block processing → Spectral analysis → Binary
streaming → Live visualization.

It forms a foundation for: - Digital audio effects - Spectrum
analyzers - Embedded signal diagnostics - Real-time monitoring systems
