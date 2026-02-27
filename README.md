# STM32 DSP Lab

Real-time digital signal processing experiments on STM32 microcontrollers.

---

## Overview

STM32 DSP Lab is a structured collection of firmware experiments focused on **real-time digital signal processing under embedded constraints**.

Unlike simulation-only environments, this project emphasizes:

- Deterministic timing behavior
- Hardware-driven data movement (DMA, timers, interrupts)
- Measurable latency and throughput
- Practical trade-offs between numerical formats
- Reproducible experiments on real silicon

All implementations are written in C and designed to run in minimal runtime or bare-metal environments.

The guiding principle is simple:

> DSP theory is only interesting if it survives contact with clock cycles.

---

## Project Goals

This repository serves as a long-term experimental platform to:

- Implement fundamental DSP building blocks (filters, transforms, modulation)
- Explore real-time constraints and deterministic execution
- Compare floating-point and fixed-point implementations
- Measure performance across STM32 families
- Analyze memory usage, latency, and throughput
- Evaluate CMSIS-DSP versus custom implementations
- Build reusable real-time streaming backbones

The emphasis is on **measurable behavior**, not abstract correctness alone.

---

## Design Principles

- Deterministic execution over abstraction convenience
- Hardware-timed pipelines (timer + DMA driven)
- Clear separation between algorithm and hardware layers
- Minimal hidden complexity
- Explicit control over memory layout and buffer handling
- Transparent interrupt behavior
- Reproducible and benchmarkable experiments

Every lab builds on a stable streaming foundation before adding computational complexity.

---

## Target Platform

Primary target:

- STM32 microcontrollers (starting with STM32F2xx family)
- ARM Cortex-M cores
- Bare-metal or minimal runtime environments
- C language

Optional components:

- CMSIS
- CMSIS-DSP
- ST HAL or direct register-level programming

Specific board configurations and MCU families will be documented per lab.

---

## Repository Structure

Current and planned structure:

```
/core        -> Shared utilities and hardware abstraction
/dsp         -> DSP algorithm implementations
/examples    -> Self-contained demo applications
/drivers     -> Peripheral configuration (ADC, DAC, DMA, timers)
/benchmarks  -> Performance and timing measurements
/docs        -> Notes, experiments, theory references
/labs        -> Structured step-by-step lab exercises
```

The structure may evolve as the project matures.

---

## Implemented / Planned Topics

Signal Processing:

- FIR filters
- IIR filters
- FFT and spectral analysis
- Window functions
- Envelope detection
- Modulation techniques
- Noise generation
- Audio processing pipelines

System-Level Topics:

- DMA-based circular streaming
- Ping-pong buffer architectures
- Interrupt-driven processing windows
- Real-time scheduling constraints
- Fixed-point arithmetic strategies
- Latency analysis
- Cache and memory alignment considerations

---

## Lab Structure

The project is organized into incremental labs.

Example progression:

- Lab 1: Deterministic DMA-based streaming backbone
- Lab 2: Inserting real-time DSP into ping-pong windows
- Lab 3: Fixed-point optimization
- Lab 4: Performance benchmarking and profiling
- Lab 5+: Advanced processing (spectral, modulation, filtering chains)

Each lab introduces controlled complexity while preserving real-time guarantees.

---

## Build System

Typical toolchain:

- ARM GNU Toolchain (arm-none-eabi-gcc)
- Makefile or CMake-based build
- Optional STM32CubeMX for peripheral scaffolding
- OpenOCD / ST-Link for flashing and debugging

Exact build instructions are provided per lab once stable configurations are finalized.

---

## Measurement & Validation Philosophy

All experiments should be:

- Observable (UART logging, FFT visualization, buffer dumps)
- Measurable (cycle counts, CPU load, memory footprint)
- Repeatable
- Hardware-verified

Simulation is useful — but hardware truth is final.

---

## Status

This project is in active development.

The streaming backbone is established.
Subsequent labs will expand into structured real-time DSP processing and performance analysis.
