# STM32 DSP Lab

Real-time digital signal processing experiments on STM32 microcontrollers.

## Overview

This repository contains firmware prototypes and experimental implementations of digital signal processing (DSP) algorithms running on STM32 hardware.
The goal is to explore DSP concepts under real-time constraints, directly on embedded systems programmed in C. The focus lies on practical implementation, performance analysis, and hardware-aware optimization.
This project is intended as a structured playground for testing and validating signal processing ideas on actual microcontrollers rather than in simulation-only environments.

## Objectives

* Implement fundamental DSP building blocks (filters, transforms, modulation, etc.)
* Explore real-time constraints and deterministic timing behavior
* Compare floating-point and fixed-point approaches
* Investigate performance limits of different STM32 families
* Evaluate memory usage, latency, and throughput
* Experiment with CMSIS-DSP and custom implementations

## Target Platform

* STM32 microcontrollers
* Bare-metal or minimal runtime environments
* Written in C
* Optional use of CMSIS and CMSIS-DSP

Specific board configurations and supported MCU families will be documented as the project evolves.

## Repository Structure (Planned)

```Code/core```          -> Shared utilities and hardware abstraction  
```/dsp```               -> DSP algorithm implementations  
```/examples```          -> Self-contained demo applications  
```/drivers```           -> Peripheral configuration (ADC, DAC, DMA, etc.)  
```/benchmarks```        -> Performance and timing tests  
```/docs```              -> Notes and theoretical background

Structure may evolve as the project grows.

## Implemented / Planned Topics

* FIR and IIR filters
* FFT and spectral analysis
* Window functions
* Signal generation (sine, noise, modulation)
* Audio processing
* Fixed-point arithmetic techniques
* DMA-based real-time streaming
* Interrupt-driven processing pipelines

## Design Philosophy

* Deterministic execution over abstraction convenience
* Clear separation between algorithm and hardware layer
* Measurable performance
* Reproducible experiments
* Minimal hidden complexity

DSP theory is only interesting if it survives contact with clock cycles.

## Build

Toolchain and build instructions will be added once a baseline configuration is established.

Typical setup:
* ARM GCC toolchain
* STM32CubeMX or manual peripheral configuration
* Makefile or CMake-based build system

## Status

This project is currently in its early experimental phase.
