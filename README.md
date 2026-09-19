# Deluxe Analog — BBD Delay Pedal (STM32G474 control firmware)

Firmware for the control side of an analog bucket-brigade (BBD) delay pedal.
The STM32 generates the BBD clock and handles the user interface; the audio
path itself stays fully analog.

## Hardware

| Block | Part | Notes |
|---|---|---|
| MCU | STM32G474RET6 (NUCLEO-G474RE) | 170 MHz, HRTIM + CORDIC |
| Delay line | Panasonic MN3005 | 4096-stage BBD, two-phase clock |
| Noise reduction | SA572 | compandor: compress in / expand out |
| Level control | PGA2310 | SPI volume control — **not yet implemented** |

### Pin map

| Pin | Function |
|---|---|
| PA8 | HRTIM1 CHA1 — BBD clock output |
| PA10 | HRTIM1 CHB1 — reserved (2nd clock phase) |
| PA0 | ADC1_IN1 — Delay Time pot |
| PA1 | ADC1_IN2 — LFO Rate pot |
| PB0 | ADC1_IN15 — LFO Depth pot |
| PC13 | User button — tap tempo (active high) |
| PA5 | User LED — tempo metronome |
| PA2/PA3 | LPUART1 — debug (initialised, unused) |

## Delay-time maths

The MN3005 has 4096 stages clocked in two phases, so

```
delay = 2048 / f_clock
```

| Delay | BBD clock | HRTIM ticks @ 42.5 MHz |
|---|---|---|
| 20 ms | 102.4 kHz | 425 |
| 512 ms | 4.0 kHz | 10625 |

HRTIM runs at `f_HRTIM / 4` = **42.5 MHz** (`HRTIM_PRESCALERRATIO_DIV4`).
This prescaler is mandatory: at the CubeMX default of `MUL32` the HRTIM cannot
produce anything below roughly 83 kHz, putting the whole delay range out of reach.

## Features

- HRTIM BBD clock, 4 kHz – 100 kHz, 50 % duty, updated live
- Three pots polled on ADC1, quadratic audio taper on delay time
- Wow & flutter LFO via the hardware CORDIC, ±10 % of current delay
- Tap tempo: 4 ms shift-register debounce, averages up to 4 taps,
  locks in 1.5 s after the last tap
- "Last touched" priority — turning the Time pot overrides a tapped tempo
- LED metronome synced to the current delay time

## Building

Import into STM32CubeIDE (project targets CubeMX 6.18 / HAL for STM32G4) and
build the `Debug` configuration. `Core/Src/snippets.c` is a scratchpad of
unused draft code and is excluded from the build in `.cproject`.

> When regenerating from `NUCLEO_Delay.ioc`, confirm the HRTIM prescaler still
> reads `fHRTIM / 4` in the Master / Timer A / Timer B timebase settings.

## Status

Working: BBD clock, pots, tap tempo, LED. See `docs/findings.md` in the project
notes for known issues — chiefly the LFO rate scaling and the unconfigured
CORDIC — and for open hardware questions (two-phase BBD clocking, PGA2310).
