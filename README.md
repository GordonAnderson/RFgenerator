# RF Generator Firmware

Firmware for the GAA Custom Electronics RF Generator board — a standalone RF
driver with USB and Ethernet host interfaces, built on the Adafruit ItsyBitsy
M4 (SAMD51) and managed with PlatformIO.

Author: Gordon Anderson

## Hardware

- **MCU**: Adafruit ItsyBitsy M4 Express (ATSAMD51, `adafruit_itsybitsy_m4`)
- **RF clock source**: Si5351 PLL (Rev 2 boards) or FS7140 PLL (Rev 1 boards)
- **Bias control**: on-board 12-bit DAC drives a FET to set RF drive level / DC bias
- **Relays**: up to 4 relays for symmetrical inductance switching on the RF coil
- **I/O**: push button (on/off + tune), status/RF LEDs, TTL gate input, hardware
  trigger input (ramp/quench), external RS-232 port, optional Ethernet-to-RS232
  adapter (USR-TCP232-T) bridged over the RS-232 port

### Board revisions

| Revision | Clock chip | Relays | Notes |
|---|---|---|---|
| Rev 1 | FS7140 | 3 | Original design (April 2022) |
| Rev 2 | Si5351 | 4 | Adds fast bias ramping, gate input, symmetric relay switching |

## Features

- USB virtual COM port and external RS-232 port, both running the same ASCII
  command protocol (see below)
- Optional Ethernet bridge auto-detected at boot (USR-TCP232-T over TWI/RS-232)
- Push-button control: short press toggles RF on/off, press-and-hold (3 s)
  starts auto-tune
- RF level lookup table (piecewise-linear calibration) as an alternative to
  the analog calibration model
- Closed-loop drive control (setpoint + gain) or open-loop drive control
- Hardware trigger input for gating the RF output, with selectable active
  level and an optional fast bias ramp on trigger edges
- Calibration wizards for DC bias, drive current sensor, and RF level (PWL
  table) run interactively over the serial connection
- Settings and calibration data persisted to internal flash (`FlashStorage`)
  and/or external SPI flash filesystem (`SAVEF`/`LOADF`/`SAVECAL`/`LOADCAL`)

## Building

This is a [PlatformIO](https://platformio.org/) project.

```bash
pio run                  # build
pio run -t upload        # build and flash
pio run -t clean         # clean build artifacts
```

Environment: `adafruit_itsybitsy_m4` (see [platformio.ini](platformio.ini)).
Library dependencies (fetched automatically by PlatformIO) include
`GAACE_Core`, `ArduinoThread`, `Adafruit_DotStar`, `FlashStorage`,
`arduino-timer`, `Adafruit_SPIFlash`, and `Si5351Arduino`.

### Build-time hardware configuration

The firmware currently selects its hardware variant via `#define`s at the top
of [include/RFgenerator.h](include/RFgenerator.h) rather than via PlatformIO
build flags/environments, so switching hardware variants means editing and
rebuilding from source:

| Define | Effect |
|---|---|
| `RFGEN2` | Build for Rev 2 hardware (Si5351 clock, 4 relays, fast bias ramping) |
| `SINGLEENDED` | Force the RF- channel reading to 0 (single-ended RF sense) |
| `TRIGINVERTED` | Invert the sense of `STRGL`/`GTRGL` for boards with an inverting trigger buffer |
| `QUENCH` | Enable the fast RF-quench relay logic for boards with the quench module installed |

## Serial command protocol

Commands are ASCII, comma-delimited, and terminated with `;`, `\r`, or `\n`.
A `G`-prefixed command reads a value; the matching `S`-prefixed command writes
it. Successful commands are acknowledged with `ACK` (`0x06`); failures return
`NAK` (`0x15?`) and set an error code retrievable with `GERR`. The full,
authoritative list is in [src/Serial.cpp](src/Serial.cpp) (`CmdArray`) and can
be queried live with `GCMDS`.

| Category | Commands |
|---|---|
| General | `GVER`, `GERR`, `GNAME`/`SNAME`, `MUTE`, `ECHO`, `DELAY`, `GCMDS`, `RESET`, `SAVE`, `RESTORE`, `FORMAT`, `THREADS`, `STHRDENA` |
| Ethernet | `ENTEST`, `GEIP`/`SEIP`, `GESNIP`/`SESNIP`, `GEPORT`/`SEPORT`, `GEGATE`/`SEGATE` |
| Flash filesystem | `SAVEF`/`LOADF`, `SAVECAL`/`LOADCAL` |
| RF generator | `SENA`/`GENA`, `SFREQ`/`GFREQ`, `SDRV`/`GDRV`, `SRFV`/`GRFV`, `GRFAVP`/`GRFAVN`, `GDRVV`, `GDRVI`, `GPWR`, `SGAIN`/`GGAIN`, `SMODE`/`GMODE`, `SMDRV`/`GMDRV`, `SMPWR`/`GMPWR`, `SRLY`/`GRLY`, `SBIAS`/`GBIAS` |
| Auto-tune | `TUNE`, `ISTUNE`, `SMAXTF`/`GMAXTF`, `SMINTF`/`GMINTF` |
| Bias ramp (RFGEN2 only) | `SRPSV`/`GRPSV`, `SRPEV`/`GRPEV`, `SRPTM`/`GRPTM`, `STRP`, `SRPTRG`/`GRPTRG`, `SRPCT`/`GRPCT`, `SRPRM`/`GRPRM` |
| Hardware trigger | `STRIG`/`GTRIG`, `STRGL`/`GTRGL` |
| Misc | `SGATE`/`GGATE`, `USEDMA` |
| Calibration | `CALBIAS`, `CALRFL`, `CALCUR` |

## Repository layout

```
include/    Header files (Hardware.h, RFgenerator.h, ethernet.h, FS7140.h, Serial.h, Button.h)
src/        Implementation files (RFgenerator.cpp is the sketch entry point: setup()/loop())
platformio.ini  Build configuration
```

## Version history

See the header comment in [src/RFgenerator.cpp](src/RFgenerator.cpp) for the
full design log. Latest released version: **1.7** (July 19, 2025) — added
continuous DMA output of the bias DAC value (`USEDMA` command).

## Known issues

See [TODO.md](TODO.md) for a list of bugs, dead code, and robustness issues
found during review.
