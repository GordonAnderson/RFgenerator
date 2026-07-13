# TODO / Issues

Findings from a code review of the firmware. Items are grouped by severity.
File:line references point at the current source.

## Functional bugs

- [ ] **PWM duty-cycle register can exceed the PWM period, saturating drive
  above ~60%.** [src/Hardware.cpp:86](src/Hardware.cpp#L86) configures
  `TCC0->PER.reg = 600`, but `MAXDRIVE` is `999`
  ([include/Hardware.h:42](include/Hardware.h#L42)) and `setD10pwm()`
  ([src/Hardware.cpp:97-102](src/Hardware.cpp#L97-L102)) clamps to 999, not
  600. `setDrive()` computes `value = MAXDRIVE * percent / 100`, so any
  drive request above ~60% writes a `CC` value greater than the timer
  period — the comparator never matches within the period and the output
  saturates instead of continuing to scale linearly. Either restore
  `PER.reg = 999` or change `MAXDRIVE`/the `setD10pwm` clamp to 600 so the
  duty-cycle range matches the configured period. (The initial `CC[0].reg =
  740` in [src/Hardware.cpp:89](src/Hardware.cpp#L89) has the same
  >PER mismatch.)

- [ ] **`setFrequency()` doesn't do what its comment says.** A frequency
  outside 400 kHz–5 MHz should "turn off the RF signal to FET" but the code
  only `return`s — the FET/RF drive is left in whatever state it was in.
  [src/Hardware.cpp:130-138](src/Hardware.cpp#L130-L138).

- [ ] **DAC/ADC count clamp doesn't match configured resolution.**
  `Value2Counts()` clamps to `0..65535` (16-bit) in both overloads
  ([src/Hardware.cpp:29-47](src/Hardware.cpp#L29-L47)), but `setup()` configures
  12-bit resolution (`analogReadResolution(12)`, `analogWriteResolution(12)`,
  [src/RFgenerator.cpp:558-559](src/RFgenerator.cpp#L558-L559)) and the SAMD51
  DAC data register is 12-bit. A calibration (`m`/`b`) that produces a count
  above 4095 will silently write an out-of-range value to the DAC instead of
  saturating at 4095.

- [ ] **`si5351initPLL()`'s return value is discarded**, so a failed PLL init
  (chip not powered/not present) is never retried or reported — `init` is
  still latched `true` after the first call.
  [src/Hardware.cpp:139-145](src/Hardware.cpp#L139-L145) calling
  [src/Hardware.cpp:104-121](src/Hardware.cpp#L104-L121). The changelog
  ("Improved startup, it was locking up trying to turn off the PLL with no
  power on the chip", v1.3) suggests this area has bitten before.

- [ ] **Possible stale `FreqMax` on the scan-down→scan-up transition in
  auto-tune.** In `RFdriver_tune()`'s `TUNE_SCAN_DOWN` case
  ([src/RFgenerator.cpp:689-714](src/RFgenerator.cpp#L689-L714)), `rfgendata.Freq`
  is decremented, then (if the down-scan just finished) reset back to the
  scan's starting frequency *before* the `if(Current > Max)` check that
  records `FreqMax = rfgendata.Freq`. On the iteration where the state
  transitions to `TUNE_SCAN_UP`, `FreqMax` can be recorded as the scan's
  start frequency rather than the frequency at which `Current` was actually
  sampled. Worth re-verifying against a scope/known-good tune, especially
  given the comment "Move this code here to force tune system to not use the
  limit value" suggesting this was already adjusted once.

## Concurrency / real-time hazards

- [ ] **Bias ramp execution disables all interrupts for the full ramp
  duration, and can be triggered from inside a GPIO ISR.**
  `executeRamp()` ([src/RFgenerator.cpp:1313-1328](src/RFgenerator.cpp#L1313-L1328))
  wraps its loop in `noInterrupts()`/`interrupts()`; `trigRamp_isr()`
  ([src/RFgenerator.cpp:1355-1392](src/RFgenerator.cpp#L1355-L1392)), attached
  with `attachInterrupt(..., CHANGE)`, calls `prepairRamp()` +
  `executeRamp()` directly. For long ramp times this can block SysTick
  (`millis()`/`delay()`), USB, and every other interrupt source for tens of
  milliseconds on every trigger edge — risking dropped USB/serial data,
  missed trigger edges, and `millis()` drift. Ramp execution should happen
  outside ISR context (e.g. flag it and run from the main loop or a thread).

- [ ] **Shared state is read/written from both ISR and main-loop/thread
  context without synchronization.** `currentBias` and fields of
  `rfgendata` (e.g. `.Bias`) are written from `trigRamp_isr`/`trigFast_isr`
  and also from `loop()` ([src/RFgenerator.cpp:770](src/RFgenerator.cpp#L770))
  and the `Update()` thread, with no `noInterrupts()`/atomic guard around the
  non-ISR accesses. `AtomicBlock.h` is already included in
  [src/Hardware.cpp:4](src/Hardware.cpp#L4) but never actually used anywhere
  in the project — consider applying it to these shared variables.

- [ ] `loop()` unconditionally calls `writeBIAS` on every iteration when
  `currentBias > 0` with a comment acknowledging it "causes 10uS interrupt
  jitter on IO lines" ([src/RFgenerator.cpp:770](src/RFgenerator.cpp#L770)).
  Worth re-evaluating whether this still needs to run every loop iteration
  now that DMA-based output (`USEDMA`) exists as an alternative.

## Dead code

- [ ] `ProcessEthernet()` ([src/ethernet.cpp:194-204](src/ethernet.cpp#L194-L204))
  is never called — `Serial1` is already drained directly inside
  `ProcessSerial()` ([src/RFgenerator.cpp:606-610](src/RFgenerator.cpp#L606-L610)).
  Remove it or remove the duplicate logic in `ProcessSerial`.
- [ ] `LoadAltRev()` ([src/Serial.cpp:196-209](src/Serial.cpp#L196-L209)) is
  never called from anywhere (not in `setup()`, not in the command table).
  Either wire it up or remove it.
- [ ] `valueUpdate(bool *bval, char *cval)` overload
  ([src/RFgenerator.cpp:841-856](src/RFgenerator.cpp#L841-L856)) has no call
  sites.
- [ ] Globals `dmaDAC` ([src/RFgenerator.cpp:170](src/RFgenerator.cpp#L170))
  and `PWLch` ([src/RFgenerator.cpp:1050](src/RFgenerator.cpp#L1050)) are
  declared but never read or written elsewhere.
- [X] `Debug()` ([src/RFgenerator.cpp:899-1038](src/RFgenerator.cpp#L899-L1038))
  is ~140 lines of leftover scratch/bring-up code (large commented-out
  blocks, hardcoded magic numbers, direct register pokes). Either delete it
  or move actual bring-up tests behind a clearly-named, documented debug
  build flag.
- [X] Large commented-out block in `TUNE_SCAN_DOWN`
  ([src/RFgenerator.cpp:690-694](src/RFgenerator.cpp#L690-L694)).

## Robustness / error handling

- [ ] I2C transactions don't check status: `Wire.endTransmission()` return
  value is ignored in `FS7140setup()`
  ([src/FS7140.cpp:123](src/FS7140.cpp#L123)), so a bus error or missing
  device fails silently.
- [ ] `UserInput()` ([src/Serial.cpp:364-381](src/Serial.cpp#L364-L381)) has
  no timeout — calibration wizards (`calBias`, `calCurrent`,
  `genPWLcalTable`) block forever waiting for host input. If the host
  app hangs or disconnects mid-wizard, the device needs a manual reset to
  resume normal command processing.
- [ ] `genPWLcalTable()` instructs the user that "Voltage must be
  increasing" but never validates that entered `ADCvalue`/`Value` points are
  monotonically increasing before storing them
  ([src/RFgenerator.cpp:1058-1111](src/RFgenerator.cpp#L1058-L1111)).
  `PWLlookup()`'s interpolation assumes sorted input
  ([src/RFgenerator.cpp:1116-1132](src/RFgenerator.cpp#L1116-L1132)) and will
  produce nonsensical results if it isn't.
- [ ] `ExecuteCommand()`'s `CMDint`/`CMDfloat` cases rely on a `break`
  inside every `if` to avoid falling through into the next `case`
  ([src/Serial.cpp:450-475](src/Serial.cpp#L450-L475)) — correct today only
  because no `CMDint`/`CMDfloat` table entry has `NumArgs` outside `{0,1}`.
  Add a trailing `break`/`default` so this can't silently misbehave if a
  future command entry changes that.

## Maintainability

- [ ] `setDrive()` flips `percent = 100 - percent` and then computes
  `MAXDRIVE * (100 - percent) / 100`, which is algebraically just
  `MAXDRIVE * percent / 100` — the double negation cancels out but makes the
  function read backwards and is an easy trap for a future "fix."
  [src/Hardware.cpp:154-170](src/Hardware.cpp#L154-L170).
- [ ] Redundant DAC write in `setup()`: `analogWrite(rfgendata.DCbias.Chan,
  ...)` immediately followed by `writeBIAS` (which itself calls
  `myAnalogWriteFunction`, i.e. a second, different write path to the same
  DAC channel). [src/RFgenerator.cpp:560-562](src/RFgenerator.cpp#L560-L562).
- [ ] Hardware variant selection (`RFGEN2`, `SINGLEENDED`, `TRIGINVERTED`,
  `QUENCH`) is done via `#define` in
  [include/RFgenerator.h:10-17](include/RFgenerator.h#L10-L17) rather than
  per-environment `build_flags` in `platformio.ini`. Building firmware for a
  different board revision currently means editing and recommitting a
  header rather than selecting a PlatformIO environment.
- [ ] Heavy use of the Arduino `String` class throughout the serial command
  parser and calibration wizards (`src/Serial.cpp`, `src/RFgenerator.cpp`)
  risks heap fragmentation on a long-running embedded target; consider
  bounded C-string parsing for the hot command path.
- [ ] `FS7140setup()` declares its working variables `static` for no
  apparent reason ([src/FS7140.cpp:42-45](src/FS7140.cpp#L42-L45)) — they're
  fully reassigned before use each call, so `static` only adds confusion
  about lifetime/reentrancy.
- [ ] The PWM clock-setup comments in `setupD10pwm()`
  ([src/Hardware.cpp:61-67](src/Hardware.cpp#L61-L67)) describe a different
  divider/source ("48MHz...100MHz/2 = 50MHz") than what the code actually
  configures (`DIV(4)` off `DPLL0`), and the resulting PWM frequency with
  `PER = 600` is ~42 kHz, not the stated 50 kHz. Worth reconciling comment
  vs. actual configuration.
