# Resolver Hybrid Decoder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the host-tested `encoder::ResolverEncoder` template the only resolver decoder on the C2000, with what `encoder.cpp` got right on hardware, the known defects fixed, and per-channel offset/gain calibration.

**Architecture:** All decode logic lives in header-only, host-testable units in `include/`: `ResolverEncoder` (angle, turns, frequency), `ResolverSignalMonitor` (already done: live loss-of-signal check), and a new `resolvercalibration.h` (calibration maths and the calibrator). `c2000::Encoder` becomes a thin wrapper that instantiates the template on a C2000 sample adapter which applies the calibration to the raw ADC values.

**Tech Stack:** C++ (host gnu++20 / TI cl2000 25.11.1.LTS, effectively C++03: no `static_assert`, no `constexpr`), GoogleTest + gmock, CMake presets `host-FOC`, `host-SINE`, `c2000-FOC`, `c2000-SINE`.

**Spec:** `docs/RDU-FOC-bringup-plan.md`, Phase 1 "Resolver (decision 2, hybrid)", Decision 2, Appendix A items 2, 4, 5. Open items are tracked in `todo.md`.

## Global Constraints

- Plain `int`, `short`, `long`, `unsigned` are forbidden in ISR-path files (`IsrIntWidthTest` enforces this); use fixed-width types. On the C28x `int` is 16 bits.
- Single-precision maths only on the ISR path: `atan2f`, `fabsf`, `float` literals with `f`. `atan2`/`double` on C28x EABI is 64-bit software emulation. The TMU maps `atan2f` to hardware.
- Code in `include/` and `libopeninv/` must compile on both the host and cl2000 (no `static_assert`, `constexpr`, `<cmath>` overloads, or `std::`). Volatile `++` is a host error (`-Werror=volatile`): write `x = x + 1`.
- Parameter IDs must be unique. Next free: params **175–178**, values **2070**.
- Load firmware to RAM only on hardware. No hardware steps in this plan.
- Test output goes to a log file; read the tail or grep it. Never dump full build or test logs.
- Commit per task, with the attribution lines from the session's system reminder.

## Decisions pinned

1. **Angle range:** `ResolverEncoder` stores the angle normalised to **[0, 2π)** right after `atan2f`. `GetRotorAngle()` must never convert a negative float to `uint16_t`.
2. **Full turns:** these are odometry of *mechanical* turns. Every wrap of the resolver angle (a step across 0/2π, detected in `UpdateTurns` as |diff| > π) counts one resolver electrical turn, **in either direction**. After `respolepairs` such turns, `sm_fullTurns` goes up by 1. This replaces the `sm_lastAngle <= Pi && sm_angle > Pi` test, which can never fire with an `atan2` range of (−π, π], and the off-by-one pole counter.
3. **Frequency window:** `ResolverEncoder::UpdateRotorAngle` counts samples and calls `UpdateRotorFrequency(10)` itself every `max(1, sm_pwmFrq / 10)` samples, which is 100 ms. `UpdateRotorFrequency` stays public. The hard-coded `1220` block in the C2000 ISR goes away: it was only right at 12.2 kHz, and `pwmfrq` can select 6.1 or 24.4 kHz.
4. **Frequency plausibility:** a window whose candidate resolver frequency is **≥ 500 Hz** is a spike. The previous frequency and direction are kept, ported from `encoder.cpp`. No 200 Hz clamp in `GetRotorFrequency()`: it hid real speed from angle advance and field weakening (Appendix A item 5).
5. **Stationary:** a window with |turns| ≤ `StableAngle` (10°), or one inside the startup delay, sets frequency 0 **and direction `Stationary`**, as `encoder.cpp` does. The template currently keeps the last direction.
6. **Speed:** `GetSpeed()` = resolver frequency × 60 / `respolepairs`, as rpm (mechanical). The encoder measures the *resolver* electrical frequency (mechanical rev/s × `respolepairs`), so this is the true shaft speed. `encoder.cpp` divided by the motor's `polepairs`, which is wrong whenever the two differ: with the defaults (2 and 1) it reports half the real rpm (Appendix A item 5). This is a fix. It only affects the `speed` value and what reads it (throttle `dirchrpm`, idle-speed control, CAN). Motor control uses `polePairRatio` and is not affected. Return type `int32_t` (the template now returns `uint32_t` and loses the sign; speed is magnitude, so it stays ≥ 0).
7. **Calibration parameters** (CAT_MOTOR, in `MOTOR_PARAMETERS_COMMON` after `sincosofs`):
   - `resofssin`, "dig", 0, 4095, default 2048, id **175**
   - `resofscos`, "dig", 0, 4095, default 2048, id **176**
   - `resgaincos`, "%", 50, 200, default 100, id **177**: multiplies the centred cosine so its amplitude matches the sine.
   - `rescal`, "", 0, 1, default 0, id **178**: set to 1 over CAN to start calibration. The firmware sets it back to 0 when done.
   - value `rescalstat` with enum `RESCALSTATES` "0=Idle, 1=Running, 2=Done, 3=Failed", id **2070**.
   - `sincosofs` stays (the STM32 build uses it). The C2000 build no longer reads it.
8. **Calibration procedure:** while `rescal == 1`, the C2000 ISR feeds raw ADC samples to `ResolverCalibrator`, and the operator hand-turns the shaft. The calibrator succeeds once both channels' spreads (max − min) are **≥ 1200 counts** and at least **2 s** of samples (`2 × pwmFrq`) have been seen. Offsets are (max + min) / 2. `resgaincos` = 100 × sinSpread / cosSpread. A result with `resgaincos` outside [50, 200] is **Failed**, as is reaching **30 s** (`30 × pwmFrq` samples) without success. On Done, the firmware writes the three parameters; on either outcome it sets `rescal` to 0 and sets `rescalstat`. Saving to EEPROM is left to the operator (`save`).
9. **Correction maths** (pure function, used by the C2000 sample adapter): `centred = raw − offset`. The cosine gets `centred × gain% / 100`, done in `int32_t` and then saturated to `int16_t`. A gain outside [50, 200] falls back to 100 (fail safe against a corrupt parameter).
10. **Diagnostics:** `GetLastAbsTurns`, `GetLastMaxSignedDiff` and `GetLastSampleCount` are removed. The `inverter_main.cpp` debug print shows raw and calibrated sin/cos, `fstat`, speed, and `rescalstat` instead.

## Review Focus

1. **PWM frequency changed between windows** (`SetPwmFrequency` while rotating): the next window must use the new length, and the frequency must stay right. Test `FrequencyCorrectAfterPwmFrequencyChange`, Task 2.
2. **Calibration with the shaft not turned, or turned only a little:** must end Failed at 30 s and never write parameters. Tests `CalibratorFailsWithoutRotation` and `CalibratorFailsOnPartialSwing`, Task 3.
3. **Calibration with one channel dead or weak** (gain outside 50–200 %): Failed, parameters untouched. Test `CalibratorFailsOnUnbalancedChannels`, Task 3.
4. **Corrupt or zero `resgaincos`** from EEPROM: the correction falls back to 100 %. Test `CorrectionFallsBackToUnityGainOutOfRange`, Task 3.
5. **Turning backwards across 0/2π:** full turns count in both directions, and direction reports −1. Test `FullTurnsCountedInBothDirections`, Task 1.

---

### Task 1: ResolverEncoder angle, turns and direction fixes

**Files:**
- Modify: `include/resolverencoder.h`
- Test: `test/testresolverencoder.cpp`

**Interfaces:**
- Consumes: `encoder::ResolverSignalMonitor` (existing: `Update(int16_t, int16_t)`, `Locked()`, `SampleUsable()`), `Param::respolepairs`.
- Produces (unchanged names): `ResolverEncoder<T>::UpdateRotorAngle(int32_t dir)`, `GetRotorAngle() -> uint16_t` (0..65535 for [0, 2π)), `GetFullTurns() -> uint32_t`, `GetRotorDirection() -> int32_t` (−1/0/1).

Design: decode with `atan2f((float)sin, (float)cos)`, then add 2π if negative (decision 1). `UpdateTurns` reports whether this step wrapped across 0/2π; on a wrap, a resolver-turn counter goes up, and when it reaches `respolepairs`, `sm_fullTurns++` and the counter resets (decision 2). Drop `sm_poleCounter`'s old logic. Use `fabsf`, and change `details::Pi`/`TwoPi`/`StableAngle` initialisers to `f` literals.

Test helper (in the test file): `RotateResolver(MockResolverSampleImpl&, float startDeg, float degPerSample, int samples, int16_t amplitude = 1800)` sets gmock `WillRepeatedly(Invoke(...))` on both channels, keeping a phase variable that advances **after each cosine read** (`DecodeAngle` reads sine first), then calls `UpdateRotorAngle(1)` `samples` times. Every test calls `Resolver::Reset()`, `Param::LoadDefaults()` and `Resolver::SetPwmFrequency(DefaultPwmFrequency)` first.

- [ ] **Step 1: Write the failing tests** in `test/testresolverencoder.cpp`:
  - `NegativeHalfAngleMapsToUpperRange`: parked at 250°, amplitude 1800, `StartupCount + 1` samples → `GetRotorAngle()` within ±2 of `250 * 65536 / 360`.
  - `FullTurnsCountedPerMechanicalTurn`: `respolepairs` = 1, rotate +5 turns at 3°/sample, starting at 10° → `GetFullTurns() == 5`.
  - `FullTurnsHonourResolverPolePairs`: `respolepairs` = 2, rotate +6 turns → 3.
  - `FullTurnsCountedInBothDirections`: `respolepairs` = 1, rotate −4 turns (−3°/sample) → 4.
  - `NoFullTurnWithoutCrossingZero`: oscillate ±60° around 90° for 2000 samples → 0.
- [ ] **Step 2: Run, confirm RED**
  Run: `cmake --build build/host-FOC > $LOG 2>&1; build/host-FOC/test/OpenInverterTest --gtest_filter='TestResolverEncoder.*' > $T 2>&1; grep -E "FAILED|PASSED" $T`
  Expected: the 5 new tests FAIL. The angle test reads a wrong or wrapped value; the turn counts are 0 or off by one. The existing 5 tests PASS.
- [ ] **Step 3: Implement** per the design above.
- [ ] **Step 4: Run, confirm GREEN** (same command). Expected: all `TestResolverEncoder.*` and `ResolverSignalMonitor.*` PASS.
- [ ] **Step 5: Full suites.** `ctest --test-dir build/host-FOC/test > $T 2>&1; tail -3 $T`, and the same for `build/host-SINE`. Expected: 100 % passed.
- [ ] **Step 6: Commit** `fix(resolver): normalise angle and count mechanical turns`.

### Task 2: ResolverEncoder frequency, direction and speed

**Files:**
- Modify: `include/resolverencoder.h`
- Test: `test/testresolverencoder.cpp`

**Interfaces:**
- Consumes: Task 1's normalised angle and `UpdateTurns` accumulation (`sm_turnsSinceLastSample`, signed radians).
- Produces: `UpdateRotorFrequency(int32_t callingFrequency)` (public, unchanged), `GetRotorFrequency() -> u32fp` (resolver electrical Hz, **no clamp**), `GetSpeed() -> int32_t` (mechanical rpm, divides by `respolepairs`), `GetRotorDirection() -> int32_t`. New private constant `details::MaxPlausibleFrequency = 500.0f` (Hz). New private member `sm_samplesInWindow` (`uint32_t`).

Design: decisions 3–6. The window length is computed on each sample from the current `sm_pwmFrq`, so a change takes effect at the next window. A spike window keeps the previous frequency and direction. A stationary window sets 0 and `Stationary`. The 4000-sample startup delay stays.

Rotation test helper as in Task 1 (`RotateResolver`). A resolver frequency `f` Hz at PWM frequency `p` Hz is `360 * f / p` degrees per sample. Tests do **not** call `UpdateRotorFrequency` themselves, except `SpikeWindow...`.

- [ ] **Step 1: Write the failing tests:**
  - `FrequencyMeasuredWithoutExternalCalls`: 20 Hz forwards for `StartupCount + 3 * 1220` samples at 12207 Hz → `FP_TOFLOAT(GetRotorFrequency())` within 20 ± 0.5, direction 1.
  - `BackwardsRotationReportsMinusOne`: −20 Hz → frequency 20 ± 0.5, direction −1.
  - `StoppingResetsDirectionToStationary`: 20 Hz forwards, then parked for 2 windows → frequency 0, direction 0.
  - `FrequencyAboveOldClampIsReported`: 300 Hz → 300 ± 5 (the old code clamped at 200).
  - `SpikeWindowKeepsPreviousFrequency`: 20 Hz settled, then one window at 600 Hz (17.7°/sample) → still 20 ± 0.5, direction 1.
  - `SpeedUsesResolverPolePairs`: `respolepairs` 2, `polepairs` 4, 20 Hz → `GetSpeed() == 600` (±6).
  - `FrequencyCorrectAt6kAnd24kPwm`: repeat the 20 Hz measurement with `SetPwmFrequency(6103)` and `(24414)` → 20 ± 0.5 each.
  - `FrequencyCorrectAfterPwmFrequencyChange`: settle at 12207, switch to 24414 mid-rotation (adjust degrees per sample), run 3 windows → 20 ± 0.5.
- [ ] **Step 2: Run, confirm RED** (Task 1 command). Expected: all 8 FAIL. Frequency is 0 because nothing calls `UpdateRotorFrequency`; the 300 Hz and spike cases fail once the window exists. Existing tests PASS.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run, confirm GREEN.** Expected: all resolver tests PASS.
- [ ] **Step 5: Full suites** (both host builds). Expected: 100 % passed.
- [ ] **Step 6: Commit** `feat(resolver): self-timed 100 ms frequency window, spike rejection, no clamp`.

### Task 3: Resolver calibration unit

**Files:**
- Create: `include/resolvercalibration.h`
- Create: `test/testresolvercalibration.cpp` (add it to the `OpenInverterTest` source list in `test/CMakeLists.txt`, and add the header to `ISR_PATH_FILES` there)

**Interfaces (all in `namespace encoder`):**
```cpp
int16_t CorrectResolverSample(uint16_t raw, int32_t offset, int32_t gainPercent);
class ResolverCalibrator {
public:
    enum State { IDLE = 0, RUNNING = 1, DONE = 2, FAILED = 3 }; // matches rescalstat
    void Start(uint32_t pwmFrequency);   // clears min/max, sets RUNNING
    void Update(uint16_t rawSin, uint16_t rawCos); // no-op unless RUNNING
    State GetState() const;
    int32_t SinOffset() const;  int32_t CosOffset() const;  int32_t CosGainPercent() const; // valid when DONE
};
```
Constants in the class: `MinSpread = 1200`, `MinSeconds = 2`, `TimeoutSeconds = 30`, `MinGainPercent = 50`, `MaxGainPercent = 200`. All arithmetic in `int32_t`/`uint32_t`. Decisions 8 and 9 hold the rules.

- [ ] **Step 1: Write the failing tests** (gtest, suite `ResolverCalibration`):
  - `CorrectionCentresOnOffset`: raw 2548, offset 2048, gain 100 → 500. Raw 1548 → −500.
  - `CorrectionAppliesGain`: raw 2448, offset 2048, gain 125 → 500.
  - `CorrectionSaturatesToInt16`: raw 4095, offset 0, gain 200 → 8190 (no wrap). Raw 0, offset 4095, gain 200 → −8190.
  - `CorrectionFallsBackToUnityGainOutOfRange`: gains 0, 49, 201, −100 with raw 2548, offset 2048 → 500.
  - `CalibratorIdleIgnoresSamples`: `Update` without `Start` → `IDLE`.
  - `CalibratorFindsOffsetsAndGain`: `Start(12207)`, feed 3 s of a turning signal with sin centre 2000 / amplitude 800 and cos centre 2100 / amplitude 640 → `DONE`, `SinOffset` 2000 ± 2, `CosOffset` 2100 ± 2, `CosGainPercent` 125 ± 1.
  - `CalibratorWaitsForMinimumTime`: the same signal for 1 s → `RUNNING`.
  - `CalibratorFailsWithoutRotation`: parked (constant samples) for 30 s + 1 sample → `FAILED`.
  - `CalibratorFailsOnPartialSwing`: sweep only ±20° around 0° for 30 s → `FAILED`.
  - `CalibratorFailsOnUnbalancedChannels`: cos amplitude 300 (spread 600 < 1200) but sin 800 → `FAILED` at timeout. Then cos amplitude 1700 with sin 800 (gain 47 %) → `FAILED` as soon as the spreads qualify.
  - `CalibratorRestartClearsPreviousResult`: after `DONE`, `Start` again → `RUNNING`, and old min/max don't leak in (a parked signal then reaches `FAILED`).
- [ ] **Step 2: Run, confirm RED.** `cmake --build build/host-FOC > $LOG 2>&1; grep -m1 "error:" $LOG`. Expected: `resolvercalibration.h: No such file or directory`.
- [ ] **Step 3: Implement** `include/resolvercalibration.h`.
- [ ] **Step 4: Run, confirm GREEN.** `build/host-FOC/test/OpenInverterTest --gtest_filter='ResolverCalibration.*'`. Expected: all PASS.
- [ ] **Step 5: Full suites** (both host builds, including `IsrIntWidthTest`). Expected: 100 % passed.
- [ ] **Step 6: Commit** `feat(resolver): per-channel offset/gain calibration unit`.

### Task 4: C2000 Encoder becomes a wrapper; parameters and ISR wiring

No new host tests: this is C2000-only glue. Its maths is covered by Tasks 1–3. Verify by building, the map check, and the full host suites.

**Interfaces:**
- Consumes: `encoder::ResolverEncoder<T>` (Tasks 1–2: `Reset`, `UpdateRotorAngle(int32_t)`, `UpdateRotorFrequency(int32_t)`, `SetPwmFrequency(uint32_t)`, `GetRotorAngle() -> uint16_t`, `GetRotorFrequency() -> u32fp`, `GetRotorDirection() -> int32_t`, `GetSpeed() -> int32_t`, `SeenNorthSignal() -> bool`), and `encoder::CorrectResolverSample` and `encoder::ResolverCalibrator` (Task 3).
- Produces: `c2000::Encoder` with exactly those static methods (callers in `pwmdriver.cpp`, `inverter_main.cpp` and the `PwmGeneration` templates stay source-compatible). Params `resofssin`, `resofscos`, `resgaincos`, `rescal`, value `rescalstat`.

**Files:**
- Modify: `include/param_prj.h` (decision 7: four params after `sincosofs`, value `rescalstat` after `hvilstate`, `#define RESCALSTATES "0=Idle, 1=Running, 2=Done, 3=Failed"` next to `HVILSTATES`, and the `VALUE_ENTRY` uses it)
- Create: `platform/c2000/inverter_drivers/c2000/resolversample.h`: `struct CalibratedResolverSample { static int16_t ResolverSine(); static int16_t ResolverCosine(); };` using `encoder::CorrectResolverSample(MotorAnalogCapture::ResolverSine(), Param::GetInt(Param::resofssin), 100)` and the same for cosine with `resofscos` and `resgaincos`.
- Modify: `platform/c2000/inverter_drivers/c2000/encoder.h`: `c2000::Encoder` keeps its public static API minus the three `GetLast*` diagnostics (decision 10). Each method forwards to `encoder::ResolverEncoder<CalibratedResolverSample>`; add `Reset()` forwarding.
- Delete: `platform/c2000/inverter_drivers/encoder.cpp`. The forwarding methods go inline in `encoder.h`. Remove `encoder.cpp` from `platform/c2000/inverter_drivers/CMakeLists.txt` and from `ISR_PATH_FILES`.
- Modify: `platform/c2000/inverter_drivers/pwmdriver.cpp` ISR: delete the `s_freqUpdateCounter`/`1220` block. Set `Param::speed` from `Encoder::GetSpeed()` every cycle. Add a file-static `encoder::ResolverCalibrator`: when `rescal` reads 1 and the calibrator is not RUNNING, call `Start(EPWM_FREQ / (2UL * pwmmax))`. Use the same frequency expression as the HVIL `Configure`, and store it in a file-static when the PWM is set up. While RUNNING, call `Update` with the raw ADC values. On DONE, write `resofssin`, `resofscos`, `resgaincos`. On DONE or FAILED, set `rescal` to 0. Always set `rescalstat` from `GetState()`.
- Modify: `platform/c2000/inverter/inverter_main.cpp`: the debug print per decision 10. Remove the comment about the ISR calling the frequency task every 1220 cycles.
- Modify: `test/CMakeLists.txt` `ISR_PATH_FILES`: add `platform/c2000/inverter_drivers/c2000/resolversample.h` and `include/resolvercalibration.h` if Task 3 didn't.
- Modify: `todo.md` (close the "make `c2000::Encoder` use `ResolverEncoder`" item; add the bench calibration step), and `docs/RDU-FOC-bringup-plan.md` Phase 3 (calibrate the resolver with `rescal` while hand-turning, then `save`).

- [ ] **Step 1: Implement** the above.
- [ ] **Step 2: Build both C2000 images.** `cmake --build build/c2000-FOC > $LOG 2>&1; echo $?`, and the same for `c2000-SINE`. Expected: 0, no new warnings (`grep -E "warning #" $LOG | grep -v 10063`).
- [ ] **Step 3: Map check.** `grep -ciE "atan2" build/c2000-FOC/OpenInverter.map`. Expected: `0` (the TMU handles `atan2f`; a non-zero count means double `atan2` slipped in).
- [ ] **Step 4: Full host suites** (both builds). Expected: 100 % passed, `IsrIntWidthTest` included.
- [ ] **Step 5: Commit** `feat(c2000): resolver decoding via ResolverEncoder with calibration`.

## Out of scope (recorded in `todo.md`)

- Per-sample angle spike rejection: a single glitchy ADC pair feeding FOC. Decide after seeing real signals in Phase 3.
- Replacing `atan2f` with a fixed-point `SineCore::Atan2`.
- Detecting a single dead channel while parked near the other channel's peak.
- Hardware verification of the lock thresholds (600/350), the calibration limits and the angle-to-PWM phase (Phase 3/5).
