# TODO

Working list; the phased plan is in `docs/RDU-FOC-bringup-plan.md`.

## Open

- [ ] **Over-current trip: explicit re-enable after a trip (Phase 1).**
  The software trip (`include/overcurrenttrip.h`, wired into
  `motor_control_adc_isr` in `platform/c2000/inverter_drivers/pwmdriver.cpp`)
  latches until reboot, and nothing calls `OverCurrentTrip::Reset()`. Add a
  deliberate clear command, probably over CAN (an `oic`/parameter command
  such as a `cmd` verb or a write-only param). Open points:
  - Only allow the clear while `opmode` is OFF and the current is back under
    `ocurlim`.
  - Decide whether it also resets the base-class `tripped` flag and clears
    `ERR_OVERCURRENT`.
  - Write the host test first (clear is refused while the fault persists).
  - Decide whether HVIL should get the same clear path.
- [ ] Over-current trip: decide on debounce (a sample count) once real sensor
  noise is seen on the bench. Currently a single sample over the limit trips.
- [ ] Over-current trip: the ISR wiring and the ADC scaling have no host test;
  verify on the scope in Phase 7.
- [ ] **DC-link trip vs bench voltages.** Under-voltage trips below `udcmin`
  (default 60 V) while running, and over-voltage above `udclim` (default
  450 V). Before any run on the 12 to 20 V supply or the 53 V pack, set
  `udcmin` below the bus voltage and `udclim` just above the pack's maximum
  (about 58.8 V for 14S). Add both to the Phase 2/3 checklists. Also: the
  trip uses the `udc` value as scaled in the ISR, which ignores `udcofs`.
- [ ] DC-link trip: re-enable after a trip uses the same clear command as the
  over-current trip (`DcLinkTrip::Reset()` is not called anywhere yet).
- [ ] **Resolver calibration on the bench (Phase 3).** With the motor phases
  disconnected, set `rescal` to 1 over CAN and turn the shaft by hand for at
  least 2 s (several turns). `rescalstat` goes to 2 (Done) and `resofssin`,
  `resofscos`, `resgaincos` are written; 3 (Failed) means the swing was too
  small, a channel is weak or dead, or 30 s passed. Then `save`.
- [ ] Resolver, out of scope of the hybrid decoder: per-sample angle spike
  rejection (one glitchy ADC pair feeds FOC; decide after seeing real signals),
  a fixed-point `SineCore::Atan2` instead of `atan2f`, and hardware checks of
  the angle-to-PWM phase (Phase 3/5).
- [ ] Resolver review leftovers (minor): a re-lock leaves a partial frequency
  window (first 100 ms after re-lock reads low; reset `sm_samplesInWindow` on
  the lock transition); the 500 Hz spike limit is a resolver electrical
  frequency and may sit below top speed once `respolepairs` >= 2 (check in
  Phase 5, or derive it from `fmax`); `CorrectResolverSample` range-checks the
  gain but not the offset (a corrupt offset could overflow int32).
- [ ] Resolver signal monitor limits: radius lock/unlock levels (600/350
  counts) and the 16-sample debounce are estimates, assumed centred samples.
  Check against real amplitude and offset on the bench (Phase 3). A single dead
  channel is not detected by the radius check while the rotor is parked near
  the other channel's peak.
- [ ] 16-bit audit, second pass: the `IsrIntWidthTest` guard only catches
  declarations. Still to review by hand: 16-bit intermediates (`uint16_t *
  uint16_t`, `int16_t` arithmetic and integer literals all promote to a 16-bit
  `int` on the C28x), and `FP_FROMINT` on values above 1023 that are plain
  `int`. Keep the `ISR_PATH_FILES` list in `test/CMakeLists.txt` in step when
  files join the ISR path (e.g. when `c2000::Encoder` wraps `ResolverEncoder`).
- [ ] **MTPA formula (before using `lqminusld` > 0, Phase 8).** The fixed-point
  `FOC::Mtpa` gives about half the d-axis current of its own float reference
  in `test/testfoccalc.cpp`: the former constant 0.01722 is half of
  (Lq - Ld)^2 * 1024, and `SetMtpaParameters` keeps that halving on purpose.
  Neither matches TI SPRACF3 (id = (psi - sqrt(psi^2 + 8 (Lq-Ld)^2 is^2)) /
  (4 (Lq-Ld))). Decide on the correct formula with measured motor constants.
- [ ] **MTPA overflows above about 400 A**: `FP_DIV(fluxLinkage - term1,
  lqminusld)` shifts a Q15 value left by 15 and overflows int32 (is = 600
  gives id = -249 instead of about -409). Irrelevant while `lqminusld` is 0.
- [ ] `FocPwmGeneration::SetTorquePercent` reads `fluxlink`/`lqminusld` on each
  call; there is no host test at the generation level for that wiring.
- [ ] **`polePairRatio` is integer division** (`polepairs / respolepairs` in
  `Param::Change`, `inverter_main.cpp`): `polepairs` must be a whole multiple
  of `respolepairs`, or the motor electrical frequency and angle are silently
  wrong (3 and 2 gives ratio 1). Check when the resolver pole pairs are known
  (Phase 5); reject or flag a non-integer ratio.
- [ ] **Only `pwmfrq` = 1 (12.2 kHz) is valid on this board.** The resolver
  exciter (EPWM1) shares the phase-PWM period, and `ResolverPhaseDelay` (3230
  counts, tuned at 12.2 kHz) is a fixed counter start offset for the ADC and
  phase EPWMs. At 24.4 kHz (period 2048) the offset exceeds the period; at
  6.1 kHz the exciter frequency halves and the samples miss the sine peak.
  Before anyone changes `pwmfrq`: lock it to 1 on the C2000 (reject or force
  with a message), or decouple the exciter and scale the delay.
- [ ] **Resolver signal loss while running does not stop the output.**
  `ERR_LORESAMP` is `ERROR_DISPLAY`; the decoder holds the last angle and FOC
  keeps commutating on it, then steps the angle at re-lock. Upstream hardware
  history (5033f09): angle steps gave ~180 degree voltage jumps and 50 A+
  spikes. Trip (Trip-Zone, opmode OFF, latched) on loss of lock in MANUAL/RUN,
  like the other Phase 1 trips; host-test it via the generation templates.
- [ ] **Verify the current gain on the rear unit before trusting `ocurlim`.**
  Upstream FDU calibration (e1d9a8b): MLX91209 sensors, 1.64 dig/A (default
  here 1.625), about 0.61 A per count; a wrong gain there once let 44 A flow
  while the firmware saw 15 A (wire fire). The H-revision sensors are
  unverified. At 3 to 5 A the trip is only 5 to 8 counts above the offset, so
  expect noise; cross-check `idc` against the DC shunt (appendix D).
- [ ] `execTicks > 16000` overrun flag (`pwmdriver.cpp`) assumes the 12.2 kHz
  budget; derive it from the PWM period. The 4000-sample resolver startup delay
  is time-dependent too (0.16 to 0.66 s).
- [ ] From the code review (plan appendix A item 7), not yet scheduled: no
  DC-link normalisation of the PI gains (re-tune per bus voltage, Phase 6); no
  dead-time compensation; PI integral uses integer division by the PWM
  frequency; sine lookup has no interpolation; ADC sampling instant versus the
  PWM centre unverified (scope, Phase 3).
- [ ] All trips (HVIL, over-current, DC link) run inside the control ISR and
  fail under ISR starvation; the CMPSS comparator (Phase 7) covers only
  over-current. (The HVIL review's "Post every cycle" concern is moot:
  `ErrorMessage::Post` posts each error once.)
- [ ] Pre-existing, seen in review: `SetOpmode` sets `opmode` before
  `PwmInit`, so the ISR can run `Run()` mid-setup; the debug `printf` runs with
  interrupts disabled.
- [ ] Phase 5 reference: upstream ran the FDU with `polepairs` = 3 and
  `respolepairs` = 3 (a473b7c); the rear unit may differ.
- [ ] Remaining Phase 1 item: HVIL window and debounce.

## Done

- [x] Resolver hybrid decoder (plan
  `docs/superpowers/plans/2026-10-11-resolver-hybrid-decoder.md`):
  `c2000::Encoder` wraps `encoder::ResolverEncoder` on calibrated samples;
  angle in [0, 2pi), mechanical turn counting, self-timed 100 ms frequency
  window at any PWM frequency, 500 Hz spike rejection, no 200 Hz clamp, speed
  from `respolepairs`, per-channel offset/gain calibration (`rescal`).
- [x] MTPA constants into parameters: `fluxlink` (mWb, default 90, id 173) and
  `lqminusld` (uH, default 0, id 174) via `FOC::SetMtpaParameters`. Default 0
  runs with the d-axis current at zero for bring-up; 90/5800 reproduces the
  former results exactly (golden test).
- [x] 16-bit `int` audit of the ISR path: `syncadv` and `moddedfwkp` in
  `include/focpwmgeneration.h` overflowed on the C28x (for example 200 Hz with
  `syncadv` 6 gives 38400). All plain `int` in the ISR files replaced with
  fixed-width types; `IsrIntWidthTest` (`test/testisrintwidth.cpp`) fails on
  any new one.
- [x] Live resolver loss-of-signal check (`include/resolversignalmonitor.h`,
  `test/testresolversignal.cpp`) in `ResolverEncoder`: locks at any parked
  angle, unlocks when the signal disappears, holds the last angle. The angle is
  now held until 16 good samples (the old code trusted the first sample).
- [x] DC-link over- and under-voltage trip (`include/dclinktrip.h`,
  `test/testdclinktrip.cpp`), wired into the ISR; new `ERR_UNDERVOLTAGE`.
- [x] Software over-current trip logic and ISR wiring, with host tests
  (`test/testovercurrent.cpp`).
