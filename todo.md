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
- [ ] **Resolver: make `c2000::Encoder` use `ResolverEncoder`.** The live
  loss-of-signal check exists only in the host-testable
  `include/resolverencoder.h`; `platform/c2000/inverter_drivers/encoder.cpp`
  still has the latched `s_resolverPeakAmplitude`, so the hardware path is
  unchanged. This is the hybrid decoder (decision 2): port the 100 ms
  frequency calculation, spike rejection and calibration, then thin
  `c2000::Encoder` into a wrapper.
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
- [ ] Remaining Phase 1 items: resolver hybrid decoder, HVIL window and
  debounce.

## Done

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
