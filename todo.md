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
- [ ] Remaining Phase 1 items: live resolver-signal check,
  16-bit `int` overflow audit with a host test, resolver hybrid decoder, MTPA
  constants into parameters, HVIL window and debounce.

## Done

- [x] DC-link over- and under-voltage trip (`include/dclinktrip.h`,
  `test/testdclinktrip.cpp`), wired into the ISR; new `ERR_UNDERVOLTAGE`.
- [x] Software over-current trip logic and ISR wiring, with host tests
  (`test/testovercurrent.cpp`).
