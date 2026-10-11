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
- [ ] Remaining Phase 1 items: live resolver-signal check, `udc*` trip,
  16-bit `int` overflow audit with a host test, resolver hybrid decoder, MTPA
  constants into parameters, HVIL window and debounce.

## Done

- [x] Software over-current trip logic and ISR wiring, with host tests
  (`test/testovercurrent.cpp`).
