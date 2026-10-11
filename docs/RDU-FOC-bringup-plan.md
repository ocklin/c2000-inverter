# Rear drive unit FOC bring-up plan

Target: Tesla Model 3 rear drive unit **1120990-00-H** (PCBA 1078289), permanent-magnet motor, resolver, TMS320F28377D, running the `CONTROL=FOC` build of this repository.

Starting point: `main` at `78eb369`. It builds the FOC and SINE C2000 images with TI C2000-CGT 25.11.1.LTS and passes the host tests (SINE 35/35, FOC 44/44). Nothing has run on rear-unit hardware yet.

Ordering rule: protection is implemented and unit-tested in software before any motor test (Phase 4), and proven on the bench (Phase 7) before power goes up. Always load firmware to RAM only; writing flash erases the Tesla firmware.

Values marked *(estimate)* are conservative planning numbers, not measured or datasheet values.

## Phases

### Phase 0: Baseline (done)
Repo layout, toolchain, host tests, both C2000 images building, `main` pushed with a provenance note in the README.

### Phase 1: Harden the software (no hardware)
Safety:
- Implement over-current protection. `PwmDriver::SetOverCurrentLimits` is an empty stub and `ERR_OVERCURRENT` is never posted on the C2000. Either a comparator (CMPSS) into the Trip-Zone added by the HVIL change, or a software trip in the ISR.
- Add a live resolver-signal check using the current sin²+cos². The latched peak amplitude in `encoder.cpp` can never report loss of signal.
- Check how `udclim`, `udcmin` and `udcmax` are used. Only derating was found; add an over- and under-voltage trip if missing.

Correctness:
- Audit every plain `int` in the control ISR path for 16-bit overflow on the C28x. Known: `syncadv` and `moddedfwkp` in `include/focpwmgeneration.h`. Add a host test that catches these (the host `int` is 32-bit and hides them).
- Resolver: calibrate offset and gain of both channels; decide between the float `atan2f` decoder in `platform/c2000/inverter_drivers/encoder.cpp` and the unit-tested `ResolverEncoder` in `include/resolverencoder.h`.
- Move the MTPA constants in `libopeninv/src/foc.cpp` into parameters. Run with the d-axis current at zero for bring-up.
- HVIL: re-check the 10 to 30 mA window against the actual loop wiring; decide whether to debounce.

Done when: new logic has host tests and both images build and pass.

### Phase 2: Identify the unit and first power-up
Confirm 1120990-00-H against the documented -G units, then follow the power-up checklist (appendix C).
Done when: the board boots RAM firmware repeatably and a baseline is recorded.

### Phase 3: Bench bring-up, no motor
Check ADC channels (currents, DC link, temperatures), PWM waveforms at the MCU test points, and the resolver angle tracking a hand-turned shaft. Bus from the 12 to 20 V / 1 A supply.

### Phase 4: Motor open loop, low power
Forced rotation (`MANUAL` opmode) at tiny current. Verify phase order, direction and current-sensor signs. Supply limited to 1 A and phase current limited to 3 to 5 A in software *(estimate)*.

### Phase 5: Resolver alignment
Determine the pole-pair ratio, direction and zero offset (`respolepairs`, `syncofs`). Compare the decoded speed with the commanded rotation.

### Phase 6: Closed-loop current control
Enable `RUN` with a tiny current limit and tune the d/q PI gains. Supply: 53 V pack through a precharge resistor, fuse and disconnect. Re-tune at this voltage; the controller has no DC-link normalisation.

### Phase 7: Prove the protections
Inject each fault (HVIL open, resolver unplugged, over-current, over-voltage, over-temperature) and measure shutdown behaviour and latency. Stay at low power until every fault gives a safe, latched shutdown.

### Phase 8: Load and speed
Raise torque and speed in steps. Watch heating, dead-time effects and ISR timing (`execTicks`).

### Phase 9: Vehicle integration (optional)
Throttle and CAN control, the oil pump, a limp mode.

### Phase 10: Share
Offer fixes back to the upstream fork as PRs. Report H-revision findings to the openinverter forum thread.

## Open decisions
1. Over-current protection: comparator into the Trip-Zone (most robust, needs a hardware check) or a software check first.
2. Resolver decoder: keep the float version or return to the tested `ResolverEncoder`.
3. Upstream: open PRs to the upstream fork as fixes land, or keep work here until the rear unit spins.

## Appendix A: Code review findings (commit d52a194 plus the HVIL PR)

The Clarke/Park transforms, inverse Park, min/max (SVPWM-equivalent) harmonic injection, voltage-circle limiting and PI anti-windup are correct. Issues in the C2000-specific layers, most severe first:

1. Over-current protection is not implemented on the C2000 (`pwmdriver.cpp`, `SetOverCurrentLimits` stub).
2. Resolver loss of signal is never detected after the first good signal (`encoder.cpp`, latched `s_resolverPeakAmplitude`).
3. 16-bit `int` overflow risks in the ISR path (`focpwmgeneration.h`: `syncadv`, `moddedfwkp`).
4. Resolver decode uses fixed centre offset (`sincosofs` 2048), no gain/offset calibration, and float `atan2f` in a 12.2 kHz ISR.
5. Rotor frequency is estimated over 100 ms windows, clamped at 200 Hz; it feeds angle advance, integral-gain scheduling and field weakening. `GetSpeed` divides by the motor pole pairs, not the resolver's: verify.
6. MTPA uses hard-coded motor constants (`foc.cpp`: flux linkage 0.09, saliency term) that do not describe this motor.
7. Smaller: no DC-link normalisation of the PI gains; no dead-time compensation; default current gain 1.625 dig/A (about 0.6 A per count); sampling instant versus PWM centre unverified; PI integral uses integer division by the PWM frequency; sine lookup has no interpolation.

HVIL PR (by a third-party contributor, unmerged upstream, not hardware-validated): the latch, Trip-Zone shutdown and startup zeroing of commands are sound. Concerns: the trip runs inside the control ISR (fails under ISR starvation); the window assumes about 12 V / 560 ohm; no debounce; `ErrorMessage::Post` runs every cycle while tripped; scope mixes in the CONTROL switch and test-fixture edits. Its `CONTROL=SINE|FOC` CMake option is required: on the upstream branch the FOC image does not build.

## Appendix B: Supplies and limits *(estimates)*

Available: 12 V bench supply (about 20 A, no limiter), 12 to 20 V / 1 A current-limited supply, 53 V bicycle battery pack.

Phase current is not DC current: at low bus voltage the inverter draws little DC current while the windings carry several times more, so limit phase current in software as well (`manualid`/`manualiq`, `throtcur`). The firmware `ocurlim` is not enforced until Phase 1 is done.

| Step | Supply | Limits |
|---|---|---|
| 1. Logic only | 12 V bench | start on the 1 A supply; then fused 20 A supply |
| 2. HV bus, no motor | 12 to 20 V / 1 A | 1 A (supply limit) |
| 3. Motor, open loop | same | 1 A, phase 3 to 5 A in software |
| 4. Closed-loop current | 53 V pack via precharge | 10 A fuse, phase 10 A or less |
| 5. More torque | 53 V pack | 20 to 25 A fuse, phase 20 to 30 A; only after Phase 7 |

Wiring: no alligator clamps above about 5 A; at least 2.5 mm2 wire to 10 A; fuse in the positive lead close to the source; a switch or contactor within reach. The pack's BMS is not a current limiter: use a fuse, a precharge resistor (for example 47 to 100 ohm, 25 W or more, sized once the DC-link capacitance is known) and a disconnect. A 14S pack can reach about 58.8 V fully charged. Do not back-drive the shaft at speed: a permanent-magnet motor spun above the bus voltage charges the DC link through the inverter diodes and a bench supply cannot absorb it. The DC-link capacitance and the unit's maximum bus voltage have not been looked up yet.

The HVIL window of 10 to 30 mA fits a 12 V loop through about 560 ohm; the same loop at 20 V is about 36 mA and would trip.

## Appendix C: Power-up checklist (Phase 2)

Identify (no power):
- [ ] Photograph the unit label and logic board; record 1120990-00-H and PCBA 1078289.
- [ ] Compare the LV connector pinout with `M3_RearDU_OEM_Pinout.ods` in damienmaguire/Tesla-Model-3-Drive-Unit, and the gate-driver and PMIC chips with the notes in this repository.
- [ ] Confirm a permanent-magnet motor (cogging by hand, resistance with the three phase leads shorted).

Prepare (no power):
- [ ] HV connector and motor phase leads disconnected for the logic power-up.
- [ ] Fuse in the 12 V lead near the supply (3 to 5 A fast-blow to start), short leads, proper terminals, polarity checked with a meter, a switch within reach.
- [ ] JTAG per `docs/Tesla-M3-JTAG-cable.md` (LAUNCHXL-F28379D as XDS100v2). RAM only.
- [ ] Assume the stock Tesla firmware boots when powered (unverified); keep HV off and the motor disconnected.

First power-up:
- [ ] 12.0 V / 1 A supply; record idle current and board temperature after a minute. Stop if the limit hits at once, anything heats, or anything smells.
- [ ] If the board hit 1 A and sagged: use the 20 A supply behind the fuse, sized at about twice the idle draw. An optional series resistor R = (12 V - minimum board voltage) / highest expected current doubles as an ammeter; check for brown-outs.

HVIL loop: 12 V, about 560 ohm, HV connector HVIL IN to OUT, LV connector pin 23 (IN) to pin 4 (OUT), GND. Target 15 to 24 mA (13 to 20 mA reported upstream); the HV discharge resistors switch off when the loop closes.

Firmware smoke test (RAM load):
- [ ] LED heartbeat, main loop running.
- [ ] PMIC reaches NORMAL (DEVSTAT 0x02) and the PWM buffer enables.
- [ ] Gate-driver init OK.
- [ ] `oic` over CAN: `dumpall` works, `cmd save` / `load` persist through the SPI EEPROM.
- [ ] ADC readings plausible with no HV and no motor: current offsets near mid-scale, DC link near 0 V, six temperatures sensible.
- [ ] Hand-turn the shaft: resolver sin/cos swing and the decoded angle tracks it.
- Stop if the PMIC watchdog resets, the supply current jumps, or readings stick at rail values.

Record a baseline: idle current at 12 V, HVIL current, current offsets, temperatures, resolver amplitude.

Done when: the unit boots RAM firmware repeatably, nothing overheats, and the baseline is recorded.
