# ESPFlight Firmware v1.0.0 — Documentation Errata

This note records a documentation correction for the published ESPFlight Firmware v1.0.0 release.

## Assisted Landing ramp

The `ASSISTED_FLIGHT.md` file contained in the original `v1.0.0` tag and release source archive states that, after the Landing command is acknowledged, the ESPFlight Application ramps visible throttle from 1500 to 1050 over **about 6 seconds**.

The validated ESPFlight Application v1.0.0 release uses the tested production value of **about 3 seconds** for this ramp.

The current `main` documentation has been corrected to 3 seconds.

## Scope

This is a documentation correction only.

- No ESPFlight Firmware v1.0.0 flight-control code was changed for this correction.
- The published `v1.0.0` tag remains unchanged for release reproducibility.
- ESPFlight Application v1.0.0 is the validated application baseline for the corrected 3-second Landing ramp.

For the current assisted-flight documentation, see [`ASSISTED_FLIGHT.md`](ASSISTED_FLIGHT.md).
