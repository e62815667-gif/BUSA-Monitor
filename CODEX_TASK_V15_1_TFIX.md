# Codex: audit and fix BUSA-1M #001 V15.1-TFIX

## Starting point
This branch is a frozen-source handoff copy of commit `62b6289d4a6b269b7f7cc1f3bdf22a485a625ab3`, which built successfully in GitHub Actions run [40484024247](https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/40484024247). Do **not** work from the repository default branch or from a stale V13/V14 build.

Arduino entry point: `firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_1M_WIFI_3v_GOOGLE.ino`. All adjacent `BUSA_*.h` files are required; the short sketch alone is not a complete firmware.

Board in use: ESP32-S3 N8R2, 8 MB flash, ESP32 Arduino core 2.0.17. Future N16R8 is a *different* target and must not be assumed interchangeable.

## Real-world incident / why this task exists
- Previous ESP32-S3 board reportedly stopped working after a DS18B20 was wired incorrectly. **Do not claim a firmware defect caused the hardware failure.**
- Last hardware diagnostic before that incident: V15.1-TFIX booted; T1 GPIO16 and T4 GPIO21 were verified; T2 GPIO17 and T3 GPIO18 both reported `LINE_LOW` with idle=0, after_reset=0 and no ROM/presence. The actual wiring has since been found incorrect. The failure cause, possible damage to DS18B20 probes, and any damage to external modules are not known.
- New board is not yet installed. No hardware tests or flash operations are authorized.
- User wants a corrected, robust source tree to review before the new board is connected.

## Current pin map — preserve unless explicitly approved
- T1/T2/T3/T4 DS18B20 one-wire DATA: GPIO16 / 17 / 18 / 21; separate 4.7-kOhm pull-ups from DATA to 3.3 V (where not already present).
- I2C SDA=GPIO8, SCL=GPIO9. PCF8575 0x20 (keyboard); PCA9685 0x40 (ACK seen; not enabled); 0x70 may be the PCA9685 all-call address.
- SPI TFT: CS=GPIO10, MOSI=GPIO11, SCK=GPIO12, DC=GPIO13, RESET=GPIO14. Physical display model GMT028-05 V1.1; exact controller still requires confirmation.
- GPIO15 is a pulse-count input; signal MUST stay in 0–3.3 V range.
- K1–K4: GPIO7/6/5/4. **Treat all four as potentially connected to real relay-driver circuitry; boot must be fail-safe.**
- Local AP and STA are both needed. Do not expose SSID passwords, URLs with credentials, Google Script secrets or tokens in new commits, logs, or issue text.

## Requested review
1. **Critical: safe GPIO startup**. Inspect `BUSA_V15_Board.h`, `BUSA_V15_Alarm.h`, startup order in `BUSA_V15_Runtime.h`, reset and OTA paths. Identify discrepancies between comments/configuration and actual behavior. Note current board header says KPIN “never configured as outputs” but the alarm driver **does** call `pinMode(KPIN[i],OUTPUT)`. Resolve this contradiction with an explicit, testable *safe-by-default* hardware policy and a clear report. On new/unknown wiring, keep physical outputs inactive until their polarity, current, VCC/JD-VCC logic, and 3.3 V compatibility have been independently validated. Do not enable actuation by default.
2. **DS18B20 diagnosis**. Preserve strict CRC, fresh-sample and ROM uniqueness per logical role; do not invent readings. Review `LINE_LOW`, `NO_PRESENCE`, `MULTIPLE_ROM`, GPIO idle sampling and manual binding. Make display and /expert diagnostic distinguish electrically LOW, missing probe, ambiguous ROM, stale conversion, and an intentionally unassigned probe. Add unit tests for all. Do **not** mask shorted/floating DATA lines with a software fallback.
3. **Boot and resilience**. Review AP+STA connection/recovery, HTTP/Google delivery and local iPhone app accessibility. Keep the current working configuration if already tested; regressions must be guarded.
4. **TFT and keyboard**. Check initialization and pin mapping against source. PCF8575 keyboard currently reports `keyscan_error=6`, `conflict_mask_hex=c0`, an ambiguous key matrix. Diagnose and test; do not assume a particular unmeasured wiring order.
5. **Temperature UI**. Each channel must have fields labeled **«ВКЛ, °C»** and **«ОТКЛ, °C»**, without adding words “Звук” or “Сигнализация” in these field labels. Four channels remain logically independent.
6. **Build hygiene**. Retain original release and stable snapshots: do not modify V13, V14.9 or the existing source handoff branch. Create a new `codex/` or `fix/` branch for changes. Return a line-by-line changelog and commit SHA.
7. **Security and documentation**. Produce an updated connection matrix with GPIO, actual voltage rail, common GND, active output state, and uncertain electrical assumptions. Explicitly warn that DS18B20 should be powered from a verified 3.3 V supply for this wiring, while GPIO DATA must never receive 5/12 V. Do not guess sensor wire colors.

## Deliverables
- Code diff + changed .ino/.h files **inside the repository**, complete and compilable.
- All automated tests, including `python3 tests/test_temperature.py`, `python3 tests/test_wifi_recovery.py`, `node tests/test_temperature_ui.js`, and GitHub Actions firmware compile for the exact N8R2 target.
- Build artifacts: complete 8 MB flash `USB_merged.bin` (offset 0x000000), app-only `OTA.bin`, and SHA-256 checksums, using a **new version label**. Do not present binaries as tested on the physical board.
- A succinct report separating **source-proven issues**, **software changes**, and **items requiring measurement on the replacement board**.
- Keep outputs disabled until a separate physical wiring review approves the hardware. Never flash or command physical hardware automatically.

## Hardware checks, for documentation only
Before any new power-up, the owner must verify that no 5 V or 12 V is connected to 3.3 V GPIO, identify DS18B20 VDD/GND/DATA by actual marking or continuity rather than wire color, verify DATA pull-ups go to 3.3 V, and ensure no unexpected voltage on GPIO17/GPIO18. A burnt board can indicate a wiring fault; reusing the same wiring on a new board without checking may destroy it too.

## References
- Repository: https://github.com/e62815667-gif/BUSA-Monitor
- Base source: https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/temperature-rom-observation-20261027/firmware/BUSA_1M_WIFI_3v_GOOGLE
- Successful base compile: https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/40484024247
