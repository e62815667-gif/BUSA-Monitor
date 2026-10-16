# BUSA-1M-001 direct DS18B20 + iPhone hotspot commissioning

Status: preparation branch only. Do not flash until GPIO and the iPhone hotspot SSID are confirmed and the build passes.

## Hardware target
- Controller: ESP32-S3 N16R8.
- Four DS18B20 directly on one 1-Wire bus (three-wire, external power).
- Existing I2C bus remains SDA=GPIO8, SCL=GPIO7 for PCF8575/PCA9685.
- The DS2482-800 is not purchased and must not be required by the direct-bus variant.
- Physical relay outputs remain disabled.

## Blocking values that must not be guessed
1. DIRECT_ONEWIRE_GPIO: choose an actually free exposed GPIO after checking the existing wiring. Do not use GPIO7/8 (I2C) or pins already used by TFT/other peripherals. Current photos are insufficient to prove a free pin.
2. iPhone hotspot SSID: use the exact name displayed on iPhone 16 Settings > Personal Hotspot. Do not assume it is simply "iPhone".
3. The password was supplied by the owner in chat; do not commit it into source code, logs, build artifacts, or this repository. Enter it into device NVS through the existing serial command at commissioning time.

## Direct DS18B20 requirements
- One common 1-Wire data line with a 4.7 kOhm pull-up to 3.3 V; each sensor gets 3.3 V and GND (not parasite power).
- The four sensor ROM IDs must be read and recorded. Do not assign T1-T4 by bus enumeration order because ROM enumeration order is not a guaranteed physical location mapping. Bind each ROM ID to T1=Water, T2=Column, T3=Cube, T4=Column top after identifying each sensor physically.
- Validate ROM CRC, scratchpad CRC, supported family 0x28, stale-value rejection and disconnected/short-circuit faults.
- Keep all actuator outputs hard-disabled in firmware.

## Existing Wi-Fi commissioning command
The current LAB firmware creates a temporary AP BUSA-1M-001 and prints the settings token and AP URL on UART0 at 115200. The existing serial command handler accepts one JSON line followed by newline:
{"cmd":"wifi","ssid":"<EXACT_IPHONE_HOTSPOT_SSID>","password":"<OWNER_PASSWORD>"}
Send one JSON line followed by newline. The command stores credentials in NVS and starts station connection. Never replace the placeholders with guesses and never save the real password to GitHub.
The iPhone and ESP32 must be on the same hotspot network to reach the ESP32 station IP. If iOS hotspot compatibility requires it, enable Settings > Personal Hotspot > Maximize Compatibility. Keep the BUSA AP as fallback during initial setup.

## Validation gate before flashing
- CI compile + all host/browser regression tests pass on this branch.
- Confirm actual GPIO from the physical assembly and set it in the direct-bus firmware.
- Confirm exact hotspot SSID.
- Inspect merged binary build commit and SHA-256.
- First boot: no relays or loads connected; check UART logs, AP/STA IP, and four real sensor readings.
- This document is not proof that direct-bus firmware has been implemented or hardware-tested.
