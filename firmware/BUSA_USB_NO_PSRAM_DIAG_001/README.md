# ESP32-S3 USB diagnostic, PSRAM OFF

This is a standalone **no-output**, USB-only diagnostic for an ESP32-S3. It does not drive GPIO, sensors, actuators or relays. The previous diagnostic failed to boot with `PSRAM ID read error: 0x00000000`; this build intentionally disables PSRAM.

GitHub Actions builds with Arduino-ESP32 2.0.17, Flash 16 MB, PSRAM disabled and native USB CDC enabled. Once the workflow completes successfully, its artifact should contain `BUSA_USB_NO_PSRAM_DIAG_001_merged.bin` for a `0x000000` single-image upload using Espressif Flash Download Tool. Do not try to flash until the workflow reports success and the image is available.

This build has **no control functions**. It is not a complete device firmware and has not been tested on the user's hardware.
