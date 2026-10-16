# BUSA v0.7.2-lab.1 monitoring firmware
ESP32-S3 N16R8; ESP-IDF 5.3.2 with Arduino Core 3.1.3.
CMake builds main/LabMain.cpp. Historical main/main.cpp is not compiled.
Real DS2482-800/DS18B20 monitoring is enabled by a USB command only after wiring checks.
Wi-Fi AP/STA, local web UI, telemetry and persisted threshold settings are implemented.
Physical relay outputs, TFT, keypad and PCA9685 writes remain disabled.
Full agreed automation and a physical liquid-level sensor are not implemented.

See [Russian commissioning guide](../../docs/LAB-v0.7.2-lab.1-RU.md).
Authoritative workflow: busa-v072-lab-build.yml. Successful compilation does not verify hardware.
Flash the merged image at 0x000000 only after reviewing its build commit and SHA-256.
