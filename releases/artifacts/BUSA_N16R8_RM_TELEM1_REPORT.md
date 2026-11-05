# TELEM1 — результаты сборки и проверки

Исходный коммит: c697553949e4315c1376758730075e985c7dd992.
База FIX3: 46baec115f91e2637c2adf493e9449093700c9d1.

Полный технический отчёт: [RM_TELEM1_BUILD.md](../../docs/RM_TELEM1_BUILD.md).

Локальная сборка ESP32-S3 N16R8 успешно завершена, APP файл 2017264 байта,
USB MERGED 2082800 байт; слот 3145728 байт. Arduino core 2.0.17, OPI PSRAM,
16MB flash, штатная разметка FIX3. ELF storage 2016901 байт, RAM 79220 байт.

Автоматические проверки: BLE/OTA helper regression PASS; PUBACK tracker PASS;
реальный telemetry C++ с SDK моделью PASS (час/360 пакетов, отказы/восстановление);
побайтовая сохранность утверждённых модулей и cloud schema PASS;
4 firmware contract tests PASS; embedded JavaScript syntax PASS;
APP image checksum и validation hash PASS, merged byte layout PASS.

Сборка GitHub Actions для того же исходного коммита:
https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42056243639
Публикуемые BIN собраны локально из этого коммита с тем же профилем; CI независимо
собирает/проверяет исходники (идентичность SHA с CI не заявляется).

Физические ESP32/iPhone, live MQTT/RainMaker, сохранение регистрации после
реального OTA/RESET и длительный тест на оборудовании НЕ проверены.
Устройство не прошивалось. Broker PUBACK означает приём брокером, а не
подтверждённое обновление приложения RainMaker.

Для обновления действующего устройства выбирайте OTA BIN в штатном локальном
OTA. Не выбирайте USB MERGED в OTA. USB записывает NVS/otadata; локальные
настройки могут быть потеряны. Без резервной копии USB не использовать.
Erase-all удаляет регистрацию и недопустим для её сохранения.

SHA-256:

```text
23309d60f0cf14664c474b07c78dcf38628bad3c2b87ceed8e90d3a062e958a3  BUSA_N16R8_RM_TELEM1_OTA.bin
321c509e7e72fcf4272dbc2491c8bf0c97de40948037c13e7d26cdf5329e02d7  BUSA_N16R8_RM_TELEM1_USB_MERGED.bin
bb8c09e7532ef0644700415ee139f8b4a4e8522026058a443a7d08027cae9163  BUSA_N16R8_RM_TELEM1_SOURCE.zip
```
