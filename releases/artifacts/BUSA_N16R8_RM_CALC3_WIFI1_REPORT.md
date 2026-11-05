# CALC3 WIFI1 — исправление поиска и восстановления Wi-Fi

Исходный коммит 6f88a7487befe9c2aaed5af4413fe9a7024cfb4a. Версия 14v9-N16R8-RM-CALC3-WIFI1-20261105.
Ветка feat/busa-calculator-20261105; main и опубликованная CALC2 сохранены.

Подтверждено в исходниках: незаполненный home_chan_dwell_time в scanNetworks
официального core2.0.17; принудительное отключение подключённого STA при пустом
повторе; отсутствие supervisor после потери связи; устаревшие scan диагностические
счётчики; отсутствие ручного SSID/повторного поиска в веб-интерфейсе.

Исправлено: официальный асинхронный esp_wifi_scan_start с полностью заданным
config; connected STA не отключается при поиске; bounded reconnect использует
только текущий сохранённый target и уважает staOff; обработка scan_done/status,
timeout/stop/OTA, реальные коды ошибок. Добавлены ручной SSID, «Повторить поиск»,
UTF-8 byte validation. SDK не патчился. Калькуляторы native RainMaker, TFT и local
из CALC2 сохранены; GPIO/звук/DS18B20/PCF8575/Google/BLE/разделы не изменялись.

Подробный отчёт и инструкция:
[RM_CALC3_WIFI1_BUILD.md](../../docs/RM_CALC3_WIFI1_BUILD.md).

**GitHub Actions SUCCESS:**
https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42084146541
Все шаги compile/package/upload PASS. Production scanner/supervisor с радио/event
mock, кириллица/дубликаты, active/passive, STA preservation, native errors,
timeout/late event/OTA, off/current-target/reconnect/rollover PASS.
Chromium/WebKit Wi-Fi HTTP fixture, manual POST после scan error, retry и UTF-8
PASS. Прежние калькуляторы/306 C++-JS scenarios/keypad/native RainMaker/телеметрия
/PUBACK/BLE/OTA/preservation/4 contract tests PASS.

Локальная сборка N16R8/core2.0.17: storage 2047805 байт, RAM 81844 байта.
APP BIN 2048176 байт ≤ OTA slot 3145728; USB MERGED 2113712 байт.
Flash16MB, PSRAM OPI, прежняя CSV-разметка. Image checksum/validation hash/chip ID
/APP boundaries/merged layout PASS. Локальные BIN и независимая CI-сборка используют
один исходный коммит; совпадение SHA локальных и CI BIN не заявляется.

**Настоящая ESP32 не прошивалась; реальный RF/iPhone/RainMaker не проверен.**
Найденные ошибки исходников доказаны; конкретная причина пустого списка на фото
без device-диагностики не установлена. Тестовая HTTP/radio модель не выдаётся
за физическое испытание. Требуются проверка точки доступа владельца, AP+STA при
поиске/переключении iPhone, reconnect/Google/RainMaker, OTA/RESET/identity/NVS.

Для текущего устройства использовать OTA BIN через 192.168.4.1. После OTA:
Настройки → Wi-Fi → Повторить поиск, выбрать сеть или ввести SSID вручную,
пароль точки доступа → Сохранить и подключить. Сохранённый staOff не снимается
одним поиском, его снимает именно Save. Не удалять БУСА из RainMaker, не повторять BLE.
USB с 0x0 затрагивает обычный NVS/настройки — сначала резервная копия;
не выбирать USB в OTA, не erase-all для сохранения identity.

SHA-256:

```text
0d651195e93336093e0c6bae604919f6df99fa67f66974df0ef600086a79a47a  BUSA_N16R8_RM_CALC3_WIFI1_OTA.bin
65f6bb4a57f422baf8dd88fa45bb9767342f27bffff94311f657172483985d85  BUSA_N16R8_RM_CALC3_WIFI1_USB_MERGED.bin
5c3329a9847b91c99b25588613fe9271ae6811e1667b503a29b6eded18682d48  BUSA_N16R8_RM_CALC3_WIFI1_SOURCE.zip
```
