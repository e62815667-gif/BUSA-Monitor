# БУСА-1М №001 — CALC3 WIFI1: ремонт поиска Wi-Fi

Версия `14v9-N16R8-RM-CALC3-WIFI1-20261105`, продолжение CALC2.
Ветка feat/busa-calculator-20261105. Main и готовые CALC2/TELEM2 сохранены.
Физический ESP32 не прошивался.

## Подтверждённые программные дефекты

1. В официальном используемом Arduino ESP32 **2.0.17**, WiFiScan.cpp,
   scanNetworks создаёт `wifi_scan_config_t config;` без обнуления и не задаёт
   `home_chan_dwell_time`. В SDK ESP32-S3 это реальное поле структуры.
   Его значение зависело от содержимого стека. Сам scanNetworks возвращал -2,
   теряя исходный ESP-IDF код ошибки. Это дефект исходников, не доказательство,
   что именно он вызвал конкретный пустой список на фото владельца.
2. В прошивке CALC2 первая пустая попытка приводила к `WiFi.disconnect(false,false)`
   даже при работающем внешнем STA. AutoReconnect отключён, а отдельного
   supervisor в loop не было: поиск мог лишить БУСА Интернета до нового Save/RESET.
3. Boot запускал одну попытку WiFi.begin, после временной пропажи точки доступа
   не было управляемого повторного подключения.
4. Общий scanner не обновлял wifiUiLastScanCode/Attempts, из-за чего старые поля
   диагностического JSON/Google могли оставаться -99/0 после реального поиска.
5. В интерфейсе отсутствовали ручной SSID и отдельная кнопка повторного поиска;
   точный код драйвера не показывался.

Ссылка на исходник Espressif:
https://github.com/espressif/arduino-esp32/blob/2.0.17/libraries/WiFi/src/WiFiScan.cpp
Порядок стандартного сборщика событий:
https://github.com/espressif/arduino-esp32/blob/2.0.17/libraries/WiFi/src/WiFiGeneric.cpp
SDK/Arduino core не патчились; никаких скрытых замен во время сборки.

## Исправления

**BUSA_WifiScan.h** — единственный общий асинхронный scanner для TFT и браузеров.
Запуск через официальный `esp_wifi_scan_start` с `wifi_scan_config_t config={}`:
все каналы разрешённой настройки страны, show_hidden, active 120…300 мс,
второй passive 450 мс/канал, home dwell 60 мс. Никаких изменений RF-страны,
распиновки, TX-мощности, AP SSID/пароля или ручных калибровок.

Стандартный Arduino обработчик сначала собирает записи `_scanDone`, затем
вызывает пользовательский onEvent. Наш observer после этого ставит atomic-флаг;
main читает SSID/RSSI штатными scanComplete/SSID/RSSI. Нет конкурирующего чтения
AP-списка из ESP-IDF и нет вмешательства в BLE provisioning.
Нативные status/count отличают ошибку драйвера и потерю записей от нуля сетей.

Поиск не отключает уже подключённый STA даже на пустом результате/повторе.
Отменяется только незавершённая ассоциация. Не более двух конечных попыток, ожидание
SCAN_DONE до 10 секунд, при timeout/OTA вызывается scan_stop и до 1 секунды
ожидается завершение cleanup; приложение не остаётся в бесконечном «Поиск».
AP программно не останавливается/не пересоздаётся. Один радиомодуль всё ещё
меняет каналы при AP+STA: непрерывность реальной RF-связи требует hardware-теста.

**Reconnect supervisor** в wifiUiPoll: после потери связи/сканирования пытается
подключиться только к **текущему** сохранённому stationSsid с backoff 5→10→20→30 с
и прежним ограничением ассоциации ~18 с. Не запускается при scan/OTA/сохранённом
staOff/отсутствии credentials. Выбор новой сети немедленно сбрасывает backoff;
предыдущая сеть не берётся из старого снимка/отдельного fallback. Явное
«Отключить Wi-Fi» остаётся приоритетным, поиск сам не снимает staOff.

Браузер: **«Повторить поиск»**, **«Ввести SSID вручную»**, точные причины ошибок
и raw IDF код. Ручное подключение доступно даже после ошибки сканирования;
проверка SSID/пароля идёт по длине UTF-8 байт, как в ESP32.
Успешное «Сохранить и подключить» штатно сохраняет credentials и снимает staOff.
Пароли не показываются в диагностике и не записываются в тестовые отчёты.

## Что сохранено

Все три native-карточки калькулятора RainMaker из CALC2, локальный калькулятор
и TFT, температуры до десятых/пакет 10 секунд, Google, звук, четыре DS18B20,
PCF8575, GPIO, расходомер, BLE-регистрация, NVS и OTA-разметка сохранены.
Проверка сохранности допускает только описанные изменения Wi-Fi helpers;
все неизменяемые функции keypad/editor/manual-off/NVS commit сравниваются
по хешам, Save отличается только сбросом retry-таймера.

## Диагностика

`GET /api/wifi/scan?start=1&fresh=1` — один новый запрос, объединяет уже
идущий поиск. `GET /api/wifi/scan` — чтение состояния. `scanning` включает cleanup.
В diagnostic: start_api=esp_wifi_scan_start, start_result (сырой esp_err_t,
0=успех), scan_result, event_status, event_number, wifi_mode, station_status.
Число 0 в start_result в этой версии означает успешный IDF start, а не ноль сетей.
Error: no_networks, scan_start_failed, scan_driver_failed,
scan_results_unavailable, scan_timeout, only_hidden_networks, scan_cancelled_ota.

На /api/status обновляются реальные scan_result_code/scan_attempts/count,
добавлены scan_driver_rc/scan_event_status. Эти диагностические данные также
попадают в прежний статус; формат остальных показаний не менялся.

## Проверки и сборка

```
python3 -B tests/test_wifi_scan.py
python3 -B tests/test_wifi_browser.py
BUSA_TEST_BROWSER=webkit python3 -B tests/test_wifi_browser.py
python3 -B tests/test_rainmaker_calculator.py
python3 -B tests/test_rm_preservation.py
python3 -B tests/test_ble_ota.py
python3 -B tests/test_rm_telemetry.py
python3 -B tests/test_rm_tracker.py
python3 -B tests/test_calculator.py
python3 -B tests/test_calculator_keypad.py
python3 -B -m unittest discover -s tests -p test_firmware_contract.py -v
```

Тесты исполняют производственные scanner/supervisor функции с радио/event
моделью. Проверяются полностью заданная IDF config, active/passive, кириллица,
дубликаты, сохранение подключённого STA, ошибки старта/завершения/потери записей,
timeout, позднее событие вне активного поиска, OTA, explicit off, новый target,
reconnect и millis rollover. Browser HTTP fixture проверяет ошибки, ручной
SSID/POST, повторный поиск и UTF-8; это **не настоящий ESP32/iPhone/RF-тест**.

CLI 1.3.1, ESP32 Arduino 2.0.17, прежние библиотеки/CA/разделы N16R8.
Полный CI: .github/workflows/busa-calculator.yml.

```
python3 ci/generate_calculator.py
python3 ci/generate_google_ca.py --output firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_GoogleCA.h
arduino-cli compile --clean --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,CDCOnBoot=cdc,USBMode=hwcdc,FlashMode=qio,DebugLevel=none' --output-dir out firmware/BUSA_1M_WIFI_3v_GOOGLE
python3 ci/package_wifi1.py --build-dir out --core-dir /path/to/esp32/2.0.17 --esptool /path/to/esptool_py/4.5.1/esptool.py
```

## Обновление и оставшиеся проверки

Для действующего зарегистрированного устройства использовать **OTA BIN**
через 192.168.4.1. После обновления: Настройки → Wi-Fi → «Повторить поиск»,
выбрать нужную сеть либо ввести точный SSID вручную, пароль точки доступа,
«Сохранить и подключить». При сохранённом staOff этот Save необходим;
сканирование не меняет настройку отключения само. Не удалять БУСА из RainMaker,
не повторять BLE. Обновление OTA не включает NVS/fctry.
USB MERGED с 0x0 затрагивает обычный NVS/настройки: сначала резервная копия,
не выбирать его в OTA и не выполнять erase-all для сохранения identity.

Конкретная причина пустого списка на фото владельца без device-диагностики
не установлена. Исправлены подтверждённые программные дефекты.
Физически не проверены: обнаружение точки доступа владельца, реальная AP+STA
связь при scan/смене iPhone, восстановление Интернета/RainMaker/Google после
потери сети, сохранение настроек/identity после OTA/RESET и длительная работа.
ESP32 этой работой не прошивалась; физические выходы не включались.
