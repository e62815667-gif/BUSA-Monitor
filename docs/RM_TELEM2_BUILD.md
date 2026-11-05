# БУСА-1М №001 — RainMaker TELEM2

Версия `14v9-N16R8-RM-TELEM2-20261105`, продолжение TELEM1 на исходной базе
BLE FIX3 `46baec115f91e2637c2adf493e9449093700c9d1`.

Изменение по снимку экрана владельца: в RainMaker T1–T4 округляются
до десятых градуса перед публикацией числового параметра Temperature.
Примеры: 30.875 → 30.9; 35.75 → 35.8; -2.25 → -2.3.
Округление до ближайшего значения, половины от нуля; отрицательный ноль удаляется.
Измеренные значения датчиков, Google, локальное приложение, пороги и звуковая
логика сохраняют прежнюю точность. Снимок диагностики хранит исходные значения.
Названия, типы, права и схема уже зарегистрированных параметров не изменены.
Нет повторной BLE-регистрации, очистки NVS или изменения аппаратных функций.

**Оформление RainMaker:** прошивка отправляет число, не строку «35,7гр».
Запятая/точка, знак °C и сохранение завершающего нуля (35.0 вместо 35)
зависят от iPhone-приложения RainMaker. Не заявляется, что прошивка принудительно
изменяет его форматирование. Предотвращена отправка показаний с точностью
выше одного десятичного знака. На настоящем iPhone результат не проверялся.

Сохранены пакетная публикация каждые 10 секунд, cache настроек, work queue,
MQTT/PUBACK диагностика и обработка восстановления из TELEM1.
Подробный предыдущий аудит, ограничения, разметка и установка:
[RM_TELEM1_BUILD.md](docs/RM_TELEM1_BUILD.md) (в ZIP) / файл в docs репозитория.

Автоматические проверки:

```
python3 -B tests/test_rm_telemetry.py
python3 -B tests/test_rm_tracker.py
python3 -B tests/test_rm_preservation.py
python3 -B tests/test_ble_ota.py
python3 -B -m unittest discover -s tests -p test_firmware_contract.py -v
```

Добавлены проверки округления 30.875, 35.75, отрицательных значений,
отрицательного нуля, всех T1–T4 и неизменности исходных измерений.
Часовой сценарий, отказы API/транспорта, очереди, reconnect и invalid sensor
проверяются тем же кодом прошивки с моделью SDK. Это не испытание настоящего MQTT.

Сборка: `.github/workflows/busa-rm-telemetry.yml`, Arduino CLI 1.3.1,
ESP32 core 2.0.17, прежние версии библиотек и Google CA.

```
python3 ci/generate_google_ca.py --output firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_GoogleCA.h
arduino-cli compile --clean --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,CDCOnBoot=cdc,USBMode=hwcdc,FlashMode=qio,DebugLevel=none' --output-dir out firmware/BUSA_1M_WIFI_3v_GOOGLE
python3 ci/package_rm_telem2.py --build-dir out --core-dir /path/to/esp32/2.0.17 --esptool /path/to/esptool_py/4.5.1/esptool.py
```

OTA BIN — штатное обновление приложения на ESP32-S3 N16R8 с разметкой FIX3.
Скачайте в «Файлы» iPhone, подключитесь к BUSA-1M-001-R04 и выберите **OTA BIN**
в штатном локальном обновлении 192.168.4.1. Не сбрасывать настройки/регистрацию.
Разделы NVS/fctry не входят в OTA. USB MERGED записывается с 0x0 и затрагивает
обычный NVS/otadata, поэтому требует резервной копии и может удалить настройки.
USB не выбирать в OTA. Не использовать erase-all для сохранения RainMaker identity.

Физический ESP32 не прошивался. На устройстве остаётся проверить: T1–T4
(включая 30.875 → 30.9), обновление времени работы в RainMaker, восстановление
Интернета/RESET, сохранение регистрации и настроек после OTA, iPhone UI,
длительную устойчивость и взаимодействие с остальными модулями.
