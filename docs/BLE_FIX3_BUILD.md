# Сборка БУСА N16R8 BLE FIX3

Исходники FIX3 уже содержат все изменения. Не запускайте старые генераторы,
не накладывайте патчи FIX2 и не подменяйте VERSION перед компиляцией.
Три старых entry point в `ci/` теперь только проверяют исходники, без записи.

Цель — **ESP32-S3 N16R8**, не N8R2. Платформа Arduino ESP32 **2.0.17**.
Используйте полный каталог `firmware/BUSA_1M_WIFI_3v_GOOGLE`, включая все соседние
заголовки и **имеющуюся partitions.csv**. Один файл .ino не является полной прошивкой.

Зависимости workflow закреплены: OneWire 2.3.8, DallasTemperature 4.0.6,
Adafruit BusIO 1.17.4, GFX 1.13.0, ST7735/ST7789 1.12.0, ILI9341 1.7.0,
U8g2_for_Adafruit_GFX 1.8.0. RainMaker/WiFiProv входят в платформу 2.0.17.
Патчи framework от других веток не применяются.

Из корня распакованного source ZIP или checkout:

```sh
python3 -B ci/validate_ble_ota.py
python3 -B tests/test_ble_ota.py
python3 ci/generate_google_ca.py --output firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_GoogleCA.h
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,CDCOnBoot=cdc,USBMode=hwcdc,FlashMode=qio,DebugLevel=none' --output-dir out firmware/BUSA_1M_WIFI_3v_GOOGLE
```

Старый вложенный архив `.ino.zip` не компилируется и не включается в FIX3 source ZIP.
В source ZIP добавлены регрессионные тесты.
Готовый source ZIP содержит сгенерированный проверенный `BUSA_GoogleCA.h`.
Для повторной генерации используются GTS Root R1–R4 с обязательной сверкой SHA-256.
TLS-проверка не отключается. Подробная установка библиотек и упаковка приведены
в `.github/workflows/busa-ble-fix3.yml`.

## Два BIN

- **OTA.bin** — только APP для существующего локального OTA через Wi-Fi.
- **USB_MERGED.bin** — объединённые bootloader/partitions/boot_app0/APP от 0x0.
  Он рассчитан на Flash 16 МБ, но **не является дампом всех 16 МБ**. Область
  RainMaker `fctry` с адреса 0x610000 находится вне этого компактного файла.
  Запись с полным стиранием Flash всё равно уничтожит её; файл от этого не защищает.

При `FlashMode=qio` Arduino ESP32 2.0.17 намеренно формирует bootloader с DIO-заголовком
для ROM-стадии и QIO boot-кодом. Поэтому упаковщик сохраняет параметры заголовка
(`keep`), а не переписывает его в QIO после компиляции.
USB-образ затрагивает NVS/otadata и таблицу разделов: настройки могут быть потеряны.
APP OTA не заменяет bootloader/разметку и требует совпадающей установленной схемы
app0 0x10000 / app1 0x310000, по 0x300000. USB MERGED нельзя выбирать как APP OTA.

Эта работа не содержит команд записи, стирания или перезапуска физического ESP32.
Наличие BIN и хэшей не подтверждает работу на настоящем N16R8.
