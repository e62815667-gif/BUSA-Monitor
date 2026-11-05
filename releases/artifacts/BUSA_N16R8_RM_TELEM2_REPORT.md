# TELEM2 — T1–T4 с точностью до десятых

Исходный коммит: 9c99111c0205d45bf15cdfdf3d42704179cfa452.

Исправление: только значения Temperature, отправляемые в RainMaker,
округляются до 0,1 °C. Пример на снимке: 30.875 → 30.9.
Исходные измерения, Google, локальный UI и утверждённая логика не изменены.
Запятую/точку, знак °C и завершающий ноль оформляет приложение RainMaker.
Новая регистрация и очистка NVS не добавлены.

Полная инструкция: [RM_TELEM2_BUILD.md](../../docs/RM_TELEM2_BUILD.md).

Локальная сборка ESP32-S3 N16R8 PASS, core 2.0.17/CLI 1.3.1, flash16MB,
PSRAM OPI, прежняя разметка FIX3. APP 2017312 байт; слот 3145728 байт.
Storage 2016953 байт; RAM 79220 байт. APP image/checksum/hash, USB layout PASS.
Тесты округления и сохранения исходных значений PASS; часовая модель RainMaker,
PUBACK tracker, BLE/OTA, сохранность модулей/схемы облачных параметров PASS;
4 firmware contract tests PASS.

Физический ESP32 не прошивался. На настоящем iPhone/RainMaker отображение
и live-доставка ещё не проверены. Остальные физические проверки — в инструкции.
Для действующего устройства использовать OTA BIN в 192.168.4.1. USB MERGED
записывается с 0x0, затрагивает NVS/настройки и требует резервной копии.
Не выбирать USB BIN для OTA; не выполнять erase-all для сохранения identity.

SHA-256:

```text
b624b994e45f93453ca813b817faf92b07fdddcb4c474e4853ba25529514fea1  BUSA_N16R8_RM_TELEM2_OTA.bin
c435405a62602620dd833e379af49dd8b17435cbbb1fa654125aca61b968675a  BUSA_N16R8_RM_TELEM2_USB_MERGED.bin
c07d58696904fd90607e6f23edb92683da9bd6728954f159e2ef5cdd3f028bff  BUSA_N16R8_RM_TELEM2_SOURCE.zip
```
