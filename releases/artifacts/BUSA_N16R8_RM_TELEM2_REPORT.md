# TELEM2 — T1–T4 с точностью до десятых

Исходный коммит: cb8d43c886fddbd823616bf6b02658a8464c20c4.

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
Также проверена сериализация до десятых на всём штатном диапазоне DS18B20,
с учётом формата %.5f из json_generator поставляемого SDK.
Исходники firmware cb8d43c и 9c99111 побайтово идентичны.

Физический ESP32 не прошивался. На настоящем iPhone/RainMaker отображение
и live-доставка ещё не проверены. Остальные физические проверки — в инструкции.
Для действующего устройства использовать OTA BIN в 192.168.4.1. USB MERGED
записывается с 0x0, затрагивает NVS/настройки и требует резервной копии.
Не выбирать USB BIN для OTA; не выполнять erase-all для сохранения identity.

GitHub Actions **SUCCESS** (все шаги, включая compile/package/upload):
https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42061575773

BIN собраны локально; Actions независимо собрал те же исходники с тем же
профилем. Идентичность SHA локальной и CI сборок не заявляется.

SHA-256:

```text
b624b994e45f93453ca813b817faf92b07fdddcb4c474e4853ba25529514fea1  BUSA_N16R8_RM_TELEM2_OTA.bin
c435405a62602620dd833e379af49dd8b17435cbbb1fa654125aca61b968675a  BUSA_N16R8_RM_TELEM2_USB_MERGED.bin
6f3b2f6634f8b2d8c764c8146f3646495fefcc969581313003a4f066100e2be0  BUSA_N16R8_RM_TELEM2_SOURCE.zip
```
