# БУСА-1М №001 — CALC1, калькулятор

Исходный коммит: f322b11 (SOURCE_COMMIT.txt содержит полный SHA).
База — TELEM2, рабочие выпуски и main не изменены.

Добавлены три расчёта — разбавление, ареометр, вторая перегонка —
в локальное приложение, онлайн PWA и TFT с PCF8575.
Подробности/модели/условия/сборка/установка:
[RM_CALC1_BUILD.md](../../docs/RM_CALC1_BUILD.md).

Локальная сборка N16R8 PASS: APP 2038816 байт, USB 2104352 байт.
APP помещается в слот 3145728 байт, core 2.0.17/Flash16MB/PSRAM OPI.
Storage 2038457 байт, RAM 79548 байт. Образ/валидационный hash/merged layout PASS.
306 C++/JS сценариев с OIML reference points PASS; реальная клавиатурная
логика с String mock PASS; Chromium local/online на 390×844 и 393×852 PASS,
включая запятую, 3 расчёта, ошибки, localStorage/сброс, тёмную тему и PWA offline.
Существующие BLE/OTA/RainMaker/4 contract tests PASS.

GitHub Actions: https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42071696736
Статус этого прогона будет дополнен после завершения. Первый браузерный прогон
не прошёл, добавлен выбор bundled Chromium и открытый диагностический журнал.
Локальный WebKit отсутствует, скачать его не позволил сетевой шлюз (403).
Наличие Linux WebKit теста не означает проверку настоящего Safari на iPhone.

**Онлайн ещё не опубликовано:** GitHub protection rule отклонило ветку:
“Branch feat/busa-calculator-20261105 is not allowed to deploy to github-pages
due to environment protection rules.”
https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42070996734
Нужна настройка владельцем разрешённых Deployment branches окружения
github-pages; ограничения не обходились. ONLINE.zip содержит готовые файлы сайта.

TFT/реальный PCF8575/настоящий iPhone, live RainMaker, сохранение регистрации
после OTA/RESET и длительный аппаратный тест НЕ проверены. ESP32 не прошивалась.
Калькулятор к GPIO/NVS/сети не обращается; существующая логика сохранена.
Имена/типы cloud параметров сохранены, только диапазон номера TFT экрана
расширен до 7 для нового раздела.

Для действующего устройства использовать OTA BIN через 192.168.4.1.
USB MERGED записывается с 0x0 и затрагивает NVS/настройки — нужна резервная копия.
USB BIN не выбирать в OTA. Не использовать erase-all для сохранения регистрации.

SHA-256:

```text
0f55e7ea93ebc67b5051f139aec9cad848a18a6603181cff0d5e2c20ce5061f2  BUSA_N16R8_RM_CALC1_OTA.bin
7246458f8b19425f2b8728c786be8c74054cf1607b060dffb63eff575f0360cd  BUSA_N16R8_RM_CALC1_USB_MERGED.bin
e4eae20cbec41a22b9c0083ba598c93c217e51eb844de7dd08762cf69440f484  BUSA_N16R8_RM_CALC1_SOURCE.zip
```
