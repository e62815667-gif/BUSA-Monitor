# CALC2 — три калькулятора в ESP RainMaker

Версия 14v9-N16R8-RM-CALC2-20261105. Исходный коммит: f2363875874a9a43e045e246fac3e158fe29afeb.
Работа в feat/busa-calculator-20261105; main, CALC1 и TELEM2 не изменялись.

Пользователь уточнил: онлайн — именно ESP RainMaker. Добавлены native карточки:
1. Калькулятор · Разбавление.
2. Калькулятор · Ареометр.
3. Калькулятор · Перегонка.

Это стандартные esp.device.other с float вводом, toggle Рассчитать/Сбросить
и readonly текстовыми результатами/статусом. Собственные вкладки/жёлтые
веб-экраны внутрь стороннего iOS-приложения прошивка не устанавливает.
Вопрос GitHub Pages больше не относится к запросу; его защиту менять не нужно.

Ввод обрабатывается через очередь в main loop, не в SDK callback. Учтено,
что Arduino RainMaker передаёт временный Device wrapper: сравниваются SDK
handles, а не адреса обёрток. Все результаты входят в один существующий
10-секундный пакет с температурами; unchanged calculator revisions не
обновляются, не увеличивая частоту MQTT. Неверный/изменённый ввод даёт «—»,
а не устаревший или вымышленный нулевой результат.

Полная инструкция/модели/ограничения/сборка:
[RM_CALC2_BUILD.md](../../docs/RM_CALC2_BUILD.md).

**GitHub Actions SUCCESS:**
https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42077310576
Все шаги, включая compile/package/upload, прошли. Native calculator
regression, прежние BLE/OTA/telemetry/PUBACK/сохранность модулей/4 contract
tests PASS; 306 C++/JS сценариев и keypad PASS; Chromium/WebKit mobile UI PASS.
Native-тест исполняет настоящие функции с SDK mock, НЕ реальный облачный обмен.
Проверены 3 схемы и права, временные wrappers, очередь/типы, расчёты/сброс/ошибки,
неизменяемый снимок, silent API success retry, отсутствие лишних отправок за час,
изоляция недоступной модели от T1–T4.

Локальная сборка ESP32-S3 N16R8/core2.0.17: storage 2044905 байт, RAM 81828.
APP BIN 2045264 байт ≤ слот 3145728; USB MERGED 2110800 байт. Flash16MB,
PSRAM OPI, прежняя CSV-разметка. Chip ID/image checksum/validation hash/merged
layout/APP boundaries PASS. BIN собраны локально из того же исходного коммита;
совпадение SHA локальной и независимой CI-сборки не заявляется.

После **OTA BIN** через 192.168.4.1 обновить список устройств в RainMaker,
затем Edit полей → включить «Рассчитать» → дождаться «Готово» и результата.
Переключатель автоматически сбрасывается в Off следующим отчётом.
Не удалять БУСА из аккаунта и не повторять BLE. На /api/rainmaker/status
поле telemetry.calculator_devices_ready должно стать 3; это только локальная
инициализация SDK-моделей, не доказательство их доставки/появления в приложении.
USB с 0x0 затрагивает NVS/настройки, требует резервной копии. Не выбирать
USB MERGED в OTA и не erase-all для сохранения identity.

**Не проверено физически:** ESP32/iPhone, появление карточек у владельца,
реальный ввод/обратная доставка через RainMaker, сохранение identity после
нового OTA/RESET, длительные память/TLS/worker stack. ESP32 не прошивалась.
Сборка и SDK-модель не выдаются за подтверждение облачной доставки.

SHA-256:

```text
2dabe7ddc2a6b815bd4f64480120b9378a05b7a0c18d5bc94255fd3cad2c3a0b  BUSA_N16R8_RM_CALC2_OTA.bin
c3411fa640aae889bfd920ef72d70a0399fefe534744af2c4a8b0827dd96e0b5  BUSA_N16R8_RM_CALC2_USB_MERGED.bin
b47146ad02eba36601008aa93a5dc9f9c9003265dacf375bd77f3680452496d8  BUSA_N16R8_RM_CALC2_SOURCE.zip
```
