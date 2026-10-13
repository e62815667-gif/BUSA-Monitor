# БУСА-1М ↔ БУСА-Монитор: единый формат данных
Версия протокола: 1.0 (проект, не прошивка). Статус: предварительный до утверждения датчиков.

## Транспорт
UTF-8 JSON. Транспорт пока не выбран: формат применим к локальному HTTP, WebSocket, последовательному мосту или импорту файла. Это только телеметрия, без команд управления.

## Обязательная структура сообщения
- `protocol`: "busa.telemetry.v1"
- `device_id`: строковый идентификатор устройства
- `sequence`: неотрицательный целый счётчик пакетов, растёт на стороне устройства
- `timestamp`: UTC ISO 8601 с Z; если часы устройства не синхронизированы, значение null
- `mode`: "demo" | "live"
- `channels`: массив записей
- `channels[].id`: T01, T02, T03, T04, L01, ABV_OUT
- `channels[].value`: конечное число или null
- `channels[].unit`: "C" для температур, "percent" / "mm" / "L" для L01 после выбора датчика, "percent_abv" для ABV_OUT
- `channels[].quality`: "valid" | "stale" | "invalid" | "disconnected"
- `channels[].calibration`: "unknown" | "pending" | "verified"
- `channels[].sensor_status`: "candidate" | "approved" | "installed"
- `channels[].measured_at`: UTC ISO 8601 с Z или null

## Правила проверки
1. Неподдерживаемые версии протокола отклоняются.
2. ID каналов не дублируются; неизвестные ID игнорируются с диагностикой.
3. Значение null при отсутствии достоверного измерения; качество не valid при null.
4. Если quality != valid, интерфейс показывает «Нет данных» и не трактует value как достоверное.
5. Режим demo всегда явно обозначается и не может считаться реальным измерением.
6. Значения не должны автоматически запускать управляющие команды.
7. Пакеты с пропущенным sequence диагностируются как потеря данных. Устаревание рассчитывается по времени приёма и интервалу опроса, который определим после выбора аппаратуры.
8. Единицы не конвертируются без явно указанного метода. Для ABV_OUT значение percent_abv является справочным, пока измеритель и метод калибровки не подтверждены.
9. В приложении не утверждать sensor_status=approved/installed без отдельной проверки готовой прошивки, физического монтажа и совместимости.

## Пример без вымышленных измерений
```json
{
  "protocol": "busa.telemetry.v1",
  "device_id": "busa-1m-prototype",
  "sequence": 0,
  "timestamp": null,
  "mode": "demo",
  "channels": [
    {"id":"T01","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"T02","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"T03","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"T04","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"L01","value":null,"unit":"percent","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"ABV_OUT","value":null,"unit":"percent_abv","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null}
  ]
}
```

## Перед утверждением оборудования
Проверить габариты, расположение, питание, интерфейс, материалы, обслуживание, безопасность, точность и возможность интеграции в прошивку. Защиты автономны от приложения и ИИ.
