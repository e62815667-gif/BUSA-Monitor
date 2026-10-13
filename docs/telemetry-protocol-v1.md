# БУСА-1М ↔ БУСА-Монитор — формат телеметрии
Редакция 1.1 · 2026-10-13 · предварительный формат до готовности прошивки.

## Состав каналов
T01, T02, T03, T04 (температура, единица C), L01 (уровень, единица выбирается по датчику). Всего пять. Канала измерения крепости нет.

## JSON-пакет
Обязательные поля: protocol="busa.telemetry.v1", device_id, sequence, timestamp, mode, channels.
Каждый канал: id, value, unit, quality, calibration, sensor_status, measured_at.
Значение value — конечное число либо null; quality — valid, stale, invalid или disconnected; calibration — unknown, pending или verified.
В случае недостоверных показаний приложение показывает «Нет данных». В режиме demo реальные измерения не подразумеваются.
Транспорт (локальный HTTP, WebSocket, последовательный мост) будет выбран после разработки прошивки. Формат только для мониторинга, не для управляющих команд.

```json
{
  "protocol":"busa.telemetry.v1",
  "device_id":"busa-1m-prototype",
  "sequence":0,
  "timestamp":null,
  "mode":"demo",
  "channels":[
    {"id":"T01","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"T02","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"T03","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"T04","value":null,"unit":"C","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null},
    {"id":"L01","value":null,"unit":"percent","quality":"disconnected","calibration":"pending","sensor_status":"candidate","measured_at":null}
  ]
}
```
Примечание: candidate в примере относится к моделям физических датчиков; состав каналов утверждён.
