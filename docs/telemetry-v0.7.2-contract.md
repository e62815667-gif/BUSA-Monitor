# BUSA-1M-001 — proposed read-only telemetry contract (v0.7.2)

Status: **integration proposal, not a verified firmware implementation**. Existing iPhone PWA v0.6.2 already expects these read-only endpoints. Do not enable physical outputs.

## GET /api/capabilities
```json
{"device":"BUSA-1M-001","version":"v0.7.2","api_read_only":true,"physical_outputs":false,"temperature_channels":4,"keyboard":"5x4 PCF8575","display":false}
```
Keyboard value may instead be `4x4 PCF8575` if confirmed by hardware. This field is checked by the current PWA.

## GET /api/telemetry
```json
{"device":"BUSA-1M-001","version":"v0.7.2","mode":"MONITOR_ONLY","outputs_enabled":false,"bridge_online":false,"keypad_online":false,"temperatures":[null,null,null,null],"channels":[{"valid":false,"age_ms":null},{"valid":false,"age_ms":null},{"valid":false,"age_ms":null},{"valid":false,"age_ms":null}],"last_key":null,"key_seq":0,"key_stream_id":"boot-1"}
```
T1 water, T2 column, T3 cube, T4 upper column. Valid readings must be numeric -55..125 Celsius, with `age_ms` 0..3000 and bridge online. On disconnect, send `valid:false` and `null` temperature, never simulated measurements marked real.

## GET /api/keys
```json
{"stream_id":"boot-1","events":[]}
```
At most 64 events, each with positive integer `seq` and string `key`. Stream ID must match telemetry and change after reboot.

## Safety and implementation gates
- HTTP GET endpoints are read-only; no relay/valve/heat-control endpoints.
- Physical outputs disabled, independent hardware protection remains required.
- No claim of algorithm equivalence for T1–T4 or K1–K4 until original `main.cpp`, `BusaLogic.h`, and `BusaAutomation.h` are recovered and tested.
- GitHub Pages HTTPS cannot reliably fetch controller HTTP because of mixed-content restrictions; provide a local read-only controller page or verified HTTPS solution.
- This is an API contract draft based on existing PWA validation, **not** proof of a running ESP32 endpoint.
