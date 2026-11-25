from pathlib import Path
import re, subprocess, tempfile
root=Path(__file__).resolve().parents[1]
main=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
# The actual loop must no longer invoke the blocking stock transport.
assert 'BusaHttpServer server(80);' in main, 'stock synchronous HTTP server remains in loop'
assert 'server.startWorker();' in main, 'network worker is never started'
assert 'server.pollApplication();' in main and 'server.handleClient();' not in main[main.index('void loop(){'):]
assert 'http_worker_worst_ms' in main and 'http_worker_enabled' in main
# Hardware mapping, endpoint names, UART/display compatibility must be preserved.
assert 'constexpr uint8_t TPIN[4]={16,17,18,21};' in main
assert 'constexpr uint8_t COLLECTION_GPIO=1;' in main
assert 'server.on("/api/ota/upload",HTTP_POST' in main
assert 'server.on("/api/display-ota/upload",HTTP_POST' in main
assert 'collectionMeter.sample(now,collectionPulses());' in main
print('PASS: latency integration and existing endpoint contract')
