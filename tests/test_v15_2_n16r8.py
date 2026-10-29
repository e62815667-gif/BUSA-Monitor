"""BUSA full V15.2/N16R8 board and application compatibility checks."""
from pathlib import Path
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
board=(root/"BUSA_V15_Board.h").read_text()
main=(root/"BUSA_Main.h").read_text()
runtime=(root/"BUSA_V15_Runtime.h").read_text()
memory=(root/"BUSA_N16R8_Memory.h").read_text()
alarms=(root/"BUSA_V15_Alarm.h").read_text()
assert '15v2-N16R8-TFT-REPIN-PSRAM' in main
assert 'TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14' in board
assert 'TPIN[4]={16,17,18,21}' in board
assert 'KPIN[4]={7,6,5,4}' in board
assert 'BUS_SDA=8,BUS_SCL=9' in board
assert 'FLOW_GPIO=15' in board
assert 'SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS)' in main
assert 'Adafruit_ST7789 screenSt(TFT_CS,TFT_DC,TFT_RST)' in main
assert '#include "BUSA_N16R8_Memory.h"' in main
assert 'busaN16MemoryBegin()' in runtime
assert 'busaN16MemoryLog(now,temperature,flowLpm,flowPulses())' in runtime
assert 'MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT' in memory
assert 'BUSA_PSRAM_SAMPLE_CAPACITY=512' in memory
assert 'psram_bytes' in runtime
assert 'N16R8 HEARTBEAT' in runtime
assert 'WiFi.mode(WIFI_AP_STA)' in runtime
assert 'server.on("/api/ping"' in runtime
assert 'server.on("/api/status"' in runtime
assert 'server.on("/api/alarm/config"' in runtime
assert 'server.on("/api/temperature/bind"' in runtime
assert 'server.handleClient()' in runtime
assert 'server.on("/api/wifi"' in runtime
assert 'pinMode(KPIN[i],OUTPUT)' in alarms
assert 'SOUND' in alarms and 'alarmPoll(now)' in runtime
assert 'Google' in main
assert 'YF-S401' in main
assert 'BUSA_V15_Status.h' in main
assert 'BUSA_PCA9685_Leds.h' in main
assert 'BUSA_Temperature.h' in main
assert not any(text in main for text in ('15v2-N8R2', 'FLOW_GPIO=5'))
partition=(root/"partitions.csv").read_text()
assert 'ota_0' in partition and 'ota_1' in partition
assert '0xFF0000' in partition
print("PASS full firmware V15.2 N16R8 pin, network, audio and PSRAM contracts")
