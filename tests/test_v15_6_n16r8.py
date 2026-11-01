"""BUSA full V15.6/N16R8 board and application compatibility checks."""
from pathlib import Path
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
board=(root/"BUSA_V15_Board.h").read_text()
main=(root/"BUSA_Main.h").read_text()
runtime=(root/"BUSA_V15_Runtime.h").read_text()
memory=(root/"BUSA_N16R8_Memory.h").read_text()
alarms=(root/"BUSA_V15_Alarm.h").read_text()
assert '15v6-N16R8-MODULE-BOOTFIX' in main
assert 'TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14' in board
assert 'TPIN[4]={16,17,18,21}' in board
assert 'KPIN[4]={7,6,5,4}' in board
assert 'BUS_SDA=8,BUS_SCL=9' in board
assert 'FLOW_GPIO=15' in board
assert 'SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS)' in main
assert 'BusaST7789 screenSt(TFT_CS,TFT_DC,TFT_RST)' in main
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
boot=(root/"BUSA_Boot.h").read_text()
web=Path("online/index.html").read_text()
sw=Path("online/sw.js").read_text()
assert "busaTftBootIssued" in runtime and "elapsed(now,startedAt,1500U)" in runtime
assert "elapsed(now,startedAt,5000U)" in runtime
assert runtime.index("void loop(){") < runtime.index("startScreen();")
assert runtime.index("void loop(){") < runtime.index("Wire.begin(BUS_SDA,BUS_SCL,100000)")
assert "tft_failure_count" in boot and "i2c_failure_count" in boot
assert "peripheral_boot" in (root/"BUSA_V15_Status.h").read_text()
assert 'repeat(6,minmax(0,1fr))' in web
assert '<span>Кальк.</span>' not in web and '<span>Кальк.</span>' not in main
assert 'busa-monitor-v15.6-module-bootfix' in sw
assert 'setInterval(invalidateStaleRemote,5000)' in web
assert 'ch.sound_on===true' in web
print("PASS V15.6: N16R8 pins, delayed TFT/I2C, reset diagnostics, six approved online pages and stale-data protection")
