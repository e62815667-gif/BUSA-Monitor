from pathlib import Path
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
m=(root/"BUSA_Main.h").read_text()
r=(root/"BUSA_V15_Runtime.h").read_text()
w=(root/"BUSA_WifiRecovery.h").read_text()
g=(root/"BUSA_Google.h").read_text()
b=(root/"BUSA_V15_Board.h").read_text()
assert '15v5-N16R8-STABLE-LOCAL' in m
assert 'TPIN[4]={16,17,18,21}' in b
assert 'TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14' in b
assert 'KPIN[4]={7,6,5,4}' in b
assert r.index('wifiStartAccessPoint(millis())') < r.index('server.begin()') < r.index('startScreen();')
assert r.index('server.handleClient()') < r.index('temperatureTick(millis())')
assert 'if(nextTempAt==0 || elapsed(now,nextTempAt,40U))' in r
assert 'if(googleSnapshotAt && !elapsed(now,googleSnapshotAt,8000))return;' in g
assert 'edge-flowLastEdgeUs)<2000U' in m
assert 'pinMode(FLOW_GPIO,INPUT_PULLUP)' in r
assert 'if(scanStepAt && !elapsed(now,scanStepAt,100U))return;' in m
assert 'if(i2cFullScanAt && !elapsed(now,i2cFullScanAt,60000))return;' in m
assert 'BUSA_BOOT_GRACE_MS=3000U' in w
assert 'server.on("/api/ping"' in r and 'server.on("/recovery"' in r
assert 'server.on("/api/status"' in r
print("PASS V15.5: local first, bounded sensor/I2C/Google work, approved wiring")
