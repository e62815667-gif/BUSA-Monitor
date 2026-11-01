"""V15.5 nonblocking priorities: local HTTP must not starve behind peripherals.
Static regression guard; bench latency/uptime must still be measured on BUSA hardware.
"""
from pathlib import Path
r=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE')
rt=(r/'BUSA_V15_Runtime.h').read_text()
main=(r/'BUSA_Main.h').read_text()
google=(r/'BUSA_Google.h').read_text()
boot=(r/'BUSA_Boot.h').read_text()
wifi=(r/'BUSA_WifiRecovery.h').read_text()
assert '15v5-N16R8-STABLE-LOCAL' in main
assert '15.5-STABLE-LOCAL' in boot
assert 'FW V15.5 N16R8' in main
setup=rt.split('void setup(){',1)[1].split('void loop(){',1)[0]
loop=rt.split('void loop(){',1)[1]
assert setup.index('wifiStartAccessPoint(millis())') < setup.index('server.begin()')
assert setup.index('server.begin()') < setup.index('startScreen()')
assert setup.index('server.begin()') < setup.index('launchGoogleCloud()')
assert 'server.handleClient();' in loop
assert loop.index('server.handleClient();') < loop.index('temperatureTick(millis());')
assert loop.index('server.handleClient();') < loop.index('publishGoogleSnapshot(now);')
assert 'elapsed(now,nextTempAt,40U)' in loop
assert 'elapsed(now,lastBusProbe,15000)' in loop
assert 'elapsed(now,lastRender,4000)' in loop
assert 'elapsed(now,googleSnapshotAt,8000)' in google
assert 'edge-flowLastEdgeUs)<2000U' in main
assert 'scanStepAt,100U' in main
assert 'i2cFullScanAt,60000' in main
assert 'BUSA_BOOT_GRACE_MS=3000U' in wifi
assert 'const char WEB_PAGE[] PROGMEM=' in main
assert 'recovery' in rt and '/api/ping' in rt
assert 'WiFi.mode(WIFI_AP_STA)' in setup
assert 'server.send_P(200,"text/html; charset=utf-8",WEB_PAGE)' in rt
print('PASS V15.5: HTTP-first, reduced OneWire/I2C/Google/SPI load, WiFi AP+STA, fallback and pin-safe flow')
