"""V15.4 early AP and boot-loop TFT failover source contracts.
Hardware WiFi visibility must still be checked on installed ESP32-S3.
"""
from pathlib import Path
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
runtime=(root/"BUSA_V15_Runtime.h").read_text()
main=(root/"BUSA_Main.h").read_text()
status=(root/"BUSA_V15_Status.h").read_text()
board=(root/"BUSA_V15_Board.h").read_text()
setup=runtime.split("void setup(){",1)[1].split("void loop(){",1)[0]
loop=runtime.split("void loop(){",1)[1]
assert '15v4-N16R8-EARLY-AP-RECOVERY' in main
assert 'FW V15.4 N16R8' in main
assert setup.count("WiFi.mode(WIFI_AP_STA)") == 1
assert setup.count('WiFi.onEvent(wifiRecoveryEvent)') == 1
assert setup.count('apPassword="123456789"') == 1
assert setup.count("WiFi.softAP(") == 2  # one bounded retry only
assert setup.index('WiFi.softAP(') < setup.index('busaN16MemoryBegin()')
assert setup.index('WiFi.softAP(') < setup.index('prefs.begin(')
assert setup.index('WiFi.softAP(') < setup.index('startScreen();')
assert setup.index('WiFi.softAP(') < setup.index('setupGoogleCloud();')
assert 'if(!busaSkipTftAtBoot){screenAttempted=true;startScreen();}' in setup
assert 'busaBootStreak>=3' in setup
assert 'RTC_DATA_ATTR uint32_t busaBootStreak' in runtime
assert 'elapsed(now,startedAt,30000U)' in loop
assert 'busaBootStreak=0;' in loop
assert 'early_ap_active' in status and 'tft_skipped' in status
assert 'BUSA-1M-001-R04' in setup
assert 'server.handleClient()' in loop and 'wifiRecoveryTick(now)' in loop
assert 'TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14' in board
assert 'KPIN[4]={7,6,5,4}' in board
print("PASS V15.4: early WiFi before display/NVS/Google, reboot failover, existing APIs and pins")
