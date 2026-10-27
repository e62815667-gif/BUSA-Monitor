# Построчная карта изменений SAFE1

Исходный коммит: `5db974bc45c746cd9e665a67be18518b1434c6a3`. Полный diff: SOURCE_CHANGES.patch в архиве выпуска и diff коммита на GitHub.


## .github/workflows/busa-google-build.yml

- `@@ -4 +4 @@ on:`
- `@@ -50 +50 @@ jobs:`
- `@@ -57 +57 @@ jobs:`
- `@@ -65 +65 @@ jobs:`
- `@@ -74,2 +74,5 @@ jobs:`
- `@@ -130 +133 @@ jobs:`
- `@@ -133 +136 @@ jobs:`
- `@@ -141 +144 @@ jobs:`
- `@@ -178 +181 @@ jobs:`
- `@@ -194 +197 @@ jobs:`
- `@@ -203 +206 @@ jobs:`
- `@@ -231 +234 @@ jobs:`
- `@@ -307,2 +310,2 @@ jobs:`
- `@@ -313 +316 @@ jobs:`
- `@@ -315 +318 @@ jobs:`
- `@@ -318 +321 @@ jobs:`

## docs/V15_1_TFIX_SAFE1_AUDIT.md

- `@@ -0,0 +1,80 @@`

## docs/V15_1_TFIX_SAFE1_WIRING.md

- `@@ -0,0 +1,46 @@`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h

- `@@ -5 +5 @@`
- `@@ -39 +39 @@`
- `@@ -327 +327 @@ int scanKeyPair(){`
- `@@ -329 +329 @@ int scanKeyPair(){`
- `@@ -360,3 +360,2 @@ int scanKeyPair(){`
- `@@ -377,0 +377,10 @@ int scanKeyPair(){`
- `@@ -379 +388 @@ String pairName(int p){`
- `@@ -731 +740 @@ void darkenTftDisplay(){`
- `@@ -1077 +1086 @@ main>*{max-width:100%;min-width:0}`
- `@@ -1091 +1100 @@ main>*{max-width:100%;min-width:0}`
- `@@ -1103,4 +1112,4 @@ main>*{max-width:100%;min-width:0}`
- `@@ -1109,2 +1118,2 @@ main>*{max-width:100%;min-width:0}`
- `@@ -1474 +1483 @@ function graphs(){graphAllTemperatures();plot('graphF',waterPulseLog)}`
- `@@ -1487 +1496 @@ function renderTemperatureBindings(s){`
- `@@ -1540 +1549 @@ if(s.sound_alarms && Array.isArray(s.sound_alarms.channels)){`
- `@@ -1603 +1612,2 @@ if(s.keypad){`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Temperature.h

- `@@ -25,0 +26 @@ TempBinding tempBindings[4];`
- `@@ -29 +30,2 @@ bool tempRomValid(const uint8_t* rom){return rom[0]==0x28 && OneWire::crc8(rom,7`
- `@@ -32 +34 @@ int tempDeviceForLogical(int n){`
- `@@ -41,2 +43,2 @@ bool tempRomUniqueAcrossBuses(const uint8_t* rom){`
- `@@ -47,0 +50,8 @@ bool tempExplicitMatchOnBus(int b,const uint8_t* rom){`
- `@@ -49,2 +59,2 @@ bool oneWireVerified(int n){`
- `@@ -52,2 +62,2 @@ bool oneWireVerified(int n){`
- `@@ -60,0 +71 @@ String wireError(int n,uint32_t now){`
- `@@ -62,0 +74 @@ String wireError(int n,uint32_t now){`
- `@@ -64,0 +77,2 @@ String wireError(int n,uint32_t now){`
- `@@ -66,2 +79,0 @@ String wireError(int n,uint32_t now){`
- `@@ -69 +81,4 @@ String wireError(int n,uint32_t now){`
- `@@ -71 +86,2 @@ String wireError(int n,uint32_t now){`
- `@@ -96 +112 @@ void tempCommitProbe(uint8_t b,uint32_t now){`
- `@@ -113 +129 @@ void temperatureTick(uint32_t now){`
- `@@ -141 +157,6 @@ void temperatureTick(uint32_t now){`
- `@@ -166,5 +187,3 @@ void temperatureTick(uint32_t now){`
- `@@ -175,2 +194,4 @@ bool tempBindingRecordValid(const TempBindingRecord& r){`
- `@@ -179,0 +201,7 @@ bool tempBindingRecordValid(const TempBindingRecord& r){`
- `@@ -182 +210,4 @@ bool tempValidateBindings(const TempBinding candidate[4]){`
- `@@ -185 +216 @@ bool tempValidateBindings(const TempBinding candidate[4]){`
- `@@ -187,2 +218 @@ bool tempValidateBindings(const TempBinding candidate[4]){`
- `@@ -193 +223,3 @@ bool tempValidateBindings(const TempBinding candidate[4]){`
- `@@ -195 +227 @@ bool tempValidateBindings(const TempBinding candidate[4]){`
- `@@ -202,0 +235 @@ bool tempParseSelection(const String& selection,TempBinding& b){`
- `@@ -217 +250 @@ String temperatureInventoryJSON(uint32_t now){`
- `@@ -221,0 +255,2 @@ String temperatureInventoryJSON(uint32_t now){`
- `@@ -223 +258 @@ String temperatureInventoryJSON(uint32_t now){`
- `@@ -238 +273,3 @@ String temperatureDiagnosticsJSON(uint32_t now){`
- `@@ -240 +277 @@ String temperatureDiagnosticsJSON(uint32_t now){`
- `@@ -244,0 +282 @@ String temperatureDiagnosticsJSON(uint32_t now){`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_Alarm.h

- `@@ -2,3 +2,3 @@`
- `@@ -6,2 +5,0 @@`
- `@@ -10 +8 @@ BusaAlarm::Channel alarmChannels[4];`
- `@@ -13,3 +10,0 @@ uint32_t alarmLastPoll=0;`
- `@@ -18,2 +13,2 @@ void alarmOutputsOff(){`
- `@@ -23,6 +18,2 @@ void alarmInitPins(){`
- `@@ -32 +23,2 @@ void alarmLoadSettings(){`
- `@@ -35,14 +27,3 @@ void alarmPoll(uint32_t now){`
- `@@ -52,2 +33,2 @@ String alarmStatusJSON(uint32_t now){`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_AlarmLogic.h

- `@@ -7 +7 @@ namespace BusaAlarm {`
- `@@ -33,0 +34 @@ inline const char* reasonText(Reason r){`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_Architecture.h

- `@@ -7 +7 @@ constexpr uint8_t kLogicalRelayChannels=4;`
- `@@ -12 +12 @@ String v15ArchitectureJSON(){`
- `@@ -14 +14 @@ String v15ArchitectureJSON(){`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_Board.h

- `@@ -2 +2 @@`
- `@@ -6 +6 @@ constexpr uint8_t TPIN[4]={16,17,18,21};`
- `@@ -10,2 +10,2 @@ constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_LandscapeSafe.h

- `@@ -0,0 +1,442 @@`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_RelayModel.h

- `@@ -5,2 +5,2 @@`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_Runtime.h

- `@@ -4 +4 @@ void setup(){`
- `@@ -8 +8 @@ void setup(){`
- `@@ -16,4 +16,4 @@ void setup(){`
- `@@ -123 +123 @@ void setup(){`
- `@@ -130 +130 @@ void setup(){`
- `@@ -136 +136 @@ void setup(){`
- `@@ -142,0 +143,5 @@ void setup(){`
- `@@ -156 +161,5 @@ void setup(){`
- `@@ -493 +502 @@ void setup(){`

## firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_V15_Status.h

- `@@ -8 +8 @@ String statusJSON(){`
- `@@ -32 +32 @@ String statusJSON(){`
- `@@ -61,0 +62 @@ String statusJSON(){`
- `@@ -69,0 +71 @@ String statusJSON(){`

## tests/test_gpio_safety.py

- `@@ -0,0 +1,69 @@`

## tests/test_keypad.py

- `@@ -0,0 +1,56 @@`

## tests/test_temperature.py

- `@@ -24 +24 @@ struct OneWire{`
- `@@ -31 +31 @@ struct OneWire{`
- `@@ -33 +33 @@ struct OneWire{`
- `@@ -45 +45,6 @@ void run(uint32_t duration){uint32_t end=nowMs+duration;while((int32_t)(nowMs-en`
- `@@ -80 +85 @@ int main(){`
- `@@ -83,2 +88,2 @@ int main(){`
- `@@ -89 +94 @@ int main(){`
- `@@ -91,3 +96,2 @@ int main(){`
- `@@ -116,0 +121,59 @@ int main(){`

## tests/test_temperature_ui.js

- `@@ -19,0 +20,19 @@ console.log('PASS: actual app helpers: verified finite readings only, real 85 C,`

## tests/test_v1427_user_patch.py

- `@@ -24 +24 @@ assert 'server.collectHeaders(headers,1);\n  server.begin();' in main`

## tests/test_v15_architecture.py

- `@@ -22 +22 @@ assert "BUS_SDA=8,BUS_SCL=9" in parts["BUSA_V15_Board.h"]`

## tests/test_v15_sound_alarms.py

- `@@ -59 +58,0 @@ assert 'WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str()' in runtime`
- `@@ -67,2 +66,4 @@ assert "id=\"alarmSave\"" in main and "alarmEnable0" in main`
- `@@ -70 +71 @@ assert "KPIN[4]={7,6,5,4}" in (src/"BUSA_V15_Board.h").read_text()`

## tests/test_wifi_integration.py

- `@@ -25 +25 @@ assert 'web_health' in m`
