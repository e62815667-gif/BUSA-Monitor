"""V15 migration contract: a real modular source split, not a version rename."""
from pathlib import Path
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
main=(root/"BUSA_Main.h").read_text()
names=("BUSA_V15_Board.h","BUSA_V15_RelayModel.h","BUSA_V15_Architecture.h","BUSA_V15_Status.h","BUSA_V15_Runtime.h")
parts={n:(root/n).read_text() for n in names}
for name in names:
 assert ('#include "'+name+'"') in main,name
 assert parts[name].startswith("#pragma once"),name
assert '15v2-N16R8-TFT-REPIN-PSRAM' in main
assert "void setup(){" not in main and "void loop(){" not in main
assert parts["BUSA_V15_Runtime.h"].count("void setup(){")==1
assert parts["BUSA_V15_Runtime.h"].count("void loop(){")==1
assert "String statusJSON(){" not in main
assert "String statusJSON(){" in parts["BUSA_V15_Status.h"]
assert "s.reserve(8192);" in parts["BUSA_V15_Status.h"]
assert "void relaySimTick(" not in main
assert "void relaySimTick(" in parts["BUSA_V15_RelayModel.h"]
assert "TPIN[4]={16,17,18,21}" in parts["BUSA_V15_Board.h"]
assert "KPIN[4]={7,6,5,4}" in parts["BUSA_V15_Board.h"]
assert "BUS_SDA=8,BUS_SCL=9" in parts["BUSA_V15_Board.h"]
assert "RELAY_PHYSICAL_OUTPUTS=true,LED_PHYSICAL_OUTPUTS=false" in parts["BUSA_V15_Board.h"]
assert '#include "BUSA_V15_Alarm.h"' in main
assert "physical_relay_driver_implemented" in parts["BUSA_V15_Architecture.h"]
assert "v15ArchitectureJSON()" in parts["BUSA_V15_Status.h"]
assert "wifiRecoveryTick(now)" in parts["BUSA_V15_Runtime.h"]
assert "temperatureTick(millis())" in parts["BUSA_V15_Runtime.h"]
assert "server.handleClient()" in parts["BUSA_V15_Runtime.h"]
assert "relaySimTick(now)" in parts["BUSA_V15_Runtime.h"]
assert 'void uploadOTA()' in main
for body in [main,*parts.values()]:
 assert "setInsecure(" not in body
 assert "ledcWrite(" not in body
 assert "setInsecure(" not in body
print("PASS V15: board, relay model, telemetry, architecture and runtime are separate; legacy compatibility retained")
