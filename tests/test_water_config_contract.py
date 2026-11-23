from pathlib import Path
import re

main=(Path(__file__).resolve().parents[1]/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
math=(Path(__file__).resolve().parents[1]/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_WaterFlowMath.h').read_text()

def test_contract():
    assert 'kPassportPulsesPerLitre=kHzPerLpm*60.0f' in math
    assert 'float waterPulsesPerLitre=BusaWater::kPassportPulsesPerLitre' in main
    assert 'prefs.getFloat("waterPpl",BusaWater::kPassportPulsesPerLitre)' in main
    assert 'prefs.putFloat("waterPpl",requested)!=sizeof(float)' in main
    assert 'if(!apClient()||server.arg("confirm")!="YES")' in main
    assert 'server.on("/api/water/coefficient",HTTP_POST' in main
    assert 'BusaWater::lpm(diff,dt,waterPulsesPerLitre)' in main
    assert 'BusaWater::credible(diff,dt,rejectedDiff,waterPulsesPerLitre)' in main
    assert 'BusaWater::litres(flowPulses(),waterPulsesPerLitre)' in main
    assert 'id="waterPplPassport"' in main
    assert 'id="waterPplSave"' in main
    assert 'id="waterPplInput"' in main
    assert 'MOTION1-20261122-YFS401-PPL1-20261123' in main
    assert re.search(r'if\(!flowReady\)flowVolumeValid=false;',main)

if __name__=='__main__':
    test_contract()
    print('PASS local YF-S401 config contract')
