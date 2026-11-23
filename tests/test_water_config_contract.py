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
    assert 'waterVolume.record(diff,rejectedDiff,flowReady,waterPulsesPerLitre)' in main
    assert 'float totalWaterLitres(){return (float)waterVolume.countedLitres;}' in main
    assert 'id="waterPplPassport"' in main
    assert 'id="waterPplSave"' in main
    assert 'id="waterPplInput"' in main
    assert 'MOTION1-20261122-YFS401-PPL1-20261123' in main
    assert 'flowVolumeValid' not in main
    assert 'verifiedBusa001Calibration(waterPulsesPerLitre)' in main
    assert 'waterCalibrationVerified()?"true":"false"' in main
    assert 'volume_excluded_intervals' in main and 'volume_complete' in main
    assert 'BusaWater::diagnosticStatus(waterSampleReady,flowReady,waterPulseDelta)' in main
    assert 'quoteJson(!flowVolumeValid?' not in main
    assert 'rejected_edges_window' in main and 'sample_window_ms' in main
    assert 'FLOWSTATE1-20261124-WATERLIVE1' in main

if __name__=='__main__':
    test_contract()
    print('PASS local YF-S401 config contract')
