"""Source integration guard; not a physical touch latency test."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
ui = (root/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
cal = (root/'firmware/BUSA_Display_N4R8/BUSA_TouchCalibration.h').read_text()
assert 'lv_timer_set_period(input_.read_timer,busa_touch::readPeriodMs);' in ui, 'LVGL input still uses the default 30 ms cadence'
assert 'readPeriodMs=10' in cal, 'Touch input cadence is not 10 ms'
assert 'if(!busa_touch::sampleDue(millis(),lastCalibrationRead_))return;' in ui, 'Calibration may count cached sensor samples'
assert 'if(calSamples_<32)' in ui and 'if(calSamples_<3)' in ui, 'Calibration sample bounds changed'
assert 'abs(fitted.x-expected.x)>18||abs(fitted.y-expected.y)>18' in ui, 'Fourth-point accuracy check changed'
print('PASS: touch timer and calibration integration (static)')
