"""Verify actual equipment corrections, using production polarity function."""
from pathlib import Path
import re, subprocess, tempfile
root=Path(__file__).resolve().parents[1]
fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
main=(fw/'BUSA_Main.h').read_text()
sound=(fw/'BUSA_SoundAlarms.h').read_text()
history=(fw/'BUSA_ApprovedUI_V13.h').read_text()
assert not re.search(r'pca9685|pcaDetected|pcaAddr|pcaMode|busaI2c.*Pca',main+'\n'+history,re.I)
assert '\\"installed\\":true,\\"pulse_seen\\"' in main
assert 'Нет импульсов' in main and 'Нет датчика отбора' not in main
flag=re.search(r'constexpr bool BUSA_BUZZER_ACTIVE_HIGH\s*=\s*(?:true|false);',sound).group()
fn=re.search(r'int buzzerPinLevel\(bool active\)\{[^}]+\}',sound).group()
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'polarity.cpp'; exe=Path(d)/'polarity'
 p.write_text('#include <cassert>\nconstexpr int HIGH=1,LOW=0;\n'+flag+'\n'+fn+'\nint main(){assert(buzzerPinLevel(false)==HIGH);assert(buzzerPinLevel(true)==LOW);}\n')
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
assert 'String(BUSA_BUZZER_ACTIVE_HIGH?' in sound
print('PASS HW1: no obsolete I2C module, installed vs observed pulses, active-low sound input')
