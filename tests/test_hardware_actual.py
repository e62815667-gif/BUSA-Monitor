from pathlib import Path
import re, subprocess, tempfile, json
root=Path(__file__).resolve().parents[1]
fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
main=(fw/'BUSA_Main.h').read_text()
sound=(fw/'BUSA_SoundAlarms.h').read_text()
assert 'PCA9685' not in main and 'pca9685' not in main, 'obsolete PCA diagnostics'
assert not any(x in main for x in ('Adafruit_ST7789','Adafruit_ILI9341','SPI.begin(', 'TFT_CS')), 'obsolete physical display'
assert '#include "BUSA_History.h"' in main
assert 'collection_installed' in main and '\\"pulse_seen\\"' in main
flag=re.search(r'constexpr bool BUSA_BUZZER_ACTIVE_HIGH\s*=\s*(?:true|false);',sound).group()
fn=re.search(r'(?:constexpr )?int buzzerPinLevel\(bool active\)\{[^}]+\}',sound).group()
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'polarity.cpp'; exe=Path(d)/'polarity'
 p.write_text('#include <cassert>\nconstexpr int HIGH=1,LOW=0;\n'+flag+'\n'+fn+'\nint main(){assert(buzzerPinLevel(false)==HIGH);assert(buzzerPinLevel(true)==LOW);}\n')
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
assert 'String(BUSA_BUZZER_ACTIVE_HIGH?' in sound
assert 'v13Collect(now);' in main and 'busaCaptureChart(now);' in (fw/'BUSA_History.h').read_text()
assert 'pcaDetected' not in (fw/'BUSA_History.h').read_text()
receiver=(root/'google/BUSA_GoogleSheets_Receiver.gs').read_text()
assert 'Boolean(pca.ack)' not in receiver
expr=re.search(r'const hasErrors = ([\s\S]*?);',receiver).group(1)
script='const vm=require("vm"),assert=require("assert/strict");const test=ctx=>vm.runInNewContext('+json.dumps(expr)+',ctx);'
script+='assert.equal(test({errs:["OK","OK","OK","OK"],keyboard:{scan_error:0},modules:{pcf8575:true}}),false);'
script+='assert.equal(test({errs:["OK","OK","NO_DATA","OK"],keyboard:{scan_error:0},modules:{pcf8575:true}}),true);'
script+='assert.equal(test({errs:["OK","OK","OK","OK"],keyboard:{scan_error:2},modules:{pcf8575:true}}),true);'
script+='assert.equal(test({errs:["OK","OK","OK","OK"],keyboard:{scan_error:0},modules:{pcf8575:false}}),true);'
subprocess.run(['node','-e',script],check=True)
print('PASS: hardware, polarity, obsolete diagnostics and pulse evidence')
