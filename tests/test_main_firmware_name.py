"""Verify the actual browser label code with Node, including old-build fallback."""
from pathlib import Path
import re
import subprocess
s=(Path(__file__).resolve().parents[1]/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
version=re.search(r'const char VERSION\[\]="([^"]+)";',s).group(1)
assert version.endswith('-PERF4-HW2-NAME1')
start=s.index('const firmwareLabel=')
end=s.index("get('aboutUptime')",start)
snippet=s[start:end]
js=r"""
const assert=require('node:assert/strict');
function render(firmware){
  const s={firmware};
  const nodes={aboutFirmware:{},otaFirmware:{}};
  const get=id=>nodes[id];
  """+snippet+r"""
  return [nodes.aboutFirmware.textContent,nodes.otaFirmware.textContent];
}
assert.deepEqual(render('БУСА 1М WiFi example-PERF4-HW2-NAME1'),['BUSA 1M AI 1v','BUSA 1M AI 1v']);
assert.deepEqual(render('БУСА 1М WiFi example-PERF4-HW2'),['example-PERF4-HW2','example-PERF4-HW2']);
"""
subprocess.run(['node','-e',js],check=True)
assert 'device:s.device,firmware:s.firmware,uptime_ms:s.uptime_ms' in s
assert "get('ver').textContent=s.firmware;" in s
print('PASS short firmware name in both pages, old build fallback and technical diagnostics')
