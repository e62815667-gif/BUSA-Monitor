"""Verify the actual browser label code with Node, including old-build fallback."""
from pathlib import Path
import re
import subprocess
s=(Path(__file__).resolve().parents[1]/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
version=re.search(r'const char VERSION\[\]="([^"]+)";',s).group(1)
assert version.endswith('-PERF4-HW2-NAME1-TIME1')
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
assert.deepEqual(render('БУСА 1М WiFi example-PERF4-HW2-NAME1-TIME1'),['BUSA 1M AI 1v','BUSA 1M AI 1v']);
assert.deepEqual(render('БУСА 1М WiFi example-PERF4-HW2-NAME1'),['BUSA 1M AI 1v','BUSA 1M AI 1v']);
assert.deepEqual(render('БУСА 1М WiFi example-PERF4-HW2'),['example-PERF4-HW2','example-PERF4-HW2']);
"""
subprocess.run(['node','-e',js],check=True)
assert 'device:s.device,firmware:s.firmware,uptime_ms:s.uptime_ms' in s
assert "get('ver').textContent=s.firmware;" in s
print('PASS short firmware name in both pages, old build fallback and technical diagnostics')

fmt=re.search(r'function formatUptime\(ms\)\{[^\n]+\}',s).group(0)
measured=re.search(r'function measured\(v\)\{[^\n]+\}',s).group(0)
subprocess.run(['node','-e',measured+'\n'+fmt+'\n'+r"""
const assert=require('node:assert/strict');
for(const [ms,expected] of [[0,"0ч00мин"],[59999,"0ч00мин"],[180000,"0ч03мин"],[3599999,"0ч59мин"],[3600000,"1ч00мин"],[3660000,"1ч01мин"],[90000000,"25ч00мин"],[4294967295,"1193ч02мин"],[null,"—"],[-1,"—"]])assert.equal(formatUptime(ms),expected);
"""],check=True)
assert "get('aboutUptime').textContent=formatUptime(s.uptime_ms);" in s
assert "get('uptime').textContent=formatUptime(s.uptime_ms);" in s
print('PASS uptime minutes, hour rollover and missing data')
