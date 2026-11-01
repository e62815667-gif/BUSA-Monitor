from pathlib import Path
import gzip,re
r=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE');s=(r/'BUSA_Main.h').read_text()
html=s.split('R"HTMLBUSA(')[1].split(')HTMLBUSA"')[0].encode()
h=(r/'BUSA_WebGzip.h').read_text().split('PROGMEM={')[1].split('};')[0]
b=bytes(map(int,re.findall(r'\d+',h)));assert gzip.decompress(b)==html
runtime=(r/'BUSA_V15_Runtime.h').read_text();assert 'Content-Encoding","gzip' in runtime and 'sizeof(BUSA_WEB_GZIP)' in runtime
assert len(b)<len(html)//2
assert 'enableLoopWDT();' not in runtime
print('PASS: lossless compressed app, correct HTTP encoding, recovery without newly added loop watchdog')
