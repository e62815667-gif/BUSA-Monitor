from pathlib import Path
import gzip
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE')
s=(root/'BUSA_Main.h').read_text();html=s.split('R"HTMLBUSA(')[1].split(')HTMLBUSA"')[0].encode()
b=gzip.compress(html,compresslevel=9,mtime=0)
(root/'BUSA_WebGzip.h').write_text('#pragma once\n// Generated losslessly from WEB_PAGE by tools/generate_webgzip.py.\nconst uint8_t BUSA_WEB_GZIP[] PROGMEM={\n'+',\n'.join(','.join(str(x) for x in b[i:i+32]) for i in range(0,len(b),32))+'\n};\n')
assert gzip.decompress(b)==html
print('HTML bytes:',len(html),'gzip:',len(b))
