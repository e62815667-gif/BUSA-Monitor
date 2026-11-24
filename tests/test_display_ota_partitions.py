from pathlib import Path
import csv, io, sys
p=Path(sys.argv[1]); rows=list(csv.reader(line for line in p.read_text().splitlines() if line.strip() and not line.startswith('#')))
parts={r[0]:(r[1],r[2],int(r[3],0),int(r[4],0)) for r in rows}
assert len(parts)==len(rows)
assert parts['nvs']==('data','nvs',0x9000,0x5000)
assert parts['otadata']==('data','ota',0xe000,0x2000)
assert 'app1' in parts, 'second OTA partition missing'
assert parts['app0']==('app','ota_0',0x10000,0x1c0000)
assert parts['app1']==('app','ota_1',0x1d0000,0x1c0000)
last=0x9000
for name,(kind,sub,offset,size) in sorted(parts.items(), key=lambda x:x[1][2]):
    assert size>0 and offset>=last and offset%0x1000==0 and size%0x1000==0
    if kind=='app': assert offset%0x10000==0
    last=offset+size
assert last<=0x400000
print('PASS N4R8: two equal OTA slots, alignment, no overlap, NVS preserved, 4 MiB boundary')
