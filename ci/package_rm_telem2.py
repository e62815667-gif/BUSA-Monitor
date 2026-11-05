"""Offline-only image packaging/verification. Never connects to a serial device."""
from pathlib import Path
import argparse,hashlib,struct,subprocess,sys,zipfile


def package(build,core,esptool,sketch_dir=None):
    build=Path(build).resolve();core=Path(core).resolve()
    base=build/'BUSA_1M_WIFI_3v_GOOGLE.ino'
    app=build/'BUSA_N16R8_RM_TELEM2_OTA.bin'
    usb=build/'BUSA_N16R8_RM_TELEM2_USB_MERGED.bin'
    app.write_bytes(base.with_suffix('.ino.bin').read_bytes())
    # Preserve the official Arduino bootloader header (QIO menu uses DIO for ROM boot).
    # Keep the compact USB layout; do NOT fill/erase RainMaker fctry.
    subprocess.run([sys.executable,str(esptool),'--chip','esp32s3','merge_bin',
        '--flash_mode','keep','--flash_freq','keep','--flash_size','keep','-o',str(usb),
        '0x0',str(base)+'.bootloader.bin','0x8000',str(base)+'.partitions.bin',
        '0xe000',str(core/'tools/partitions/boot_app0.bin'),'0x10000',str(app)],check=True)
    data=app.read_bytes();merged=usb.read_bytes()
    assert 60000<len(data)<=0x300000
    assert data[0]==0xe9 and struct.unpack_from('<H',data,12)[0]==9
    assert struct.unpack_from('<I',data,32)[0]==0xabcd5432
    assert b'14v9-N16R8-RM-TELEM2-20261105' in data
    assert merged[0x10000:]==data and len(merged)<=0x310000
    for offset,path in [(0,Path(str(base)+'.bootloader.bin')),(0x8000,Path(str(base)+'.partitions.bin')),(0xe000,core/'tools/partitions/boot_app0.bin')]:
        blob=path.read_bytes();assert merged[offset:offset+len(blob)]==blob
    info=subprocess.check_output([sys.executable,str(esptool),'--chip','esp32s3','image_info','--version','2',str(app)],text=True)
    assert 'Checksum:' in info and 'Validation hash:' in info and info.count('(valid)')>=2
    (build/'OTA_IMAGE_INFO.txt').write_text(info)
    partitions=subprocess.check_output([sys.executable,str(core/'tools/gen_esp32part.py'),str(base)+'.partitions.bin'],text=True)
    assert '0x310000' in partitions and 'fctry' in partitions
    (build/'PARTITIONS.csv').write_text(partitions)
    lines=[]
    for path in [app,usb]:
        digest=hashlib.sha256(path.read_bytes()).hexdigest();lines.append(digest+'  '+path.name)
        print(path.name,path.stat().st_size,digest)
    (build/'SHA256SUMS.txt').write_text('\n'.join(lines)+'\n')
    root=Path(__file__).resolve().parents[1];sketch=Path(sketch_dir) if sketch_dir else root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
    try:commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:commit=(root/'SOURCE_COMMIT.txt').read_bytes()
    (build/'SOURCE_COMMIT.txt').write_bytes(commit)
    # Do not republish the legacy .ino.zip snapshot as if it were FIX3 source.
    source_suffixes={'.ino','.h','.hpp','.c','.cpp','.s','.S','.csv'}
    files=sorted(p for p in sketch.iterdir() if p.is_file() and p.suffix in source_suffixes)
    (build/'SOURCE_SHA256SUMS.txt').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in files))
    with zipfile.ZipFile(build/'BUSA_N16R8_RM_TELEM2_SOURCE.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for path in files:archive.write(path,'firmware/'+sketch.name+'/'+path.name)
        for path in sorted((root/'ci').glob('*.py')):archive.write(path,'ci/'+path.name)
        archive.write(root/'.github/workflows/busa-rm-telemetry.yml','.github/workflows/busa-rm-telemetry.yml')
        archive.writestr('SOURCE_COMMIT.txt',commit)
        for path in sorted((root/'tests').rglob('*')):
            if path.is_file() and path.suffix in {'.py','.js','.json','.hpp','.cpp'}:archive.write(path,str(path.relative_to(root)))
        archive.write(root/'index.html','index.html')
        if (root/'docs/RM_TELEM2_BUILD.md').exists():archive.write(root/'docs/RM_TELEM2_BUILD.md','README_BUILD.md')
        archive.write(root/'docs/RM_TELEM1_BUILD.md','docs/RM_TELEM1_BUILD.md')
        archive.write(root/'docs/RM_TELEM1_SDK_EVIDENCE.txt','docs/RM_TELEM1_SDK_EVIDENCE.txt')
    print('PASS APP/USB target, checksums, boundaries, partitions and source snapshot')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--build-dir',required=True);parser.add_argument('--core-dir',required=True);parser.add_argument('--esptool',required=True);parser.add_argument('--sketch-dir')
    args=parser.parse_args();package(args.build_dir,args.core_dir,args.esptool,args.sketch_dir)
