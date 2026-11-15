"""Build a signed standalone APK with the official SDK; no Gradle/dependencies.

Example: BUSA_KEYSTORE_PASS=... python3 android/build.py --sdk /path/to/sdk
         --keystore /private/busa-release.jks --out /path/to/output
The signing key is supplied by the caller and never copied into the APK.
"""
from pathlib import Path
import argparse, hashlib, json, os, shutil, subprocess, sys, zipfile

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--sdk',default=os.environ.get('ANDROID_HOME') or os.environ.get('ANDROID_SDK_ROOT'))
parser.add_argument('--keystore',required=True)
parser.add_argument('--alias',default='busa')
parser.add_argument('--out',default=str(root/'android/out'))
parser.add_argument('--debug',action='store_true')
args=parser.parse_args()
assert args.sdk,'Pass --sdk or set ANDROID_HOME'
assert os.environ.get('BUSA_KEYSTORE_PASS'),'BUSA_KEYSTORE_PASS must be set (never printed)'
sdk=Path(args.sdk).resolve();tools=sdk/'build-tools/35.0.0';android=sdk/'platforms/android-35/android.jar'
out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=True)
work=out/'intermediate';shutil.rmtree(work,ignore_errors=True);work.mkdir()
src=root/'android/app/src/main'
def run(*command):subprocess.run([str(p) for p in command],check=True)
def tool(name):return tools/(name+('.exe' if os.name=='nt' else ''))
run(sys.executable,root/'ci/generate_android_assets.py','--check')
manifest=(src/'AndroidManifest.xml').read_text()
if args.debug:manifest=manifest.replace('android:debuggable="false"','android:debuggable="true"')
(work/'AndroidManifest.xml').write_text(manifest)
generated=work/'generated';generated.mkdir()
run(tool('aapt2'),'compile','--dir',src/'res','-o',work/'resources.zip')
run(tool('aapt2'),'link','-I',android,'--manifest',work/'AndroidManifest.xml',
    '--java',generated,'-A',src/'assets','-o',work/'resources.apk',work/'resources.zip')
classes=work/'classes';classes.mkdir()
sources=sorted((src/'java').rglob('*.java'))+sorted(generated.rglob('*.java'))
# The installed Java runtime may expose jdk.compiler without a javac launcher.
run('java','com.sun.tools.javac.Main','-source','8','-target','8','-encoding','UTF-8',
    '-classpath',android,'-d',classes,*sources)
dex=work/'dex';dex.mkdir()
run('java','-cp',tools/'lib/d8.jar','com.android.tools.r8.D8',
    '--debug' if args.debug else '--release','--min-api','23','--lib',android,
    '--output',dex,*sorted(classes.rglob('*.class')))
unsigned=work/'unsigned.apk';shutil.copyfile(work/'resources.apk',unsigned)
with zipfile.ZipFile(unsigned,'a',zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(dex.glob('*.dex')):archive.write(path,path.name)
run(tool('zipalign'),'-p','-f','4',unsigned,work/'aligned.apk')
name='BUSA_Android_1.0.0_UI16'+('_DEBUG' if args.debug else '')+'.apk'
apk=out/name
signer=['java','-jar',str(tools/'lib/apksigner.jar')]
run(*signer,'sign','--ks',Path(args.keystore).resolve(),'--ks-key-alias',args.alias,
    '--ks-pass','env:BUSA_KEYSTORE_PASS','--key-pass','env:BUSA_KEYSTORE_PASS',
    '--out',apk,work/'aligned.apk')
verify=subprocess.check_output(signer+['verify','--verbose','--print-certs',str(apk)],text=True)
(out/'APK_SIGNATURE.txt').write_text(verify)
assert 'Verified using v2 scheme (APK Signature Scheme v2): true' in verify
badging=subprocess.check_output([str(tool('aapt2')),'dump','badging',str(apk)],text=True)
(out/'APK_MANIFEST.txt').write_text(badging)
assert "package: name='ru.busa.monitor'" in badging
assert "minSdkVersion:'23'" in badging and "targetSdkVersion:'35'" in badging
assert ("application-debuggable" in badging)==args.debug
with zipfile.ZipFile(apk) as archive:
    for path in sorted((src/'assets').rglob('*')):
        if path.is_file():assert archive.read('assets/'+str(path.relative_to(src/'assets')))==path.read_bytes()
digest=hashlib.sha256(apk.read_bytes()).hexdigest()
(out/'SHA256SUMS.txt').write_text(digest+'  '+name+'\n')
(out/'BUILD.json').write_text(json.dumps({'package':'ru.busa.monitor','version':'1.0.0-UI16',
    'debug':args.debug,'apk':name,'bytes':apk.stat().st_size,'sha256':digest,
    'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()},indent=2)+'\n')
print('PASS signed APK, SDK versions, embedded UI bytes and signature\n'+str(apk)+'\nSHA256 '+digest)
