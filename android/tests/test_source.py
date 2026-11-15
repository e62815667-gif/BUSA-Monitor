"""Native policy and UI parity; no sensor values or fake API inside the APK."""
from pathlib import Path
import hashlib, json, os, subprocess, tempfile, xml.etree.ElementTree as E
root=Path(__file__).resolve().parents[2]
subprocess.run(['python3',str(root/'ci/generate_android_assets.py'),'--check'],check=True)
with tempfile.TemporaryDirectory() as tmp:
    subprocess.run(['java','com.sun.tools.javac.Main','-d',tmp,
        str(root/'android/app/src/main/java/ru/busa/monitor/DeviceAddress.java'),
        str(root/'android/tests/DeviceAddressTest.java')],check=True)
    subprocess.run(['java','-cp',tmp,'ru.busa.monitor.DeviceAddressTest'],check=True)
sdk=Path(os.environ.get('ANDROID_HOME') or '/workspace/work/android-sdk')
platform=sdk/'platforms/android-35/android.jar'
if platform.exists():
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(['java','com.sun.tools.javac.Main','-cp',str(platform),'-d',tmp,
            str(root/'android/app/src/main/java/ru/busa/monitor/DeviceAddress.java'),
            str(root/'android/app/src/main/java/ru/busa/monitor/CloudClient.java'),
            str(root/'android/tests/CloudClientPolicyTest.java')],check=True)
        subprocess.run(['java','-ea','-cp',tmp+os.pathsep+str(platform),'ru.busa.monitor.CloudClientPolicyTest'],check=True)
manifest=E.parse(root/'android/app/src/main/AndroidManifest.xml').getroot()
name='{http://schemas.android.com/apk/res/android}name'
assert {e.attrib[name] for e in manifest.findall('uses-permission')}=={
    'android.permission.INTERNET','android.permission.ACCESS_NETWORK_STATE','android.permission.CHANGE_NETWORK_STATE'}
main=(root/'android/app/src/main/java/ru/busa/monitor/MainActivity.java').read_text()
assert 'addJavascriptInterface' not in main
assert 'onReceivedSslError' not in main # Default SSL error rejection must stay intact.
assert 'startActivityForResult(intent, FILE_REQUEST)' in main
print('PASS Android manifest permissions, no native JS bridge or SSL bypass, document picker retained')
