"""Run the built APK in an Android emulator against a local ESP32 HTTP fixture.

Requires adb, a booted emulator, a DEBUG APK and Playwright. No real ESP32.
"""
from pathlib import Path
import copy, http.server, json, socketserver, subprocess, sys, threading, time, urllib.parse

root=Path(__file__).resolve().parents[2]
apk=Path(sys.argv[1]).resolve()
out=root/'android/out/emulator';out.mkdir(parents=True,exist_ok=True)
fixture=json.loads((root/'tests/fixtures/ui10-status.json').read_text())
fixture['collection']={'sensor_present':True,'ml_per_min':24,'total_ml':1234,'duration_ms':3600000}
state={'offline':True,'tick':0,'posts':[],'gets':[],'cloud':'fresh'}
html=(root/'android/app/src/main/assets/local/index.html').read_bytes()
# Emulate a pre-Chromium-103 WebView on the device-served page. The APK
# compatibility layer must restore bounded fetches, not invent API responses.
assert b'<head>' in html
html=html.replace(b'<head>', b'<head><script>delete AbortSignal.timeout;document.documentElement.dataset.fixtureDevicePage="1";</script>',1)
def adb(*args,data=None,timeout=30):
    return subprocess.run(['adb',*args],input=data,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,timeout=timeout).stdout
def collect(name,*args):
    try:
        result=subprocess.run(['adb',*args],stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=20)
        (out/name).write_bytes(result.stdout)
        if result.returncode:(out/(name+'.error.txt')).write_bytes(result.stderr)
    except (subprocess.TimeoutExpired,OSError) as error:
        (out/(name+'.error.txt')).write_text(str(error))
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def do_GET(self):
        path=urllib.parse.urlsplit(self.path).path
        state['gets'].append(self.path)
        body=b'{}';kind='application/json';code=200
        if path=='/__test/offline':state['offline']=True
        elif path=='/__test/online':state['offline']=False
        elif path=='/__test/cloud':
            state['cloud']=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)['state'][0]
        elif path=='/__test/posts':body=json.dumps(state['posts']).encode()
        elif path=='/__test/gets':body=json.dumps(state['gets']).encode()
        elif path=='/exec':
            q=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
            if q.get('readToken')!=['a'*64] or state['cloud']=='denied':packet={'ok':False,'error':'Unauthorized'}
            elif state['cloud']=='empty':packet={'ok':True,'latest':None,'history':[]}
            else:
                f=copy.deepcopy(fixture);f['device']='BUSA-1M-001'
                at=int(time.time()*1000)-(180000 if state['cloud']=='stale' else 0)
                packet={'ok':True,'latest':{'at':at,'telemetry':f},'history':[
                    {'at':at-i*30000,'temperatures':[26.4,78.6,96.2,80]} for i in [2,1,0]]}
            body=('busaViewReceive('+json.dumps(packet)+');').encode()
        elif state['offline']:code=503
        elif path=='/':
            time.sleep(6) # Old APK's 4.5-second root-page cutoff would fail here.
            body=html;kind='text/html; charset=utf-8'
        elif path=='/api/wifi/scan':
            body=b'{"scanning":false,"networks":[{"ssid":"BUSA router"},{"ssid":"Phone hotspot"}]}'
        elif path in ['/calculator.js','/calculator.css']:
            body=(root/path[1:]).read_bytes();kind='application/javascript' if path.endswith('.js') else 'text/css'
        elif path=='/api/status':
            f=copy.deepcopy(fixture);state['tick']+=1;f['uptime_ms']=10000+state['tick']*2000
            f['water']['total_l_estimate']=12.32+state['tick']*.04;body=json.dumps(f).encode()
        elif path=='/api/web-events':body=b'{"boot_id":1,"events":[]}'
        elif path=='/api/chart-history':
            f={'ok':True,'minutes':5,'temperature':[[v]*36 for v in [26.4,78.6,96.2,80]],'collection_ml_min':[24]*36,'cooling_lph':[72]*36,'collection_sensor':True,'cooling_calibrated':True}
            body=json.dumps(f).encode()
        else:code=503
        self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length','0')));state['posts'].append(self.path)
        self.send_response(200);self.send_header('Content-Length','2');self.end_headers();self.wfile.write(b'OK')
class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address=True;daemon_threads=True
server=Server(('127.0.0.1',18080),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
try:
    adb('wait-for-device');adb('install','-r',str(apk),timeout=90)
    adb('shell','am','force-stop','ru.busa.monitor')
    adb('shell','run-as','ru.busa.monitor','mkdir','-p','shared_prefs')
    # The emulator's actual host route, not adb reverse / loopback forwarding.
    prefs=b'<?xml version="1.0" encoding="utf-8"?><map><string name="mode">local</string><string name="address">http://10.0.2.2:18080</string></map>'
    adb('shell','run-as','ru.busa.monitor','tee','shared_prefs/connection.xml',data=prefs)
    (out/'local-start.txt').write_bytes(adb('shell','am','start','-W','-n','ru.busa.monitor/.MainActivity'))
    collect('local-start-screen.png','exec-out','screencap','-p')
    collect('local-start-activity.txt','shell','dumpsys','activity','activities')
    collect('webview-provider.txt','shell','dumpsys','webviewupdate')
    collect('startup-logcat.txt','logcat','-d','-t','1000')
    subprocess.run(['node',str(root/'android/tests/emulator_ui.js')],cwd=root,check=True,timeout=240)
    # Mobile-only internet route, with no Wi-Fi connection or local ESP32 API.
    adb('shell','am','force-stop','ru.busa.monitor')
    prefs=b'<?xml version="1.0" encoding="utf-8"?><map><string name="mode">cloud</string><string name="cloud_url">http://10.0.2.2:18080/exec</string><string name="view_token">'+b'a'*64+b'</string></map>'
    adb('shell','run-as','ru.busa.monitor','tee','shared_prefs/connection.xml',data=prefs)
    adb('shell','svc','wifi','disable');adb('shell','svc','data','enable')
    (out/'cloud-start.txt').write_bytes(adb('shell','am','start','-W','-n','ru.busa.monitor/.MainActivity'))
    subprocess.run(['node',str(root/'android/tests/emulator_cloud.js')],cwd=root,check=True,timeout=300)

finally:
    server.shutdown()
    collect('last-screen.png','exec-out','screencap','-p')
    collect('last-activity.txt','shell','dumpsys','activity','activities')
    collect('logcat.txt','logcat','-d','-t','2500')
