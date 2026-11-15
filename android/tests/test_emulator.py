"""Run the built APK in an Android emulator against a local ESP32 HTTP fixture.

Requires adb, a booted emulator, a DEBUG APK and Playwright. No real ESP32.
"""
from pathlib import Path
import copy, http.server, json, socketserver, subprocess, sys, threading, urllib.parse

root=Path(__file__).resolve().parents[2]
apk=Path(sys.argv[1]).resolve()
out=root/'android/out/emulator';out.mkdir(parents=True,exist_ok=True)
fixture=json.loads((root/'tests/fixtures/ui10-status.json').read_text())
fixture['collection']={'sensor_present':True,'ml_per_min':24,'total_ml':1234,'duration_ms':3600000}
state={'offline':False,'tick':0,'posts':[]}
html=(root/'android/app/src/main/assets/local/index.html').read_bytes()
def adb(*args,data=None):
    return subprocess.run(['adb',*args],input=data,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def do_GET(self):
        path=urllib.parse.urlsplit(self.path).path
        body=b'{}';kind='application/json';code=200
        if path=='/__test/offline':state['offline']=True
        elif path=='/__test/posts':body=json.dumps(state['posts']).encode()
        elif state['offline']:code=503
        elif path=='/':body=html;kind='text/html; charset=utf-8'
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
    adb('wait-for-device');adb('install','-r',str(apk));adb('reverse','tcp:18080','tcp:18080')
    adb('shell','am','force-stop','ru.busa.monitor')
    adb('shell','run-as','ru.busa.monitor','mkdir','-p','shared_prefs')
    prefs=b'<?xml version="1.0" encoding="utf-8"?><map><string name="address">http://127.0.0.1:18080</string></map>'
    adb('shell','run-as','ru.busa.monitor','tee','shared_prefs/connection.xml',data=prefs)
    adb('shell','am','start','-W','-n','ru.busa.monitor/.MainActivity')
    subprocess.run(['node',str(root/'android/tests/emulator_ui.js')],cwd=root,check=True)

finally:
    server.shutdown()
    (out/'logcat.txt').write_bytes(adb('logcat','-d','-t','2500'))
