from pathlib import Path
import subprocess, tempfile, sys, os
src=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Google.h').read_text()
declarations=src[src.index('struct BusaCloudRelease'):src.index('String cloudUpdateJSON()')]
parsers=src[src.index('bool cloudShaIsValid('):src.index('// Sending is one-way HTTPS.')]
helpers=declarations+parsers+src[src.index('class GoogleReplyBuffer'):src.index('void googleConfigureHttp')]
roots=([Path(os.environ['BUSA_ARDUINOJSON_SRC'])/'ArduinoJson.h'] if os.getenv('BUSA_ARDUINOJSON_SRC') else list(Path.home().glob('Arduino/libraries/ArduinoJson/src/ArduinoJson.h')))
assert roots, 'ArduinoJson library missing'
stub=r'''
#include <string>
#include <cstring>
#include <cstdint>
#include <cassert>
#include <ArduinoJson.h>
class String: public std::string {
 public: using std::string::string;
 bool startsWith(const char* x) const { return rfind(x,0)==0; }
};
class Stream {
 public: virtual int available()=0;virtual int read()=0;virtual int peek()=0;
 virtual void flush()=0;virtual size_t write(uint8_t)=0;
 virtual size_t write(const uint8_t*,size_t)=0;
};
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)0)
#define portEXIT_CRITICAL(p) ((void)0)
class HTTPClient {
 public: std::string body;int known=-1;bool fail=false;
 int getSize(){return known;}
 int writeToStream(Stream* s){if(fail)return -1;auto n=s->write((const uint8_t*)body.data(),body.size());return n==body.size()?n:-1;}
};
'''
tests=r'''
int main(){
 assert(googleRedirectValid("https://script.googleusercontent.com/macros/echo?key=abc"));
 for(auto s:{"http://script.googleusercontent.com/x","https://script.googleusercontent.com.evil/x","https://script.googleusercontent.com@evil/x","https://accounts.google.com/x","https://script.googleusercontent.com/x\r\nX:y","https://script.googleusercontent.com/x#f","https://script.googleusercontent.com/x\\y"})assert(!googleRedirectValid(s));
 HTTPClient h;
 for(auto s:{"{\"stored\":true}","{ \"stored\" : true }"}){h.body=s;assert(googleStoredReply(h));}
 for(auto s:{"{\"stored\":false}","{\"stored\":\"true\"}","{\"stored\":1}","{\"nested\":{\"stored\":true}}","<html>login</html>","{\"stored\":tru","{}","[]"}){h.body=s;assert(!googleStoredReply(h));}
 h.body=R"release({"stored":true,"release":{"version":"V14.11","target":"ESP32-S3","app_sha256":"1fc280130318c26bda982d48e5b2d2b3b3171c1697e79e7a9ea63b308860cc08","app_size_bytes":1104224,"build_flash_bytes":8388608}})release";
 assert(googleStoredReply(h) && cloudReleaseSnapshot().present && cloudReleaseSnapshot().minor==11 && cloudReleaseSnapshot().flash==8388608 && cloudReleaseSnapshot().size==1104224);
 cloudReleaseData.present=false;
 h.body=R"release({"stored":true,"release":{"version":"V14.11","target":"ESP32-S3","app_sha256":"INVALID","app_size_bytes":1104224,"build_flash_bytes":8388608}})release";
 assert(googleStoredReply(h) && !cloudReleaseSnapshot().present); // bad metadata cannot enable update
 h.body=std::string(2049,'x');assert(!googleStoredReply(h));
 h.body="{\"stored\":true}";h.fail=true;assert(!googleStoredReply(h));h.fail=false;h.known=2049;assert(!googleStoredReply(h));
 GoogleReplyBuffer b;std::string full(2048,'x');assert(b.write((const uint8_t*)full.data(),2048)==2048);assert(b.write('x')==0);assert(b.overflow);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';p.write_text(stub+helpers+tests)
 subprocess.run(['g++','-std=c++11','-I'+str(roots[0].parent),str(p),'-o',d+'/test'],check=True)
 subprocess.run([d+'/test'],check=True)
print('PASS: actual firmware helpers: redirect allowlist, JSON types, malformed/HTML responses, response bounds and read errors')
