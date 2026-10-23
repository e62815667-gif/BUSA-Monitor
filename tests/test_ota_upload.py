from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
s=(root/'BUSA_Main.h').read_text();fn=s[s.index('void uploadOTA(){'):s.index('#include "BUSA_Google.h"')]
stub=r'''
#include <string>
#include <vector>
#include <cstring>
#include <cassert>
#include <cstdint>
class String:public std::string{public:using std::string::string;
bool endsWith(const char*p)const{size_t n=strlen(p);return size()>=n&&compare(size()-n,n,p)==0;}};
const int UPLOAD_FILE_START=0,UPLOAD_FILE_WRITE=1,UPLOAD_FILE_END=2,UPLOAD_FILE_ABORTED=3,U_FLASH=0;
struct HTTPUpload{int status;String filename="test.bin";size_t currentSize=0;uint8_t buf[2048];};
struct Server{HTTPUpload u;HTTPUpload&upload(){return u;}String header(const char*){return "0123456789abcdef";}}server;
int Serial0=0;
struct Updater{
 size_t expected=0;std::vector<uint8_t> data;bool aborted=false;int ends=0,begins=0;
 bool begin(size_t n,int){expected=n;data.clear();aborted=false;++begins;return true;}
 size_t write(uint8_t*p,size_t n){data.insert(data.end(),p,p+n);return n;}
 bool end(bool partial){assert(!partial);++ends;return data.size()==expected;}
 void abort(){aborted=true;}void printError(int){}
}Update;
bool cloudBusy=false;
bool otaWriteAllowed=false,otaActive=false,otaSuccess=false,otaRestart=false,otaUploadCompleted=false;
uint32_t otaBytes=0,otaExpectedBytes=60000,otaTokenIssued=0,otaRestartAt=0;
uint8_t otaPrefix[36];size_t otaPrefixUsed=0;String otaMessage,otaToken="0123456789abcdef";
bool apClient(){return true;}bool wifiScanBusy(){return false;}uint32_t millis(){return 100;}
#include "BUSA_AppHeader.h"
'''
tests=r'''
void start(){otaRestart=false;otaToken="0123456789abcdef";server.u.status=UPLOAD_FILE_START;uploadOTA();assert(otaActive);}
void chunk(std::vector<uint8_t>&v,size_t offset,size_t n){server.u.status=UPLOAD_FILE_WRITE;server.u.currentSize=n;memcpy(server.u.buf,v.data()+offset,n);uploadOTA();}
int main(){
 std::vector<uint8_t> img(60000,0);img[0]=0xe9;img[1]=3;img[12]=9;img[32]=0x32;img[33]=0x54;img[34]=0xcd;img[35]=0xab;
 start();chunk(img,0,1);chunk(img,1,12);chunk(img,13,22);assert(Update.data.empty());
 chunk(img,35,1);assert(Update.data.size()==36);
 for(size_t i=36;i<img.size();){size_t n=std::min(size_t(2048),img.size()-i);chunk(img,i,n);i+=n;}
 server.u.status=UPLOAD_FILE_END;uploadOTA();assert(otaSuccess&&otaRestart&&Update.data==img&&Update.ends==1);
 start();chunk(img,0,64);server.u.status=UPLOAD_FILE_END;uploadOTA();assert(!otaSuccess&&Update.aborted&&Update.ends==1);
 start();img[32]=0;chunk(img,0,36);assert(!otaActive&&Update.aborted&&Update.data.empty());img[32]=0x32;
 start();otaBytes=59999;chunk(img,0,2);assert(!otaActive&&Update.aborted);
 start();server.u.status=UPLOAD_FILE_ABORTED;uploadOTA();assert(!otaActive&&Update.aborted);
 otaToken="bad";int begins=Update.begins;server.u.status=UPLOAD_FILE_START;uploadOTA();assert(!otaActive&&Update.begins==begins);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';p.write_text(stub+fn+tests)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(root),str(p),'-o',d+'/test'],check=True)
 subprocess.run([d+'/test'],check=True)
print('PASS: real OTA upload handler: fragmented header, complete APP, truncation, wrong image, oversize, abort, invalid token')
