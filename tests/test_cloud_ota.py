from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
s=(root/'BUSA_CloudOTA.h').read_text();fn=s[s.index('void cloudInstall(){'):s.index('void cloudOtaWorker')]
stub=r'''
#include <cassert>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include <ctime>
#include <openssl/sha.h>
#include "BUSA_CloudPolicy.h"
#include "BUSA_AppHeader.h"
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define pdMS_TO_TICKS(x) (x)
uint32_t tick=100;uint32_t millis(){return tick++;}void vTaskDelay(int ms){tick+=ms;}
using mbedtls_sha256_context=SHA256_CTX;
void mbedtls_sha256_init(SHA256_CTX*){}void mbedtls_sha256_free(SHA256_CTX*){}
int mbedtls_sha256_starts_ret(SHA256_CTX*c,int){return SHA256_Init(c)==1?0:-1;}
int mbedtls_sha256_update_ret(SHA256_CTX*c,const void*p,size_t n){return SHA256_Update(c,p,n)==1?0:-1;}
int mbedtls_sha256_finish_ret(SHA256_CTX*c,uint8_t*p){return SHA256_Final(p,c)==1?0:-1;}
struct CloudCandidate{char url[256]={0},sha[65]={0},version[32]={0};uint32_t size=0,build=0;}cloudCandidate;
bool slotOK=true;bool cloudPartitionOK(uint32_t){return slotOK;}
int phase=0,errorCode=0;void cloudResult(uint8_t p,int e=0,uint32_t=0){phase=p;errorCode=e;}
const int WL_CONNECTED=3,U_FLASH=0;struct{int status(){return 3;}}WiFi;
std::vector<uint8_t> source;size_t position=0;int httpCode=200,declaredSize=60000;
class WiFiClient{public:int available(){return position<source.size()?int(std::min(size_t(17),source.size()-position)):0;}
int read(uint8_t*p,size_t n){memcpy(p,source.data()+position,n);position+=n;return n;}};
class WiFiClientSecure{public:void stop(){}};
class HTTPClient{public:WiFiClient stream;bool begin(WiFiClientSecure&,const char*){position=0;return true;}void addHeader(const char*,const char*){}int GET(){return httpCode;}int getSize(){return declaredSize;}void end(){}WiFiClient* getStreamPtr(){return &stream;}bool connected(){return position<source.size();}};
void cloudHttpSetup(WiFiClientSecure&,HTTPClient&){}
struct Updater{std::vector<uint8_t> data;int begins=0,ends=0,aborts=0;bool fail=false;
bool begin(size_t,int){++begins;data.clear();return true;}size_t write(uint8_t*p,size_t n){if(fail)return 0;data.insert(data.end(),p,p+n);return n;}bool end(bool b){assert(!b);++ends;return true;}void abort(){++aborts;}}Update;
void reset(){source.assign(60000,0);source[0]=0xe9;source[1]=3;source[12]=9;source[32]=0x32;source[33]=0x54;source[34]=0xcd;source[35]=0xab;
strcpy(cloudCandidate.url,"https://raw.githubusercontent.com/e62815667-gif/BUSA-Monitor/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/releases/cloud/APP.bin");
strcpy(cloudCandidate.version,"V14.17");cloudCandidate.size=60000;cloudCandidate.build=141700;
uint8_t digest[32];SHA256(source.data(),source.size(),digest);for(int i=0;i<32;++i)sprintf(cloudCandidate.sha+2*i,"%02x",digest[i]);
httpCode=200;declaredSize=60000;slotOK=true;Update=Updater{};errorCode=0;phase=0;tick=100;}
'''
tests=r'''
int main(){
 reset();cloudInstall();assert(phase==5&&Update.ends==1&&Update.data==source);
 reset();cloudCandidate.sha[0]=cloudCandidate.sha[0]=='0'?'1':'0';cloudInstall();assert(errorCode==10&&Update.ends==0&&Update.aborts==1);
 reset();source.resize(100);cloudInstall();assert(errorCode==7&&Update.ends==0&&Update.aborts==1);
 reset();source[32]=0;cloudInstall();assert(errorCode==8&&Update.begins==0);
 reset();declaredSize=59999;cloudInstall();assert(errorCode==6&&Update.begins==0);
 reset();httpCode=302;cloudInstall();assert(errorCode==6&&Update.begins==0);
 reset();Update.fail=true;cloudInstall();assert(errorCode==9&&Update.ends==0&&Update.aborts==1);
 reset();slotOK=false;cloudInstall();assert(errorCode==5&&Update.begins==0);
 reset();cloudCandidate.build=BUSA_BUILD;cloudInstall();assert(errorCode==5&&Update.begins==0);
 reset();strcpy(cloudCandidate.url,"http://evil/APP.bin");cloudInstall();assert(errorCode==5&&Update.begins==0);
 reset();assert(busaCloudMetaOK(cloudCandidate.url,cloudCandidate.sha,"V14.17",60000,141700));
 for(auto tail:{"/releases/cloud/../APP.bin","/releases/cloud/APP.bin?x=1","/releases/cloud/APP.bin#x","/releases/cloud/APP.BIN"}){
 std::string url="https://raw.githubusercontent.com/e62815667-gif/BUSA-Monitor/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";url+=tail;
 assert(!busaCloudMetaOK(url.c_str(),cloudCandidate.sha,"V14.17",60000,141700));}
 assert(!busaSlotOK(0x10000,0x140000));assert(busaSlotOK(0x340000,0x330000));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';p.write_text(stub+fn+tests)
 subprocess.run(['g++','-std=c++11','-Wno-deprecated-declarations','-I'+str(root),str(p),'-lcrypto','-o',d+'/test'],check=True)
 subprocess.run([d+'/test'],check=True)
print('PASS cloud installer: actual streaming handler + real SHA256: complete image, fragmented header, hash mismatch, truncation, wrong image, HTTP/size error, write failure, partitions, downgrade, URL confinement')
