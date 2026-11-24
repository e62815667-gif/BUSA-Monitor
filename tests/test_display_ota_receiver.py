"""Exercise the production receiver with fake flash/NVS/reboot, real SHA-256 via OpenSSL."""
from pathlib import Path
import tempfile, subprocess
root=Path(__file__).resolve().parents[1]
headers={
'Arduino.h':'''#pragma once
#include <stdint.h>
#include <stddef.h>
extern uint32_t clockMs;
inline uint32_t millis(){return clockMs;}
struct ESPStub {int restarts=0;void restart(){++restarts;}};
extern ESPStub ESP;
''',
'Preferences.h':'''#pragma once
#include <vector>
#include <cstring>
extern std::vector<uint8_t> saved;
extern bool nvsFail;
class Preferences {
public:
 bool begin(const char*,bool){return true;}
 size_t getBytesLength(const char*){return saved.size();}
 size_t getBytes(const char*,void* dst,size_t n){if(n!=saved.size())return 0;memcpy(dst,saved.data(),n);return n;}
 size_t putBytes(const char*,const void* p,size_t n){if(nvsFail)return 0;const uint8_t* b=(const uint8_t*)p;saved.assign(b,b+n);return n;}
 bool remove(const char*){saved.clear();return true;}
};
''',
'sdkconfig.h':'#define CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE 1\n',
'esp_ota_ops.h':'''#pragma once
#include <stdint.h>
#include <stddef.h>
using esp_err_t=int;constexpr int ESP_OK=0;
using esp_ota_handle_t=uint32_t;
struct esp_partition_t{uint32_t address,size;};
enum esp_ota_img_states_t{ESP_OTA_IMG_UNDEFINED,ESP_OTA_IMG_PENDING_VERIFY,ESP_OTA_IMG_VALID};
extern esp_partition_t slot0,slot1;
extern const esp_partition_t* running;
extern const esp_partition_t* boot;
extern esp_ota_img_states_t state;
extern bool writeFail,canRollback;
extern int aborted,confirmed,rolledBack;
extern std::vector<uint8_t> flashed;
inline const esp_partition_t* esp_ota_get_running_partition(){return running;}
inline const esp_partition_t* esp_ota_get_next_update_partition(const esp_partition_t*){return running==&slot0?&slot1:&slot0;}
inline esp_err_t esp_ota_get_state_partition(const esp_partition_t*,esp_ota_img_states_t* s){*s=state;return ESP_OK;}
inline esp_err_t esp_ota_begin(const esp_partition_t* p,size_t n,esp_ota_handle_t* h){if(p==running||n>p->size)return 1;flashed.clear();*h=1;return ESP_OK;}
inline esp_err_t esp_ota_write(esp_ota_handle_t,const void* p,size_t n){if(writeFail)return 1;const uint8_t* b=(const uint8_t*)p;flashed.insert(flashed.end(),b,b+n);return ESP_OK;}
inline esp_err_t esp_ota_abort(esp_ota_handle_t){++aborted;return ESP_OK;}
inline esp_err_t esp_ota_end(esp_ota_handle_t){return ESP_OK;}
inline esp_err_t esp_ota_set_boot_partition(const esp_partition_t* p){boot=p;return ESP_OK;}
inline bool esp_ota_check_rollback_is_possible(){return canRollback;}
inline esp_err_t esp_ota_mark_app_valid_cancel_rollback(){++confirmed;state=ESP_OTA_IMG_VALID;return ESP_OK;}
inline esp_err_t esp_ota_mark_app_invalid_rollback_and_reboot(){++rolledBack;return ESP_OK;}
''',
'mbedtls/sha256.h':'''#pragma once
#include <openssl/evp.h>
struct mbedtls_sha256_context{EVP_MD_CTX* ctx;};
inline void mbedtls_sha256_init(mbedtls_sha256_context* p){p->ctx=EVP_MD_CTX_new();}
inline int mbedtls_sha256_starts_ret(mbedtls_sha256_context* p,int){return EVP_DigestInit_ex(p->ctx,EVP_sha256(),nullptr)==1?0:1;}
inline int mbedtls_sha256_update_ret(mbedtls_sha256_context* p,const uint8_t* b,size_t n){return EVP_DigestUpdate(p->ctx,b,n)==1?0:1;}
inline int mbedtls_sha256_finish_ret(mbedtls_sha256_context* p,uint8_t* b){unsigned n=0;return EVP_DigestFinal_ex(p->ctx,b,&n)==1&&n==32?0:1;}
inline void mbedtls_sha256_free(mbedtls_sha256_context* p){EVP_MD_CTX_free(p->ctx);}
'''}
code=r'''
#include <cassert>
#include <vector>
#include <cstring>
#include <Arduino.h>
#include <esp_ota_ops.h>
#include "BUSA_DisplayOtaReceiver.h"
using namespace busa_ota;
using busa_guition::DisplayOtaReceiver;
using busa_uart::Frame;using busa_uart::write32;using busa_uart::read32;
uint32_t clockMs=0;ESPStub ESP;std::vector<uint8_t> saved,flashed;
bool nvsFail=false,writeFail=false,canRollback=true;
int aborted=0,confirmed=0,rolledBack=0;
esp_partition_t slot0{0x10000,slotSize},slot1{0x1d0000,slotSize};
const esp_partition_t* running=&slot0;const esp_partition_t* boot=&slot0;
esp_ota_img_states_t state=ESP_OTA_IMG_VALID;
static std::vector<uint8_t> image(){
 std::vector<uint8_t> p(60001);p[0]=0xe9;p[12]=9;p[32]=0x32;p[33]=0x54;p[34]=0xcd;p[35]=0xab;
 memcpy(p.data()+100,displayFamily,sizeof(displayFamily));return p;
}
static void offerImage(DisplayOtaReceiver& r,const std::vector<uint8_t>& p,bool badHash=false){
 uint8_t offerData[40]={};write32(offerData,123);write32(offerData+4,p.size());unsigned n=0;
 assert(EVP_Digest(p.data(),p.size(),offerData+8,&n,EVP_sha256(),nullptr)==1&&n==32);
 if(badHash)offerData[8]^=1;
 assert(r.reply(Frame{offer,1,offerData,sizeof(offerData)}));
}
static void transfer(DisplayOtaReceiver& r,const std::vector<uint8_t>& p){
 for(uint32_t offset=0;offset<p.size()&&r.receiving();offset+=blockSize){
  uint8_t req[8]={},kind=0,expected=0;size_t count=0;
  assert(r.next(++clockMs,true,true,true,kind,expected,req,count)&&kind==request&&expected==chunk&&count==8);
  assert(read32(req)==123&&read32(req+4)==offset);
  uint8_t packet[8+blockSize];write32(packet,123);write32(packet+4,offset);
  size_t len=chunkLength(p.size(),offset);memcpy(packet+8,p.data()+offset,len);
  assert(r.reply(Frame{chunk,1,packet,len+8}));
 }
}
static void report(DisplayOtaReceiver& r,uint8_t value,bool respond=true){
 uint8_t req[8]={},kind=0,expected=0;size_t count=0;
 assert(r.next(++clockMs,true,true,true,kind,expected,req,count)&&kind==result&&expected==ack&&count==5&&read32(req)==123&&req[4]==value);
 if(respond)assert(r.reply(Frame{ack,1,req,4}));
}
static void reset(){
 running=boot=&slot0;state=ESP_OTA_IMG_VALID;saved.clear();flashed.clear();writeFail=nvsFail=false;clockMs=0;canRollback=true;
}
int main(){
 auto p=image();reset();DisplayOtaReceiver good;good.begin();
 offerImage(good,p);assert(good.receiving());transfer(good,p);
 assert(!good.receiving()&&flashed==p&&boot==&slot1&&!saved.empty());report(good,written);assert(ESP.restarts==1);
 running=&slot1;state=ESP_OTA_IMG_PENDING_VERIFY;DisplayOtaReceiver after;after.begin();
 after.tick(++clockMs,true,false);assert(confirmed==0);after.tick(++clockMs,true,true);assert(confirmed==1);
 report(after,bootOk);assert(saved.empty());
 reset();DisplayOtaReceiver badHash;badHash.begin();offerImage(badHash,p,true);transfer(badHash,p);assert(boot==&slot0&&saved.empty());report(badHash,failed);
 reset();DisplayOtaReceiver badWrite;badWrite.begin();offerImage(badWrite,p);writeFail=true;transfer(badWrite,p);assert(boot==&slot0);report(badWrite,failed);
 reset();DisplayOtaReceiver badNvs;badNvs.begin();offerImage(badNvs,p);nvsFail=true;transfer(badNvs,p);assert(boot==&slot0&&saved.empty());report(badNvs,failed);
 reset();DisplayOtaReceiver partial;partial.begin();offerImage(partial,p);partial.timeout(chunk);assert(boot==&slot0&&!partial.receiving());report(partial,failed);
 reset();DisplayOtaReceiver first;first.begin();offerImage(first,p);transfer(first,p);assert(boot==&slot1);
 running=&slot1;state=ESP_OTA_IMG_PENDING_VERIFY;DisplayOtaReceiver failedBoot;failedBoot.begin();
 failedBoot.tick(clockMs+30000,true,false);assert(rolledBack==1);
 running=&slot0;state=ESP_OTA_IMG_VALID;DisplayOtaReceiver rolled;rolled.begin();
 uint8_t req[8]={},kind=0,expected=0;size_t count=0;
 assert(!rolled.next(++clockMs,false,true,true,kind,expected,req,count));
 report(rolled,bootFailed);assert(saved.empty());
 reset();state=ESP_OTA_IMG_UNDEFINED;canRollback=false;DisplayOtaReceiver initial;initial.begin();
 const int before=confirmed;initial.tick(++clockMs,true,false);assert(confirmed==before+1&&state==ESP_OTA_IMG_VALID);
}
'''
with tempfile.TemporaryDirectory() as d:
    d=Path(d)
    for name,content in headers.items():
        p=d/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(content)
    p=d/'test.cpp';p.write_text(code);exe=d/'test'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(d),'-I'+str(root/'firmware/common/src'),'-I'+str(root/'firmware/BUSA_Display_N4R8'),str(p),'-lcrypto','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS production display OTA receiver: SHA-256, alternate partition, write/NVS failures, timeout, boot confirmation and rollback (mock flash)')
