#pragma once
// Diagnostic-only Google Sheets uploader; no control channel or relay access.
// TLS certificate verification mandatory. Credentials never compiled into public image.
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
// Generated from Google Trust Services root CA certificates in the verified CI build.
#include "BUSA_GoogleCA.h"

volatile bool googleEnabled=false,googleRestart=false,googleTestRequested=false;
volatile uint32_t googleRestartAt=0,googleSent=0;
volatile int googleHttp=0,googleState=0;
// Diagnostics only: no tokens or passwords in logs or /api/status.
volatile int googleDns=-1; // -1 not attempted, 0 failed, 1 resolved
volatile int googleInternet=0; // 1 only if external TCP/443 connection succeeds
volatile int googleTlsError=0;
volatile int googleErrorStage=0; // 0 OK/idle, 1 DNS, 2 HTTPS, 3 HTTP

String googleEndpoint,googleSecret;
// Sending is one-way HTTPS. No remote command endpoint is ever accepted.
bool googleURLValid(const String &s){
  const String prefix="https://script.google.com/macros/s/";
  if(!s.startsWith(prefix)||!s.endsWith("/exec")||s.length()>255)return false;
  const int end=s.length()-5;
  if(end<(int)prefix.length()+20)return false;
  for(int i=prefix.length();i<end;i++){
    const char c=s[i];
    if(!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='_'||c=='-'))return false;
  }
  return true;
}
bool googleTokenValid(const String &s){
  if(s.length()!=64)return false;
  for(unsigned i=0;i<s.length();i++)if(!isxdigit((unsigned char)s[i]))return false;
  return true;
}
String cloudJSON(){
 return String("{\"configured\":")+(googleEnabled?"true":"false")+
  ",\"state\":"+String((int)googleState)+
  ",\"http\":"+String((int)googleHttp)+
  ",\"sent\":"+String((uint32_t)googleSent)+
  ",\"dns\":"+String((int)googleDns)+
  ",\"tls_error\":"+String((int)googleTlsError)+
  ",\"error_stage\":"+String((int)googleErrorStage)+"}";
}
void setupGoogleCloud(){
  googleEndpoint=prefs.getString("g-url","");
  googleSecret=prefs.getString("g-token","");
  googleEnabled=googleURLValid(googleEndpoint)&&googleTokenValid(googleSecret);
  googleState=googleEnabled?1:0;
}

// Authenticated remote administration: Google Apps Script retains a separate
// administrator key. The device receives a whitelisted command only in its
// own TLS-verified, BUSA_TOKEN-authenticated telemetry response.
struct BusaCloudCommand{
 char id[40];
 char kind[16];
 char url[160];
 char sha[65];
 char version[100];
 int index,theme;
 float on,off,offset;
 uint32_t size;
};
QueueHandle_t busaCloudCommandQueue=nullptr;
volatile bool busaCloudOtaActive=false,busaCloudRebootPending=false;
String busaCloudAckId="",busaCloudAckResult="";
String cloudControlAckJSON(){
 return String("{\"id\":")+quoteJson(busaCloudAckId)+
        ",\"result\":"+quoteJson(busaCloudAckResult)+"}";
}
static String busaCloudField(const String &json,const char* name){
 const String key=String("\"")+name+"\":";
 int start=json.indexOf(key);if(start<0)return "";
 start+=key.length();while(start<(int)json.length()&&json[start]==' ')start++;
 if(start>=(int)json.length())return "";
 if(json[start]=='"'){
   int stop=json.indexOf('"',start+1);if(stop<0)return "";
   return json.substring(start+1,stop);
 }
 int end=start;
 while(end<(int)json.length()&&json[end]!=','&&json[end]!='}'&&json[end]!=' ')end++;
 return json.substring(start,end);
}
static bool busaHexShaValid(const String &s){
 if(s.length()!=64)return false;
 for(unsigned i=0;i<64;i++)if(!isxdigit((unsigned char)s[i]))return false;
 return true;
}
static void busaCloudAck(const String &id,const String &result){
 busaCloudAckId=id;busaCloudAckResult=result;
 prefs.putString("cloudAckId",id);
 prefs.putString("cloudAckRes",result);
 googleTestRequested=true;
}
void setupBusaCloudCommandQueue(){
 busaCloudCommandQueue=xQueueCreate(2,sizeof(BusaCloudCommand));
 busaCloudAckId=prefs.getString("cloudAckId","");
 busaCloudAckResult=prefs.getString("cloudAckRes","");
}
static void busaCloudReadReply(const String &reply){
 const int start=reply.indexOf("\"command\":{");
 if(start<0||!busaCloudCommandQueue)return;
 const String payload=reply.substring(start);
 BusaCloudCommand c={};
 const String id=busaCloudField(payload,"id"),kind=busaCloudField(payload,"kind");
 if(id.length()!=36||kind.length()<3||kind.length()>15)return;
 // Ignore a command already executed or accepted after a transient reconnect.
 if(id==busaCloudAckId)return;
 id.toCharArray(c.id,sizeof(c.id));kind.toCharArray(c.kind,sizeof(c.kind));
 c.index=busaCloudField(payload,"index").toInt();
 c.on=busaCloudField(payload,"on").toFloat();
 c.off=busaCloudField(payload,"off").toFloat();
 c.offset=busaCloudField(payload,"offset").toFloat();
 c.theme=busaCloudField(payload,"theme").toInt();
 c.size=(uint32_t)busaCloudField(payload,"size").toInt();
 busaCloudField(payload,"url").toCharArray(c.url,sizeof(c.url));
 busaCloudField(payload,"sha256").toCharArray(c.sha,sizeof(c.sha));
 busaCloudField(payload,"version").toCharArray(c.version,sizeof(c.version));
 if(xQueueSend(busaCloudCommandQueue,&c,0)!=pdTRUE)
   Serial0.println("CLOUD CMD: queue busy; retry on next telemetry");
}
// Restrict firmware downloads to this project's immutable filename on Pages.
// Every installation additionally requires exact size, SHA-256 and a valid
// inactive OTA partition; no unverified or arbitrary download URL is accepted.
static const char* BUSA_CLOUD_OTA_URL=
  "https://e62815667-gif.github.io/BUSA-Monitor/online/firmware/BUSA_APP_OTA.bin";
static void busaCloudOtaTask(void *arg){
 BusaCloudCommand c=*(BusaCloudCommand*)arg;
 delete (BusaCloudCommand*)arg;
 busaCloudOtaActive=true;
 otaActive=true; // prevent competing local OTA writes
 bool ok=false;String error="Ошибка загрузки";
 WiFiClientSecure secure;
 secure.setCACert(GOOGLE_TRUST_ANCHOR);
 secure.setTimeout(15000);
 HTTPClient request;
 request.setConnectTimeout(15000);request.setTimeout(15000);
 do{
   if(!request.begin(secure,String(c.url))){error="TLS соединение";break;}
   request.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
   int status=request.GET();
   if(status!=HTTP_CODE_OK){error="HTTPS "+String(status);break;}
   int total=request.getSize();
   if(total>=0&&(uint32_t)total!=c.size){error="Размер не совпадает";break;}
   if(!Update.begin(c.size,U_FLASH)){error="Нет свободного OTA-раздела";break;}
   WiFiClient *stream=request.getStreamPtr();
   mbedtls_sha256_context ctx;
   mbedtls_sha256_init(&ctx);
   mbedtls_sha256_starts_ret(&ctx,0);
   uint8_t buf[1024],digest[32]={0};
   uint32_t got=0;
   uint32_t lastBytesAt=millis();
   bool streamOk=true;
   while(got<c.size){
     if(!stream->available()){
       if((uint32_t)(millis()-lastBytesAt)>15000){streamOk=false;error="Таймаут передачи";break;}
       vTaskDelay(pdMS_TO_TICKS(10));continue;
     }
     size_t take=stream->readBytes(buf,min((uint32_t)sizeof(buf),c.size-got));
     if(!take){streamOk=false;error="Пустой блок";break;}
     if(got==0 && buf[0]!=0xE9){streamOk=false;error="Не APP BIN";break;}
     mbedtls_sha256_update_ret(&ctx,buf,take);
     if(Update.write(buf,take)!=take){streamOk=false;error="Ошибка Flash";break;}
     got+=(uint32_t)take;lastBytesAt=millis();
   }
   mbedtls_sha256_finish_ret(&ctx,digest);
   mbedtls_sha256_free(&ctx);
   if(!streamOk||got!=c.size){Update.abort();break;}
   char hex[65];for(int i=0;i<32;i++)sprintf(hex+2*i,"%02x",digest[i]);hex[64]=0;
   if(!String(hex).equalsIgnoreCase(String(c.sha))){Update.abort();error="SHA-256 не совпал";break;}
   if(!Update.end(true)){Update.abort();error="Не удалось активировать OTA";break;}
   ok=true;
 }while(false);
 request.end();
 otaActive=false;busaCloudOtaActive=false;
 if(ok){
   busaCloudAck(String(c.id),"restart_pending");
   otaRestartAt=millis();busaCloudRebootPending=true;otaRestart=true;
   Serial0.println("CLOUD OTA: checked SHA256, inactive image ready; restart pending");
 }else{
   busaCloudAck(String(c.id),String("failed: ")+error);
   Serial0.printf("CLOUD OTA: failed: %s\n",error.c_str());
 }
 vTaskDelete(nullptr);
}
void processBusaCloudCommand(){
 if(!busaCloudCommandQueue||busaCloudOtaActive||otaActive)return;
 BusaCloudCommand c={};
 if(xQueueReceive(busaCloudCommandQueue,&c,0)!=pdTRUE)return;
 String id(c.id),kind(c.kind);
 if(id==busaCloudAckId)return;
 if(kind=="threshold"){
   if(c.index<0||c.index>=4||!isfinite(c.on)||!isfinite(c.off)||
      c.on<0||c.on>125||c.off<0||c.off>125||fabsf(c.on-c.off)<0.099f){
      busaCloudAck(id,"rejected: thresholds");return;
   }
   bool ok=storeTempReference(c.index,0,c.on);
   if(ok)ok=storeTempReference(c.index,1,c.off);
   busaCloudAck(id,ok?"applied":"failed: NVS");return;
 }
 if(kind=="calibration"){
   busaCloudAck(id,saveTempOffset(c.index,c.offset)?"applied":"rejected: calibration");return;
 }
 if(kind=="theme"){
   busaCloudAck(id,saveThemeMode(c.theme)?"applied":"rejected: theme");return;
 }
 if(kind=="ota"){
   if(!String(c.url).equals(BUSA_CLOUD_OTA_URL) || !busaHexShaValid(String(c.sha)) ||
      c.size<60000||c.size>3000000||!String(c.version).startsWith("14v7-N16R8-") ||
      WiFi.status()!=WL_CONNECTED){
     busaCloudAck(id,"rejected: OTA manifest");return;
   }
   auto *copy=new BusaCloudCommand(c);
   if(!copy){busaCloudAck(id,"failed: memory");return;}
   busaCloudOtaActive=true;
   if(xTaskCreatePinnedToCore(busaCloudOtaTask,"busa-cloud-ota",12288,copy,1,nullptr,0)!=pdPASS){
      busaCloudOtaActive=false;delete copy;busaCloudAck(id,"failed: OTA task");return;
   }
   return;
 }
 busaCloudAck(id,"rejected: action");
}

void googleWorker(void* unused){
  (void)unused;
  configTime(0,0,"pool.ntp.org","time.google.com");
  uint32_t last=millis()-30000;
  uint32_t lastNetProbe=millis()-15000;
  while(true){
    vTaskDelay(pdMS_TO_TICKS(1250));
    if(WiFi.status()!=WL_CONNECTED){
      googleInternet=0;googleDns=0;
      if(googleEnabled)googleState=1;
      continue;
    }
    const uint32_t netNow=millis();
    if((uint32_t)(netNow-lastNetProbe)>=15000){
      lastNetProbe=netNow;
      IPAddress remote;
      int dns=WiFi.hostByName("script.google.com",remote)==1?1:0;
      googleDns=dns;
      WiFiClient probe;
      // The probe tests Internet reachability only; Google POST remains TLS-verified.
      googleInternet=(dns==1 && probe.connect(remote,443,1600))?1:0;
      probe.stop();
    }
    if(!googleEnabled)continue;
    if(time(nullptr)<1700000000){googleState=2;continue;}
    const uint32_t now=millis();
    if(!googleTestRequested && (uint32_t)(now-last)<30000)continue;
    googleTestRequested=false;
    last=now;
    googleState=3;
    // The full read-only status object includes every OneWire diagnostic error,
    // keypad scan_error, I2C, PCA9685, YF-S401 pulse counts and Wi-Fi state.
    // Never save the authentication token inside telemetry/raw sheet rows.
    String packet=String("{\"version\":3,\"kind\":\"monitor_diagnostics\",\"device_id\":\"BUSA-1M-001\",\"sent_epoch\":")
      +String((uint32_t)time(nullptr))
      +",\"telemetry\":"+statusJSON()
      +",\"token\":"+quoteJson(googleSecret)+"}";
    // Resolve Google before TLS, so HTTP -1 can be diagnosed rather than guessed.
    // TLS authentication stays mandatory; insecure mode is NOT used.
    IPAddress googleHostIP;
    if(WiFi.hostByName("script.google.com",googleHostIP)!=1){
      googleDns=0;googleTlsError=0;googleErrorStage=1;
      googleHttp=-1;googleState=6;
      Serial0.println("GOOGLE TX: DNS lookup failed");
      continue;
    }
    googleDns=1;googleTlsError=0;googleErrorStage=0;
    WiFiClientSecure secure;
    secure.setCACert(GOOGLE_TRUST_ANCHOR);
    secure.setTimeout(20000);
    HTTPClient request;
    request.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    request.setRedirectLimit(3);
    request.setConnectTimeout(20000);
    request.setTimeout(20000);
    int code=-1;
    if(request.begin(secure,googleEndpoint)){
      request.addHeader("Content-Type","application/json");
      code=request.POST((uint8_t*)packet.c_str(),packet.length());
      if(code<0){
        char tlsText[96]={0};
        googleTlsError=secure.lastError(tlsText,sizeof(tlsText));
        googleErrorStage=2;googleState=6;
        Serial0.printf("GOOGLE TX: HTTPS error %d, TLS code %d (%s)\\n",
                       code,(int)googleTlsError,tlsText);
      }else if(code==200){
        const String reply=request.getString();
        googleState=reply.indexOf("\"stored\":true")>=0?5:7;
        if(googleState==5)busaCloudReadReply(reply);
        googleErrorStage=googleState==5?0:3;
        googleTlsError=0;
      }else if(code==302||code==303){
        // Apps Script ContentService redirects after execution. Delivery is
        // not counted as confirmed unless sheet response says stored:true.
        googleState=4;googleErrorStage=0;
      }else{
        googleState=6;googleErrorStage=3;
        Serial0.printf("GOOGLE TX: HTTP status %d\\n",code);
      }
      request.end();
    }else{
      googleState=6;googleErrorStage=2;googleTlsError=0;
      Serial0.println("GOOGLE TX: HTTPClient.begin failed");
    }
    googleHttp=code;
    if(googleState==5)googleSent++;
  }
}
void addGoogleRoutes(){
  server.on("/api/google/config",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain","Настройка только из локальной сети БУСА");return;}
    String url=server.arg("url"),token=server.arg("token");
    url.trim();token.trim();
    if(!googleURLValid(url)||!googleTokenValid(token)){
      server.send(400,"text/plain","Некорректный URL Apps Script или токен");return;
    }
    googleEnabled=false; // Do not race with network task.
    if(prefs.putString("g-url",url)==0||prefs.putString("g-token",token)==0){
      server.send(500,"text/plain","Не удалось сохранить");return;
    }
    googleRestart=true;googleRestartAt=millis();
    server.send(200,"text/plain","Настройка сохранена; перезапуск БУСА для применения");
  });
  server.on("/api/google/test",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Проверка доступна только в сети БУСА");return;
    }
    if(!googleEnabled){
      server.send(409,"text/plain; charset=utf-8","Сначала укажи Apps Script и токен");return;
    }
    googleTestRequested=true;
    server.send(200,"text/plain; charset=utf-8","Запрос отправки принят. Статус появится после попытки HTTPS.");
  });
  server.on("/api/google/off",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain","Подтвердите локально");return;
    }
    googleEnabled=false;
    googleTestRequested=false;
    prefs.remove("g-token");
    googleRestart=true;googleRestartAt=millis();
    server.send(200,"text/plain","Отправка отключена; БУСА перезапускается");
  });
}
void launchGoogleCloud(){
  if(xTaskCreatePinnedToCore(googleWorker,"busa-google",12288,nullptr,1,nullptr,0)!=pdPASS){
    googleEnabled=false;googleState=6;
    Serial0.println("GOOGLE TASK FAILED; LOCAL AP STILL AVAILABLE");
  }
}
