#pragma once
// Diagnostic-only Google Sheets uploader; no control channel or relay access.
// TLS certificate verification mandatory. Credentials never compiled into public image.
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>
// Generated from Google Trust Services root CA certificates in the verified CI build.
#include "BUSA_GoogleCA.h"

volatile bool googleEnabled=false,googleRestart=false,googleTestRequested=false;
volatile uint32_t googleRestartAt=0,googleSent=0;
volatile int googleHttp=0,googleState=0;
// Diagnostics only: no tokens or passwords in logs or /api/status.
volatile int googleDns=-1; // -1 not attempted, 0 failed, 1 resolved
volatile int googleInternet=0; // 1 only if external TCP/443 connection succeeds
volatile int googleTlsError=0;
volatile int googleErrorStage=0; // 0 OK/idle, 1 DNS, 2 HTTPS, 3 HTTP/response, 4 redirect, 5 packet size

String googleEndpoint,googleSecret;
// Cloud update PRE-FLIGHT: metadata only, no installer, no flash writes.
// Fixed-size integral fields avoid sharing dynamic Strings between FreeRTOS tasks.
volatile int cloudReleaseMinor=-1;
volatile uint32_t cloudReleaseFlash=0,cloudReleaseSize=0;
volatile bool cloudReleaseShaValid=false,cloudReleasePresent=false,cloudReleaseTargetValid=false;
String cloudUpdateJSON(){
  const int minor=cloudReleaseMinor;
  return String("{\"catalog_seen\":")+(cloudReleasePresent?"true":"false")+
    ",\"available_version\":"+(minor>=0?quoteJson(String("V14.")+String(minor)):String("null"))+
    ",\"target_valid\":"+(cloudReleaseTargetValid?"true":"false")+
    ",\"sha_format_valid\":"+(cloudReleaseShaValid?"true":"false")+
    ",\"build_flash_bytes\":"+String((uint32_t)cloudReleaseFlash)+
    ",\"app_size_bytes\":"+String((uint32_t)cloudReleaseSize)+
    ",\"device_flash_bytes\":"+String((uint32_t)ESP.getFlashChipSize())+
    ",\"flash_size_matches\":"+(cloudReleasePresent&&cloudReleaseFlash==ESP.getFlashChipSize()?"true":"false")+
    ",\"remote_install_enabled\":false,\"ota_partition_verified\":false}";
}
bool cloudShaIsValid(const char* hex){
  if(!hex || strlen(hex)!=64)return false;
  for(int i=0;i<64;i++)if(!isxdigit((unsigned char)hex[i]))return false;
  return true;
}
void cloudReleaseFromReply(JsonVariantConst obj){
  if(!obj.is<JsonObjectConst>())return;
  const char* version=obj["version"]|"";
  const char* target=obj["target"]|"";
  const char* sha=obj["app_sha256"]|"";
  uint32_t size=obj["app_size_bytes"]|0;
  uint32_t flash=obj["build_flash_bytes"]|0;
  if(strncmp(version,"V14.",4)!=0)return;
  const char* digits=version+4;
  if(!*digits || strlen(digits)>3)return;
  int minor=0;
  while(*digits){if(*digits<'0'||*digits>'9')return;minor=minor*10+(*digits++-'0');}
  if(size<60000||size>4000000 || (flash!=8388608UL&&flash!=16777216UL))return;
  cloudReleaseTargetValid=strcmp(target,"ESP32-S3")==0;
  cloudReleaseShaValid=cloudShaIsValid(sha);
  if(!cloudReleaseTargetValid||!cloudReleaseShaValid)return;
  cloudReleaseMinor=minor;cloudReleaseSize=size;cloudReleaseFlash=flash;cloudReleasePresent=true;
}
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
  ",\"error_stage\":"+String((int)googleErrorStage)+",\"cloud_update\":"+cloudUpdateJSON()+"}";
}
// Limit the decoded response, including chunked HTTP bodies.
class GoogleReplyBuffer : public Stream {
public:
  char data[2049] = {};
  size_t used=0;
  bool overflow=false;
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t b) override { return write(&b,1); }
  size_t write(const uint8_t* src,size_t n) override {
    if(n>2048-used){overflow=true;return 0;}
    memcpy(data+used,src,n);used+=n;data[used]=0;return n;
  }
};
bool googleRedirectValid(const String& url){
  // Exact HTTPS authority. Never follow sign-in, lookalike or HTTP URLs.
  if(!url.startsWith("https://script.googleusercontent.com/") || url.length()>4096)return false;
  for(size_t i=0;i<url.length();i++){
    unsigned char c=url[i];
    if(c<=32 || c>=127 || c=='\\' || c=='#')return false;
  }
  return true;
}
bool googleStoredReply(HTTPClient& request){
  if(request.getSize()>2048)return false;
  GoogleReplyBuffer body;
  if(request.writeToStream(&body)<0 || body.overflow || !body.used)return false;
  StaticJsonDocument<1024> doc;
  DeserializationError err=deserializeJson(doc,body.data,body.used);
  if(err || !doc.is<JsonObject>() || !doc["stored"].is<bool>())return false;
  const bool confirmed=doc["stored"].as<bool>();
  if(confirmed)cloudReleaseFromReply(doc["release"].as<JsonVariantConst>());
  return confirmed;
}
void googleConfigureHttp(HTTPClient& request){
  request.setConnectTimeout(20000);
  request.setTimeout(20000); // HTTPClient uses milliseconds.
  request.setReuse(false);
  request.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
}
void setupGoogleCloud(){
  googleEndpoint=prefs.getString("g-url","");
  googleSecret=prefs.getString("g-token","");
  googleEnabled=googleURLValid(googleEndpoint)&&googleTokenValid(googleSecret);
  googleState=googleEnabled?1:0;
}
// Only loop() builds statusJSON: mutable UI Strings and sensor state are not
// safe to traverse from the networking core. Hold the lock only to copy.
SemaphoreHandle_t googleSnapshotMutex=nullptr;
String googleSnapshot;
uint32_t googleSnapshotAt=0;
void publishGoogleSnapshot(uint32_t now){
  if(!googleEnabled||otaActive||!googleSnapshotMutex)return;
  if(googleSnapshotAt && !elapsed(now,googleSnapshotAt,2000))return;
  String next=statusJSON();
  if(xSemaphoreTake(googleSnapshotMutex,0)==pdTRUE){
    googleSnapshot=next;googleSnapshotAt=now;
    xSemaphoreGive(googleSnapshotMutex);
  }
}
String copyGoogleSnapshot(){
  String result;
  if(googleSnapshotMutex && xSemaphoreTake(googleSnapshotMutex,pdMS_TO_TICKS(20))==pdTRUE){
    result=googleSnapshot;
    xSemaphoreGive(googleSnapshotMutex);
  }
  return result;
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
    if(otaActive)continue;
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
    if(!googleEnabled || otaActive)continue;
    if(time(nullptr)<1700000000){googleState=2;continue;}
    const uint32_t now=millis();
    if(!googleTestRequested && (uint32_t)(now-last)<30000)continue;
    String snapshot=copyGoogleSnapshot();
    if(snapshot.isEmpty())continue;
    googleTestRequested=false;
    last=now;
    googleState=3;
    // The full read-only status object includes every OneWire diagnostic error,
    // keypad scan_error, I2C, PCA9685, YF-S401 pulse counts and Wi-Fi state.
    // Never save the authentication token inside telemetry/raw sheet rows.
    String packet=String("{\"version\":3,\"kind\":\"monitor_diagnostics\",\"device_id\":\"BUSA-1M-001\",\"sent_epoch\":")
      +String((uint32_t)time(nullptr))
      +",\"telemetry\":"+snapshot
      +",\"token\":"+quoteJson(googleSecret)+"}";
    if(packet.length()>12000){googleState=7;googleHttp=0;googleErrorStage=5;continue;}
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
    secure.setTimeout(20); // WiFiClientSecure 2.0.17 uses seconds.
    secure.setHandshakeTimeout(20);
    HTTPClient request;
    googleConfigureHttp(request);
    int code=-1;
    if(request.begin(secure,googleEndpoint)){
      const char* headers[]={"Location"};
      request.collectHeaders(headers,1);
      request.addHeader("Content-Type","application/json");
      code=request.POST((uint8_t*)packet.c_str(),packet.length());
      // ContentService executes POST once, then serves its result via GET.
      // Never replay the telemetry/token to the redirected host.
      for(unsigned hop=0;(code==302 || code==303) && hop<3;hop++){
        String next=request.header("Location");
        if(!googleRedirectValid(next)){googleErrorStage=4;break;}
        request.end();secure.stop();
        googleConfigureHttp(request);
        if(!request.begin(secure,next)){code=-1;break;}
        request.collectHeaders(headers,1);
        code=request.GET();
      }
      if(code<0){
        char tlsText[96]={0};
        googleTlsError=secure.lastError(tlsText,sizeof(tlsText));
        googleErrorStage=2;googleState=6;
        Serial0.printf("GOOGLE TX: HTTPS error %d, TLS code %d (%s)\n",
                       code,(int)googleTlsError,tlsText);
      }else if(code==200){
        googleState=googleStoredReply(request)?5:7;
        googleErrorStage=googleState==5?0:3;
        googleTlsError=0;
      }else if(code==302||code==303){
        // Unsafe, missing or excessive redirects are unconfirmed delivery.
        googleState=4;googleErrorStage=4;
      }else{
        googleState=6;googleErrorStage=3;
        Serial0.printf("GOOGLE TX: HTTP status %d\n",code);
      }
      request.end();
    }else{
      googleState=6;googleErrorStage=2;googleTlsError=0;
      Serial0.println("GOOGLE TX: HTTPClient.begin failed");
    }
    googleHttp=code;
    if(googleState==5)googleSent++;
    last=millis(); // Avoid immediate new POST after a slow timeout.
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
  googleSnapshotMutex=xSemaphoreCreateMutex();
  if(!googleSnapshotMutex){googleEnabled=false;googleState=6;return;}
  if(xTaskCreatePinnedToCore(googleWorker,"busa-google",12288,nullptr,1,nullptr,0)!=pdPASS){
    googleEnabled=false;googleState=6;
    Serial0.println("GOOGLE TASK FAILED; LOCAL AP STILL AVAILABLE");
  }
}


