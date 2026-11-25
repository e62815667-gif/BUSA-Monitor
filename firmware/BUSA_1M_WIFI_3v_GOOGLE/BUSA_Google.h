#pragma once
// Google Sheets uploader and authenticated SOUND-threshold configuration polling; no direct relay/GPIO control.
// TLS certificate verification mandatory. Credentials never compiled into public image.
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <new>
#include "BUSA_SnapshotMailbox.h"
// Generated from Google Trust Services root CA certificates in the verified CI build.
#include "BUSA_GoogleCA.h"

volatile bool googleEnabled=false,googleRestart=false,googleTestRequested=false;
volatile uint32_t googleRestartAt=0,googleSent=0;
volatile int googleHttp=0,googleState=0;
// Diagnostics only: no tokens or passwords in logs or /api/status.
volatile int googleDns=-1; // -1 not attempted, 0 failed, 1 resolved
volatile int googleInternet=0; // 1 after a successful HTTPS telemetry response; no extra reachability probes
volatile int googleTlsError=0;
volatile int googleErrorStage=0; // 0 OK/idle, 1 DNS, 2 HTTPS, 3 HTTP

String googleEndpoint,googleSecret;
volatile bool busaGoogleNetworkBusy=false; // Distinguishes HTTPS transactions from AP scans
// Device-initiated HTTPS command polling accepts only verified T1–T4 sound thresholds.
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
// Parse ONLY the fixed Apps Script notification-threshold payload.
// Avoid an additional JSON library and ignore malformed or oversized responses.
static bool busaRemoteParseFour(const String &json,const char *key,float out[4]) {
  const String needle=String("\"")+key+"\":[";
  const int at=json.indexOf(needle);
  if(at<0)return false;
  const char *begin=json.c_str();
  const char *p=begin+at+needle.length();
  for(int i=0;i<4;i++){
    while(*p==' '||*p=='\t')p++;
    char *end=nullptr;
    out[i]=strtof(p,&end);
    if(!end||end==p||!isfinite(out[i]))return false;
    p=end;
    while(*p==' '||*p=='\t')p++;
    if(*p!=(i==3?']':','))return false;
    p++;
  }
  return true;
}
static bool busaRemoteParseCommand(const String &json,BusaRemoteAlarmCommand &out){
  if(json.length()>1200 || json.indexOf("\"ok\":true")<0 ||
     json.indexOf("\"command\":{")<0)return false;
  const int at=json.indexOf("\"id\":\"");
  if(at<0)return false;
  const int p=at+6,finish=json.indexOf('"',p);
  if(finish-p!=24)return false;
  const String id=json.substring(p,finish);
  if(!busaRemoteValidId(id.c_str()))return false;
  memcpy(out.id,id.c_str(),25);
  if(!busaRemoteParseFour(json,"on",out.on)||
     !busaRemoteParseFour(json,"off",out.off))return false;
  for(int i=0;i<4;i++)if(!busaRemoteValidPair(out.on[i],out.off[i]))return false;
  return true;
}
static void busaGooglePollRemoteAlarms(){
  if(!googleEnabled||WiFi.status()!=WL_CONNECTED||busaWifiScanRunning)return;
  // The Google device secret is already stored in ESP32 NVS; users never re-enter it.
  if(!googleTokenValid(googleSecret))return;
  const String url=googleEndpoint+"?mode=device_command&deviceToken="+googleSecret;
  if(url.length()>360)return;
  busaGoogleNetworkBusy=true;
  WiFiClientSecure secure;
  secure.setCACert(GOOGLE_TRUST_ANCHOR);
  secure.setTimeout(7500);
  HTTPClient request;
  request.setConnectTimeout(6500);
  request.setTimeout(7500);
  request.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if(request.begin(secure,url)){
    const int code=request.GET();
    if(code==200){
      const String reply=request.getString();
      BusaRemoteAlarmCommand command;
      if(busaRemoteParseCommand(reply,command))busaRemoteEnqueue(command);
    }
    request.end();
  }
  busaGoogleNetworkBusy=false;
}

struct BusaGoogleStatusSnapshot {uint32_t at;String json;};
static BusaSnapshotMailbox<BusaGoogleStatusSnapshot> busaGoogleSnapshots;
// Called only by the Arduino loop; no live String/model access from Google task.
static void busaGoogleSnapshotPoll(){
  static uint32_t lastAttempt=0;
  const uint32_t now=millis();
  if(lastAttempt&&(uint32_t)(now-lastAttempt)<1000)return;
  if(!busaGoogleSnapshots.needsSnapshot())return;
  lastAttempt=now;
  auto* frame=new(std::nothrow) BusaGoogleStatusSnapshot;
  if(!frame){busaGoogleSnapshots.request();return;}
  frame->at=now;frame->json=statusJSON();
  if(frame->json.length()<2){delete frame;busaGoogleSnapshots.request();return;}
  busaGoogleSnapshots.publish(frame);
}

void googleWorker(void* unused){
  (void)unused;
  // NTP is configured once in setup(); this worker never launches network probes.
  uint32_t last=millis()-30000;
  uint32_t lastRemoteCheck=millis();
  while(true){
    vTaskDelay(pdMS_TO_TICKS(1250));
    // A user's disconnect request blocks NEW Google HTTPS transactions;
    // the current bounded request is allowed to finish before STA goes down.
    if(wifiUiStaDisabled || wifiUiDisablePending){
      googleInternet=0;googleDns=0;
      if(googleEnabled)googleState=1;
      continue;
    }
    if(WiFi.status()!=WL_CONNECTED){
      googleInternet=0;googleDns=0;
      if(googleEnabled)googleState=1;
      continue;
    }
    // One radio serves both the local AP and Google uplink. Do not run HTTPS
    // while a user-requested Wi-Fi scan is switching 2.4 GHz channels.
    if(busaWifiScanRunning)continue;
    if(!googleEnabled)continue;
    if(time(nullptr)<1700000000){googleState=2;continue;}
    const uint32_t now=millis();
    // Poll the protected Google command inbox on a separate bounded interval.
    // A separate HTTPS request never overlaps the telemetry POST or AP scan.
    if((uint32_t)(now-lastRemoteCheck)>=45000 && !googleTestRequested){
      lastRemoteCheck=now;
      busaGooglePollRemoteAlarms();
      continue;
    }
    if(!googleTestRequested && (uint32_t)(now-last)<30000)continue;
    auto* snapshot=busaGoogleSnapshots.take();
    if(!snapshot||(uint32_t)(now-snapshot->at)>5000){
      delete snapshot;busaGoogleSnapshots.request();continue;
    }
    googleTestRequested=false;
    last=now;
    busaGoogleNetworkBusy=true;
    googleState=3;
    // The full read-only status object includes every OneWire diagnostic error,
    // keypad scan_error, I2C, YF-S401 pulse counts and Wi-Fi state.
    // Never save the authentication token inside telemetry/raw sheet rows.
    String packet=String("{\"version\":3,\"kind\":\"monitor_diagnostics\",\"device_id\":\"BUSA-1M-001\",\"sent_epoch\":")
      +String((uint32_t)time(nullptr))
      +",\"telemetry\":"+snapshot->json
      +",\"token\":"+quoteJson(googleSecret)+"}";
    delete snapshot; // Packet owns its JSON copy; the live model stays on main loop.
    // HTTPS already resolves the host. A second DNS lookup and a separate
    // TCP:443 probe caused unnecessary radio traffic every 15 seconds.
    googleDns=-1;googleTlsError=0;googleErrorStage=0;
    WiFiClientSecure secure;
    secure.setCACert(GOOGLE_TRUST_ANCHOR);
    secure.setTimeout(10000);
    HTTPClient request;
    request.setConnectTimeout(8000);
    request.setTimeout(10000);
    int code=-1;
    if(request.begin(secure,googleEndpoint)){
      request.addHeader("Content-Type","application/json");
      code=request.POST((uint8_t*)packet.c_str(),packet.length());
      if(code<0){
        googleInternet=0;
        char tlsText[96]={0};
        googleTlsError=secure.lastError(tlsText,sizeof(tlsText));
        googleErrorStage=2;googleState=6;
        Serial0.printf("GOOGLE TX: HTTPS error %d, TLS code %d (%s)\\n",
                       code,(int)googleTlsError,tlsText);
      }else if(code==200){
        googleInternet=1;googleDns=1;
        const String reply=request.getString();
        googleState=reply.indexOf("\"stored\":true")>=0?5:7;
        googleErrorStage=googleState==5?0:3;
        googleTlsError=0;
      }else if(code==302||code==303){
        googleInternet=1;googleDns=1;
        // Apps Script ContentService redirects after execution. Delivery is
        // not counted as confirmed unless sheet response says stored:true.
        googleState=4;googleErrorStage=0;
      }else{
        googleInternet=0;
        googleState=6;googleErrorStage=3;
        Serial0.printf("GOOGLE TX: HTTP status %d\\n",code);
      }
      request.end();
    }else{
      googleInternet=0;
      googleState=6;googleErrorStage=2;googleTlsError=0;
      Serial0.println("GOOGLE TX: HTTPClient.begin failed");
    }
    googleHttp=code;
    if(googleState==5)googleSent++;
    busaGoogleNetworkBusy=false;
  }
}
static void busaApi_googleConfig(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_googleTest(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Проверка доступна только в сети БУСА");return;
    }
    if(!googleEnabled){
      server.send(409,"text/plain; charset=utf-8","Сначала укажи Apps Script и токен");return;
    }
    googleTestRequested=true;
    server.send(200,"text/plain; charset=utf-8","Запрос отправки принят. Статус появится после попытки HTTPS.");
}

static void busaApi_googleOff(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain","Подтвердите локально");return;
    }
    googleEnabled=false;
    googleTestRequested=false;
    prefs.remove("g-token");
    googleRestart=true;googleRestartAt=millis();
    server.send(200,"text/plain","Отправка отключена; БУСА перезапускается");
}

void addGoogleRoutes(){
  server.on("/api/google/config",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_googleConfig(request);
  });
  server.on("/api/google/test",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_googleTest(request);
  });
  server.on("/api/google/off",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_googleOff(request);
  });
}
void launchGoogleCloud(){
  if(xTaskCreatePinnedToCore(googleWorker,"busa-google",12288,nullptr,1,nullptr,0)!=pdPASS){
    googleEnabled=false;googleState=6;
    Serial0.println("GOOGLE TASK FAILED; LOCAL AP STILL AVAILABLE");
  }
}
