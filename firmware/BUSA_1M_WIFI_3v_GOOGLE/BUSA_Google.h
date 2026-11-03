#pragma once
// Google Sheets uploader and authenticated SOUND-threshold configuration polling; no direct relay/GPIO control.
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
volatile int googleInternet=0; // 1 after a successful HTTPS telemetry response; no extra reachability probes
volatile int googleTlsError=0;
volatile int googleErrorStage=0; // 0 OK/idle, 1 DNS, 2 HTTPS, 3 HTTP

String googleEndpoint,googleSecret;
// Both are stored in ESP32 NVS, never in the publicly built firmware.

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
bool busaGoogleSavedCredentials(){
  return googleURLValid(googleEndpoint)&&googleTokenValid(googleSecret);
}
bool busaGoogleUiToggle(bool enable){
  // Simple reversible switch. Never erase the existing token on OFF.
  if(enable && !busaGoogleSavedCredentials())return false;
  if(prefs.putBool("g-enabled",enable)!=sizeof(bool))return false;
  googleEnabled=enable && busaGoogleSavedCredentials();
  googleTestRequested=false;
  googleState=googleEnabled?1:0;
  if(!googleEnabled){googleInternet=0;googleDns=-1;}
  return true;
}
String cloudJSON(){
 return String("{\"configured\":")+(busaGoogleSavedCredentials()?"true":"false")+
  ",\"enabled\":"+(googleEnabled?"true":"false")+
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
  googleEnabled=busaGoogleSavedCredentials() && prefs.getBool("g-enabled",true);
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
  if(!googleEnabled||WiFi.status()!=WL_CONNECTED||busaWifiScanRunning||
     wifiUiScanPending||wifiUiOwnsScan||busaStaSwitchPending||busaStaSwitching)return;
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

void googleWorker(void* unused){
  (void)unused;
  // NTP is configured once in setup(); this worker never launches network probes.
  uint32_t last=millis()-30000;
  uint32_t lastRemoteCheck=millis();
  while(true){
    vTaskDelay(pdMS_TO_TICKS(1250));
    if(WiFi.status()!=WL_CONNECTED){
      googleInternet=0;googleDns=0;
      if(googleEnabled)googleState=1;
      continue;
    }
    // One radio serves both the local AP and Google uplink. Do not run HTTPS
    // while a user-requested Wi-Fi scan is switching 2.4 GHz channels.
    if(busaWifiScanRunning||wifiUiScanPending||wifiUiOwnsScan||
       busaStaSwitchPending||busaStaSwitching)continue;
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
    googleTestRequested=false;
    last=now;
    busaGoogleNetworkBusy=true;
    googleState=3;
    // The full read-only status object includes every OneWire diagnostic error,
    // keypad scan_error, I2C, PCA9685, YF-S401 pulse counts and Wi-Fi state.
    // Never save the authentication token inside telemetry/raw sheet rows.
    String packet=String("{\"version\":3,\"kind\":\"monitor_diagnostics\",\"device_id\":\"BUSA-1M-001\",\"sent_epoch\":")
      +String((uint32_t)time(nullptr))
      +",\"telemetry\":"+statusJSON()
      +",\"token\":"+quoteJson(googleSecret)+"}";
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
void addGoogleRoutes(){
  server.on("/api/google/config",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain","Настройка только из локальной сети БУСА");return;}
    String url=server.arg("url"),token=server.arg("token");
    url.trim();token.trim();
    if(!googleURLValid(url)||!googleTokenValid(token)){
      server.send(400,"text/plain","Некорректный URL Apps Script или токен");return;
    }
    googleEnabled=false; // Stop new transfers while credentials are changed.
    if(prefs.putString("g-url",url)!=url.length()||
       prefs.putString("g-token",token)!=token.length()){
      server.send(500,"text/plain","Не удалось сохранить");return;
    }
    googleEndpoint=url;googleSecret=token;
    if(!busaGoogleUiToggle(true)){
      server.send(500,"text/plain","Настройки сохранены, включение не удалось");return;
    }
    server.send(200,"text/plain; charset=utf-8","Google сохранён и подключён. Повторный ввод токена не требуется.");
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
  server.on("/api/google/on",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;
    }
    if(!busaGoogleSavedCredentials()){
      server.send(409,"text/plain; charset=utf-8","Первичная настройка Google нужна один раз в приложении");return;
    }
    if(!busaGoogleUiToggle(true)){
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить включение");return;
    }
    server.send(200,"text/plain; charset=utf-8","Передача Google включена, настройки сохранены");
  });
  server.on("/api/google/off",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;
    }
    if(!busaGoogleUiToggle(false)){
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить отключение");return;
    }
    server.send(200,"text/plain; charset=utf-8","Передача Google отключена; URL и токен сохранены");
  });
}
void launchGoogleCloud(){
  if(xTaskCreatePinnedToCore(googleWorker,"busa-google",12288,nullptr,1,nullptr,0)!=pdPASS){
    googleEnabled=false;googleState=6;
    Serial0.println("GOOGLE TASK FAILED; LOCAL AP STILL AVAILABLE");
  }
}
