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
void googleWorker(void* unused){
  (void)unused;
  configTime(0,0,"pool.ntp.org","time.google.com");
  uint32_t last=millis()-30000;
  while(true){
    vTaskDelay(pdMS_TO_TICKS(1250));
    if(!googleEnabled)continue;
    if(WiFi.status()!=WL_CONNECTED){googleState=1;continue;}
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
