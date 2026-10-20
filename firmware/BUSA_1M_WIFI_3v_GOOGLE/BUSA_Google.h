#pragma once
// Diagnostic-only Google Sheets uploader; no control channel or relay access.
// TLS certificate verification mandatory. Credentials never compiled into public image.
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
const char GOOGLE_TRUST_ANCHOR[] PROGMEM=R"GOOGLECERT(
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo
27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w
Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw
TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl
qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH
szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8
Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk
MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92
wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p
aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN
VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb
C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe
QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4
7HLwEXWdyzRSjeZ2axfG34arJ45JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
-----END CERTIFICATE-----
)GOOGLECERT";
volatile bool googleEnabled=false,googleRestart=false;
volatile uint32_t googleRestartAt=0,googleSent=0;
volatile int googleHttp=0,googleState=0;
String googleEndpoint,googleSecret;
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
  ",\"sent\":"+String((uint32_t)googleSent)+"}";
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
    if((uint32_t)(now-last)<30000)continue;
    last=now;
    googleState=3;
    String packet=String("{\"version\":2,\"kind\":\"monitor\",\"device_id\":\"BUSA-1M-001\",\"firmware\":")
      +quoteJson(String(VERSION))
      +",\"uptime_ms\":"+String(now)
      +",\"wifi_rssi_dbm\":"+String(WiFi.RSSI())
      +",\"free_heap_bytes\":"+String(ESP.getFreeHeap())
      +",\"temperatures\":[";
    for(int i=0;i<4;i++){if(i)packet+=",";packet+=hasTemp(i,now)?String(temperature[i],2):String("null");}
    packet+="],\"cooling_lpm\":";
    packet+=(flowSeen&&flowReady)?String(flowLpm,3):String("null");
    packet+=",\"cooling_pulses\":"+String(flowPulses());
    packet+=",\"relays_enabled\":false,\"token\":"+quoteJson(googleSecret)+"}";
    WiFiClientSecure secure;
    secure.setCACert(GOOGLE_TRUST_ANCHOR);
    secure.setTimeout(10000);
    HTTPClient request;request.setTimeout(10000);
    int code=-1;
    if(request.begin(secure,googleEndpoint)){
      request.addHeader("Content-Type","application/json");
      code=request.POST((uint8_t*)packet.c_str(),packet.length());
      // ContentService often redirects to script.googleusercontent.com.
      // 302 only proves delivery to Google's front-end, not database insertion.
      if(code==200){
        const String reply=request.getString();
        googleState=reply.indexOf("\"stored\":true")>=0?5:7;
      }else if(code==302||code==303){googleState=4;}
      else googleState=6;
      request.end();
    }else googleState=6;
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
  server.on("/api/google/off",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain","Подтвердите локально");return;
    }
    googleEnabled=false;
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
