#pragma once
extern volatile int googleState,googleInternet,googleHttp;
// Shared history and events, independent of display rendering.
static float v13History[4][48];
static uint8_t v13HistoryCount=0,v13HistoryHead=0;
static uint32_t v13LastCapture=0;
static String v13Events[7];
static uint8_t v13EventCount=0;
static bool v13EventBoot=false,v13OldWifi=false,v13OldPcf=false,v13OldTemps[4]={false,false,false,false},v13OldFlow=false;
static void v13Log(const String &msg){
  if(v13EventCount<7)v13EventCount++;
  for(int i=(int)v13EventCount-1;i>0;i--)v13Events[i]=v13Events[i-1];
  String stamp=clockStamp(true);
  v13Events[0]=(deviceClockValid()?stamp.substring(stamp.length()-5):String(millis()/60000)+" мин")+" · "+msg;
}
static uint32_t v13LastCollectAt=0;
static void v13Collect(uint32_t now){
  // The display polls at 5 Hz; histories/event state need at most 1 Hz.
  // Keep front-panel responsiveness without repeated Strings and Wi-Fi queries.
  if(v13LastCollectAt && !elapsed(now,v13LastCollectAt,1000))return;
  v13LastCollectAt=now;
  if(!v13EventBoot){
    v13EventBoot=true;
    v13Log("Устройство включено");
    v13OldWifi=WiFi.status()==WL_CONNECTED;
    v13OldPcf=pcfDetected;
    for(int i=0;i<4;i++)v13OldTemps[i]=tempPresent[i];
    v13OldFlow=flowSeen;
  }
  const bool wifi=WiFi.status()==WL_CONNECTED;
  if(wifi!=v13OldWifi){v13Log(wifi?"Wi-Fi STA подключен":"Wi-Fi STA отключен");v13OldWifi=wifi;}
  if(pcfDetected!=v13OldPcf){v13Log(pcfDetected?"PCF8575 найден":"PCF8575 недоступен");v13OldPcf=pcfDetected;}
  for(int i=0;i<4;i++){
    if(tempPresent[i]!=v13OldTemps[i]){
      v13Log(String("T")+String(i+1)+(tempPresent[i]?" обнаружен":" не отвечает"));
      v13OldTemps[i]=tempPresent[i];
    }
  }
  if(flowSeen!=v13OldFlow){v13Log("YF-S401: импульсы воды");v13OldFlow=flowSeen;}
  if(v13HistoryCount==0 || elapsed(now,v13LastCapture,2000)){
    // Keep the 2s schedule anchored; a slow loop must not accumulate drift.
    if(v13HistoryCount==0)v13LastCapture=now;
    else v13LastCapture+=((uint32_t)(now-v13LastCapture)/2000u)*2000u;
    busaCaptureChart(now);
    for(int i=0;i<4;i++)v13History[i][v13HistoryHead]=hasTemp(i,now)?temperature[i]:NAN;
    v13HistoryHead=(v13HistoryHead+1)%48;
    if(v13HistoryCount<48)v13HistoryCount++;
  }
}
