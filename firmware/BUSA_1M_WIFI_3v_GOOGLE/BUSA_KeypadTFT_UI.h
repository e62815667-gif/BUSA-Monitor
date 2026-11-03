#pragma once
// BUSA 14.8. Physical 20-key navigation + TFT Wi-Fi scan/password.
// Key-to-PCF8575 pairs remain in original NVS "busa-r04"/"keys20"
// with backup "busa-key20"/"keys20"; no retraining at OTA.
enum BusaWifiUiStage : uint8_t { WIFIUI_STATUS=0, WIFIUI_SCANNING=1,
  WIFIUI_NETWORKS=2, WIFIUI_PASSWORD=3, WIFIUI_CONNECTING=4 };
BusaWifiUiStage wifiUiStage=WIFIUI_STATUS;
String wifiUiNetworks[16],wifiUiSelectedSsid="",wifiUiPassword="";
int wifiUiSignals[16]={0};
uint8_t wifiUiCount=0,wifiUiIndex=0,wifiUiMode=0;
uint8_t wifiUiLastTapKey=255,wifiUiLastTapIndex=0;
uint32_t wifiUiLastTapAt=0,wifiUiScanAt=0,wifiUiConnectAt=0;
uint8_t wifiUiCursor=0;
bool wifiUiOwnsScan=false,wifiUiConnectPending=false;
String wifiUiFeedback="";
extern volatile bool busaWifiScanRunning,busaGoogleNetworkBusy;
extern uint32_t busaWifiScanLastAt,busaWifiScanStartedAt,lastStationCheck;
extern String stationSsid,stationPass;
const char* wifiUiModeCaption(){static const char* modes[]={"abc","ABC","123","SYM"};return modes[wifiUiMode%4];}
String wifiUiPasswordMask(){
  String hidden="";
  for(unsigned i=0;i<wifiUiPassword.length() && i<63;i++)hidden+="*";
  return hidden;
}
String wifiUiPasswordPreview(){
  String v=wifiUiPasswordMask();
  if(wifiUiLastTapKey!=255 && wifiUiCursor>0 &&
     (uint32_t)(millis()-wifiUiLastTapAt)<1300 &&
     wifiUiCursor<=v.length()){
    v.setCharAt(wifiUiCursor-1,wifiUiPassword.charAt(wifiUiCursor-1));
  }
  return v;
}
void wifiUiResetTap(){wifiUiLastTapKey=255;wifiUiLastTapIndex=0;wifiUiLastTapAt=0;}
int wifiUiDigitFromKey(int k){
  switch(k){case 4:return 1;case 5:return 2;case 6:return 3;
  case 8:return 4;case 9:return 5;case 10:return 6;
  case 12:return 7;case 13:return 8;case 14:return 9;case 17:return 0;
  default:return -1;}
}
const char* wifiUiLetters(int digit){
  static const char* lower[]={"0", ".,!?1", "abc2", "def3", "ghi4", "jkl5",
    "mno6", "pqrs7", "tuv8", "wxyz9"};
  static const char* upper[]={"0", ".,!?1", "ABC2", "DEF3", "GHI4", "JKL5",
    "MNO6", "PQRS7", "TUV8", "WXYZ9"};
  static const char* symbols[]={"0"," .,!?:;1","@#_$2","%&*3","()-4","+={}5",
    "[]~6","/\\|7","<>8","^'\"9"};
  if(digit<0||digit>9)return "";
  if(wifiUiMode==2)return ""; // numbers are immediate in 123 mode
  return wifiUiMode==0?lower[digit]:(wifiUiMode==1?upper[digit]:symbols[digit]);
}
void wifiUiInsertChar(char c){
  if(wifiUiPassword.length()>=63){wifiUiFeedback="Максимум 63 символа";return;}
  if(wifiUiCursor>wifiUiPassword.length())wifiUiCursor=wifiUiPassword.length();
  wifiUiPassword=wifiUiPassword.substring(0,wifiUiCursor)+String(c)+
    wifiUiPassword.substring(wifiUiCursor);
  wifiUiCursor++;
  tftDirty=true;
}
void wifiUiTypeDigit(int k){
  const int digit=wifiUiDigitFromKey(k);
  if(wifiUiStage!=WIFIUI_PASSWORD||digit<0)return;
  const uint32_t now=millis();
  if(wifiUiMode==2 || digit==0){
    wifiUiResetTap();wifiUiInsertChar(char('0'+digit));return;
  }
  const char* letters=wifiUiLetters(digit);
  const size_t total=strlen(letters);
  if(!total)return;
  if(wifiUiLastTapKey==k && (uint32_t)(now-wifiUiLastTapAt)<950 &&
      wifiUiCursor>0 && wifiUiCursor<=wifiUiPassword.length()){
    wifiUiLastTapIndex=(wifiUiLastTapIndex+1)%total;
    wifiUiPassword.setCharAt(wifiUiCursor-1,letters[wifiUiLastTapIndex]);
    tftDirty=true;
  }else{
    wifiUiLastTapIndex=0;wifiUiInsertChar(letters[0]);
  }
  wifiUiLastTapKey=k;wifiUiLastTapAt=now;
}
void wifiUiBackspace(){
  wifiUiResetTap();
  if(wifiUiCursor>0 && wifiUiCursor<=wifiUiPassword.length()){
    wifiUiPassword.remove(wifiUiCursor-1,1);
    wifiUiCursor--;
  }
  tftDirty=true;
}
void wifiUiBeginScan(){
  const uint32_t now=millis();
  if(busaWifiScanRunning||busaGoogleNetworkBusy){
    wifiUiFeedback="Сеть занята. Повтори поиск";tftDirty=true;return;
  }
  if(busaWifiScanLastAt&&(uint32_t)(now-busaWifiScanLastAt)<15000){
    wifiUiFeedback="Поиск повторить через 15 секунд";tftDirty=true;return;
  }
  wifiUiCount=0;wifiUiIndex=0;wifiUiSelectedSsid="";
  wifiUiFeedback="Поиск Wi-Fi...";
  WiFi.scanDelete();
  busaWifiScanLastAt=now;busaWifiScanStartedAt=now;wifiUiScanAt=now;
  const int result=WiFi.scanNetworks(true,false,false,80);
  wifiUiOwnsScan=result==WIFI_SCAN_RUNNING||result>=0;
  busaWifiScanRunning=wifiUiOwnsScan;
  wifiUiStage=wifiUiOwnsScan?WIFIUI_SCANNING:WIFIUI_STATUS;
  if(!wifiUiOwnsScan)wifiUiFeedback="Ошибка поиска Wi-Fi";
  tftDirty=true;
}
void wifiUiPoll(){
  const uint32_t now=millis();
  if(wifiUiStage==WIFIUI_SCANNING && wifiUiOwnsScan){
    const int count=WiFi.scanComplete();
    if(count==WIFI_SCAN_RUNNING && (uint32_t)(now-wifiUiScanAt)<10000)return;
    if(count<0 || (uint32_t)(now-wifiUiScanAt)>=10000){
      WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
      wifiUiStage=WIFIUI_STATUS;wifiUiFeedback="Не удалось найти сети. ENT повторить";
    }else{
      for(int i=0;i<count && wifiUiCount<16;i++){
        const String candidate=WiFi.SSID(i);
        if(candidate.isEmpty()||candidate.length()>32)continue;
        bool duplicate=false;
        for(int j=0;j<wifiUiCount;j++)
          if(wifiUiNetworks[j]==candidate){duplicate=true;break;}
        if(duplicate)continue;
        wifiUiNetworks[wifiUiCount]=candidate;
        wifiUiSignals[wifiUiCount]=WiFi.RSSI(i);
        wifiUiCount++;
      }
      WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
      wifiUiStage=wifiUiCount?WIFIUI_NETWORKS:WIFIUI_STATUS;
      wifiUiFeedback=wifiUiCount?"Выбери сеть стрелками":"Сети не найдены. ENT повторить";
    }
    tftDirty=true;
  }
  if(wifiUiConnectPending && !busaGoogleNetworkBusy && !busaWifiScanRunning){
    wifiUiConnectPending=false;
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    lastStationCheck=now;wifiUiConnectAt=now;
    wifiUiFeedback="Подключение к выбранной сети...";
    tftDirty=true;
  }
  if(wifiUiStage==WIFIUI_CONNECTING && !wifiUiConnectPending){
    if(WiFi.status()==WL_CONNECTED){
      wifiUiFeedback="Wi-Fi подключён";tftDirty=true;
      wifiUiStage=WIFIUI_STATUS;
    }else if((uint32_t)(now-wifiUiConnectAt)>16000){
      wifiUiFeedback="Нет соединения. Проверь пароль";tftDirty=true;
      wifiUiStage=WIFIUI_STATUS;
    }
  }
}
void wifiUiSavePassword(){
  wifiUiResetTap();
  if(wifiUiSelectedSsid.isEmpty() || wifiUiSelectedSsid.length()>32 ||
     wifiUiPassword.length()<8||wifiUiPassword.length()>63){
    wifiUiFeedback="Пароль: 8..63 символа";tftDirty=true;return;
  }
  const String oldSsid=prefs.getString("ssid","");
  const String oldPass=prefs.getString("pass","");
  if(prefs.putString("pass",wifiUiPassword)!=wifiUiPassword.length() ||
     prefs.putString("ssid",wifiUiSelectedSsid)!=wifiUiSelectedSsid.length() ||
     prefs.getString("pass","")!=wifiUiPassword ||
     prefs.getString("ssid","")!=wifiUiSelectedSsid){
    prefs.putString("ssid",oldSsid);prefs.putString("pass",oldPass);
    wifiUiFeedback="Ошибка сохранения Wi-Fi";tftDirty=true;return;
  }
  stationSsid=wifiUiSelectedSsid;stationPass=wifiUiPassword;
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiStage=WIFIUI_CONNECTING;
  tftView=0; // ENT means Save and Back: no extra confirmation or popup.
  wifiUiConnectPending=true;
  wifiUiConnectAt=millis();
  wifiUiFeedback="Сеть сохранена. Подключаю...";
  tftDirty=true;
}
bool wifiUiKey(int k){
  if(k==1)return false; // F2 remains display-off
  if(currentPage!=5||approvedSettingsItem!=1||!tftView)return false;
  if(wifiUiStage==WIFIUI_PASSWORD){
    if(k==15){wifiUiPassword="";wifiUiCursor=0;wifiUiStage=WIFIUI_NETWORKS;wifiUiResetTap();}
    else if(k==0)wifiUiBackspace(); // F1 backspace
    else if(k==7||k==11){wifiUiMode=(wifiUiMode+(k==7?3:1))%4;wifiUiResetTap();}
    else if(k==16){if(wifiUiCursor)wifiUiCursor--;wifiUiResetTap();}
    else if(k==18){if(wifiUiCursor<wifiUiPassword.length())wifiUiCursor++;wifiUiResetTap();}
    else if(k==19)wifiUiSavePassword();
    else if(wifiUiDigitFromKey(k)>=0)wifiUiTypeDigit(k);
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_NETWORKS){
    if(k==7||k==11){
      if(wifiUiCount)wifiUiIndex=(wifiUiIndex+wifiUiCount+(k==7?-1:1))%wifiUiCount;
    }else if(k==19 && wifiUiCount){
      wifiUiSelectedSsid=wifiUiNetworks[wifiUiIndex];
      wifiUiPassword="";wifiUiCursor=0;wifiUiMode=0;wifiUiResetTap();
      wifiUiStage=WIFIUI_PASSWORD;wifiUiFeedback="Введи пароль Wi-Fi";
    }else if(k==15){wifiUiStage=WIFIUI_STATUS;wifiUiFeedback="";}
    else if(k==0)wifiUiBeginScan();
    else if(k==16||k==18)return false; // left/right = main tabs
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_SCANNING){
    if(k==15)wifiUiFeedback="Поиск выполняется";
    else if(k==16||k==18)return false;
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_STATUS||wifiUiStage==WIFIUI_CONNECTING){
    if(k==19)wifiUiBeginScan();
    else if(k==15)tftView=0;
    else if(k==16||k==18)return false;
    tftDirty=true;return true;
  }
  return false;
}
void wifiUiLongZero(){
  if(editingTemp){
    if(editTempBuffer.indexOf(',')<0 && editTempBuffer.indexOf('.')<0){
      editTempBuffer=editTempBuffer.substring(0,editCursor)+","+
        editTempBuffer.substring(editCursor);
      editCursor++;
    }
  }else if(wifiUiStage==WIFIUI_PASSWORD && currentPage==5 &&
           approvedSettingsItem==1 && tftView){
    wifiUiResetTap();wifiUiInsertChar(',');
  }
  tftDirty=true;
}
