#pragma once
// BUSA 14.8. Physical 20-key navigation + TFT Wi-Fi scan/password.
// Key-to-PCF8575 pairs remain in original NVS "busa-r04"/"keys20"
// with backup "busa-key20"/"keys20"; no retraining at OTA.
enum BusaWifiUiStage : uint8_t { WIFIUI_STATUS=0, WIFIUI_SCANNING=1,
  WIFIUI_NETWORKS=2, WIFIUI_PASSWORD=3, WIFIUI_CONNECTING=4 };
BusaWifiUiStage wifiUiStage=WIFIUI_STATUS;
uint8_t wifiUiMenuIndex=0; // Select network, password, connect (minimal physical UI)
bool wifiUiStaDisabled=false; // persistent NVS "staOff" flag; AP always stays ON
bool wifiUiDisablePending=false; // defer STA disconnect while Google HTTPS is active
String wifiUiNetworks[48],wifiUiSelectedSsid="",wifiUiPassword="";
int wifiUiSignals[48]={0};
uint8_t wifiUiCount=0,wifiUiIndex=0,wifiUiMode=0;
uint8_t wifiUiLastTapKey=255,wifiUiLastTapIndex=0;
uint32_t wifiUiLastTapAt=0,wifiUiScanAt=0,wifiUiConnectAt=0;
uint8_t wifiUiCursor=0;
bool wifiUiOwnsScan=false,wifiUiConnectPending=false;
bool busaWifiRequestSharedScan(bool physical); // implemented in BUSA_Main.h
uint32_t wifiUiAssociationDueAt=0;
String wifiUiLastConnectError=""; // displayed on physical TFT + local app
uint8_t wifiUiScanAttempts=0; // One automatic retry for failed or empty scans
uint32_t wifiUiScanRetryAt=0;
bool wifiUiAutoScanPending=false;
uint32_t wifiUiAutoScanAt=0;
bool wifiUiResumeStation=false; // restore prior STA after search, never AP
bool wifiUiPassiveFallback=false; // second scan listens for 2.4GHz beacons
int wifiUiLastScanCode=-99; // -3=timeout, -2=failed, >=0=number found
String wifiUiFeedback="";
extern volatile bool busaWifiScanRunning,busaGoogleNetworkBusy;
extern uint32_t busaWifiScanLastAt,busaWifiScanStartedAt,lastStationCheck;
extern String stationSsid,stationPass;
extern bool otaActive;
extern volatile int googleInternet;
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
// Disconnect ONLY the external station. The BUSA access point, local web app,
// and OTA stay available at 192.168.4.1, even after reboot.
void wifiUiApplyStationDisconnect(){
  wifiUiResumeStation=false; // explicit user STA disconnect wins
  wifiUiConnectPending=false;wifiUiAssociationDueAt=0;wifiUiConnectAt=0;
  wifiUiLastConnectError="";
  wifiUiOwnsScan=false;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false,false); // NEVER stop the radio or the BUSA softAP
  wifiUiDisablePending=false;
  googleInternet=0;
  wifiUiStage=WIFIUI_STATUS;
  wifiUiFeedback="Внешняя сеть отключена; AP БУСА работает";
  tftDirty=true;
}
void wifiUiRequestStationDisconnect(){
  if(!prefs.putBool("staOff",true) || !prefs.getBool("staOff",false)){
    wifiUiFeedback="Ошибка сохранения отключения";tftDirty=true;return;
  }
  wifiUiStaDisabled=true;
  wifiUiDisablePending=true;
  wifiUiConnectPending=false;wifiUiAssociationDueAt=0;wifiUiConnectAt=0;
  wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=0;
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiFeedback="Отключаю внешнюю сеть...";
  if(!busaGoogleNetworkBusy && !otaActive)wifiUiApplyStationDisconnect();
  tftDirty=true;
}
void wifiUiRestoreStationAfterScan(){
  // Search is read-only. Do NOT reconnect the router the user left.
  // A fresh explicit Save is required for any external Wi-Fi association.
  wifiUiResumeStation=false;
}
void wifiUiLeave(){
  // Scan is owned by the shared radio service, not this UI page.
  wifiUiOwnsScan=false;
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiStage=WIFIUI_STATUS;
  wifiUiMenuIndex=0;wifiUiScanRetryAt=0;
  wifiUiAutoScanPending=false;
  wifiUiRestoreStationAfterScan();
  // Leaving the menu NEVER disconnects BUSA's own Wi-Fi hotspot.
  tftDirty=true;
}

// TFT and browser use ONE async scanner, serviced by BUSA_Main.loop().
void wifiUiBeginScan(){
  wifiUiIndex=0;
  wifiUiSelectedSsid="";wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiAutoScanPending=false;wifiUiScanRetryAt=0;
  wifiUiMenuIndex=0;
  // The shared service can reuse a recent completed result.
  if(busaWifiRequestSharedScan(true)){
    if(wifiUiStage!=WIFIUI_NETWORKS){
      wifiUiStage=WIFIUI_SCANNING;
      wifiUiFeedback="Поиск сетей...";
    }
  }else{
    wifiUiStage=WIFIUI_STATUS;
    wifiUiFeedback="Поиск недоступен";
  }
  tftDirty=true;
}
void wifiUiPoll(){
  const uint32_t now=millis();
  // One finite scan run; no automatic retry loop in the physical menu.
  if(wifiUiDisablePending && !otaActive){
    wifiUiApplyStationDisconnect();
    return;
  }
  if(wifiUiConnectPending && !wifiUiDisablePending && !otaActive &&
     !busaWifiScanRunning){
    wifiUiConnectPending=false;
    wifiUiResumeStation=false; // a new saved SSID wins over old iPhone
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false,false); // disconnect STA only; keep BUSA local AP
    lastStationCheck=now;
    wifiUiAssociationDueAt=now+550;
    wifiUiConnectAt=0;wifiUiLastConnectError="";
    wifiUiStage=WIFIUI_CONNECTING;
    wifiUiFeedback="Переключаю на "+stationSsid;
    tftDirty=true;
  }
  if(wifiUiAssociationDueAt && (int32_t)(now-wifiUiAssociationDueAt)>=0 &&
     !wifiUiDisablePending && !otaActive){
    wifiUiAssociationDueAt=0;
    if(WiFi.getMode()!=WIFI_AP_STA)WiFi.mode(WIFI_AP_STA);
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    wifiUiConnectAt=now;lastStationCheck=now;
    wifiUiFeedback="Подключаю: "+stationSsid;
    tftDirty=true;
  }
  if(wifiUiStage==WIFIUI_CONNECTING && !wifiUiConnectPending &&
     !wifiUiAssociationDueAt && wifiUiConnectAt){
    if(WiFi.status()==WL_CONNECTED && WiFi.SSID()==stationSsid){
      wifiUiLastConnectError="";
      WiFi.setAutoReconnect(false);
      wifiUiFeedback="Подключено: "+stationSsid;
      wifiUiConnectAt=0;wifiUiStage=WIFIUI_STATUS;
      tftDirty=true;
    }else if((uint32_t)(now-wifiUiConnectAt)>18000){
      wifiUiLastConnectError="Не подключено. Проверь пароль";
      WiFi.setAutoReconnect(false);
      WiFi.disconnect(false,false); // no silent fallback to the old hotspot
      wifiUiFeedback=wifiUiLastConnectError;
      wifiUiConnectAt=0;wifiUiStage=WIFIUI_STATUS;
      tftDirty=true;
    }
  }
}
// Only this explicit Save menu item commits Wi-Fi credentials and enables
// automatic reconnection after ESP32 reboot; password entry does not commit.
void wifiUiSaveConnection(){
  wifiUiResetTap();
  if(wifiUiSelectedSsid.isEmpty()||wifiUiSelectedSsid.length()>32||
     wifiUiPassword.length()<8||wifiUiPassword.length()>63){
    wifiUiFeedback="Сначала выбери сеть и пароль 8-63";
    tftDirty=true;return;
  }
  const String oldSsid=prefs.getString("ssid","");
  const String oldPass=prefs.getString("pass","");
  const bool oldOff=prefs.getBool("staOff",false);
  const bool saved=
    prefs.putString("pass",wifiUiPassword)==wifiUiPassword.length() &&
    prefs.putString("ssid",wifiUiSelectedSsid)==wifiUiSelectedSsid.length() &&
    prefs.putBool("staOff",false) &&
    prefs.getString("pass","")==wifiUiPassword &&
    prefs.getString("ssid","")==wifiUiSelectedSsid &&
    !prefs.getBool("staOff",true);
  if(!saved){
    prefs.putString("ssid",oldSsid);prefs.putString("pass",oldPass);
    prefs.putBool("staOff",oldOff);
    wifiUiFeedback="Ошибка сохранения Wi-Fi";tftDirty=true;return;
  }
  stationSsid=wifiUiSelectedSsid;
  stationPass=wifiUiPassword;
  wifiUiStaDisabled=false;wifiUiDisablePending=false;
  wifiUiResumeStation=false;wifiUiAssociationDueAt=0;wifiUiConnectAt=0;
  wifiUiLastConnectError="";
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiConnectPending=true;
  wifiUiConnectAt=millis();wifiUiStage=WIFIUI_CONNECTING;wifiUiMenuIndex=2;
  wifiUiFeedback="Сеть сохранена. Подключаю...";
  tftDirty=true;
}
// Three ordinary actions only: choose a network, enter password, connect.
// A scan begins when Settings -> Wi-Fi opens; no separate Search menu.
bool wifiUiKey(int k){
  if(k==1)return false; // F2 still turns TFT display OFF
  if(currentPage!=5 || approvedSettingsItem!=1 || !tftView)return false;
  if(wifiUiStage==WIFIUI_PASSWORD){
    if(k==15){wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=1;wifiUiResetTap();}
    else if(k==0)wifiUiBackspace();
    else if(k==7||k==11){wifiUiMode=(wifiUiMode+(k==7?3:1))%4;wifiUiResetTap();}
    else if(k==16){if(wifiUiCursor)wifiUiCursor--;wifiUiResetTap();}
    else if(k==18){if(wifiUiCursor<wifiUiPassword.length())wifiUiCursor++;wifiUiResetTap();}
    else if(k==19){
      wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=2;
      wifiUiFeedback="Пароль введён. Выбери Подключить";
      wifiUiResetTap();
    }
    else if(wifiUiDigitFromKey(k)>=0)wifiUiTypeDigit(k);
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_NETWORKS){
    if(k==7||k==11){
      if(wifiUiCount)wifiUiIndex=(wifiUiIndex+wifiUiCount+(k==7?-1:1))%wifiUiCount;
    }else if(k==19 && wifiUiCount){
      wifiUiSelectedSsid=wifiUiNetworks[wifiUiIndex];
      wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
      wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=1;
      wifiUiFeedback="Сеть выбрана";
    }else if(k==15){wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=0;}
    else if(k==16||k==18)return false;
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_SCANNING){
    if(k==15){wifiUiStage=WIFIUI_STATUS;wifiUiFeedback="";wifiUiAutoScanPending=false;}
    else if(k==16||k==18)return false;
    tftDirty=true;return true;
  }
  // While idle, show just the three actions. No old-router waiting screen.
  if(wifiUiStage==WIFIUI_STATUS||wifiUiStage==WIFIUI_CONNECTING){
    if(k==7||k==11){
      wifiUiMenuIndex=(wifiUiMenuIndex+(k==7?2:1))%3;
    }else if(k==19){
      switch(wifiUiMenuIndex){
        case 0:
          if(wifiUiCount){wifiUiIndex=0;wifiUiStage=WIFIUI_NETWORKS;}
          else wifiUiBeginScan(); // automatic retry when no networks are listed
          break;
        case 1:
          if(wifiUiSelectedSsid.length()){
            wifiUiMode=0;wifiUiResetTap();wifiUiStage=WIFIUI_PASSWORD;
          }else wifiUiFeedback="Выберите сеть";
          break;
        case 2:wifiUiSaveConnection();break;
      }
    }else if(k==15){wifiUiLeave();tftView=0;}
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
