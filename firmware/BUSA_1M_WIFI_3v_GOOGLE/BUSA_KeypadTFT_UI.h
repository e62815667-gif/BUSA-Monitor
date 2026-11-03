#pragma once
// BUSA 14.8. Physical 20-key navigation + TFT Wi-Fi scan/password.
// Key-to-PCF8575 pairs remain in original NVS "busa-r04"/"keys20"
// with backup "busa-key20"/"keys20"; no retraining at OTA.
enum BusaWifiUiStage : uint8_t { WIFIUI_STATUS=0, WIFIUI_SCANNING=1,
  WIFIUI_NETWORKS=2, WIFIUI_PASSWORD=3, WIFIUI_CONNECTING=4 };
BusaWifiUiStage wifiUiStage=WIFIUI_STATUS;
uint8_t wifiUiMenuIndex=0; // Search, select SSID, password, save, disconnect from external STA
bool wifiUiStaDisabled=false; // persistent NVS "staOff" flag; AP always stays ON
bool wifiUiDisablePending=false; // defer STA disconnect while Google HTTPS is active
String wifiUiNetworks[16],wifiUiSelectedSsid="",wifiUiPassword="";
int wifiUiSignals[16]={0};
uint8_t wifiUiCount=0,wifiUiIndex=0,wifiUiMode=0;
uint8_t wifiUiLastTapKey=255,wifiUiLastTapIndex=0;
uint32_t wifiUiLastTapAt=0,wifiUiScanAt=0,wifiUiConnectAt=0;
uint8_t wifiUiCursor=0;
bool wifiUiOwnsScan=false,wifiUiConnectPending=false;
uint8_t wifiUiScanAttempts=0; // One automatic retry for failed or empty scans
uint32_t wifiUiScanRetryAt=0;
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
  wifiUiConnectPending=false;
  WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
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
  wifiUiConnectPending=false;
  wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=4;
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiFeedback="Отключаю внешнюю сеть...";
  if(!busaGoogleNetworkBusy && !otaActive)wifiUiApplyStationDisconnect();
  tftDirty=true;
}
void wifiUiRestoreStationAfterScan(){
  if(!wifiUiResumeStation)return;
  wifiUiResumeStation=false;
  if(wifiUiStaDisabled || wifiUiDisablePending || wifiUiConnectPending ||
     stationSsid.isEmpty() || stationPass.length()<8)return;
  WiFi.setAutoReconnect(true);
  WiFi.begin(stationSsid.c_str(),stationPass.c_str());
  lastStationCheck=millis();
}
void wifiUiLeave(){
  if(wifiUiOwnsScan){
    WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
  }
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiStage=WIFIUI_STATUS;
  wifiUiMenuIndex=0;wifiUiScanRetryAt=0;
  wifiUiRestoreStationAfterScan();
  // Leaving the menu NEVER disconnects BUSA's own Wi-Fi hotspot.
  tftDirty=true;
}

// Recover STA scan capability without stopping/restarting the BUSA softAP.
// ESP32-S3 uses one radio for AP+STA; scanning still works when external
// internet was explicitly disabled with WiFi.disconnect(false,false).
void wifiUiStartScanAttempt(uint32_t now){
  ++wifiUiScanAttempts;
  wifiUiScanRetryAt=0;
  wifiUiStage=WIFIUI_SCANNING;
  if(WiFi.getMode()!=WIFI_AP_STA){
    if(!WiFi.mode(WIFI_AP_STA)){
      wifiUiLastScanCode=WIFI_SCAN_FAILED;
      wifiUiFeedback="STA недоступна. AP БУСА работает";
      wifiUiOwnsScan=false;busaWifiScanRunning=false;
      if(wifiUiScanAttempts<2)wifiUiScanRetryAt=now+1200;
      else wifiUiStage=WIFIUI_STATUS;
      tftDirty=true;return;
    }
  }
  WiFi.setSleep(false);
  WiFi.scanDelete();
  wifiUiScanAt=now;
  busaWifiScanLastAt=now;busaWifiScanStartedAt=now;
  // Passive all-channel fallback differs from first active scan.
  wifiUiPassiveFallback=(wifiUiScanAttempts>1);
  const int result=wifiUiPassiveFallback?
    WiFi.scanNetworks(true,true,true,350):
    WiFi.scanNetworks(true,true,false,180);
  wifiUiLastScanCode=result;
  wifiUiOwnsScan=(result==WIFI_SCAN_RUNNING || result>=0);
  busaWifiScanRunning=wifiUiOwnsScan;
  if(!wifiUiOwnsScan){
    wifiUiFeedback="Ошибка сканера. Повторяю...";
    if(wifiUiScanAttempts<2)wifiUiScanRetryAt=now+1200;
    else{
      wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=0;
      wifiUiFeedback="Сканер Wi-Fi: ошибка "+String(result);
    }
  }else{
    wifiUiFeedback=wifiUiScanAttempts>1?
      "Повторный поиск сетей 2,4 ГГц...":
      "Поиск сетей 2,4 ГГц...";
  }
  tftDirty=true;
}
void wifiUiBeginScan(){
  const uint32_t now=millis();
  if(otaActive || busaGoogleNetworkBusy){
    wifiUiFeedback="Сеть занята, повтори поиск";tftDirty=true;return;
  }
  // The local app's scan can remain flagged when its browser stopped polling.
  // Recover only a stale request; never interrupt a valid in-flight scan.
  if(busaWifiScanRunning && !wifiUiOwnsScan &&
     (uint32_t)(now-busaWifiScanStartedAt)>=16000){
    WiFi.scanDelete();busaWifiScanRunning=false;
  }
  if(busaWifiScanRunning){
    wifiUiFeedback="Сканирование уже выполняется";tftDirty=true;return;
  }
  if(busaWifiScanLastAt && (uint32_t)(now-busaWifiScanLastAt)<3000){
    wifiUiFeedback="Повтори поиск через 3 секунды";tftDirty=true;return;
  }
  wifiUiCount=0;wifiUiIndex=0;wifiUiSelectedSsid="";
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiScanAttempts=0;wifiUiScanRetryAt=0;wifiUiLastScanCode=-99;
  wifiUiPassiveFallback=false;
  // A connected STA can make ESP32-S3 Wi-Fi scans report zero networks.
  // Pause only STA; keep WIFI_AP_STA, BUSA AP, HTTP and OTA active.
  wifiUiResumeStation=!wifiUiStaDisabled && WiFi.status()==WL_CONNECTED &&
    !stationSsid.isEmpty() && stationPass.length()>=8;
  if(WiFi.status()==WL_CONNECTED){
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false,false); // STA only, never disable BUSA softAP
    lastStationCheck=now;
    wifiUiStage=WIFIUI_SCANNING;
    wifiUiFeedback="Освобождаю STA для поиска...";
    wifiUiScanRetryAt=now+650; // wait for driver to process disassociation
    tftDirty=true;return;
  }
  wifiUiStartScanAttempt(now);
}
void wifiUiPoll(){
  const uint32_t now=millis();
  if(wifiUiDisablePending && !busaGoogleNetworkBusy && !otaActive){
    wifiUiApplyStationDisconnect();
    return;
  }
  if(wifiUiStage==WIFIUI_SCANNING){
    if(wifiUiScanRetryAt){
      if((int32_t)(now-wifiUiScanRetryAt)>=0){
        wifiUiStartScanAttempt(now);
      }
      return; // Waiting for the automatic retry, not a failed search menu.
    }
    if(!wifiUiOwnsScan)return;
    const int count=WiFi.scanComplete();
    if(count==WIFI_SCAN_RUNNING &&
       (uint32_t)(now-wifiUiScanAt)<14000)return;
    const bool timedOut=count==WIFI_SCAN_RUNNING;
    wifiUiLastScanCode=timedOut?-3:count;
    if(timedOut || count<=0){
      WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
      if(wifiUiScanAttempts<2){
        wifiUiScanRetryAt=now+1000;
        wifiUiFeedback="Поиск не завершён. Повторяю...";
      }else{
        wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=0;
        if(timedOut)wifiUiFeedback="Поиск Wi-Fi: превышено время";
        else if(count<0)wifiUiFeedback="Ошибка сканера: "+String(count);
        else wifiUiFeedback="Сети 2,4 ГГц не найдены";
        wifiUiRestoreStationAfterScan();
      }
      tftDirty=true;return;
    }
    for(int i=0;i<count && wifiUiCount<16;i++){
      const String candidate=WiFi.SSID(i);
      if(candidate.isEmpty()||candidate.length()>32)continue;
      bool duplicate=false;
      for(int j=0;j<wifiUiCount;j++)
        if(wifiUiNetworks[j]==candidate){duplicate=true;break;}
      if(duplicate)continue;
      wifiUiNetworks[wifiUiCount]=candidate;
      wifiUiSignals[wifiUiCount]=WiFi.RSSI(i);
      ++wifiUiCount;
    }
    WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
    wifiUiStage=WIFIUI_STATUS;
    wifiUiMenuIndex=wifiUiCount?1:0;
    wifiUiFeedback=wifiUiCount?
      "Найдено сетей: "+String(wifiUiCount)+". ENT: выбор":
      "Сети не отображаются: повтори поиск";
    wifiUiRestoreStationAfterScan();
    tftDirty=true;
  }
  if(wifiUiConnectPending && !wifiUiDisablePending &&
     !busaGoogleNetworkBusy && !busaWifiScanRunning){
    wifiUiConnectPending=false;
    WiFi.setAutoReconnect(true);
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    lastStationCheck=now;wifiUiConnectAt=now;
    wifiUiStage=WIFIUI_CONNECTING;
    wifiUiFeedback="Подключение к выбранной сети...";
    tftDirty=true;
  }
  if(wifiUiStage==WIFIUI_CONNECTING && !wifiUiConnectPending){
    if(WiFi.status()==WL_CONNECTED){
      wifiUiFeedback="Подключено: "+WiFi.SSID();
      wifiUiStage=WIFIUI_STATUS;
      tftDirty=true;
    }else if((uint32_t)(now-wifiUiConnectAt)>16000){
      wifiUiFeedback="Нет соединения. Проверь пароль";
      wifiUiStage=WIFIUI_STATUS;
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
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiConnectPending=true;
  wifiUiConnectAt=millis();wifiUiStage=WIFIUI_CONNECTING;wifiUiMenuIndex=3;
  wifiUiFeedback="Сеть сохранена. Подключаю...";
  tftDirty=true;
}
// Five ordered Wi-Fi actions: Search / Select / Password / Save / STA Disconnect.
// The two-stage selection follows the same UP/DOWN -> ENTER -> ESC pattern as Settings.
bool wifiUiKey(int k){
  if(k==1)return false; // F2 still turns TFT display OFF
  if(currentPage!=5 || approvedSettingsItem!=1 || !tftView)return false;
  if(wifiUiStage==WIFIUI_PASSWORD){
    if(k==15){wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=2;wifiUiResetTap();}
    else if(k==0)wifiUiBackspace();
    else if(k==7||k==11){wifiUiMode=(wifiUiMode+(k==7?3:1))%4;wifiUiResetTap();}
    else if(k==16){if(wifiUiCursor)wifiUiCursor--;wifiUiResetTap();}
    else if(k==18){if(wifiUiCursor<wifiUiPassword.length())wifiUiCursor++;wifiUiResetTap();}
    else if(k==19){
      wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=3;
      wifiUiFeedback="Пароль введён. Выбери Сохранить";
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
      wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=2;
      wifiUiFeedback="Сеть выбрана. ENTER: пароль";
    }else if(k==15){wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=1;}
    else if(k==16||k==18)return false;
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_SCANNING){
    if(k==15){
      WiFi.scanDelete();wifiUiOwnsScan=false;busaWifiScanRunning=false;
      wifiUiScanRetryAt=0;
      wifiUiStage=WIFIUI_STATUS;wifiUiFeedback="Поиск отменён";
      wifiUiRestoreStationAfterScan();
    }else if(k==16||k==18)return false;
    tftDirty=true;return true;
  }
  // The menu is always stable on entry: no unsolicited scan or password popup.
  if(wifiUiStage==WIFIUI_STATUS||wifiUiStage==WIFIUI_CONNECTING){
    if(k==7||k==11){
      wifiUiMenuIndex=(wifiUiMenuIndex+(k==7?4:1))%5;
    }else if(k==19){
      switch(wifiUiMenuIndex){
        case 0:wifiUiBeginScan();break;
        case 1:
          if(wifiUiCount){wifiUiIndex=0;wifiUiStage=WIFIUI_NETWORKS;}
          else{wifiUiFeedback="Сначала запусти поиск сетей";}
          break;
        case 2:
          if(wifiUiSelectedSsid.length()){
            wifiUiMode=0;wifiUiResetTap();wifiUiStage=WIFIUI_PASSWORD;
          }else wifiUiFeedback="Сначала выбери сеть";
          break;
        case 3:wifiUiSaveConnection();break;
        case 4:wifiUiRequestStationDisconnect();break;
      }
    }else if(k==15){
      wifiUiLeave();tftView=0;
    }else if(k==16||k==18)return false; // normal main section switching
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
