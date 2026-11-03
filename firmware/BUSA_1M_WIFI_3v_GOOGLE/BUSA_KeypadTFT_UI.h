#pragma once
// BUSA 15.0. Shared Wi-Fi scan queue for the physical TFT and the phone UI.
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
uint8_t wifiUiFocus=0; // One Wi-Fi child page: status, search, network, password, save.
bool wifiUiOwnsScan=false,wifiUiConnectPending=false;
bool wifiUiScanPending=false;
uint8_t wifiUiRetries=0,wifiUiScanPhase=0; // 0 init, 1 settling STA, 2 running
uint32_t wifiUiRetryAt=0,wifiUiScanPrepareAt=0;
int wifiUiLastScanCode=0; // 0: no networks, -2: scanner failed
const uint8_t WIFIUI_MAX_SCAN_ATTEMPTS=3;
String wifiUiFeedback="";
extern volatile bool busaWifiScanRunning,busaGoogleNetworkBusy;
extern volatile bool busaStaSwitchPending,busaStaSwitching;
extern bool busaStaCancelRequested,busaStaDisconnectRequested;
extern bool otaActive;
extern uint32_t busaStaDisconnectAt;
extern volatile int8_t busaStaLastResult;
extern String busaStaMessage;
extern uint32_t busaWifiScanLastAt,busaWifiScanStartedAt,lastStationCheck;
extern String stationSsid,stationPass;
String quoteJson(const String &x); // implemented below this header in BUSA_Main.h
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
// Both the physical TFT and phone app request this SAME nonblocking scanner.
// Google HTTPS or reconnect may delay a scan; neither causes an error screen.
// The scanner is shared by the physical TFT and local iPhone app.
// Never perform a synchronous channel scan in an HTTP callback.
void wifiUiBeginScan(){
  if(otaActive || busaStaDisconnectRequested){
    wifiUiFeedback="Wi-Fi занят: подожди окончания обновления";tftDirty=true;return;
  }
  if(busaStaSwitchPending || busaStaSwitching) {
    // Deliberate search takes priority over a stalled STA association.
    busaStaCancelRequested=true;
  }
  wifiUiCount=0;wifiUiIndex=0;wifiUiSelectedSsid="";
  wifiUiRetries=0;wifiUiScanPhase=0;wifiUiLastScanCode=0;
  wifiUiScanPending=true;wifiUiOwnsScan=false;
  wifiUiStage=WIFIUI_SCANNING;
  wifiUiRetryAt=millis()+150;wifiUiFeedback="Подготовка поиска Wi-Fi";
  tftDirty=true;
}
void wifiUiFinishScan(){
  wifiUiScanPending=false;wifiUiOwnsScan=false;
  busaWifiScanRunning=false;wifiUiScanPhase=0;
  wifiUiStage=WIFIUI_STATUS;
  if(wifiUiCount)wifiUiFeedback="Найдено сетей: "+String(wifiUiCount);
  else if(wifiUiLastScanCode<0)
    wifiUiFeedback="Сбой Wi-Fi-сканера: "+String(wifiUiLastScanCode);
  else wifiUiFeedback="Поиск выполнен. Сети не обнаружены";
  // No successful external connection: restore the local-only mode.
  if(WiFi.status()!=WL_CONNECTED && !busaStaSwitchPending && !busaStaSwitching)
    busaStaRestoreLocal();
  tftDirty=true;
}
void wifiUiRetryOrFinish(){
  WiFi.scanDelete();
  busaWifiScanRunning=false;wifiUiOwnsScan=false;
  if(wifiUiRetries<WIFIUI_MAX_SCAN_ATTEMPTS){
    wifiUiScanPending=true;
    wifiUiScanPhase=0;
    wifiUiRetryAt=millis()+650;
    wifiUiStage=WIFIUI_SCANNING;
    wifiUiFeedback="Повтор поиска "+String(wifiUiRetries+1)+"/"+
      String(WIFIUI_MAX_SCAN_ATTEMPTS);
    tftDirty=true;
  }else wifiUiFinishScan();
}
String wifiUiScanJSON(){
  if(wifiUiScanPending||wifiUiOwnsScan||wifiUiStage==WIFIUI_SCANNING)
    return "{\"scanning\":true,\"networks\":[],\"code\":0}";
  String out="{\"scanning\":false,\"networks\":[";
  for(int i=0;i<wifiUiCount;i++){
    if(i)out+=",";
    out+="{\"ssid\":"+quoteJson(wifiUiNetworks[i])+
      ",\"rssi\":"+String(wifiUiSignals[i])+"}";
  }
  out+="],\"code\":"+String(wifiUiLastScanCode)+
      ",\"attempts\":"+String(wifiUiRetries);
  if(wifiUiLastScanCode<0&&!wifiUiCount)
    out+=",\"error\":\"Wi-Fi scanner failed\"";
  out+="}";
  return out;
}
void wifiUiPoll(){
  const uint32_t now=millis();
  // AP remains enabled while temporary STA capability is prepared for scanning.
  // googleWorker checks pending/ownsScan and yields radio time to this operation.
  if(wifiUiScanPending && !wifiUiOwnsScan &&
     !otaActive && !busaStaDisconnectRequested &&
     !busaGoogleNetworkBusy && !busaStaSwitchPending && !busaStaSwitching){
    if(wifiUiScanPhase==0 && (int32_t)(now-wifiUiRetryAt)>=0){
      // A failed first scan can be caused by an existing STA association.
      // Only a retry detaches it, never erasing saved NVS.
      if(wifiUiRetries>0 && WiFi.status()==WL_CONNECTED)
        WiFi.disconnect(false,false);
      if(!WiFi.enableSTA(true)){
        wifiUiLastScanCode=-3;
        wifiUiRetries++;wifiUiRetryOrFinish();
      }else{
        wifiUiScanPhase=1;wifiUiScanPrepareAt=now;
        wifiUiFeedback="Подготовка радиомодуля";
        tftDirty=true;
      }
    }else if(wifiUiScanPhase==1 &&
      (uint32_t)(now-wifiUiScanPrepareAt)>=500){
      // Set the busy flag BEFORE esp_wifi_scan_start so Google cannot race.
      wifiUiScanPending=false;
      busaWifiScanRunning=true;
      busaWifiScanLastAt=now;busaWifiScanStartedAt=now;
      wifiUiScanAt=now;
      wifiUiRetries++;
      WiFi.scanDelete();
      const int result=WiFi.scanNetworks(true,false,false,180);
      if(result==WIFI_SCAN_FAILED){
        wifiUiLastScanCode=WIFI_SCAN_FAILED;
        wifiUiRetryOrFinish();
      }else{
        wifiUiOwnsScan=true;
        wifiUiScanPhase=2;
        wifiUiStage=WIFIUI_SCANNING;
        wifiUiFeedback="Поиск Wi-Fi: "+String(wifiUiRetries)+"/"+
          String(WIFIUI_MAX_SCAN_ATTEMPTS);
        tftDirty=true;
      }
    }
  }
  if(wifiUiStage==WIFIUI_SCANNING && wifiUiOwnsScan){
    const int count=WiFi.scanComplete();
    if(count==WIFI_SCAN_RUNNING && (uint32_t)(now-wifiUiScanAt)<11000){
      // Non-blocking; the HTTP server and TFT keep running.
    }else if(count<0 || (uint32_t)(now-wifiUiScanAt)>=11000){
      wifiUiLastScanCode=(count<0?count:-10);
      wifiUiRetryOrFinish();
    }else{
      wifiUiLastScanCode=0;
      wifiUiCount=0;
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
      WiFi.scanDelete();
      if(wifiUiCount==0 && wifiUiRetries<WIFIUI_MAX_SCAN_ATTEMPTS)
        wifiUiRetryOrFinish();
      else wifiUiFinishScan();
    }
  }
  if(wifiUiStage==WIFIUI_CONNECTING){
    if(busaStaLastResult==1 && !busaStaSwitching && !busaStaSwitchPending){
      wifiUiConnectPending=false;
      wifiUiFeedback="Wi-Fi подключён";wifiUiStage=WIFIUI_STATUS;tftDirty=true;
    }else if(busaStaLastResult==-1 && !busaStaSwitching && !busaStaSwitchPending){
      wifiUiConnectPending=false;
      wifiUiFeedback="Нет связи. БУСА работает локально";
      wifiUiStage=WIFIUI_STATUS;tftDirty=true;
    }
  }
}
void wifiUiSavePassword(){
  wifiUiResetTap();
  if(wifiUiSelectedSsid.isEmpty() || wifiUiSelectedSsid.length()>32 ||
     wifiUiPassword.length()<8||wifiUiPassword.length()>63){
    wifiUiFeedback="Пароль: 8..63 символа";tftDirty=true;return;
  }
  if(!busaStageStaCredentials(wifiUiSelectedSsid,wifiUiPassword)){
    wifiUiFeedback="Wi-Fi занят. Повтори позже";tftDirty=true;return;
  }
  wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();
  wifiUiStage=WIFIUI_CONNECTING;
  wifiUiFocus=0;
  wifiUiConnectPending=true;
  wifiUiConnectAt=millis();
  wifiUiFeedback="Проверяю сеть перед сохранением...";
  tftDirty=true;
}
bool wifiUiKey(int k){
  if(k==1)return false; // F2: screen off.
  if(currentPage!=5||approvedSettingsItem!=1||!tftView)return false;
  if(k==15){
    if(wifiUiStage==WIFIUI_NETWORKS||wifiUiStage==WIFIUI_PASSWORD){
      wifiUiStage=WIFIUI_STATUS;wifiUiResetTap();
      wifiUiFeedback="Изменение отменено";
    }else{tftView=0;wifiUiFocus=0;}
    tftDirty=true;return true;
  }
  if(k==0 && wifiUiStage==WIFIUI_STATUS && wifiUiFocus==0){
    // Explicit F1 disconnect of external Wi-Fi; never erases other settings.
    if(!otaActive && !busaStaDisconnectRequested){
      busaStaDisconnectRequested=true;busaStaDisconnectAt=millis();
      wifiUiFeedback="Отключаю внешний Wi-Fi";tftDirty=true;
    }
    return true;
  }
  if(wifiUiStage==WIFIUI_PASSWORD){
    if(k==0)wifiUiBackspace();
    else if(k==7||k==11){wifiUiMode=(wifiUiMode+(k==7?3:1))%4;wifiUiResetTap();}
    else if(k==16){if(wifiUiCursor)wifiUiCursor--;wifiUiResetTap();}
    else if(k==18){if(wifiUiCursor<wifiUiPassword.length())wifiUiCursor++;wifiUiResetTap();}
    else if(k==19){wifiUiStage=WIFIUI_STATUS;wifiUiResetTap();wifiUiFocus=3;
      wifiUiFeedback="Пароль введён. Вниз: Сохранить";}
    else if(wifiUiDigitFromKey(k)>=0)wifiUiTypeDigit(k);
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_NETWORKS){
    if(k==7||k==11){
      if(wifiUiCount)wifiUiIndex=(wifiUiIndex+wifiUiCount+(k==7?-1:1))%wifiUiCount;
    }else if(k==19&&wifiUiCount){
      wifiUiSelectedSsid=wifiUiNetworks[wifiUiIndex];
      wifiUiPassword="";wifiUiCursor=0;wifiUiMode=0;wifiUiResetTap();
      wifiUiStage=WIFIUI_STATUS;wifiUiFocus=2;
      wifiUiFeedback="Сеть выбрана. Вниз: Пароль";
    }else if(k==0)wifiUiBeginScan();
    tftDirty=true;return true;
  }
  if(k==7||k==11)wifiUiFocus=(wifiUiFocus+(k==7?4:1))%5;
  else if(k==19){
    switch(wifiUiFocus){
      case 0: break;
      case 1: if(wifiUiStage!=WIFIUI_SCANNING)wifiUiBeginScan();break;
      case 2:
        if(wifiUiStage==WIFIUI_SCANNING)wifiUiFeedback="Поиск выполняется";
        else if(!wifiUiCount)wifiUiFeedback="Сначала выполни поиск сети";
        else{wifiUiStage=WIFIUI_NETWORKS;wifiUiFeedback="Стрелки: сеть, ENT: выбрать";}
        break;
      case 3:
        if(wifiUiSelectedSsid.isEmpty())wifiUiFeedback="Сначала выбери сеть";
        else{wifiUiStage=WIFIUI_PASSWORD;wifiUiCursor=wifiUiPassword.length();
             wifiUiMode=0;wifiUiResetTap();wifiUiFeedback="ENT: закончить ввод";}
        break;
      case 4:
        if(wifiUiStage==WIFIUI_SCANNING)wifiUiFeedback="Дождись поиска сети";
        else wifiUiSavePassword();
        break;
    }
  }else if(k==16||k==18)return false;
  tftDirty=true;return true;
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
