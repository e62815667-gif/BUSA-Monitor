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
uint8_t wifiUiRetries=0;
uint32_t wifiUiRetryAt=0;
String wifiUiFeedback="";
extern volatile bool busaWifiScanRunning,busaGoogleNetworkBusy;
extern volatile bool busaStaSwitchPending,busaStaSwitching;
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
void wifiUiBeginScan(){
  if(busaStaSwitchPending||busaStaSwitching){
    wifiUiFeedback="Дождись окончания подключения";tftDirty=true;return;
  }
  wifiUiCount=0;wifiUiIndex=0;wifiUiSelectedSsid="";
  wifiUiRetries=0;
  wifiUiScanPending=true;
  wifiUiOwnsScan=false;
  wifiUiStage=WIFIUI_SCANNING;
  wifiUiRetryAt=millis();
  wifiUiFeedback="";
  tftDirty=true;
}
void wifiUiFinishScan(){
  wifiUiScanPending=false;
  wifiUiOwnsScan=false;
  busaWifiScanRunning=false;
  wifiUiStage=WIFIUI_STATUS; // Remain on the Wi-Fi child page, not an automatic picker.
  wifiUiFeedback=wifiUiCount?("Найдено сетей: "+String(wifiUiCount)):"Сети не найдены";
  if(WiFi.status()!=WL_CONNECTED && !busaStaSwitchPending && !busaStaSwitching)
    busaStaRestoreLocal();
  tftDirty=true;
}
void wifiUiRetryOrFinish(){
  WiFi.scanDelete();
  busaWifiScanRunning=false;wifiUiOwnsScan=false;
  if(wifiUiRetries<2){
    wifiUiScanPending=true;
    wifiUiRetryAt=millis()+1500;
    wifiUiStage=WIFIUI_SCANNING;
  }else wifiUiFinishScan();
}
String wifiUiScanJSON(){
  if(wifiUiScanPending || wifiUiOwnsScan || wifiUiStage==WIFIUI_SCANNING)
    return "{\"scanning\":true,\"networks\":[]}";
  String out="{\"scanning\":false,\"networks\":[";
  for(int i=0;i<wifiUiCount;i++){
    if(i)out+=",";
    out+="{\"ssid\":"+quoteJson(wifiUiNetworks[i])+
      ",\"rssi\":"+String(wifiUiSignals[i])+"}";
  }
  out+="]}";
  return out;
}
void wifiUiPoll(){
  const uint32_t now=millis();
  if(wifiUiScanPending && !wifiUiOwnsScan && (int32_t)(now-wifiUiRetryAt)>=0 &&
     !busaGoogleNetworkBusy && !busaStaSwitchPending && !busaStaSwitching){
    wifiUiScanPending=false;
    wifiUiScanAt=now;
    WiFi.scanDelete();
    if(WiFi.getMode()==WIFI_AP)WiFi.mode(WIFI_AP_STA);
    busaWifiScanLastAt=now;busaWifiScanStartedAt=now;
    wifiUiRetries++; // count even immediate failures to prevent an endless scan loop
    const int scan=WiFi.scanNetworks(true,false,false,160);
    if(scan==WIFI_SCAN_FAILED){
      wifiUiRetryOrFinish();
    }else{
      wifiUiOwnsScan=true;
      busaWifiScanRunning=true;
      wifiUiStage=WIFIUI_SCANNING;
    }
  }
  if(wifiUiStage==WIFIUI_SCANNING && wifiUiOwnsScan){
    const int count=WiFi.scanComplete();
    if(count==WIFI_SCAN_RUNNING && (uint32_t)(now-wifiUiScanAt)<13000){
      // Do not block HTTP, TFT, relay alerts or DS18B20 temperature sampling.
    }else if(count<0 || (uint32_t)(now-wifiUiScanAt)>=13000){
      wifiUiRetryOrFinish();
    }else{
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
      if(wifiUiCount==0 && wifiUiRetries<2){
        busaWifiScanRunning=false;wifiUiOwnsScan=false;
        wifiUiScanPending=true;wifiUiRetryAt=now+1500;
      }else wifiUiFinishScan();
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
  wifiUiFocus=0; // Stay on the Wi-Fi child page and show connection progress.
  wifiUiConnectPending=true;
  wifiUiConnectAt=millis();
  wifiUiFeedback="Проверяю сеть перед сохранением...";
  tftDirty=true;
}
bool wifiUiKey(int k){
  if(k==1)return false; // F2: display OFF is always available.
  if(currentPage!=5||approvedSettingsItem!=1||!tftView)return false;
  if(k==15){ // ESC leaves an inline selector/editor, or returns to Settings.
    if(wifiUiStage==WIFIUI_NETWORKS || wifiUiStage==WIFIUI_PASSWORD){
      wifiUiStage=WIFIUI_STATUS;wifiUiResetTap();
      wifiUiFeedback="Изменение не сохранено";
    }else{tftView=0;wifiUiFocus=0;}
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_PASSWORD){
    if(k==0)wifiUiBackspace(); // F1: delete one character.
    else if(k==7||k==11){wifiUiMode=(wifiUiMode+(k==7?3:1))%4;wifiUiResetTap();}
    else if(k==16){if(wifiUiCursor)wifiUiCursor--;wifiUiResetTap();}
    else if(k==18){if(wifiUiCursor<wifiUiPassword.length())wifiUiCursor++;wifiUiResetTap();}
    else if(k==19){ // Enter confirms the password field; it does NOT save Wi-Fi yet.
      wifiUiStage=WIFIUI_STATUS;wifiUiResetTap();wifiUiFocus=3;
      wifiUiFeedback="Пароль введён. Вниз: Сохранить";
    }else if(wifiUiDigitFromKey(k)>=0)wifiUiTypeDigit(k);
    tftDirty=true;return true;
  }
  if(wifiUiStage==WIFIUI_NETWORKS){
    if(k==7||k==11){
      if(wifiUiCount)wifiUiIndex=(wifiUiIndex+wifiUiCount+(k==7?-1:1))%wifiUiCount;
    }else if(k==19 && wifiUiCount){
      wifiUiSelectedSsid=wifiUiNetworks[wifiUiIndex];
      wifiUiPassword="";wifiUiCursor=0;wifiUiMode=0;wifiUiResetTap();
      wifiUiStage=WIFIUI_STATUS;wifiUiFocus=2;
      wifiUiFeedback="Сеть выбрана. Вниз: Пароль";
    }else if(k==0)wifiUiBeginScan(); // F1: refresh the list.
    tftDirty=true;return true;
  }
  // Scanning and connecting never block arrow navigation on the Wi-Fi page.
  if(k==7||k==11){
    wifiUiFocus=(wifiUiFocus+(k==7?4:1))%5;
  }else if(k==19){
    switch(wifiUiFocus){
      case 0: // Status is informational, not a connection retry.
        break;
      case 1: // Explicit network search.
        if(wifiUiStage!=WIFIUI_SCANNING)wifiUiBeginScan();
        break;
      case 2: // Open the network list in this same Wi-Fi tab.
        if(wifiUiStage==WIFIUI_SCANNING)wifiUiFeedback="Поиск выполняется";
        else if(!wifiUiCount)wifiUiFeedback="Сначала выполни поиск сети";
        else{wifiUiStage=WIFIUI_NETWORKS;wifiUiFeedback="Стрелки: сеть, ENT: выбрать";}
        break;
      case 3: // Start editing the password only after SSID selection.
        if(wifiUiSelectedSsid.isEmpty())wifiUiFeedback="Сначала выбери сеть";
        else{
          wifiUiStage=WIFIUI_PASSWORD;wifiUiCursor=wifiUiPassword.length();
          wifiUiMode=0;wifiUiResetTap();wifiUiFeedback="ENT: закончить ввод";
        }
        break;
      case 4: // Separate Save action: validate, persist to NVS, then connect.
        if(wifiUiStage==WIFIUI_SCANNING)wifiUiFeedback="Дождись поиска сети";
        else wifiUiSavePassword();
        break;
    }
  }else if(k==16||k==18)return false; // LEFT/RIGHT switch main pages only.
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
