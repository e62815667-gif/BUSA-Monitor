#pragma once
/*
  BUSA-1M-001 БУСА 1М WiFi 3v-GOOGLE
  ESP32-S3 N8R2 44-pin, TFT GMT028-05 V1.1 320x240 landscape, monitoring-only.
  LOW-VOLTAGE MONITORING ONLY. DO NOT DRIVE RELAYS OR POWER OUTPUTS.
  Approved hardware scope: mains power controller and motorized valve are
  REMOVED from this project; they are operated manually, outside ESP32.
  Approved informational section "Отбор" is retained without actuator control.
  Real 4 x DS18B20 + YF-S401 cooling-water pulse readings, no simulated values.
  Hardware compatibility of TFT controller, PCF8575 keypad and pulse output 
  must be verified on a powered-off low-voltage bench before connecting.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <SPI.h>
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <sys/time.h>
#include <esp_system.h>
#include <Wire.h>
#include "BUSA_PCA9685_Leds.h"
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_ILI9341.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <math.h>

// DS18B20 T1-T4: four separate direct 1-Wire buses, each with its own 4.7k pull-up to 3.3V.
// Old GPIO38-41 are no longer used for temperature sensors.
constexpr uint8_t TPIN[4]={16,17,18,21};
constexpr uint8_t KPIN[4]={7,6,5,4}; // Reference only: never configured as outputs
constexpr uint8_t BUS_SDA=8,BUS_SCL=9;
constexpr uint8_t TFT_CS=10,TFT_MOSI=11,TFT_SCK=12,TFT_DC=13,TFT_RST=14;
constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V
constexpr bool RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false;
static_assert(!RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,"Physical outputs must stay locked");
static_assert(FLOW_GPIO==15 && TPIN[0]==16 && TPIN[1]==17 && TPIN[2]==18 && TPIN[3]==21 && BUS_SDA==8 && BUS_SCL==9,"Temperature GPIO mapping changed unexpectedly");
const char VERSION[]="БУСА 1М WiFi 14v27-N8R2-SIMULTANEOUS-AP-STA";

OneWire ow1(TPIN[0]),ow2(TPIN[1]),ow3(TPIN[2]),ow4(TPIN[3]);
OneWire* const temperatureBuses[4]={&ow1,&ow2,&ow3,&ow4};
bool elapsed(uint32_t now,uint32_t then,uint32_t ms){return (uint32_t)(now-then)>=ms;}
String quoteJson(const String &x);
#include "BUSA_Temperature.h"

// YF-S401 pulse reader for ordinary cooling WATER only. No process control.
// Until a real measured-volume calibration, only raw pulses are factual.
// Never use zero pulse count or calculated flow as a safety interlock.
portMUX_TYPE flowMux=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseCounter=0;
void IRAM_ATTR flowISR(){
  portENTER_CRITICAL_ISR(&flowMux);
  ++pulseCounter;
  portEXIT_CRITICAL_ISR(&flowMux);
}
uint32_t flowPulses(){
  uint32_t n;
  portENTER_CRITICAL(&flowMux);n=pulseCounter;portEXIT_CRITICAL(&flowMux);
  return n;
}
// Calibrated pulses per litre are stored in Preferences, never hardcoded.
// A calibration is a local, explicit, measured-volume water-only action.
float waterPulsesPerLiter=NAN;
bool waterCalibrated=false,waterCalSession=false;
uint32_t waterCalStartPulses=0,waterCalLastPulses=0;
float waterCalLastMl=0.0f;
uint32_t flowCycle=0,flowLastCount=0;
float flowLpm=NAN,flowTrace[40]={};
uint32_t waterPulseHistory[40]={}; // raw pulses only, not calibrated
uint32_t waterPulseDelta=0;
bool flowReady=false,flowSeen=false;
uint8_t flowTraceHead=0;
void sampleFlow(uint32_t now){
  if(!flowCycle){flowCycle=now;return;}
  uint32_t dt=now-flowCycle;
  if(dt<2000)return;
  uint32_t count=flowPulses();
  uint32_t diff=count-flowLastCount;
  flowLastCount=count;flowCycle=now;
  // Expose no calculated litres/minute before calibration.
  flowLpm=waterCalibrated && isfinite(waterPulsesPerLiter) && waterPulsesPerLiter>0
     ?(float)diff*60000.0f/(dt*waterPulsesPerLiter):NAN;
  if(diff)flowSeen=true;
  flowReady=true;
  waterPulseDelta=diff;
  waterPulseHistory[flowTraceHead%40]=diff;
  flowTrace[flowTraceHead++%40]=flowLpm;
}
float totalWaterLitres(){
  return waterCalibrated && isfinite(waterPulsesPerLiter) && waterPulsesPerLiter>0
      ?(float)flowPulses()/waterPulsesPerLiter:NAN;
}
String waterCalJSON(){
  const uint32_t nowPulses=flowPulses();
  const uint32_t delta=waterCalSession?(nowPulses-waterCalStartPulses):0;
  return String("{\"calibrated\":")+(waterCalibrated?"true":"false")+
    ",\"pulses_per_litre\":"+(waterCalibrated?String(waterPulsesPerLiter,3):String("null"))+
    ",\"session_active\":"+(waterCalSession?"true":"false")+
    ",\"session_pulses\":"+String(delta)+
    ",\"last_test_pulses\":"+String(waterCalLastPulses)+
    ",\"last_test_ml\":"+String(waterCalLastMl,1)+"}";
}

// I2C devices: PCF8575 keypad is monitored; PCA9685 ACK-only (never PWM).
bool i2cOnline=false,pcfDetected=false,pcaDetected=false;
uint8_t pcfAddr=0;uint32_t lastBusProbe=0;
uint8_t i2cFound[24]={0},i2cFoundCount=0;
uint16_t i2cAckCount=0;
uint32_t i2cFullScanAt=0;
int pcaMode1=-1,pcaMode2=-1;
// CH0..CH10: read-only register evidence plus independent virtual commands.
// Never infer real LED light, current, OE state or 3.3V load protection.
BusaLed::ChannelSnapshot pcaLedRegs[BusaLed::kCount];
uint32_t pcaLedReadErrors=0,pcaLedReadOk=0,pcaLedScanAt=0;
constexpr uint32_t LED_SIM_MAX_MS=8000U;
int8_t ledSimChannel=-1; // -1 means all simulated channels off
uint32_t ledSimUntil=0;
String ledSimReason="BOOT_LOCKED";
static const char* const LED_CAPS[BusaLed::kCount]={
 "T1 Water Blue","T2 Column Green","T3 Cube Yellow","T4 Top Red",
 "Installation Green","Installation Red","Valve Green","Valve Red",
 "WiFi Green","WiFi Red","Alarm Red"};
void ledSimStop(const char* reason){
  ledSimChannel=-1;ledSimUntil=0;ledSimReason=reason;
}
void ledSimTick(uint32_t now){
  if(ledSimChannel>=0 && (int32_t)(now-ledSimUntil)>=0)
    ledSimStop("TIME_LIMIT");
  if(ledSimChannel>=0 && WiFi.softAPgetStationNum()==0)
    ledSimStop("AP_CLIENT_LOST");
}
String quoteJson(const String &x); // forward declaration
String ledSimJSON(){
  const uint32_t now=millis();
  ledSimTick(now);
  String o="{\"physical_enabled\":false,\"current_limit_verified\":false";
  o+=",\"owner_confirmed_supply_mv\":"+String(BusaLed::kOwnerConfirmedLedSupplyMv);
  o+=",\"owner_confirmed_individual_leds\":"+(BusaLed::kOwnerConfirmedIndividualLeds?String("true"):String("false"));
  o+=",\"physical_test_ready\":"+(BusaLed::physicalTestPreflight(0)?String("true"):String("false"));
  o+=",\"physical_test_block_reason\":\"CURRENT_LIMIT_AND_OE_UNVERIFIED\"";
  o+=",\"oe_state_verified\":false,\"driver_connected\":"+(pcaDetected?String("true"):String("false"));
  o+=",\"mode1\":"+(pcaMode1>=0?String(pcaMode1):String("null"));
  o+=",\"mode2\":"+(pcaMode2>=0?String(pcaMode2):String("null"));
  o+=",\"read_ok_total\":"+String(pcaLedReadOk)+",\"read_error_total\":"+String(pcaLedReadErrors);
  o+=",\"register_scan_ms\":"+String(pcaLedScanAt);
  o+=",\"sim_channel\":"+String((int)ledSimChannel);
  o+=",\"sim_remaining_ms\":"+String(ledSimChannel>=0?(uint32_t)(ledSimUntil-now):0);
  o+=",\"sim_reason\":"+quoteJson(ledSimReason)+",\"channels\":[";
  for(uint8_t ch=0;ch<BusaLed::kCount;ch++){
    if(ch)o+=",";
    const auto &r=pcaLedRegs[ch];
    o+="{\"ch\":"+String(ch)+",\"name\":"+quoteJson(String(LED_CAPS[ch]));
    o+=",\"virtual_on\":"+(ledSimChannel==ch?String("true"):String("false"));
    o+=",\"real_led_on\":null";
    o+=",\"read_ok\":"+(r.valid?String("true"):String("false"));
    o+=",\"pwm_on_full\":"+String(r.valid?((r.onH&0x10)?"true":"false"):"null");
    o+=",\"pwm_on_count\":"+(r.valid?String(((uint16_t)(r.onH&15)<<8)|r.onL):String("null"));
    o+=",\"pwm_off_full\":"+String(r.valid?((r.offH&0x10)?"true":"false"):"null");
    o+=",\"pwm_off_count\":"+(r.valid?String(((uint16_t)(r.offH&15)<<8)|r.offL):String("null"));
    o+="}";
  }
  return o+"]}";
}
void probePcaLedRegisters(uint32_t now){
  pcaLedScanAt=now;
  for(uint8_t ch=0;ch<BusaLed::kCount;ch++){
    if(!pcaDetected){pcaLedRegs[ch]=BusaLed::ChannelSnapshot();continue;}
    pcaLedRegs[ch]=BusaLed::snapshot(ch);
    if(pcaLedRegs[ch].valid)pcaLedReadOk++;
    else pcaLedReadErrors++;
  }
}

bool pcfIdleReadOk=false;
uint16_t pcfIdleDirect=0xffff;
uint32_t keyCount=0,lastKeyScan=0,pendingAt=0;
int pendingPair=-1;bool keyHeld=false;
String keyMessage="NO DATA";
uint16_t pcfIdleMask=65535,pcfObservedLow=0;
uint8_t pcfPairCount=0,pcfScanError=0;
int8_t pcfConflictDrive=-1;
uint16_t pcfConflictMask=0;
uint32_t pcfError6Total=0,pcfIdleLowTotal=0,pcfValidPairTotal=0;
uint32_t pcfLastScanAt=0;
Preferences prefs;
// Wall clock: phone supplies UTC epoch and local UTC offset via protected AP;
// optional NTP keeps UTC updated when station internet is available.
int32_t clockUtcOffsetMinutes=0;
bool clockZoneKnown=false;
bool clockNtpStarted=false;
bool deviceClockValid(){
  return clockZoneKnown && time(nullptr)>=(time_t)1609459200;
}
String clockStamp(bool compact=false){
  if(!deviceClockValid())return compact?String("--.--.---- --:--"):String("Время не синхронизировано");
  time_t localEpoch=time(nullptr)+(time_t)(clockUtcOffsetMinutes*60);
  struct tm t={};
  gmtime_r(&localEpoch,&t);
  char buf[32];
  if(compact)snprintf(buf,sizeof(buf),"%02d.%02d.%04d %02d:%02d",
     t.tm_mday,t.tm_mon+1,t.tm_year+1900,t.tm_hour,t.tm_min);
  else snprintf(buf,sizeof(buf),"%02d.%02d.%04d %02d:%02d:%02d",
     t.tm_mday,t.tm_mon+1,t.tm_year+1900,t.tm_hour,t.tm_min,t.tm_sec);
  return String(buf);
}
bool ack(uint8_t address){
  Wire.beginTransmission(address);return Wire.endTransmission()==0;
}
// I2C discovery plus read-only register inspection; never modify PWM outputs.
void scanI2cAddresses(uint32_t now){
  if(!i2cOnline)return;
  i2cFoundCount=0;i2cAckCount=0;
  for(uint8_t addr=0x08;addr<=0x77;addr++){
    if(ack(addr)){
      i2cAckCount++;
      if(i2cFoundCount<24)i2cFound[i2cFoundCount++]=addr;
    }
  }
  i2cFullScanAt=now;
}
int readPcaReg(uint8_t reg){
  if(!pcaDetected)return -1;
  Wire.beginTransmission(0x40);
  Wire.write(reg); // Register pointer only, NOT a PWM/output value.
  if(Wire.endTransmission(false)!=0)return -1;
  if(Wire.requestFrom((uint8_t)0x40,(uint8_t)1)!=(uint8_t)1)return -1;
  return Wire.read();
}
void probeI2c(uint32_t now){
  if(!i2cOnline)return;
  pcfDetected=false;pcfAddr=0;
  for(uint8_t i=0x20;i<=0x27;i++)if(ack(i)){
    pcfDetected=true;pcfAddr=i;break;
  }
  pcaDetected=ack(0x40);
  pcaMode1=readPcaReg(0x00);
  pcaMode2=readPcaReg(0x01);
  probePcaLedRegisters(now);
  // PCF8575 keypad inputs: release scanning lines then sample idle state.
  pcfIdleReadOk=false;
  if(pcfDetected){
    Wire.beginTransmission(pcfAddr);
    Wire.write((uint8_t)0xff);Wire.write((uint8_t)0xff);
    if(Wire.endTransmission()==0 &&
       Wire.requestFrom(pcfAddr,(uint8_t)2)==(uint8_t)2){
      const uint8_t lo=(uint8_t)Wire.read();
      const uint8_t hi=(uint8_t)Wire.read();
      pcfIdleDirect=(uint16_t)lo|((uint16_t)hi<<8);
      pcfIdleReadOk=true;
    }
  }
  if(!i2cFullScanAt||elapsed(now,i2cFullScanAt,15000))scanI2cAddresses(now);
  lastBusProbe=now;
}
bool pcfWrite(uint16_t v){
  if(!pcfDetected)return false;
  Wire.beginTransmission(pcfAddr);
  Wire.write((uint8_t)v);Wire.write((uint8_t)(v>>8));
  return Wire.endTransmission()==0;
}
bool pcfRead(uint16_t &v){
  if(!pcfDetected||Wire.requestFrom(pcfAddr,(uint8_t)2)!=(uint8_t)2)return false;
  v=(uint8_t)Wire.read();v|=(uint16_t)((uint8_t)Wire.read())<<8;return true;
}
uint8_t currentPage=0; // Six approved UI pages
uint8_t graphSubView=0; // 0 T1-T4, 1 cooling water, 2 collection no sensor
uint8_t approvedSettingsItem=0; // The six approved settings rows on the physical TFT
uint8_t settingsSubTab=0,tftView=0,previousMainPage=0;
bool tftDirty=true;
float refTemp[4]={NAN,NAN,NAN,NAN},noticeTemp[4]={NAN,NAN,NAN,NAN};
// Informational display preferences. Never used for physical outputs.
int selectedTemp=0,selectedField=0;
bool editingTemp=false;
String editTempBuffer="";
uint8_t editCursor=0;
bool storeTempReference(int sensor,int field,float value){
  if(sensor<0||sensor>3||field<0||field>1||!isfinite(value)||value<0||value>125)return false;
  String key=String(field?"nt":"rf")+String(sensor);
  if(prefs.putFloat(key.c_str(),value)==0)return false;
  (field?noticeTemp[sensor]:refTemp[sensor])=value;
  return true;
}

// Keyboard faceplate layout confirmed by photo. Wiring is learned, not guessed.
const char* const KEY_CAPS[20]={"F1","F2","#","*","1","2","3","UP",
 "4","5","6","DOWN","7","8","9","ESC","LEFT","0","RIGHT","ENT"};
uint8_t keyPairs[20],newPairs[20];
uint8_t learnStep=0;
bool keypadReady=false,keypadLearning=false;
String keypadMessage="Обучение не выполнено",rawPair="—";
int candidatePair=-1;uint32_t candidateAt=0;
uint8_t candidateStep=255;
bool learnCommitPair(int pair); // Shared by auto capture and "Следующая" button

bool validKeymap(const uint8_t *m){
  uint8_t counts[16]={0};
  for(int i=0;i<20;i++){
    uint8_t a=m[i]>>4,b=m[i]&15;
    if(m[i]==255||a>=b)return false;
    for(int j=0;j<i;j++)if(m[i]==m[j])return false;
    counts[a]++;counts[b]++;
  }
  int r=0,c=0;
  for(int i=0;i<16;i++){
    if(counts[i]==4)r++;
    else if(counts[i]==5)c++;
    else if(counts[i])return false;
  }
  return r==5&&c==4;
}
void loadKeymap(){
  memset(keyPairs,255,sizeof(keyPairs));
  memset(newPairs,255,sizeof(newPairs));
  keypadReady=(prefs.getBytesLength("keys20")==sizeof(keyPairs) &&
      prefs.getBytes("keys20",keyPairs,sizeof(keyPairs))==sizeof(keyPairs) &&
      validKeymap(keyPairs));
  if(!keypadReady)memset(keyPairs,255,sizeof(keyPairs));
  keypadMessage=keypadReady?"Раскладка загружена":"Нужно обучить 20 клавиш";
}
// Detect a single closed physical pair on any two PCF8575 P0..P15.
int scanKeyPair(){
  if(!pcfDetected){pcfScanError=1;return -1;}
  pcfLastScanAt=millis();
  pcfScanError=0;pcfPairCount=0;pcfObservedLow=0;pcfConflictDrive=-1;pcfConflictMask=0;
  if(!pcfWrite(65535)){pcfScanError=2;return -2;}
  delayMicroseconds(160);
  if(!pcfRead(pcfIdleMask)){pcfScanError=3;return -2;}
  // Record lines already low with all PCF8575 ports released.
  // Such lines cannot be attributed to a newly driven matrix column.
  const uint16_t idleLow=(uint16_t)~pcfIdleMask;
  pcfObservedLow=idleLow;
  if(idleLow){
    pcfScanError=9; // stuck-low / idle-low wiring or simultaneous contact
    pcfConflictMask=idleLow;pcfIdleLowTotal++;
    pcfWrite(65535);
    return -2;
  }
  uint8_t scanCounts[16]={0};
  if(keypadReady){
    for(int k=0;k<20;k++){
      ++scanCounts[keyPairs[k]>>4];
      ++scanCounts[keyPairs[k]&15];
    }
  }
  int found=-1;bool ambiguous=false;
  for(int a=0;a<16;a++){
    if(keypadReady && scanCounts[a]!=5)continue; // 4 matrix columns only

    if(!pcfWrite((uint16_t)(65535u & ~(1u<<a)))){pcfScanError=4;ambiguous=true;break;}
    delayMicroseconds(160);
    uint16_t v;
    if(!pcfRead(v)){pcfScanError=5;ambiguous=true;break;}
    uint16_t lows=(uint16_t)(~v) & (uint16_t)~(1u<<a);
    if(lows && (lows&(lows-1))){
      // Verify a multi-line reading after settling; never forge a keypress.
      delayMicroseconds(350);
      uint16_t retry=65535;
      if(pcfRead(retry)){
        lows=(uint16_t)(~retry) & (uint16_t)~(1u<<a);
      }
    }
    pcfObservedLow|=lows;
    if(!lows)continue;
    if(lows&(lows-1)){pcfScanError=6;pcfError6Total++;pcfConflictDrive=a;pcfConflictMask=lows;ambiguous=true;break;}
    int b=0;while(b<16 && !(lows&(1u<<b)))b++;
    if(b>=16)continue;
    int p=(min(a,b)<<4)|max(a,b);
    if(found>=0 && found!=p){pcfScanError=7;pcfConflictDrive=a;pcfConflictMask=lows;ambiguous=true;break;}
    if(found<0)pcfPairCount=1;
    found=p;
  }
  if(!pcfWrite(65535)){pcfScanError=8;ambiguous=true;}
  if(!ambiguous&&found>=0)pcfValidPairTotal++;
  return ambiguous?-2:found;
}
String pairName(int p){
  if(p==-2)return "НЕСКОЛЬКО КНОПОК";
  if(p<0)return "НЕТ";
  return "P"+String(p>>4)+"-P"+String(p&15);
}
void startKeyLearning(){
  keypadLearning=true;learnStep=0;
  memset(newPairs,255,sizeof(newPairs));
  pendingPair=-1;keyHeld=false;pendingAt=millis();
  candidatePair=-1;candidateStep=255;candidateAt=0;
  keypadMessage="Нажми F1, затем отпусти";
  // Stay in Settings / System keyboard diagnostics; never jump to Events.
  previousMainPage=currentPage;
  currentPage=5;approvedSettingsItem=4;tftView=1;tftDirty=true;
}

void navigatePage(uint8_t target){
  if(target>=6||target==currentPage)return;
  previousMainPage=currentPage;
  currentPage=target;tftView=0;editingTemp=false;tftDirty=true;
}
void deleteSelectedReference(){
  if(currentPage!=1 || selectedTemp<0 || selectedTemp>3)return;
  // Only local informational reference field is removed; no DS18B20
  // measurement, GPIO configuration, or protective output is touched.
  const char* prefix=selectedField?"nt":"rf";
  String key=String(prefix)+String(selectedTemp);
  prefs.remove(key.c_str());
  if(selectedField)noticeTemp[selectedTemp]=NAN;
  else refTemp[selectedTemp]=NAN;
  keyMessage=String("T")+String(selectedTemp+1)+
    (selectedField?" Пред.":" Зад.")+" - удалено";
}
void moveApprovedTab(int direction); // declaration required by keypad navigation
void darkenTftDisplay(); // manual screen-off via F2 or local iPhone UI
void keypadAction(int k){
  // Confirmed user assignments:
  // UP=7, DOWN=11, LEFT=16, RIGHT=18, ENT=19, ESC=15, F1=0.
  if(editingTemp){
    if(k==0){ // F1: delete the ENTIRE entered value
      editTempBuffer="";editCursor=0;
      keyMessage="Ввод очищен. Ent сохранить, Esc отмена";
    }else if(k==16){ // LEFT
      if(editCursor)editCursor--;
    }else if(k==18){ // RIGHT
      if(editCursor<editTempBuffer.length())editCursor++;
    }else if(k==7){ // UP: move caret to beginning
      editCursor=0;
    }else if(k==11){ // DOWN: move caret to end
      editCursor=(uint8_t)editTempBuffer.length();
    }else if(k==15){ // ESC: cancel and go BACK to selected reference
      editingTemp=false;editTempBuffer="";
    }else if(k==19){ // ENT: OK; accept 0..125 or cleared blank value
      if(editTempBuffer.isEmpty()){
        deleteSelectedReference();
        editingTemp=false;
      }else if(storeTempReference(selectedTemp,selectedField,editTempBuffer.toFloat())){
        editingTemp=false;editTempBuffer="";
        keyMessage="Справочное значение сохранено";
      }else keyMessage="Проверь значение 0..125";
    }else{
      const int digitKey[10]={17,4,5,6,8,9,10,12,13,14};
      String insert="";
      for(int d=0;d<10;d++)if(k==digitKey[d])insert=String(d);
      if(k==1 && editTempBuffer.indexOf('.')<0)insert="."; // F2 decimal separator
      if(insert.length() && editTempBuffer.length()<7){
        editTempBuffer=editTempBuffer.substring(0,editCursor)+
          insert+editTempBuffer.substring(editCursor);
        editCursor=(uint8_t)(editCursor+insert.length());
      }
    }
    tftDirty=true;return;
  }

  // Fixed main menu shortcuts, independent of the four physical arrows.
  if(k==2 || k==3){ // # previous / * next section
    moveApprovedTab(k==2?-1:+1);
  }else if(k==15){ // ESC: BACK (first close detail, then previous screen)
    if(tftView){tftView=0;}
    else if(currentPage!=previousMainPage){
      uint8_t target=previousMainPage;
      previousMainPage=currentPage;
      currentPage=target;
    }else if(currentPage!=0){navigatePage(0);}
  }else if(k==0){ // F1: remove selected informational value
    deleteSelectedReference();
  }else if(k==7 || k==11){ // UP / DOWN move vertically
    if(currentPage==1){
      selectedTemp=(selectedTemp+(k==7?3:1))%4; // sensor row
    }else if(currentPage==5){
      approvedSettingsItem=(approvedSettingsItem+(k==7?6:1))%7;
    }else if(currentPage==3){
      graphSubView=(graphSubView+(k==7?2:1))%3;
    }else{
      tftView=k==7?0:1; // scroll between upper/lower information
    }
  }else if(k==16 || k==18){ // LEFT / RIGHT move horizontally
    if(currentPage==1){
      selectedField=k==16?0:1; // target / warning informational fields
    }else if(currentPage==5){
      approvedSettingsItem=(approvedSettingsItem+(k==16?6:1))%7;
    }else if(currentPage==3){
      graphSubView=(graphSubView+(k==16?2:1))%3;
    }else{
      moveApprovedTab(k==16?-1:+1);
    }
  }else if(k==19){ // ENT: OK / enter selected item
    if(currentPage==1){
      editingTemp=true;
      float v=selectedField?noticeTemp[selectedTemp]:refTemp[selectedTemp];
      editTempBuffer=isfinite(v)?String(v,1):String("");
      editCursor=(uint8_t)editTempBuffer.length();
    }else if(currentPage==0){
      navigatePage(1); // Open temperature cards from home
    }else if(currentPage==5){
      if(tftView && approvedSettingsItem==0)navigatePage(1);
      else tftView=1;
    }else if(currentPage==3){
      graphSubView=(graphSubView+1)%3;
    }else{
      tftView=1; // Open detailed read-only view
    }
  }else if(k==1){ // F2 outside numeric editor: display OFF, monitoring stays active
    darkenTftDisplay();
    keyMessage="TFT погашен (ручной режим)";
  }else if(k==17 || k==4){navigatePage(0);} // 0,1 Home
  else if(k==5){navigatePage(1);selectedTemp=0;} // 2 Temperatures
  else if(k==6){navigatePage(2);} // 3 Collection
  else if(k==8){navigatePage(3);} // 4 Graphs
  else if(k==9){navigatePage(4);} // 5 Events
  else if(k==10){navigatePage(5);} // 6 Settings
  else if(k==12){navigatePage(5);settingsSubTab=0;} // 7 Wi-Fi
  else if(k==13){navigatePage(5);settingsSubTab=1;} // 8 Google
  else if(k==14){navigatePage(4);} // 9 Diagnostics
  tftDirty=true;
}


bool learnCommitPair(int pair){
  if(!keypadLearning||pair<0||pair>255||learnStep>=20)return false;
  for(int n=0;n<learnStep;n++)if(newPairs[n]==pair){
    keypadMessage="Контакты уже записаны: "+pairName(pair)+
      ". Повтори "+String(KEY_CAPS[learnStep]);
    keyMessage=keypadMessage;
    tftDirty=true;
    return false;
  }
  newPairs[learnStep]=pair;
  keyMessage=String(KEY_CAPS[learnStep])+" = "+pairName(pair);
  learnStep++;
  candidatePair=-1;candidateStep=255;
  if(learnStep==20){
    if(validKeymap(newPairs) &&
      prefs.putBytes("keys20",newPairs,sizeof(newPairs))==sizeof(newPairs)){
      memcpy(keyPairs,newPairs,sizeof(keyPairs));
      keypadReady=true;keypadLearning=false;
      keypadMessage="Все 20 клавиш сохранены";
    }else{
      learnStep=19;newPairs[19]=255;
      keypadMessage="Матрица не прошла проверку. Повтори Ent";
    }
  }else{
    keypadMessage="Далее: "+String(KEY_CAPS[learnStep])+
       " ("+String(learnStep+1)+"/20)";
  }
  tftDirty=true;
  return true;
}
void wakeTftDisplay(); // Wake works even when the 5x4 keymap is untrained.
bool tftDisplayOff=false;
uint32_t tftDisplayActivityMs=0;
// No inactivity auto-off. Only F2 or explicit iPhone screen button blanks TFT.
void pollKeys(uint32_t now){
  if(!pcfDetected||!elapsed(now,lastKeyScan,60))return;
  lastKeyScan=now;
  int pair=scanKeyPair();
  if(pair>=0){
    rawPair=pairName(pair);
    if(keypadLearning && (pair!=candidatePair||candidateStep!=learnStep)){
      candidatePair=pair;candidateStep=learnStep;candidateAt=now;
    }
  }else if(pair==-2){
    rawPair="НЕОДНОЗНАЧНО: проверь кнопку";
  }else{
    rawPair="НЕТ"; // V9 kept stale ambiguous pairs after release
  }
  if(pair!=pendingPair){pendingPair=pair;pendingAt=now;keyHeld=false;}
  if(pair<0||keyHeld||!elapsed(now,pendingAt,80))return;
  keyHeld=true;
  keyCount++;
  if(tftDisplayOff){
    // First recognized electrical key press only wakes the TFT.
    // No menu action, setpoint edit, or output action on wake.
    wakeTftDisplay();
    keyMessage="Экран пробуждён";
    return;
  }
  tftDisplayActivityMs=now;
  if(keypadLearning){
    learnCommitPair(pair);
  }else{
    int k=-1;
    if(keypadReady)for(int i=0;i<20;i++)if(keyPairs[i]==pair){k=i;break;}
    if(k<0)keyMessage="Не назначено: "+rawPair;
    else{keyMessage=String(KEY_CAPS[k])+" ("+rawPair+")";keypadAction(k);}
  }
  tftDirty=true;
}

Adafruit_ST7789 screenSt(TFT_CS,TFT_DC,TFT_RST);
Adafruit_ILI9341 screenIli(TFT_CS,TFT_DC,TFT_RST);
Adafruit_GFX* screen=nullptr;
U8G2_FOR_ADAFRUIT_GFX ru;
uint8_t tftProfile=1;
bool tftStarted=false;
bool tftInverted=false; // Hardware inversion remains DISABLED
bool tftThemeDark=false; // Real saved software color palette
uint32_t splashStartedAt=0,splashHoldMs=1700;
uint8_t tftRotation=0; // V13 approved default: 240x320 portrait; rotation 2 is upside-down
uint8_t bootStage=0; // 0 TFT, 1 WiFi, 2 modules, 3 1-Wire, 4 ready
bool bootSplashComplete=false;
bool tftRebootPending=false;
uint32_t tftRebootAt=0;
const char* tftProfileName(){return tftProfile==2?"ILI9341":"ST7789";}
const uint16_t BG=0x080f,PANEL=0x10a3,BLUE=0x1c9f,GREEN=0x07e0,WHITE=0xffff,
               YELLOW=0xffe0,GRAY=0xbdf7;
const char* pages[6]={"Главная","Температуры","Отбор","Графики","События","Настройки"};
const uint8_t UI_ORDER[6]={0,1,2,5,3,4}; // Main T Collection Settings Graphs Events
void navigatePage(uint8_t target); // forward declaration
uint8_t orderSlot(uint8_t page){for(uint8_t i=0;i<6;i++)if(UI_ORDER[i]==page)return i;return 0;}
void moveApprovedTab(int direction){int slot=(orderSlot(currentPage)+6+direction)%6;navigatePage(UI_ORDER[slot]);}
// Defined below with the web server; needed by the TFT renderer.
extern String stationSsid,otaMessage,nearbyNames[5];
extern uint8_t nearbyCount;
void ruText(const String &v,int x,int y,uint16_t color=WHITE){
  // Explicit UTF-8 glyph renderer; transparent background, 0-degree orientation.
  // Avoid Print::write byte-by-byte path and opaque font-cell rectangles.
  ru.setFontMode(1);
  ru.setFontDirection(0);
  ru.setForegroundColor(color);
  ru.drawUTF8(x,y,v.c_str());
}

void panelAt(int x,int y,int w,int h){
  screen->fillRoundRect(x,y,w,h,5,PANEL);
  screen->drawRoundRect(x,y,w,h,5,BLUE);
}
// Cache 24 small fields so unchanged values are not erased and redrawn.
struct TftField {
  int16_t x=0,y=0,w=0;
  uint16_t color=0;
  String value;
};
TftField tftFields[24];
uint8_t tftFieldCount=0;
void screenField(const String &v,int x,int y,int w,uint16_t color=WHITE){
  int found=-1;
  for(int i=0;i<tftFieldCount;i++){
    if(tftFields[i].x==x && tftFields[i].y==y && tftFields[i].w==w){
      found=i;break;
    }
  }
  if(found>=0){
    if(tftFields[found].value==v && tftFields[found].color==color)return;
  }else if(tftFieldCount<24){
    found=tftFieldCount++;
    tftFields[found].x=x;
    tftFields[found].y=y;
    tftFields[found].w=w;
  }
  if(found>=0){
    tftFields[found].value=v;
    tftFields[found].color=color;
  }
  screen->fillRect(x,y-13,w,16,PANEL);
  ruText(v,x,y,color);
}
void updateBootStage(uint8_t stage){
  if(!screen||!tftStarted)return;
  bootStage=stage>4?4:stage;
  const int16_t w=screen->width(),h=screen->height();
  // Only update the footer. The white logo on black stays stationary.
  const int16_t y=h-58;
  screen->fillRect(0,y,w,58,0x0000);
  screen->setTextColor(0xffff);
  screen->setTextSize(1);
  screen->setTextWrap(false);
  const char* steps[]={"TFT INIT","WIFI INITIALIZING","MODULES CHECK","SENSOR BUS SCAN","READY"};
  String status=String("BOOT: ")+steps[bootStage];
  screen->setCursor(max(0,(w-(int)status.length()*6)/2),y+5);
  screen->print(status);
  const String version="FW V14.1 LIGHT";
  screen->setCursor(max(0,(w-(int)version.length()*6)/2),y+19);
  screen->print(version);
  const int16_t width=w-46;
  screen->drawRect(22,y+38,width+2,10,0xffff);
  const int16_t filled=(int16_t)((int32_t)width*bootStage/4);
  if(filled>0)screen->fillRect(23,y+39,filled,8,0xffff);
}
void drawBootSplash(uint32_t holdMs){
  if(!screen)return;
  const int16_t w=screen->width(),h=screen->height();
  screen->fillScreen(0x0000);
  screen->setTextColor(0xffff);
  screen->setTextWrap(false);
  const int scale=w>=300?3:2;
  screen->setTextSize(scale);
  screen->setCursor((w-10*6*scale)/2,h/2-50);
  screen->print("BUSA 1M AI");
  screen->setTextSize(2);
  screen->setCursor((w-36)/2,h/2+3);
  screen->print("001");
  screen->setTextSize(1);
  splashHoldMs=holdMs;
  splashStartedAt=millis();
  tftDirty=true;
  tftFieldCount=0;
  updateBootStage(bootStage);
}
void startScreen(){
  Serial0.println("TFT: HW SPI landscape; controller model unverified");
  SPI.begin(TFT_SCK,-1,TFT_MOSI,TFT_CS);
  SPI.setFrequency(16000000); // moderate stable starting frequency
  if(tftProfile==2){screenIli.begin();screen=&screenIli;}
  else{screenSt.init(240,320);screen=&screenSt;}
  screen->setRotation(tftRotation); // 0..3 selected from iPhone
  screen->invertDisplay(tftInverted);
  ru.begin(*screen);
  ru.setFont(u8g2_font_6x13_t_cyrillic);
  ru.setFontMode(1);
  tftStarted=true;
  tftDisplayOff=false;tftDisplayActivityMs=millis();
  drawBootSplash(8000); // Wi-Fi AP + web start while splash visible; max 8 seconds
  Serial0.printf("TFT init command sent: %s (pixels not yet confirmed)\n",tftProfileName());
}
// Display-only standby; does NOT sleep ESP32, alter sensors, Wi-Fi or GPIO.
void wakeTftDisplay(){
  if(!tftStarted)return;
  screenSt.enableDisplay(true);
  tftDisplayOff=false;
  tftDisplayActivityMs=millis();
  tftDirty=true;
}
void darkenTftDisplay(){
  if(!tftStarted||tftDisplayOff)return;
  screenSt.enableDisplay(false);
  tftDisplayOff=true;
}
// Note: a separately wired always-on TFT backlight may still glow in display-off.
 // Portrait-specific layout prevents 320px-wide landscape cards from clipping at 240px.
#include "BUSA_ApprovedUI_V13.h"

#include "BUSA_ApprovedGraphics_HQ_V14.h"
#include "BUSA_Landscape_V14.h"
void renderScreen(uint32_t now){
  if(!tftStarted||!screen)return;
  if(!bootSplashComplete || !elapsed(now,splashStartedAt,splashHoldMs))return;
  if(screen->width()>=300){renderLandscape(now);return;}
  renderPortrait(now);
}

WebServer server(80);
volatile bool otaActive=false;
bool otaSuccess=false,otaRestart=false,otaWriteAllowed=false,otaUploadCompleted=false;
uint32_t otaExpectedBytes=0;
uint8_t otaPrefix[36]={0};
size_t otaPrefixUsed=0;
#include "BUSA_AppHeader.h"
uint32_t otaBytes=0,otaRestartAt=0,otaTokenIssued=0,startedAt=0;
String otaToken="",otaMessage="Ожидание",nearbyNames[5];
uint8_t nearbyCount=0;
bool screenAttempted=false;
bool apClient(){return server.client().localIP()==WiFi.softAPIP();}

String apPassword="";
bool wifiRunning=false;
uint32_t lastRender=0,lastUart=0;
uint32_t webMaxServiceMs=0,loopMaxGapMs=0,lastLoopStart=0;
String consoleLine="",stationSsid="",stationPass="";
String quoteJson(const String &x){
  String out="\"";
  for(unsigned i=0;i<x.length();i++){
    char c=x[i];
    if(c=='\"'||c=='\\'){out+='\\';out+=c;}
    else if(c=='\n')out+="\\n";
    else if(c=='\r')out+="\\r";
    else if((unsigned char)c<32){char escaped[7];snprintf(escaped,sizeof(escaped),"\\u%04x",(unsigned char)c);out+=escaped;}
    else out+=c;
  }
  return out+"\"";
}
#include "BUSA_WifiRecovery.h"
// Safety-bounded virtual relay diagnostics ONLY. No GPIO writes, no PCA PWM.
// This mode never energizes SRD-12VDC-SL-C coils; there is no feedback sensor.
static_assert(!RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,
              "Dry-run build cannot enable external control");
constexpr uint32_t RELAY_SIM_MAX_MS=10000U;
uint8_t relaySimChannel=0;
uint32_t relaySimStart=0,relaySimUntil=0;
String relaySimReason="IDLE";
void relaySimStop(const char* why){
  relaySimChannel=0;relaySimStart=0;relaySimUntil=0;
  relaySimReason=why;
}
void relaySimTick(uint32_t now){
  if(relaySimChannel && (int32_t)(now-relaySimUntil)>=0){
    relaySimStop("TIME_LIMIT");
  }
  if(relaySimChannel && WiFi.softAPgetStationNum()==0){
    relaySimStop("AP_CLIENT_LOST");
  }
}
String relaySimJSON(){
  relaySimTick(millis());
  String s="{\"dry_run_only\":true,\"physical_gpio_enabled\":false";
  s+=",\"pca_pwm_enabled\":false,\"coil_12v_measured\":false";
  s+=",\"contact_feedback_available\":false,\"physical_contacts\":null";
  s+=",\"simulated_channel\":"+String(relaySimChannel);
  s+=",\"remaining_ms\":"+String(relaySimChannel?(uint32_t)(relaySimUntil-millis()):0);
  s+=",\"reason\":"+quoteJson(relaySimReason);
  s+=",\"simulated_channels\":[";
  for(int i=0;i<4;i++){if(i)s+=",";s+=(relaySimChannel==i+1?"true":"false");}
  return s+"]}";
}
// Read-only diagnostic of the ACTUAL installed ESP32 OTA partitions.
// Not an authorization to flash: presence/size cannot prove image signatures,
// compatibility of the current bootloader or a working rollback policy.
extern volatile uint32_t cloudReleaseSize; // declared in BUSA_Google.h
String otaPartitionPreflightJSON(){
  const esp_partition_t* running=esp_ota_get_running_partition();
  const esp_partition_t* next=esp_ota_get_next_update_partition(nullptr);
  const esp_partition_t* boot=esp_ota_get_boot_partition();
  const uint32_t candidate=cloudReleaseSize;
  const bool hasSlot=running && next && (running->address!=next->address);
  const bool fits=hasSlot && candidate>=60000U && candidate<=next->size;
  String out="{\"running_address\":";
  out+=running?String((uint32_t)running->address):String("null");
  out+=",\"running_size\":"+(running?String((uint32_t)running->size):String("null"));
  out+=",\"next_address\":"+(next?String((uint32_t)next->address):String("null"));
  out+=",\"next_size\":"+(next?String((uint32_t)next->size):String("null"));
  out+=",\"boot_address\":"+(boot?String((uint32_t)boot->address):String("null"));
  out+=",\"slot_available\":"+(hasSlot?String("true"):String("false"));
  out+=",\"candidate_fits_slot\":"+(fits?String("true"):String("false"));
  out+=",\"signed_image_verified\":false,\"rollback_tested\":false,\"remote_install_enabled\":false}";
  return out;
}
String otaJSON(); // forward declaration
String cloudJSON(); // forward declaration; read-only Google status
extern volatile int googleInternet; // external TCP health, no credentials
String statusJSON(){
  uint32_t now=millis();
  String s="{\"device\":\"BUSA-1M-001\",\"firmware\":\""+String(VERSION)+"\"";
  s+=",\"mode\":\"LOW_VOLTAGE_MONITOR_ONLY\",\"relays_enabled\":false,\"pwm_enabled\":false";
  s+=",\"leds\":"+ledSimJSON();
  s+=",\"relay_simulation\":"+relaySimJSON();
  s+=",\"configuration\":{\"manual_external_controls\":true,\"mains_control_in_project\":false,\"motorized_valve_in_project\":false}";
  s+=",\"ui_page\":"+String(currentPage+1)+",\"ui_subtab\":"+String(settingsSubTab)+",\"tft_view\":"+String(tftView)+",\"uptime_ms\":"+String(now);
  s+=",\"clock\":{\"synchronized\":"+(deviceClockValid()?String("true"):String("false"));
  s+=",\"datetime\":"+quoteJson(clockStamp(false));
  s+=",\"utc_offset_minutes\":"+String(clockUtcOffsetMinutes)+"}";
  s+=",\"display_references\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"target\":"+(isfinite(refTemp[i])?String(refTemp[i],1):String("null"));
    s+=",\"notice\":"+(isfinite(noticeTemp[i])?String(noticeTemp[i],1):String("null"))+"}";
  }
  s+="]";
  s+=",\"temperatures\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"name\":"+quoteJson(String(i==0?"Вода":i==1?"Колонна":i==2?"Куб":"Верх колонны"));
    s+=",\"gpio\":"+String(TPIN[tempBusForLogical(i)])+",\"present\":"+(tempPresent[i]?"true":"false");
    s+=",\"verified\":";s+=(hasTemp(i,now)?"true":"false");
    s+=",\"error\":"+quoteJson(wireError(i,now));
    s+=",\"binding\":"+quoteJson(tempBindings[i].explicitRom?String(tempBindings[i].bus)+":"+romHex(tempBindings[i].rom):String("auto"));
    s+=",\"sample_age_ms\":"+(hasTemp(i,now)?String((uint32_t)(now-tempAt[i])):String("null"));
    s+=",\"c\":"+(hasTemp(i,now)?String(temperature[i],2):String("null"))+"}";
  }
  s+="],\"water\":{\"gpio\":15,\"pulses\":"+String(flowPulses());
  s+=",\"lpm\":"+(flowReady && waterCalibrated && isfinite(flowLpm)?String(flowLpm,3):String("null"));
  s+=",\"total_l_estimate\":"+(waterCalibrated?String(totalWaterLitres(),3):String("null"));
  s+=",\"calibrated\":"+(waterCalibrated?String("true"):String("false"));
  s+=",\"calibration\":"+waterCalJSON();
  s+=",\"seen_pulses\":"+(flowSeen?String("true"):String("false"))+"}";
  s+=",\"collection\":{\"sensor_present\":false,\"ml_per_min\":null,\"total_ml\":null}";
  s+=",\"modules\":{\"pcf8575\":"+(pcfDetected?String("true"):String("false"));
  s+=",\"pca9685_ack_only\":"+(pcaDetected?String("true"):String("false"));
  s+=",\"tft_attempted\":"+(tftStarted?String("true"):String("false"));
  s+=",\"tft_rotation\":"+String(tftRotation);
  s+=",\"tft_boot_complete\":"+(bootSplashComplete?String("true"):String("false"));
  s+=",\"tft_profile_id\":"+String(tftProfile);
  s+=",\"tft_profile\":"+quoteJson(tftProfileName());
  s+=",\"tft_physical_pixels_confirmed\":false";
  s+=",\"last_key\":"+quoteJson(keyMessage)+"}";
  s+=",\"tft_theme_inverted\":"+(tftInverted?String("true"):String("false"));
  s+=",\"tft_theme_dark\":"+(tftThemeDark?String("true"):String("false"));
  s+=",\"tft_theme_mode\":\""+String(tftThemeDark?"dark":"light")+"\"";
  s+=",\"display_sleep\":"+(tftDisplayOff?String("true"):String("false"));
  s+=",\"keypad\":{\"ready\":"+(keypadReady?String("true"):String("false"));
  s+=",\"learning\":"+(keypadLearning?String("true"):String("false"));
  s+=",\"step\":"+String(learnStep);
  s+=",\"expected\":"+quoteJson(keypadLearning&&learnStep<20?String(KEY_CAPS[learnStep]):String(""));
  s+=",\"raw_pair\":"+quoteJson(rawPair);
  s+=",\"message\":"+quoteJson(keypadMessage)+"}";
  s+=",\"temperature_inventory\":"+temperatureInventoryJSON(now);
  s+=",\"diagnostics\":{\"onewire\":"+temperatureDiagnosticsJSON(now);
  s+=",\"keyboard\":{\"pcf_address\":"+String(pcfAddr);
  s+=",\"last_scan_ms\":"+String(pcfLastScanAt);
  s+=",\"idle_hex\":"+quoteJson(String(pcfIdleMask,HEX));
  s+=",\"low_hex\":"+quoteJson(String(pcfObservedLow,HEX));
  s+=",\"pairs\":"+String(pcfPairCount);
  s+=",\"scan_error\":"+String(pcfScanError);
  s+=",\"conflict_drive\":"+String((int)pcfConflictDrive);
  s+=",\"conflict_mask_hex\":"+quoteJson(String(pcfConflictMask,HEX));
  s+=",\"error6_total\":"+String(pcfError6Total);
  s+=",\"idle_low_total\":"+String(pcfIdleLowTotal);
  s+=",\"valid_pair_total\":"+String(pcfValidPairTotal);
  s+=",\"raw_pair\":"+quoteJson(rawPair)+"}";
  s+=",\"i2c\":{\"sda_gpio\":8,\"scl_gpio\":9,\"controller_started\":"+(i2cOnline?String("true"):String("false"));
  s+=",\"last_scan_ms\":"+String(i2cFullScanAt);
  s+=",\"ack_total\":"+String(i2cAckCount)+",\"addresses\":[";
  for(uint8_t i=0;i<i2cFoundCount;i++){if(i)s+=",";s+=String(i2cFound[i]);}
  s+="]}";
  s+=",\"pcf8575\":{\"expected_address\":32,\"online\":"+(pcfDetected?String("true"):String("false"));
  s+=",\"address\":"+String(pcfAddr);
  s+=",\"read_ok\":"+(pcfIdleReadOk?String("true"):String("false"));
  s+=",\"idle_pins_hex\":"+quoteJson(String(pcfIdleDirect,HEX));
  s+=",\"keyscan_error\":"+String(pcfScanError)+"}";
  s+=",\"pca9685\":{\"expected_address\":64,\"ack\":"+(pcaDetected?String("true"):String("false"));
  s+=",\"mode1\":"+String(pcaMode1)+",\"mode2\":"+String(pcaMode2);
  s+=",\"outputs_enabled\":false}";
  s+=",\"tft\":{\"spi_cs\":10,\"spi_mosi\":11,\"spi_sck\":12,\"dc\":13,\"rst\":14";
  s+=",\"profile\":"+quoteJson(tftProfileName())+",\"rotation\":"+String(tftRotation);
  s+=",\"init_sent\":"+(tftStarted?String("true"):String("false"));
  s+=",\"boot_complete\":"+(bootSplashComplete?String("true"):String("false"));
  s+=",\"pixel_feedback_available\":false}";
  s+=",\"water_flow\":{\"gpio\":15,\"input_level\":"+String(digitalRead(FLOW_GPIO));
  s+=",\"pulses\":"+String(flowPulses())+",\"pulse_seen\":"+(flowSeen?String("true"):String("false"));
  s+=",\"frequency_calibrated\":"+(waterCalibrated?String("true"):String("false"));
  s+=",\"status\":"+quoteJson(flowSeen?"PULSES_SEEN":"WAITING_FOR_PULSES")+"}";
  s+=",\"esp32\":{\"chip\":"+quoteJson(ESP.getChipModel());
  s+=",\"free_heap_bytes\":"+String(ESP.getFreeHeap());
  s+=",\"min_free_heap_bytes\":"+String(ESP.getMinFreeHeap());
  s+=",\"flash_bytes\":"+String(ESP.getFlashChipSize());
  s+=",\"reset_reason_code\":"+String((int)esp_reset_reason());
  s+=",\"uptime_ms\":"+String(now)+"}";
  s+=",\"wifi\":{\"ap_running\":"+(wifiRunning?String("true"):String("false"));
  s+=",\"ap_clients\":"+String(WiFi.softAPgetStationNum());
  s+=",\"sta_connected\":"+(WiFi.status()==WL_CONNECTED?String("true"):String("false"));
  s+=",\"sta_rssi_dbm\":"+(WiFi.status()==WL_CONNECTED?String(WiFi.RSSI()):String("null"));
  s+=",\"station_credentials_protected\":true}";
  s+="}";
  s+=",\"ap\":{\"enabled\":"+(wifiRunning?String("true"):String("false"));
  s+=",\"ip\":"+quoteJson(WiFi.softAPIP().toString())+"}";
  s+=",\"ota\":"+otaJSON();
  s+=",\"ota_preflight\":"+otaPartitionPreflightJSON();
  s+=",\"bluetooth_enabled\":false";
  s+=",\"google\":"+cloudJSON();
  s+=",\"network_indicators\":{\"internet\":"+(googleInternet==1?String("true"):String("false"));
  s+=",\"local_client\":"+(WiFi.softAPgetStationNum()>0?String("true"):String("false"));
  s+=",\"ap_running\":"+(wifiRunning?String("true"):String("false"))+"}";
  s+=",\"wifi_recovery\":"+wifiRecoveryJSON();
  s+=",\"web_health\":{\"max_service_ms\":"+String(webMaxServiceMs)+",\"max_loop_gap_ms\":"+String(loopMaxGapMs)+"}";
  s+=",\"station\":{\"ssid\":"+quoteJson(WiFi.status()==WL_CONNECTED?WiFi.SSID():stationSsid);
  s+=",\"target_ssid\":"+quoteJson(stationSsid);
  s+=",\"connected\":"+(WiFi.status()==WL_CONNECTED?String("true"):String("false"));
  s+=",\"ip\":"+quoteJson(WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():String(""))+"}";
  s+="}";
  return s;
}
// Manual local-only OTA; cloud/automatic OTA is NOT enabled.
String otaJSON(){
 return String("{\"active\":")+(otaActive?"true":"false")
   +",\"success\":"+(otaSuccess?"true":"false")
   +",\"bytes\":"+String(otaBytes)+",\"message\":"+quoteJson(otaMessage)+"}";
}
void uploadOTA(){
 HTTPUpload &u=server.upload();
 if(u.status==UPLOAD_FILE_START){
  otaWriteAllowed=false;otaActive=false;otaSuccess=false;otaBytes=0;otaPrefixUsed=0;otaUploadCompleted=false;
  if(otaRestart||wifiScanBusy()||otaExpectedBytes<60000||!apClient()||otaToken.length()<16||(uint32_t)(millis()-otaTokenIssued)>90000
     ||server.header("X-Busa-Token")!=otaToken||!u.filename.endsWith(".bin")){
    otaMessage="Отказ: локальная авторизация / формат BIN";return;
  }
  otaToken="";
  if(!Update.begin(otaExpectedBytes,U_FLASH)){
    otaMessage="Не найден OTA-раздел";Update.printError(Serial0);return;
  }
  otaWriteAllowed=true;otaActive=true;otaMessage="Записывается";
 }else if(u.status==UPLOAD_FILE_WRITE){
  if(!otaWriteAllowed)return;
  if(u.currentSize>otaExpectedBytes-otaBytes){
    otaMessage="Размер больше разрешённого";Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  size_t used=0;
  if(otaPrefixUsed<sizeof(otaPrefix)){
    const size_t need=sizeof(otaPrefix)-otaPrefixUsed;
    used=u.currentSize<need?u.currentSize:need;
    memcpy(otaPrefix+otaPrefixUsed,u.buf,used);otaPrefixUsed+=used;
    if(otaPrefixUsed==sizeof(otaPrefix)){
      if(!busaAppHeaderValid(otaPrefix,sizeof(otaPrefix))){
        otaMessage="Нужен APP ESP32-S3, USB MERGED запрещён";
        Update.abort();otaActive=false;otaWriteAllowed=false;return;
      }
      if(Update.write(otaPrefix,sizeof(otaPrefix))!=sizeof(otaPrefix)){
        otaMessage="Ошибка записи заголовка";Update.abort();otaActive=false;otaWriteAllowed=false;return;
      }
    }
  }
  if(u.currentSize>used && Update.write(u.buf+used,u.currentSize-used)!=u.currentSize-used){
    otaMessage="Ошибка записи";Update.printError(Serial0);
    Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  otaBytes+=u.currentSize;
 }else if(u.status==UPLOAD_FILE_END){
  if(!otaWriteAllowed)return;
  otaSuccess=otaBytes==otaExpectedBytes && otaPrefixUsed==sizeof(otaPrefix) && Update.end(false);
  if(!otaSuccess)Update.abort();
  otaUploadCompleted=true;otaActive=false;otaWriteAllowed=false;
  otaMessage=otaSuccess?"Успешно, перезагрузка":"BIN не прошёл проверку";
  if(!otaSuccess)Update.printError(Serial0);
  if(otaSuccess){otaRestart=true;otaRestartAt=millis();}
 }else if(u.status==UPLOAD_FILE_ABORTED){
  if(otaWriteAllowed)Update.abort();
  otaWriteAllowed=false;otaActive=false;otaMessage="Отменено";
 }
}
#include "BUSA_Google.h"
const char WEB_PAGE[] PROGMEM=R"HTMLBUSA(
<!doctype html><html lang="ru"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>БУСА-1М №001</title>
<style>
:root{color-scheme:light;--ink:#263d50;--muted:#667989;--paper:#faf9f5;--surface:#fff;--line:#dee4e7;--sea:#0b7da6;--orange:#e39952;--warn:#a84225}
*{box-sizing:border-box}
body{font:15px -apple-system,BlinkMacSystemFont,system-ui,sans-serif;margin:0;padding:0 13px 25px;background:var(--paper);color:var(--ink)}
header{position:sticky;top:0;z-index:5;background:#fff;padding:13px 0 8px;border-bottom:2px solid var(--orange);box-shadow:0 2px 13px #344f6511}
h1{font-size:20px;color:#2b4357;margin:0 0 9px;letter-spacing:.2px}
h2{font-size:17px;color:#284a5c;margin:2px 0 13px}
h3{font-size:14px;color:#38586b;margin:20px 0 8px}
nav{display:flex;overflow-x:auto;gap:5px;padding-bottom:5px;scrollbar-width:none}
nav::-webkit-scrollbar{display:none}
button{font:inherit;cursor:pointer;border:1px solid #b8c8ce;background:#f9fcfd;color:#254456;border-radius:10px;padding:10px}
button:disabled{opacity:.65}
nav button{font-size:13px;white-space:nowrap;min-width:78px;padding:11px 8px;border-radius:9px}
nav .on{background:#f7c58f;border-color:#e4a064;color:#3c342a;font-weight:700}
main{max-width:680px;margin:auto;padding-bottom:23px}
.card{background:#fff;border:1px solid var(--line);border-radius:14px;padding:14px;margin:12px 0;box-shadow:0 3px 12px #4260780b}
.card h2{margin-top:1px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:9px}
.tile{background:#f7fafb;border:1px solid #e0e9ed;border-radius:12px;padding:10px;min-height:82px}
.tile small{display:block;color:#496575;font-size:12px}
.tile strong{display:block;color:#168165;margin:7px 0 2px;font-size:20px}
.tempDot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
.t1{background:#079ac1}.t2{background:#d87a37}.t3{background:#ca5143}.t4{background:#42a175}
.row{display:flex;justify-content:space-between;border-bottom:1px solid #e5ebed;padding:9px 0;gap:9px;align-items:center}
.row:last-child{border:0}.row b{text-align:right;overflow-wrap:anywhere}
.green{color:#127b5a}.warn{color:var(--warn)}.hint{color:var(--muted);font-size:12px;line-height:1.4}
.page[hidden],.settingCard[hidden],.graphPanel[hidden]{display:none!important}
.settingsTabs{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px;margin:12px 0}
.settingsTabs button.on{background:#f7c58f;border-color:#db9a52;font-weight:700}
input,select{width:100%;background:#fff;border:1px solid #b2c4cf;color:#263d50;font:inherit;padding:12px;border-radius:9px}
label{display:block;margin:12px 0 5px}.full{width:100%;margin:12px 0}
#response{color:#a74429;white-space:pre-wrap}
#appHealth{font-size:12px;padding:8px 12px;margin:7px 0;border-radius:9px;background:#e9f8f2;color:#11684e}
#appHealth.failed{background:#fff0e9;color:#9a371e}
canvas{width:100%;height:180px;border:1px solid #d2dee5;border-radius:10px;background:#fff}
pre,textarea{background:#f5f7f8;color:#253d50;border:1px solid #d3dfe4;border-radius:9px;padding:8px}
a{color:#0a709a}
.homeHero{padding:17px 16px 12px;margin-top:12px;background:linear-gradient(125deg,#fff6e6,#eaf7fb);border:1px solid #e8d8c0;border-radius:14px}
.heroEyebrow{font-size:10px;letter-spacing:1.1px;font-weight:700;color:#98622c}
.homeHero h2{font-size:25px;margin:7px 0}
.heroStatus{display:flex;justify-content:space-between;gap:9px;font-size:12px;flex-wrap:wrap;color:#2c655b}
.rigCard svg{display:block;margin:5px auto}
.homeMini>.card{margin:0;min-width:0}.homeMini h2{font-size:14px}.homeMini .row{flex-wrap:wrap;font-size:12px}
.relayGrid{display:grid;grid-template-columns:repeat(4,1fr);gap:7px}
.relayGrid span{padding:9px 4px;background:#fff5eb;border:1px solid #ead4bf;border-radius:9px;text-align:center;font-size:12px}
.relayGrid b{display:block;font-size:11px;margin-top:4px;color:#ad5835}
#clockDisplay{color:#627989!important;font-size:12px!important}
@media(max-width:380px){.homeMini{grid-template-columns:1fr}nav button{min-width:71px;padding:9px 6px}.tile strong{font-size:18px}}

/* Vibrant, explicitly colored dark mode. Never invert whole document/pixels. */
:root[data-theme="dark"]{color-scheme:dark;--ink:#f0f7ff;--muted:#b3ceda;--paper:#0b1928;--surface:#152a3d;--line:#34546a;--sea:#61c5ee;--orange:#ffa861;--warn:#ffa27b}
:root[data-theme="dark"] body{background:var(--paper);color:var(--ink)}
:root[data-theme="dark"] header,:root[data-theme="dark"] nav{background:#112436;border-color:#f1a45b}
:root[data-theme="dark"] h1,:root[data-theme="dark"] h2,:root[data-theme="dark"] h3{color:#e4f2fa}
:root[data-theme="dark"] button{background:#264559;color:#d8f1ff;border-color:#4b7186}
:root[data-theme="dark"] nav button.on,:root[data-theme="dark"] .settingsTabs button.on{background:#e18b3d;color:#142534;border-color:#ffbd77}
:root[data-theme="dark"] .card{background:#172b40;border-color:#3d657a;box-shadow:none}
:root[data-theme="dark"] .tile,:root[data-theme="dark"] pre,:root[data-theme="dark"] textarea{background:#1e3b50;color:#e4f4ff;border-color:#406982}
:root[data-theme="dark"] .tile small,:root[data-theme="dark"] .row,:root[data-theme="dark"] .hint{color:#c0d5df}
:root[data-theme="dark"] .tile strong,:root[data-theme="dark"] .green{color:#70e5b7}
:root[data-theme="dark"] .warn{color:#ff9b71}
:root[data-theme="dark"] input,:root[data-theme="dark"] select{background:#142a3e;color:#fff;border-color:#557f97}
:root[data-theme="dark"] .homeHero{background:linear-gradient(125deg,#43301d,#17364b);border-color:#a37243}
:root[data-theme="dark"] .heroEyebrow{color:#ffd291}
:root[data-theme="dark"] .heroStatus{color:#7ee4cb}
:root[data-theme="dark"] .relayGrid span{background:#463225;color:#fff0d9;border-color:#b27b51}
:root[data-theme="dark"] .relayGrid b{color:#ffad84}
:root[data-theme="dark"] a{color:#85d4ff}
:root[data-theme="dark"] #clockDisplay{color:#b3ceda!important}
:root[data-theme="dark"] canvas{background:#183044;border-color:#416b83}

/* V14.5 PHONE LAYOUT: mobile app shell, no horizontal clipping */
html,body{width:100%;max-width:100%;overflow-x:clip}
body{padding:0!important;padding-bottom:90px!important;-webkit-text-size-adjust:100%;background:var(--paper)}
.appHeader{position:sticky;top:0;z-index:9;padding:10px 12px 8px!important;box-shadow:0 2px 10px #25384b12}
.headerRow{display:flex;align-items:center;justify-content:space-between;gap:8px;max-width:620px;margin:auto}
.brandSmall{font-size:10px;letter-spacing:1px;font-weight:700;color:#ce8238}
.headerRow h1{margin:1px 0 5px;font-size:20px}
.brandSub{font-size:13px;font-weight:500;color:var(--muted)}
.readOnlyLabel{font-size:10px;color:#20866c;font-weight:700;flex-shrink:0}
.clockBar{max-width:620px;margin:auto;font-size:12px!important;color:var(--muted)!important;padding:0!important}
.appTabs{position:fixed;bottom:0;left:0;right:0;z-index:15;display:grid!important;grid-template-columns:repeat(7,minmax(0,1fr));gap:2px!important;overflow:visible!important;background:var(--surface);border-top:2px solid var(--orange);padding:6px 5px max(7px,env(safe-area-inset-bottom));box-shadow:0 -4px 15px #2f485c18}
.appTabs button{min-width:0!important;width:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;padding:7px 0!important;border:1px solid transparent;border-radius:9px;font-size:clamp(9px,2.55vw,12px)!important;line-height:1.15;white-space:nowrap}
.appTabs .on{background:#f7c58f;color:#3c342a;border-color:#e4a064}
.tabIcon{font-family:system-ui,sans-serif;font-size:18px;line-height:19px;color:var(--sea)}
.appTabs .on .tabIcon{color:#82551f}
main{width:100%;max-width:620px;padding:0 11px 23px;margin:0 auto!important;min-width:0}
main>*{max-width:100%;min-width:0}
.card{overflow-wrap:anywhere;min-width:0;padding:13px}
.homeHero{padding:15px;margin-top:12px}
.homeHero h2{font-size:22px}
.grid{grid-template-columns:repeat(2,minmax(0,1fr))}
.grid>*{min-width:0}
.tile{min-width:0;overflow:hidden;display:flex;flex-direction:column;justify-content:center}
.tile small{font-size:12px;white-space:normal;line-height:1.3}
.tile strong{font-size:clamp(14px,4.4vw,22px);line-height:1.15;overflow-wrap:anywhere}
.rigCard{overflow:hidden;width:100%}
.rigViewport{width:100%;max-width:430px;aspect-ratio:360 / 225;max-height:269px;margin:8px auto;overflow:hidden;position:relative}
.rigViewport svg{display:block!important;width:100%!important;height:100%!important;max-width:100%!important;max-height:100%!important;overflow:hidden}
.rigCard p{clear:both;position:relative}
.homeMini>.card{padding:10px}
.homeMini>.card h2{font-size:14px}
.relayGrid{grid-template-columns:repeat(4,minmax(0,1fr))}
.settingsOverview{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:7px;margin:12px 0}
.settingsOverview button{min-width:0;min-height:52px;padding:8px 6px;white-space:normal;text-align:left;display:flex;align-items:center;gap:7px;font-size:12px}
.settingsOverview .miniIcon{font-size:19px;color:var(--sea)}
.settingsTools{margin:15px 0 8px}
.settingsTools h2{font-size:15px}
.settingsTabs button{min-width:0;font-size:12px}
.eventFilters{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:5px}
.eventFilters button{min-width:0;padding:9px 3px;font-size:11px}
.eventFilters button.on{background:#f7c58f;border-color:#e4a064}
#events{white-space:pre-line;overflow-wrap:anywhere;line-height:1.6}
:root[data-theme="dark"] .appTabs{background:#112436}
:root[data-theme="dark"] .readOnlyLabel{color:#78e9bf}
:root[data-theme="dark"] .appTabs .on{background:#e18b3d}
:root[data-theme="dark"] .appTabs .tabIcon{color:#60c8ef}
:root[data-theme="dark"] .appTabs .on .tabIcon{color:#1c3c4b}
@media(min-width:650px){.appTabs{max-width:620px;margin:auto;border-left:1px solid var(--line);border-right:1px solid var(--line)}}
@media(max-width:350px){.appTabs button{font-size:9px!important}.tabIcon{font-size:16px}.card{padding:10px}}

/* Two always-visible independent network badges; fixed colors in every theme. */
.netIndicators{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin:5px auto 3px;max-width:620px;font-size:11px;line-height:1.3}
.netPill{display:inline-flex;align-items:center;gap:5px;padding:4px 7px;border:1px solid var(--line);background:var(--surface);border-radius:8px;font-weight:700;color:var(--ink);max-width:100%}
.netDot{display:inline-block;width:10px;height:10px;flex:0 0 10px;border-radius:50%;background:#d33a32;box-shadow:0 0 0 1px #73737355}
.netDot.internetOn,.netDot.dataOn{background:#13a44c}.netDot.localOn{background:#168de3}
.netText{font-size:10px;font-weight:500;color:var(--muted);overflow:hidden;text-overflow:ellipsis;white-space:nowrap;max-width:145px}

.localShortcuts{max-width:620px;margin:7px auto 2px;display:grid;grid-template-columns:1fr 1fr;gap:7px}
.localShortcuts button{border:1px solid #3d83aa;border-radius:9px;background:#17425b;color:#ecfaff;font-size:13px;font-weight:800;min-height:39px}
.localShortcuts button:focus-visible{outline:3px solid #4cdb94}
/* Shared design system with online/index.html. Browser views only. */
:root{--unified-panel:#152a3d;--unified-line:#34546a;--unified-green:#0b994f;--unified-blue:#299bde;}
:root[data-theme="dark"] body{background:#0b1928;color:#f0f7ff}
:root[data-theme="dark"] header{background:#112436;border-color:var(--unified-blue)}
:root[data-theme="dark"] .card{background:var(--unified-panel);border-color:var(--unified-line)}
:root[data-theme="dark"] .homeHero{background:linear-gradient(125deg,#17374f,#0b2943);border-color:#3878a1}
:root[data-theme="dark"] .tile{background:#1a364b;border-color:#346583}
:root[data-theme="dark"] .tile small{color:#b9d6e5}
:root[data-theme="dark"] .tile strong{color:#70e4b0}
:root[data-theme="dark"] .appTabs{background:#10283b;border-top-color:var(--unified-blue)}
.appTabs .on,:root[data-theme="dark"] .appTabs .on{background:var(--unified-green)!important;color:#eafff2!important;border-color:#41d98d!important;font-weight:800}
.appTabs .on .tabIcon,:root[data-theme="dark"] .appTabs .on .tabIcon{color:#e5fff0!important}
.appTabs button{font-size:clamp(8px,2.35vw,11px)!important;padding:9px 1px!important}
@media(max-width:380px){.appTabs button{font-size:8px!important}.tabIcon{font-size:16px}}
.netIndicators{flex-wrap:wrap}
.calcComing{min-height:200px;border:1px dashed #5586a5;border-radius:13px;display:flex;flex-direction:column;gap:10px;align-items:center;justify-content:center;text-align:center;padding:20px;color:var(--muted)}
.calcComing span{font-size:45px;color:var(--sea)}.calcComing strong{font-size:20px;color:var(--ink)}
.calcComing p{margin:0;max-width:370px}

</style></head><body>
<header class="appHeader"><div class="headerRow"><div>
<div class="brandSmall">BUSA MONITOR · №001</div><h1>БУСА-1М <span class="brandSub">Монитор</span></h1></div>
<span class="readOnlyLabel">● МОНИТОРИНГ</span></div>
<div id="clockDisplay" class="clockBar" aria-live="off">Время синхронизируется…</div>
<div class="netIndicators" role="status" aria-label="Два подключения БУСА">
<span class="netPill"><span id="internetLed" class="netDot"></span>ИНТЕРНЕТ <span id="internetLabel" class="netText">нет связи</span></span>
<span class="netPill"><span id="localLed" class="netDot"></span>БУСА Wi-Fi <span id="localLabel" class="netText">нет клиента</span></span>
<span class="netPill"><span id="dataLed" class="netDot"></span>ДАН <span id="dataLabel" class="netText">не настроено</span></span>
</div>
<div id="appHealth" role="status">Связь с ESP32 проверяется...</div>
<div class="localShortcuts"><button type="button" data-quick-local="wifi">⌁ Связь</button><button type="button" data-quick-local="diagnostics">⊞ Диагностика</button><a href="/recovery" style="padding:10px;display:inline-flex;align-items:center">Проверка связи</a></div>
<nav class="appTabs" aria-label="Разделы">
<button data-page="0" class="on"><span class="tabIcon">⌂</span><span>Главная</span></button>
<button data-page="1"><span class="tabIcon">♨</span><span>Температуры</span></button>
<button data-page="2"><span class="tabIcon">♧</span><span>Отбор</span></button>
<button data-page="3"><span class="tabIcon">⌁</span><span>Графики</span></button>
<button data-page="4"><span class="tabIcon">▤</span><span>События</span></button>
<button data-page="5"><span class="tabIcon">⚙</span><span>Настр.</span></button>
<button data-page="6"><span class="tabIcon">▦</span><span>Кальк.</span></button>
</nav></header><main>
<section class="page" data-section="0">
  <div class="homeHero"><div class="heroEyebrow">БУСА-1М №001 · ПАНЕЛЬ МОНИТОРИНГА</div>
  <h2>Главная</h2><div class="heroStatus"><span>● Только показания</span><span id="mainwifi">Wi-Fi: ожидание</span></div></div>
  <div class="card"><h2>Температура · T1–T4</h2><div class="grid temps">
    <div class="tile"><small><span class="tempDot t1"></span>T1 · Вода</small><strong id="t0">Н/Д</strong><small id="ts0">Ожидание измерения</small></div>
    <div class="tile"><small><span class="tempDot t2"></span>T2 · Колонна</small><strong id="t1">Н/Д</strong><small id="ts1">Ожидание измерения</small></div>
    <div class="tile"><small><span class="tempDot t3"></span>T3 · Куб</small><strong id="t2">Н/Д</strong><small id="ts2">Ожидание измерения</small></div>
    <div class="tile"><small><span class="tempDot t4"></span>T4 · Верх колонны</small><strong id="t3">Н/Д</strong><small id="ts3">Ожидание измерения</small></div>
  </div></div>
  <div class="card rigCard"><h2>Схема установки</h2><div class="rigViewport">
  <svg viewBox="0 0 360 225" role="img" aria-label="Схема установки БУСА с датчиком YF-S401 на линии охлаждающей воды" preserveAspectRatio="xMidYMid meet">
<defs><linearGradient id="steel" x1="0" x2="1"><stop stop-color="#71899a"/><stop offset=".45" stop-color="#e6f0f3"/><stop offset="1" stop-color="#768b9b"/></linearGradient><linearGradient id="water" x1="0" x2="0" y1="0" y2="1"><stop stop-color="#31b6e8"/><stop offset="1" stop-color="#087eae"/></linearGradient></defs>
<rect width="360" height="225" rx="14" fill="#f8fbfd"/>
<path d="M88 179 V93 Q88 85 96 85 H164" fill="none" stroke="#738c9b" stroke-width="7"/>
<rect x="26" y="144" width="126" height="63" rx="18" fill="url(#steel)" stroke="#607b8a" stroke-width="2"/>
<rect x="40" y="155" width="99" height="38" rx="9" fill="#809aaa" stroke="#526d7e"/>
<path d="M58 178 V160 M71 177 V157 M83 178 V157 M96 178 V157 M109 178 V157" stroke="#bde5f2" stroke-width="3"/>
<rect x="83" y="22" width="24" height="126" rx="7" fill="url(#steel)" stroke="#6b8699" stroke-width="2"/>
<g stroke="#657f91" stroke-width="3"><path d="M78 43 H112 M78 58 H112 M78 73 H112 M78 88 H112 M78 103 H112 M78 118 H112"/></g>
<path d="M95 22 V16 H255 V49" fill="none" stroke="#798c99" stroke-width="6" stroke-linejoin="round"/>
<rect x="230" y="48" width="57" height="81" rx="10" fill="url(#steel)" stroke="#6c8494" stroke-width="2"/>
<path d="M242 59 Q272 65 242 72 T242 88 T242 103 T242 119" fill="none" stroke="#1e9dc9" stroke-width="4"/>
<path d="M231 139 H317 V167" stroke="#1599ca" stroke-width="5" stroke-linecap="round" fill="none"/>
<path d="M214 117 V139 H231" stroke="#1599ca" stroke-width="5" fill="none"/>
<path d="M278 167 H321" stroke="#1687b9" stroke-width="5"/>
<rect x="288" y="126" width="37" height="26" rx="7" fill="#dcf4fa" stroke="#087faf" stroke-width="3"/>
<path d="M296 139 H316 M309 134 L316 139 L309 144" stroke="#0b749d" stroke-width="2" fill="none"/>
<text x="306" y="122" text-anchor="middle" font-size="12" font-weight="bold" fill="#07668f">YF-S401</text>
<text x="306" y="164" text-anchor="middle" font-size="10" fill="#176a91">ВОДА</text>
<path d="M261 129 H341 V184" fill="none" stroke="#7c8c98" stroke-width="4"/>
<rect x="312" y="183" width="32" height="33" rx="5" fill="#fff" stroke="#7b92a1" stroke-width="2"/>
<rect x="315" y="199" width="26" height="13" rx="3" fill="#b4e4ed"/>
<text x="95" y="217" text-anchor="middle" font-size="11" fill="#526779">Мониторинг температур T1–T4</text>
<text x="254" y="41" text-anchor="middle" font-size="10" fill="#526779">Охлаждение</text>
</svg></div>
  <p class="hint">Схема условная. YF-S401 показан на линии <b>охлаждающей воды</b>, а не на линии отбора.</p></div>
  <div class="grid homeMini">
   <div class="card"><h2>Охлаждающая вода</h2><div class="row">YF-S401 <b>GPIO15</b></div>
   <div class="row">Импульсы <b id="pulses">—</b></div>
   <div class="row">Расход, л/мин <b id="flow">—</b></div><p class="hint">Не откалибровано; 0 импульсов не доказывает отсутствие воды.</p></div>
   <div class="card"><h2>Отбор</h2><div class="row">Расход <b class="warn">НЕТ ДАТЧИКА</b></div>
   <div class="row">Собрано <b class="warn">Н/Д</b></div>
   <p class="hint">Датчик охлаждающей воды не измеряет отбор.</p></div>
  </div>
  <div class="card"><h2>Состояние K1–K4</h2>
   <div class="relayGrid"><span>K1 <b>БЛОК</b></span><span>K2 <b>БЛОК</b></span><span>K3 <b>БЛОК</b></span><span>K4 <b>БЛОК</b></span></div>
   <p class="hint">Состояние информационное. Физические выходы ESP32 отключены.</p>
  </div></section>
<section class="page" data-section="1" hidden><div class="card"><h2>Температуры · DS18B20</h2><div id="temperatures">Ожидание...</div><p id="lastTUpdate" class="hint">Данных от ESP32 пока нет.</p>
<div id="temperatureDetails"></div>
<h3>Привязка найденных датчиков</h3>
<p class="hint">Если на одной линии несколько датчиков, назначь каждый по ROM. Идентифицируй датчик по изменению его температуры при лёгком нагреве рукой. Один ROM нельзя назначить двум каналам.</p>
<div class="grid">
<label>T1 Вода<select id="bind0"><option value="auto">Авто GPIO16</option></select></label>
<label>T2 Колонна<select id="bind1"><option value="auto">Авто GPIO17</option></select></label>
<label>T3 Куб<select id="bind2"><option value="auto">Авто GPIO18</option></select></label>
<label>T4 Верх<select id="bind3"><option value="auto">Авто GPIO21</option></select></label></div>
<button type="button" class="full" id="saveBindings">Сохранить привязку T1–T4</button>
<p id="bindingResult" class="hint">Названия каналов автоматически не переставляются.</p>
<pre id="temperatureInventory" style="white-space:pre-wrap;word-break:break-word">Поиск ROM датчиков...</pre>
<p class="hint">Информационные поля для отображения на TFT. Не управляют реле и не заменяют защиту.</p>
<div class="grid">
<div><label>T1 Вода · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref0"></div>
<div><label>T1 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt0"></div>
<div><label>T2 Колонна · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref1"></div>
<div><label>T2 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt1"></div>
<div><label>T3 Куб · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref2"></div>
<div><label>T3 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt2"></div>
<div><label>T4 Верх · Зад., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="ref3"></div>
<div><label>T4 Предупр., °C</label><input inputmode="decimal" type="number" min="0" max="125" step="0.1" id="nt3"></div>
</div><button id="refsSave" type="button" class="full">Сохранить значения отображения</button>
<p id="refsResult" class="hint">Редактирование только через локальную сеть БУСА.</p></div></section>
<section class="page" data-section="2" hidden><div class="card"><h2>Отбор · информационный раздел</h2><p class="hint">В этом разделе только отображение данных. Команды физическим выходам не передаются.</p><div class="row">Расход, мл/мин <b class="warn">НЕТ ДАТЧИКА</b></div><div class="row">Собрано, мл <b class="warn">НЕТ ДАННЫХ</b></div><p class="hint">Датчик воды охлаждения не измеряет поток отбора.</p></div><div class="card"><h2>Вода охлаждения</h2><div class="row">Расход, л/мин <b id="flow2">—</b></div><div class="row">Объём, л (оценка) <b id="total">—</b></div></div></section>
<section class="page" data-section="3" hidden>
 <div class="card"><h2>Графики · реальные измерения</h2>
 <div class="settingsTabs" style="grid-template-columns:repeat(3,1fr)"><button type="button" data-graph-tft="0" class="on">T1–T4</button><button type="button" data-graph-tft="1">Вода</button><button type="button" data-graph-tft="2">Отбор</button></div>
 <div class="graphPanel" data-graph-panel="0"><h3>Четыре температуры, °C</h3><canvas id="graphT" width="600" height="200"></canvas><p class="hint">Только измерения текущей сессии. Каждая линия имеет свой цвет.</p></div>
 <div class="graphPanel" data-graph-panel="1" hidden><h3>YF-S401 · вода охлаждения</h3><canvas id="graphF" width="600" height="200"></canvas><p class="hint">Показаны реальные импульсы опроса, не литры: датчик не откалиброван.</p></div>
 <div class="graphPanel" data-graph-panel="2" hidden><h3>График отбора</h3><p class="warn">Нет датчика отбора. График без выдуманных данных.</p></div>
 </div></section>
<section class="page" data-section="4" hidden><div class="card"><h2>Журнал событий</h2>
<div class="eventFilters">
<button type="button" class="on" data-event="all">Все</button>
<button type="button" data-event="warn">Предупр.</button>
<button type="button" data-event="alarm">Аварии</button>
<button type="button" data-event="system">Система</button></div>
<div id="events">Ожидание...</div><p class="hint">Только события текущей сессии. Записи без подтверждения не создаются.</p></div></section>
<section class="page" data-section="5" hidden><div class="card">
<h2>Настройки</h2><div class="settingsOverview">
<button type="button" data-settings-link="temp"><span class="miniIcon">♨</span>Температуры</button>
<button type="button" data-settings-link="collection"><span class="miniIcon">♧</span>Отбор жидкости</button>
<button type="button" data-settings-link="wifi"><span class="miniIcon">⌁</span>Wi-Fi</button>
<button type="button" data-settings-link="display"><span class="miniIcon">▣</span>Экран и звук</button>
<button type="button" data-settings-link="system"><span class="miniIcon">⚙</span>Система</button>
<button type="button" data-settings-link="about"><span class="miniIcon">ⓘ</span>О системе</button>
</div><p class="hint">Режим мониторинга: исполнительные выходы не включаются.</p></div>
<div class="card"><h2>YF-S401 · калибровка по воде</h2>
<div class="row"><span>Калибровка</span><b id="waterCalState">НЕ ПРОВЕДЕНА</b></div>
<div class="row"><span>Импульсов в текущей пробе</span><b id="waterCalPulses">—</b></div>
<div class="row"><span>Коэффициент, имп./л</span><b id="waterCalFactor">—</b></div>
<p class="hint">Сначала проверяем только импульсы. Коэффициент сохраняется после измерения реального объёма обычной воды.
До этого расход и объём показываются как Н/Д. Это не система защиты или управления.</p>
<button id="waterCalStart" type="button" class="full">Начать запись импульсов пробы воды</button>
<label for="waterCalVolume">Фактически измеренный объём воды, мл</label>
<input id="waterCalVolume" inputmode="decimal" type="number" min="100" max="50000" step="1" placeholder="Ввести после измерения">
<button id="waterCalFinish" type="button" class="full">Сохранить результат калибровки</button>
<button id="waterCalReset" type="button" class="full">Сбросить коэффициент</button>
<p id="waterCalResult" role="status" class="hint">Калибровка ещё не начата.</p>
</div>
<div class="card settingsTools"><h2>Технические разделы</h2><div class="row">Экран TFT <b>ST7789 · 320×240</b></div>
<div class="grid"><button type="button" id="tftUp">TFT ↑</button><button type="button" id="tftDown">TFT ↓</button></div>
<p class="hint">Управление видом TFT; физические выходы не затрагиваются.</p></div>
<div id="settingsTabs" class="settingsTabs"><button data-stab-btn="0" class="on">Wi-Fi</button><button data-stab-btn="1">Google</button><button data-stab-btn="2">Обновление</button><button data-stab-btn="3">Диагностика</button></div><div class="card settingCard" data-stab="0"><h2>Wi-Fi</h2>
<div id="wstatus">Проверка подключения...</div><button type="button" class="full" id="scan">Найти сети Wi-Fi 2,4 ГГц</button>
<label for="net">Доступные сети Wi-Fi</label><select id="net"><option value="">Выберите сеть из списка</option></select>
<label for="manualSsid">Имя сети вручную (если её нет в списке)</label><input id="manualSsid" maxlength="32" placeholder="Точное имя точки доступа">
<p class="hint">Для iPhone 16 включи «Максимальная совместимость» в режиме модема и оставь этот экран открытым во время поиска. Локальная сеть и подключение БУСА к интернету работают одновременно. Отключать телефон или ноутбук от БУСА не нужно.</p>
<label for="pass">Пароль Wi-Fi</label><input id="pass" type="password" minlength="8" maxlength="63" placeholder="Введите пароль самостоятельно">
<button type="button" class="full" id="save">Сохранить и подключить</button>
<p id="response" role="status"></p><p class="hint">Настройка доступна при подключении к защищённой сети БУСА. Пароль остаётся на контроллере.</p></div>
<div class="card settingCard" data-stab="1" hidden><h2>Google Диск · диагностика БУСА</h2>
<div class="row">Состояние канала <b id="googleState">НЕ НАСТРОЕН</b></div>
<p class="hint">БУСА передаёт реальные показания и коды ошибок каждые 30 секунд в вашу Google Таблицу по HTTPS. Это только мониторинг, без управления внешним оборудованием.</p>
<p class="hint"><b>Важно:</b> приложение «Google Диск» на iPhone показывает файлы, но само не принимает данные с ESP32. Один раз создайте свою Google Таблицу и установите Apps Script-приёмник, затем вставьте адрес /exec и секрет ниже.</p>
<p><a href="https://github.com/e62815667-gif/BUSA-Monitor/blob/lab/busa-v14-3-final-display-google/google/README_BUSA_Google_Drive.md" target="_blank" rel="noopener noreferrer" style="color:#a6dfff">Пошаговая инструкция настройки Google Таблицы</a></p>
<label for="googleUrl">Адрес Apps Script (ссылка /exec)</label>
<input id="googleUrl" type="url" placeholder="https://script.google.com/macros/s/.../exec">
<label for="googleToken">Секрет устройства (токен из Apps Script)</label>
<input id="googleToken" type="password" autocomplete="off" placeholder="Вставить на устройстве, не в чат">
<button id="googleSave" type="button" class="full">Сохранить подключение Google</button>
<button id="googleTest" type="button" class="full">Проверить отправку диагностики сейчас</button>
<button id="googleStop" type="button" class="full">Отключить передачу в Google</button>
<p id="googleResult" role="status" class="hint">Отправка выключена до настройки.</p>
</div><div class="card settingCard" data-stab="2" hidden><h2>Обновления · Google и локально</h2>
<p class="hint">Облачная проверка версии поступает в проверенном HTTPS-ответе Google вместе с подтверждением телеметрии. Дистанционная установка НЕ включена.</p>
<div class="row"><span>Каталог Google</span><b id="cloudCatalogue">Ожидание</b></div>
<div class="row"><span>Новая версия</span><b id="cloudVersion">—</b></div>
<div class="row"><span>Flash на ESP32</span><b id="cloudFlash">—</b></div>
<div class="row"><span>Flash в сборке</span><b id="cloudBuildFlash">—</b></div>
<div class="row"><span>Активный раздел OTA</span><b id="cloudOtaRunning">—</b></div>
<div class="row"><span>Резервный раздел OTA</span><b id="cloudOtaNext">—</b></div>
<div class="row"><span>Образ помещается</span><b id="cloudOtaFits">Неизвестно</b></div>
<div class="row"><span>OTA через интернет</span><b>ЗАБЛОКИРОВАНО</b></div>
<p class="hint" id="cloudSafety">Совпадение Flash не подтверждает совместимость разделов OTA и откат. Не устанавливайте непроверенную прошивку.</p>
<h2>Ручное обновление через локальный Wi-Fi</h2>
<p class="hint">Только через локальную сеть БУСА. Выберите APP BIN для ESP32-S3 N8R2,
НЕ объединённый merged BIN. Только проверенные диагностические сборки.
Перед обновлением отключите внешние исполнительные цепи.</p>
<input type="file" accept=".bin" id="firmwareFile">
<button type="button" class="full" id="manualUpdate">Выбрать и обновить</button>
<p id="updateResult" role="status">Ожидание подтверждения</p>
</div><div class="card settingCard" data-stab="3" hidden><h2>Диагностика</h2>
<div class="row">Экран ST7789 <b id="displayState">—</b></div>
<p class="hint">Экран гаснет только по F2 или кнопке на iPhone. Любая распознанная клавиша снова включает экран без выполнения команды. Датчики, Wi-Fi и отправка в Google продолжают работать. Подсветка, постоянно подключённая к питанию, может не погаснуть.</p>
<div class="grid"><button id="screenSleep" type="button">Погасить экран</button><button id="screenWake" type="button">Разбудить экран</button></div>
<div class="row">Прошивка <b id="ver">—</b></div>
<div class="row">Время работы <b id="uptime">—</b></div>
<div class="row">PCF8575 <b id="pcf">—</b></div>
<div class="row">PCA9685 <b id="pca">—</b></div><div class="row">TFT <b id="tft">—</b></div>
<h2>К1–К4 · виртуальная проверка (выходы отключены)</h2>
<p class="hint">Только программная имитация на 10 секунд. Никаких сигналов на GPIO4–7 и PCA9685; фактические контакты и питание 12 В не измеряются.</p>
<div class="row">Виртуальный канал <b id="relaySimChannel">0 / 4</b></div>
<div class="row">Оставшееся время <b id="relaySimRemaining">0 с</b></div>
<div class="row">Контакты реле <b>НЕ ПОДТВЕРЖДЕНЫ</b></div>
<div class="grid">
<button type="button" data-relay-sim="1">Тест К1</button>
<button type="button" data-relay-sim="2">Тест К2</button>
<button type="button" data-relay-sim="3">Тест К3</button>
<button type="button" data-relay-sim="4">Тест К4</button>
</div>
<button type="button" class="full" id="relaySimStop">СТОП · отмена виртуального теста</button>
<p id="relaySimResult" role="status" class="hint">Реле физически заблокированы.</p>
<h2>PCA9685 · 11 светодиодных каналов · тест</h2>
<p class="hint">Подтверждено владельцем: отдельные светодиоды на 3,3 В, 11 цветовых каналов. Для физического включения через PCA9685 ещё нужно подтвердить токоограничение и состояние OE. Ниже доступен виртуальный тест на 8 секунд; лампочки он не зажигает.</p>
<div class="row">Тип и питание <b>Отдельные LED, 3,3 В</b></div>
<div class="row">Физический тест <b id="ledPreflight">ЗАБЛОКИРОВАН</b></div>
<div class="row">Проверка <b id="ledSimActive">НЕТ</b></div>
<div class="row">Осталось <b id="ledSimRemaining">0 с</b></div>
<div class="row">MODE1/MODE2 <b id="ledSimModes">—</b></div>
<div class="row">Чтение регистров <b id="ledSimRead">—</b></div>
<div class="grid">
<button type="button" data-led-channel="0">CH0 · T1 синий</button>
<button type="button" data-led-channel="1">CH1 · T2 зелёный</button>
<button type="button" data-led-channel="2">CH2 · T3 жёлтый</button>
<button type="button" data-led-channel="3">CH3 · T4 красный</button>
<button type="button" data-led-channel="4">CH4 · установка зел.</button>
<button type="button" data-led-channel="5">CH5 · установка крас.</button>
<button type="button" data-led-channel="6">CH6 · кран зелёный</button>
<button type="button" data-led-channel="7">CH7 · кран красный</button>
<button type="button" data-led-channel="8">CH8 · Wi-Fi зелёный</button>
<button type="button" data-led-channel="9">CH9 · Wi-Fi красный</button>
<button type="button" data-led-channel="10">CH10 · авария красный</button>
</div>
<button type="button" class="full" id="ledSimStop">Остановить виртуальную проверку LED</button>
<p id="ledSimResult" class="hint">Физические выходы отключены. OE и токи не подтверждены.</p>
<h2>Калибровка клавиатуры 5×4</h2><p class="hint">Назначения: ↑ вверх, ↓ вниз, ← влево, → вправо, Ent ОК, Esc назад, F1 удалить значение.</p>
<p class="hint">Не переставляй провода. После запуска нажимай кнопки последовательно слева направо, сверху вниз по фотографии. Полностью отпускай клавишу между нажатиями. Карта сохраняется в ESP32.</p>
<div class="row">Состояние <b id="keyState">—</b></div>
<div class="row">Следующая клавиша <b id="keyExpected">—</b></div>
<div class="row">Контакты <b id="keyRaw">—</b></div><p id="keyInfo" class="hint"></p>
<h2>Диагностика V14 · все модули</h2>
<p class="hint">Отображаются ответы интерфейсов, а не гарантии исправности оборудования. Отсутствие импульсов расходомера не доказывает отсутствия воды.</p>
<pre id="diagOneWire" style="white-space:pre-wrap;word-break:break-word">Ожидание опроса DS18B20...</pre>
<pre id="diagKeyboard" style="white-space:pre-wrap;word-break:break-word">Ожидание PCF8575...</pre>
<pre id="diagI2c" style="white-space:pre-wrap;word-break:break-word">Сканирование I2C...</pre>
<pre id="diagOther" style="white-space:pre-wrap;word-break:break-word">TFT / ESP32 / Wi-Fi / YF-S401...</pre>
<h2>Отчёт для анализа</h2>
<p class="hint">Скопируй отчёт и пришли в наш чат. Wi-Fi пароли и Google-токены в отчёт не включены.</p>
<textarea id="diagReport" rows="9" readonly style="width:100%;font:11px ui-monospace,monospace;white-space:pre;overflow:auto">Ожидание диагностики...</textarea>
<button type="button" id="diagCopy" class="full">Скопировать отчёт всех модулей</button>
<p id="diagCopyResult" class="hint">—</p>
<p class="hint">Температура: T1 GPIO16, T2 GPIO17, T3 GPIO18, T4 GPIO21. IDLE=1 — линия DATA высока в покое; IDLE=0 или LINE_LOW — линия низкая. AFTER — состояние после импульса сброса. NO_PRESENCE при IDLE=1 — нет ответа датчика, но это не доказывает, что сам датчик неисправен. NO_ROM — не найден адрес; ROM_CRC_BAD — неверная CRC.</p>
<p class="hint">Клавиатура: idle_hex — линии в покое, low_hex — ответившие линии, scan_error — код ошибки опроса.</p>
<p class="hint">После нажатия клавиши переход происходит автоматически. Если не произошёл, нажми «Считать и далее» в течение 10 секунд. Без распознанных контактов пропуск запрещён, чтобы не записать неверную раскладку.</p>
<h2>Экран TFT · V14</h2>
<p class="hint">Белый TFT может означать неверный контроллер дисплея или проблему SPI. Профиль выбирается вручную: ESP32 не умеет подтвердить, что пиксели действительно видны.</p>
<label for="tftDriver">Драйвер TFT</label>
<select id="tftDriver"><option value="1">ST7789 · горизонтально 320×240</option></select>
<label for="tftRotation">Ориентация экрана</label>
<select id="tftRotation"><option value="1">1 · Горизонтально 320×240</option></select>
<button type="button" id="tftSave" class="full">Сохранить профиль и перезапустить ESP32</button>
<button type="button" id="tftTest" class="full">Тест TFT: чёрный фон, BUSA 1M AI · 5 секунд</button>
<p id="tftProfileStatus" class="hint">Драйвер ST7789 закреплён. Проверка физического экрана выполняется отдельно.</p>
<p class="hint">Обе темы цветные: синяя вода, разноцветные датчики, оранжевые выделения. Аппаратная инверсия выключена. Драйвер ST7789 не меняется.</p>
<label for="tftThemeSelect">Тема физического TFT БУСА (не тема iPhone)</label>
<select id="tftThemeSelect"><option value="dark">Тёмная</option><option value="light">Светлая</option></select>
<button type="button" id="tftTheme" class="full">Применить тему TFT БУСА</button>
<p id="tftThemeStatus" class="hint">—</p>
<button id="keyStart" type="button" class="full">Начать обучение 20 клавиш</button>
<button id="keyNext" type="button" class="full">Считать и далее →</button>
<div class="grid"><button type="button" id="keyUndo">Назад на шаг</button>
<button type="button" id="keyCancel">Отмена</button></div></div></section>
<footer class="hint">БУСА-1М №001 · только мониторинг · управление внешними устройствами вручную · физические выходы заблокированы</footer><section class="page" data-section="6" hidden><div class="card">
<h2>Калькулятор</h2><div class="calcComing"><span aria-hidden="true">▦</span><strong>Раздел зарезервирован</strong><p>Расчёты добавим позднее по утверждённому заданию. Сейчас эта вкладка ничего не измеряет и не управляет оборудованием.</p></div>
</div></section>
</main>
<script>
async function localFetch(url,options={}){
  const ctl=new AbortController();
  const timer=setTimeout(()=>ctl.abort(),10000);
  try{
    const response=await fetch(url,{...options,signal:ctl.signal});
    const body=await response.arrayBuffer();
    return new Response(body,{status:response.status,statusText:response.statusText,headers:response.headers});
  }finally{clearTimeout(timer)}
}

const get=id=>document.getElementById(id),logs=[],tlog=[],flog=[],tlog4=[[],[],[],[]],waterPulseLog=[];
let lastWaterPulse=null,themeInitialized=false;
let active=0,connected=false,pollBusy=false,settingsActive=0,lastLocalNav=0,refsInit=false,lastClockPush=0,tftProfileLoaded=false;
let lastStatusError="",successfulStatusReads=0;
let eventCategory="all",eventRecords=[];
function networkLeds(info,ssid){
 const online=info&&info.internet===true;
 const local=info&&info.local_client===true;
 get('internetLed').classList.toggle('internetOn',!!online);
 get('localLed').classList.toggle('localOn',!!local);
 get('internetLabel').textContent=online?(ssid||'есть сеть'):'нет связи';
 get('localLabel').textContent=local?'клиент подключён':'нет клиента';
}

let lastDiagnosticReport='';
function showPhoneClock(){
  get('clockDisplay').textContent='Дата и время: '+
    new Date().toLocaleString('ru-RU',{year:'numeric',month:'2-digit',day:'2-digit',
      hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
}
async function syncClock(){
  const d=new Date();
  // Negative getTimezoneOffset is the conventional minutes EAST of UTC.
  const body=new URLSearchParams({epoch:String(Math.floor(d.getTime()/1000)),
    tz:String(-d.getTimezoneOffset())});
  try{
    const r=await localFetch('/api/time/sync',{method:'POST',body});
    if(r.ok)lastClockPush=Date.now();
  }catch(e){}
}
function drawEvents(){
 const filtered=eventRecords.filter(e=>eventCategory==='all'||e.kind===eventCategory);
 get('events').textContent=filtered.length?filtered.map(e=>e.text).join('\n'):'Событий выбранного типа нет';
}
function message(s,kind='system'){
 const text=new Date().toLocaleTimeString('ru-RU')+' · '+s;
 logs.unshift(text);logs.splice(12);
 eventRecords.unshift({text,kind});eventRecords.splice(0,40);drawEvents();
}
document.querySelectorAll('[data-event]').forEach(b=>b.onclick=()=>{
 eventCategory=b.dataset.event;
 document.querySelectorAll('[data-event]').forEach(x=>x.classList.toggle('on',x===b));
 drawEvents();
});
document.querySelectorAll('[data-settings-link]').forEach(b=>b.onclick=()=>{
 const action=b.dataset.settingsLink;
 if(action==='temp'){page(1);return;}
 if(action==='collection'){page(2);return;}
 settingsTab(action==='wifi'?0:action==='display'||action==='system'||action==='about'?3:1);
 const top=get('settingsTabs').getBoundingClientRect().top+window.scrollY-115;
 window.scrollTo({top:Math.max(0,top),behavior:'smooth'});
});
function page(n,send=true){active=n;document.querySelectorAll('[data-section]').forEach(e=>e.hidden=Number(e.dataset.section)!==n);document.querySelectorAll('nav button').forEach(e=>e.classList.toggle('on',Number(e.dataset.page)===n));if(n===6)send=false;if(send){lastLocalNav=Date.now();localFetch('/api/page?n='+(n+1)).catch(()=>{})}if(n===3)graphs()}
document.querySelectorAll('[data-quick-local]').forEach(b=>b.addEventListener('click',()=>{
 page(5);settingsTab(b.dataset.quickLocal==='wifi'?0:3);
 document.getElementById('settingsTabs').scrollIntoView({behavior:'smooth',block:'start'});
}));
function settingsTab(n,send=true){settingsActive=n;document.querySelectorAll('.settingCard').forEach(e=>e.hidden=Number(e.dataset.stab)!==n);document.querySelectorAll('[data-stab-btn]').forEach(e=>e.classList.toggle('on',Number(e.dataset.stabBtn)===n));if(send){lastLocalNav=Date.now();localFetch('/api/settings-tab?n='+n).catch(()=>{})}}
document.querySelectorAll('[data-stab-btn]').forEach(b=>b.onclick=()=>settingsTab(Number(b.dataset.stabBtn)));
document.querySelectorAll('[data-graph-tft]').forEach(b=>b.onclick=async()=>{
  const n=Number(b.dataset.graphTft);
  document.querySelectorAll('[data-graph-tft]').forEach(c=>c.classList.toggle('on',c===b));
  document.querySelectorAll('[data-graph-panel]').forEach(p=>p.hidden=Number(p.dataset.graphPanel)!==n);
  await localFetch('/api/graph-view?view='+n).catch(()=>{});
});
get('tftUp').onclick=()=>localFetch('/api/tft/scroll?dir=up').catch(()=>{});
get('tftDown').onclick=()=>localFetch('/api/tft/scroll?dir=down').catch(()=>{});
async function keyCommand(action){
 try{
   let r=await localFetch('/api/keypad/'+action,{method:'POST',body:new URLSearchParams({confirm:'YES'})});
   get('keyInfo').textContent=await r.text();
 }catch(e){get('keyInfo').textContent='Ошибка: '+e.message}
}
async function setScreenPower(action){
 try{
  const resp=await localFetch('/api/tft/screen',{method:'POST',body:new URLSearchParams({action})});
  if(!resp.ok)throw new Error(await resp.text());
  get('displayState').textContent=action==='wake'?'РАБОТАЕТ':'ЭКРАН ПОГАШЕН';
 }catch(e){get('displayState').textContent='ОШИБКА: '+e.message}
}
get('screenSleep').onclick=()=>setScreenPower('sleep');
get('screenWake').onclick=()=>setScreenPower('wake');
get('keyStart').onclick=()=>{if(confirm('Обучить клавиатуру, начиная с F1?'))keyCommand('start')};
get('keyNext').onclick=()=>keyCommand('next');
get('tftSave').onclick=async()=>{
 if(!confirm('Сохранить профиль TFT и перезапустить ESP32? Реле остаются отключены.'))return;
 try{
  const r=await localFetch('/api/tft/profile',{method:'POST',body:new URLSearchParams({confirm:'YES',profile:get('tftDriver').value,rotation:get('tftRotation').value})});
  get('tftProfileStatus').textContent=await r.text();
 }catch(e){get('tftProfileStatus').textContent='Ошибка: '+e.message}
};
get('tftTest').onclick=async()=>{
 try{
  const r=await localFetch('/api/tft/test',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
  get('tftProfileStatus').textContent=await r.text();
 }catch(e){get('tftProfileStatus').textContent='Ошибка: '+e.message}
};
get('diagCopy').onclick=async()=>{
 const t=get('diagReport');
 if(!lastDiagnosticReport){get('diagCopyResult').textContent='Отчёт ещё не получен';return;}
 try{
  if(navigator.clipboard && window.isSecureContext){
   await navigator.clipboard.writeText(lastDiagnosticReport);
   get('diagCopyResult').textContent='Скопировано — вставь в чат';
  }else{
   t.focus();t.select();t.setSelectionRange(0,t.value.length);
   const ok=document.execCommand('copy');
   get('diagCopyResult').textContent=ok?'Скопировано — вставь в чат':'Выдели отчёт и скопируй вручную';
  }
 }catch(e){t.focus();t.select();get('diagCopyResult').textContent='Выдели отчёт и скопируй вручную'}
};
async function setTftPhysicalTheme(theme){
 if(theme!=='dark'&&theme!=='light')throw Error('Недопустимая тема');
 const r=await localFetch('/api/tft/theme',{method:'POST',
  body:new URLSearchParams({confirm:'YES',theme})});
 const payload=await r.text();
 if(!r.ok)throw Error(payload);
 const info=JSON.parse(payload);
 get('tftThemeSelect').value=info.theme;
 get('tftThemeStatus').textContent='TFT БУСА: '+(info.theme==='dark'?'тёмная':'светлая')+
  ' · сохранено в ESP32. Тема iPhone не изменена.';
 return info;
}
get('tftTheme').onclick=async()=>{
 try{await setTftPhysicalTheme(get('tftThemeSelect').value);}
 catch(e){get('tftThemeStatus').textContent='Ошибка TFT: '+e.message;}
};
// GitHub Pages can navigate here, but HTTPS cannot post directly to HTTP ESP32.
// Never apply a theme merely by opening a link without explicit user consent.
const requestedTftTheme=new URLSearchParams(location.search).get('tfttheme');
if(requestedTftTheme==='dark'||requestedTftTheme==='light'){
  window.addEventListener('load',async()=>{
   if(!window.confirm('Установить '+(requestedTftTheme==='dark'?'ТЁМНУЮ':'СВЕТЛУЮ')+
     ' тему физического TFT БУСА? Тема приложения iPhone не изменится.'))return;
   try{await setTftPhysicalTheme(requestedTftTheme);}
   catch(e){get('tftThemeStatus').textContent='Ошибка: '+e.message;}
   history.replaceState(null,'',location.pathname);
  });
}
get('keyUndo').onclick=()=>keyCommand('undo');
get('keyCancel').onclick=()=>keyCommand('cancel');
get('refsSave').onclick=async()=>{
 let body=new URLSearchParams();
 for(let i=0;i<4;i++){
   for(let k of ['ref','nt']){
     let v=get(k+i).value.trim();
     if(v&&(!Number.isFinite(Number(v))||Number(v)<0||Number(v)>125)){
       get('refsResult').textContent='Допустимы только значения 0..125 °C';return;
     }
     body.set(k+i,v);
   }
 }
 let r=await localFetch('/api/display-refs',{method:'POST',body});
 get('refsResult').textContent=await r.text();
 if(r.ok)refsInit=false;
};

document.querySelectorAll('nav button').forEach(e=>e.onclick=()=>page(Number(e.dataset.page)));
function plot(id,values){const c=get(id),x=c.getContext('2d');x.fillStyle='#061b2c';x.fillRect(0,0,600,200);const a=values.filter(v=>v!==null);if(!a.length){x.fillStyle='#d9bd75';x.fillText('НЕТ ДАННЫХ',24,60);return}let lo=Math.min(...a),hi=Math.max(...a);if(hi-lo<.1)hi=lo+.1;x.strokeStyle='#70e99b';x.beginPath();values.forEach((v,i)=>{if(v===null)return;let xx=10+i*580/80,yy=185-(v-lo)/(hi-lo)*165;if(i===0||values[i-1]===null)x.moveTo(xx,yy);else x.lineTo(xx,yy)});x.stroke()}
function graphAllTemperatures(){
 const c=get('graphT'),x=c.getContext('2d'),colors=['#048abe','#d46237','#a83c23','#38a878'];
 x.fillStyle='#fffdf3';x.fillRect(0,0,600,200);
 x.strokeStyle='#d4d6d6';
 for(let j=0;j<=4;j++){x.beginPath();x.moveTo(30,20+j*40);x.lineTo(595,20+j*40);x.stroke()}
 x.font='13px sans-serif';
 for(let k=0;k<4;k++){
  x.fillStyle=colors[k];x.fillText(['T1 Вода','T2 Колонна','T3 Куб','T4 Верх'][k],33+k*139,13);
  x.strokeStyle=colors[k];x.lineWidth=2;x.beginPath();
  let began=false,arr=tlog4[k];
  for(let i=0;i<arr.length;i++){
   const v=arr[i],px=30+i*7,py=180-(v===null?0:Math.max(0,Math.min(v,125))*160/125);
   if(v===null){began=false;continue}
   if(!began){x.moveTo(px,py);began=true}else x.lineTo(px,py);
  }
  x.stroke();
 }
 if(!tlog4.some(a=>a.some(v=>v!==null))){x.fillStyle='#a43c24';x.fillText('НЕТ ИЗМЕРЕНИЙ T1–T4',180,107)}
}
function graphs(){graphAllTemperatures();plot('graphF',waterPulseLog)}
const temperatureErrors={OK:'Измерение подтверждено',WAIT_DIAG:'Поиск датчика',WAIT_CONVERSION:'Ожидание измерения',NO_PRESENCE:'Нет ответа датчика',LINE_LOW:'Линия удерживается в нуле',NO_ROM:'ROM не прочитан',ROM_CRC_BAD:'Ошибка CRC адреса',ROM_SEARCH_NOISE:'Нестабильный поиск ROM',MULTIPLE_ROM:'Несколько ROM — выбери датчик',BOUND_ROM_MISSING:'Назначенный ROM не найден',ROM_ON_MULTIPLE_BUSES:'Один ROM найден на разных линиях',PARASITE_POWER_UNSUPPORTED:'Требуется отдельное питание датчика',BUS_POWER_UNSUPPORTED:'На общей линии есть датчик без отдельного питания',SCRATCHPAD_CRC_BAD:'Ошибка CRC измерения',POWER_ON_RESET:'Датчик перезапустился — ожидание новой конверсии',INSUFFICIENT_POWER:'Недостаточное питание датчика',CONFIG_INVALID:'Неверный ответ датчика',OUT_OF_RANGE:'Значение вне диапазона',STALE_READING:'Показание устарело',TOO_MANY_ROMS:'Превышен предел ROM на линии'};
function temperatureView(t,d={}){
 const valid=t?.verified===true&&typeof t.c==='number'&&Number.isFinite(t.c)&&t.c>=-55&&t.c<=125;
 const error=valid?'OK':t?.error||d?.result||'NO_VALID_READING';
 return {valid,value:valid?t.c:null,text:valid?t.c.toFixed(1)+' °C':'Н/Д',detail:'GPIO'+t?.gpio+' · '+(temperatureErrors[error]||error),error};
}
let bindingFingerprint='',bindingsDirty=false;
function renderTemperatureBindings(s){
 const inventory=Array.isArray(s.temperature_inventory)?s.temperature_inventory:[];
 const fingerprint=inventory.map(x=>x.bus+':'+x.rom).join('|');
 if(fingerprint!==bindingFingerprint){
  for(let i=0;i<4;i++){
   const select=get('bind'+i),previous=select.value;
   select.replaceChildren(new Option('Авто GPIO'+[16,17,18,21][i],'auto'));
   inventory.forEach(x=>select.add(new Option('GPIO'+x.gpio+' · '+x.rom,x.bus+':'+x.rom)));
   if(Array.from(select.options).some(x=>x.value===previous))select.value=previous;
  }
  bindingFingerprint=fingerprint;
 }
 if(!bindingsDirty)for(let i=0;i<4;i++){
  const select=get('bind'+i),value=s.temperatures[i]?.binding||'auto';
  if(!Array.from(select.options).some(x=>x.value===value))select.add(new Option(value+' · не обнаружен',value));
  select.value=value;
 }
 get('temperatureInventory').textContent=inventory.length?inventory.map(x=>'GPIO'+x.gpio+' · '+x.rom+' · '+(x.verified===true&&typeof x.c==='number'?x.c.toFixed(2)+' °C':temperatureErrors[x.error]||x.error)).join('\n'):'Найденных ROM пока нет.';
 get('temperatureDetails').textContent=s.temperatures.map((t,i)=>'T'+(i+1)+' '+(t.name||'')+': '+temperatureView(t,s.diagnostics?.onewire?.[i]).detail+(t.sample_age_ms===null?'':' · возраст '+Math.round(t.sample_age_ms/1000)+' с')).join(' | ');
}
function temperatureConnectionLost(){
 for(let i=0;i<4;i++){get('t'+i).textContent='НЕТ СВЯЗИ';get('ts'+i).textContent='Измерение не обновляется';get('t'+i).classList.add('warn');}
 get('lastTUpdate').textContent='Связь потеряна. Показания не подтверждены.';
 get('temperatures').textContent='Текущие измерения недоступны';
 get('temperatureDetails').textContent='Нет ответа ESP32';
 get('temperatureInventory').textContent='Нет связи. Для обновления ROM и показаний подключись к БУСА.';
 const dot=get('dataLed');if(dot)dot.classList.remove('dataOn');
}
for(let i=0;i<4;i++)get('bind'+i).onchange=()=>{bindingsDirty=true;};
get('saveBindings').onclick=async function(){
 if(!window.confirm('Сохранить выбранные ROM для T1–T4? Проверь соответствие датчиков названиям каналов.'))return;
 this.disabled=true;
 try{
  const body=new URLSearchParams({confirm:'BIND_TEMPERATURES'});for(let i=0;i<4;i++)body.set('t'+i,get('bind'+i).value);
  const r=await localFetch('/api/temperature/bind',{method:'POST',body});
  if(!r.ok)throw Error(await r.text());const j=await r.json();if(j.saved!==true)throw Error('Нет подтверждения сохранения');
  bindingsDirty=false;get('bindingResult').textContent='Привязка сохранена. Ожидание свежих измерений.';message('Привязка T1–T4 сохранена');
 }catch(e){get('bindingResult').textContent='Ошибка: '+e.message;}finally{this.disabled=false;}
};
let otaUiUploading=false;
async function update(){if(pollBusy||otaUiUploading)return;pollBusy=true;try{let r=await localFetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);let s=await r.json();get('tftThemeSelect').value=s.tft_theme_dark?'dark':'light';if(!connected)message('Связь с ESP32 восстановлена');connected=true;
get('appHealth').classList.remove('failed');
get('appHealth').textContent='ESP32 отвечает · '+s.firmware+' · обновлений: '+(++successfulStatusReads);
networkLeds(s.network_indicators,s.station?.connected?s.station.ssid:'');
// The TFT and iPhone use independent navigation; never switch iPhone screens during background polling.
let names=['Вода','Колонна','Куб','Верх колонны'];
s.temperatures.forEach((t,i)=>{
 const v=temperatureView(t,s.diagnostics?.onewire?.[i]);
 get('t'+i).textContent=v.text;get('ts'+i).textContent=v.detail;
 get('t'+i).classList.toggle('warn',!v.valid);
 const a=tlog4[i];a.push(v.valid?v.value:null);if(a.length>80)a.shift();
 if(i===0){tlog.push(v.valid?v.value:null);if(tlog.length>80)tlog.shift();}
});
renderTemperatureBindings(s);
get('lastTUpdate').textContent='Обновлено '+new Date().toLocaleTimeString('ru-RU')+' · показаны только подтверждённые измерения';
get('temperatures').textContent=names.map((v,i)=>v+': '+get('t'+i).textContent).join(' | ');
if(!refsInit && Array.isArray(s.display_references)){
  s.display_references.forEach((r,i)=>{get('ref'+i).value=r.target===null?'':r.target;get('nt'+i).value=r.notice===null?'':r.notice});
  refsInit=true;
}
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?'Н/Д · не калибровано':f.toFixed(2);get('flow2').textContent=get('flow').textContent;get('pulses').textContent=s.water.pulses;get('total').textContent=s.water.total_l_estimate===null?'Н/Д':Number(s.water.total_l_estimate).toFixed(3);
const wc=s.water?.calibration||{};
get('waterCalState').textContent=wc.calibrated?'СОХРАНЕНА':wc.session_active?'ИДЁТ ПРОБА':'НЕ ПРОВЕДЕНА';
get('waterCalPulses').textContent=wc.session_active?String(wc.session_pulses??0):'—';
get('waterCalFactor').textContent=wc.pulses_per_litre===null||wc.pulses_per_litre===undefined?'Н/Д':Number(wc.pulses_per_litre).toFixed(2);
flog.push(f);if(flog.length>80)flog.shift();
const currentPulse=Number(s.water.pulses);
if(Number.isFinite(currentPulse)){
 let delta=lastWaterPulse===null?null:Math.max(0,currentPulse-lastWaterPulse);
 lastWaterPulse=currentPulse;waterPulseLog.push(delta);
 if(waterPulseLog.length>80)waterPulseLog.shift();
}
let ss=s.station.connected?'ПОДКЛЮЧЕНО: '+s.station.ssid+' · '+s.station.ip:(s.station.ssid?'Ожидание подключения: '+s.station.ssid:'Wi-Fi не настроен. Выберите сеть ниже.');if(s.wifi_recovery?.connecting)ss='Подключение к интернету: '+(s.station.target_ssid||s.station.ssid)+' · локальная сеть включена';else if(s.wifi_recovery?.connect_pending&&!s.station.connected)ss='Ожидание попытки подключения: '+(s.station.target_ssid||s.station.ssid)+' · локальная сеть включена';get('wstatus').textContent=ss;get('mainwifi').textContent=s.station.connected?'Подключено':'Локальная сеть';
if(s.station.connected&&s.station.ssid){
 let select=get('net'),found=Array.from(select.options).some(o=>o.value===s.station.ssid);
 if(!found)select.add(new Option(s.station.ssid+' (подключена)',s.station.ssid));
 if(!select.value)select.value=s.station.ssid;
}
const cs=s.google;
const cloud=cs.cloud_update||{};
get('cloudCatalogue').textContent=cloud.catalog_seen?'Прочитан':'Нет метаданных';
get('cloudVersion').textContent=cloud.available_version||'—';
get('cloudFlash').textContent=cloud.device_flash_bytes?Math.round(cloud.device_flash_bytes/1048576)+' МБ':'—';
get('cloudBuildFlash').textContent=cloud.build_flash_bytes?Math.round(cloud.build_flash_bytes/1048576)+' МБ':'—';
const part=s.ota_preflight||{};
get('cloudOtaRunning').textContent=Number.isInteger(part.running_address)?'0x'+part.running_address.toString(16)+' · '+Math.round(part.running_size/1024)+' КБ':'—';
get('cloudOtaNext').textContent=Number.isInteger(part.next_address)?'0x'+part.next_address.toString(16)+' · '+Math.round(part.next_size/1024)+' КБ':'—';
get('cloudOtaFits').textContent=part.slot_available!==true?'Нет второго раздела':!cloud.catalog_seen?'Нет каталога':part.candidate_fits_slot===true?'По размеру да':'По размеру нет';
get('cloudSafety').textContent=cloud.catalog_seen&&!cloud.flash_size_matches?'Размер Flash устройства отличается от каталога. Установка запрещена.':'Облачная установка отключена. Реальные разделы OTA отображаются выше, но подпись, совместимость и безопасный откат пока не проверены.';
const dataDot=get('dataLed'),dataLabel=get('dataLabel');
const delivered=cs?.configured===true&&cs.state===5&&cs.http===200&&cs.sent>0;
if(dataDot)dataDot.classList.toggle('dataOn',delivered);
if(dataLabel)dataLabel.textContent=delivered?'подтверждено':cs?.configured===true?'ожидание':'выключено';
get('displayState').textContent=s.display_sleep?'ПОГАШЕН':'РАБОТАЕТ';
if(!themeInitialized && typeof s.tft_theme_dark==='boolean'){
  document.documentElement.dataset.theme=s.tft_theme_dark?'dark':'light';
  themeInitialized=true;
}
get('googleState').textContent=!cs.configured?'НЕ НАСТРОЕН':cs.state===5?'ОТВЕТ ПОЛУЧЕН':cs.state===4?'ПЕРЕДАНО, ПРОВЕРЬТЕ ТАБЛИЦУ':cs.state===3?'ОТПРАВКА':cs.state===2?'ВРЕМЯ':cs.state===1?'НЕТ ИНТЕРНЕТА':cs.state===6?'ОШИБКА':cs.state===7?'ОТКЛОНЕНО':'ОЖИДАНИЕ';
get('googleResult').textContent='HTTP '+cs.http+' · удачных подтверждений '+cs.sent+' · DNS '+(cs.dns??'?')+' · TLS '+(cs.tls_error??'?')+' · этап '+(cs.error_stage??'?');
get('updateResult').textContent=s.ota.message+(s.ota.active?' '+s.ota.bytes+' байт':'');
if(s.keypad){
 get('keyState').textContent=s.keypad.learning?
   ('Обучение '+s.keypad.step+'/20'):(s.keypad.ready?'Сохранена':'НЕТ');
 get('keyExpected').textContent=s.keypad.expected||'—';
 get('keyRaw').textContent=s.keypad.raw_pair||'—';
 if(s.diagnostics){
   get('diagOneWire').textContent=(s.diagnostics.onewire||[]).map((p,i)=>
     'T'+(i+1)+' GPIO'+p.gpio+': '+p.result+' | IDLE='+p.idle_gpio+
     ' | AFTER='+p.after_reset_gpio+' | RESET='+p.presence+
     ' | ROM='+(p.rom||'—')+' | CRC='+p.rom_crc_ok).join('\n');
   const k=s.diagnostics.keyboard||{};
   get('diagKeyboard').textContent='PCF 0x'+Number(k.pcf_address||0).toString(16)+
      ' | idle=0x'+k.idle_hex+' | low=0x'+k.low_hex+
      ' | pair='+k.raw_pair+' | error='+k.scan_error;
   const d=s.diagnostics,addresses=(d.i2c?.addresses||[]);
   get('diagI2c').textContent='I2C GPIO8/9 | ACK total='+d.i2c?.ack_total+
     ' | addresses='+addresses.map(a=>'0x'+Number(a).toString(16).padStart(2,'0')).join(', ')+
     '\nPCF8575: ACK='+d.pcf8575?.online+' read='+d.pcf8575?.read_ok+
     ' idle=0x'+d.pcf8575?.idle_pins_hex+
     '\nPCA9685: ACK='+d.pca9685?.ack+' MODE1='+d.pca9685?.mode1+
     ' MODE2='+d.pca9685?.mode2+' PWM disabled';
   get('diagOther').textContent=
     'TFT: '+d.tft?.profile+' rotation='+d.tft?.rotation+
       ' init='+d.tft?.init_sent+' (pixels not verified by software)'+
     '\nESP32: '+d.esp32?.chip+' heap='+d.esp32?.free_heap_bytes+
       ' flash='+d.esp32?.flash_bytes+' reset='+d.esp32?.reset_reason_code+
     '\nWi-Fi: AP='+d.wifi?.ap_running+' clients='+d.wifi?.ap_clients+
       ' STA='+d.wifi?.sta_connected+' RSSI='+d.wifi?.sta_rssi_dbm+
     '\nYF-S401 GPIO15: input='+d.water_flow?.input_level+
       ' pulses='+d.water_flow?.pulses+' '+d.water_flow?.status+' (not calibrated)';
   lastDiagnosticReport=JSON.stringify({
     device:s.device,firmware:s.firmware,uptime_ms:s.uptime_ms,
     temperatures:s.temperatures,temperature_inventory:s.temperature_inventory,modules:s.modules,diagnostics:d,
     station_connected:s.station?.connected,ap_enabled:s.ap?.enabled,
     network_indicators:s.network_indicators,google:s.google,
     water_calibration:s.water?.calibration,
     wifi_recovery:s.wifi_recovery,web_health:s.web_health,
     relay_outputs_enabled:s.relays_enabled
   },null,2);
   const report=get('diagReport');
   if(document.activeElement!==report)report.value=lastDiagnosticReport;
 }
 if(s.keypad.message)get('keyInfo').textContent=s.keypad.message;
}
if(s.relay_simulation){
 get('relaySimChannel').textContent=String(s.relay_simulation.simulated_channel||0)+' / 4 (ТОЛЬКО ИМИТАЦИЯ)';
 get('relaySimRemaining').textContent=Math.ceil((s.relay_simulation.remaining_ms||0)/1000)+' с';
}
if(s.leds){
  get('ledPreflight').textContent=s.leds.physical_test_ready?'РАЗРЕШЁН':'ЗАБЛОКИРОВАН: ТОК/OE НЕ ПОДТВЕРЖДЕНЫ';
  get('ledSimActive').textContent=s.leds.sim_channel>=0?('CH'+s.leds.sim_channel+' · ТОЛЬКО ИМИТАЦИЯ'):'НЕТ';
  get('ledSimRemaining').textContent=Math.ceil((s.leds.sim_remaining_ms||0)/1000)+' с';
  get('ledSimModes').textContent=String(s.leds.mode1??'—')+' / '+String(s.leds.mode2??'—');
  get('ledSimRead').textContent=String(s.leds.read_ok_total)+' успешно / '+String(s.leds.read_error_total)+' ошибок';
}
get('ver').textContent=s.firmware;get('uptime').textContent=Math.floor(s.uptime_ms/1000)+' сек';get('pcf').textContent=s.modules.pcf8575?'Найден':'Не найден';get('pca').textContent=s.modules.pca9685_ack_only?'Найден':'Не найден';get('tft').textContent=s.modules.tft_attempted?('Контроллер '+s.modules.tft_profile+', поворот '+s.modules.tft_rotation+', загрузка '+(s.modules.tft_boot_complete?'завершена':'идёт')+' (пиксели не подтверждены)'):'Запуск ещё не выполнен';if(!tftProfileLoaded){get('tftDriver').value=String(s.modules.tft_profile_id||1);get('tftRotation').value=String(s.modules.tft_rotation===undefined?1:s.modules.tft_rotation);tftProfileLoaded=true}if(active===3)graphs();
}catch(e){
  lastStatusError=e?.message||String(e);
  if(connected)message('Потеряна связь с ESP32');
  connected=false;networkLeds(null,'');
  temperatureConnectionLost();
  get('appHealth').classList.add('failed');
  get('appHealth').textContent='Нет ответа /api/status: '+lastStatusError+'. Проверь сеть БУСА или открой «Проверка связи».';
}finally{pollBusy=false}}
document.querySelectorAll('[data-relay-sim]').forEach(b=>b.onclick=async()=>{
  if(!confirm('Только виртуальный тест K'+b.dataset.relaySim+' на 10 секунд. Физические реле не включатся. Продолжить?'))return;
  try{
    const r=await localFetch('/api/relay/sim',{method:'POST',body:new URLSearchParams({
      action:'single',channel:b.dataset.relaySim,confirm:'DRY_RUN_ONLY'})});
    get('relaySimResult').textContent=await r.text();
  }catch(e){get('relaySimResult').textContent='Ошибка симуляции: '+e.message}
});
get('relaySimStop').onclick=async()=>{
 try{
   const r=await localFetch('/api/relay/sim',{method:'POST',
      body:new URLSearchParams({action:'stop'})});
   get('relaySimResult').textContent=await r.text();
 }catch(e){get('relaySimResult').textContent='Ошибка остановки: '+e.message}
};
document.querySelectorAll('[data-led-channel]').forEach(b=>b.onclick=async()=>{
  if(!confirm('Виртуальная проверка CH'+b.dataset.ledChannel+' на 8 секунд без подачи сигнала на лампочку?'))return;
  try{
    const r=await localFetch('/api/led/sim',{method:'POST',body:new URLSearchParams({action:'test',channel:b.dataset.ledChannel,confirm:'DRY_RUN_ONLY'})});
    get('ledSimResult').textContent=await r.text();
  }catch(e){get('ledSimResult').textContent='Ошибка: '+e.message}
});
get('ledSimStop').onclick=async()=>{
  try{
    const r=await localFetch('/api/led/sim',{method:'POST',body:new URLSearchParams({action:'stop'})});
    get('ledSimResult').textContent=await r.text();
  }catch(e){get('ledSimResult').textContent='Ошибка: '+e.message}
};
get('waterCalStart').onclick=async()=>{
 if(!confirm('Начать запись импульсов для будущей пробы обычной воды?'))return;
 try{const r=await localFetch('/api/water/calibration/start',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
 get('waterCalResult').textContent=await r.text();}
 catch(e){get('waterCalResult').textContent='Ошибка: '+e.message}
};
get('waterCalFinish').onclick=async()=>{
 const value=get('waterCalVolume').value.trim(),n=Number(value);
 if(!value||!Number.isFinite(n)||n<100||n>50000){get('waterCalResult').textContent='Укажите измеренный объём 100–50000 мл';return}
 if(!confirm('Сохранить коэффициент по действительно измеренному объёму обычной воды?'))return;
 try{const r=await localFetch('/api/water/calibration/finish',{method:'POST',
 body:new URLSearchParams({volume_ml:value,confirm:'YES'})});
 get('waterCalResult').textContent=await r.text();}
 catch(e){get('waterCalResult').textContent='Ошибка: '+e.message}
};
get('waterCalReset').onclick=async()=>{
 if(!confirm('Стереть сохранённый коэффициент YF-S401?'))return;
 try{const r=await localFetch('/api/water/calibration/reset',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
 get('waterCalResult').textContent=await r.text();}
 catch(e){get('waterCalResult').textContent='Ошибка: '+e.message}
};
get('googleSave').onclick=async()=>{
const url=get('googleUrl').value.trim(),token=get('googleToken').value.trim();
if(!url.startsWith('https://script.google.com/macros/s/')||!url.endsWith('/exec')||!/^[a-fA-F0-9]{64}$/.test(token)){
 get('googleResult').textContent='Требуется ссылка Apps Script /exec и секрет из 64 шестнадцатеричных символов';return;
}
if(!confirm('Включить отправку показаний в свой Google Диск? Только мониторинг.'))return;
let resp=await localFetch('/api/google/config',{method:'POST',body:new URLSearchParams({url,token})});
get('googleResult').textContent=await resp.text();if(resp.ok)get('googleToken').value='';
};
get('googleTest').onclick=async()=>{
  try{
    const r=await localFetch('/api/google/test',{method:'POST',
      body:new URLSearchParams({confirm:'YES'})});
    get('googleResult').textContent=await r.text();
  }catch(e){get('googleResult').textContent='Ошибка проверки: '+e.message}
};
get('googleStop').onclick=async()=>{
if(!confirm('Отключить отправку диагностических данных в Google?'))return;
const resp=await localFetch('/api/google/off',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
get('googleResult').textContent=await resp.text();
};
get('manualUpdate').onclick=async function(){
  const file=get('firmwareFile').files[0];
  if(!file || !file.name.endsWith('.bin') || file.size<60000 || file.size>3340000){
    get('updateResult').textContent='Нужен APP BIN подходящего размера';return;
  }
  if(!confirm('Подтвердите ручное обновление диагностической прошивки. ESP32-S3 N8R2, внешние выходы отключены, выбран APP BIN?'))return;
  this.disabled=true;
  try{
    const header=new DataView(await file.slice(0,36).arrayBuffer());
    if(header.byteLength<36||header.getUint8(0)!==0xe9||header.getUint16(12,true)!==9||header.getUint32(32,true)!==0xabcd5432)throw Error('Выбран не APP ESP32-S3. USB MERGED нельзя передавать по OTA');
    const begin=await localFetch('/api/ota/prepare',{method:'POST',body:new URLSearchParams({confirm:'YES',size:String(file.size)})});
    if(!begin.ok)throw Error(await begin.text());
    const grant=await begin.json(),body=new FormData();
    body.append('firmware',file,file.name);
    get('updateResult').textContent='Загрузка APP BIN по Wi-Fi — не отключайте БУСА';
    otaUiUploading=true;
    let lastOtaPaint=0;
    const t0=Date.now();
    await new Promise((resolve,reject)=>{
      const xhr=new XMLHttpRequest();
      xhr.open('POST','/api/ota/upload',true);
      xhr.setRequestHeader('X-Busa-Token',grant.token);
      xhr.upload.onprogress=e=>{
        if(!e.lengthComputable)return;
        const ts=Date.now();if(ts-lastOtaPaint<180&&e.loaded<e.total)return;
        lastOtaPaint=ts;
        const secs=Math.max((ts-t0)/1000,0.1);
        const pct=Math.min(100,Math.round(e.loaded*100/e.total));
        get('updateResult').textContent='Передача APP BIN: '+pct+'% · '+(e.loaded/1024).toFixed(0)+' КБ · '+(e.loaded/1024/secs).toFixed(1)+' КБ/с';
      };
      xhr.onload=()=>{try{const result=JSON.parse(xhr.responseText);if(xhr.status!==200||result.success!==true)throw Error(result.message||'HTTP '+xhr.status);resolve(result);}catch(e){reject(e)}};
      xhr.onerror=()=>reject(Error('Wi-Fi связь прервана при передаче'));
      xhr.ontimeout=()=>reject(Error('Время передачи истекло'));
      xhr.timeout=180000;
      xhr.send(body);
    });
    get('updateResult').textContent='APP BIN записан. БУСА перезапускается';
  }catch(e){get('updateResult').textContent='Ошибка: '+e.message}
  finally{otaUiUploading=false;this.disabled=false;}
};
get('net').onchange=()=>{if(get('net').value)get('response').textContent='Выбрана сеть: '+get('net').value+'. Введите пароль для подключения.';};
get('scan').onclick=async function(){
  this.disabled=true;get('response').textContent='Идёт поиск сетей...';
  try{
    let r=await localFetch('/api/wifi/scan?start=1',{cache:'no-store'});
    if(!r.ok)throw Error('сервер поиска вернул HTTP '+r.status);
    let j=await r.json();
    for(let i=0;i<20&&j.scanning;i++){
      await new Promise(done=>setTimeout(done,1000));
      r=await localFetch('/api/wifi/scan',{cache:'no-store'});
      if(!r.ok)throw Error('сервер поиска вернул HTTP '+r.status);
      j=await r.json();
    }
    let sel=get('net'),previous=sel.value;
    sel.innerHTML='<option value="">Выберите сеть</option>';
    const seen=new Set();
    (j.networks||[]).forEach(w=>{if(!w.ssid||seen.has(w.ssid))return;seen.add(w.ssid);sel.add(new Option(w.ssid+' ('+w.rssi+' dBm)',w.ssid));});
    if(previous&&seen.has(previous))sel.value=previous;
    const count=sel.options.length-1;
    get('response').textContent=j.scanning?'Поиск ещё выполняется. Нажми повторно через несколько секунд.':
      j.error?'Ошибка поиска: '+j.error+'. Открой Диагностику и скопируй отчёт.':
      count?'Найдено сетей: '+count:'Найдено 0 сетей. Проверь 2,4 ГГц и скопируй диагностику.';
  }catch(e){get('response').textContent='Ошибка поиска: '+e.message+'. Скопируй диагностику.';}
  finally{this.disabled=false;}
};
get('save').onclick=async function(){
  const ssid=get('manualSsid').value.trim()||get('net').value,pass=get('pass').value;
  const nameBytes=new TextEncoder().encode(ssid).length;
  if(!ssid||nameBytes>32||pass.length<8||pass.length>63){
    get('response').textContent='Выбери сеть или введи её имя и пароль (8–63 символа).';return;
  }
  this.disabled=true;get('response').textContent='Сохраняю Wi-Fi...';
  try{
    const r=await localFetch('/api/wifi',{method:'POST',body:new URLSearchParams({ssid,pass}),cache:'no-store'});
    get('response').textContent=await r.text();
    if(r.ok){get('pass').value='';message('Настройки сети сохранены: '+ssid);}
  }catch(e){get('response').textContent='Ошибка: '+e.message;}
  finally{this.disabled=false;}
};
message('Веб-приложение запущено');
showPhoneClock();setInterval(showPhoneClock,1000);
syncClock();setInterval(syncClock,300000);
update();setInterval(update,1500);
</script></body></html>
)HTMLBUSA";

void handleCommand(String x){
  x.trim();x.toUpperCase();
  if(x=="STATUS") {Serial0.println(statusJSON());Serial.println(statusJSON());}
  else if(x.startsWith("PAGE ")){
    int n=x.substring(5).toInt();
    if(n>=1&&n<=6){navigatePage((uint8_t)(n-1));editingTemp=false;tftView=0;tftDirty=true;lastRender=0;Serial0.printf("PAGE=%d\n",n);}
  }
  else if(x=="TFT ST7789"){
    tftProfile=1;
    prefs.putUChar("tft",tftProfile);
    Serial0.println("TFT profile saved. Reset board to apply.");
  }
  else Serial0.println("Commands: STATUS | PAGE 1..6 | TFT ST7789");
}
void readConsole(){
  // CH343 UART0 (COM6 on current board) is diagnostic console.
  while(Serial0.available()){
    char c=(char)Serial0.read();
    if(c=='\n'){handleCommand(consoleLine);consoleLine="";}
    else if(c!='\r'&&consoleLine.length()<80)consoleLine+=c;
  }
}
void setup(){
  relaySimStop("BOOT_SAFE");
  ledSimStop("BOOT_LOCKED");
  Serial0.begin(115200);
  Serial0.println("\nBUSA 1M WiFi V14.1 LIGHT-WATER; physical outputs LOCKED");
  Serial.begin(115200);
  delay(400);
  pinMode(FLOW_GPIO,INPUT); // requires an electrically verified 3.3 V logic signal
  attachInterrupt(digitalPinToInterrupt(FLOW_GPIO),flowISR,FALLING);
  Serial0.println("WATER GPIO15 interrupt attached (input only)");
  prefs.begin("busa-r04",false);
  TempBindingRecord bindings={};
  if(prefs.getBytesLength("temp_bind")==sizeof(bindings) &&
     prefs.getBytes("temp_bind",&bindings,sizeof(bindings))==sizeof(bindings) && tempBindingRecordValid(bindings)){
    memcpy(tempBindings,bindings.bindings,sizeof(tempBindings));
  }
  waterPulsesPerLiter=prefs.getFloat("wf_ppl",NAN);
  waterCalibrated=prefs.getBool("wf_ready",false) && isfinite(waterPulsesPerLiter) && waterPulsesPerLiter>=10.0f && waterPulsesPerLiter<=500000.0f;
  if(!waterCalibrated)waterPulsesPerLiter=NAN;
  // User-approved ST7789 320x240 rotation 1 and software-drawn light theme.
  if(!prefs.getBool("v14lightSt",false)){
    prefs.putUChar("tft",1);
    prefs.putUChar("tftRotate",1);
    prefs.putBool("tftInvert",false);
    prefs.putBool("v14lightSt",true);
  }
  tftProfile=1;tftRotation=1;tftInverted=false;
  tftThemeDark=prefs.getBool("tftDarkColor",false);
  startedAt=millis();
  tftProfile=1; // ST7789 only
  // Initialize TFT first so splash actually covers Wi-Fi and sensor initialization.
  screenAttempted=true;
  startScreen();
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  loadKeymap();
  clockZoneKnown=prefs.getBool("clkZone",false);
  clockUtcOffsetMinutes=prefs.getInt("clkTz",0);
  if(clockUtcOffsetMinutes<(-12*60)||clockUtcOffsetMinutes>14*60)clockZoneKnown=false;
  for(int i=0;i<4;i++){
    String k=String("rf")+String(i),j=String("nt")+String(i);
    refTemp[i]=prefs.getFloat(k.c_str(),NAN);
    noticeTemp[i]=prefs.getFloat(j.c_str(),NAN);
    if(!isfinite(refTemp[i])||refTemp[i]<0||refTemp[i]>125)refTemp[i]=NAN;
    if(!isfinite(noticeTemp[i])||noticeTemp[i]<0||noticeTemp[i]>125)noticeTemp[i]=NAN;
  }
  setupGoogleCloud();
  WiFi.persistent(false); // Credentials live only in application Preferences.
  WiFi.setAutoReconnect(false); // Only wifiRecoveryTick owns STA attempts.
  WiFi.onEvent(wifiRecoveryEvent);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false); // Favor interactive local response over power saving
  configTime(0,0,"pool.ntp.org","time.google.com"); // NTP UTC; local offset from paired phone
  clockNtpStarted=true;
  // Private STA password must be provisioned into NVS after public GitHub build;
  // no STA password is embedded in this public source. Local AP fallback remains.
  apPassword="BUSA"+String((uint32_t)ESP.getEfuseMac(),HEX);
  wifiRunning=WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str(),BUSA_AP_CHANNEL,false,4);
  esp_wifi_set_bandwidth(WIFI_IF_AP,WIFI_BW_HT20);
  updateBootStage(1);
  Serial0.printf("AP=%d IP=%s SSID=BUSA-1M-001-R04\n",
                 (int)wifiRunning,WiFi.softAPIP().toString().c_str());
  wifiStartPolicy(millis()); // Brief startup grace; local clients do not block internet association.
  server.on("/",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store, max-age=0");
    server.send_P(200,"text/html; charset=utf-8",WEB_PAGE);
  });
  // Minimal independent endpoints for diagnosing an unreachable main app.
  server.on("/api/ping",HTTP_GET,[]{
    String out="{\"device\":\"BUSA-1M-001\",\"version\":"+quoteJson(String(VERSION));
    out+=",\"uptime_ms\":"+String(millis());
    out+=",\"ap_running\":"+(wifiRunning?String("true"):String("false"));
    out+=",\"ap_clients\":"+String(WiFi.softAPgetStationNum());
    out+=",\"sta_connected\":"+(WiFi.status()==WL_CONNECTED?String("true"):String("false"));
    out+=",\"free_heap\":"+String(ESP.getFreeHeap())+"}";
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",out);
  });
  server.on("/recovery",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"text/html; charset=utf-8",
      String("<!doctype html><html lang='ru'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>")
      +"<title>БУСА · проверка связи</title><body style='font:17px system-ui;padding:16px'>"
      +"<h2>БУСА-1М · Проверка связи</h2><p>Версия: "+String(VERSION)+"</p>"
      +"<p>Локальная сеть: "+(wifiRunning?String("работает"):String("нет"))+"</p>"
      +"<p>Клиентов: "+String(WiFi.softAPgetStationNum())+"</p>"
      +"<p>Подключение к интернету: "+(WiFi.status()==WL_CONNECTED?String("да"):String("нет"))+"</p>"
      +"<p><a href='/api/ping'>Ответ ESP32 (JSON)</a></p>"
      +"<p><a href='/'>Открыть приложение</a></p></body></html>");
  });
  server.on("/api/time/sync",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная сеть БУСА");return;
    }
    String e=server.arg("epoch"),tz=server.arg("tz");
    char *end=nullptr,*tzEnd=nullptr;
    long long sec=strtoll(e.c_str(),&end,10);
    long offset=strtol(tz.c_str(),&tzEnd,10);
    if(e.isEmpty()||!end||*end||tz.isEmpty()||!tzEnd||*tzEnd||
       sec<1609459200LL||sec>4102444800LL||offset<-720||offset>840){
       server.send(400,"text/plain","Неверные дата/время/часовой пояс");return;
    }
    timeval tv;tv.tv_sec=(time_t)sec;tv.tv_usec=0;
    if(settimeofday(&tv,nullptr)!=0){
      server.send(500,"text/plain","Не удалось синхронизировать часы");return;
    }
    clockUtcOffsetMinutes=(int32_t)offset;clockZoneKnown=true;
    prefs.putInt("clkTz",clockUtcOffsetMinutes);
    prefs.putBool("clkZone",true);
    Serial0.println("CLOCK: synchronized from phone, UTC offset stored");
    // V14.6 live clock refreshes its own header strip; no full-screen flash.
    if(screen && screen->width()<300)tftDirty=true;
    server.send(200,"application/json",String("{\"datetime\":")+quoteJson(clockStamp(false))+"}");
  });
  server.on("/api/temperature/bind",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="BIND_TEMPERATURES"||otaActive||otaRestart){
      server.send(403,"text/plain; charset=utf-8","Нужны локальная сеть и подтверждение привязки");return;
    }
    TempBindingRecord record={};record.magic=0x42544D50U;record.version=1;
    for(int i=0;i<4;i++){
      if(!tempParseSelection(server.arg(String("t")+String(i)),record.bindings[i])){
        server.send(400,"text/plain; charset=utf-8","Неверный GPIO или ROM датчика");return;
      }
    }
    if(!tempValidateBindings(record.bindings)){
      server.send(409,"text/plain; charset=utf-8","Датчик не найден или один ROM назначен двум каналам. Обнови список.");return;
    }
    record.crc=OneWire::crc8((const uint8_t*)&record,offsetof(TempBindingRecord,crc));
    if(prefs.putBytes("temp_bind",&record,sizeof(record))!=sizeof(record)){
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить привязку");return;
    }
    memcpy(tempBindings,record.bindings,sizeof(tempBindings));tempSyncLogical(millis());tftDirty=true;
    server.send(200,"application/json","{\"saved\":true}");
  });
  server.on("/api/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",statusJSON());
  });
  // Local Wi-Fi-only in-memory simulation. No direct GPIO, coil or PWM actuation.
  server.on("/api/relay/sim",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Только защищённая локальная сеть");
      return;
    }
    const String action=server.arg("action");
    if(action=="stop"){
      relaySimStop("EMERGENCY_STOP");
      server.send(200,"application/json",relaySimJSON());return;
    }
    if(action!="single"||server.arg("confirm")!="DRY_RUN_ONLY"||otaActive){
      server.send(403,"text/plain; charset=utf-8","Только подтверждённая симуляция без физических сигналов");return;
    }
    const String ch=server.arg("channel");
    if(ch.length()!=1||ch[0]<'1'||ch[0]>'4'||relaySimChannel){
      server.send(409,"text/plain; charset=utf-8","Только один виртуальный канал за раз");return;
    }
    relaySimChannel=(uint8_t)(ch[0]-'0');
    relaySimStart=millis();relaySimUntil=relaySimStart+RELAY_SIM_MAX_MS;
    relaySimReason="DRY_RUN_ONLY";
    server.send(200,"application/json",relaySimJSON());
  });
  server.on("/api/led/sim",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Требуется защищённая локальная сеть БУСА");return;
    }
    const String action=server.arg("action");
    if(action=="stop"){
      ledSimStop("STOP_REQUESTED");
      server.send(200,"application/json",ledSimJSON());return;
    }
    if(action!="test"||server.arg("confirm")!="DRY_RUN_ONLY"||otaActive){
      server.send(403,"text/plain; charset=utf-8","Разрешена только виртуальная проверка LED");return;
    }
    const String arg=server.arg("channel");
    if(arg.isEmpty()||arg.length()>2){
      server.send(400,"text/plain","Неверный канал");return;
    }
    for(unsigned i=0;i<arg.length();i++)if(arg[i]<'0'||arg[i]>'9'){
      server.send(400,"text/plain","Неверный номер");return;
    }
    const int ch=arg.toInt();
    if(ch<0||ch>=BusaLed::kCount||ledSimChannel>=0){
      server.send(409,"text/plain","Тест занят или канал недоступен");return;
    }
    ledSimChannel=(int8_t)ch;ledSimUntil=millis()+LED_SIM_MAX_MS;
    ledSimReason="DRY_RUN_ONLY";
    server.send(200,"application/json",ledSimJSON());
  });
  // All calibration actions are low-voltage, read-only monitoring settings.
  // Never expose these write operations through the public Internet.
  server.on("/api/water/calibration/start",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    waterCalStartPulses=flowPulses();waterCalSession=true;
    server.send(200,"text/plain; charset=utf-8","Старт записан. Измеренные импульсы считаются, пока активна проба.");
  });
  server.on("/api/water/calibration/finish",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    if(!waterCalSession){
      server.send(409,"text/plain; charset=utf-8","Сначала начните пробу калибровки");return;
    }
    const String input=server.arg("volume_ml");
    const float measuredMl=input.toFloat();
    if(input.isEmpty()||!isfinite(measuredMl)||measuredMl<100.0f||measuredMl>50000.0f){
      server.send(400,"text/plain; charset=utf-8","Укажите реальный измеренный объём воды: 100–50000 мл");return;
    }
    const uint32_t pulseDelta=flowPulses()-waterCalStartPulses;
    if(pulseDelta<10){
      server.send(400,"text/plain; charset=utf-8","Импульсов недостаточно для калибровки");return;
    }
    const float factor=(float)pulseDelta*1000.0f/measuredMl;
    if(!isfinite(factor)||factor<10.0f||factor>500000.0f){
      server.send(400,"text/plain; charset=utf-8","Получен неправдоподобный коэффициент. Проверьте объём и импульсы.");return;
    }
    if(prefs.putFloat("wf_ppl",factor)==0||prefs.putBool("wf_ready",true)==0){
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить калибровку");return;
    }
    waterPulsesPerLiter=factor;waterCalibrated=true;waterCalSession=false;
    waterCalLastPulses=pulseDelta;waterCalLastMl=measuredMl;
    server.send(200,"text/plain; charset=utf-8",
      String("Калибровка сохранена: ")+String(pulseDelta)+" имп. / "+
      String(measuredMl,1)+" мл; "+String(factor,2)+" имп./л");
  });
  server.on("/api/water/calibration/reset",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    prefs.remove("wf_ready");prefs.remove("wf_ppl");
    waterCalibrated=false;waterCalSession=false;waterPulsesPerLiter=NAN;flowLpm=NAN;
    server.send(200,"text/plain; charset=utf-8","Калибровка сброшена; отображаются только импульсы.");
  });

  addGoogleRoutes();
  server.on("/api/ota/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",otaJSON());
  });
  server.on("/api/ota/prepare",HTTP_POST,[]{
    relaySimStop("OTA_PREPARE");
    ledSimStop("OTA_PREPARE");
    if(!apClient()||server.arg("confirm")!="YES"||otaActive||otaRestart||wifiScanBusy()){
      server.send(403,"text/plain","Требуется локальная сеть БУСА и согласие");return;
    }
    const String sizeText=server.arg("size");
    char *sizeEnd=nullptr;
    const unsigned long size=strtoul(sizeText.c_str(),&sizeEnd,10);
    const esp_partition_t* slot=esp_ota_get_next_update_partition(nullptr);
    if(sizeText.isEmpty()||!sizeEnd||*sizeEnd||!slot||size<60000||size>slot->size){
      server.send(400,"text/plain","APP size / OTA partition invalid");return;
    }
    otaExpectedBytes=size;otaSuccess=false;otaUploadCompleted=false;
    char token[17];
    snprintf(token,sizeof(token),"%08lx%08lx",
      (unsigned long)esp_random(),(unsigned long)esp_random());
    otaToken=token;otaTokenIssued=millis();
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"token\":")+quoteJson(otaToken)+"}");
  });
  server.on("/api/ota/upload",HTTP_POST,[]{
    server.sendHeader("Cache-Control","no-store");
    const bool completed=otaUploadCompleted;
    if(!completed){server.send(400,"application/json","{\"success\":false,\"message\":\"No completed upload\"}");return;}
    server.send(otaSuccess?200:400,"application/json",otaJSON());
    otaUploadCompleted=false;
  },uploadOTA);
  server.on("/api/wifi/scan",HTTP_GET,[]{
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"application/json","{\"error\":\"Only protected local AP\"}");return;
    }
    if(otaActive || otaRestart){
      server.send(409,"application/json","{\"error\":\"ota_busy\"}");return;
    }
    if(server.hasArg("start") && server.arg("start")=="1")wifiRequestScan(millis());
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",wifiScanJSON());
  });
  server.on("/api/wifi",HTTP_POST,[]{
    // Credentials may only be provisioned while connected to the protected local AP.
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"text/plain; charset=utf-8","Настройка Wi-Fi доступна только через сеть БУСА.");return;
    }
    if(otaActive||otaRestart){server.send(409,"text/plain","OTA busy");return;}
    String ssid=server.arg("ssid"),pass=server.arg("pass");
    ssid.trim();
    if(ssid.isEmpty()||ssid.length()>32||pass.length()<8||pass.length()>63){
      server.send(400,"text/plain; charset=utf-8","Недопустимое имя сети или длина пароля (8–63).");return;
    }
    // NVS is encrypted only if board flash encryption is enabled; keep local AP secure.
    bool ok=(prefs.putString("ssid",ssid)>0 && prefs.putString("pass",pass)>0);
    if(!ok){server.send(500,"text/plain; charset=utf-8","Не удалось сохранить Wi-Fi настройки");return;}
    stationSsid=ssid;stationPass=pass;
    wifiCancelRequested=true;
    wifiScheduleConnect(millis());
    server.send(202,"text/plain; charset=utf-8","Wi-Fi сохранён. БУСА подключится к интернету автоматически; оставайся в локальной сети. При смене радиоканала возможна краткая пауза, приложение повторит запрос.");
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>6){server.send(400,"application/json","{\"error\":\"page 1..6\"}");return;}
    navigatePage((uint8_t)(n-1));editingTemp=false;tftView=0;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/tft/profile",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная сеть БУСА");return;
    }
    if(server.arg("profile")!="1"||server.arg("rotation")!="1"){
      server.send(400,"text/plain; charset=utf-8","Зафиксированы ST7789 и поворот 1");return;
    }
    prefs.putUChar("tft",1);prefs.putUChar("tftRotate",1);
    server.send(200,"text/plain; charset=utf-8","ST7789 320×240 сохранён");
  });
  server.on("/api/tft/screen",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;}
    String action=server.arg("action");
    if(action=="wake"){
      wakeTftDisplay();
      server.send(200,"text/plain; charset=utf-8","TFT включён");
    }else if(action=="sleep"){
      darkenTftDisplay();
      server.send(200,"text/plain; charset=utf-8","TFT погашен; ESP32 продолжает мониторинг");
    }else server.send(400,"text/plain; charset=utf-8","Неверная команда");
  });
  server.on("/api/tft/test",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    if(!tftStarted||!screen){
      server.send(409,"text/plain; charset=utf-8","Дисплей ещё не запускался");return;
    }
    drawBootSplash(5000);
    lastRender=0;
    server.send(200,"text/plain; charset=utf-8",
      "Тест отправлен: 5 секунд чёрного фона и белой надписи BUSA 1M AI.");
  });
  server.on("/api/tft/theme",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;
    }
    const String theme=server.arg("theme");
    if(theme!="dark"&&theme!="light"){
      server.send(400,"application/json","{\"error\":\"use explicit theme dark or light\"}");return;
    }
    const bool dark=theme=="dark";
    if(dark!=tftThemeDark && prefs.putBool("tftDarkColor",dark)!=1){
      server.send(500,"application/json","{\"error\":\"theme persist failed\"}");return;
    }
    tftThemeDark=dark;
    tftDirty=true;lastRender=0;
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",
      String("{\"theme\":")+(dark?"\"dark\"":"\"light\"")+",\"saved\":true}");
  });
  server.on("/api/keypad/start",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(!pcfDetected){server.send(409,"text/plain","PCF8575 not found");return;}
    startKeyLearning();
    server.send(200,"text/plain; charset=utf-8","Обучение: нажми F1 и отпусти");
  });
  server.on("/api/keypad/next",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain","Local AP only");return;
    }
    if(!keypadLearning){
      server.send(409,"text/plain; charset=utf-8","Обучение сейчас не запущено");return;
    }
    // Try a fresh scan if the key is still held; otherwise use the last pair
    // observed at the CURRENT step, only within the last 10 seconds.
    int detected=scanKeyPair(),pair=-1;
    if(detected>=0)pair=detected;
    else if(candidatePair>=0&&candidateStep==learnStep &&
            !elapsed(millis(),candidateAt,10000))pair=candidatePair;
    if(pair<0){
      keypadMessage=detected==-2?
        "Несколько линий замкнуты: отпусти другие клавиши":
        "Сигнала нет: F1 или текущая клавиша не распознана. Повтори нажатие.";
      server.send(409,"text/plain; charset=utf-8",keypadMessage);return;
    }
    if(!learnCommitPair(pair)){
      server.send(409,"text/plain; charset=utf-8",keypadMessage);return;
    }
    keyHeld=true; // Avoid double submission of the same held key
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
  });
  server.on("/api/keypad/undo",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(!keypadLearning||learnStep==0){server.send(409,"text/plain","No previous step");return;}
    learnStep--;newPairs[learnStep]=255;
    pendingPair=-1;keyHeld=false;candidatePair=-1;candidateStep=255;
    keypadMessage="Повтори "+String(KEY_CAPS[learnStep]);
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
  });
  server.on("/api/keypad/cancel",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    keypadLearning=false;learnStep=0;candidatePair=-1;candidateStep=255;
    keypadMessage=keypadReady?"Сохранённая раскладка сохранена":"Обучение отменено";
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
  });
  server.on("/api/display-refs",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Нужна локальная точка доступа БУСА");return;}
    float target[4],notice[4];
    for(int i=0;i<4;i++){
      String a=server.arg(String("ref")+i),b=server.arg(String("nt")+i);
      target[i]=a.length()?a.toFloat():NAN;
      notice[i]=b.length()?b.toFloat():NAN;
      if((isfinite(target[i])&&(target[i]<0||target[i]>125))||
         (isfinite(notice[i])&&(notice[i]<0||notice[i]>125))){
         server.send(400,"text/plain","Диапазон 0..125");return;
      }
    }
    for(int i=0;i<4;i++){
      String k=String("rf")+i,j=String("nt")+i;
      if(isfinite(target[i]))storeTempReference(i,0,target[i]);
      else{prefs.remove(k.c_str());refTemp[i]=NAN;}
      if(isfinite(notice[i]))storeTempReference(i,1,notice[i]);
      else{prefs.remove(j.c_str());noticeTemp[i]=NAN;}
    }
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8","Справочные значения сохранены. Реле не затронуты.");
  });
  server.on("/api/graph-view",HTTP_GET,[]{
    int n=server.arg("view").toInt();
    if(n<0||n>2){server.send(400,"text/plain","Bad graph");return;}
    graphSubView=(uint8_t)n;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/settings-tab",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<0||n>3){server.send(400,"text/plain","bad subtab");return;}
    settingsSubTab=(uint8_t)n;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/tft/scroll",HTTP_GET,[]{
    String dir=server.arg("dir");
    if(dir!="up"&&dir!="down"){server.send(400,"text/plain","bad dir");return;}
    tftView=(dir=="down")?1:0;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  const char *headers[]={"X-Busa-Token"};
  server.collectHeaders(headers,1);
  server.begin();
  Serial0.println("WEB started; temperature scan & I2C next");
  i2cOnline=Wire.begin(BUS_SDA,BUS_SCL,100000);
  if(i2cOnline){Wire.setTimeOut(25);probeI2c(millis());}
  updateBootStage(2);
  Serial0.printf("I2C=%d PCF=%d PCA=%d\n",(int)i2cOnline,(int)pcfDetected,(int)pcaDetected);
  updateBootStage(3);
  launchGoogleCloud();
  Serial0.println("SETUP DONE: relay GPIO4/5/6/7 NEVER configured/driven");
}
void loop(){
  uint32_t now=millis();
  relaySimTick(now);
  ledSimTick(now);
  if(lastLoopStart){const uint32_t gap=now-lastLoopStart;if(gap>loopMaxGapMs)loopMaxGapMs=gap;}
  lastLoopStart=now;
  wifiRecoveryTick(now);
  const uint32_t webStart=millis();
  server.handleClient();
  const uint32_t webElapsed=millis()-webStart;
  if(webElapsed>webMaxServiceMs)webMaxServiceMs=webElapsed;
  readConsole();
  if(otaRestart && elapsed(now,otaRestartAt,4000))ESP.restart();
  // Avoid I2C/1-Wire/SPI scanning while Wi-Fi OTA flash chunks arrive.
  // Flash throughput remains hardware/network limited.
  if(otaActive){relaySimStop("OTA_TRANSFER");ledSimStop("OTA_TRANSFER");delay(0);return;}
  if(tftRebootPending && elapsed(now,tftRebootAt,3000))ESP.restart();
  if(googleRestart && elapsed(now,googleRestartAt,3500))ESP.restart();
  publishGoogleSnapshot(now);
  temperatureTick(millis()); // One bounded temperature bus operation per iteration
  // A missing sensor is a diagnostic error, not a reason to trap the UI on boot.
  // Wi-Fi AP and HTTP server initialize during the splash.
  // Missing sensors must not delay the user interface.
  if(!bootSplashComplete && elapsed(now,splashStartedAt,8000)){
    bootSplashComplete=true;
    tftDirty=true;
  }
    sampleFlow(now);
  if(i2cOnline){
    if(elapsed(now,lastBusProbe,6000))probeI2c(now);
    pollKeys(now);
  }
  // Landscape V14.6: rebuild background ONLY on explicit view/theme change.
  // A 2.5 s telemetry tick refreshes changed text without wiping the TFT.
  if(tftStarted && !tftDisplayOff && !otaActive && bootSplashComplete &&
     elapsed(now,splashStartedAt,splashHoldMs) &&
     (tftDirty||lastRender==0 || elapsed(now,lastRender,2500))){
    const bool wholeFrame=tftDirty||lastRender==0;
    lastRender=now;
    if(screen->width()>=300 && !wholeFrame)v14RefreshLive(now);
    else renderScreen(now);
  }
  if(elapsed(now,lastUart,5000)){
    lastUart=now;
    Serial0.printf("N8R2 HEARTBEAT ms=%lu wifi=%d i2c=%d tft=%d p=%lu flow=%.3f\n",
      (unsigned long)now,(int)wifiRunning,(int)i2cOnline,(int)tftStarted,
      (unsigned long)flowPulses(),(double)flowLpm);
  }
  delay(2);
}


