#pragma once
/*
  BUSA-1M-001 БУСА 1М WiFi 3v-GOOGLE
  ESP32-S3 N16R8, TFT GMT028-05 V1.1 320x240 landscape, monitoring-only.
  LOW-VOLTAGE MONITORING + four relay channels for 12V SOUND ONLY. No motor or mains output.
  Approved hardware scope: mains power controller and motorized valve are
  REMOVED from this project; they are operated manually, outside ESP32.
  Approved informational section "Отбор" is retained without actuator control.
  Real 4 x DS18B20 + YF-S401 cooling-water pulse readings, no simulated values.
  Hardware compatibility of TFT controller, PCF8575 keypad and pulse output 
  must be verified on a powered-off low-voltage bench before connecting.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <WebServer.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <sys/time.h>
#include <esp_system.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_ILI9341.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <math.h>

constexpr uint8_t TPIN[4]={16,17,18,21};
constexpr uint8_t KPIN[4]={7,6,5,4}; // Four 12 V sound-annunciator relay driver inputs ONLY
constexpr uint8_t BUS_SDA=40,BUS_SCL=41; // Isolated PCF8575 keypad only; GPIO8/9 disconnected
constexpr uint8_t TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14;
constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V
constexpr uint8_t BUSA_RESERVED_BUZZER_GPIO=2; // active 3.3V logic buzzer input, T1/T2
constexpr bool RELAY_PHYSICAL_OUTPUTS=true,LED_PHYSICAL_OUTPUTS=false;
static_assert(RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,"K1-K4 sound-only relays; no PWM equipment outputs");
static_assert(BUSA_RESERVED_BUZZER_GPIO!=FLOW_GPIO && BUSA_RESERVED_BUZZER_GPIO!=BUS_SDA &&
  BUSA_RESERVED_BUZZER_GPIO!=BUS_SCL && BUSA_RESERVED_BUZZER_GPIO!=TFT_CS &&
  BUSA_RESERVED_BUZZER_GPIO!=TFT_DC && BUSA_RESERVED_BUZZER_GPIO!=TFT_RST &&
  BUSA_RESERVED_BUZZER_GPIO!=TFT_MOSI && BUSA_RESERVED_BUZZER_GPIO!=TFT_SCK,
  "Buzzer pin conflicts with I2C, TFT or flow pins");
static_assert(FLOW_GPIO==15 && TPIN[0]==16 && TPIN[1]==17 && TPIN[2]==18 && TPIN[3]==21 && BUS_SDA==40 && BUS_SCL==41,"GPIO changed");
static_assert(TFT_CS==10 && TFT_DC==11 && TFT_RST==12 && TFT_MOSI==13 && TFT_SCK==14,"TFT GPIO changed");
const char VERSION[]="БУСА 1М WiFi 15v0-N16R8-WIFI-GOOGLE-TFT-NVS-20261103-APFIX-R1";
// Diagnostic logging targets both the CH343 UART0 and native USB CDC (COM8).
uint32_t busaBootAt=0,busaSetupDoneMs=0,busaSlowLoops=0,busaMaxLoopMs=0;
uint32_t busaMaxKeyScanMs=0,busaLastKeyScanMs=0,busaKeySkipUntil=0;
uint32_t busaLastI2cStepMs=0,busaMaxI2cStepMs=0;
uint32_t busaI2cSuspendedUntil=0,busaI2cErrorCount=0,busaI2cStallCount=0;
uint32_t busaTempWorstMs=0,busaOneWireWorstMs=0,busaTftWorstMs=0;
uint32_t busaI2cWorstMs=0,busaHttpWorstMs=0;
constexpr uint32_t BUSA_TFT_LIVE_REFRESH_MS=200; // Responsive changed-field refresh, not a full redraw
constexpr uint32_t BUSA_KEYPAD_SLICE_MS=16; // four matrix rows per slice, max ~64 ms per sweep
uint32_t busaLastTftRenderMs=0,busaTftFullRenderCount=0,busaTftLiveRenderCount=0;

// Generic ESP32-S3 performance telemetry; no change to sensor/control functionality.
uint32_t busaCpuMhz=0,busaPsramBytes=0,busaPsramFreeBytes=0;
bool busaCpu240Ok=false,busaPsramDetected=false;

const char* busaSlowComponent="NONE";
void busaObserve(const char* component,uint32_t started,uint32_t &worst){
  const uint32_t dt=millis()-started;
  if(dt>worst)worst=dt;
  if(dt>=100){busaSlowComponent=component;Serial.printf("BUSA SLOW %s %lu ms\n",component,(unsigned long)dt);}
}

const char* busaBootStage="RESET";
void busaReportStage(const char* stage){
  busaBootStage=stage;
  const uint32_t age=millis()-busaBootAt;
  Serial.printf("BUSA STAGE %s ms=%lu\n",stage,(unsigned long)age);
  Serial0.printf("BUSA STAGE %s ms=%lu\n",stage,(unsigned long)age);
}


OneWire ow1(TPIN[0]),ow2(TPIN[1]),ow3(TPIN[2]),ow4(TPIN[3]);
DallasTemperature ds1(&ow1),ds2(&ow2),ds3(&ow3),ds4(&ow4);
DallasTemperature* sensors[4]={&ds1,&ds2,&ds3,&ds4};
float temperature[4]={NAN,NAN,NAN,NAN};
float tempOffset[4]={0,0,0,0}; // Celsius sensor correction, separate from alarms
bool tempPresent[4]={false,false,false,false};
uint32_t tempAt[4]={0,0,0,0},tempCycle=0,tempRequestAt=0;
bool tempInitialized[4]={false,false,false,false};
DeviceAddress cachedTempRom[4]={};
bool cachedTempValid[4]={false,false,false,false};
uint32_t tempDiscoverAt[4]={0,0,0,0};
bool tempConverting=false;
// Read-only health checks; run only between conversion cycles.
OneWire* const temperatureBuses[4]={&ow1,&ow2,&ow3,&ow4};
struct OneWireProbe {
  bool presence=false,romFound=false,crcOk=false,familyOk=false,multiple=false;
  int8_t idleLevel=-1,afterResetLevel=-1; // GPIO state; not proof of sensor health
  uint8_t rom[8]={0};
  uint32_t checkedAt=0;
};
OneWireProbe wireProbe[4];
uint32_t lastWireProbe=0;
uint8_t wireProbeIndex=0;
bool elapsed(uint32_t now,uint32_t then,uint32_t ms){return (uint32_t)(now-then)>=ms;}
void sampleTemperatures(uint32_t now){
  if(tempConverting && elapsed(now,tempRequestAt,825)){
    for(int i=0;i<4;i++){
      float v=cachedTempValid[i]?sensors[i]->getTempC(cachedTempRom[i]):DEVICE_DISCONNECTED_C;
      if(tempPresent[i] && isfinite(v) && v>=-55 && v<=125 && v!=DEVICE_DISCONNECTED_C){
        temperature[i]=v+tempOffset[i];tempAt[i]=now;
      }else{temperature[i]=NAN;tempAt[i]=0;}
    }
    tempConverting=false;
  }
  if(!tempConverting && (tempCycle==0||elapsed(now,tempCycle,2500))){
    tempCycle=now;
    for(int i=0;i<4;i++){
      // Do not rediscover four OneWire buses every 2.5 seconds.
      // A disconnected bus is retried every 12 seconds.
      if(!tempInitialized[i] || (!tempPresent[i] && elapsed(now,tempDiscoverAt[i],12000))){
        sensors[i]->begin();
        tempInitialized[i]=true;
        tempDiscoverAt[i]=now;
        tempPresent[i]=sensors[i]->getAddress(cachedTempRom[i],0);
        cachedTempValid[i]=tempPresent[i];
        if(tempPresent[i]){
          sensors[i]->setResolution(cachedTempRom[i],12);
          sensors[i]->setWaitForConversion(false);
        }
      }
      if(tempPresent[i])sensors[i]->requestTemperatures();
      else{temperature[i]=NAN;tempAt[i]=0;}
    }
    tempConverting=true;tempRequestAt=now;
  }
  for(int i=0;i<4;i++)if(tempAt[i] && elapsed(now,tempAt[i],7000)){
    temperature[i]=NAN;tempAt[i]=0;
  }
}
void probeOneWire(uint32_t now){
  if(tempConverting||(lastWireProbe && !elapsed(now,lastWireProbe,3000)))return;
  lastWireProbe=now;
  // Spread CRC and discovery probes across the four sensors; not all at once.
  const int i=wireProbeIndex++%4;
    OneWireProbe &p=wireProbe[i];
    p=OneWireProbe();
    p.checkedAt=now;
    OneWire* bus=temperatureBuses[i];
    p.idleLevel=digitalRead(TPIN[i]);
    p.presence=bus->reset()!=0;
    p.afterResetLevel=digitalRead(TPIN[i]);
    bus->reset_search();
    uint8_t first[8]={0},second[8]={0};
    p.romFound=bus->search(first);
    if(p.romFound){
      memcpy(p.rom,first,8);
      p.crcOk=OneWire::crc8(first,7)==first[7];
      p.familyOk=first[0]==0x28; // DS18B20 family
      p.multiple=bus->search(second);
    }
    bus->reset_search();
}
String romHex(const uint8_t rom[8]){
  char hex[17];for(int j=0;j<8;j++)snprintf(hex+2*j,3,"%02X",(unsigned)rom[j]);
  return String(hex);
}
bool hasTemp(int n,uint32_t now);
String wireError(int i,uint32_t now){
  const OneWireProbe& p=wireProbe[i];
  if(!p.checkedAt)return "WAIT_DIAG";
  if(!p.presence && p.idleLevel==0)return "LINE_LOW";
  if(!p.presence)return "NO_PRESENCE";
  if(!p.romFound)return "NO_ROM";
  if(!p.crcOk)return "ROM_CRC_BAD";
  if(!p.familyOk)return "NOT_DS18B20";
  if(p.multiple)return "MULTIPLE_ROM";
  if(!tempPresent[i])return "LIBRARY_NOT_DETECTED";
  if(!hasTemp(i,now))return "NO_VALID_READING";
  return "OK";
}
bool hasTemp(int n,uint32_t now){
  return tempAt[n] && !elapsed(now,tempAt[n],7000) && isfinite(temperature[n]);
}

// YF-S401 pulse reader for ordinary cooling WATER only. No process control.
// 98Hz per litre/minute is a provisional example, NOT factory-calibrated for
// this individual sensor. Range and pulse format depend on sensor revision.
// Never interpret 0 pulses as proof of cooling water or as a safety interlock.
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
constexpr float FLOW_HZ_PER_LPM=98.0f; // must be calibrated by water-volume test
uint32_t flowCycle=0,flowLastCount=0;
float flowLpm=0.0f,flowTrace[40]={};
uint32_t waterPulseHistory[40]={}; // raw pulses only, not calibrated
uint32_t waterPulseDelta=0;
bool flowReady=false,flowSeen=false;
uint8_t flowTraceHead=0,waterSamplesRecorded=0;
void sampleFlow(uint32_t now){
  if(!flowCycle){flowCycle=now;return;}
  uint32_t dt=now-flowCycle;
  if(dt<2000)return;
  uint32_t count=flowPulses();
  uint32_t diff=count-flowLastCount;
  flowLastCount=count;flowCycle=now;
  flowLpm=(float)diff*60000.0f/(dt*FLOW_HZ_PER_LPM);
  if(diff)flowSeen=true;
  flowReady=true;
  waterPulseDelta=diff;
  waterPulseHistory[flowTraceHead]=diff;
  flowTrace[flowTraceHead]=flowLpm;
  flowTraceHead=(flowTraceHead+1)%40;
  if(waterSamplesRecorded<40)waterSamplesRecorded++;
}
float totalWaterLitres(){return (float)flowPulses()/(FLOW_HZ_PER_LPM*60.0f);}

// Isolated 3.3V I2C keypad PCF8575/PCF8575A only; PCA9685 LED board detached.
bool i2cOnline=false,pcfDetected=false,pcaDetected=false;
uint8_t pcfAddr=0,pcaAddr=0x40;uint32_t lastBusProbe=0;
uint8_t i2cFound[24]={0},i2cFoundCount=0;
uint16_t i2cAckCount=0;
uint32_t i2cFullScanAt=0;
// A probe is a sample, not a state change. Keep each device's most recent
// confirmed ACK, but report "online" only while confirmation is fresh.
uint32_t busaI2cLastPcfAckMs=0,busaI2cLastPcaAckMs=0;
uint32_t busaI2cPcfAcks=0,busaI2cPcaAcks=0;
uint32_t busaI2cLastPcfNackMs=0,busaI2cLastPcaNackMs=0;
constexpr uint32_t BUSA_I2C_ACK_FRESH_MS=30000;
// GPIO-only, open-drain I2C. ESP32-S3 hardware controller previously blocked for
// 1000ms even with Wire.setTimeOut(25). This avoids the controller entirely.
// Every SCL-high wait is bounded, with no long global bus quarantine.
// ~18-25 kHz software I2C: allow slow pull-ups, long harnesses, and CPU preemption.
constexpr uint32_t BUSA_SOFTI2C_HALF_PERIOD_US=20;
constexpr uint32_t BUSA_SOFTI2C_SCL_WAIT_US=800;
uint32_t busaI2cNackCount=0,busaI2cTimeoutCount=0,busaI2cRecoverCount=0;
int busaI2cLastResult=0; // 0 OK, 1 NACK, 2 SCL held low, 3 SDA held low, 4 no bus
uint8_t busaI2cLastAddress=0,busaI2cLastSda=1,busaI2cLastScl=1;
// Read-back at the ESP32 pads; not a substitute for sampling at the modules.
uint32_t busaI2cLastGpioTestMs=0,busaI2cGpioTests=0;
uint8_t busaI2cTestIdleSda=0,busaI2cTestIdleScl=0;
uint8_t busaI2cTestSdaForcedLow=1,busaI2cTestSclForcedLow=1;
uint8_t busaI2cTestSdaReleased=0,busaI2cTestSclReleased=0;
uint8_t busaI2cTestSdaWhileSclLow=0,busaI2cTestSclWhileSdaLow=0;
uint32_t busaI2cSdaOutputFaults=0,busaI2cAckUnstableCount=0;
uint32_t busaI2cClockStretchCount=0,busaI2cLowDuringHighBits=0;
uint8_t busaI2cAckSampleEarly=1,busaI2cAckSampleLate=1;
uint8_t busaI2cLastTxByte=0;
bool busaI2cGpioTestPassed=false;
bool busaI2cElectricalFault=true;
uint32_t busaI2cDisabledScanCount=0;
uint32_t busaI2cGpioConfigErrors=0;
uint32_t busaI2cPadReinitCount=0,busaI2cLastPadReinitMs=0;
uint8_t busaI2cSdaIdfLow=1,busaI2cSclIdfLow=1;
bool busaI2cSuspended(uint32_t now){return busaI2cSuspendedUntil && (int32_t)(now-busaI2cSuspendedUntil)<0;}
void busaI2cQuarantine(uint32_t now,uint32_t durationMs=500){
  // Do not erase positive device discoveries on a single bad sample.
  busaI2cSuspendedUntil=now+durationMs;
  ++busaI2cErrorCount;
}
bool busaI2cLinesFree(){
  busaI2cLastSda=(uint8_t)digitalRead(BUS_SDA);
  busaI2cLastScl=(uint8_t)digitalRead(BUS_SCL);
  return busaI2cLastSda==HIGH && busaI2cLastScl==HIGH;
}
// GPIO_MODE_INPUT_OUTPUT_OD enables the ESP32-S3 pad input buffer; Arduino's
// output-only open drain can make digitalRead() report HIGH even while
// the pad is commanded LOW. IDF writes 1 = release, 0 = sink only.
inline void busaSda(bool release){
  if(gpio_set_level(static_cast<gpio_num_t>(BUS_SDA),release?1:0)!=ESP_OK)++busaI2cGpioConfigErrors;
}
inline void busaScl(bool release){
  if(gpio_set_level(static_cast<gpio_num_t>(BUS_SCL),release?1:0)!=ESP_OK)++busaI2cGpioConfigErrors;
}
// Reconfigure SDA/SCL only when needed. Never drive an I2C line HIGH.
void busaI2cConfigurePads(){
  const gpio_num_t sda=static_cast<gpio_num_t>(BUS_SDA);
  const gpio_num_t scl=static_cast<gpio_num_t>(BUS_SCL);
  gpio_hold_dis(sda);gpio_hold_dis(scl);
  if(gpio_reset_pin(sda)!=ESP_OK)++busaI2cGpioConfigErrors;
  if(gpio_reset_pin(scl)!=ESP_OK)++busaI2cGpioConfigErrors;
  gpio_config_t i2cPinCfg={};
  i2cPinCfg.pin_bit_mask=(1ULL<<BUS_SDA)|(1ULL<<BUS_SCL);
  i2cPinCfg.mode=GPIO_MODE_INPUT_OUTPUT_OD;
  i2cPinCfg.pull_up_en=GPIO_PULLUP_ENABLE;
  i2cPinCfg.pull_down_en=GPIO_PULLDOWN_DISABLE;
  i2cPinCfg.intr_type=GPIO_INTR_DISABLE;
  if(gpio_config(&i2cPinCfg)!=ESP_OK)++busaI2cGpioConfigErrors;
  busaSda(true);busaScl(true);
  delayMicroseconds(300);
  busaI2cLastPadReinitMs=millis();
  ++busaI2cPadReinitCount;
}
inline void busaI2cPause(){delayMicroseconds(BUSA_SOFTI2C_HALF_PERIOD_US);}
void busaI2cRunGpioSelfTest(){
  busaSda(true);busaScl(true);delayMicroseconds(200);
  busaI2cTestIdleSda=digitalRead(BUS_SDA);
  busaI2cTestIdleScl=digitalRead(BUS_SCL);
  // Check clock-low drive, then release it before the brief SDA START/STOP.
  busaScl(false);delayMicroseconds(30);
  busaI2cTestSclForcedLow=gpio_get_level(static_cast<gpio_num_t>(BUS_SCL));
  busaI2cSclIdfLow=busaI2cTestSclForcedLow;
  busaI2cTestSdaWhileSclLow=gpio_get_level(static_cast<gpio_num_t>(BUS_SDA));
  busaScl(true);delayMicroseconds(200);
  busaSda(false);delayMicroseconds(30);
  busaI2cTestSdaForcedLow=gpio_get_level(static_cast<gpio_num_t>(BUS_SDA));
  busaI2cSdaIdfLow=busaI2cTestSdaForcedLow;
  busaI2cTestSclWhileSdaLow=gpio_get_level(static_cast<gpio_num_t>(BUS_SCL));
  busaSda(true);delayMicroseconds(200);
  busaI2cTestSdaReleased=digitalRead(BUS_SDA);
  busaI2cTestSclReleased=digitalRead(BUS_SCL);
  busaI2cGpioTestPassed=busaI2cTestIdleSda && busaI2cTestIdleScl &&
    !busaI2cTestSdaForcedLow && !busaI2cTestSclForcedLow &&
    busaI2cTestSdaReleased && busaI2cTestSclReleased &&
    busaI2cTestSdaWhileSclLow && busaI2cTestSclWhileSdaLow;
  busaI2cElectricalFault=!busaI2cGpioTestPassed;
  ++busaI2cGpioTests;busaI2cLastGpioTestMs=millis();
}
bool busaI2cClockHigh(){
  busaScl(true);
  const uint32_t started=micros();
  bool stretched=false;
  while(digitalRead(BUS_SCL)==LOW){
    stretched=true;
    if((uint32_t)(micros()-started)>=BUSA_SOFTI2C_SCL_WAIT_US){
      busaI2cLastResult=2;return false;
    }
  }
  if(stretched)++busaI2cClockStretchCount;
  busaI2cPause();
  return true;
}
bool busaI2cStart(){
  busaSda(true);busaI2cPause();
  if(!busaI2cClockHigh())return false;
  if(digitalRead(BUS_SDA)==LOW){busaI2cLastResult=3;return false;}
  busaSda(false);busaI2cPause();
  if(digitalRead(BUS_SDA)!=LOW)++busaI2cSdaOutputFaults;
  busaScl(false);busaI2cPause();
  return true;
}
bool busaI2cRestart(){
  busaSda(true);busaI2cPause();
  if(!busaI2cClockHigh())return false;
  busaSda(false);busaI2cPause();busaScl(false);busaI2cPause();
  return true;
}
void busaI2cStop(){
  busaScl(false);busaSda(false);busaI2cPause();
  if(busaI2cClockHigh()){busaSda(true);busaI2cPause();}
  // Ensure both lines are released, even if clock stretching timed out.
  busaSda(true);busaScl(true);
}
bool busaI2cWriteByte(uint8_t value){
  busaI2cLastTxByte=value;
  for(uint8_t mask=0x80;mask;mask>>=1){
    const bool high=(value&mask)!=0;
    busaSda(high);busaI2cPause();
    if(!busaI2cClockHigh())return false;
    const bool padHigh=digitalRead(BUS_SDA)==HIGH;
    if(high && !padHigh)++busaI2cLowDuringHighBits;
    if(!high && padHigh)++busaI2cSdaOutputFaults;
    busaScl(false);busaI2cPause();
  }
  busaSda(true);busaI2cPause();
  if(!busaI2cClockHigh())return false;
  busaI2cAckSampleEarly=(uint8_t)digitalRead(BUS_SDA);
  delayMicroseconds(5);
  busaI2cAckSampleLate=(uint8_t)digitalRead(BUS_SDA);
  if(busaI2cAckSampleEarly!=busaI2cAckSampleLate)++busaI2cAckUnstableCount;
  const bool ack=busaI2cAckSampleEarly==LOW && busaI2cAckSampleLate==LOW;
  busaScl(false);busaI2cPause();
  if(!ack)busaI2cLastResult=1;
  return ack;
}
bool busaI2cReadByte(uint8_t &out,bool acknowledge){
  out=0;busaSda(true);
  for(int i=0;i<8;i++){
    busaI2cPause();
    if(!busaI2cClockHigh())return false;
    out=(uint8_t)((out<<1)|(digitalRead(BUS_SDA)==HIGH?1:0));
    busaScl(false);busaI2cPause();
  }
  busaSda(!acknowledge);busaI2cPause();
  if(!busaI2cClockHigh())return false;
  busaScl(false);busaI2cPause();busaSda(true);
  return true;
}
void busaI2cRecover(){
  // Clock nine times to release a slave stuck part-way through a byte.
  ++busaI2cRecoverCount;
  busaSda(true);
  for(int i=0;i<9;i++){
    busaScl(false);busaI2cPause();
    if(!busaI2cClockHigh())break;
  }
  busaI2cStop();
}
bool busaI2cExchange(uint8_t address,const uint8_t* tx,size_t txLen,uint8_t* rx,size_t rxLen){
  const uint32_t started=millis();
  if(!i2cOnline || busaI2cSuspended(started)){busaI2cLastResult=4;return false;}
  if(busaI2cElectricalFault){busaI2cLastResult=5;return false;}
  busaI2cLastAddress=address;busaI2cLastResult=0;
  bool ok=busaI2cStart();
  if(ok)ok=busaI2cWriteByte((uint8_t)(address<<1)|(txLen||!rxLen?0:1));
  for(size_t i=0;ok && i<txLen;i++)ok=busaI2cWriteByte(tx[i]);
  if(ok && rxLen && txLen){
    ok=busaI2cRestart();
    if(ok)ok=busaI2cWriteByte((uint8_t)(address<<1)|1);
  }
  for(size_t i=0;ok && i<rxLen;i++)ok=busaI2cReadByte(rx[i],i+1<rxLen);
  busaI2cStop();
  const uint32_t ms=millis()-started;
  busaLastI2cStepMs=ms;if(ms>busaMaxI2cStepMs)busaMaxI2cStepMs=ms;
  if(!ok && (busaI2cLastResult==2||busaI2cLastResult==3)){
    ++busaI2cTimeoutCount;++busaI2cStallCount;
    busaI2cQuarantine(millis());
    // Recover only after a genuine held bus, never after a normal NACK.
    busaI2cRecover();
  }else if(!ok && busaI2cLastResult==1){
    ++busaI2cNackCount;
  }
  if(ms>100){
    ++busaI2cStallCount;busaSlowComponent="SOFTI2C";
  }
  return ok;
}

int pcaMode1=-1,pcaMode2=-1;
bool pcfIdleReadOk=false;
uint16_t pcfIdleDirect=0xffff;
uint32_t keyCount=0,lastKeyScan=0,pendingAt=0;
int pendingPair=-1;bool keyHeld=false;
String keyMessage="NO DATA";
uint16_t pcfIdleMask=65535,pcfObservedLow=0;
uint8_t pcfPairCount=0,pcfScanError=0;
uint32_t pcfLastScanAt=0;
Preferences prefs;
Preferences keypadBackupPrefs;
bool keypadBackupOpen=false;
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
  return busaI2cExchange(address,nullptr,0,nullptr,0);
}
// Only scan the PCF8575 (0x20..0x27) or PCF8575A (0x38..0x3F) address sets.
// LED driver PCA9685 is disconnected and must not be addressed.
constexpr uint8_t BUSA_I2C_SCAN_START=0x20,BUSA_I2C_SCAN_END=0x3F;
constexpr uint8_t BUSA_I2C_PRIORITY_ADDRS[]={0x20,0x38};
uint8_t busaI2cScanNext=0x20,busaI2cPriorityNext=0;
uint32_t busaI2cScanLastStep=0;
uint32_t busaI2cAddressLastSeen[128]={};
uint16_t busaI2cAddressAckTotal[128]={};
bool busaI2cIsPcfAddress(uint8_t a){
  return (a>=0x20 && a<=0x27)||(a>=0x38 && a<=0x3F);
}
void busaI2cRebuildAddressCache(uint32_t now){
  i2cFoundCount=0;
  for(uint8_t a=BUSA_I2C_SCAN_START;a<=BUSA_I2C_SCAN_END;a++){
    if(busaI2cIsPcfAddress(a) && busaI2cAddressLastSeen[a] &&
       (uint32_t)(now-busaI2cAddressLastSeen[a])<=BUSA_I2C_ACK_FRESH_MS &&
       i2cFoundCount<sizeof(i2cFound)){
      i2cFound[i2cFoundCount++]=a;
    }
  }
  i2cAckCount=i2cFoundCount;
}
void scanI2cAddresses(uint32_t now){
  if(!i2cOnline || busaI2cSuspended(now))return;
  if(busaI2cElectricalFault){++busaI2cDisabledScanCount;return;}
  if(!elapsed(now,busaI2cScanLastStep,120))return;
  busaI2cScanLastStep=now;
  uint8_t address;
  if(busaI2cPriorityNext<sizeof(BUSA_I2C_PRIORITY_ADDRS)){
    address=BUSA_I2C_PRIORITY_ADDRS[busaI2cPriorityNext++];
  }else{
    if(busaI2cScanNext==0x28)busaI2cScanNext=0x38;
    address=busaI2cScanNext++;
    if(busaI2cScanNext>BUSA_I2C_SCAN_END){
      busaI2cScanNext=BUSA_I2C_SCAN_START;
      i2cFullScanAt=millis();
    }
  }
  const bool found=ack(address);
  if(found){
    busaI2cAddressLastSeen[address]=millis();
    if(busaI2cAddressAckTotal[address]<UINT16_MAX)++busaI2cAddressAckTotal[address];
    if(busaI2cIsPcfAddress(address)){
      busaI2cLastPcfAckMs=millis();++busaI2cPcfAcks;
    }
  }else if(busaI2cIsPcfAddress(address)){
    busaI2cLastPcfNackMs=millis();
  }
  busaI2cRebuildAddressCache(millis());
}
void probeI2c(uint32_t now){
  if(!i2cOnline || busaI2cSuspended(now))return;
  busaI2cRebuildAddressCache(millis());
  pcfDetected=false;pcfAddr=0;pcaDetected=false;pcaAddr=0;
  // Only PCF8575 keypad is installed; no LED driver discovery or reads.
  for(uint8_t j=0;j<i2cFoundCount;j++){
    const uint8_t address=i2cFound[j];
    if(busaI2cIsPcfAddress(address) &&
       (!pcfDetected || address==0x20)){pcfDetected=true;pcfAddr=address;}
  }
  pcaMode1=-1;pcaMode2=-1;
  pcfIdleReadOk=false;
  if(pcfDetected){
    uint8_t allHigh[2]={0xff,0xff},received[2]={};
    if(busaI2cExchange(pcfAddr,allHigh,2,nullptr,0) &&
       busaI2cExchange(pcfAddr,nullptr,0,received,2)){
      pcfIdleDirect=(uint16_t)received[0]|((uint16_t)received[1]<<8);
      pcfIdleReadOk=true;
    }
  }
  lastBusProbe=now;
}
bool pcfWrite(uint16_t value){
  if(!pcfDetected || busaI2cSuspended(millis()))return false;
  uint8_t bytes[2]={(uint8_t)value,(uint8_t)(value>>8)};
  return busaI2cExchange(pcfAddr,bytes,2,nullptr,0);
}
bool pcfRead(uint16_t &value){
  if(!pcfDetected || busaI2cSuspended(millis()))return false;
  uint8_t bytes[2]={};
  if(!busaI2cExchange(pcfAddr,nullptr,0,bytes,2))return false;
  value=(uint16_t)bytes[0]|((uint16_t)bytes[1]<<8);
  return true;
}
uint8_t currentPage=0; // 7 tabs including Diagnostics
uint8_t graphSubView=0; // 0 four isolated temperature lanes; 1 water + collection lanes
uint32_t busaGraphLastDrawnCapture=0;
uint8_t approvedSettingsItem=0; // 5 approved settings
uint8_t settingsSubTab=0,tftView=0,previousMainPage=0;
uint8_t diagModule=0,calSensor=0,themeDraft=2,tftThemeMode=2;
uint8_t googleUiChoice=0; // Physical Google page: 0 Connect, 1 Disconnect
bool busaGoogleUiToggle(bool enable); // implemented in BUSA_Google.h
bool busaGoogleSavedCredentials(); // never expose credentials to the TFT
bool calEditing=false;float calDraft=0;
bool tftDirty=true;
float refTemp[4]={NAN,NAN,NAN,NAN},noticeTemp[4]={NAN,NAN,NAN,NAN};
// TFT temperature setpoints mirror the actual active sound threshold settings.
int selectedTemp=0,selectedField=0;
bool editingTemp=false;
bool deleteConfirmPending=false; // Explicit confirmation for sound threshold removal
String editTempBuffer="";
uint8_t editCursor=0;
bool storeTempReference(int sensor,int field,float value); // writes real sound setpoint

// Keyboard faceplate layout confirmed by photo. Wiring is learned, not guessed.
const char* const KEY_CAPS[20]={"F1","F2","#","*","1","2","3","UP",
 "4","5","6","DOWN","7","8","9","ESC","LEFT","0","RIGHT","ENT"};
uint8_t keyPairs[20],newPairs[20];
uint8_t learnStep=0;
bool keypadReady=false,keypadLearning=false;
String keypadMessage="Обучение не выполнено",rawPair="—";
String keypadStorageStatus="NOT_LOADED"; // NVS health only; never expose the physical keymap
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
// The primary "busa-r04/keys20" is fully compatible with the approved training.
 // A second NVS namespace protects against a damaged/deleted individual record.
 // A full-chip erase still destroys BOTH namespaces: USB updates must preserve NVS.
bool readValidKeymap(Preferences &store,const char *key,uint8_t *dest){
  if(store.getBytesLength(key)!=sizeof(keyPairs))return false;
  if(store.getBytes(key,dest,sizeof(keyPairs))!=sizeof(keyPairs))return false;
  return validKeymap(dest);
}
bool writeVerifiedKeymap(Preferences &store,const char *key,const uint8_t *src){
  if(store.putBytes(key,src,sizeof(keyPairs))!=sizeof(keyPairs))return false;
  uint8_t check[20]={0};
  return readValidKeymap(store,key,check) && memcmp(check,src,sizeof(check))==0;
}
void loadKeymap(){
  memset(keyPairs,255,sizeof(keyPairs));
  memset(newPairs,255,sizeof(newPairs));
  uint8_t backup[20]={0};
  const bool primaryOk=readValidKeymap(prefs,"keys20",keyPairs);
  const bool backupOk=keypadBackupOpen && readValidKeymap(keypadBackupPrefs,"keys20",backup);
  if(primaryOk){
    keypadReady=true;
    if(backupOk && memcmp(keyPairs,backup,sizeof(backup))==0){
      keypadStorageStatus="PRIMARY_AND_BACKUP_OK";
    }else if(keypadBackupOpen && writeVerifiedKeymap(keypadBackupPrefs,"keys20",keyPairs)){
      keypadStorageStatus="PRIMARY_BACKUP_REPAIRED";
    }else{
      keypadStorageStatus="PRIMARY_BACKUP_UNAVAILABLE";
    }
    keypadMessage="Раскладка загружена";
  }else if(backupOk){
    memcpy(keyPairs,backup,sizeof(keyPairs));
    keypadReady=true; // Always allow keys to work from a valid backup.
    keypadStorageStatus=writeVerifiedKeymap(prefs,"keys20",keyPairs)?
      "RESTORED_FROM_BACKUP":"BACKUP_ACTIVE_PRIMARY_WRITE_FAILED";
    keypadMessage="Раскладка восстановлена";
  }else{
    memset(keyPairs,255,sizeof(keyPairs));
    keypadReady=false;
    keypadStorageStatus="NO_VALID_KEYMAP";
    keypadMessage="Сохранённая раскладка не найдена: необходимо обучить 20 клавиш";
  }
  Serial0.printf("BUSA KEYPAD NVS: %s primary_len=%u backup_len=%u ready=%d\\n",
    keypadStorageStatus.c_str(),(unsigned)prefs.getBytesLength("keys20"),
    (unsigned)(keypadBackupOpen?keypadBackupPrefs.getBytesLength("keys20"):0),
    (int)keypadReady);
}
// Detect a single closed physical pair on any two PCF8575 P0..P15.
// Scan the 16 PCF8575 rows in four short transactions per pass, not all 16
// during a single UI frame. A partial scan returns -3 until all rows are tested.
uint8_t busaKeyScanRow=0;
int busaKeyScanFound=-1;
bool busaKeyScanAmbiguous=false;
void busaKeyScanReset(){busaKeyScanRow=0;busaKeyScanFound=-1;busaKeyScanAmbiguous=false;}
int scanKeyPair(){
  if(!pcfDetected){pcfScanError=1;busaKeyScanReset();return -2;}
  if(busaKeyScanRow==0){
    pcfLastScanAt=millis();
    pcfScanError=0;pcfPairCount=0;pcfObservedLow=0;
    busaKeyScanFound=-1;busaKeyScanAmbiguous=false;
    if(!pcfWrite(65535)){pcfScanError=2;return -2;}
    delayMicroseconds(100);
    if(!pcfRead(pcfIdleMask)){pcfScanError=3;return -2;}
  }
  for(uint8_t step=0;step<4 && busaKeyScanRow<16;step++){
    const int a=busaKeyScanRow++;
    if(!pcfWrite((uint16_t)(65535u & ~(1u<<a)))){
      pcfScanError=4;busaKeyScanAmbiguous=true;break;
    }
    delayMicroseconds(100);
    uint16_t v=0;
    if(!pcfRead(v)){pcfScanError=5;busaKeyScanAmbiguous=true;break;}
    const uint16_t lows=(uint16_t)(~v) & (uint16_t)~(1u<<a);
    pcfObservedLow|=lows;
    if(!lows)continue;
    if(lows&(lows-1)){pcfScanError=6;busaKeyScanAmbiguous=true;break;}
    int b=0;while(b<16 && !(lows&(1u<<b)))b++;
    if(b>=16)continue;
    const int pair=(min(a,b)<<4)|max(a,b);
    if(busaKeyScanFound>=0 && busaKeyScanFound!=pair){
      pcfScanError=7;busaKeyScanAmbiguous=true;break;
    }
    if(busaKeyScanFound<0)pcfPairCount=1;
    busaKeyScanFound=pair;
  }
  if(busaKeyScanAmbiguous || busaKeyScanRow==16){
    if(!pcfWrite(65535)){pcfScanError=8;busaKeyScanAmbiguous=true;}
    const int result=busaKeyScanAmbiguous?-2:busaKeyScanFound;
    busaKeyScanReset();
    return result;
  }
  // End the slice with all PCF pins high while HTTP/TFT/1-Wire run.
  if(!pcfWrite(65535)){pcfScanError=8;busaKeyScanReset();return -2;}
  return -3;
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
  currentPage=6;diagModule=1;tftView=1;tftDirty=true;
}

bool busaStageStaCredentials(const String& ssid,const String& pass,bool fromBoot=false);
void busaStaRestoreLocal();
#include "BUSA_KeypadTFT_UI.h"
void navigatePage(uint8_t target){
  if(target>=7||target==currentPage)return;
  if(currentPage==5 && target!=5 && wifiUiStage==WIFIUI_PASSWORD){
    wifiUiPassword="";wifiUiCursor=0;wifiUiResetTap();wifiUiStage=WIFIUI_STATUS;
  }
  previousMainPage=currentPage;
  currentPage=target;tftView=0;editingTemp=false;deleteConfirmPending=false;calEditing=false;tftDirty=true;
}
void deleteSelectedReference(){
  if(currentPage!=1 || selectedTemp<0 || selectedTemp>3)return;
  if(!storeTempReference(selectedTemp,selectedField,NAN))return;
  keyMessage=String("T")+String(selectedTemp+1)+
    (selectedField?" ВЫКЛ":" ВКЛ")+" - удалено";
}
void moveApprovedTab(int direction); // declaration required by keypad navigation
bool saveTempOffset(uint8_t sensor,float v){
 if(sensor>=4||!isfinite(v)||v< -10||v>10)return false;
 v=roundf(v*10)/10.0f;String key="tCal"+String(sensor);
 if(prefs.putFloat(key.c_str(),v)!=sizeof(float))return false;
 if(isfinite(temperature[sensor]))temperature[sensor]+=v-tempOffset[sensor];
 tempOffset[sensor]=v;tftDirty=true;return true;
}
extern bool tftThemeDark; // state declared below in TFT globals
bool saveThemeMode(uint8_t mode){
 if(mode>2||prefs.putUChar("themeMode",mode)!=sizeof(uint8_t))return false;
 tftThemeMode=mode;tftThemeDark=mode==1;
 prefs.putBool("tftDarkColor",tftThemeDark);
 tftDirty=true;return true;
}
void darkenTftDisplay(); // manual screen-off via F2 or local iPhone UI
void keypadAction(int k){
  // # and * always remain unassigned; hold 0 >2 seconds for decimal comma.
  if(k==2 || k==3)return; // # and * always unused
  if(wifiUiKey(k))return;
  if(calEditing){
    // A single inline editor for each of the four visible sensor rows.
    if(k==7||k==11){
      calDraft=constrain(roundf((calDraft+(k==7?0.1f:-0.1f))*10.0f)/10.0f,-10.0f,10.0f);
    }else if(k==0){calDraft=0.0f;keyMessage="Поправка: 0.0 C";}
    else if(k==19){
      if(saveTempOffset(calSensor,calDraft)){calEditing=false;keyMessage="Поправка T"+String(calSensor+1)+" сохранена";}
      else keyMessage="Не удалось сохранить поправку";
    }else if(k==15){calEditing=false;keyMessage="Изменение отменено";}
    // Left/right never silently commit a value: return to navigation.
    else if(k==16||k==18){calEditing=false;tftView=0;moveApprovedTab(k==16?-1:1);}
    tftDirty=true;return;
  }
  // Confirmation is the only path that removes an existing sound threshold.
  if(deleteConfirmPending){
    if(k==19){ // ENT confirms delete
      deleteSelectedReference();
      deleteConfirmPending=false;
    }else if(k==15 || k==0){ // ESC or F1 cancels
      deleteConfirmPending=false;
      keyMessage="Удаление отменено";
    }else{
      keyMessage="ENT: удалить, ESC: отменить";
    }
    tftDirty=true;return;
  }
  if(editingTemp){
    if(k==0){ // F1 clears only the input; the saved threshold is untouched
      editTempBuffer="";editCursor=0;
      keyMessage="Поле очищено. Введи число или ESC";
    }else if(k==16){ // LEFT: move numeric cursor, not a page
      if(editCursor)editCursor--;
    }else if(k==18){ // RIGHT: move numeric cursor, not a page
      if(editCursor<editTempBuffer.length())editCursor++;
    }else if(k==7){ // UP: beginning of input
      editCursor=0;
    }else if(k==11){ // DOWN: end of input
      editCursor=(uint8_t)editTempBuffer.length();
    }else if(k==15){ // ESC: discard edits
      editingTemp=false;editTempBuffer="";
      keyMessage="Изменение отменено";
    }else if(k==19){ // ENT: save only a valid numeric value
      if(editTempBuffer.isEmpty()){
        keyMessage="Пустое поле: введи число или ESC";
      }else {String parsed=editTempBuffer;parsed.replace(',','.');
       if(storeTempReference(selectedTemp,selectedField,parsed.toFloat())){
        editingTemp=false;editTempBuffer="";
        keyMessage="Порог звука сохранён";
       }else keyMessage="Проверь значение 0..125";
      }
    }else{
      const int digitKey[10]={17,4,5,6,8,9,10,12,13,14};
      String insert="";
      for(int d=0;d<10;d++)if(k==digitKey[d])insert=String(d);
      // Decimal comma is entered by holding 0 for >2 seconds; * and # are unused.
      if(insert.length() && editTempBuffer.length()<7){
        editTempBuffer=editTempBuffer.substring(0,editCursor)+
          insert+editTempBuffer.substring(editCursor);
        editCursor=(uint8_t)(editCursor+insert.length());
      }
    }
    tftDirty=true;return;
  }
  if(k==15){ // ESC: close detail or go back
    if(tftView){tftView=0;}
    else if(currentPage!=previousMainPage){
      uint8_t target=previousMainPage;
      previousMainPage=currentPage;currentPage=target;
    }else if(currentPage!=0){navigatePage(0);}
  }else if(k==0){ // F1 deletes the selected setting only.
    if(currentPage==5 && tftView && approvedSettingsItem==0){
      // Calibration deletion = revert only this sensor's correction to zero.
      if(saveTempOffset(calSensor,0.0f)){
        keyMessage="Поправка T"+String(calSensor+1)+" удалена";
      }else keyMessage="Ошибка сброса поправки";
    }else if(currentPage==1){
      deleteConfirmPending=true;
      keyMessage=String("Удалить T")+String(selectedTemp+1)+
        (selectedField?" ВЫКЛ":" ВКЛ")+"? ENT/ESC";
    }
  }else if(k==7 || k==11){ // UP/DOWN: move within current section
    if(currentPage==1){
      // Eight entries: T1 ON, T1 OFF, ... T4 ON, T4 OFF
      int item=selectedTemp*2+selectedField;
      item=(item+(k==7?7:1))%8;
      selectedTemp=item/2;selectedField=item%2;
    }else if(currentPage==5){
      if(!tftView){
        approvedSettingsItem=(approvedSettingsItem+(k==7?4:1))%5;
        settingsSubTab=approvedSettingsItem;
      }else if(approvedSettingsItem==0){
        // Temperature subpage always shows T1..T4 simultaneously.
        calSensor=(calSensor+(k==7?3:1))%4;
      }else if(approvedSettingsItem==2){
        themeDraft=(themeDraft+(k==7?2:1))%3;
      }else if(approvedSettingsItem==3){
        googleUiChoice=(googleUiChoice+1)%2;
      }
    }else if(currentPage==6){
      // Match physical top-to-bottom order in the two-column diagnostic grid.
      // DOWN: Screen -> Relays -> Wi-Fi -> Flow -> Keyboard -> Sensors -> ESP32.
      static const uint8_t diagOrder[7]={0,2,4,6,1,3,5};
      uint8_t slot=0;
      for(uint8_t n=0;n<7;n++)if(diagOrder[n]==diagModule){slot=n;break;}
      diagModule=diagOrder[(slot+(k==7?6:1))%7];tftView=0;
    }else if(currentPage==3){
      graphSubView=(graphSubView+1)%2;
    }else{
      tftView=k==7?0:1;
    }
  }else if(k==16 || k==18){ // LEFT/RIGHT: change sections everywhere
    moveApprovedTab(k==16?-1:+1);
  }else if(k==19){ // ENT: open selected element
    if(currentPage==1){
      editingTemp=true;
      float v=selectedField?noticeTemp[selectedTemp]:refTemp[selectedTemp];
      editTempBuffer=isfinite(v)?String(v,1):String("");
      editTempBuffer.replace('.',',');
      editCursor=(uint8_t)editTempBuffer.length();
    }else if(currentPage==0){
      navigatePage(1);
    }else if(currentPage==5){
      if(!tftView){
        // Settings is a real page; ENT opens the selected nested page.
        tftView=1;
        themeDraft=tftThemeMode;
        if(approvedSettingsItem==1){
          wifiUiStage=WIFIUI_STATUS;wifiUiFeedback="";wifiUiFocus=0;
          // Search starts on deliberate ENT "Поиск сети", not automatically.
        }
        if(approvedSettingsItem==3)googleUiChoice=0;
      }else if(approvedSettingsItem==0){
        calEditing=true;calDraft=tempOffset[calSensor];
      }else if(approvedSettingsItem==2){
        if(saveThemeMode(themeDraft)){
          keyMessage="Тема сохранена";
          tftView=0; // One ENT saves and returns to Settings.
        }else keyMessage="Ошибка сохранения темы";
      }else if(approvedSettingsItem==3){
        const bool enable=(googleUiChoice==0);
        if(busaGoogleUiToggle(enable)){
          keyMessage=enable?"Google подключён":"Google отключён (настройки сохранены)";
          tftView=0;
        }else keyMessage=enable?"Сначала настрой Google в приложении":"Ошибка сохранения Google";
      }
    }else if(currentPage==6){tftView=!tftView;
    }else if(currentPage==3){
      graphSubView=(graphSubView+1)%2;
    }else{
      tftView=1;
    }
  }else if(k==1){ // F2: display OFF; monitoring continues
    darkenTftDisplay();
    keyMessage="TFT погашен (ручной режим)";
  }else if(wifiUiDigitFromKey(k)>=0){
    // Numbers NEVER open sections. Digits are used only inside an editor.
    keyMessage="Цифры работают только в полях ввода";
  }
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
      writeVerifiedKeymap(prefs,"keys20",newPairs)){
      memcpy(keyPairs,newPairs,sizeof(keyPairs));
      keypadReady=true;keypadLearning=false;
      const bool backedUp=keypadBackupOpen &&
        writeVerifiedKeymap(keypadBackupPrefs,"keys20",newPairs);
      keypadStorageStatus=backedUp?"LEARNED_BACKED_UP":"LEARNED_PRIMARY_ONLY";
      keypadMessage=backedUp?
        "Все 20 клавиш сохранены и зарезервированы":
        "Все 20 клавиш сохранены (резерв недоступен)";
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
bool keypadDeferredZero=false;
// No inactivity auto-off. Only F2 or explicit iPhone screen button blanks TFT.
void pollKeys(uint32_t now){
  if(!pcfDetected||busaI2cSuspended(now)||!busaI2cLinesFree()){
    busaKeyScanReset();return;
  }
  if(!elapsed(now,lastKeyScan,BUSA_KEYPAD_SLICE_MS))return;
  if(busaKeySkipUntil && (int32_t)(now-busaKeySkipUntil)<0)return;
  lastKeyScan=now;
  const uint32_t keyStarted=millis();
  int pair=scanKeyPair();
  busaLastKeyScanMs=millis()-keyStarted;
  if(busaLastKeyScanMs>busaMaxKeyScanMs)busaMaxKeyScanMs=busaLastKeyScanMs;
  // Misbehaving I2C bus can stall the keypad scanner; back off to protect HTTP.
  if(busaLastKeyScanMs>100){
    busaI2cQuarantine(millis());
    busaKeySkipUntil=millis()+500;
    Serial.printf("BUSA SLOW KEYPAD scan_ms=%lu backoff=500\n",
      (unsigned long)busaLastKeyScanMs);
    return;
  }
  if(pair==-3)return; // incomplete four-row scan: keep debounce state untouched
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
  if(pair!=pendingPair){
    // Zero is classified only on release, so a long press never enters "0,".
    if(pair<0 && keypadDeferredZero && keyHeld && !keypadLearning){
      if((uint32_t)(now-pendingAt)>2000)wifiUiLongZero();
      else keypadAction(17);
    }
    pendingPair=pair;pendingAt=now;keyHeld=false;keypadDeferredZero=false;
  }
  if(pair<0)return;
  if(keyHeld){
    if(keypadDeferredZero && (uint32_t)(now-pendingAt)>2000){
      keypadDeferredZero=false;
      wifiUiLongZero();
    }
    return;
  }
  if(!elapsed(now,pendingAt,80))return;
  keyHeld=true;
  keyCount++;
  if(tftDisplayOff){
    // Any key wakes TFT without performing the key's original function.
    wakeTftDisplay();keyMessage="Экран пробуждён";return;
  }
  tftDisplayActivityMs=now;
  if(keypadLearning){learnCommitPair(pair);}
  else{
    int k=-1;
    if(keypadReady)for(int i=0;i<20;i++)if(keyPairs[i]==pair){k=i;break;}
    if(k<0)keyMessage="Не назначено: "+rawPair;
    else{
      keyMessage=String(KEY_CAPS[k])+" ("+rawPair+")";
      if(k==17 && (editingTemp ||
          (currentPage==5 && approvedSettingsItem==1 &&
           tftView && wifiUiStage==WIFIUI_PASSWORD))){
        keypadDeferredZero=true;
      }else keypadAction(k);
    }
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
const char* pages[7]={"Главная","Температуры","Отбор","Графики","События","Настройки","Диагностика"};
const uint8_t UI_ORDER[7]={0,1,2,5,6,3,4}; // Main T Collection Settings Graphs Events
void navigatePage(uint8_t target); // forward declaration
uint8_t orderSlot(uint8_t page){for(uint8_t i=0;i<7;i++)if(UI_ORDER[i]==page)return i;return 0;}
void moveApprovedTab(int direction){int slot=(orderSlot(currentPage)+7+direction)%7;navigatePage(UI_ORDER[slot]);}
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
  const String version="FW V15.0 N16R8";
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
  drawBootSplash(2500); // 2.5 s splash; Wi-Fi AP and web start before peripheral scans
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
bool busaSoundEnabled(uint8_t i); bool busaSoundActive(uint8_t i);
extern volatile bool googleEnabled; // Google status read by both TFT renderers
extern volatile int googleInternet,googleHttp; // TFT indicators; definitions in Google module
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
bool otaActive=false,otaSuccess=false,otaRestart=false,otaWriteAllowed=false;
uint32_t otaBytes=0,otaRestartAt=0,otaTokenIssued=0,startedAt=0;
String otaToken="",otaMessage="Ожидание",nearbyNames[5];
uint8_t nearbyCount=0;
volatile bool busaWifiScanRunning=false;
volatile bool busaStaSwitchPending=false,busaStaSwitching=false;
volatile int8_t busaStaLastResult=0; // 1 successful; -1 local fallback
String busaStaNextSsid="",busaStaNextPass="",busaStaMessage="Готово";
uint32_t busaStaSwitchAt=0,busaStaPendingAt=0;
bool busaStaBootAttempt=false;
uint32_t busaWifiScanStartedAt=0,busaWifiScanLastAt=0;
bool screenAttempted=false;
bool apClient(){return server.client().localIP()==WiFi.softAPIP();}

String apPassword="";
bool wifiRunning=false;
uint32_t lastRender=0,lastUart=0,lastStationCheck=0;
String consoleLine="",stationSsid="",stationPass="";
// Local AP and HTTP control take priority over optional external Wi-Fi STA.
void busaStaRestoreLocal(){
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false,false);
  WiFi.mode(WIFI_AP);
  wifiRunning=(WiFi.softAPIP()!=IPAddress(0,0,0,0));
  if(!wifiRunning)wifiRunning=WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str());
  lastStationCheck=millis();
}
bool busaStageStaCredentials(const String& ssid,const String& pass,bool fromBoot){
  if(otaActive||busaStaSwitchPending||busaStaSwitching||
     ssid.isEmpty()||ssid.length()>32||pass.length()<8||pass.length()>63)return false;
  busaStaNextSsid=ssid;
  busaStaNextPass=pass;
  busaStaBootAttempt=fromBoot;
  busaStaLastResult=0;
  busaStaMessage="Проверка выбранной сети";
  busaStaPendingAt=millis();
  busaStaSwitchPending=true;
  return true;
}
void busaStaService(){
  const uint32_t now=millis();
  if(busaStaSwitchPending && (uint32_t)(now-busaStaPendingAt)>=1500 &&
     !otaActive && !busaGoogleNetworkBusy &&
     !busaWifiScanRunning && !wifiUiScanPending && !wifiUiOwnsScan){
    // Association is requested AFTER the HTTP 202 response is sent.
    busaStaSwitchPending=false;
    busaStaSwitching=true;
    busaStaSwitchAt=now;
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect(false,false);
    WiFi.begin(busaStaNextSsid.c_str(),busaStaNextPass.c_str());
    busaStaMessage="Подключение к выбранной сети";
  }
  if(busaStaSwitching){
    if(WiFi.status()==WL_CONNECTED && WiFi.SSID()==busaStaNextSsid){
      bool persisted=true;
      if(!busaStaBootAttempt){
        const String oldSsid=prefs.getString("ssid","");
        const String oldPass=prefs.getString("pass","");
        persisted=(prefs.putString("pass",busaStaNextPass)==busaStaNextPass.length() &&
          prefs.putString("ssid",busaStaNextSsid)==busaStaNextSsid.length() &&
          prefs.getString("pass","")==busaStaNextPass &&
          prefs.getString("ssid","")==busaStaNextSsid);
        if(!persisted){
          prefs.putString("ssid",oldSsid);
          prefs.putString("pass",oldPass);
        }
      }
      if(persisted){
        stationSsid=busaStaNextSsid;stationPass=busaStaNextPass;
        busaStaMessage="Wi-Fi подключён, настройки сохранены";
        busaStaLastResult=1;busaStaSwitching=false;
      }else{
        busaStaMessage="Ошибка записи NVS. Локальная сеть сохранена";
        busaStaLastResult=-1;busaStaSwitching=false;
        busaStaRestoreLocal();
      }
      busaStaNextPass="";busaStaNextSsid="";
      tftDirty=true;
    }else if((uint32_t)(now-busaStaSwitchAt)>=12000){
      busaStaLastResult=-1;
      busaStaMessage="Сеть недоступна. БУСА работает локально";
      busaStaSwitching=false;
      busaStaNextSsid="";busaStaNextPass="";
      busaStaRestoreLocal();
      tftDirty=true;
    }
  }else if(!busaStaSwitchPending && !otaActive &&
      !wifiUiScanPending && !wifiUiOwnsScan && !busaWifiScanRunning &&
      WiFi.getMode()==WIFI_AP_STA && WiFi.status()!=WL_CONNECTED){
    // Never keep chasing a missing phone hotspot.
    if((uint32_t)(now-lastStationCheck)>1800)busaStaRestoreLocal();
  }
  if(!wifiRunning && WiFi.softAPIP()!=IPAddress(0,0,0,0))wifiRunning=true;
}
String quoteJson(const String &x){
  String out="\"";
  for(unsigned i=0;i<x.length();i++){
    char c=x[i];
    if(c=='\"'||c=='\\'){out+='\\';out+=c;}
    else if(c=='\n')out+="\\n";
    else if(c=='\r')out+="\\r";
    else out+=c;
  }
  return out+"\"";
}
String otaJSON(); // forward declaration
#include "BUSA_SoundAlarms.h"
#include "BUSA_RemoteAlarm.h"
// Shared TFT/editor setpoint writer; controls sound notifications only.
bool storeTempReference(int sensor,int field,float value){
  if(sensor<0||sensor>3||field<0||field>1||(isfinite(value)&&(value<0||value>125)))return false;
  const char* onKeys[4]={"buzOn0","buzOn1","sndT3On","sndT4On"};
  const char* offKeys[4]={"buzOff0","buzOff1","sndT3Off","sndT4Off"};
  const char* key=field?offKeys[sensor]:onKeys[sensor];
  const bool ok=isfinite(value)?(prefs.putFloat(key,value)>0):
    (!prefs.isKey(key)||prefs.remove(key));
  if(!ok)return false;
  (field?noticeTemp[sensor]:refTemp[sensor])=value;
  if(sensor<2){
    (field?buzzerOffC[sensor]:buzzerOnC[sensor])=value;
    buzzerTempChannels[sensor]=BusaSound::Channel();
  }else{
    if(sensor==2){
      for(int i=0;i<3;i++)(field?soundOffC[i]:soundOnC[i])=value;
    }else (field?soundOffC[3]:soundOnC[3])=value;
    soundResetState();
  }
  soundOutputsOff();
  tftDirty=true;lastRender=0;
  return true;
}
bool busaSoundEnabled(uint8_t i){return i<4&&soundEnabled[i];}
bool busaSoundActive(uint8_t i){return i<4&&soundChannels[i].on;}
String cloudJSON(); // forward declaration; read-only Google status
extern volatile int googleInternet; // external TCP health, no credentials
String statusJSON(){
  uint32_t now=millis();
  String s="{\"device\":\"BUSA-1M-001\",\"firmware\":\""+String(VERSION)+"\"";
  s+=",\"mode\":\"LOW_VOLTAGE_MONITOR_SOUND_ONLY\",\"relays_enabled\":true,\"pwm_enabled\":false";
  s+=",\"performance\":{\"stage\":"+quoteJson(busaBootStage);
  s+=",\"setup_done_ms\":"+String(busaSetupDoneMs);
  s+=",\"loop_worst_ms\":"+String(busaMaxLoopMs);
  s+=",\"slow_loop_count\":"+String(busaSlowLoops);
  s+=",\"key_scan_last_ms\":"+String(busaLastKeyScanMs);
  s+=",\"key_scan_worst_ms\":"+String(busaMaxKeyScanMs);
  s+=",\"i2c_step_last_ms\":"+String(busaLastI2cStepMs);
  s+=",\"i2c_step_worst_ms\":"+String(busaMaxI2cStepMs);
  s+=",\"i2c_errors\":"+String(busaI2cErrorCount);
  s+=",\"i2c_stalls\":"+String(busaI2cStallCount);
  s+=",\"i2c_quarantined\":"+(busaI2cSuspended(now)?String("true"):String("false"));
  s+=",\"last_slow_component\":"+quoteJson(busaSlowComponent);
  s+=",\"temp_worst_ms\":"+String(busaTempWorstMs);
  s+=",\"onewire_worst_ms\":"+String(busaOneWireWorstMs);
  s+=",\"http_worst_ms\":"+String(busaHttpWorstMs);
  s+=",\"tft_worst_ms\":"+String(busaTftWorstMs);
  s+=",\"tft_last_ms\":"+String(busaLastTftRenderMs);
  s+=",\"tft_live_refresh_target_ms\":"+String(BUSA_TFT_LIVE_REFRESH_MS);
  s+=",\"tft_full_render_count\":"+String(busaTftFullRenderCount);
  s+=",\"tft_live_render_count\":"+String(busaTftLiveRenderCount);
  s+=",\"cpu_mhz\":"+String(ESP.getCpuFreqMHz());
  s+=",\"cpu_240mhz\":"+(busaCpu240Ok?String("true"):String("false"));
  s+=",\"psram_detected\":"+(busaPsramDetected?String("true"):String("false"));
  s+=",\"psram_total_bytes\":"+String(ESP.getPsramSize());
  s+=",\"psram_free_bytes\":"+String(ESP.getFreePsram());
  s+=",\"wifi_tx_power_dbm\":"+String(((int)WiFi.getTxPower())/4.0f,1)+"}";
  s+=",\"configuration\":{\"manual_external_controls\":true,\"mains_control_in_project\":false,\"motorized_valve_in_project\":false}";
  s+=",\"ui_page\":"+String(currentPage+1)+",\"ui_subtab\":"+String(settingsSubTab)+",\"graph_view\":"+String(graphSubView)+",\"tft_view\":"+String(tftView)+",\"uptime_ms\":"+String(now);
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
  s+=",\"sound_alerts\":"+soundJSON(now);
  s+=",\"remote_alarm\":"+busaRemoteAlarmJSON();
  s+=",\"temperatures\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    s+="{\"name\":"+quoteJson(String(i==0?"Вода":i==1?"Колонна":i==2?"Куб":"Верх колонны"));
    s+=",\"gpio\":"+String(TPIN[i])+",\"present\":"+(tempPresent[i]?"true":"false");
    s+=",\"c\":"+(hasTemp(i,now)?String(temperature[i],2):String("null"));
    s+=",\"offset_c\":"+String(tempOffset[i],1)+"}";
  }
  s+="],\"water\":{\"gpio\":15,\"pulses\":"+String(flowPulses());
  s+=",\"lpm\":"+(flowReady?String(flowLpm,3):String("null"));
  s+=",\"total_l_estimate\":"+String(totalWaterLitres(),3);
  s+=",\"calibrated\":false,\"seen_pulses\":"+(flowSeen?String("true"):String("false"))+"}";
  s+=",\"collection\":{\"sensor_present\":false,\"ml_per_min\":null,\"total_ml\":null}";
  s+=",\"modules\":{\"pcf8575\":"+(pcfDetected?String("true"):String("false"));
  s+=",\"pca9685_ack_only\":false,\"pca9685_enabled\":false";
  s+=",\"tft_attempted\":"+(tftStarted?String("true"):String("false"));
  s+=",\"tft_rotation\":"+String(tftRotation);
  s+=",\"tft_boot_complete\":"+(bootSplashComplete?String("true"):String("false"));
  s+=",\"tft_profile_id\":"+String(tftProfile);
  s+=",\"tft_profile\":"+quoteJson(tftProfileName());
  s+=",\"tft_physical_pixels_confirmed\":false";
  s+=",\"last_key\":"+quoteJson(keyMessage)+"}";
  s+=",\"tft_theme_inverted\":"+(tftInverted?String("true"):String("false"));
  s+=",\"tft_theme_dark\":"+(tftThemeDark?String("true"):String("false"));
  s+=",\"tft_theme_mode\":"+String(tftThemeMode);
  s+=",\"calibration_offsets\":[";
  for(int i=0;i<4;i++){if(i)s+=",";s+=String(tempOffset[i],1);}s+="]";
  s+=",\"display_sleep\":"+(tftDisplayOff?String("true"):String("false"));
  s+=",\"keypad\":{\"ready\":"+(keypadReady?String("true"):String("false"));
  s+=",\"learning\":"+(keypadLearning?String("true"):String("false"));
  s+=",\"step\":"+String(learnStep);
  s+=",\"expected\":"+quoteJson(keypadLearning&&learnStep<20?String(KEY_CAPS[learnStep]):String(""));
  s+=",\"raw_pair\":"+quoteJson(rawPair);
  s+=",\"message\":"+quoteJson(keypadMessage);
  s+=",\"storage\":"+quoteJson(keypadStorageStatus);
  s+=",\"primary_bytes\":"+String(prefs.getBytesLength("keys20"));
  s+=",\"backup_bytes\":"+String(keypadBackupOpen?keypadBackupPrefs.getBytesLength("keys20"):0)+"}";
  s+=",\"diagnostics\":{\"onewire\":[";
  for(int i=0;i<4;i++){
    if(i)s+=",";
    const OneWireProbe& p=wireProbe[i];
    s+="{\"gpio\":"+String(TPIN[i]);
    s+=",\"probe_ms\":"+String(p.checkedAt);
    s+=",\"presence\":"+(p.presence?String("true"):String("false"));
    s+=",\"rom_found\":"+(p.romFound?String("true"):String("false"));
    s+=",\"idle_gpio\":"+String(p.idleLevel);
    s+=",\"after_reset_gpio\":"+String(p.afterResetLevel);
    s+=",\"rom_crc_ok\":"+(p.crcOk?String("true"):String("false"));
    s+=",\"family_ds18b20\":"+(p.familyOk?String("true"):String("false"));
    s+=",\"multiple_rom\":"+(p.multiple?String("true"):String("false"));
    s+=",\"rom\":"+quoteJson(p.romFound?romHex(p.rom):String(""));
    s+=",\"result\":"+quoteJson(wireError(i,now))+"}";
  }
  s+="],\"keyboard\":{\"pcf_address\":"+String(pcfAddr);
  s+=",\"last_scan_ms\":"+String(pcfLastScanAt);
  s+=",\"idle_hex\":"+quoteJson(String(pcfIdleMask,HEX));
  s+=",\"low_hex\":"+quoteJson(String(pcfObservedLow,HEX));
  s+=",\"pairs\":"+String(pcfPairCount);
  s+=",\"scan_error\":"+String(pcfScanError);
  s+=",\"raw_pair\":"+quoteJson(rawPair)+"}";
  s+=",\"i2c\":{\"sda_gpio\":"+String(BUS_SDA)+",\"scl_gpio\":"+String(BUS_SCL)+",\"controller_started\":"+(i2cOnline?String("true"):String("false"));
  s+=",\"last_scan_ms\":"+String(i2cFullScanAt);
  s+=",\"scan_range\":\"PCF_ONLY_0x20_0x27_0x38_0x3F\"";
  s+=",\"scan_next\":"+String(busaI2cScanNext);
  s+=",\"pcf_address\":"+String(pcfAddr);
  s+=",\"pca_address\":null";
  s+=",\"ack_fresh_ms\":"+String(BUSA_I2C_ACK_FRESH_MS);
  s+=",\"pcf_last_ack_ms\":"+String(busaI2cLastPcfAckMs);
  s+=",\"pca_last_ack_ms\":0";
  s+=",\"pcf_ack_total\":"+String(busaI2cPcfAcks);
  s+=",\"pca_ack_total\":0";
  s+=",\"pcf_last_nack_ms\":"+String(busaI2cLastPcfNackMs);
  s+=",\"pca_last_nack_ms\":0";
  s+=",\"driver\":\"IDF_INPUT_OUTPUT_OPEN_DRAIN\"";
  s+=",\"electrical_fault\":"+(busaI2cElectricalFault?String("true"):String("false"));
  s+=",\"scan_suppressed\":"+(busaI2cElectricalFault?String("true"):String("false"));
  s+=",\"gpio_config_errors\":"+String(busaI2cGpioConfigErrors);
  s+=",\"pad_reinit_count\":"+String(busaI2cPadReinitCount);
  s+=",\"pad_last_reinit_ms\":"+String(busaI2cLastPadReinitMs);
  s+=",\"sda_idf_low_readback\":"+String(busaI2cSdaIdfLow);
  s+=",\"scl_idf_low_readback\":"+String(busaI2cSclIdfLow);
  s+=",\"fault_hint\":"+quoteJson(!busaI2cElectricalFault?String("NONE"):
    busaI2cTestSdaForcedLow?String("SDA_GPIO40_NOT_LOW"):
    busaI2cTestSclForcedLow?String("SCL_GPIO41_NOT_LOW"):
    String("I2C_PAD_READBACK_FAILED"));
  s+=",\"gpio_test\":{\"count\":"+String(busaI2cGpioTests);
  s+=",\"tested_at_ms\":"+String(busaI2cLastGpioTestMs);
  s+=",\"passed\":"+(busaI2cGpioTestPassed?String("true"):String("false"));
  s+=",\"idle_sda\":"+String(busaI2cTestIdleSda);
  s+=",\"idle_scl\":"+String(busaI2cTestIdleScl);
  s+=",\"sda_forced_low\":"+String(busaI2cTestSdaForcedLow);
  s+=",\"scl_forced_low\":"+String(busaI2cTestSclForcedLow);
  s+=",\"sda_released\":"+String(busaI2cTestSdaReleased);
  s+=",\"scl_released\":"+String(busaI2cTestSclReleased);
  s+=",\"sda_while_scl_low\":"+String(busaI2cTestSdaWhileSclLow);
  s+=",\"scl_while_sda_low\":"+String(busaI2cTestSclWhileSdaLow)+"}";
  s+=",\"ack_early_level\":"+String(busaI2cAckSampleEarly);
  s+=",\"ack_late_level\":"+String(busaI2cAckSampleLate);
  s+=",\"last_tx_byte\":"+String(busaI2cLastTxByte);
  s+=",\"ack_unstable\":"+String(busaI2cAckUnstableCount);
  s+=",\"sda_output_faults\":"+String(busaI2cSdaOutputFaults);
  s+=",\"low_during_high_bits\":"+String(busaI2cLowDuringHighBits);
  s+=",\"clock_stretch_events\":"+String(busaI2cClockStretchCount);
  s+=",\"last_result\":"+String(busaI2cLastResult);
  s+=",\"last_address\":"+String(busaI2cLastAddress);
  s+=",\"nack_count\":"+String(busaI2cNackCount);
  s+=",\"timeout_count\":"+String(busaI2cTimeoutCount);
  s+=",\"recovery_count\":"+String(busaI2cRecoverCount);
  s+=",\"sda_level\":"+String(busaI2cLastSda);
  s+=",\"scl_level\":"+String(busaI2cLastScl);
  s+=",\"ack_total\":"+String(i2cAckCount)+",\"addresses\":[";
  for(uint8_t i=0;i<i2cFoundCount;i++){if(i)s+=",";s+=String(i2cFound[i]);}
  s+="]}";
  s+=",\"pcf8575\":{\"expected_address\":32,\"online\":"+(pcfDetected?String("true"):String("false"));
  s+=",\"address\":"+String(pcfAddr);
  s+=",\"read_ok\":"+(pcfIdleReadOk?String("true"):String("false"));
  s+=",\"idle_pins_hex\":"+quoteJson(String(pcfIdleDirect,HEX));
  s+=",\"keyscan_error\":"+String(pcfScanError)+"}";
  s+=",\"pca9685\":{\"expected_address\":64,\"installed\":false,\"address\":0,\"ack\":false";
  s+=",\"mode1\":"+String(pcaMode1)+",\"mode2\":"+String(pcaMode2);
  s+=",\"outputs_enabled\":false}";
  s+=",\"tft\":{\"spi_cs\":"+String(TFT_CS)+",\"spi_mosi\":"+String(TFT_MOSI)+",\"spi_sck\":"+String(TFT_SCK)+",\"dc\":"+String(TFT_DC)+",\"rst\":"+String(TFT_RST);
  s+=",\"profile\":"+quoteJson(tftProfileName())+",\"rotation\":"+String(tftRotation);
  s+=",\"init_sent\":"+(tftStarted?String("true"):String("false"));
  s+=",\"boot_complete\":"+(bootSplashComplete?String("true"):String("false"));
  s+=",\"pixel_feedback_available\":false}";
  s+=",\"water_flow\":{\"gpio\":15,\"input_level\":"+String(digitalRead(FLOW_GPIO));
  s+=",\"pulses\":"+String(flowPulses())+",\"pulse_seen\":"+(flowSeen?String("true"):String("false"));
  s+=",\"frequency_calibrated\":false,\"status\":"+quoteJson(flowSeen?"PULSES_SEEN":"WAITING_FOR_PULSES")+"}";
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
  s+=",\"bluetooth_enabled\":false";
  s+=",\"google\":"+cloudJSON();
  s+=",\"network_indicators\":{\"internet\":"+(googleInternet==1?String("true"):String("false"));
  s+=",\"local_client\":"+(WiFi.softAPgetStationNum()>0?String("true"):String("false"));
  s+=",\"ap_running\":"+(wifiRunning?String("true"):String("false"))+"}";
  s+=",\"sta_switch\":{\"pending\":"+(busaStaSwitchPending?String("true"):String("false"))+
    ",\"connecting\":"+(busaStaSwitching?String("true"):String("false"))+
    ",\"result\":"+String((int)busaStaLastResult)+
    ",\"message\":"+quoteJson(busaStaMessage)+"}";
  s+=",\"station\":{\"ssid\":"+quoteJson(WiFi.status()==WL_CONNECTED?WiFi.SSID():stationSsid);
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
  otaWriteAllowed=false;otaActive=false;otaSuccess=false;otaBytes=0;
  if(!apClient()||otaToken.length()<16||(uint32_t)(millis()-otaTokenIssued)>90000
     ||server.header("X-Busa-Token")!=otaToken||!u.filename.endsWith(".bin")){
    otaMessage="Отказ: локальная авторизация / формат BIN";return;
  }
  otaToken="";
  if(!Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH)){
    otaMessage="Не найден OTA-раздел";Update.printError(Serial0);return;
  }
  otaWriteAllowed=true;otaActive=true;otaMessage="Записывается";
 }else if(u.status==UPLOAD_FILE_WRITE){
  if(!otaWriteAllowed)return;
  if(otaBytes==0 && (u.currentSize<4||u.buf[0]!=0xe9)){
    otaMessage="Неверный образ APP BIN";Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  if(Update.write(u.buf,u.currentSize)!=u.currentSize){
    otaMessage="Ошибка записи";Update.printError(Serial0);
    Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  otaBytes+=u.currentSize;
 }else if(u.status==UPLOAD_FILE_END){
  if(!otaWriteAllowed)return;
  otaSuccess=Update.end(true);otaActive=false;otaWriteAllowed=false;
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
.appTabs{position:fixed;bottom:0;left:0;right:0;z-index:15;display:grid!important;grid-template-columns:repeat(6,minmax(0,1fr));gap:2px!important;overflow:visible!important;background:var(--surface);border-top:2px solid var(--orange);padding:6px 5px max(7px,env(safe-area-inset-bottom));box-shadow:0 -4px 15px #2f485c18}
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
#settingsTabs,.diagTabs{display:flex;flex-wrap:wrap;gap:6px}
#settingsTabs button,.diagTabs button{flex:1 1 100px}
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

/* Three independent network badges; red disconnected, green connected. */
.netIndicators{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin:5px auto 3px;max-width:620px;font-size:11px;line-height:1.3}
.netPill{display:inline-flex;align-items:center;gap:5px;padding:4px 7px;border:1px solid var(--line);background:var(--surface);border-radius:8px;font-weight:700;color:var(--ink);max-width:100%}
.netDot{display:inline-block;width:10px;height:10px;flex:0 0 10px;border-radius:50%;background:#d33a32;box-shadow:0 0 0 1px #73737355}
.netDot.internetOn,.netDot.localOn,.netDot.googleOn{background:#13a44c}
.netText{font-size:10px;font-weight:500;color:var(--muted);overflow:hidden;text-overflow:ellipsis;white-space:nowrap;max-width:145px}

.busaWifiMode{padding:10px 0;border-top:1px solid var(--line)}
.busaWifiMode h3{font-size:14px;margin:5px 0 8px}
/* SYSTEM THEME: mirror the approved BUSA public online monitor UI.
   Palette values are copied from online/index.html in main on 2026-11-02.
   This is strictly presentation; existing local operations stay unchanged. */
:root[data-theme="system"]{
 color-scheme:dark;
 --busa-bg:#0b1928;--busa-panel:#152a3d;--busa-edge:#34546a;
 --busa-blue:#299bde;--busa-green:#0b994f;--busa-green-text:#eafff2;
 --busa-red:#df4855;--busa-text:#f0f7ff;--busa-muted:#a9c5d7;
 --ink:var(--busa-text);--muted:var(--busa-muted);--paper:var(--busa-bg);
 --surface:var(--busa-panel);--line:var(--busa-edge);--sea:var(--busa-blue);
 --orange:var(--busa-blue);--warn:#ff7584;
 background:var(--busa-bg);color:var(--busa-text);
}
:root[data-theme="system"] body{background:var(--busa-bg)!important;color:var(--busa-text)}
:root[data-theme="system"] header,
:root[data-theme="system"] .appHeader{
 background:#112436;color:var(--busa-text);border-bottom:2px solid var(--busa-blue);
 box-shadow:0 3px 12px #0004;
}
:root[data-theme="system"] h1,:root[data-theme="system"] h2,
:root[data-theme="system"] h3{color:var(--busa-text)}
:root[data-theme="system"] .brandSmall{color:#73c9ff}
:root[data-theme="system"] .brandSub,
:root[data-theme="system"] .hint,
:root[data-theme="system"] .clockBar,
:root[data-theme="system"] #clockDisplay{color:var(--busa-muted)!important}
:root[data-theme="system"] .readOnlyLabel{color:#72e4ad}
:root[data-theme="system"] main .card{
 background:var(--busa-panel);border:1px solid var(--busa-edge);
 color:var(--busa-text);box-shadow:0 4px 12px #0002;
}
:root[data-theme="system"] .homeHero{
 background:linear-gradient(125deg,#17374f,#0b2943);
 border:1px solid #3878a1;
}
:root[data-theme="system"] .homeHero h2{color:var(--busa-text)}
:root[data-theme="system"] .heroEyebrow{color:#8ecfec}
:root[data-theme="system"] .heroStatus{color:#8ce6c3}
:root[data-theme="system"] .tile{
 background:#1a364b;border-color:#346583;color:var(--busa-text)
}
:root[data-theme="system"] .tile small{color:#b9d6e5}
:root[data-theme="system"] .tile strong,
:root[data-theme="system"] .green{color:#70e4b0}
:root[data-theme="system"] .row{border-color:#2c5068}
:root[data-theme="system"] .netPill,
:root[data-theme="system"] .relayGrid span{
 background:#17364a;border-color:#3c6a83;color:#dceefb;
}
:root[data-theme="system"] .relayGrid b{color:#70e4b0}
:root[data-theme="system"] .netText{color:var(--busa-muted)}
:root[data-theme="system"] .appTabs{
 background:#10283b!important;border-top:2px solid var(--busa-blue)!important;
}
:root[data-theme="system"] .appTabs button{
 background:transparent!important;color:#b3cadb;border:1px solid transparent;
}
:root[data-theme="system"] .appTabs button.on{
 background:var(--busa-green)!important;color:var(--busa-green-text)!important;
 border:1px solid #41d98d!important;font-weight:800;
}
:root[data-theme="system"] .tabIcon{color:#79d3ff}
:root[data-theme="system"] .appTabs button.on .tabIcon{color:#e5fff0!important}
:root[data-theme="system"] button{
 background:#17425b;color:#ecfaff;border:1px solid #3d83aa;
}
:root[data-theme="system"] button:focus-visible,
:root[data-theme="system"] input:focus-visible,
:root[data-theme="system"] select:focus-visible{
 outline:2px solid #4cdb94;outline-offset:2px;
}
:root[data-theme="system"] .settingsTabs button.on,
:root[data-theme="system"] .eventFilters button.on,
:root[data-theme="system"] .graphTabs button.on{
 background:var(--busa-green)!important;color:#fff!important;
 border-color:#41d98d!important;font-weight:800;
}
:root[data-theme="system"] input,
:root[data-theme="system"] select,
:root[data-theme="system"] textarea,
:root[data-theme="system"] pre{
 background:#112b40;color:#f4faff;border-color:#4c819b;
}
:root[data-theme="system"] input::placeholder{color:#a3bdce}
:root[data-theme="system"] a{color:#82d0ff}
:root[data-theme="system"] canvas{
 background:#192f45;border:1px solid #34546a;
}
:root[data-theme="system"] .rigViewport{
 box-shadow:inset 0 0 18px #0b2d4266;border-radius:13px;
}
:root[data-theme="system"] .rigViewport svg{filter:drop-shadow(0 6px 8px #0002)}
/* Seven tabs: the extra Diagnostics section remains user-approved. */
.appTabs{grid-template-columns:repeat(7,minmax(0,1fr))!important}
.appTabs button{font-size:clamp(8px,2.1vw,10px)!important;padding:7px 0!important}
.appTabs .tabIcon{font-size:17px}
@media(max-width:380px){
 .appTabs button{font-size:8px!important}
 .appTabs .tabIcon{font-size:15px}
}


/* USER APPROVED 20–21 OCTOBER SIX-SCREEN BUSA APP DESIGN.
   Layout is compact, dark, cyan-outlined; this CSS affects the PHONE,
   not the physical TFT and not sensor/relay/communications logic. */
:root[data-theme="system"],:root[data-theme="dark"]{
 --approved-bg:#04111f;--approved-panel:#071c32;--approved-panel-hi:#0c2947;
 --approved-edge:#195486;--approved-glow:#078ff6;--approved-text:#e5f6ff;
 --approved-number:#4ee84c;--approved-muted:#8cb8d8;
}
:root[data-theme="system"] body,:root[data-theme="dark"] body{
 background:var(--approved-bg)!important;color:var(--approved-text)
}
:root[data-theme="system"] .appHeader,:root[data-theme="dark"] .appHeader{
 background:#03111f!important;color:#dff4ff!important;border-bottom:1px solid #144b7b!important;box-shadow:none!important;
}
:root[data-theme="system"] .appHeader .netPill,:root[data-theme="dark"] .appHeader .netPill{
 background:transparent!important;border-color:#20517a!important;color:#a7dcf5!important
}
:root[data-theme="system"] .appTabs,:root[data-theme="dark"] .appTabs{
 background:#041423!important;border-top:1px solid #13568a!important;box-shadow:0 -4px 14px #0007!important
}
:root[data-theme="system"] .appTabs button,:root[data-theme="dark"] .appTabs button{
 background:transparent!important;border:1px solid transparent!important;color:#b6d9eb!important
}
:root[data-theme="system"] .appTabs button.on,:root[data-theme="dark"] .appTabs button.on{
 background:linear-gradient(160deg,#117cf2,#0344b4)!important;
 border:1px solid #57bdff!important;color:#fff!important;box-shadow:0 0 12px #0a80ff77;
}
:root[data-theme="system"] .appTabs button .uiIcon,:root[data-theme="dark"] .appTabs button .uiIcon{color:#75d9ff}
:root[data-theme="system"] .appTabs button.on .uiIcon,:root[data-theme="dark"] .appTabs button.on .uiIcon{color:#fff}
:root[data-theme="system"] main .card,:root[data-theme="dark"] main .card{
 background:var(--approved-panel)!important;border:1px solid var(--approved-edge)!important;color:var(--approved-text);box-shadow:inset 0 1px 0 #336f991a
}
:root[data-theme="system"] main h2,:root[data-theme="dark"] main h2{color:var(--approved-text)}
:root[data-theme="system"] .settingsTabs button.on,:root[data-theme="dark"] .settingsTabs button.on,
:root[data-theme="system"] .eventFilters button.on,:root[data-theme="dark"] .eventFilters button.on{
 background:linear-gradient(160deg,#087fee,#03449b)!important;border-color:#45bfff!important;
 color:white!important;box-shadow:0 0 10px #119afc77
}
:root[data-theme="system"] .approvedSensorLive,:root[data-theme="dark"] .approvedSensorLive,
:root[data-theme="system"] .approvedTemp strong,:root[data-theme="dark"] .approvedTemp strong{color:#4ee84c!important}
:root[data-theme="system"] .approvedSensor,:root[data-theme="dark"] .approvedSensor,
:root[data-theme="system"] .approvedTemp,:root[data-theme="dark"] .approvedTemp,
:root[data-theme="system"] .approvedMetric,:root[data-theme="dark"] .approvedMetric,
:root[data-theme="system"] .approvedCollectionCell,:root[data-theme="dark"] .approvedCollectionCell{
 background:linear-gradient(145deg,#0d3456,#06192f)!important;border-color:#145c9a!important
}
:root[data-theme="system"] .approvedPageTitle .uiIcon,:root[data-theme="dark"] .approvedPageTitle .uiIcon{color:#40c8ff}
:root[data-theme="system"] .approvedMenuItem,:root[data-theme="dark"] .approvedMenuItem{
 background:linear-gradient(90deg,#0c2844,#06192f)!important;border-color:#19568b!important;color:#e3f6ff!important
}
:root[data-theme="system"] .approvedMenuIcon,:root[data-theme="dark"] .approvedMenuIcon{color:#4ec7ff}
:root[data-theme="system"] .approvedGraphPlaceholder,:root[data-theme="dark"] .approvedGraphPlaceholder{
 background:#061b30;color:#88bcd6;border-color:#255b8a
}
:root[data-theme="system"] .approvedEvents .eventRow,:root[data-theme="dark"] .approvedEvents .eventRow{border-color:#205482}
:root[data-theme="system"] .approvedSensor small,:root[data-theme="dark"] .approvedSensor small,
:root[data-theme="system"] .approvedTemp small,:root[data-theme="dark"] .approvedTemp small,
:root[data-theme="system"] .approvedMetric small,:root[data-theme="dark"] .approvedMetric small,
:root[data-theme="system"] .approvedCollectionCell small,:root[data-theme="dark"] .approvedCollectionCell small{color:#92c9e7}
:root[data-theme="system"] .approvedBack,:root[data-theme="dark"] .approvedBack{color:#79d2ff;border-color:#24639a;background:#08233c}
.uiIcon{display:block;flex:0 0 auto;width:22px;height:22px}
.appHeader .headerRow h1{font-size:18px}
.appTabs{grid-template-columns:repeat(6,minmax(0,1fr))!important;padding:4px 3px max(6px,env(safe-area-inset-bottom))!important}
.appTabs button{font-size:clamp(8px,2.45vw,11px)!important;min-height:50px;padding:7px 1px!important;gap:3px}
.appTabs .uiIcon{width:22px;height:22px}
@media(max-width:380px){.appTabs button{font-size:8px!important}.appTabs .uiIcon{width:20px;height:20px}}
.approvedHomeTitle{display:flex;justify-content:space-between;align-items:center;gap:6px;padding:11px 4px 8px;font-size:13px}
.approvedHomeTitle strong{font-size:16px;color:#56cfff}
.approvedHomeTitle span{color:#8dd9b0;font-size:11px}
.approvedDashboard{display:grid;grid-template-columns:minmax(0,.91fr) minmax(0,1.09fr);gap:7px;align-items:stretch}
.approvedTempGrid{display:flex;flex-direction:column;gap:6px;min-width:0}
.approvedTemp{border:1px solid var(--line);border-radius:9px;padding:7px 7px 6px;min-height:68px;min-width:0}
.approvedTempTitle{display:flex;align-items:center;gap:5px;line-height:1.1;font-size:12px}
.approvedTempIcon{display:flex;align-items:center;justify-content:center;min-width:22px;font-size:21px;color:#3ccafa}
.approvedTemp strong{display:block;font-size:clamp(17px,4.1vw,25px);line-height:1.12;margin:4px 0 0;color:#24965c}
.approvedTemp small{display:block;white-space:normal;line-height:1.15;font-size:10px}
.approvedRig{position:relative;overflow:hidden;min-width:0;min-height:292px;border:1px solid #115d92;border-radius:10px;background:radial-gradient(ellipse at 50% 45%,#17466a,#03121f 75%);display:flex;align-items:center;justify-content:center}
.approvedRig svg{display:block;width:100%;height:100%;max-height:315px;min-width:0;overflow:hidden}
.approvedRigLabel{position:absolute;bottom:3px;font:9px system-ui;letter-spacing:.4px;color:#70c6ff;background:#081c2ddd;padding:2px 5px;border-radius:5px}
.approvedMetrics{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:7px;margin:8px 0}
.approvedMetric{border:1px solid var(--line);border-radius:9px;padding:9px;display:flex;align-items:center;gap:8px;min-width:0}
.approvedMetric>span{font-size:28px;color:#46bbff}
.approvedMetric>div{display:flex;flex-direction:column;min-width:0}
.approvedMetric small{font-size:10px}
.approvedMetric strong{font-size:20px;color:#72d5ef;line-height:1.25}
.approvedStatus{display:flex;gap:7px;justify-content:space-between;align-items:center;padding:8px 9px;background:#0c2740;border:1px solid #195c91;border-radius:9px;font-size:11px;color:#67e5a2}
.approvedAdditional{font-size:13px}
.approvedAdditional summary{cursor:pointer;font-weight:600;color:inherit}
.approvedPageTitle{display:flex;align-items:center;gap:8px;margin:11px 0 8px;font-size:18px;font-weight:700;letter-spacing:.3px}
.approvedPageTitle .uiIcon{width:24px;height:24px}
.approvedSensorList{display:grid;grid-template-columns:1fr;gap:9px;margin-bottom:9px}
.approvedSensor{border:1px solid var(--line);border-radius:10px;overflow:hidden}
.approvedSensorHead{display:flex;align-items:center;gap:11px;padding:13px 11px;min-height:81px}
.approvedSensorIcon{font-size:30px;width:42px;text-align:center;color:#53d4ff}
.approvedSensorText{flex:1;min-width:0;display:flex;flex-direction:column;gap:4px}
.approvedSensorText strong{font-size:14px}
.approvedSensorText small{font-size:11px;line-height:1.2}
.approvedSensorLive{font-size:clamp(19px,5.5vw,29px);white-space:nowrap;color:#269657}
.sensorEdit{border-top:1px solid #1c5984;padding:8px 11px 10px}
.sensorEdit summary{font-size:12px;cursor:pointer;color:#70c5f2}
.approvedFieldGrid{margin-top:8px}
.approvedCollectionMetrics{display:grid;grid-template-columns:1fr;gap:8px;margin-bottom:10px}
.approvedCollectionCell{border:1px solid var(--line);border-radius:10px;min-height:78px;padding:12px;display:flex;flex-direction:column;gap:5px}
.approvedCollectionCell strong{font-size:23px;color:#74d6eb}
.approvedCollectionCell small{font-size:12px}
.approvedGraphCard{padding:12px}
.approvedGraphPlaceholder{display:flex;align-items:center;justify-content:center;height:155px;position:relative;border:1px solid #27516d;border-radius:7px;overflow:hidden}
.approvedGridLines{position:absolute;inset:10px;background:repeating-linear-gradient(to bottom,transparent 0,transparent 27px,#35628488 28px,#35628488 29px)}
.approvedGraphPlaceholder>span{position:relative;z-index:1;background:#0b1e31cb;padding:4px 8px;border-radius:5px}
.approvedGraphTabs{grid-template-columns:repeat(2,minmax(0,1fr))!important}
.approvedBlackGraph canvas.approvedLaneCanvas{height:338px!important;background:#000!important;border:1px solid #2774a5!important}
.approvedBlackGraph canvas.approvedWaterCanvas{height:245px!important;background:#000!important;border:1px solid #2774a5!important}
.approvedBlackGraph .graphPanel h3{margin:12px 0 10px}
.approvedGraphTabs button{font-size:12px;min-width:0;padding:8px 2px}
.approvedLegend{display:flex;justify-content:space-between;flex-wrap:wrap;gap:4px;font-size:10px;margin:5px 0 6px}
.approvedLegend .temp0{color:#078fbe}.approvedLegend .temp1{color:#df8744}
.approvedLegend .temp2{color:#d65446}.approvedLegend .temp3{color:#3b9f74}
.approvedSession{display:block;text-align:center;font-size:11px;margin:10px 0 2px;color:var(--muted)}
.approvedEvents{white-space:normal!important;font-size:13px}
.approvedEvents .eventRow{padding:10px 2px;border-bottom:1px solid var(--line);display:flex;gap:9px;align-items:flex-start}
.approvedEvents .eventRow:last-child{border-bottom:0}
.approvedEvents .eventRow .eventDot{width:9px;height:9px;min-width:9px;background:#22aeff;border-radius:100%;margin-top:4px}
.approvedEvents .eventRow.warn .eventDot{background:#efaa48}
.approvedEvents .eventRow.alarm .eventDot{background:#ff5965}
.approvedSettingsMain{display:flex;flex-direction:column;gap:0;padding:0!important;overflow:hidden}
.approvedMenuItem{display:flex;align-items:center;gap:12px;width:100%;min-height:53px;text-align:left;border:0!important;border-radius:0!important;border-bottom:1px solid var(--line)!important;padding:9px 12px}
.approvedMenuItem:last-child{border-bottom:0!important}
.approvedMenuIcon{display:flex;width:30px;height:30px;align-items:center;justify-content:center;color:var(--sea)}
.approvedMenuIcon .uiIcon{width:25px;height:25px}
.approvedMenuItem>span:nth-child(2){flex:1}
.approvedArrow{color:#71c7fa;font-size:30px;line-height:16px}
.approvedBack{margin:4px 0 7px;padding:10px 12px;width:100%;text-align:left;border-radius:8px}
.approvedBack[hidden],#settingsMenu[hidden]{display:none!important}
.settingCard[hidden]{display:none!important}


/* Approved 03.11 BUSA №001 dashboard design; original actual telemetry preserved. */
section[data-section="0"]{--ink:#e8f7ff;--paper:#03101f;--line:#116cab;--surface:#06182c;
 color:#e8f7ff;background:linear-gradient(160deg,#041729,#020916);
 border:1px solid #16496b;border-radius:6px;padding:7px 8px 12px}
section[data-section="0"] .approvedHomeTitle{margin:0;padding:7px 3px 11px;border-bottom:1px solid #1f5a84}
section[data-section="0"] .approvedHomeTitle strong{color:#f6faff}
section[data-section="0"] .approvedDashboard{grid-template-columns:minmax(0,1fr) minmax(0,.87fr);gap:7px}
section[data-section="0"] .approvedTemp{display:flex;flex-direction:column;justify-content:center;
 background:linear-gradient(135deg,#071b31,#031022);border:1px solid #0d78be;
 box-shadow:inset 0 0 9px #067bdb24,0 0 3px #159ce744;border-radius:11px;min-height:82px;padding:7px 6px}
section[data-section="0"] .approvedTempTitle{font-size:clamp(10px,2.6vw,15px);color:#f3faff;gap:5px}
section[data-section="0"] .approvedTempTitle>span:last-child{flex:1;min-width:0}
section[data-section="0"] .approvedTempIcon{width:32px;min-width:32px;height:37px;flex:none}
section[data-section="0"] .busaRefTempIcon{width:100%;height:100%;filter:drop-shadow(0 0 3px #00bcff91)}
section[data-section="0"] .approvedTemp strong{color:#32ee2a!important;font-weight:800;font-size:clamp(20px,4.8vw,34px)}
section[data-section="0"] .approvedTemp small{color:#b4d7ec!important;font-size:10px}
section[data-section="0"] .approvedRig{background:#031021;border:0;border-radius:0;min-height:345px}
section[data-section="0"] .approvedRig svg{max-height:345px}
section[data-section="0"] .approvedRigLabel{display:none}
section[data-section="0"] .approvedMetric{background:#051729;border:1px solid #0b80c7;
 box-shadow:inset 0 0 8px #078bde18,0 0 3px #0a72bc;border-radius:10px;padding:9px}
section[data-section="0"] .approvedMetric strong{font-size:clamp(18px,5vw,30px);color:#c9ecff!important}
section[data-section="0"] .approvedMetric small{color:#c2dded!important}
section[data-section="0"] .approvedStatus{background:#061c30;border-color:#0b80c7;color:#26ed45;font-weight:700}
section[data-section="0"] .approvedAdditional{background:#061627;color:#c4dfec;border-color:#146798}
.appTabs{background:linear-gradient(180deg,#06182b,#020c19)!important;border-color:#156db0!important}
.appTabs button{color:#d4eaff!important;border:1px solid #123d62!important;background:#041325!important}
.appTabs button.on{background:linear-gradient(160deg,#1378ff,#0351d7)!important;color:#fff!important;
 border-color:#18b5ff!important;box-shadow:inset 0 0 11px #64d4ff88,0 0 6px #0576ff7c!important}
.appTabs button .uiIcon{color:#40d4ff}
.appTabs button.on .uiIcon{color:#fff}

section[data-section="0"] .approvedRigPhoto{display:block;width:100%;max-height:345px;object-fit:contain;background:#031021}

/* One set of four approved sensor icons across Home and Temperature pages. */
section[data-section="1"] .approvedSensorIcon{display:flex;justify-content:center;align-items:center;flex:0 0 43px;width:43px;height:52px}
section[data-section="1"] .approvedSensorIcon .busaRefTempIcon{display:block;width:41px;height:51px;filter:drop-shadow(0 0 4px #00c4ffa8)}
section[data-section="1"] .approvedSensor{background:linear-gradient(140deg,#071d34,#041224);border-color:#1979b8}
section[data-section="1"] .approvedSensorLive{color:#32f03e!important;font-variant-numeric:tabular-nums}
section[data-section="1"] .approvedSensorHead{min-height:83px}
</style></head><body>
<header class="appHeader"><div class="headerRow"><div>
<div class="brandSmall">BUSA MONITOR · №001</div><h1>БУСА-1М <span class="brandSub">Монитор</span></h1></div>
<span class="readOnlyLabel">● МОНИТОРИНГ</span></div>
<div id="clockDisplay" class="clockBar" aria-live="off">Время синхронизируется…</div>
<div class="netIndicators" role="status" aria-label="Три подключения БУСА">
<span class="netPill"><span id="internetLed" class="netDot"></span>Wi-Fi <span id="internetLabel" class="netText">ОТКЛ</span></span>
<span class="netPill"><span id="googleLed" class="netDot"></span>Google <span id="googleLabel" class="netText">ОТКЛ</span></span>
<span class="netPill"><span id="localLed" class="netDot"></span>ESP32 <span id="localLabel" class="netText">ОТКЛ</span></span>
</div>
<nav class="appTabs" aria-label="Шесть разделов БУСА"><button data-page="0" class="on"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="m3 10 9-7 9 7v10H3z"/><path d="M9 20v-7h6v7"/></svg><span>Главная</span></button>
<button data-page="1"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg><span>Температуры</span></button>
<button data-page="2"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 2C8 8 5 11 5 15a7 7 0 0 0 14 0c0-4-3-7-7-13z"/></svg><span>Отбор</span></button>
<button data-page="3"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 3v18h18"/><path d="m5 17 5-6 4 3 6-9"/></svg><span>Графики</span></button>
<button data-page="4"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><rect x="5" y="3" width="14" height="18" rx="1"/><path d="M9 8h6M9 12h6M9 16h5"/></svg><span>События</span></button>
<button data-page="5"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg><span>Настройки</span></button></nav></header><main><section class="page" data-section="0">
<div class="approvedHomeTitle"><strong>БУСА-1М №001</strong><span id="mainwifi">Wi-Fi: ожидание</span></div>
<div class="approvedDashboard">
  <div class="approvedTempGrid"><div class="approvedTemp temp0"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon0" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><path d="M24 4C17 17 7 28 7 37a17 17 0 0034 0C41 28 31 17 24 4Z" fill="url(#busaIcon0)" stroke="#28cdff" stroke-width="2"/><path d="M15 35q0 12 12 13" fill="none" stroke="#dcfaff" stroke-width="2.5" stroke-linecap="round"/></svg></span><span>Вода</span></div><strong id="t0">Н/Д</strong><small id="homeSet0">ВКЛ — · ВЫКЛ —</small></div>
<div class="approvedTemp temp1"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon1" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIcon1)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></span><span>Колонна</span></div><strong id="t1">Н/Д</strong><small id="homeSet1">ВКЛ — · ВЫКЛ —</small></div>
<div class="approvedTemp temp2"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon2" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="5" y="17" width="38" height="35" rx="7" stroke="#10b9f7" stroke-width="2.5" fill="#083357"/><path d="M8 31q8-5 16 0t16 0v18H8Z" fill="url(#busaIcon2)" opacity=".92"/><path d="M14 17V9h20v8" fill="none" stroke="#55d6ff" stroke-width="3"/></svg></span><span>Куб</span></div><strong id="t2">Н/Д</strong><small id="homeSet2">ВКЛ — · ВЫКЛ —</small></div>
<div class="approvedTemp temp3"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon3" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIcon3)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></span><span>Верх колонны</span></div><strong id="t3">Н/Д</strong><small id="homeSet3">ВКЛ — · ВЫКЛ —</small></div></div>
  <div class="approvedRig"><img class="approvedRigPhoto" src="data:image/webp;base64,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" alt="Утверждённый рисунок установки БУСА-1М"><div class="approvedRigLabel">СХЕМА УСТАНОВКИ</div></div>
</div>
<div class="approvedMetrics">
  <div class="approvedMetric"><span>◉</span><div><small>Расход отбора</small><strong>Н/Д</strong><small>мл/мин · нет датчика</small></div></div>
  <div class="approvedMetric"><span>▤</span><div><small>Собрано</small><strong>Н/Д</strong><small>мл · нет датчика</small></div></div>
</div>
<div class="approvedStatus"><span>● Мониторинг</span><span>4 звуковых реле</span></div>
<details class="card approvedAdditional"><summary>Охлаждающая вода и звуковые выходы</summary>
  <div class="row">YF-S401 <b>GPIO15</b></div><div class="row">Импульсы <b id="pulses">—</b></div>
  <div class="row">Расход воды (не калиброван) <b id="flow">—</b></div>
  <p class="hint">Датчик на линии охлаждающей воды, а не на линии отбора. Его расчётный расход не подтверждён калибровкой.</p>
  <div class="relayGrid"><span>K1 <b id="soundStatus0">ВЫКЛ</b></span><span>K2 <b id="soundStatus1">ВЫКЛ</b></span><span>K3 <b id="soundStatus2">ВЫКЛ</b></span><span>K4 <b id="soundStatus3">ВЫКЛ</b></span></div>
  <p class="hint">К1–К4 используются только для звуковой сигнализации.</p>
</details></section>
<section class="page" data-section="1" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg><strong>ТЕМПЕРАТУРЫ</strong></div>
<div class="approvedSensorList"><div class="approvedSensor temp0">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail0" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><path d="M24 4C17 17 7 28 7 37a17 17 0 0034 0C41 28 31 17 24 4Z" fill="url(#busaIconDetail0)" stroke="#28cdff" stroke-width="2"/><path d="M15 35q0 12 12 13" fill="none" stroke="#dcfaff" stroke-width="2.5" stroke-linecap="round"/></svg></div><div class="approvedSensorText"><strong>Вода</strong><small id="detailSet0">Пороги не установлены</small></div><strong id="tempLive0" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary>Настроить пороги звукового сигнала <span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn0">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn0"></div>
    <div><label for="tempOff0">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff0"></div></div>
  </details></div>
<div class="approvedSensor temp1">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail1" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIconDetail1)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></div><div class="approvedSensorText"><strong>Колонна</strong><small id="detailSet1">Пороги не установлены</small></div><strong id="tempLive1" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary>Настроить пороги звукового сигнала <span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn1">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn1"></div>
    <div><label for="tempOff1">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff1"></div></div>
  </details></div>
<div class="approvedSensor temp2">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail2" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="5" y="17" width="38" height="35" rx="7" stroke="#10b9f7" stroke-width="2.5" fill="#083357"/><path d="M8 31q8-5 16 0t16 0v18H8Z" fill="url(#busaIconDetail2)" opacity=".92"/><path d="M14 17V9h20v8" fill="none" stroke="#55d6ff" stroke-width="3"/></svg></div><div class="approvedSensorText"><strong>Куб</strong><small id="detailSet2">Пороги не установлены</small></div><strong id="tempLive2" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary>Настроить пороги звукового сигнала <span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn2">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn2"></div>
    <div><label for="tempOff2">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff2"></div></div>
  </details></div>
<div class="approvedSensor temp3">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail3" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIconDetail3)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></div><div class="approvedSensorText"><strong>Верх колонны</strong><small id="detailSet3">Пороги не установлены</small></div><strong id="tempLive3" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary>Настроить пороги звукового сигнала <span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn3">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn3"></div>
    <div><label for="tempOff3">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff3"></div></div>
  </details></div></div>
<div class="card"><button id="temperatureSave" type="button" class="full">Сохранить пороги</button><p id="temperatureSaveResult" class="hint"></p></div></section>
<section class="page" data-section="2" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 2C8 8 5 11 5 15a7 7 0 0 0 14 0c0-4-3-7-7-13z"/></svg><strong>ОТБОР ЖИДКОСТИ</strong></div>
<div class="approvedCollectionMetrics"><div class="approvedCollectionCell"><small>Текущий расход</small><strong>НЕТ ДАТЧИКА</strong><small>мл/мин</small></div>
<div class="approvedCollectionCell"><small>Собрано жидкости</small><strong>Н/Д</strong><small>мл</small></div></div>
<div class="card approvedGraphCard"><h2>График расхода (мл/мин)</h2><div class="approvedGraphPlaceholder"><div class="approvedGridLines"></div><span>Нет датчика отбора</span></div>
</div>
<details class="card approvedAdditional"><summary>Данные охлаждающей воды</summary>
<div class="row">Расход, л/мин (оценка) <b id="flow2">—</b></div><div class="row">Объём, л (оценка) <b id="total">—</b></div>
</details></section>
<section class="page" data-section="3" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 3v18h18"/><path d="m5 17 5-6 4 3 6-9"/></svg><strong>ГРАФИКИ</strong></div>
<div class="card approvedBlackGraph"><div class="settingsTabs approvedGraphTabs"><button type="button" data-graph-tft="0" class="on">Температуры</button>
<button type="button" data-graph-tft="1">Вода и отбор</button></div>
<div class="graphPanel" data-graph-panel="0"><h3>Т1–Т4 · отдельные линии</h3>
<canvas id="graphT" class="approvedLaneCanvas" width="600" height="420" aria-label="Четыре отдельных графика T1, T2, T3 и T4 сверху вниз"></canvas>
</div>
<div class="graphPanel" data-graph-panel="1" hidden><h3>Вода и отбор · раздельно</h3>
<canvas id="graphF" class="approvedWaterCanvas" width="600" height="280" aria-label="Охлаждающая вода сверху, отбор снизу"></canvas>
</div>
</div></section>
<section class="page" data-section="4" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><rect x="5" y="3" width="14" height="18" rx="1"/><path d="M9 8h6M9 12h6M9 16h5"/></svg><strong>СОБЫТИЯ</strong></div>
<div class="card"><div class="eventFilters"><button type="button" class="on" data-event="all">Все</button><button type="button" data-event="warn">Предупр.</button><button type="button" data-event="alarm">Аварии</button><button type="button" data-event="system">Система</button></div>
<div id="events" class="approvedEvents">Ожидание...</div></div></section>
<section class="page" data-section="5" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg><strong>НАСТРОЙКИ</strong></div>
<div class="card approvedSettingsMain" id="settingsMenu"><button type="button" data-stab-btn="0" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg></span><span>Температура</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="1" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M2 8c6-5 14-5 20 0M5 12c4-4 10-4 14 0M8 16c2-2 6-2 8 0"/><circle cx="12" cy="20" r="1" fill="currentColor"/></svg></span><span>Wi-Fi</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="2" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg></span><span>Экран</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="3" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M7 19h11a4 4 0 0 0 .5-8A6.5 6.5 0 0 0 6 11a4 4 0 0 0 1 8z"/></svg></span><span>Google</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="5" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 3v12m-5-5 5 5 5-5M4 18v3h16v-3"/></svg></span><span>Обновление ПО</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="6" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 18h4l3-8 4 6 3-4h4"/><circle cx="12" cy="12" r="10"/></svg></span><span>Диагностика</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="4" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="10"/><path d="M12 11v6M12 7h.01"/></svg></span><span>О системе</span><span class="approvedArrow">›</span></button></div>
<button type="button" id="settingsBack" hidden class="approvedBack">‹ Все настройки</button>
<div class="card settingCard" data-stab="0"><h2>Калибровка температуры</h2>
<p class="hint">Поправки показаний T1–T4, шаг 0,1 °C, от −10 до +10 °C. Это НЕ пороги звуковых сигналов.</p>
<div class="row"><label for="calOffset0">T1 · Вода</label><input id="calOffset0" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset1">T2 · Колонна</label><input id="calOffset1" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset2">T3 · Куб</label><input id="calOffset2" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset3">T4 · Верх колонны</label><input id="calOffset3" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<button id="tempCalSave" type="button" class="full">Сохранить поправки</button>
<p id="tempCalResult" role="status" class="hint">—</p></div>

<div class="card settingCard" data-stab="1" hidden><h2>Wi-Fi</h2>
<div class="busaWifiMode"><h3>ESP32 · локальная сеть</h3>
<div id="espAPStatus">Проверка сети ESP32…</div>
<p class="hint">Точка доступа БУСА для локального просмотра и настройки порогов звуковых оповещений.</p></div>
<div class="busaWifiMode"><h3>Wi-Fi · роутер с интернетом</h3>
<div id="wstatus">Проверка подключения...</div><button type="button" class="full" id="scan">Найти сети Wi-Fi 2,4 ГГц</button>
<label for="net">Доступные сети Wi-Fi</label><select id="net" required><option value="">Выберите сеть из списка</option></select>
<p class="hint">Выбери найденную сеть из списка, введи пароль и нажми «Сохранить и подключить».</p>
<label for="pass">Пароль Wi-Fi</label><input id="pass" type="password" minlength="8" maxlength="63" placeholder="Введите пароль самостоятельно">
<button type="button" class="full" id="save">Сохранить и подключить</button>
<p id="response" role="status"></p><p class="hint">Настройка доступна только в локальной сети ESP32. Пароль хранится на контроллере.</p></div></div>

<div class="card settingCard" data-stab="2" hidden><h2>Экран</h2>
<p class="hint">Три темы: светлая, тёмная и системная с утверждёнными рисунками. Драйвер и распиновка не меняются.</p>
<label for="themeModeSelect">Тема TFT</label><select id="themeModeSelect">
<option value="0">Светлая</option><option value="1">Тёмная</option>
<option value="2">Системная · утверждённые рисунки БУСА</option>
</select>
<button type="button" id="tftTheme" class="full">Применить и сохранить тему</button>
<p id="tftThemeStatus" class="hint">—</p></div>

<div class="card settingCard" data-stab="3" hidden><h2>Google · телеметрия БУСА</h2>
<div class="row">Состояние канала <b id="googleState">НЕ НАСТРОЕН</b></div>
<p class="hint">БУСА передаёт измерения и диагностику в Google Таблицу. Настройка подключения сохранена.</p>
<label for="googleUrl">Адрес Apps Script (/exec)</label>
<input id="googleUrl" type="url" placeholder="https://script.google.com/macros/s/.../exec">
<label for="googleToken">Секрет устройства (64 символа)</label>
<input id="googleToken" type="password" autocomplete="off" placeholder="Не пересылайте секрет в чат">
<button id="googleSave" type="button" class="full">Сохранить подключение Google</button>
<button id="googleStart" type="button" class="full">Подключить Google (сохранённые настройки)</button>
<button id="googleTest" type="button" class="full">Проверить отправку диагностики</button>
<button id="googleStop" type="button" class="full">Отключить передачу в Google</button>
<p id="googleResult" role="status" class="hint">—</p></div>

<div class="card settingCard" data-stab="4" hidden><h2>О системе</h2>
<div class="row">Подключение к Wi-Fi <b id="aboutWifi">—</b></div>
<div class="row">Передача в Google <b id="aboutGoogle">—</b></div>
<div class="row">Прошивка <b id="aboutFirmware">—</b></div>
<div class="row">Время работы <b id="aboutUptime">—</b></div></div>

<div class="card settingCard" data-stab="5" hidden><h2>Обновление ПО</h2>
<div class="row">Установленная прошивка <b id="otaFirmware">—</b></div>
<p class="hint">Обновление только локально по Wi-Fi БУСА. Выбирайте APP_OTA.bin, настройки и раскладку клавиш не стирайте.</p>
<h3>Обновление прошивки</h3>
<p class="hint">Для OTA выбирай APP_OTA.bin, подключившись к Wi-Fi БУСА. Не стирай NVS — раскладка клавиатуры и настройки сохраняются.</p>
<input type="file" accept=".bin" id="firmwareFile">
<button type="button" class="full" id="manualUpdate">Выбрать и обновить</button>
<p id="updateResult" role="status">Ожидание подтверждения</p></div>
<div class="settingCard" data-stab="6" hidden><div class="card"><h2>Диагностика</h2>
<div class="settingsTabs diagTabs">
<button type="button" data-diag-btn="screen" class="on">Экран</button>
<button type="button" data-diag-btn="keys">Клавиатура</button>
<button type="button" data-diag-btn="relays">Реле</button>
<button type="button" data-diag-btn="temps">Датчики</button>
<button type="button" data-diag-btn="wifi">Wi-Fi / Google</button>
<button type="button" data-diag-btn="esp">ESP32</button>
<button type="button" data-diag-btn="flow">Расходомер</button>
</div></div>
<div class="card" data-diag-panel="screen"><h2>Экран TFT</h2>
<div class="row">Состояние <b id="displayState">—</b></div>
<div class="row">Контроллер <b id="tft">—</b></div>
<p class="hint">ST7789 инициализируется программой; правильность изображения проверяется визуально.</p>
<div class="grid"><button id="screenSleep" type="button">Погасить экран</button><button id="screenWake" type="button">Разбудить экран</button></div>
<label for="tftDriver">Контроллер</label><select id="tftDriver"><option value="1">ST7789 320×240</option></select>
<label for="tftRotation">Ориентация</label><select id="tftRotation"><option value="1">Горизонтальная · 1</option></select>
<button id="tftSave" type="button" class="full">Сохранить профиль TFT</button>
<button id="tftTest" type="button" class="full">Тест изображения</button>
<p id="tftProfileStatus" class="hint">—</p></div>
<div class="card" data-diag-panel="keys" hidden><h2>Клавиатура 5×4</h2>
<div class="row">PCF8575 <b id="pcf">—</b></div>
<div class="row">Состояние <b id="keyState">—</b></div>
<div class="row">Следующая клавиша <b id="keyExpected">—</b></div>
<div class="row">Контакты <b id="keyRaw">—</b></div>
<p id="keyInfo" class="hint">—</p>
<pre id="diagKeyboard" style="white-space:pre-wrap;word-break:break-word">Ожидание PCF8575...</pre>
<pre id="diagI2c" style="white-space:pre-wrap;word-break:break-word">Ожидание I²C...</pre>
<button id="keyStart" type="button" class="full">Начать обучение 20 клавиш</button>
<button id="keyNext" type="button" class="full">Считать и далее →</button>
<div class="grid"><button id="keyUndo" type="button">Назад на шаг</button>
<button id="keyCancel" type="button">Отмена</button></div></div>
<div class="card" data-diag-panel="relays" hidden><h2>Звуковые реле K1–K4</h2>
<pre id="diagRelay" style="white-space:pre-wrap;word-break:break-word">Ожидание статуса...</pre>
<p class="hint">Отчёт показывает команды реле, а не физический отклик звуковых оповещателей.</p></div>
<div class="card" data-diag-panel="temps" hidden><h2>Датчики T1–T4</h2>
<pre id="diagOneWire" style="white-space:pre-wrap;word-break:break-word">Ожидание DS18B20...</pre></div>
<div class="card" data-diag-panel="wifi" hidden><h2>Wi-Fi / Google</h2>
<pre id="diagNetwork" style="white-space:pre-wrap;word-break:break-word">Ожидание подключения...</pre></div>
<div class="card" data-diag-panel="esp" hidden><h2>ESP32-S3</h2>
<div class="row">Прошивка <b id="ver">—</b></div>
<div class="row">Время работы <b id="uptime">—</b></div>
<pre id="diagOther" style="white-space:pre-wrap;word-break:break-word">Ожидание системы...</pre>
<h3>Диагностический отчёт</h3>
<textarea id="diagReport" rows="9" readonly style="width:100%;font:11px ui-monospace,monospace;white-space:pre;overflow:auto">Ожидание диагностики...</textarea>
<button type="button" id="diagCopy" class="full">Скопировать отчёт всех модулей</button>
<p id="diagCopyResult" class="hint">—</p>
</div>
<div class="card" data-diag-panel="flow" hidden><h2>Расходомер охлаждающей воды</h2>
<pre id="diagFlow" style="white-space:pre-wrap;word-break:break-word">Ожидание импульсов...</pre>
<p class="hint">Пока не откалиброван; значения расхода и объёма оценочные.</p></div>
</div>
</section>
</main>
<script>
const get=id=>document.getElementById(id),logs=[],tlog=[],flog=[],tlog4=[[],[],[],[]],waterPulseLog=[];
let busaGraphData={temperature:[[],[],[],[]],water_pulses:[]},busaGraphFetchedAt=0,busaGraphLoading=false;
let lastWaterPulse=null,themeInitialized=false,temperatureThresholdsInitialized=false,calInitialized=false;
let active=0,connected=false,pollBusy=false,statusFailures=0,settingsActive=-1,lastLocalNav=0,lastClockPush=0,tftProfileLoaded=false;
let eventCategory="all",eventRecords=[];
function networkLeds(info,station,google){
 const wifi=station?.connected===true,local=info?.local_client===true;
 // 302 means Apps Script accepted the POST redirect; do not treat "configured" as proof.
 const cloud=google?.configured===true&&info?.internet===true&&[200,302].includes(google?.http);
 get('internetLed').classList.toggle('internetOn',wifi);
 get('googleLed').classList.toggle('googleOn',cloud);
 get('localLed').classList.toggle('localOn',local);
 get('internetLabel').textContent=wifi?'ПОДКЛ':'ОТКЛ';
 get('googleLabel').textContent=cloud?'ПОДКЛ':'ОТКЛ';
 get('localLabel').textContent=local?'ПОДКЛ':'ОТКЛ';
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
    const r=await fetch('/api/time/sync',{method:'POST',body});
    if(r.ok)lastClockPush=Date.now();
  }catch(e){}
}
function drawEvents(){
 const filtered=eventRecords.filter(e=>eventCategory==='all'||e.kind===eventCategory);
 const holder=get('events');holder.replaceChildren();
 if(!filtered.length){holder.textContent='Событий выбранного типа нет';return;}
 filtered.forEach(e=>{
  const row=document.createElement('div');row.className='eventRow '+e.kind;
  const dot=document.createElement('span');dot.className='eventDot';
  const label=document.createElement('span');label.textContent=e.text;
  row.append(dot,label);holder.append(row);
 });
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
document.querySelectorAll('[data-diag-btn]').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('[data-diag-btn]').forEach(x=>x.classList.toggle('on',x===b));
 document.querySelectorAll('[data-diag-panel]').forEach(x=>x.hidden=x.dataset.diagPanel!==b.dataset.diagBtn);
});
function page(n,send=true){
 const inDiag=n===6,visible=inDiag?5:n;
 active=visible;
 document.querySelectorAll('[data-section]').forEach(e=>e.hidden=Number(e.dataset.section)!==visible);
 document.querySelectorAll('.appTabs button').forEach(e=>e.classList.toggle('on',Number(e.dataset.page)===visible));
 if(inDiag)settingsTab(6,false);
 else if(visible===5&&send)settingsTab(-1,false);
 if(send){lastLocalNav=Date.now();fetch('/api/page?n='+(n+1)).catch(()=>{});}
 if(visible===3){graphs();refreshGraphHistory(true);}
}
function settingsTab(n,send=true){
 settingsActive=n;
 get('settingsMenu').hidden=n!==-1;
 get('settingsBack').hidden=n===-1;
 document.querySelectorAll('.settingCard').forEach(e=>e.hidden=Number(e.dataset.stab)!==n);
 document.querySelectorAll('[data-stab-btn]').forEach(e=>e.classList.toggle('on',Number(e.dataset.stabBtn)===n));
 if(send){
  lastLocalNav=Date.now();
  if(n===6)fetch('/api/page?n=7').catch(()=>{});
  else{
   fetch('/api/page?n=6').then(()=>{
     if(n>=0&&n<=4)return fetch('/api/settings-tab?n='+n);
   }).catch(()=>{});
  }
 }
}
get('settingsBack').onclick=()=>{
 settingsTab(-1,false);lastLocalNav=Date.now();
 fetch('/api/page?n=6').catch(()=>{});
};
document.querySelectorAll('[data-stab-btn]').forEach(b=>b.onclick=()=>settingsTab(Number(b.dataset.stabBtn)));
function selectGraphView(n,send=true){
 if(n<0||n>1)return;
 document.querySelectorAll('[data-graph-tft]').forEach(c=>c.classList.toggle('on',Number(c.dataset.graphTft)===n));
 document.querySelectorAll('[data-graph-panel]').forEach(p=>p.hidden=Number(p.dataset.graphPanel)!==n);
 graphs();
 if(send){lastLocalNav=Date.now();fetch('/api/graph-view?view='+n).catch(()=>{});}
}
document.querySelectorAll('[data-graph-tft]').forEach(b=>b.onclick=()=>{
 selectGraphView(Number(b.dataset.graphTft));refreshGraphHistory(true);
});
async function keyCommand(action){
 try{
   let r=await fetch('/api/keypad/'+action,{method:'POST',body:new URLSearchParams({confirm:'YES'})});
   get('keyInfo').textContent=await r.text();
 }catch(e){get('keyInfo').textContent='Ошибка: '+e.message}
}
async function setScreenPower(action){
 try{
  const resp=await fetch('/api/tft/screen',{method:'POST',body:new URLSearchParams({action})});
  if(!resp.ok)throw new Error(await resp.text());
  get('displayState').textContent=action==='wake'?'РАБОТАЕТ':'ЭКРАН ПОГАШЕН';
 }catch(e){get('displayState').textContent='ОШИБКА: '+e.message}
}
get('screenSleep').onclick=()=>setScreenPower('sleep');
get('screenWake').onclick=()=>setScreenPower('wake');
get('keyStart').onclick=()=>{if(confirm('Обучить клавиатуру, начиная с F1?'))keyCommand('start')};
get('keyNext').onclick=()=>keyCommand('next');
get('tftSave').onclick=async()=>{
 if(!confirm('Сохранить профиль TFT и перезапустить ESP32? Звуковые реле временно выключатся на перезагрузке.'))return;
 try{
  const r=await fetch('/api/tft/profile',{method:'POST',body:new URLSearchParams({confirm:'YES',profile:get('tftDriver').value,rotation:get('tftRotation').value})});
  get('tftProfileStatus').textContent=await r.text();
 }catch(e){get('tftProfileStatus').textContent='Ошибка: '+e.message}
};
get('tftTest').onclick=async()=>{
 try{
  const r=await fetch('/api/tft/test',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
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
get('tftTheme').onclick=async()=>{
 try{
  const mode=get('themeModeSelect').value;
  const r=await fetch('/api/tft/theme',{method:'POST',body:new URLSearchParams({confirm:'YES',mode})});
  get('tftThemeStatus').textContent=await r.text();
  if(r.ok){document.documentElement.dataset.theme=mode==='1'?'dark':mode==='2'?'system':'light';if(active===3)graphs();}
 }catch(e){get('tftThemeStatus').textContent='Ошибка: '+e.message}
};
get('tempCalSave').onclick=async()=>{
 const button=get('tempCalSave'),data=new URLSearchParams({confirm:'YES'});
 for(let i=0;i<4;i++){
  const raw=get('calOffset'+i).value.trim(),n=Number(raw);
  if(!raw||!Number.isFinite(n)||n< -10||n>10||Math.abs(n*10-Math.round(n*10))>0.02){
   get('tempCalResult').textContent='T'+(i+1)+': от -10 до +10 °C с шагом 0,1';return;
  }
  data.set('offset'+i,String(n));
 }
 button.disabled=true;
 try{
  const r=await fetch('/api/temperature/calibration',{method:'POST',body:data});
  if(!r.ok)throw Error(await r.text());
  get('tempCalResult').textContent=await r.text();calInitialized=false;
 }catch(e){get('tempCalResult').textContent='Ошибка: '+e.message}
 button.disabled=false;
};
get('keyUndo').onclick=()=>keyCommand('undo');
get('keyCancel').onclick=()=>keyCommand('cancel');
document.querySelectorAll('nav button').forEach(e=>e.onclick=()=>page(Number(e.dataset.page)));
// One controller history feeds the TFT and iPhone: no invented collection values.
function chartFrame(ctx,w,h){ctx.fillStyle='#000';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#2774a5';ctx.lineWidth=2;ctx.strokeRect(1,1,w-2,h-2);}
function chartLine(ctx,values,color,x,y,w,h,zero=false,slots=48){
 const a=values.filter(v=>Number.isFinite(v));if(!a.length)return;
 let low=zero?0:Math.min(...a),high=Math.max(...a);
 if(zero)high=Math.max(1,high);else{let p=Math.max(.2,(high-low)*.22);low-=p;high+=p;}
 if(high<=low)high=low+1;ctx.strokeStyle=color;ctx.lineWidth=3;ctx.lineJoin='round';ctx.beginPath();
 let drawing=false;values.forEach((v,i)=>{
  if(!Number.isFinite(v)){drawing=false;return;}
  let xx=x+i*w/Math.max(1,slots-1);
  let yy=y+h-3-Math.min(1,Math.max(0,(v-low)/(high-low)))*(h-6);
  if(!drawing){ctx.moveTo(xx,yy);drawing=true}else ctx.lineTo(xx,yy);
 });ctx.stroke();
}
function graphAllTemperatures(){
 const c=get('graphT'),x=c.getContext('2d');chartFrame(x,c.width,c.height);
 const names=['Т1 · Вода','Т2 · Колонна','Т3 · Куб','Т4 · Верх'];
 const colors=['#36c5f3','#ffac56','#ff6965','#5ee897'],series=busaGraphData.temperature||[[],[],[],[]];
 x.textBaseline='middle';
 for(let k=0;k<4;k++){
  const y=8+k*102,vals=series[k]||[],good=vals.filter(Number.isFinite);
  if(k){x.strokeStyle='#28608b';x.lineWidth=2;x.beginPath();x.moveTo(8,y-3);x.lineTo(592,y-3);x.stroke();}
  x.font='16px sans-serif';x.fillStyle=colors[k];x.textAlign='left';x.fillText(names[k],14,y+16);
  x.textAlign='right';x.fillStyle='#e4f5ff';x.fillText(good.length?good[good.length-1].toFixed(1)+' °C':'Н/Д',582,y+16);x.textAlign='left';
  x.strokeStyle='#17384f';x.lineWidth=1;x.strokeRect(12,y+34,576,61);
  chartLine(x,vals,colors[k],17,y+38,566,53);
  if(good.length<2){x.fillStyle='#799eb8';x.font='12px sans-serif';x.fillText('Ожидание измерений',22,y+72);}
 }
}
function graphWaterAndCollection(){
 const c=get('graphF'),x=c.getContext('2d');chartFrame(x,c.width,c.height);
 x.textBaseline='middle';x.textAlign='left';x.font='18px sans-serif';x.fillStyle='#58d8ff';x.fillText('ВОДА · YF-S401',15,23);
 x.font='13px sans-serif';x.fillStyle='#bce6fb';x.textAlign='right';x.fillText('Импульсы за 2 с · без калибровки',585,23);x.textAlign='left';
 x.strokeStyle='#1a3f57';x.lineWidth=1;x.strokeRect(12,40,576,96);
 chartLine(x,busaGraphData.water_pulses||[],'#37bcff',17,43,566,88,true,40);
 x.strokeStyle='#28608b';x.lineWidth=2;x.beginPath();x.moveTo(8,146);x.lineTo(592,146);x.stroke();
 x.font='18px sans-serif';x.fillStyle='#ffae65';x.fillText('ОТБОР',15,173);
 x.fillStyle='#b6d6e9';x.font='17px sans-serif';x.fillText('НЕТ ДАТЧИКА ОТБОРА',20,215);
 x.font='13px sans-serif';x.fillText('Измерения отсутствуют — линия не строится',20,245);
}
function graphs(){graphAllTemperatures();graphWaterAndCollection();}
async function refreshGraphHistory(force=false){
 if(busaGraphLoading||(!force&&Date.now()-busaGraphFetchedAt<4500))return;
 busaGraphLoading=true;busaGraphFetchedAt=Date.now();
 try{
  const r=await fetch('/api/graph-history',{cache:'no-store'});
  if(!r.ok)throw Error('HTTP '+r.status);
  const j=await r.json();
  if(j.ok&&Array.isArray(j.temperature)&&Array.isArray(j.water_pulses)){
   busaGraphData=j;if(active===3)graphs();
  }
 }catch(e){}finally{busaGraphLoading=false;}
}
async function update(){if(pollBusy)return;pollBusy=true;try{let r=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(6000)});if(!r.ok)throw Error(r.status);let s=await r.json();if(!connected)message('Связь с ESP32 восстановлена');connected=true;statusFailures=0;
networkLeds(s.network_indicators,s.station,s.google);if(Date.now()-lastLocalNav>1800){
 const physicalPage=(s.ui_page||1)-1;
 if(typeof s.ui_page==='number'&&(physicalPage===6?active!==5||settingsActive!==6:physicalPage!==active))page(physicalPage,false);
 if(physicalPage===5&&settingsActive>=0&&settingsActive<=4&&typeof s.ui_subtab==='number'&&s.ui_subtab!==settingsActive)settingsTab(s.ui_subtab,false);
 if(physicalPage===3&&typeof s.graph_view==='number'){
   const currentGraph=Number(document.querySelector('[data-graph-tft].on')?.dataset.graphTft||0);
   if(currentGraph!==s.graph_view)selectGraphView(s.graph_view,false);
 }
}let names=['Вода','Колонна','Куб','Верх колонны'];
s.temperatures.forEach((t,i)=>{
 get('t'+i).textContent=t.c===null?'Н/Д':Number(t.c).toFixed(1)+' °C';
 get('tempLive'+i).textContent=get('t'+i).textContent;
 const a=tlog4[i];a.push(t.c===null?null:Number(t.c));if(a.length>80)a.shift();
 if(i===0){tlog.push(t.c===null?null:Number(t.c));if(tlog.length>80)tlog.shift();}
});
if(s.sound_alerts?.channels?.length===4 && s.sound_alerts?.buzzer?.sensors?.length===2){
 const c=s.sound_alerts.channels,b=s.sound_alerts.buzzer.sensors;
 const pairs=[[b[0].on_c,b[0].off_c],[b[1].on_c,b[1].off_c],[c[0].on_c,c[0].off_c],[c[3].on_c,c[3].off_c]];
 pairs.forEach((p,i)=>{
  if(!temperatureThresholdsInitialized){get('tempOn'+i).value=p[0]??'';get('tempOff'+i).value=p[1]??'';}
  get('homeSet'+i).textContent='Зад: '+(p[0]??'—')+'°';
   get('detailSet'+i).textContent='ВКЛ '+(p[0]??'—')+'° · ВЫКЛ '+(p[1]??'—')+'°';
 });
 temperatureThresholdsInitialized=true;
}
if(s.sound_alerts?.channels?.length===4)
 s.sound_alerts.channels.forEach((ch,i)=>{get('soundStatus'+i).textContent=ch.sound_on?(ch.timed?'ЗВУК '+ch.remaining_seconds+'с':'ЗВУК'):(!ch.enabled?'ВЫКЛ':ch.reason==='SETPOINTS_REQUIRED'?'НУЖНЫ ПОРОГИ':'ГОТОВ')});
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?'НЕТ ДАННЫХ':f.toFixed(2);get('flow2').textContent=get('flow').textContent;get('pulses').textContent=s.water.pulses;get('total').textContent=Number(s.water.total_l_estimate).toFixed(3);
flog.push(f);if(flog.length>80)flog.shift();
const currentPulse=Number(s.water.pulses);
if(Number.isFinite(currentPulse)){
 let delta=lastWaterPulse===null?null:Math.max(0,currentPulse-lastWaterPulse);
 lastWaterPulse=currentPulse;waterPulseLog.push(delta);
 if(waterPulseLog.length>80)waterPulseLog.shift();
}
let ss=s.station.connected?'ПОДКЛЮЧЕНО: '+s.station.ssid+' · '+s.station.ip:'Нет подключения. Нажмите «Найти сети Wi-Fi» и выберите сеть.';
get('wstatus').textContent=s.sta_switch?.connecting?("Проверяю Wi-Fi... "+(s.sta_switch.message||"")):
  (s.sta_switch?.result===-1?(s.sta_switch.message+" · Локальная сеть доступна"):ss);
get('espAPStatus').textContent=s.network_indicators?.local_client?'ESP32: ПОДКЛ · '+(s.ap?.ip||'192.168.4.1'):(s.ap?.enabled?'ESP32: сеть доступна · '+(s.ap?.ip||'192.168.4.1'):'ESP32: ОТКЛ');
get('mainwifi').textContent='Wi-Fi '+(s.station.connected?'ПОДКЛ':'ОТКЛ')+' · Google '+(s.google?.configured&&s.network_indicators?.internet?'ПОДКЛ':'ОТКЛ')+' · ESP32 '+(s.network_indicators?.local_client?'ПОДКЛ':'ОТКЛ');
if(s.station.connected&&s.station.ssid){
 let select=get('net'),found=Array.from(select.options).some(o=>o.value===s.station.ssid);
 if(!found)select.add(new Option(s.station.ssid+' (подключена)',s.station.ssid));
 if(!select.value)select.value=s.station.ssid;
}
const cs=s.google;
get('displayState').textContent=s.display_sleep?'ПОГАШЕН':'РАБОТАЕТ';
if(!themeInitialized && typeof s.tft_theme_mode==='number'){
  get('themeModeSelect').value=String(s.tft_theme_mode);
  document.documentElement.dataset.theme=s.tft_theme_mode===1?'dark':s.tft_theme_mode===2?'system':'light';
  themeInitialized=true;
}
if(!calInitialized&&Array.isArray(s.calibration_offsets)){
  s.calibration_offsets.forEach((v,i)=>{if(i<4)get('calOffset'+i).value=Number(v).toFixed(1)});
  calInitialized=true;
}
get('aboutWifi').textContent=s.station.connected?'Подключён':'Нет связи';
get('aboutGoogle').textContent=!s.google.configured?'Не настроена':!s.google.enabled?'Отключена':s.google.state===6?'Ошибка':'Подключена';
get('aboutFirmware').textContent=s.firmware;
get('otaFirmware').textContent=s.firmware;
get('aboutUptime').textContent=Math.floor(s.uptime_ms/3600000)+' ч '+Math.floor((s.uptime_ms%3600000)/60000)+' мин';
get('googleState').textContent=!cs.configured?'НЕ НАСТРОЕН':!cs.enabled?'ОТКЛЮЧЕНО (ДАННЫЕ СОХРАНЕНЫ)':cs.state===5?'ОТВЕТ ПОЛУЧЕН':cs.state===4?'ПЕРЕДАНО, ПРОВЕРЬТЕ ТАБЛИЦУ':cs.state===3?'ОТПРАВКА':cs.state===2?'ВРЕМЯ':cs.state===1?'НЕТ ИНТЕРНЕТА':cs.state===6?'ОШИБКА':cs.state===7?'ОТКЛОНЕНО':'ОЖИДАНИЕ';
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
   get('diagI2c').textContent='I2C GPIO40/41 · клавиатура | ACK total='+d.i2c?.ack_total+
     ' | addresses='+addresses.map(a=>'0x'+Number(a).toString(16).padStart(2,'0')).join(', ')+
     '\nPCF8575: ACK='+d.pcf8575?.online+' read='+d.pcf8575?.read_ok+
     ' idle=0x'+d.pcf8575?.idle_pins_hex+
     '\nPCA9685: отключён, не опрашивается';
   get('diagNetwork').textContent='Wi-Fi STA: '+(s.station.connected?'подключен':'нет связи')+
      '\nAP: '+(s.ap?.enabled?'работает':'нет')+' · '+(s.ap?.ip||'—')+
      '\nМощность TX: '+(s.performance?.wifi_tx_power_dbm??'—')+' dBm'+
      '\nGoogle: '+(s.google?.configured?'настроен':'не настроен')+
      ' · HTTP '+s.google?.http+' · TLS '+s.google?.tls_error;
   get('diagRelay').textContent=(s.sound_alerts?.channels||[]).map(c=>
     'K'+c.channel+' GPIO'+c.gpio+' '+(c.sound_on?'ЗВУК':'ОЖИДАНИЕ')+
     ' · ВКЛ '+(c.on_c??'—')+' °C · ВЫКЛ '+(c.off_c??'—')+
     ' °C · '+c.reason).join('\n')+
     '\nЗуммер T1/T2: '+(s.sound_alerts?.buzzer?.sound_on?'ЗВУК':'ОЖИДАНИЕ');
   get('diagFlow').textContent='GPIO15 · '+d.water_flow?.status+
     '\nИмпульсы: '+d.water_flow?.pulses+
     '\nКалибровка: отсутствует'+
     '\nРасход (оценка): '+(s.water?.lpm??'—')+' л/мин';
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
     temperatures:s.temperatures,modules:s.modules,diagnostics:d,
     station_connected:s.station?.connected,ap_enabled:s.ap?.enabled,
     relay_outputs_enabled:s.relays_enabled,sound_alerts:s.sound_alerts
   },null,2);
   const report=get('diagReport');
   if(document.activeElement!==report)report.value=lastDiagnosticReport;
 }
 if(s.keypad.message)get('keyInfo').textContent=s.keypad.message;
}
get('ver').textContent=s.firmware;get('uptime').textContent=Math.floor(s.uptime_ms/60000)+' мин';get('pcf').textContent=s.modules.pcf8575?'Найден':'Не найден';get('tft').textContent=s.modules.tft_attempted?('Контроллер '+s.modules.tft_profile+', поворот '+s.modules.tft_rotation+', загрузка '+(s.modules.tft_boot_complete?'завершена':'идёт')+' (проверка изображения визуальная)'):'Запуск ещё не выполнен';if(!tftProfileLoaded){get('tftDriver').value=String(s.modules.tft_profile_id||1);get('tftRotation').value=String(s.modules.tft_rotation===undefined?1:s.modules.tft_rotation);tftProfileLoaded=true}if(active===3)refreshGraphHistory();
}catch(e){statusFailures++;if(statusFailures>=3){if(connected)message('Нет ответа ESP32: проверь локальный Wi-Fi');connected=false;networkLeds(null,'')}}finally{pollBusy=false}}
get('temperatureSave').onclick=async()=>{
 const button=get('temperatureSave'),result=get('temperatureSaveResult');
 const data=new URLSearchParams();
 for(let i=0;i<4;i++){
  const on=get('tempOn'+i).value.trim(),off=get('tempOff'+i).value.trim();
  if((!on)!==(!off)){result.textContent='Т'+(i+1)+': заполните оба значения';return;}
  if(on){
   const a=Number(on),b=Number(off);
   if(!Number.isFinite(a)||!Number.isFinite(b)||a<0||a>125||b<0||b>125||Math.abs(a-b)<0.099){
    result.textContent='Т'+(i+1)+': проверьте ВКЛ и ВЫКЛ';return;
   }
  }
  data.set('on'+i,on);data.set('off'+i,off);
 }
 button.disabled=true;result.textContent='Сохранение...';
 try{
  const r=await fetch('/api/temperature-alarms',{method:'POST',body:data});
  if(!r.ok)throw Error(await r.text());
  result.textContent='Сохранено';
  temperatureThresholdsInitialized=false;
 }catch(e){result.textContent='Ошибка: '+e.message;}
 button.disabled=false;
};
get('googleSave').onclick=async()=>{
const url=get('googleUrl').value.trim(),token=get('googleToken').value.trim();
if(!url.startsWith('https://script.google.com/macros/s/')||!url.endsWith('/exec')||!/^[a-fA-F0-9]{64}$/.test(token)){
 get('googleResult').textContent='Требуется ссылка Apps Script /exec и секрет из 64 шестнадцатеричных символов';return;
}
if(!confirm('Включить отправку показаний в свой Google Диск? Только мониторинг.'))return;
let resp=await fetch('/api/google/config',{method:'POST',body:new URLSearchParams({url,token})});
get('googleResult').textContent=await resp.text();if(resp.ok)get('googleToken').value='';
};
get('googleStart').onclick=async()=>{
  try{
    const r=await fetch('/api/google/on',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
    get('googleResult').textContent=await r.text();
  }catch(e){get('googleResult').textContent='Ошибка подключения: '+e.message}
};
get('googleTest').onclick=async()=>{
  try{
    const r=await fetch('/api/google/test',{method:'POST',
      body:new URLSearchParams({confirm:'YES'})});
    get('googleResult').textContent=await r.text();
  }catch(e){get('googleResult').textContent='Ошибка проверки: '+e.message}
};
get('googleStop').onclick=async()=>{
if(!confirm('Отключить отправку диагностических данных в Google?'))return;
const resp=await fetch('/api/google/off',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
get('googleResult').textContent=await resp.text();
};
get('manualUpdate').onclick=async function(){
  const file=get('firmwareFile').files[0];
  if(!file || !file.name.endsWith('.bin') || file.size<60000 || file.size>3340000){
    get('updateResult').textContent='Нужен APP BIN подходящего размера';return;
  }
  if(!confirm('Подтвердите ручное обновление диагностической прошивки. ESP32-S3 N16R8, звуковые реле К1–К4 временно отключаются, выбран APP BIN?'))return;
  this.disabled=true;
  try{
    const begin=await fetch('/api/ota/prepare',{method:'POST',body:new URLSearchParams({confirm:'YES'})});
    if(!begin.ok)throw Error(await begin.text());
    const grant=await begin.json(),body=new FormData();
    body.append('firmware',file,file.name);
    get('updateResult').textContent='Загрузка, не отключайте USB';
    const t0=Date.now();
    await new Promise((resolve,reject)=>{
      const xhr=new XMLHttpRequest();
      xhr.open('POST','/api/ota/upload',true);
      xhr.setRequestHeader('X-Busa-Token',grant.token);
      xhr.upload.onprogress=e=>{
        if(!e.lengthComputable)return;
        const secs=Math.max((Date.now()-t0)/1000,0.1);
        const pct=Math.min(100,Math.round(e.loaded*100/e.total));
        get('updateResult').textContent='Передача APP BIN: '+pct+'% · '+(e.loaded/1024).toFixed(0)+' КБ · '+(e.loaded/1024/secs).toFixed(1)+' КБ/с';
      };
      xhr.onload=()=>xhr.status===200?resolve(xhr.responseText):reject(Error(xhr.responseText||'HTTP '+xhr.status));
      xhr.onerror=()=>reject(Error('Wi-Fi связь прервана при передаче'));
      xhr.ontimeout=()=>reject(Error('Время передачи истекло'));
      xhr.timeout=180000;
      xhr.send(body);
    });
    get('updateResult').textContent='APP BIN записан. БУСА перезапускается';
  }catch(e){get('updateResult').textContent='Ошибка: '+e.message}
  this.disabled=false;
};
get('net').onchange=()=>{if(get('net').value)get('response').textContent='Выбрана сеть: '+get('net').value+'. Введите пароль для подключения.'};
get('scan').onclick=async function(){
 this.disabled=true;get('response').textContent='Поиск Wi-Fi 2,4 ГГц...';
 try{
  // Start only one radio scan; retry fetching its results if HTTP is delayed.
  let j=null;
  try{
   const r=await fetch('/api/wifi/scan?start=1',{cache:'no-store',signal:AbortSignal.timeout(16000)});
   j=await r.json();
   if(!r.ok)throw Error(j.error||'HTTP '+r.status);
  }catch(e){
   if(e.name!=='TimeoutError'&&e.name!=='AbortError')throw e;
   get('response').textContent='Ответ задерживается, проверяю результат поиска...';
  }
  for(let i=0;i<36 && (!j||j.scanning);i++){
   await new Promise(r=>setTimeout(r,800));
   try{
    const r=await fetch('/api/wifi/scan',{cache:'no-store',signal:AbortSignal.timeout(10000)});
    const result=await r.json();
    if(!r.ok)throw Error(result.error||'HTTP '+r.status);
    j=result;
   }catch(e){
    if(i===35)throw e;
   }
  }
  if(!j||j.scanning)throw Error('Нет ответа от ESP32 при сканировании');
  const sel=get('net');sel.innerHTML='<option value="">Выберите сеть</option>';
  const seen=new Set();(j.networks||[]).forEach(w=>{
   if(!w.ssid||seen.has(w.ssid))return;
   seen.add(w.ssid);
   sel.add(new Option(w.ssid+' ('+w.rssi+' dBm)',w.ssid));
  });
  get('response').textContent=sel.options.length>1?'Найдено сетей: '+(sel.options.length-1):
   'Сети не найдены. Проверь режим 2,4 ГГц внешней точки доступа.';
 }catch(e){
  get('response').textContent='Ошибка поиска Wi-Fi: '+e.message+'. Подключение БУСА не изменено.';
 }
 this.disabled=false;
};
get('save').onclick=async function(){let ssid=get('net').value,pass=get('pass').value;if(!ssid||ssid.length>32||pass.length<8||pass.length>63){get('response').textContent='Выбери найденную сеть и введи пароль (8–63 символа)';return}this.disabled=true;get('response').textContent='Сохраняю Wi-Fi...';
try{let r=await fetch('/api/wifi',{method:'POST',body:new URLSearchParams({ssid,pass})});get('response').textContent=await r.text();if(r.ok){get('pass').value='';message('Проверяю сеть: '+ssid+'; сохранение только после подключения')}}catch(e){get('response').textContent='Ошибка: '+e.message}this.disabled=false};
document.documentElement.dataset.theme='system';
settingsTab(-1,false);
message('Веб-приложение запущено');
showPhoneClock();setInterval(showPhoneClock,1000);
syncClock();setInterval(syncClock,300000);
update();setInterval(update,3000);
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
  Serial0.begin(115200);
  Serial.begin(115200);
  // ESP32-S3 documented maximum CPU clock is 240 MHz; do not overclock.
  // This call is idempotent when the board menu already selects 240 MHz.
  const bool cpuClockSet=setCpuFrequencyMhz(240);
  busaCpuMhz=ESP.getCpuFreqMHz();
  busaCpu240Ok=cpuClockSet && (busaCpuMhz==240);
  busaPsramDetected=psramFound();
  busaPsramBytes=busaPsramDetected?ESP.getPsramSize():0;
  busaPsramFreeBytes=busaPsramDetected?ESP.getFreePsram():0;
  Serial.printf("BUSA PERFORMANCE CPU=%luMHz target=240 result=%d PSRAM=%d total=%lu free=%lu\n",
     (unsigned long)busaCpuMhz,(int)busaCpu240Ok,(int)busaPsramDetected,
     (unsigned long)busaPsramBytes,(unsigned long)busaPsramFreeBytes);
  busaBootAt=millis();
  busaReportStage("START");
  Serial.printf("BUSA N16R8 reset_reason=%d flash=%lu free_heap=%lu\n",
       (int)esp_reset_reason(),(unsigned long)ESP.getFlashChipSize(),
       (unsigned long)ESP.getFreeHeap());
  soundInitPins(); // Set relay inputs OFF before Wi-Fi/TFT and sensor startup.
  delay(100);
  pinMode(FLOW_GPIO,INPUT); // requires an electrically verified 3.3 V logic signal
  attachInterrupt(digitalPinToInterrupt(FLOW_GPIO),flowISR,FALLING);
  Serial0.println("WATER GPIO15 interrupt attached (input only)");
  prefs.begin("busa-r04",false);
  busaRemoteAlarmInit();
  keypadBackupOpen=keypadBackupPrefs.begin("busa-key20",false);
  // User-approved ST7789 320x240 rotation 1 and software-drawn light theme.
  if(!prefs.getBool("v14lightSt",false)){
    prefs.putUChar("tft",1);
    prefs.putUChar("tftRotate",1);
    prefs.putBool("tftInvert",false);
    prefs.putBool("v14lightSt",true);
  }
  tftProfile=1;tftRotation=1;tftInverted=false;
  uint8_t themeSaved=prefs.getUChar("themeMode",255);
  tftThemeMode=themeSaved<=2?themeSaved:(prefs.getBool("tftDarkColor",false)?1:2);
  tftThemeDark=tftThemeMode==1;
  for(int i=0;i<4;i++){
    String key="tCal"+String(i);float v=prefs.getFloat(key.c_str(),0);
    tempOffset[i]=isfinite(v)&&v>=-10&&v<=10?roundf(v*10)/10:0;
  }
  startedAt=millis();
  tftProfile=1; // ST7789 only
  // Preserve original V14.7 splash-first boot; time every operation.
  busaReportStage("TFT_BEGIN");
  screenAttempted=true;
  startScreen();
  busaReportStage("TFT_READY");
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  loadKeymap();
  clockZoneKnown=prefs.getBool("clkZone",false);
  clockUtcOffsetMinutes=prefs.getInt("clkTz",0);
  if(clockUtcOffsetMinutes<(-12*60)||clockUtcOffsetMinutes>14*60)clockZoneKnown=false;
  soundLoadPrefs();
  for(int i=0;i<4;i++){
    refTemp[i]=i<2?buzzerOnC[i]:soundOnC[i];
    noticeTemp[i]=i<2?buzzerOffC[i]:soundOffC[i];
  }
  setupGoogleCloud();
  busaReportStage("WIFI_BEGIN");
  WiFi.persistent(false); // Wi-Fi driver never overwrites stored NVS.
  WiFi.setAutoReconnect(false); // No infinite retry against iPhone 12/16.
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); // Favor interactive local response over power saving
  configTime(0,0,"pool.ntp.org","time.google.com"); // NTP UTC; local offset from paired phone
  clockNtpStarted=true;
  // Private STA password must be provisioned into NVS after public GitHub build;
  // no STA password is embedded in this public source. Local AP fallback remains.
  apPassword="123456789"; // User-selected AP password, V14.7 original startup unchanged
  wifiRunning=WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str());
  // Use the supported ESP32-S3 radio power limit, not an undocumented boost.
  // Actual RF range remains antenna- and environment-dependent.
  const bool txSet=WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial0.printf("Wi-Fi configured TX <=19.5 dBm: %s\n",txSet?"OK":"FAILED");
  busaReportStage(wifiRunning?"AP_READY":"AP_FAILED");
  updateBootStage(1);
  Serial0.printf("AP=%d IP=%s SSID=BUSA-1M-001-R04 PASS=%s\n",
                 (int)wifiRunning,WiFi.softAPIP().toString().c_str(),apPassword.c_str());
  if(stationSsid.length() && stationPass.length()>=8){
    busaStageStaCredentials(stationSsid,stationPass,true);
    Serial0.println("STA: one timed boot attempt; AP remains primary");
  }else Serial0.println("STA not configured; AP-only local monitoring");
  server.on("/",HTTP_GET,[]{server.sendHeader("Cache-Control","no-store");server.send_P(200,"text/html; charset=utf-8",WEB_PAGE);});
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
  server.on("/api/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",statusJSON());
  });
  // Local AP only: T3/T4 sound relay configuration; no process actuation.
  server.on("/api/sound/config",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain","BUSA local AP only");return;}
    bool requestedEnabled[4]={};
    for(int i=0;i<4;i++){
      const String key=String("en")+i;
      if(!server.hasArg(key)){server.send(400,"text/plain","Missing sound enable");return;}
      String flag=server.arg(key);
      if(flag!="0" && flag!="1"){server.send(400,"text/plain","Invalid sound enable");return;}
      requestedEnabled[i]=flag=="1";
    }
    const char* keys[4]={"t3on","t3off","t4on","t4off"};
    float requested[4]={NAN,NAN,NAN,NAN};
    bool pairPresent[2]={false,false};
    for(int pair=0;pair<2;pair++){
      const int i=pair*2;
      if(!server.hasArg(keys[i])||!server.hasArg(keys[i+1])){
        server.send(400,"text/plain","Missing T3/T4 threshold fields");return;
      }
      const String a=server.arg(keys[i]),b=server.arg(keys[i+1]);
      if(a.isEmpty() && b.isEmpty())continue; // enabled but silent until configured
      char *endA=nullptr,*endB=nullptr;
      requested[i]=strtof(a.c_str(),&endA);
      requested[i+1]=strtof(b.c_str(),&endB);
      if(a.isEmpty()||b.isEmpty()||!endA||*endA||!endB||*endB||
         !BusaSound::valid(requested[i],requested[i+1])){
        server.send(400,"text/plain","Invalid T3/T4 ON/OFF, require 0..125 C, delta >=0.1 C");return;
      }
      pairPresent[pair]=true;
    }
    // Store new T3/T4 keys. Sound outputs remain off until changes are valid.
    soundOutputsOff();soundResetState();
    bool saved=true;
    const char* stored[4]={"sndT3On","sndT3Off","sndT4On","sndT4Off"};
    for(int pair=0;pair<2;pair++){
      for(int j=0;j<2;j++){
        const int i=pair*2+j;
        if(pairPresent[pair]){
          if(!prefs.putFloat(stored[i],requested[i]))saved=false;
        }else{
          if(prefs.isKey(stored[i]) && !prefs.remove(stored[i]))saved=false;
        }
      }
    }
    for(int i=0;i<4;i++)
      if(!prefs.putBool((String("sndRoleEnable")+i).c_str(),requestedEnabled[i]))saved=false;
    if(!saved){
      for(int i=0;i<4;i++){soundEnabled[i]=false;prefs.putBool((String("sndRoleEnable")+i).c_str(),false);}
      soundOutputsOff();
      server.send(500,"text/plain","NVS save failed, audible signals disabled");return;
    }
    soundOnC[0]=requested[0];soundOffC[0]=requested[1];
    soundOnC[1]=soundOnC[2]=soundOnC[0];
    soundOffC[1]=soundOffC[2]=soundOffC[0];
    soundOnC[3]=requested[2];soundOffC[3]=requested[3];
    for(int i=0;i<4;i++)soundEnabled[i]=requestedEnabled[i];
    server.send(200,"application/json",soundJSON(millis()));
  });

  // Independent T1/T2 monitoring buzzer. Local access only; no relay commands.
  server.on("/api/buzzer/config",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain","BUSA local AP only");return;}
    bool enabled[2]={false,false};float on[2]={NAN,NAN},off[2]={NAN,NAN};
    for(int i=0;i<2;i++){
      const String enKey=String("en")+i,onKey=String("on")+i,offKey=String("off")+i;
      if(!server.hasArg(enKey)||!server.hasArg(onKey)||!server.hasArg(offKey)){
        server.send(400,"text/plain","Missing T1/T2 buzzer fields");return;
      }
      const String flag=server.arg(enKey);
      if(flag!="0" && flag!="1"){server.send(400,"text/plain","Invalid T1/T2 enable");return;}
      enabled[i]=flag=="1";
      const String a=server.arg(onKey),b=server.arg(offKey);
      if(a.isEmpty()&&b.isEmpty())continue;
      char* endA=nullptr;char* endB=nullptr;
      on[i]=strtof(a.c_str(),&endA);off[i]=strtof(b.c_str(),&endB);
      if(a.isEmpty()||b.isEmpty()||!endA||*endA||!endB||*endB||
         !BusaSound::valid(on[i],off[i])){
        server.send(400,"text/plain","Invalid T1/T2 ON/OFF, 0..125C, delta >=0.5C");return;
      }
    }
    // Keep buzzer silent until the save completes.
    digitalWrite(BUSA_BUZZER_GPIO,buzzerPinLevel(false));buzzerSoundOn=false;
    bool saved=true;
    for(int i=0;i<2;i++){
      const String ek=String("buzE")+i,ok=String("buzOn")+i,fk=String("buzOff")+i;
      if(!prefs.putBool(ek.c_str(),false))saved=false;
      if(isfinite(on[i])){
        if(!prefs.putFloat(ok.c_str(),on[i]))saved=false;
        if(!prefs.putFloat(fk.c_str(),off[i]))saved=false;
      }else{
        if(prefs.isKey(ok.c_str())&&!prefs.remove(ok.c_str()))saved=false;
        if(prefs.isKey(fk.c_str())&&!prefs.remove(fk.c_str()))saved=false;
      }
    }
    if(saved)for(int i=0;i<2;i++)
      if(!prefs.putBool((String("buzE")+i).c_str(),enabled[i]))saved=false;
    if(!saved){
      for(int i=0;i<2;i++){buzzerEnabled[i]=false;prefs.putBool((String("buzE")+i).c_str(),false);}
      digitalWrite(BUSA_BUZZER_GPIO,buzzerPinLevel(false));buzzerSoundOn=false;
      server.send(500,"text/plain","NVS buzzer settings save failed");return;
    }
    for(int i=0;i<2;i++){buzzerEnabled[i]=enabled[i];buzzerOnC[i]=on[i];buzzerOffC[i]=off[i];
      buzzerTempChannels[i]=BusaSound::Channel();}
    server.send(200,"application/json",soundJSON(millis()));
  });
  // Single minimalist T1-T4 alarm-threshold form. Monitoring and sound only.
  server.on("/api/temperature-alarms",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain","BUSA local AP only");return;}
    float requestedOn[4]={NAN,NAN,NAN,NAN},requestedOff[4]={NAN,NAN,NAN,NAN};
    bool present[4]={false,false,false,false};
    for(int i=0;i<4;i++){
      const String onKey=String("on")+i,offKey=String("off")+i;
      if(!server.hasArg(onKey)||!server.hasArg(offKey)){
        server.send(400,"text/plain","Missing T1-T4 fields");return;
      }
      const String on=server.arg(onKey),off=server.arg(offKey);
      if(on.isEmpty()&&off.isEmpty())continue; // no threshold: no sound
      char *endOn=nullptr,*endOff=nullptr;
      requestedOn[i]=strtof(on.c_str(),&endOn);
      requestedOff[i]=strtof(off.c_str(),&endOff);
      if(on.isEmpty()||off.isEmpty()||!endOn||*endOn||!endOff||*endOff||
         !BusaSound::valid(requestedOn[i],requestedOff[i])){
        server.send(400,"text/plain","Invalid ON/OFF temperatures 0..125 C, delta >=0.1 C");return;
      }
      present[i]=true;
    }
    // Do not alter any hardware output until the full submitted form is valid.
    soundOutputsOff();soundResetState();
    const char* onKeys[4]={"buzOn0","buzOn1","sndT3On","sndT4On"};
    const char* offKeys[4]={"buzOff0","buzOff1","sndT3Off","sndT4Off"};
    bool saved=true;
    for(int i=0;i<4;i++){
      if(present[i]){
        if(!prefs.putFloat(onKeys[i],requestedOn[i]))saved=false;
        if(!prefs.putFloat(offKeys[i],requestedOff[i]))saved=false;
      }else{
        if(prefs.isKey(onKeys[i])&&!prefs.remove(onKeys[i]))saved=false;
        if(prefs.isKey(offKeys[i])&&!prefs.remove(offKeys[i]))saved=false;
      }
    }
    for(int i=0;i<2;i++){
      if(!prefs.putBool((String("buzE")+i).c_str(),true))saved=false;
    }
    for(int i=0;i<4;i++){
      if(!prefs.putBool((String("sndRoleEnable")+i).c_str(),true))saved=false;
    }
    if(!saved){
      for(int i=0;i<4;i++){soundEnabled[i]=false;prefs.putBool((String("sndRoleEnable")+i).c_str(),false);}
      for(int i=0;i<2;i++){buzzerEnabled[i]=false;prefs.putBool((String("buzE")+i).c_str(),false);}
      soundOutputsOff();
      server.send(500,"text/plain","Temperature settings save failed");return;
    }
    for(int i=0;i<2;i++){
      buzzerOnC[i]=requestedOn[i];buzzerOffC[i]=requestedOff[i];buzzerEnabled[i]=true;
    }
    soundOnC[0]=soundOnC[1]=soundOnC[2]=requestedOn[2];
    soundOffC[0]=soundOffC[1]=soundOffC[2]=requestedOff[2];
    soundOnC[3]=requestedOn[3];soundOffC[3]=requestedOff[3];
    for(int i=0;i<4;i++){
      refTemp[i]=requestedOn[i];noticeTemp[i]=requestedOff[i];
      soundEnabled[i]=true;
    }
    for(int i=0;i<2;i++)buzzerTempChannels[i]=BusaSound::Channel();
    tftDirty=true;lastRender=0;
    server.send(200,"application/json",soundJSON(millis()));
  });

  addGoogleRoutes();
  server.on("/api/ota/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",otaJSON());
  });
  server.on("/api/ota/prepare",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"||otaActive||
       busaStaSwitchPending||busaStaSwitching||busaWifiScanRunning||
       wifiUiScanPending||wifiUiOwnsScan){
      server.send(403,"text/plain","Требуется локальная сеть БУСА и согласие");return;
    }
    char token[17];
    snprintf(token,sizeof(token),"%08lx%08lx",
      (unsigned long)esp_random(),(unsigned long)esp_random());
    otaToken=token;otaTokenIssued=millis();
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"token\":")+quoteJson(otaToken)+"}");
  });
  server.on("/api/ota/upload",HTTP_POST,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(otaSuccess?200:400,"application/json",otaJSON());
  },uploadOTA);
  // Same asynchronous scan queue for TFT and local phone application.
  // Neither caller consumes/deletes the other's results; no blocking wait.
  server.on("/api/wifi/scan",HTTP_GET,[]{
    if(!apClient()){
      server.send(403,"application/json","{\"error\":\"Only protected local AP\"}");return;
    }
    if(server.hasArg("start") && server.arg("start")=="1"){
      if(!wifiUiScanPending && !wifiUiOwnsScan && !busaStaSwitchPending && !busaStaSwitching)wifiUiBeginScan();
    }
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",wifiUiScanJSON());
  });
  server.on("/api/wifi",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Нужна локальная сеть БУСА");return;}
    String ssid=server.arg("ssid"),pass=server.arg("pass");ssid.trim();
    if(ssid.isEmpty()||ssid.length()>32||pass.length()<8||pass.length()>63){
      server.send(400,"text/plain; charset=utf-8","Имя сети или пароль некорректны");return;
    }
    if(!busaStageStaCredentials(ssid,pass)){
      server.send(409,"text/plain; charset=utf-8","Wi-Fi занят или OTA; повтори позднее");return;
    }
    server.sendHeader("Cache-Control","no-store");
    server.send(202,"text/plain; charset=utf-8",
      "Проверяю сеть. Если недоступна, БУСА останется в локальной сети.");
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>7){server.send(400,"application/json","{\"error\":\"page 1..7\"}");return;}
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
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Только БУСА AP");return;}
    if(!server.hasArg("mode")){server.send(400,"text/plain","Выбери тему");return;}
    int m=server.arg("mode").toInt();
    if(m<0||m>2||!saveThemeMode((uint8_t)m)){server.send(400,"text/plain","Тема не сохранена");return;}
    server.send(200,"text/plain; charset=utf-8",m==0?"Светлая тема":m==1?"Тёмная тема":"Системная тема");
  });
  server.on("/api/temperature/calibration",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Только БУСА AP");return;}
    float values[4];
    for(int i=0;i<4;i++){
      String name="offset"+String(i);
      if(!server.hasArg(name)){server.send(400,"text/plain","Нет поправки");return;}
      String v=server.arg(name);v.trim();char* stop=nullptr;
      float x=strtof(v.c_str(),&stop);
      if(v.isEmpty()||*stop!='\0'||!isfinite(x)||x< -10||x>10||
         fabsf(x*10-roundf(x*10))>0.02f){
        server.send(400,"text/plain; charset=utf-8","Поправка от -10 до +10, шаг 0,1");return;
      }
      values[i]=x;
    }
    for(int i=0;i<4;i++)if(!saveTempOffset(i,values[i])){
      server.send(500,"text/plain; charset=utf-8","Ошибка сохранения поправок");return;
    }
    server.send(200,"text/plain; charset=utf-8","Калибровка T1–T4 сохранена");
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
  // One shared history source for the physical TFT and local iPhone.
  // No invented collection values: only cooling-water pulse counts are sampled.
  server.on("/api/graph-history",HTTP_GET,[]{
    uint32_t now=millis();
    String j="{\"ok\":true,\"sample_period_ms\":5000,\"temperature\":[";
    for(int k=0;k<4;k++){
      if(k)j+=",";
      j+="[";
      for(int i=0;i<v13HistoryCount;i++){
        if(i)j+=",";
        const int idx=(v13HistoryHead+48-v13HistoryCount+i)%48;
        const float v=v13History[k][idx];
        j+=isfinite(v)?String(v,2):String("null");
      }
      j+="]";
    }
    j+="],\"water_pulses\":[";
    const int waterCount=waterSamplesRecorded;
    for(int i=0;i<waterCount;i++){
      if(i)j+=",";
      const int idx=(flowTraceHead+40-waterCount+i)%40;
      j+=String(waterPulseHistory[idx]);
    }
    j+="],\"water_interval_ms\":2000,\"water_calibrated\":false,";
    j+="\"collection_sensor\":false,\"time_ms\":"+String(now)+",";
    j+="\"temperature_samples\":"+String(v13HistoryCount)+"}";
    server.send(200,"application/json; charset=utf-8",j);
  });
  server.on("/api/graph-view",HTTP_GET,[]{
    int n=server.arg("view").toInt();
    if(n<0||n>1){server.send(400,"text/plain","Bad graph");return;}
    graphSubView=(uint8_t)n;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/settings-tab",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<0||n>4){server.send(400,"text/plain","bad subtab");return;}
    settingsSubTab=(uint8_t)n;approvedSettingsItem=(uint8_t)n;tftDirty=true;lastRender=0;
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
  busaReportStage("HTTP_READY");
  // Never do a full I2C address scan inside setup.
  busaReportStage("I2C_BEGIN");
  // Configure pads as open-drain input/output, with a reset of stale GPIO routing.
  busaI2cConfigurePads();
  busaI2cLinesFree();
  if(!busaI2cLastSda || !busaI2cLastScl)busaI2cRecover();
  busaI2cRunGpioSelfTest(); // bounded, non-destructive bus-level selftest
  i2cOnline=true;
  updateBootStage(2);
  busaReportStage("I2C_READY");
  updateBootStage(3);
  launchGoogleCloud();
  busaSetupDoneMs=millis()-busaBootAt;
  busaReportStage("SETUP_DONE");
  Serial.printf("BUSA SETUP DONE ms=%lu K1-K4 sound-only\n",(unsigned long)busaSetupDoneMs);
}
void loop(){
  uint32_t now=millis();
  const uint32_t tickStart=now;
  soundPoll(now);
  {const uint32_t started=millis();server.handleClient();busaObserve("HTTP",started,busaHttpWorstMs);}
  // Local AP continues even when no external hotspot exists.
  busaStaService();
  readConsole();
  if(!otaActive)wifiUiPoll(); // nonblocking Wi-Fi scan and password-provisioning on physical TFT
  if(otaRestart && elapsed(now,otaRestartAt,4000)){soundOutputsOff();ESP.restart();}
  // Avoid I2C/1-Wire/SPI scanning while Wi-Fi OTA flash chunks arrive.
  // Flash throughput remains hardware/network limited.
  if(otaActive){soundOutputsOff();delay(0);return;}
  if(tftRebootPending && elapsed(now,tftRebootAt,3000)){soundOutputsOff();ESP.restart();}
  if(googleRestart && elapsed(now,googleRestartAt,3500)){soundOutputsOff();ESP.restart();}
  busaRemoteAlarmApply(); // remote sound thresholds only; writes NVS in main loop
  {const uint32_t started=millis();sampleTemperatures(now);busaObserve("TEMP",started,busaTempWorstMs);}
  {const uint32_t started=millis();probeOneWire(now);busaObserve("ONEWIRE",started,busaOneWireWorstMs);} // Diagnose on idle 1-Wire bus only
  // A missing sensor is a diagnostic error, not a reason to trap the UI on boot.
  // Wi-Fi AP and HTTP server initialize during the splash.
  // Missing sensors must not delay the user interface.
  if(!bootSplashComplete && elapsed(now,splashStartedAt,2500)){
    bootSplashComplete=true;
    tftDirty=true;
  }
    sampleFlow(now);
  v13Collect(now); // Shared graph sampling, including when TFT screen is asleep.
  // TFT graph is repainted inside its own plot border by v14RefreshLive().
  // Never set tftDirty here: that would erase the entire screen every 5 s.
  // Landscape V14.7: repaint changed fields at 5 Hz; full frame only on page/theme change.
  // TFT driver writes are kept in the main loop so no concurrent SPI transactions occur.
  if(tftStarted && !tftDisplayOff && !otaActive && bootSplashComplete &&
     elapsed(now,splashStartedAt,splashHoldMs) &&
     (tftDirty||lastRender==0 || elapsed(now,lastRender,BUSA_TFT_LIVE_REFRESH_MS))){
    const bool wholeFrame=tftDirty||lastRender==0;
    lastRender=now;
    const uint32_t started=millis();
    if(screen->width()>=300 && !wholeFrame){
      v14RefreshLive(now);
      busaTftLiveRenderCount++;
    }else{
      renderScreen(now);
      busaTftFullRenderCount++;
    }
    busaLastTftRenderMs=millis()-started;
    busaObserve("TFT",started,busaTftWorstMs);
  }
  // Display updates take priority over peripheral diagnostics.
  if(i2cOnline){
    if((busaI2cElectricalFault || i2cFoundCount==0) && !busaI2cSuspended(now) &&
       elapsed(now,busaI2cLastGpioTestMs,30000)){
      const uint32_t started=millis();
      // Reapply the pad configuration after a failed readback, without restart.
      if(busaI2cElectricalFault && elapsed(now,busaI2cLastPadReinitMs,30000)){
        busaI2cConfigurePads();
      }
      busaI2cRunGpioSelfTest();
      busaObserve("I2C_GPIO_TEST",started,busaI2cWorstMs);
    }
    // Query connected PCF/PCA once per 6 s. Probe one supported I2C address
    // per loop; prevent keypad bus errors from starving the web server.
    // Profile each blocking component and quarantine a misbehaving I2C bus.
    if(!busaI2cElectricalFault){
    {const uint32_t started=millis();scanI2cAddresses(now);busaObserve("I2C_DISCOVERY",started,busaI2cWorstMs);}
    if(!lastBusProbe || elapsed(now,lastBusProbe,6000)){
      const uint32_t started=millis();probeI2c(now);busaObserve("I2C_DEVICES",started,busaI2cWorstMs);
    }
    {const uint32_t started=millis();pollKeys(now);busaObserve("KEYPAD",started,busaMaxKeyScanMs);}
    }
  }
  if(elapsed(now,lastUart,5000)){
    lastUart=now;
    Serial.printf("BUSA ALIVE ms=%lu stage=%s AP=%d i2c=%d tft=%d worst_loop=%lu key_ms=%lu i2c_ms=%lu\n",
      (unsigned long)now,busaBootStage,(int)wifiRunning,(int)i2cOnline,
      (int)tftStarted,(unsigned long)busaMaxLoopMs,(unsigned long)busaLastKeyScanMs,
      (unsigned long)busaLastI2cStepMs);
    Serial0.printf("N16R8 HEARTBEAT ms=%lu wifi=%d i2c=%d tft=%d p=%lu flow=%.3f\n",
      (unsigned long)now,(int)wifiRunning,(int)i2cOnline,(int)tftStarted,
      (unsigned long)flowPulses(),(double)flowLpm);
  }
  const uint32_t loopMs=millis()-tickStart;
  if(loopMs>busaMaxLoopMs)busaMaxLoopMs=loopMs;
  if(loopMs>100)busaSlowLoops++;
  delay(1); // Yield to Wi-Fi/RTOS each pass, with less idle latency than 2 ms.
}
