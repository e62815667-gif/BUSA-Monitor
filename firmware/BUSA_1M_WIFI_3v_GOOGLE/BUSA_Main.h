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
#include <esp_ota_ops.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <sys/time.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
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
const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-CALC4-WIFI1-UI8-20261107";
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
uint8_t currentPage=0; // 8 sections including Calculator
uint8_t graphSubView=0; // 0 four isolated temperature lanes; 1 water + collection lanes
uint8_t graphMenuCursor=0; // Selected item in the Graphs submenu; ENTER opens it
uint32_t busaGraphLastDrawnCapture=0;
uint8_t approvedSettingsItem=0; // Seven Settings entries, including Events and Diagnostics
uint8_t graphRangeIndex=0;
static const uint8_t BUSA_GRAPH_MINUTES[5]={5,10,15,30,60};
static const char* const BUSA_RELAY_LABELS[4]={"ТЭН откл.","Кран закрыт","Кран открыт","Авария"};
uint8_t settingsSubTab=0,tftView=0,previousMainPage=0;
uint8_t diagModule=0,calSensor=0,themeDraft=2,tftThemeMode=2;
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
  previousMainPage=5;approvedSettingsItem=6;
  currentPage=6;diagModule=1;tftView=1;tftDirty=true;
}

#include "BUSA_CalculatorUI.h"
#include "BUSA_KeypadTFT_UI.h"
void navigatePage(uint8_t target){
  if(target>=8||target==currentPage)return;
  busaCalcEditing=false;
  if(currentPage==5 && target!=5 && approvedSettingsItem==1 && tftView){
    wifiUiLeave(); // scrub unsaved password; AP BUSA keeps running
  }
  previousMainPage=(target==4||target==6||currentPage==4||currentPage==6)?5:currentPage;
  if(target==4)approvedSettingsItem=5;
  if(target==6)approvedSettingsItem=6;
  // Open other section lists at their first visible row, just like Settings.
  if(target==1){selectedTemp=0;selectedField=0;}
  else if(target==3)graphMenuCursor=0;
  else if(target==6)diagModule=0;
  else if(target==7){busaCalcMode=0;busaCalcField=0;}
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
  if(busaCalcKey(k))return;
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
    if((currentPage==4||currentPage==6)&&!tftView){navigatePage(5);previousMainPage=0;}
    else if(tftView){tftView=0;if(currentPage==1)selectedField=0;}
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
    }else if(currentPage==1 && tftView){
      // Removal is available only inside the selected sensor's settings page.
      deleteConfirmPending=true;
      keyMessage=String("Удалить T")+String(selectedTemp+1)+
        (selectedField?" ВЫКЛ":" ВКЛ")+"? ENT/ESC";
    }
  }else if(k==7 || k==11){ // UP/DOWN: move within current section
    if(currentPage==1){
      if(tftView){
        // Sensor subpage: navigate between ON and OFF, without another popup.
        selectedField=(selectedField+1)%2;
      }else{
        // One visible list, top to bottom: T1, T2, T3, T4.
        // Physical sensor numbering and saved values are unchanged.
        selectedTemp=(selectedTemp+(k==7?3:1))%4;
        selectedField=0;
      }
    }else if(currentPage==5){
      if(!tftView){
        approvedSettingsItem=(approvedSettingsItem+(k==7?6:1))%7;
        settingsSubTab=approvedSettingsItem;
      }else if(approvedSettingsItem==0){
        // Temperature subpage always shows T1..T4 simultaneously.
        calSensor=(calSensor+(k==7?3:1))%4;
      }else if(approvedSettingsItem==2){
        themeDraft=(themeDraft+(k==7?2:1))%3;
      }
    }else if(currentPage==6){
      // Like Settings: move in the diagnostics menu, never close an opened child.
      if(!tftView)diagModule=(diagModule+(k==7?6:1))%7;
    }else if(currentPage==3){
      // Select Graphs entries in sequence. Do not switch a graph during preview.
      if(tftView)graphRangeIndex=(graphRangeIndex+(k==7?4:1))%5;
      else {graphSubView=(graphSubView+(k==7?2:1))%3;graphMenuCursor=graphSubView;}
    }
    // Home/Collection/Events are read-only panels; no invisible submenus.
  }else if(k==16 || k==18){ // LEFT/RIGHT: change sections everywhere
    moveApprovedTab(k==16?-1:+1);
  }else if(k==19){ // ENT: open selected element
    if(currentPage==1){
      if(!tftView){
        // First ENTER opens the chosen sensor's full inline settings page.
        tftView=1;selectedField=0;
      }else{
        // Second ENTER edits the selected ON/OFF row in place.
        editingTemp=true;
        float v=selectedField?noticeTemp[selectedTemp]:refTemp[selectedTemp];
        editTempBuffer=isfinite(v)?String(v,1):String("");
        editTempBuffer.replace('.',',');
        editCursor=(uint8_t)editTempBuffer.length();
      }
    }else if(currentPage==0){
      navigatePage(1);
    }else if(currentPage==5){
      if(!tftView){
        // Events and Diagnostics belong to Settings, outside the main cycle.
        if(approvedSettingsItem==5){navigatePage(4);return;}
        if(approvedSettingsItem==6){navigatePage(6);return;}
        tftView=1;
        themeDraft=tftThemeMode;
        if(approvedSettingsItem==1){
          // Open exactly one finite scan, with no extra search menu.
          wifiUiStage=WIFIUI_STATUS;wifiUiMenuIndex=0;
          wifiUiBeginScan();
        }
      }else if(approvedSettingsItem==0){
        calEditing=true;calDraft=tempOffset[calSensor];
      }else if(approvedSettingsItem==2){
        if(saveThemeMode(themeDraft)){
          keyMessage="Тема сохранена";
          tftView=0; // One ENT saves and returns to Settings.
        }else keyMessage="Ошибка сохранения темы";
      }
    }else if(currentPage==6){
      // ENTER opens the selected diagnostic module; ESC is the only way back.
      if(!tftView)tftView=1;
    }else if(currentPage==3){
      // ENTER opens exactly the graph highlighted by UP/DOWN, not the next one.
      tftView=tftView?0:1;
    }
    // Nothing else to open in read-only sections.
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
      if((uint32_t)(now-pendingAt)>2000){if(currentPage==7&&busaCalcEditing)busaCalcDecimal();else wifiUiLongZero();}
      else keypadAction(17);
    }
    pendingPair=pair;pendingAt=now;keyHeld=false;keypadDeferredZero=false;
  }
  if(pair<0)return;
  if(keyHeld){
    if(keypadDeferredZero && (uint32_t)(now-pendingAt)>2000){
      keypadDeferredZero=false;
      if(currentPage==7&&busaCalcEditing)busaCalcDecimal();else wifiUiLongZero();
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
      if(k==17 && (editingTemp || (currentPage==7&&busaCalcEditing) ||
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
// A full-color PSRAM-backed TFT frame eliminates the blank flash on keypress.
// Render a whole landscape screen offscreen and transfer one complete frame
// over SPI only AFTER all cards, fonts and highlights are finished.
// Falls back to the existing immediate renderer if no PSRAM is available.
class BusaTftPsramFrame : public Adafruit_GFX {
 public:
  uint16_t* data=nullptr;
  BusaTftPsramFrame():Adafruit_GFX(320,240){}
  bool ensure(){
    if(!data && ESP.getPsramSize()>0){
      data=(uint16_t*)heap_caps_malloc(320u*240u*sizeof(uint16_t),
         MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    }
    return data!=nullptr;
  }
  void drawPixel(int16_t x,int16_t y,uint16_t color) override {
    if(data && x>=0 && y>=0 && x<320 && y<240)
      data[(uint32_t)y*320u+(uint16_t)x]=color;
  }
  void fillRect(int16_t x,int16_t y,int16_t w,int16_t h,uint16_t color) override {
    if(!data||w<=0||h<=0)return;
    const int16_t x1=max((int16_t)0,x),y1=max((int16_t)0,y);
    const int16_t x2=min((int16_t)320,(int16_t)(x+w));
    const int16_t y2=min((int16_t)240,(int16_t)(y+h));
    if(x2<=x1||y2<=y1)return;
    for(int16_t yy=y1;yy<y2;yy++){
      uint16_t* row=data+(uint32_t)yy*320u+x1;
      for(int16_t xx=x1;xx<x2;xx++)*row++=color;
    }
  }
  void drawFastHLine(int16_t x,int16_t y,int16_t w,uint16_t color) override {
    fillRect(x,y,w,1,color);
  }
  void drawFastVLine(int16_t x,int16_t y,int16_t h,uint16_t color) override {
    fillRect(x,y,1,h,color);
  }
  void fillScreen(uint16_t color) override {fillRect(0,0,320,240,color);}
};
static BusaTftPsramFrame busaTftFrame;

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
const char* pages[8]={"Главная","Температуры","Отбор","Графики","События","Настройки","Диагностика","Калькулятор"};
const uint8_t UI_ORDER[6]={0,1,2,3,7,5}; // Six main sections; Events/Diagnostics are Settings children
void navigatePage(uint8_t target); // forward declaration
uint8_t orderSlot(uint8_t page){if(page==4||page==6)return 5;for(uint8_t i=0;i<6;i++)if(UI_ORDER[i]==page)return i;return 0;}
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
  const String version="FW V14.9 N16R8";
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
  screen->setCursor((w-12*6*scale)/2,h/2-30);
  screen->print("BUSA MONITOR");

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
static void busaCalcDraw();
#include "BUSA_ChartHistory.h"
static BusaChart::History busaChartHistory;
static bool busaChartReady=false,busaChartAttempted=false;
static void busaCaptureChart(uint32_t now){
 if(!busaChartAttempted){
  busaChartAttempted=true;
  auto* memory=(BusaChart::Row*)heap_caps_malloc(sizeof(BusaChart::Row)*1801,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(memory){busaChartHistory.begin(memory,1801,0);busaChartReady=true;}
 }
 if(!busaChartReady)return;
 float values[4];for(int k=0;k<4;k++)values[k]=hasTemp(k,now)?temperature[k]:NAN;
 static uint32_t lastAt=0,lastPulses=0;
 const uint32_t pulses=flowPulses(),dt=now-lastAt;
 float cooling=NAN;
 if(lastAt&&dt&&flowSeen&&pulses>=lastPulses)
  cooling=(float)(pulses-lastPulses)*3600000.0f/(dt*FLOW_HZ_PER_LPM*60.0f);
 lastAt=now;lastPulses=pulses;
 busaChartHistory.capture(now,values,cooling);
}
#include "BUSA_ChartTFT.h"
#include "BUSA_ApprovedUI_V13.h"

#include "BUSA_ApprovedGraphics_HQ_V14.h"
#include "BUSA_Landscape_V14.h"
#include "BUSA_CalculatorTft.h"
void renderScreen(uint32_t now){
  if(!tftStarted||!screen)return;
  if(!bootSplashComplete || !elapsed(now,splashStartedAt,splashHoldMs))return;
  if(screen->width()>=300){
    if(busaTftFrame.ensure()){
      Adafruit_GFX* physical=screen;
      screen=&busaTftFrame;
      ru.begin(*screen);
      ru.setFont(u8g2_font_6x13_t_cyrillic);
      ru.setFontMode(1);
      renderLandscape(now); // No drawing reaches the TFT until frame is complete.
      screen=physical;
      ru.begin(*screen);
      ru.setFont(u8g2_font_6x13_t_cyrillic);
      ru.setFontMode(1);
      if(tftProfile==2){
        screenIli.startWrite();
        screenIli.setAddrWindow(0,0,320,240);
        screenIli.writePixels(busaTftFrame.data,320u*240u);
        screenIli.endWrite();
      }else{
        screenSt.startWrite();
        screenSt.setAddrWindow(0,0,320,240);
        screenSt.writePixels(busaTftFrame.data,320u*240u);
        screenSt.endWrite();
      }
    }else{
      renderLandscape(now); // Graceful fallback if PSRAM allocation fails.
    }
    return;
  }
  renderPortrait(now);
}

WebServer server(80);
static esp_err_t busaOtaConfirmResult=ESP_ERR_NOT_FOUND;
static esp_err_t busaOtaSelectResult=ESP_ERR_NOT_SUPPORTED;
static String busaOtaRunning="unknown",busaOtaBoot="unknown";
bool otaActive=false,otaSuccess=false,otaRestart=false,otaWriteAllowed=false;
uint32_t otaBytes=0,otaRestartAt=0,otaTokenIssued=0,startedAt=0;
String otaToken="",otaMessage="Ожидание",nearbyNames[5];
uint8_t nearbyCount=0;
volatile bool busaWifiScanRunning=false;
uint32_t busaWifiScanStartedAt=0,busaWifiScanLastAt=0;

bool screenAttempted=false;
bool apClient(){return server.client().localIP()==WiFi.softAPIP();}

String apPassword="";
bool wifiRunning=false;
uint32_t lastRender=0,lastUart=0,lastStationCheck=0;
String consoleLine="",stationSsid="",stationPass="";
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

#include "BUSA_WifiScan.h"

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
#include "BUSA_WebJournal.h"
static BusaWebJournal::Ring busaWebJournal;
static uint32_t busaWebBootId=0;
static void busaWebRecord(const String& text,uint8_t kind=0){
 busaWebJournal.add(millis(),deviceClockValid()?(uint32_t)time(nullptr):0,kind,text.c_str());
}
static void busaWebEventsPoll(uint32_t now){
 static bool initialized=false,attempted=false;
 static bool relays[4]={},enabled[4]={},oldWifi=false,oldGoogle=false,oldScreen=false,oldLearning=false,oldReady=false,oldOta=false,oldOtaOk=false,oldPcf=false,oldBuzzer=false;
 static uint8_t oldTheme=255,oldStep=255;static int oldGoogleState=-1;
 static String oldKey="";static uint32_t lastKeyCount=0,lastNetworkAt=0,lastPulseAt=0,lastPulses=0;
 static bool oldFlow=false;
 if(!initialized){
  if(attempted)return;attempted=true;
  auto* rows=(BusaWebJournal::Entry*)heap_caps_malloc(sizeof(BusaWebJournal::Entry)*256,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(!rows)return;
  busaWebJournal.begin(rows,256);busaWebBootId=esp_random();initialized=true;busaWebRecord("Устройство включено");
  for(int i=0;i<4;i++){relays[i]=soundChannels[i].on;enabled[i]=soundEnabled[i];}
  oldScreen=tftDisplayOff;oldTheme=tftThemeMode;oldLearning=keypadLearning;oldReady=keypadReady;oldStep=learnStep;oldPcf=pcfDetected;oldKey=keyMessage;lastKeyCount=keyCount;
 }
 for(int i=0;i<4;i++){
  const bool on=soundChannels[i].on;
  if(on!=relays[i]){busaWebRecord(String(BUSA_RELAY_LABELS[i])+": "+(on?"активен":"неактивен"),i==3&&on?2:0);relays[i]=on;}
  if(soundEnabled[i]!=enabled[i]){enabled[i]=soundEnabled[i];busaWebRecord(String(BUSA_RELAY_LABELS[i])+(enabled[i]?": разрешён":": отключён"));}
 }
 const bool buzzer=buzzerTempChannels[0].on||buzzerTempChannels[1].on;
 if(buzzer!=oldBuzzer){oldBuzzer=buzzer;busaWebRecord(buzzer?"Зуммер включён":"Зуммер выключен");}
 if(oldScreen!=tftDisplayOff){oldScreen=tftDisplayOff;busaWebRecord(oldScreen?"Экран выключен":"Экран включён");}
 if(oldTheme!=tftThemeMode){oldTheme=tftThemeMode;busaWebRecord("Тема экрана изменена");}
 if(oldLearning!=keypadLearning){oldLearning=keypadLearning;busaWebRecord(oldLearning?"Начато обучение клавиатуры":"Обучение клавиатуры завершено");}
 if(oldReady!=keypadReady){oldReady=keypadReady;busaWebRecord(oldReady?"Раскладка клавиатуры готова":"Раскладка клавиатуры недоступна",oldReady?0:1);}
 if(keypadLearning&&oldStep!=learnStep){oldStep=learnStep;busaWebRecord("Обучение клавиатуры: шаг "+String(learnStep+1));}
 if(oldKey!=keyMessage||lastKeyCount!=keyCount){lastKeyCount=keyCount;oldKey=keyMessage;if(oldKey.length()&&oldKey!="NO DATA")busaWebRecord("Клавиатура: "+oldKey);}
 if(oldPcf!=pcfDetected){oldPcf=pcfDetected;busaWebRecord(oldPcf?"Клавиатура подключена":"Клавиатура недоступна",oldPcf?0:1);}
 if(oldOta!=otaActive){oldOta=otaActive;busaWebRecord(oldOta?"Запись обновления начата":"Запись обновления завершена");}
 if(oldOtaOk!=otaSuccess){oldOtaOk=otaSuccess;if(oldOtaOk)busaWebRecord("Обновление записано успешно");}
 const uint32_t pulses=flowPulses();
 if(pulses!=lastPulses){lastPulses=pulses;lastPulseAt=now;}
 const bool flowing=lastPulseAt&&(uint32_t)(now-lastPulseAt)<5000;
 if(flowing!=oldFlow){oldFlow=flowing;busaWebRecord(flowing?"Появились импульсы охлаждения":"Импульсы охлаждения прекратились",flowing?0:1);}
 if(!lastNetworkAt||(uint32_t)(now-lastNetworkAt)>=1000){
  lastNetworkAt=now;const bool wifi=WiFi.status()==WL_CONNECTED,google=googleEnabled;
  if(wifi!=oldWifi){oldWifi=wifi;busaWebRecord(wifi?"Wi-Fi подключён":"Wi-Fi отключён",wifi?0:1);}
  if(google!=oldGoogle){oldGoogle=google;busaWebRecord(google?"Google подключён в настройках":"Google отключён в настройках");}
  if(oldGoogleState!=googleState){oldGoogleState=googleState;if(googleState==6)busaWebRecord("Ошибка отправки в Google",1);else if(googleState==5)busaWebRecord("Данные отправлены в Google");}
 }
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
  s+=",\"sta_disabled_by_user\":"+(wifiUiStaDisabled?String("true"):String("false"));
  // Scan diagnostics let Google telemetry distinguish zero visible SSIDs
  // from a driver error, a timeout or an overlapping local-app scan.
  s+=",\"scan_result_code\":"+String(wifiUiLastScanCode);
  s+=",\"scan_driver_rc\":"+String(busaSharedScanStartResult);
  s+=",\"scan_event_status\":"+String(busaScanEventStatus.load());
  s+=",\"scan_attempts\":"+String(wifiUiScanAttempts);
  s+=",\"scan_network_count\":"+String(wifiUiCount);
  s+=",\"scan_in_progress\":"+(busaWifiScanRunning?String("true"):String("false"));
  s+=",\"sta_rssi_dbm\":"+(WiFi.status()==WL_CONNECTED?String(WiFi.RSSI()):String("null"));
  s+=",\"station_credentials_protected\":true}";
  s+="}";
  s+=",\"ap\":{\"enabled\":"+(wifiRunning?String("true"):String("false"));
  s+=",\"ip\":"+quoteJson(WiFi.softAPIP().toString())+"}";
  s+=",\"ota\":"+otaJSON();
  s+=",\"ota_boot\":{\"running\":"+quoteJson(busaOtaRunning)+
    ",\"selected\":"+quoteJson(busaOtaBoot)+
    ",\"verification_rc\":"+String((int)busaOtaConfirmResult)+
    ",\"selection_rc\":"+String((int)busaOtaSelectResult)+"}";
  // Normal operation intentionally has BLE switched off. Last BLE outcome
  // persists independently of this live field.
  s+=",\"bluetooth_enabled\":false";
  s+=",\"ble_last_stage\":"+quoteJson(prefs.getString("rmStage","IDLE"));
  s+=",\"google\":"+cloudJSON();
  s+=",\"network_indicators\":{\"internet\":"+(googleInternet==1?String("true"):String("false"));
  s+=",\"local_client\":"+(WiFi.softAPgetStationNum()>0?String("true"):String("false"));
  s+=",\"ap_running\":"+(wifiRunning?String("true"):String("false"))+"}";
  const String actualSsid=WiFi.status()==WL_CONNECTED?WiFi.SSID():String("");
  const bool exactStation=WiFi.status()==WL_CONNECTED && actualSsid==stationSsid;
  s+=",\"station\":{\"ssid\":"+quoteJson(actualSsid);
  s+=",\"target_ssid\":"+quoteJson(stationSsid);
  s+=",\"connected\":"+(exactStation?String("true"):String("false"));
  s+=",\"connecting\":"+(wifiUiConnectPending || wifiUiAssociationDueAt ||
       (wifiUiStage==WIFIUI_CONNECTING && wifiUiConnectAt>0)?String("true"):String("false"));
  s+=",\"last_error\":"+quoteJson(wifiUiLastConnectError);
  s+=",\"ip\":"+quoteJson(exactStation?WiFi.localIP().toString():String(""))+"}";
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
#include "BUSA_CalculatorWeb.h"
const char WEB_PAGE[] PROGMEM=R"HTMLBUSA(
<!doctype html><html lang="ru"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BUSA MONITOR</title>
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


/* Approved 03.11 dashboard design; original actual telemetry preserved. */
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

.appTabs{display:flex!important;overflow-x:auto!important}.appTabs button{flex:0 0 78px!important;min-width:78px!important;font-size:10px!important}

/* UI5: compact single brand, open temperature readouts, transparent vector rig. */
.appTabs{display:grid!important;grid-template-columns:repeat(7,minmax(0,1fr))!important;overflow-x:visible!important;gap:2px}
.appTabs button{min-width:0!important;width:100%;padding:6px 0!important;font-size:clamp(7px,2.1vw,10px)!important;border-radius:9px;gap:4px;line-height:1.15}
.appTabs button .uiIcon{width:20px;height:20px}
.appTabs .tabIcon{font-size:22px;line-height:20px}
.netIndicators{gap:5px}
.netPill{justify-content:center;font-size:clamp(10px,2.9vw,12px);font-variant-numeric:tabular-nums}

.appHeader .headerRow{min-height:30px;padding-bottom:4px}
.appHeader .headerRow h1{margin:0;font-size:19px;letter-spacing:1.4px}
.clockBar{font-variant-numeric:tabular-nums}
section[data-section="0"]{background:transparent;border:0;border-radius:0;padding:12px 0}
section[data-section="0"] .approvedDashboard{grid-template-columns:minmax(100px,.74fr) minmax(0,1.26fr);gap:6px;min-height:364px}
section[data-section="0"] .approvedTempGrid{justify-content:space-around;gap:12px;padding:6px 0}
:root section[data-section="0"] .approvedTemp{border:0;border-radius:0;box-shadow:none;background:transparent!important;min-height:72px;padding:0 4px}
section[data-section="0"] .approvedTempTitle{font-size:clamp(11px,2.8vw,15px);gap:6px}
section[data-section="0"] .approvedTempIcon{width:25px;min-width:25px;height:30px}
section[data-section="0"] .busaRefTempIcon{filter:none}
section[data-section="0"] .approvedTemp strong{font-size:clamp(25px,6.8vw,38px);margin:3px 0}
section[data-section="0"] .approvedTemp small{font-size:11px}
section[data-section="0"] .approvedRig{background:transparent;border:0;box-shadow:none;min-height:364px;align-self:stretch}
section[data-section="0"] .approvedRig .rigIllustration{display:block;width:100%;height:100%;min-height:364px;max-height:none;overflow:visible}
:root section[data-section="1"] .approvedSensor{border:0;border-radius:0;box-shadow:none;background:transparent!important}
section[data-section="1"] .sensorEdit{border:0}
@media(min-width:600px){section[data-section="0"] .approvedDashboard{min-height:440px}section[data-section="0"] .approvedRig,section[data-section="0"] .approvedRig .rigIllustration{min-height:440px}}

/* UI6: six sections, coherent chart/calculator controls and read-only output roles. */
.appTabs{grid-template-columns:repeat(6,minmax(0,1fr))!important}
.appTabs button{font-size:clamp(8px,2.3vw,11px)!important}
.relayIndicators{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin:12px 0}
.relayBadge{display:grid;grid-template-columns:10px minmax(0,1fr);gap:5px 8px;align-items:center;border:1px solid #244356;border-radius:10px;background:#081a27;padding:11px;font-size:13px;color:#aec0cd}
.relayBadge small{grid-column:2;font-size:10px;color:inherit;opacity:.8}
.relayLamp{width:8px;height:8px;border-radius:50%;background:#526471}
.relayBadge[data-state="on"]{background:#103e3a;border-color:#50e2ab;color:#9bffd5}
.relayBadge[data-state="on"] .relayLamp{background:#62f1b6;box-shadow:0 0 8px #62f1b666}
#relayBadge3[data-state="on"]{background:#49222d;border-color:#ff6b7c;color:#ffc2cb}
#relayBadge3[data-state="on"] .relayLamp{background:#ff6b7c;box-shadow:0 0 8px #ff6b7c66}
.chartCard{background:#061522;border:1px solid #24465d;border-radius:14px;padding:12px}
.chartTabs,.chartRanges{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:6px}
.chartTabs button,.chartRanges button{min-width:0;background:#0a2032;color:#c2d8e7;border:1px solid #284b64;padding:10px 2px;font-size:12px}
.chartTabs button.on,.chartRanges button.on{background:#0872ce;color:#fff;border-color:#53cfff;box-shadow:0 0 8px #0587f733}
.chartRanges{grid-template-columns:repeat(5,minmax(0,1fr));margin-top:10px}
.chartHeading{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:16px 0 10px;font-size:13px;color:#d8eaf7}
.chartHeading span{font-size:10px;color:#98b8cd;text-align:right}
.chartLegend{display:grid;grid-template-columns:1fr 1fr;gap:6px 10px;margin:8px 0 12px;font-size:11px;color:#b8d0df}
.chartLegend[hidden]{display:none}
.chartLegend span:before{content:"";display:inline-block;width:14px;height:3px;background:var(--series);vertical-align:middle;margin-right:6px}
#chartCanvas{display:block;width:100%;height:auto;aspect-ratio:640 / 440;border:0;border-radius:8px}
@media(max-width:360px){.chartTabs button{font-size:10px}.chartCard{padding:9px}.relayBadge{font-size:12px;padding:9px}}
/* UI7 changes are confined to the local browser. Sensor IDs remain stable. */
:root{--water:#38bdf8;--cube:#f4c84a;--column:#53d486;--alarm:#ff6473}
.srOnly{position:absolute;width:1px;height:1px;padding:0;margin:-1px;overflow:hidden;clip:rect(0,0,0,0);white-space:nowrap;border:0}
.appTabs button span{max-width:100%;text-align:center;line-height:1.2}
section[data-section="0"] .approvedRig{min-height:0;overflow:visible;align-self:center}
.rigArtStage{position:relative;isolation:isolate;width:100%;aspect-ratio:821 / 1916;overflow:hidden}
.approvedRigArt{display:block;width:100%;height:auto;object-fit:contain;background:transparent}
.rigSheen{position:absolute;inset:0;pointer-events:none;background:linear-gradient(110deg,transparent 38%,#d6efff55 49%,transparent 60%);background-size:260% 100%;clip-path:polygon(31% 24%,50% 24%,50% 72%,68% 76%,68% 98%,18% 98%,18% 77%,31% 71%);animation:rigShine 3.5s ease-in-out infinite;opacity:.85}
.rigCoolantGlow{position:absolute;left:76%;top:47%;width:18%;height:25%;opacity:0;background:linear-gradient(0deg,transparent,#38bdf87a,transparent);border-radius:12px}
.boilerBoil{position:absolute;left:31.5%;top:81.6%;width:19.5%;height:9.5%;overflow:hidden;border-radius:5% 5% 30% 30%;pointer-events:none}
.boilerBoil i{position:absolute;bottom:-10%;left:var(--x);width:8%;aspect-ratio:1;border:1px solid #b8f5ffcc;border-radius:50%;background:radial-gradient(circle at 30% 25%,#e3ffffb3,transparent 65%);animation:busaBoil var(--d) linear infinite;animation-delay:calc(var(--i)*-.37s)}
.boilerSurface{position:absolute;left:-5%;top:0;width:110%;height:7%;border-radius:50%;background:#65e5ff88;animation:busaSurface 1.8s ease-in-out infinite alternate}
@keyframes busaBoil{0%{transform:translateY(0) scale(.4);opacity:0}15%{opacity:.9}85%{opacity:.9}100%{transform:translateY(-1600%) scale(1.15);opacity:0}}
@keyframes busaSurface{from{transform:translateX(-3%) rotate(-2deg);opacity:.5}to{transform:translateX(3%) rotate(2deg);opacity:.9}}
@media(prefers-reduced-motion:reduce){.boilerBoil i,.boilerSurface{animation:none!important}.boilerBoil i{bottom:40%;opacity:.7}}
.rigUpperGlow{left:6%;top:6%;width:13%;height:12%}
.hasCooling .rigCoolantGlow{opacity:.7;animation:coolantGlow 2.4s ease-in-out infinite}
.rigDrop{position:absolute;left:86.5%;top:81%;width:5px;height:8px;background:#40ceff;border-radius:70% 70% 70% 5%;opacity:0;transform:rotate(-45deg)}
.hasCollection .rigDrop{animation:rigDrop 2s linear infinite}.hasCollection .dropTwo{animation-delay:.65s}.hasCollection .dropThree{animation-delay:1.3s}
@keyframes rigShine{0%,15%{background-position:140% 0}85%,100%{background-position:-40% 0}}
@keyframes coolantGlow{50%{opacity:.2;transform:translateY(6px)}}
@keyframes rigDrop{0%{opacity:0;transform:translateY(0) rotate(-45deg)}15%{opacity:1}85%{opacity:1}100%{opacity:0;transform:translateY(50px) rotate(-45deg)}}
@media(prefers-reduced-motion:reduce){.rigSheen,.rigCoolantGlow,.rigDrop{animation:none!important}}
:root section[data-section="0"] .temp0 strong,:root section[data-section="1"] .temp0 .approvedSensorLive{color:var(--water)!important}
:root section[data-section="0"] .temp2 strong,:root section[data-section="1"] .temp2 .approvedSensorLive{color:var(--cube)!important}
:root section[data-section="0"] .temp1 strong,:root section[data-section="1"] .temp1 .approvedSensorLive{color:var(--column)!important}
:root section[data-section="0"] .temp3 strong,:root section[data-section="1"] .temp3 .approvedSensorLive{color:var(--alarm)!important}
.temp2 .busaRefTempIcon{filter:hue-rotate(-145deg)!important}.temp1 .busaRefTempIcon{filter:hue-rotate(-60deg)!important}.temp3 .busaRefTempIcon{filter:hue-rotate(165deg)!important}
section[data-section="1"] .approvedSensorList{gap:7px;margin:4px 0 12px}
section[data-section="1"] .approvedSensor{position:relative;min-height:74px}
section[data-section="1"] .approvedSensorHead{min-height:74px;padding:9px 35px 9px 8px;gap:9px}
section[data-section="1"] .approvedSensorIcon{flex:0 0 31px;width:31px;height:39px}
section[data-section="1"] .approvedSensorIcon .busaRefTempIcon{width:30px;height:38px}
section[data-section="1"] .approvedSensorText{gap:4px}
section[data-section="1"] .approvedSensorText strong{font-size:14px}
section[data-section="1"] .approvedSensorText small{font-size:10px}
section[data-section="1"] .approvedSensorLive{font-size:24px;min-width:62px;text-align:right}
section[data-section="1"] .sensorEdit{padding:0;border:0}
section[data-section="1"] .sensorEdit>summary{position:absolute;left:0;right:0;top:0;height:74px;list-style:none;display:flex;align-items:center;justify-content:flex-end;padding-right:10px;font-size:24px;color:#7dc8ee;cursor:pointer}
section[data-section="1"] .sensorEdit>summary::-webkit-details-marker{display:none}
section[data-section="1"] .sensorEdit[open]{padding:0 10px 12px}
section[data-section="1"] .sensorEdit[open]>summary>span:last-child{transform:rotate(90deg)}
.temperatureSaveRow{padding:8px 0;margin-top:4px}.temperatureSaveRow p:empty{display:none}
.liquidPanel{border:1px solid #284b61;border-radius:14px;background:#071c2b;padding:15px;margin:12px 0 16px}
.liquidPanelHeading{display:flex;justify-content:space-between;gap:10px;align-items:center;margin-bottom:17px}
.liquidPanelHeading h2{font-size:17px;margin:0!important;color:#ebf6ff}
.liquidPanelHeading>span{font-size:10px;color:#9eb9cb;text-align:right;max-width:130px}
.collectionPanel{border-color:#746537}.coolingPanel{border-color:#285878}
.liquidRateGrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.liquidRateGrid>div{display:flex;flex-direction:column;gap:7px;min-width:0}
.liquidRateGrid small{font-size:11px;color:#acc7d8}
.liquidRateGrid strong{font-size:clamp(25px,7vw,36px);font-variant-numeric:tabular-nums;overflow-wrap:anywhere;line-height:1.1}
.collectionPanel strong{color:var(--cube)}.coolingPanel strong{color:var(--water)}
.liquidSummary{border-top:1px solid #274151;margin-top:17px;padding-top:11px;display:grid;gap:13px}
.liquidSummary>div{display:flex;align-items:center;justify-content:space-between;gap:12px;font-size:12px;color:#b7cddd}
.liquidSummary strong{font-size:22px;font-variant-numeric:tabular-nums;white-space:nowrap}
#events .eventRow{display:grid;grid-template-columns:42px 8px minmax(0,1fr);align-items:start;gap:9px;padding:12px 0;font-size:12px}
#events .eventRow time{font-size:11px;color:#91acbf;font-variant-numeric:tabular-nums}
#events .eventRow .eventDot{width:6px;height:6px;min-width:6px;margin-top:5px;background:var(--water)}
#events .eventRow.warn .eventDot{background:var(--cube)}#events .eventRow.alarm .eventDot{background:var(--alarm)}
@media(max-width:360px){.liquidPanel{padding:12px}.liquidPanelHeading h2{font-size:15px}.liquidRateGrid small{font-size:10px}.chartTabs button{font-size:10px}.appTabs button{font-size:8px!important}}
</style><link rel="stylesheet" href="/calculator.css?v=CALC1"></head><body>
<header class="appHeader"><div class="headerRow"><div>
<h1>BUSA MONITOR</h1></div>
</div>
<div id="clockDisplay" class="clockBar" aria-live="off">Время синхронизируется…</div>
<div class="netIndicators" role="status" aria-label="Статусы Wi-Fi, Google и ESP32">
<span class="netPill"><span id="internetLed" class="netDot"></span>Wi-Fi <span id="internetLabel" class="netText">ОТКЛ</span></span>
<span class="netPill"><span id="googleLed" class="netDot"></span>Google <span id="googleLabel" class="netText">ОТКЛ</span></span>
<span class="netPill"><span id="localLed" class="netDot"></span>ESP32 <span id="localLabel" class="netText">ОТКЛ</span></span>

</div>
<nav class="appTabs" aria-label="Шесть разделов БУСА"><button data-page="0" class="on"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="m3 10 9-7 9 7v10H3z"/><path d="M9 20v-7h6v7"/></svg><span>Главная</span></button>
<button data-page="1"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg><span>Температуры</span></button>
<button data-page="2"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 2C8 8 5 11 5 15a7 7 0 0 0 14 0c0-4-3-7-7-13z"/></svg><span>Вода и отбор</span></button>
<button data-page="3"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 3v18h18"/><path d="m5 17 5-6 4 3 6-9"/></svg><span>Графики</span></button>
<button data-page="7"><svg class="uiIcon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7"><rect x="5" y="2" width="14" height="20" rx="2"/><path d="M8 6h8M8 10h2m4 0h2M8 14h2m4 0h2M8 18h2m4 0h2"/></svg><span>Калькулятор</span></button>
<button data-page="5"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg><span>Настройки</span></button></nav></header><main><section class="page" data-section="0">

<div class="approvedDashboard">
  <div class="approvedTempGrid"><div class="approvedTemp temp0"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon0" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><path d="M24 4C17 17 7 28 7 37a17 17 0 0034 0C41 28 31 17 24 4Z" fill="url(#busaIcon0)" stroke="#28cdff" stroke-width="2"/><path d="M15 35q0 12 12 13" fill="none" stroke="#dcfaff" stroke-width="2.5" stroke-linecap="round"/></svg></span><span>Вода</span></div><strong id="t0">Н/Д</strong><small id="homeSet0">ВКЛ — · ВЫКЛ —</small></div><div class="approvedTemp temp2"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon2" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="5" y="17" width="38" height="35" rx="7" stroke="#10b9f7" stroke-width="2.5" fill="#083357"/><path d="M8 31q8-5 16 0t16 0v18H8Z" fill="url(#busaIcon2)" opacity=".92"/><path d="M14 17V9h20v8" fill="none" stroke="#55d6ff" stroke-width="3"/></svg></span><span>Куб</span></div><strong id="t2">Н/Д</strong><small id="homeSet2">ВКЛ — · ВЫКЛ —</small></div><div class="approvedTemp temp1"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon1" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIcon1)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></span><span>Колонна</span></div><strong id="t1">Н/Д</strong><small id="homeSet1">ВКЛ — · ВЫКЛ —</small></div><div class="approvedTemp temp3"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon3" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIcon3)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></span><span>Авария</span></div><strong id="t3">Н/Д</strong><small id="homeSet3">ВКЛ — · ВЫКЛ —</small></div></div>
  <div class="approvedRig"><div class="rigArtStage" id="rigArtStage"><img class="approvedRigArt" src="data:image/webp;base64,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" alt="Аппарат с двумя холодильниками по согласованному рисунку"><div class="boilerBoil" aria-hidden="true"><span class="boilerSurface"></span><i style="--i:0;--x:12%;--d:2.4s"></i><i style="--i:1;--x:32%;--d:1.9s"></i><i style="--i:2;--x:54%;--d:2.7s"></i><i style="--i:3;--x:77%;--d:2.1s"></i><i style="--i:4;--x:22%;--d:3.1s"></i><i style="--i:5;--x:65%;--d:2.4s"></i><i style="--i:6;--x:43%;--d:2.2s"></i><i style="--i:7;--x:85%;--d:2.9s"></i></div><span class="rigSheen" aria-hidden="true"></span><span class="rigCoolantGlow" aria-hidden="true"></span><span class="rigCoolantGlow rigUpperGlow" aria-hidden="true"></span><span class="rigDrop dropOne" aria-hidden="true"></span><span class="rigDrop dropTwo" aria-hidden="true"></span><span class="rigDrop dropThree" aria-hidden="true"></span></div></div>
</div>
<div class="approvedMetrics">
  <div class="approvedMetric"><span>◉</span><div><small>Отбор</small><strong id="homeCollectionRate">—</strong><small>мл/мин</small></div></div>
  <div class="approvedMetric"><span>▤</span><div><small>Собрано</small><strong id="homeCollectionTotal">—</strong><small id="homeCollectionUnit">мл</small></div></div>
</div>
<div class="relayIndicators" role="status" aria-label="Блок реле"><div class="relayBadge" id="relayBadge0" data-state="unknown"><span class="relayLamp"></span><span>ТЭН откл.</span><small id="soundStatus0">Нет связи</small></div><div class="relayBadge" id="relayBadge1" data-state="unknown"><span class="relayLamp"></span><span>Кран закрыт</span><small id="soundStatus1">Нет связи</small></div><div class="relayBadge" id="relayBadge2" data-state="unknown"><span class="relayLamp"></span><span>Кран открыт</span><small id="soundStatus2">Нет связи</small></div><div class="relayBadge" id="relayBadge3" data-state="unknown"><span class="relayLamp"></span><span>Авария</span><small id="soundStatus3">Нет связи</small></div></div>
<details class="card approvedAdditional"><summary>Охлаждающая вода</summary>
  <div class="row">YF-S401 <b>GPIO15</b></div><div class="row">Импульсы <b id="pulses">—</b></div>
  <div class="row">Расход воды (не калиброван) <b id="flow">—</b></div>
  <p class="hint">Датчик на линии охлаждающей воды, а не на линии отбора. Его расчётный расход не подтверждён калибровкой.</p>
  
  
</details></section>
<section class="page" data-section="1" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg><strong>ТЕМПЕРАТУРЫ</strong></div>
<div class="approvedSensorList"><div class="approvedSensor temp0">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail0" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><path d="M24 4C17 17 7 28 7 37a17 17 0 0034 0C41 28 31 17 24 4Z" fill="url(#busaIconDetail0)" stroke="#28cdff" stroke-width="2"/><path d="M15 35q0 12 12 13" fill="none" stroke="#dcfaff" stroke-width="2.5" stroke-linecap="round"/></svg></div><div class="approvedSensorText"><strong>Вода</strong><small id="detailSet0">Порог: —</small></div><strong id="tempLive0" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary aria-label="Настройка порогов"><span class="srOnly">Пороги</span><span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn0">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn0"></div>
    <div><label for="tempOff0">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff0"></div></div>
  </details></div><div class="approvedSensor temp2">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail2" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="5" y="17" width="38" height="35" rx="7" stroke="#10b9f7" stroke-width="2.5" fill="#083357"/><path d="M8 31q8-5 16 0t16 0v18H8Z" fill="url(#busaIconDetail2)" opacity=".92"/><path d="M14 17V9h20v8" fill="none" stroke="#55d6ff" stroke-width="3"/></svg></div><div class="approvedSensorText"><strong>Куб</strong><small id="detailSet2">Порог: —</small></div><strong id="tempLive2" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary aria-label="Настройка порогов"><span class="srOnly">Пороги</span><span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn2">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn2"></div>
    <div><label for="tempOff2">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff2"></div></div>
  </details></div><div class="approvedSensor temp1">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail1" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIconDetail1)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></div><div class="approvedSensorText"><strong>Колонна</strong><small id="detailSet1">Порог: —</small></div><strong id="tempLive1" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary aria-label="Настройка порогов"><span class="srOnly">Пороги</span><span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn1">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn1"></div>
    <div><label for="tempOff1">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff1"></div></div>
  </details></div><div class="approvedSensor temp3">
  <div class="approvedSensorHead"><div class="approvedSensorIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIconDetail3" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIconDetail3)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></div><div class="approvedSensorText"><strong>Авария</strong><small id="detailSet3">Порог: —</small></div><strong id="tempLive3" class="approvedSensorLive">Н/Д</strong></div>
  <details class="sensorEdit"><summary aria-label="Настройка порогов"><span class="srOnly">Пороги</span><span>›</span></summary>
    <div class="grid approvedFieldGrid"><div><label for="tempOn3">ВКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOn3"></div>
    <div><label for="tempOff3">ВЫКЛ, °C</label><input type="number" inputmode="decimal" min="0" max="125" step="0.1" id="tempOff3"></div></div>
  </details></div></div>
<div class="temperatureSaveRow"><button id="temperatureSave" type="button" class="full">Сохранить пороги</button><p id="temperatureSaveResult" class="hint"></p></div></section>
<section class="page" data-section="2" hidden><div class="approvedPageTitle"><svg class="uiIcon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><path d="M12 2C8 8 5 11 5 15a7 7 0 0 0 14 0c0-4-3-7-7-13z"/></svg><strong>ВОДА И ОТБОР</strong></div>
<article class="liquidPanel collectionPanel"><div class="liquidPanelHeading"><h2>Отбор жидкости</h2><span id="collectionAvailability">Нет датчика</span></div>
<div class="liquidRateGrid"><div><small>Текущий отбор</small><strong id="collectionMlMin">—</strong><small>мл/мин</small></div><div><small>Текущий отбор</small><strong id="collectionLph">—</strong><small>л/ч</small></div></div>
<div class="liquidSummary"><div><span>Собрано</span><strong id="collectionTotal">—</strong></div><div><span>Время отбора, ч:мин</span><strong id="collectionTime">—</strong></div></div></article>
<article class="liquidPanel coolingPanel"><div class="liquidPanelHeading"><h2>Охлаждение</h2><span id="coolingAvailability">Нет данных</span></div>
<div class="liquidRateGrid"><div><small>Расход воды</small><strong id="coolingLph">—</strong><small>л/ч</small></div><div><small>Израсходовано воды</small><strong id="coolingTotal">—</strong><small>л</small></div></div></article>
</section>
<section class="page" data-section="3" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 3v18h18"/><path d="m5 17 5-6 4 3 6-9"/></svg><strong>ГРАФИКИ</strong></div>
<div class="chartCard"><div class="chartTabs" role="tablist" aria-label="Вид графика">
<button type="button" data-chart="0" class="on" aria-selected="true">Температура</button><button type="button" data-chart="1" aria-selected="false">Отбор</button><button type="button" data-chart="2" aria-selected="false">Охлаждение</button></div>
<div class="chartHeading"><strong id="chartTitle">Температура, °C</strong><span id="chartState">Ожидание данных</span></div>
<div id="chartLegend" class="chartLegend"><span style="--series:#38bdf8">Вода</span><span style="--series:#f4c84a">Куб</span><span style="--series:#53d486">Колонна</span><span style="--series:#ff6473">Авария</span></div>
<canvas id="chartCanvas" width="640" height="440" role="img" aria-label="График реальных измерений"></canvas>
<div class="chartRanges" aria-label="Период"><button type="button" data-minutes="5" class="on" aria-pressed="true">5 мин</button><button type="button" data-minutes="10" aria-pressed="false">10 мин</button><button type="button" data-minutes="15" aria-pressed="false">15 мин</button><button type="button" data-minutes="30" aria-pressed="false">30 мин</button><button type="button" data-minutes="60" aria-pressed="false">1 ч</button></div>
</div></section>
<section class="page" data-section="5" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg><strong>НАСТРОЙКИ</strong></div>
<div class="card approvedSettingsMain" id="settingsMenu"><button type="button" data-stab-btn="0" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg></span><span>Температура</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="1" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M2 8c6-5 14-5 20 0M5 12c4-4 10-4 14 0M8 16c2-2 6-2 8 0"/><circle cx="12" cy="20" r="1" fill="currentColor"/></svg></span><span>Wi-Fi</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="2" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg></span><span>Экран</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="3" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M7 19h11a4 4 0 0 0 .5-8A6.5 6.5 0 0 0 6 11a4 4 0 0 0 1 8z"/></svg></span><span>Google</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="7" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="m7 7 10 10-5 4V3l5 4L7 17"/></svg></span><span>Bluetooth · RainMaker</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="5" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 3v12m-5-5 5 5 5-5M4 18v3h16v-3"/></svg></span><span>Обновление ПО</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="8" class="approvedMenuItem"><span class="approvedMenuIcon">▤</span><span>Журнал событий</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="6" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 18h4l3-8 4 6 3-4h4"/><circle cx="12" cy="12" r="10"/></svg></span><span>Диагностика</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="4" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="10"/><path d="M12 11v6M12 7h.01"/></svg></span><span>О системе</span><span class="approvedArrow">›</span></button></div>
<button type="button" id="settingsBack" hidden class="approvedBack">‹ Все настройки</button>
<div class="settingCard" data-stab="8" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><rect x="5" y="3" width="14" height="18" rx="1"/><path d="M9 8h6M9 12h6M9 16h5"/></svg><strong>СОБЫТИЯ</strong></div>
<div class="card">
<div id="events" class="approvedEvents">Ожидание...</div></div></div>

<div class="card settingCard" data-stab="0"><h2>Калибровка температуры</h2>
<p class="hint">Поправки показаний T1–T4, шаг 0,1 °C, от −10 до +10 °C. Это НЕ пороги звуковых сигналов.</p>
<div class="row"><label for="calOffset0">T1 · Вода</label><input id="calOffset0" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset2">T3 · Куб</label><input id="calOffset2" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset1">T2 · Колонна</label><input id="calOffset1" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset3">T4 · Авария</label><input id="calOffset3" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<button id="tempCalSave" type="button" class="full">Сохранить поправки</button>
<p id="tempCalResult" role="status" class="hint">—</p></div>

<div class="card settingCard" data-stab="1" hidden><h2>Wi-Fi</h2>
<div class="busaWifiMode"><h3>Wi-Fi с интернетом</h3>
<div id="wstatus">Проверка подключения...</div>
<label for="net">Выберите сеть</label><select id="net"><option value="">Поиск сетей...</option></select>
<div id="manualSsidRow" hidden><label for="manualSsid">SSID вручную</label><input id="manualSsid" type="text" autocomplete="off" placeholder="Точное имя сети"></div>
<button type="button" id="wifiRetry" class="full">Повторить поиск</button>
<label for="pass">Пароль Wi-Fi</label><input id="pass" type="password" minlength="8" maxlength="63" placeholder="Введите пароль">
<button type="button" class="full" id="save">Сохранить и подключить</button>
<button type="button" class="full" id="disconnectWifi">Отключить Wi-Fi</button>
<p id="response" role="status"></p></div></div>

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
<button id="googleTest" type="button" class="full">Проверить отправку диагностики</button>
<button id="googleStop" type="button" class="full">Отключить передачу в Google</button>
<p id="googleResult" role="status" class="hint">—</p></div>

<div class="card settingCard" data-stab="7" hidden><h2>Bluetooth · ESP RainMaker</h2>
<div class="row">Состояние Bluetooth <b id="rmState">ПРОВЕРКА</b></div>
<div class="row">Имя устройства БУСА <b id="rmService">—</b></div>
<div class="row">Пароль сопряжения (PoP) <b>123456789</b></div>
<p class="hint">БУСА передаёт сигнал BLE, а iPhone находит её в ESP RainMaker. Сопряжение необходимо один раз.</p>
<button type="button" class="full" id="rmStart">Включить Bluetooth · Сопряжение</button>
<button type="button" class="full" id="rmStop">Отключить Bluetooth</button>
<button type="button" class="full" id="rmCheck">Обновить состояние Bluetooth</button>
<p id="rmMessage" class="hint" role="status">Рабочий Wi-Fi сохранён. На время первого BLE-сопряжения локальная сеть БУСА может временно исчезнуть; после завершения или через 3 минуты она восстановится.</p></div>

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
<div class="card" data-diag-panel="relays" hidden><h2>Блок реле</h2>
<pre id="diagRelay" style="white-space:pre-wrap;word-break:break-word">Ожидание статуса...</pre>
<p class="hint">Показано состояние выходов ESP32.</p></div>
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
<section class="page" data-section="7" hidden><div id="busaCalculator"></div></section></main>
<script>
const get=id=>document.getElementById(id),logs=[],tlog=[],flog=[],tlog4=[[],[],[],[]],waterPulseLog=[];
let busaGraphData={temperature:[[],[],[],[]],water_pulses:[]},busaGraphFetchedAt=0,busaGraphLoading=false;
let lastWaterPulse=null,themeInitialized=false,temperatureThresholdsInitialized=false,calInitialized=false;
let active=0,connected=false,pollBusy=false,statusFailures=0,settingsActive=-1,lastLocalNav=0,lastClockPush=0,tftProfileLoaded=false;
let eventRecords=[];
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
      hour:'2-digit',minute:'2-digit',hour12:false});
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
const WEB_JOURNAL_KEY='busa-web-journal-v2';
let webEventCursor=0,webBootId=null,webEventsBusy=false,webMessageSerial=0;
function loadWebJournal(){
 try{const rows=JSON.parse(localStorage.getItem(WEB_JOURNAL_KEY)||'[]');if(Array.isArray(rows))eventRecords=rows.filter(e=>e&&typeof e.text==='string'&&Number.isFinite(e.at)).slice(0,500);}catch(e){}
 drawEvents();
}
function rememberEvents(){try{localStorage.setItem(WEB_JOURNAL_KEY,JSON.stringify(eventRecords.slice(0,500)));}catch(e){}}
function drawEvents(){
 const holder=get('events');holder.replaceChildren();
 if(!eventRecords.length){holder.textContent='Событий пока нет';return;}
 eventRecords.forEach(e=>{
  const row=document.createElement('div');row.className='eventRow '+(e.kind||'system');
  const time=document.createElement('time');time.dateTime=new Date(e.at).toISOString();time.textContent=new Date(e.at).toLocaleTimeString('ru-RU',{hour:'2-digit',minute:'2-digit',hour12:false});
  const dot=document.createElement('span');dot.className='eventDot';
  const label=document.createElement('span');label.textContent=e.text.replace(/ \(P\d+-P\d+\)/g,'');
  row.append(time,dot,label);holder.append(row);
 });
}
function addJournalEntry(e){
 if(e.kind==='temperature'||e.source==='temperature')return;
 if(e.id&&eventRecords.some(x=>x.id===e.id))return;
 eventRecords.push(e);eventRecords.sort((a,b)=>b.at-a.at||(b.seq||0)-(a.seq||0));eventRecords.splice(500);rememberEvents();drawEvents();
}
function message(text,kind='system'){
 const at=Date.now();addJournalEntry({id:'web-'+at+'-'+(++webMessageSerial),at,text,kind,source:'web'});
 logs.unshift(text);logs.splice(12);
}
async function refreshWebEvents(){
 if(webEventsBusy)return;webEventsBusy=true;
 try{
  const r=await fetch('/api/web-events?after='+webEventCursor,{cache:'no-store',signal:AbortSignal.timeout(3500)});if(!r.ok)return;const j=await r.json();
  if(webBootId!==null&&webBootId!==j.boot_id){webEventCursor=0;webBootId=j.boot_id;return;}
  webBootId=j.boot_id;
  for(const e of (j.events||[])){
   if(typeof e.text!=='string'||!Number.isFinite(e.id))continue;
   addJournalEntry({id:'esp-'+j.boot_id+'-'+e.id,seq:e.id,at:e.epoch?e.epoch*1000:Date.now()-(Number.isFinite(j.uptime_ms)&&Number.isFinite(e.uptime_ms)?((j.uptime_ms-e.uptime_ms)>>>0):0),text:e.text,kind:e.kind===2?'alarm':e.kind===1?'warn':'system',source:'device'});webEventCursor=Math.max(webEventCursor,e.id);
  }
 }catch(e){}finally{webEventsBusy=false;}
}
const nativeWebFetch=window.fetch.bind(window);
window.fetch=async function(input,options){
 const url=typeof input==='string'?input:input.url,method=(options?.method||input?.method||'GET').toUpperCase();
 const names={'/api/time/sync':'Синхронизация времени','/api/wifi':'Настройки Wi-Fi','/api/wifi/disconnect':'Отключение Wi-Fi','/api/temperature-alarms':'Пороги сигнализации','/api/temperature/calibration':'Поправки датчиков','/api/display/power':'Экран','/api/theme':'Тема экрана'};
 const label=names[url.split('?')[0]]||'Действие';
 try{const r=await nativeWebFetch(input,options);if(method==='POST')message(label+(r.ok?': выполнено':': ошибка'),r.ok?'system':'warn');return r;}
 catch(e){if(method==='POST')message(label+': нет связи','warn');throw e;}
};
document.addEventListener('click',e=>{
 const button=e.target.closest('button');if(!button||button.matches('.appTabs button,[data-stab-btn],[data-chart],[data-minutes],#settingsBack'))return;
 const title=button.textContent.replace(/\s+/g,' ').trim();if(title)message('Действие: '+title);
});

document.querySelectorAll('[data-diag-btn]').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('[data-diag-btn]').forEach(x=>x.classList.toggle('on',x===b));
 document.querySelectorAll('[data-diag-panel]').forEach(x=>x.hidden=x.dataset.diagPanel!==b.dataset.diagBtn);
});
function page(n,send=true){
 const inDiag=n===6,inEvents=n===4,visible=(inDiag||inEvents)?5:n;
 active=visible;
 document.querySelectorAll('[data-section]').forEach(e=>e.hidden=Number(e.dataset.section)!==visible);
 document.querySelectorAll('.appTabs button').forEach(e=>e.classList.toggle('on',Number(e.dataset.page)===visible));
 if(inDiag)settingsTab(6,false);
 else if(inEvents)settingsTab(8,false);
 else if(visible===5&&send)settingsTab(-1,false);
 // Browser page selection never commands the physical TFT.
 if(visible===3){graphs();refreshGraphHistory(true);}
 if(send)message('Открыт раздел: '+({0:'Главная',1:'Температуры',2:'Вода и отбор',3:'Графики',7:'Калькулятор',5:'Настройки'}[visible]||'Настройки'));
}
function settingsTab(n,send=true){
 const previous=settingsActive;
 settingsActive=n;
 get('settingsMenu').hidden=n!==-1;
 get('settingsBack').hidden=n===-1;
 document.querySelectorAll('.settingCard').forEach(e=>e.hidden=Number(e.dataset.stab)!==n);
 document.querySelectorAll('[data-stab-btn]').forEach(e=>e.classList.toggle('on',Number(e.dataset.stabBtn)===n));
 if(n===1&&previous!==1)wifiRefreshNetworks();
 if(n===7&&previous!==7)rainmakerRefreshStatus();
 if(n===8)refreshWebEvents();
 if(send&&n!==-1)message('Открыты настройки: '+({0:'Температура',1:'Wi-Fi',2:'Экран',3:'Google',4:'О системе',5:'Обновление ПО',6:'Диагностика',7:'Bluetooth',8:'Журнал событий'}[n]||'Настройки'));
 // Local settings stay in browser; physical TFT navigation is independent.
}
get('settingsBack').onclick=()=>{
 settingsTab(-1,false);
};
document.querySelectorAll('[data-stab-btn]').forEach(b=>b.onclick=()=>settingsTab(Number(b.dataset.stabBtn)));
let chartView=0,chartMinutes=5,chartFetchToken=0,chartRequestController=null;
const CHART_REFRESH_MS=2000;
const DISPLAY_ORDER=[0,2,1,3];
const SENSOR_NAMES=['Вода','Колонна','Куб','Авария'];
const SENSOR_COLORS=['#38bdf8','#53d486','#f4c84a','#ff6473'];
const decimal2=new Intl.NumberFormat('ru-RU',{minimumFractionDigits:2,maximumFractionDigits:2});
let previousWater=null;
function measured(v){return typeof v==='number'&&Number.isFinite(v)&&v>=0;}
function formatVolumeMl(v){if(!measured(v))return '—';return v<1000?Math.floor(v).toLocaleString('ru-RU')+' мл':decimal2.format(v/1000)+' л';}
function formatRunTime(ms){if(!measured(ms))return '—';const minutes=Math.floor(ms/60000);return String(Math.floor(minutes/60)).padStart(2,'0')+':'+String(minutes%60).padStart(2,'0');}
function setMeasured(id,v){get(id).textContent=measured(v)?decimal2.format(v):'—';}
function updateLiquidPanels(s){
 const c=s?.collection,collection=!!c?.sensor_present;
 const mlMin=collection&&measured(c.ml_per_min)?c.ml_per_min:null,total=collection&&measured(c.total_ml)?c.total_ml:null;
 setMeasured('collectionMlMin',mlMin);setMeasured('collectionLph',measured(mlMin)?mlMin*.06:null);
 get('collectionTotal').textContent=formatVolumeMl(total);
 get('collectionTime').textContent=formatRunTime(collection?c.duration_ms??c.elapsed_ms:null);
 get('collectionAvailability').textContent=!s?'Нет связи':collection?'Данные датчика':'Нет датчика';
 setMeasured('homeCollectionRate',mlMin);
 const volume=formatVolumeMl(total),split=volume.lastIndexOf(' ');
 get('homeCollectionTotal').textContent=split<0?volume:volume.slice(0,split);get('homeCollectionUnit').textContent=split<0?'мл':volume.slice(split+1);
 const w=s?.water,ready=w?.seen_pulses===true&&measured(w?.total_l_estimate),uptime=s?.uptime_ms;
 let lph=null;
 if(ready&&previousWater&&measured(uptime)&&uptime>previousWater.at&&w.total_l_estimate>=previousWater.total)
  lph=(w.total_l_estimate-previousWater.total)*3600000/(uptime-previousWater.at);
 if(ready&&measured(uptime))previousWater={at:uptime,total:w.total_l_estimate};else previousWater=null;
 setMeasured('coolingLph',lph);setMeasured('coolingTotal',ready?w.total_l_estimate:null);
 get('coolingAvailability').textContent=!s?'Нет связи':ready?(w.calibrated?'Данные датчика':'Расчётно · без калибровки'):'Нет импульсов';
 get('rigArtStage').classList.toggle('hasCollection',measured(mlMin)&&mlMin>0);
 get('rigArtStage').classList.toggle('hasCooling',measured(lph)&&lph>0);
}

const RELAY_NAMES=['ТЭН откл.','Кран закрыт','Кран открыт','Авария'];
function updateRelayBadges(channels){
 for(let i=0;i<4;i++){
  const ch=Array.isArray(channels)?channels[i]:null,badge=get('relayBadge'+i);
  const state=typeof ch?.sound_on==='boolean'?(ch.sound_on?'on':'off'):'unknown';
  badge.dataset.state=state;get('soundStatus'+i).textContent=state==='on'?'Активен':state==='off'?'Неактивен':'Нет связи';
 }
}
document.querySelectorAll('[data-chart]').forEach(b=>b.onclick=()=>{
 chartView=Number(b.dataset.chart);message('График: '+['Температура','Отбор','Охлаждение'][chartView]);document.querySelectorAll('[data-chart]').forEach(x=>{x.classList.toggle('on',x===b);x.setAttribute('aria-selected',String(x===b));});graphs();
});
document.querySelectorAll('[data-minutes]').forEach(b=>b.onclick=()=>{
 chartMinutes=Number(b.dataset.minutes);message('Масштаб графика: '+(chartMinutes===60?'1 ч':chartMinutes+' мин'));document.querySelectorAll('[data-minutes]').forEach(x=>{x.classList.toggle('on',x===b);x.setAttribute('aria-pressed',String(x===b));});busaGraphData={};graphs();refreshGraphHistory(true);
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
 if(!confirm('Сохранить профиль TFT и перезапустить ESP32? Блок реле временно выключатся на перезагрузке.'))return;
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
async function rainmakerRefreshStatus(){
 try{
  const resp=await fetch('/api/rainmaker/status',{cache:'no-store',signal:AbortSignal.timeout(6500)});
  if(!resp.ok)throw Error('HTTP '+resp.status);
  const d=await resp.json();
  const active=d.ble==='advertising'||d.ble==='starting'||d.ble==='stopping';
  get('rmState').textContent=d.ble==='advertising'?'ВКЛ · ОЖИДАЕТ iPHONE':
    d.ble==='starting'?'ЗАПУСКАЕТСЯ':d.ble==='stopping'?'ВЫКЛЮЧАЕТСЯ':
    d.ble==='done'?'СОПРЯЖЕНИЕ ЗАВЕРШЕНО':d.ble==='error'?'ОШИБКА':'ВЫКЛ';
  get('rmService').textContent=d.service||'BUSA_XXXX';
  get('rmStart').disabled=active||d.ble==='done';
  get('rmStop').disabled=!active;
  if(d.feedback)get('rmMessage').textContent=d.feedback;
 }catch(e){
  get('rmState').textContent='НЕТ СВЯЗИ С БУСА';
  get('rmMessage').textContent='Во время BLE-сопряжения локальная Wi-Fi сеть временно недоступна. Открой ESP RainMaker и добавь устройство; после сопряжения сеть восстановится.';
 }
}
get('rmStart').onclick=async function(){
 this.disabled=true;get('rmState').textContent='ЗАПУСКАЕТСЯ';
 get('rmMessage').textContent='Включаю Bluetooth. Дальше открой ESP RainMaker → Добавить устройство → Bluetooth, выбери PROV_BUSA_XXXX и введи 123456789.';
 try{
  const resp=await fetch('/api/rainmaker/enable',{method:'POST',
    body:new URLSearchParams({confirm:'YES'}),cache:'no-store',signal:AbortSignal.timeout(7000)});
  const d=await resp.json();
  if(!resp.ok)throw Error(d.error||'HTTP '+resp.status);
  get('rmMessage').textContent='Запуск принят. Перейди в ESP RainMaker на iPhone. Пароль BLE: 123456789. Временный разрыв локального Wi-Fi возможен.';
 }catch(e){
  get('rmMessage').textContent='Ошибка запуска: '+e.message;this.disabled=false;
 }
};
get('rmStop').onclick=async function(){
 this.disabled=true;
 try{
  const resp=await fetch('/api/rainmaker/disable',{method:'POST',
    body:new URLSearchParams({confirm:'YES'}),cache:'no-store',signal:AbortSignal.timeout(7000)});
  const d=await resp.json();
  if(!resp.ok)throw Error(d.error||'HTTP '+resp.status);
  get('rmMessage').textContent='Команда отключения BLE принята. Локальная сеть БУСА останется/станет доступной.';
 }catch(e){get('rmMessage').textContent='Ошибка отключения: '+e.message}
 await rainmakerRefreshStatus();
};
get('rmCheck').onclick=()=>rainmakerRefreshStatus();
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
// Shared one-hour passive measurement history; missing data never becomes a demo line.
function graphs(){
 const canvas=get('chartCanvas'),ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
 const x0=58,y0=28,pw=w-82,ph=h-83;ctx.clearRect(0,0,w,h);ctx.fillStyle='#04101e';ctx.fillRect(0,0,w,h);
 const titles=['Температура, °C','Отбор, мл/мин','Охлаждение, л/ч'];get('chartTitle').textContent=titles[chartView];get('chartLegend').hidden=chartView!==0;
 const data=busaGraphData.minutes===chartMinutes?busaGraphData:{},temp=data.temperature||[];
 const chosen=chartView===1?data.collection_ml_min||[]:data.cooling_lph||[];
 const valid=chosen.filter(Number.isFinite),high=chartView===0?125:Math.max(10,Math.ceil(Math.max(0,...valid)/10)*10),now=Date.now();
 ctx.lineWidth=1;ctx.font='18px system-ui';ctx.textAlign='right';ctx.textBaseline='middle';
 for(let j=0;j<=5;j++){const yy=y0+ph-j*ph/5;ctx.strokeStyle='#1b354a';ctx.beginPath();ctx.moveTo(x0,yy);ctx.lineTo(x0+pw,yy);ctx.stroke();ctx.fillStyle='#90aec3';ctx.fillText(String(Math.round(j*high/5*100)/100),x0-10,yy);}
 ctx.textAlign='center';ctx.textBaseline='top';
 for(let j=0;j<=3;j++){const xx=x0+pw*j/3;ctx.strokeStyle='#142b3e';ctx.beginPath();ctx.moveTo(xx,y0);ctx.lineTo(xx,y0+ph);ctx.stroke();ctx.fillStyle='#90aec3';ctx.fillText(new Date(now-(1-j/3)*chartMinutes*60000).toLocaleTimeString('ru-RU',{hour:'2-digit',minute:'2-digit',hour12:false}),xx,y0+ph+15);}
 let points=0;
 function line(values,color){
  ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;ctx.lineJoin='round';ctx.beginPath();let drawing=false;
  values.forEach((v,i)=>{if(!Number.isFinite(v)){drawing=false;return;}points++;const xx=x0+pw*(i+.5)/values.length,yy=y0+ph-Math.max(0,Math.min(high,v))*ph/high;if(drawing)ctx.lineTo(xx,yy);else ctx.moveTo(xx,yy);drawing=true;});ctx.stroke();
  values.forEach((v,i)=>{if(Number.isFinite(v)){ctx.beginPath();ctx.arc(x0+pw*(i+.5)/values.length,y0+ph-Math.max(0,Math.min(high,v))*ph/high,2.7,0,Math.PI*2);ctx.fill();}});
 }
 if(chartView===0)DISPLAY_ORDER.forEach(k=>line(temp[k]||[],SENSOR_COLORS[k]));else line(chosen,chartView===1?'#f4c84a':'#38bdf8');
 const note=chartView===1&&!data.collection_sensor?'Нет датчика отбора':points?'':chartView===2?'Нет импульсов охлаждения':'Ожидание измерений';
 get('chartState').textContent=note||(chartView===2&&!data.cooling_calibrated?'Расчётно · без калибровки':'Данные с включения');
 if(note){ctx.fillStyle='#a6c2d5';ctx.font='21px system-ui';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(note,x0+pw/2,y0+ph/2);}
}

async function refreshGraphHistory(force=false){
 if(document.hidden&&!force)return;
 if(chartRequestController){if(!force)return;chartRequestController.abort();}
 const controller=new AbortController();chartRequestController=controller;
 const token=++chartFetchToken,minutes=chartMinutes;busaGraphFetchedAt=Date.now();
 const timeout=setTimeout(()=>controller.abort(),1800);
 try{
  const r=await fetch('/api/chart-history?minutes='+minutes,{cache:'no-store',signal:controller.signal});
  if(!r.ok)throw Error(r.status);const j=await r.json();
  if(token===chartFetchToken&&j.ok&&j.minutes===chartMinutes&&Array.isArray(j.temperature)){busaGraphData=j;if(active===3)graphs();}
 }catch(e){if(token===chartFetchToken&&active===3)get('chartState').textContent='Нет связи';}
 finally{clearTimeout(timeout);if(chartRequestController===controller)chartRequestController=null;}
}
async function update(){if(pollBusy)return;pollBusy=true;try{let r=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(6000)});if(!r.ok)throw Error(r.status);let s=await r.json();if(!connected)message('Связь с ESP32 восстановлена');connected=true;statusFailures=0;
networkLeds(s.network_indicators,s.station,s.google);refreshWebEvents();
 // Physical buttons and browser navigation are independent.
 let names=['Вода','Колонна','Куб','Авария'];
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
updateRelayBadges(s.sound_alerts?.channels);
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?'НЕТ ДАННЫХ':f.toFixed(2);get('pulses').textContent=s.water.pulses;updateLiquidPanels(s);
flog.push(f);if(flog.length>80)flog.shift();
const currentPulse=Number(s.water.pulses);
if(Number.isFinite(currentPulse)){
 let delta=lastWaterPulse===null?null:Math.max(0,currentPulse-lastWaterPulse);
 lastWaterPulse=currentPulse;waterPulseLog.push(delta);
 if(waterPulseLog.length>80)waterPulseLog.shift();
}
let ss=s.station.connected?'ПОДКЛЮЧЕНО: '+s.station.ssid+' · '+s.station.ip:
 s.station.last_error?'НЕ ПОДКЛЮЧЕНО: '+s.station.last_error:
 s.station.connecting?'Подключаю: '+(s.station.target_ssid||'выбранную сеть'):
 s.diagnostics?.wifi?.sta_disabled_by_user?'Внешняя сеть отключена':
 'НЕТ СОЕДИНЕНИЯ'+(s.station.target_ssid?' · выбрана '+s.station.target_ssid:'');
get('wstatus').textContent=ss;

// The browser's SSID picker is never automatically populated with old iPhone.
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
get('aboutGoogle').textContent=!s.google.configured?'Не настроена':s.google.state===6?'Ошибка':'Настроена';
get('aboutFirmware').textContent=s.firmware.replace(/^БУСА\s*1М\s*WiFi\s*/, '');
get('otaFirmware').textContent=s.firmware.replace(/^БУСА\s*1М\s*WiFi\s*/, '');
get('aboutUptime').textContent=Math.floor(s.uptime_ms/3600000)+' ч '+Math.floor((s.uptime_ms%3600000)/60000)+' мин';
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
     RELAY_NAMES[c.channel-1]+' · '+(c.sound_on?'Активен':'Неактивен')+
     ' · ВКЛ '+(c.on_c??'—')+' °C · ВЫКЛ '+(c.off_c??'—')+
     ' °C').join('\n')+
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
get('ver').textContent=s.firmware;get('uptime').textContent=Math.floor(s.uptime_ms/60000)+' мин';get('pcf').textContent=s.modules.pcf8575?'Найден':'Не найден';get('tft').textContent=s.modules.tft_attempted?('Контроллер '+s.modules.tft_profile+', поворот '+s.modules.tft_rotation+', загрузка '+(s.modules.tft_boot_complete?'завершена':'идёт')+' (проверка изображения визуальная)'):'Запуск ещё не выполнен';if(!tftProfileLoaded){get('tftDriver').value=String(s.modules.tft_profile_id||1);get('tftRotation').value=String(s.modules.tft_rotation===undefined?1:s.modules.tft_rotation);tftProfileLoaded=true}
}catch(e){statusFailures++;if(statusFailures>=3){if(connected)message('Нет ответа ESP32: проверь локальный Wi-Fi');connected=false;networkLeds(null,'');updateRelayBadges(null);updateLiquidPanels(null)}}finally{pollBusy=false}}
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
  if(!confirm('Подтвердите ручное обновление диагностической прошивки. ESP32-S3 N16R8, блок реле временно отключаются, выбран APP BIN?'))return;
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
let wifiListBusy=false,wifiRescanAfterDisconnect=false;
// Single bounded radio scan per opening of the Wi-Fi page.
function wifiManualVisibility(){get('manualSsidRow').hidden=get('net').value!=='__manual__';}
function wifiScanMessage(j){
 const names={scan_start_failed:'Не удалось запустить сканер',scan_driver_failed:'Ошибка драйвера поиска',scan_results_unavailable:'Результаты сканера недоступны',scan_timeout:'Поиск превысил время ожидания',sta_unavailable:'Режим AP+STA недоступен',scan_event_unavailable:'Обработчик поиска не готов',scan_cancelled_ota:'Поиск остановлен на время OTA',busy:'Устройство занято',only_hidden_networks:'Обнаружены только скрытые сети'};
 if(!j.error||j.error==='no_networks')return 'Доступные сети 2,4 ГГц не найдены. Можно ввести SSID вручную.';
 const detail=j.diagnostic?(' · код '+j.diagnostic.start_result+', событие '+j.diagnostic.event_status):'';
 return (names[j.error]||'Ошибка поиска Wi-Fi: '+j.error)+detail;
}
async function wifiRefreshNetworks(fresh=false){
 if(wifiListBusy||settingsActive!==1)return;
 wifiListBusy=true;get('wifiRetry').disabled=true;
 const sel=get('net'),keepManual=sel.value==='__manual__';
 sel.replaceChildren(new Option('Поиск сетей...',''));
 get('response').textContent='Поиск сетей...';
 try{
  const start=await fetch('/api/wifi/scan?start=1'+(fresh?'&fresh=1':''),{cache:'no-store',signal:AbortSignal.timeout(7000)});
  let j=await start.json();if(!start.ok)throw Error(wifiScanMessage(j));
  for(let i=0;i<30&&j.scanning;i++){
   if(settingsActive!==1)return;await new Promise(resolve=>setTimeout(resolve,650));
   try{const resp=await fetch('/api/wifi/scan',{cache:'no-store',signal:AbortSignal.timeout(6500)});if(resp.ok)j=await resp.json();}catch(e){}
  }
  if(settingsActive!==1)return;if(j.scanning)throw Error('Поиск превысил время ожидания');
  const unique=new Set();sel.replaceChildren(new Option('Выберите сеть',''));
  (j.networks||[]).forEach(item=>{if(item.ssid&&!unique.has(item.ssid)){unique.add(item.ssid);sel.add(new Option(item.ssid,item.ssid));}});
  sel.add(new Option('Ввести SSID вручную','__manual__'));if(keepManual||!unique.size)sel.value='__manual__';wifiManualVisibility();
  get('response').textContent=unique.size?'':wifiScanMessage(j);
 }catch(e){
  if(settingsActive===1){sel.replaceChildren(new Option('Ввести SSID вручную','__manual__'));wifiManualVisibility();get('response').textContent='Ошибка поиска Wi-Fi: '+e.message;}
 }finally{
  wifiListBusy=false;get('wifiRetry').disabled=false;
  if(wifiRescanAfterDisconnect&&settingsActive===1){wifiRescanAfterDisconnect=false;await new Promise(resolve=>setTimeout(resolve,650));if(settingsActive===1)wifiRefreshNetworks(true);}
 }
}
get('net').onchange=()=>{wifiManualVisibility();get('response').textContent='';};
get('wifiRetry').onclick=()=>wifiRefreshNetworks(true);
get('disconnectWifi').onclick=async function(){
  this.disabled=true;
  get('response').textContent='Отключаю Wi-Fi...';
  try{
    const r=await fetch('/api/wifi/disconnect',{method:'POST',
      cache:'no-store',signal:AbortSignal.timeout(9000)});
    const reply=await r.text();
    if(!r.ok)throw Error(reply||'HTTP '+r.status);
    get('response').textContent=reply;
    get('wstatus').textContent='НЕТ СОЕДИНЕНИЯ';
    get('pass').value='';
    get('net').replaceChildren(new Option('Выберите сеть',''));
    // An in-flight scan finishes first; then one fresh scan runs on disconnected STA.
    if(wifiListBusy)wifiRescanAfterDisconnect=true;
    else{
      await new Promise(resolve=>setTimeout(resolve,750));
      if(settingsActive===1)wifiRefreshNetworks();
    }
  }catch(e){get('response').textContent='Ошибка отключения: '+e.message;}
  this.disabled=false;
};
get('save').onclick=async function(){
 let ssid=(get('net').value==='__manual__'?get('manualSsid').value:get('net').value).trim(),pass=get('pass').value;
 const encoder=new TextEncoder(),ssidBytes=encoder.encode(ssid).length,passBytes=encoder.encode(pass).length;
 if(!ssid||ssidBytes>32||passBytes<8||passBytes>63){
  get('response').textContent='Выберите сеть или введите SSID (до 32 байт), пароль 8–63 байта.';return;
 }
 this.disabled=true;get('response').textContent='Подключаю к '+ssid+'...';
 try{
  let r=await fetch('/api/wifi',{method:'POST',body:new URLSearchParams({ssid,pass}),signal:AbortSignal.timeout(12000)});
  const reply=await r.text();
  if(!r.ok)throw Error(reply||'HTTP '+r.status);
  get('response').textContent=reply;
  get('pass').value='';message('Новая сеть выбрана: '+ssid);
 }catch(e){get('response').textContent='Ошибка: '+e.message}
 this.disabled=false;
};
document.documentElement.dataset.theme='system';
loadWebJournal();settingsTab(-1,false);
message('Веб-приложение запущено');
showPhoneClock();setInterval(showPhoneClock,1000);
 // No endless scan timer. Reopening Wi-Fi requests a new finite search.
syncClock();setInterval(syncClock,300000);
update();setInterval(update,3000);
// Graph cadence is fixed and independent of status polling and selected time range.
setInterval(()=>{if(active===3&&!document.hidden)refreshGraphHistory();},CHART_REFRESH_MS);
document.addEventListener('visibilitychange',()=>{if(!document.hidden&&active===3)refreshGraphHistory(true);});
</script><script src="/calculator.js?v=CALC1"></script></body></html>
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
#include "BUSA_RainMaker.h"
static bool busaRmBleBootMode=false;
static bool busaRmAutoCloud=false;
static uint32_t busaRmCloudBootAt=0;
static bool busaRmBleBootError=false;
#include "BUSA_OtaBoot.h"
#include "BUSA_BleBootRecovery.h"
static bool busaRmPairDeferred=false;
static bool busaNvsReady=false;
// The local app has its own Wi-Fi Preferences. Sync only a successful explicit
// provisioning result, on the main task, without printing credentials.
static bool busaSaveProvisionedStation(){
  wifi_config_t cfg={};
  if(esp_wifi_get_config(WIFI_IF_STA,&cfg)!=ESP_OK)return false;
  char ssid[33]={},password[65]={};
  memcpy(ssid,cfg.sta.ssid,strnlen((const char*)cfg.sta.ssid,sizeof(cfg.sta.ssid)));
  memcpy(password,cfg.sta.password,strnlen((const char*)cfg.sta.password,sizeof(cfg.sta.password)));
  const String nextSsid(ssid),nextPass(password);
  // Preserve the local app's existing supported network/password constraints.
  if(nextSsid.isEmpty()||nextPass.length()<8||nextPass.length()>63)return false;
  const String oldSsid=prefs.getString("ssid",""),oldPass=prefs.getString("pass","");
  prefs.putString("ssid",nextSsid);prefs.putString("pass",nextPass);
  const bool saved=prefs.getString("ssid","")==nextSsid&&prefs.getString("pass","")==nextPass;
  if(!saved){prefs.putString("ssid",oldSsid);prefs.putString("pass",oldPass);}
  // staOff is deliberately untouched: never override an explicit local STA-off choice.
  return saved;
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
  busaNvsReady=prefs.begin("busa-r04",false);
  // A pending OTA must first pass the normal local boot self-test. Keep the
  // request in NVS; confirm below, then reboot into the existing isolated BLE mode.
  busaRmPairDeferred=busaNvsReady&&prefs.getBool("rmPairNext",false)&&busaOtaPendingVerification();
  if(busaNvsReady&&prefs.getBool("rmPairNext",false)&&!busaRmPairDeferred){
    prefs.putBool("rmPairNext",false);
    prefs.putString("rmStage","BLE_BOOT");
    busaRmBleBootMode=true;
    busaRmBleSessionAt=millis();
    busaRainMakerStart();
    if(busaRmStarted)busaRainMakerProvision();
    else {prefs.putString("rmStage","BLE_RMAKER_ERROR");busaRmBleBootError=true;}
    return;
  }
  const esp_reset_reason_t resetReason=esp_reset_reason();
  const bool cloudCrash=resetReason==ESP_RST_PANIC||resetReason==ESP_RST_INT_WDT||
    resetReason==ESP_RST_TASK_WDT||resetReason==ESP_RST_WDT||resetReason==ESP_RST_BROWNOUT;
  if(prefs.getBool("rmCloudBusy",false)&&cloudCrash){
    prefs.putBool("rmCloudBusy",false);
    prefs.putBool("rmCloudOn",false);
    prefs.putString("rmStage","CLOUD_RECOVERED");
  }else if(prefs.getBool("rmCloudOn",false)){
    prefs.putBool("rmCloudBusy",false);
    // Only start cloud services after local Wi-Fi and HTTP are ready.
    busaRmAutoCloud=true;
    busaRmCloudBootAt=millis();
  }

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
  snprintf(busaRmService,sizeof(busaRmService),"PROV_BUSA_%04X",(unsigned)(ESP.getEfuseMac()&0xFFFF));
  startedAt=millis();
  tftProfile=1; // ST7789 only
  // Preserve original V14.7 splash-first boot; time every operation.
  busaReportStage("TFT_BEGIN");
  screenAttempted=true;
  startScreen();
  busaReportStage("TFT_READY");
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  wifiUiStaDisabled=prefs.getBool("staOff",false); // AP remains available even when STA is disabled
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
  // RainMaker starts after the first local TFT frame; no startup dependency.
  busaReportStage("WIFI_BEGIN");
  busaWifiInitScanner();
  WiFi.mode(WIFI_AP_STA);
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
  Serial0.printf("AP=%d IP=%s SSID=BUSA-1M-001-R04 password hidden\n",
                 (int)wifiRunning,WiFi.softAPIP().toString().c_str());
  if(!wifiUiStaDisabled && stationSsid.length() && stationPass.length()>=8){
    WiFi.setAutoReconnect(false); // no endless reconnect to a vanished network
    WiFi.begin(stationSsid.c_str(),stationPass.c_str());
    wifiUiConnectAt=millis();wifiUiStage=WIFIUI_CONNECTING;
    Serial0.printf("STA connect requested: SSID=%s; password is hidden\n",stationSsid.c_str());
  }else{
    WiFi.setAutoReconnect(false);
    Serial0.println(wifiUiStaDisabled?
      "STA manually disabled in NVS; BUSA softAP remains ON":
      "STA credentials missing; BUSA softAP remains ON");
  }
  server.on("/calculator.js",HTTP_GET,[]{server.sendHeader("Cache-Control","no-cache");server.send_P(200,"application/javascript; charset=utf-8",BUSA_CALCULATOR_JS);});
  server.on("/calculator.css",HTTP_GET,[]{server.sendHeader("Cache-Control","no-cache");server.send_P(200,"text/css; charset=utf-8",BUSA_CALCULATOR_CSS);});
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
    if(!apClient()||server.arg("confirm")!="YES"||otaActive){
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
  server.on("/api/wifi/scan",HTTP_GET,[]{
    if(!apClient()){
      server.send(403,"application/json","{\"error\":\"local_only\"}");return;
    }
    // HTTP is read-only except for requesting a single async scan.
    // Never manipulate the radio inside the browser request.
    if(server.arg("start")=="1" && !busaWifiRequestSharedScan(false,server.arg("fresh")=="1")){
      server.send(409,"application/json",
        "{\"scanning\":false,\"networks\":[],\"error\":\"busy\"}");return;
    }
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",busaWifiScanJSON());
  });
  // Phone-local Wi-Fi disconnect: never disable the BUSA access point.
  // The same saved 'staOff' flag is used by the existing physical disconnect.
  server.on("/api/wifi/disconnect",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная сеть БУСА.");return;
    }
    if(otaActive){
      server.send(409,"text/plain; charset=utf-8","Дождитесь завершения обновления.");return;
    }
    if(!prefs.putBool("staOff",true) || !prefs.getBool("staOff",false)){
      server.send(500,"text/plain; charset=utf-8","Ошибка сохранения отключения Wi-Fi.");return;
    }
    wifiUiStaDisabled=true;wifiUiDisablePending=true;
    wifiUiConnectPending=false;wifiUiAssociationDueAt=0;wifiUiConnectAt=0;
    wifiUiResumeStation=false;wifiUiLastConnectError="";
    WiFi.setAutoReconnect(false);
    // Don't reuse a cached scan that ran on the previous router's channel.
    if(busaSharedScanState==BUSA_SCAN_READY ||
       busaSharedScanState==BUSA_SCAN_EMPTY ||
       busaSharedScanState==BUSA_SCAN_ERROR)
      busaSharedScanState=BUSA_SCAN_IDLE;
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"text/plain; charset=utf-8",
      "Внешний Wi-Fi отключён. Локальная сеть БУСА работает.");
  });
  server.on("/api/wifi",HTTP_POST,[]{
    // Credentials may only be provisioned while connected to the protected local AP.
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"text/plain; charset=utf-8","Настройка Wi-Fi доступна только через сеть БУСА.");return;
    }
    if(busaWifiScanRunning || wifiUiOwnsScan || otaActive){
      server.send(409,"text/plain; charset=utf-8",
        "Поиск сети или обновление выполняется. Повтори после завершения.");return;
    }
    String ssid=server.arg("ssid"),pass=server.arg("pass");
    ssid.trim();
    if(ssid.isEmpty()||ssid.length()>32||pass.length()<8||pass.length()>63){
      server.send(400,"text/plain; charset=utf-8","Недопустимое имя сети или длина пароля (8–63).");return;
    }
    // NVS is encrypted only if board flash encryption is enabled; keep local AP secure.
    // Local app uses the same NVS flag as TFT. A saved network re-enables
    // only the external STA; the local AP never stops during provisioning.
    const String prevSsid=prefs.getString("ssid","");
    const String prevPass=prefs.getString("pass","");
    const bool prevOff=prefs.getBool("staOff",false);
    const bool ok=(prefs.putString("pass",pass)==pass.length() &&
      prefs.putString("ssid",ssid)==ssid.length() &&
      prefs.putBool("staOff",false));
    if(!ok){
      prefs.putString("ssid",prevSsid);prefs.putString("pass",prevPass);
      prefs.putBool("staOff",prevOff);
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить Wi-Fi настройки");return;
    }
    wifiUiResetReconnectBackoff();
    stationSsid=ssid;stationPass=pass;wifiUiStaDisabled=false;wifiUiDisablePending=false;
    wifiUiLastConnectError="";wifiUiResumeStation=false;
    wifiUiStage=WIFIUI_CONNECTING; // shared network state, NOT navigation state
    wifiUiConnectPending=true;wifiUiAssociationDueAt=0;
    wifiUiConnectAt=0;lastStationCheck=millis();
    server.send(200,"text/plain; charset=utf-8","Выбрана сеть: "+ssid+". Подключение запускается.");
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>7){server.send(400,"application/json","{\"error\":\"page 1..7\"}");return;}
    // Legacy browser callers must not alter TFT navigation.
    server.send(200,"application/json","{\"ok\":true,\"scope\":\"browser_only\"}");
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
  server.on("/api/web-events",HTTP_GET,[]{
   const uint32_t after=(uint32_t)strtoul(server.arg("after").c_str(),nullptr,10);
   String result="{\"boot_id\":"+String(busaWebBootId)+",\"uptime_ms\":"+String(millis())+",\"events\":[";bool comma=false;
   for(uint16_t i=0;i<busaWebJournal.size();i++){
    const auto* e=busaWebJournal.latest(i);if(!e||e->seq<=after)continue;
    if(comma)result+=",";comma=true;
    result+="{\"id\":"+String(e->seq)+",\"uptime_ms\":"+String(e->uptime)+",\"epoch\":"+String(e->epoch)+",\"kind\":"+String(e->kind)+",\"text\":"+quoteJson(String(e->text))+"}";
   }
   result+="]}";server.send(200,"application/json; charset=utf-8",result);
  });
  server.on("/api/chart-history",HTTP_GET,[]{
    const uint32_t now=millis();int minutes=server.arg("minutes").toInt();
    if(minutes!=5&&minutes!=10&&minutes!=15&&minutes!=30&&minutes!=60){server.send(400,"application/json","{\"error\":\"minutes\"}");return;}
    const int bins=min(minutes*30,160);int16_t selected[160];
    busaChartHistory.selectMs(now,(uint32_t)minutes*60000u,selected,bins);
    String j="{\"ok\":true,\"minutes\":"+String(minutes)+",\"temperature\":[";
    for(int k=0;k<4;k++){
      if(k)j+=",";j+="[";
      for(int b=0;b<bins;b++){if(b)j+=",";float v=busaChartHistory.value(selected[b],k);j+=isfinite(v)?String(v,2):String("null");}
      j+="]";
    }
    j+="],\"cooling_lph\":[";
    for(int i=0;i<bins;i++){if(i)j+=",";float value=busaChartHistory.coolingValue(selected[i]);j+=isfinite(value)?String(value,2):String("null");}
    j+="],\"cooling_calibrated\":false,\"collection_sensor\":false,\"history_ready\":"+String(busaChartReady?"true":"false")+"}";
    server.send(200,"application/json; charset=utf-8",j);
  });
  server.on("/api/graph-history",HTTP_GET,[]{
    uint32_t now=millis();
    String j="{\"ok\":true,\"sample_period_ms\":2000,\"temperature\":[";
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
    if(n<0||n>2){server.send(400,"text/plain","Bad graph");return;}
    server.send(200,"application/json","{\"ok\":true,\"scope\":\"browser_only\"}");
  });
  server.on("/api/rainmaker/enable",HTTP_POST,[]{
    if(!apClient() || server.arg("confirm")!="YES"){
      server.send(403,"application/json","{\"error\":\"local AP confirmation required\"}");return;
    }
    if(busaOtaPendingVerification()){
      server.send(409,"application/json","{\"error\":\"running image is not confirmed\"}");return;
    }
    if(otaActive||busaWifiScanRunning){
      server.send(409,"application/json","{\"error\":\"OTA or Wi-Fi scan busy\"}");return;
    }
    if(busaRmBleSessionStarted||busaRmStartRequested){
      server.send(200,"application/json","{\"bluetooth\":\"already_requested\"}");return;
    }
    if(busaRmBleEverCompleted){
      server.send(409,"application/json","{\"error\":\"already paired; reboot device for a new BLE session\"}");return;
    }
    if(!prefs.putBool("rmPairNext",true)){
      server.send(500,"application/json","{\"error\":\"pairing request was not saved\"}");return;
    }
    prefs.putString("rmStage","BLE_REQUESTED");
    busaRmStartRequested=true;
    busaRmBleSessionRequested=true;
    busaRmBleFeedback="Запуск BLE по запросу из локального приложения";
    server.sendHeader("Cache-Control","no-store");
    server.send(202,"application/json","{\"bluetooth\":\"starting\",\"pop\":\"123456789\",\"duration_sec\":180}");
  });
  server.on("/api/rainmaker/disable",HTTP_POST,[]{
    if(!apClient() || server.arg("confirm")!="YES"){
      server.send(403,"application/json","{\"error\":\"local AP confirmation required\"}");return;
    }
    // The AP can be temporarily absent during BLE pairing, so the timeout
    // also stops BLE when a stop request cannot be submitted over Wi-Fi.
    if(!busaRmBleSessionStarted){
      busaRmStartRequested=false;busaRmBleSessionRequested=false;
      busaRmBleFeedback="Bluetooth выключен";
      server.send(200,"application/json","{\"bluetooth\":\"off\"}");return;
    }
    busaRmBleTimeoutPending=true;
    server.send(202,"application/json","{\"bluetooth\":\"stopping\"}");
  });
  server.on("/api/rainmaker/status",HTTP_GET,[]{
    const String ble=busaRmBleSessionStarted?
      (busaRmBleEnded||busaRmBleTimeoutPending?"stopping":
       (busaRmBleAdvertising?"advertising":"starting")):
      (busaRmBleEverCompleted?"done":"off");
    const String state=busaRmStarted?"running":(busaRmStartRequested?"requested":"stopped");
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"rainmaker\":")+quoteJson(state)+
       ",\"ble\":"+quoteJson(ble)+
       ",\"feedback\":"+quoteJson(busaRmBleFeedback)+
       ",\"last_stage\":"+quoteJson(prefs.getString("rmStage","IDLE"))+
       ",\"telemetry\":"+busaRainMakerTelemetryJSON()+
       ",\"automatic_start\":false,\"service\":"+quoteJson(busaRmService)+
       ",\"pop\":\"123456789\"}");
  });
  server.on("/api/settings-tab",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<0||n>4){server.send(400,"text/plain","bad subtab");return;}
    server.send(200,"application/json","{\"ok\":true,\"scope\":\"browser_only\"}");
  });
  server.on("/api/tft/scroll",HTTP_GET,[]{
    String dir=server.arg("dir");
    if(dir!="up"&&dir!="down"){server.send(400,"text/plain","bad dir");return;}
    server.send(200,"application/json","{\"ok\":true,\"scope\":\"browser_only\"}");
  });
  const char *headers[]={"X-Busa-Token"};
  server.collectHeaders(headers,1);
  server.begin();
  busaConfirmRunningImage(busaNvsReady&&wifiRunning);
  if(busaRmPairDeferred&&!busaOtaPendingVerification())busaRmStartRequested=true;
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
  Serial0.println("BUSA CORE STABLE: local TFT + AP first; RainMaker explicit pairing; credentials hidden");
}
void loop(){
  if(busaRmBleBootMode){
    static BusaBleBootRecovery recovery;
    static bool advertisementRecorded=false;
    if(busaRmBleAdvertising&&!advertisementRecorded){
      advertisementRecorded=true;prefs.putString("rmStage","BLE_ADVERTISING");
    }
    const auto action=recovery.tick(millis(),busaRmBleSessionAt,
      busaRmBleAdvertising.load(),busaRmBleEnded.load(),busaRmBleBootError);
    if(action.stop)busaRmStopProvisioning();
    if(action.restart){
      const bool paired=busaRmBleCredentialsAccepted&&busaRmBleEnded;
      const bool saved=paired&&busaSaveProvisionedStation();
      prefs.putBool("rmCloudOn",saved);prefs.putBool("rmCloudBusy",false);
      if(saved)prefs.putString("rmStage","PAIRED");
      else if(paired)prefs.putString("rmStage","BLE_CONFIG_SAVE_ERROR");
      else if(busaRmBleBootError)prefs.putString("rmStage","BLE_RMAKER_ERROR");
      else if(recovery.stopTimeout)prefs.putString("rmStage","BLE_STOP_TIMEOUT");
      else if(recovery.noAdvertisement)prefs.putString("rmStage","BLE_NOT_ADVERTISING");
      else if(busaRmBleCredentialsRejected)prefs.putString("rmStage","BLE_CREDENTIALS_REJECTED");
      else prefs.putString("rmStage","BLE_TIMEOUT");
      soundOutputsOff();ESP.restart();
    }
    delay(20);return;
  }
  uint32_t now=millis();
  const uint32_t tickStart=now;
  soundPoll(now);
  {const uint32_t started=millis();server.handleClient();busaObserve("HTTP",started,busaHttpWorstMs);}
  // One nonblocking driver scan serves both TFT and web UI. Every scan ends.
  busaWebEventsPoll(now); // Passive local journal; no temperature readings or output writes.
  busaWifiScanPoll(now);
  // Reconnect supervisor uses only the CURRENT saved target; explicit staOff wins.
  // BUSA AP and network selection always remain responsive.
  readConsole();
  busaRainMakerApply(); // Apply queued iPhone settings on the same thread as TFT, NVS and sound
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
  busaRainMakerPoll(now);
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
  // Pairing uses a clean boot. The original Wi-Fi procedures are untouched.
  static uint32_t bleRestartAt=0;
  if(busaRmStartRequested&&!busaOtaPendingVerification()){
    busaRmStartRequested=false;
    bleRestartAt=millis();
  }
  if(bleRestartAt && elapsed(millis(),bleRestartAt,1400)&&!otaActive&&!otaRestart){
    soundOutputsOff();
    ESP.restart();
  }
  static bool busaCloudAttempted=false;
  if(busaRmAutoCloud && !busaCloudAttempted && !busaRmStartRequested && !bleRestartAt && !busaOtaPendingVerification() &&
     elapsed(millis(),busaBootAt,20000) && WiFi.status()==WL_CONNECTED &&
     !otaActive && !busaWifiScanRunning){
    busaCloudAttempted=true;
    prefs.putBool("rmCloudBusy",true);
    busaRainMakerStart();
    busaRmCloudBootAt=millis();
    if(!busaRmStarted){
      prefs.putBool("rmCloudBusy",false);
      prefs.putBool("rmCloudOn",false);
      prefs.putString("rmStage","CLOUD_INIT_ERROR");
    }
  }
  if(busaCloudAttempted && busaRmStarted && prefs.getBool("rmCloudBusy",false) &&
     elapsed(millis(),busaRmCloudBootAt,30000)){
    prefs.putBool("rmCloudBusy",false);
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
