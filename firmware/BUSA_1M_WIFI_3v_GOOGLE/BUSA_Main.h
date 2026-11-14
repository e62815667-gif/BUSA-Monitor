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
const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-CALC4-WIFI1-UI14-20261114";
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
      }else{
        temperature[i]=NAN;tempAt[i]=0;
        // Permit bounded rediscovery when a sensor is unplugged or replaced.
        tempPresent[i]=false;cachedTempValid[i]=false;
      }
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
  char buf[80]; // Full int-width fields also fit; normal dates retain their format.
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
    const unsigned char c=(unsigned char)x[i];
    if(c=='"'||c=='\\'){out+='\\';out+=(char)c;}
    else if(c<0x20){
      char escaped[7];snprintf(escaped,sizeof(escaped),"\\u%04x",(unsigned)c);
      out+=escaped;
    }else out+=(char)c;
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
.rigArtStage{position:relative;isolation:isolate;width:100%;aspect-ratio:1024 / 1536;overflow:hidden}
.approvedRigArt{display:block;width:100%;height:auto;object-fit:contain;background:transparent}
.rigSheen{display:none}
.rigCoolantGlow{position:absolute;left:76%;top:47%;width:18%;height:25%;opacity:0;background:linear-gradient(0deg,transparent,#38bdf87a,transparent);border-radius:12px}
.boilerBoil{position:absolute;left:31.5%;top:81.6%;width:19.5%;height:9.5%;overflow:hidden;border-radius:5% 5% 30% 30%;pointer-events:none}
.boilerBoil i{position:absolute;bottom:-10%;left:var(--x);width:8%;aspect-ratio:1;border:1px solid #b8f5ffcc;border-radius:50%;background:radial-gradient(circle at 30% 25%,#e3ffffb3,transparent 65%);animation:busaBoil var(--d) linear infinite;animation-delay:calc(var(--i)*-.37s)}
.boilerSurface{position:absolute;left:-5%;top:0;width:110%;height:7%;border-radius:50%;background:#65e5ff88;animation:busaSurface 1.8s ease-in-out infinite alternate}
@keyframes busaBoil{0%{transform:translateY(0) scale(.4);opacity:0}15%{opacity:.9}85%{opacity:.9}100%{transform:translateY(-1600%) scale(1.15);opacity:0}}
@keyframes busaSurface{from{transform:translateX(-3%) rotate(-2deg);opacity:.5}to{transform:translateX(3%) rotate(2deg);opacity:.9}}
@media(prefers-reduced-motion:reduce){.boilerBoil i,.boilerSurface{animation:none!important}.boilerBoil i{bottom:40%;opacity:.7}}
.rigUpperGlow{left:6%;top:6%;width:13%;height:12%}
.hasCooling .rigCoolantGlow{opacity:.35}
.rigDrop{position:absolute;left:86.5%;top:81%;width:5px;height:8px;background:#40ceff;border-radius:70% 70% 70% 5%;opacity:0;transform:rotate(-45deg)}
.hasCollection .rigDrop{animation:rigDrop 2s linear infinite}.hasCollection .dropTwo{animation-delay:.65s}.hasCollection .dropThree{animation-delay:1.3s}
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

/* UI9: full-width apparatus, temperature overlays and compact top relay row. */
section[data-section="0"] .approvedDashboard{display:block;position:relative;padding-top:62px;min-height:0}
section[data-section="0"] .approvedRig{display:block;width:100%;min-height:0;align-self:auto}
section[data-section="0"] .approvedTempGrid{position:absolute;inset:62px 0 0;display:block;padding:0;z-index:3;pointer-events:none}
:root section[data-section="0"] .approvedTemp{position:absolute;display:block;min-height:0;padding:2px 3px;background:transparent!important;box-shadow:none;line-height:1.1}
section[data-section="0"] .approvedTempIcon{display:none}
section[data-section="0"] .approvedTempTitle{display:block;font-size:12px;font-weight:600;color:#d9edf9;text-shadow:0 1px 3px #03101f}
section[data-section="0"] .approvedTemp strong{font-size:clamp(20px,6vw,29px);line-height:1.12;margin:2px 0;white-space:nowrap;font-variant-numeric:tabular-nums;text-shadow:0 1px 4px #03101f}
section[data-section="0"] .approvedTemp small{font-size:9px;line-height:1.15;text-shadow:0 1px 3px #03101f}
section[data-section="0"] .temp1{left:0;top:-62px;width:43%}
section[data-section="0"] .temp3{right:0;top:-62px;width:43%;text-align:right}
section[data-section="0"] .temp0{left:0;top:79%;width:28%}
section[data-section="0"] .temp0 strong{font-size:clamp(18px,5.1vw,25px)}
section[data-section="0"] .temp2{left:31.5%;top:84%;width:22.5%;text-align:center}
section[data-section="0"] .temp2 .approvedTempTitle{font-size:10px;color:#fff3b8}
section[data-section="0"] .temp2 strong{font-size:clamp(15px,4.25vw,24px);margin-top:3px;text-shadow:0 1px 4px #062c42,0 0 5px #062c42}
section[data-section="0"] .temp2 small{display:none}
section[data-section="0"] .relayIndicators{grid-template-columns:repeat(4,minmax(0,1fr));gap:5px;margin:0 0 10px}
section[data-section="0"] .relayBadge{display:grid;grid-template-columns:7px minmax(0,1fr);column-gap:3px;row-gap:2px;align-content:center;align-items:center;padding:5px 4px;min-height:39px;border-radius:7px;font-size:clamp(9px,2.4vw,12px);line-height:1.15}
section[data-section="0"] .relayLamp{width:6px;height:6px;margin:0}
section[data-section="0"] .relayBadge small{grid-column:2;font-size:8px;line-height:1.1;white-space:normal}
section[data-section="0"] .approvedMetrics{margin:7px 0;gap:6px}
section[data-section="0"] .approvedMetric{padding:7px;min-height:60px}
section[data-section="0"] .approvedMetric>span{font-size:22px}
section[data-section="0"] .approvedMetric strong{font-size:23px}
section[data-section="0"] .approvedMetric small{font-size:10px}
.boilerBoil{left:31.8%;top:81.7%;width:21.8%;height:10.4%;border-radius:4% 4% 22% 22%}
.boilerBoil i{width:6%;animation-duration:calc(var(--d)*.85)}
.rigCoolantGlow{left:69%;top:47%;width:11%;height:24%}
.rigUpperGlow{left:18%;top:8%;width:9%;height:13%}
.rigDrop{left:74.7%;top:76.2%}
.rigSheen{clip-path:polygon(36% 26%,49% 26%,49% 72%,60% 75%,60% 96%,24% 96%,24% 76%,36% 72%)}
.collectionLiquid{position:absolute;left:67.3%;top:87.1%;width:18.4%;height:8.5%;overflow:hidden;pointer-events:none;border-radius:3% 3% 18% 18%}
.collectionSurface{position:absolute;left:-4%;top:0;width:108%;height:10%;border-radius:50%;background:#96f3ff88;animation:busaSurface 2.2s ease-in-out infinite alternate}
.collectionRipple{position:absolute;left:27%;top:0;width:45%;height:14%;border:1px solid #bdf8ffcc;border-radius:50%;animation:busaRipple 2s ease-out infinite}
.collectionLiquid i{position:absolute;left:var(--x);bottom:-10%;width:5%;aspect-ratio:1;border-radius:50%;border:1px solid #ccffff88;background:#bdf8ff33;animation:busaCollect var(--d) linear infinite;animation-delay:calc(var(--i)*-1.1s)}
@keyframes busaRipple{from{transform:scale(.3);opacity:.8}to{transform:scale(1.8);opacity:0}}
@keyframes busaCollect{0%{transform:translateY(0) scale(.5);opacity:0}20%{opacity:.6}85%{opacity:.6}100%{transform:translateY(-1900%) scale(1);opacity:0}}
@media(prefers-reduced-motion:reduce){.collectionSurface,.collectionRipple,.collectionLiquid i{animation:none!important}.collectionRipple{opacity:.5}}
@media(min-width:600px){section[data-section="0"] .approvedDashboard{max-width:600px;margin:auto}section[data-section="0"] .approvedTempTitle{font-size:15px}section[data-section="0"] .temp2 .approvedTempTitle{font-size:13px}}

.waterMotion{position:absolute;inset:0;width:100%;height:100%;max-height:none!important;pointer-events:none;z-index:1;overflow:visible}
.waterMotion path{fill:none;stroke-linecap:round;stroke-linejoin:round}
.waterTrace{stroke:#b8f6ff;stroke-width:5;stroke-dasharray:15 55;animation:waterRun 2.1s linear infinite;opacity:.9;filter:drop-shadow(0 0 2px #39cfff)}
.transfer{animation-duration:3s}.drain{animation-duration:3.5s}
.coilTrace{stroke:#b9efff;stroke-width:4;stroke-dasharray:14 94;animation:coilRun 2.8s linear infinite;opacity:.85}
.jacketTrace{stroke:#5ce3ff;stroke-width:4;stroke-dasharray:8 35;animation:waterRun 2s linear infinite;opacity:.7}
.productTrace{stroke:#d0fbff;stroke-width:5;stroke-dasharray:6 36;animation:productRun 1.5s linear infinite;opacity:.85}
@keyframes waterRun{to{stroke-dashoffset:-140}}
@keyframes coilRun{to{stroke-dashoffset:-216}}
@keyframes productRun{to{stroke-dashoffset:-126}}
section[data-section="0"] .approvedTemp small{display:flex;flex-direction:column;gap:1px;font-size:10px;line-height:1.2}
section[data-section="0"] .temp0{height:15%;top:79%}
section[data-section="0"] .temp0 strong{font-size:clamp(15px,4.1vw,18px)}
section[data-section="0"] .temp0 small{position:absolute;top:88%;left:3px;font-size:9px;white-space:nowrap}
section[data-section="0"] .temp2{top:82.6%}
section[data-section="0"] .temp2 small{display:flex;font-size:8px;line-height:1.25;margin-top:3px;color:#e2f4ff!important}
section[data-section="0"] .temp2 strong{font-size:clamp(14px,4vw,23px)}
.flowOnPipe,.waterUsedOnPipe,.collectedOnGlass,.collectionRateLabel{position:absolute;z-index:4;text-align:center;color:#d9f3ff;text-shadow:0 1px 4px #03101f,0 0 3px #03101f;pointer-events:none}
.flowOnPipe{right:0;top:54.3%;width:18%}
.waterUsedOnPipe{right:0;top:61.2%;width:25%}
.flowOnPipe span,.waterUsedOnPipe span{display:block;font-size:9px;line-height:1.3}
.flowOnPipe b,.waterUsedOnPipe b{display:block;font-size:14px;line-height:1.3;color:#65d7ff}
.collectedOnGlass{left:67.5%;top:82.2%;width:18.8%}
.collectedOnGlass span{display:block;font-size:9px;line-height:1.25}
.collectedOnGlass b{display:block;font-size:16px;line-height:1.35;color:#e7fbff}
.collectionRateLabel{left:64%;top:97.5%;width:27%;font-size:9px;line-height:1.2;white-space:nowrap}
.rigCoolantGlow{height:12%}.rigUpperGlow{height:13%}
section[data-section="0"] .approvedMetrics{display:none}
section[data-section="0"] .approvedAdditional{display:none}
.appTabs{position:static!important;margin-top:15px!important}
body{padding-bottom:10px!important;min-height:0!important}
@media(prefers-reduced-motion:reduce){.waterMotion path{animation:none!important}}

.rigArtStage{overflow:visible}
section[data-section="0"] .approvedDashboard{padding-top:86px;padding-bottom:60px}
section[data-section="0"] .approvedTempGrid{inset:86px 0 60px}
section[data-section="0"] .temp1,section[data-section="0"] .temp3{top:-86px}
section[data-section="0"] .approvedTemp small{gap:4px;line-height:1.35}
section[data-section="0"] .temp0 small{top:92.5%;gap:4px}
section[data-section="0"] .temp2{top:83.5%}
section[data-section="0"] .temp2 strong{font-size:clamp(15px,4.3vw,24px)}
.cubeSetBelow{position:absolute;left:29%;top:99%;width:28%;z-index:5;display:flex;flex-direction:column;gap:5px;align-items:center;text-align:center;font-size:10px;line-height:1.3;color:#d8eefb;text-shadow:0 1px 3px #03101f;white-space:nowrap}
.flowOnPipe{top:52.5%;width:19%}
.waterUsedOnPipe{top:63%;width:27%}
.flowOnPipe span,.waterUsedOnPipe span{margin-bottom:4px}
.collectionRateLabel{top:99%;width:29%;left:63%;font-size:9px}
section[data-section="0"] .relayIndicators{margin-bottom:15px}

section[data-section="0"] .approvedDashboard{padding-top:96px;padding-bottom:76px}
section[data-section="0"] .approvedRig{width:88%;margin:auto}
section[data-section="0"] .approvedTempGrid{inset:96px 6% 76px}
section[data-section="0"] .temp1{top:-96px;left:-6.8%;width:45%}
section[data-section="0"] .temp3{top:-96px;right:-6.8%;width:45%}
section[data-section="0"] .temp0{left:-6%;width:33%}
.cubeSetBelow{left:10%;width:65%;gap:14px;flex-direction:row;justify-content:center;font-size:10px}
.waterUsedOnPipe{top:64%;right:-2%;width:29%}
.flowOnPipe{right:-2%;width:22%;top:51.5%}
.collectionRateLabel{top:100.5%;left:61%;width:33%}

/* Selected option 2 only: larger apparatus and clearer label spacing. */
section[data-section="0"] .approvedRig{width:104%;margin-left:-2%;margin-right:-2%}
section[data-section="0"] .approvedTempGrid{inset:96px -2% 76px}
section[data-section="0"] .temp1{left:2.5%;width:43%}
section[data-section="0"] .temp3{right:2.5%;width:43%}
section[data-section="0"] .temp0{left:0;width:29%}
section[data-section="0"] .temp0 strong{font-size:clamp(14px,3.9vw,17px)}
.flowOnPipe{right:0;width:20%}
.waterUsedOnPipe{right:0;top:64%;width:20%;text-align:right}
.waterUsedOnPipe span{font-size:9px;white-space:nowrap}
.waterUsedOnPipe b{font-size:14px;white-space:nowrap}
.collectedOnGlass{left:67.5%;top:86.6%;width:18.8%}
.collectedOnGlass span{font-size:9px}
.collectedOnGlass b{font-size:16px;line-height:1.4}

/* Keep the lower values on one baseline, with consistent units and spacing. */
.cubeSetBelow{top:100%;font-size:10px;line-height:14px;height:14px;align-items:baseline}
.cubeSetBelow span{line-height:14px}
.collectionRateLabel{top:100%;font-size:10px;line-height:14px;height:14px}
.flowOnPipe,.waterUsedOnPipe{text-align:right;right:0;width:20%}
.flowOnPipe span,.waterUsedOnPipe span{font-size:9px;line-height:14px;margin-bottom:4px}
.flowOnPipe b,.waterUsedOnPipe b{font-size:14px;line-height:20px;font-variant-numeric:tabular-nums}

/* Use lower whitespace for a larger scene; keep controls just below it. */
main{padding-bottom:0!important}
section[data-section="0"]{padding-bottom:4px}
.appTabs{margin-top:4px!important}
section[data-section="0"] .approvedDashboard{padding-top:116px;padding-bottom:28px}
section[data-section="0"] .approvedRig{width:112%;margin-left:-6%;margin-right:-6%}
section[data-section="0"] .approvedTempGrid{inset:116px -6% 28px}
section[data-section="0"] .temp1{top:-116px;left:6.5%}
section[data-section="0"] .temp3{top:-116px;right:6.5%}
section[data-section="0"] .temp0{left:3.5%;padding:2px 0}
section[data-section="0"] .temp0 strong{font-size:clamp(12.5px,3.33vw,15px)}
section[data-section="0"] .temp2 strong{font-size:clamp(16px,4.7vw,25px)}
.flowOnPipe,.waterUsedOnPipe{right:4.5%;width:20%}
.flowOnPipe b,.waterUsedOnPipe b{font-size:13px;line-height:20px}
.collectedOnGlass{top:88%}
.collectedOnGlass span{color:#f4c84a!important}

/* Release: persistent navigation; real flow animation, permanent decorative liquids. */
.appTabs{position:fixed!important;bottom:0!important;margin-top:0!important}
body{padding-bottom:68px!important}
.collectedOnGlass #homeCollectionTotal,.collectedOnGlass #homeCollectionUnit{display:inline;font:inherit;line-height:inherit;color:inherit!important;white-space:nowrap}
.collectionRateLabel #homeCollectionRate{display:inline;font:inherit;line-height:inherit;color:inherit;white-space:nowrap}
.waterMotion path{opacity:0;animation-play-state:paused}
.hasCooling .waterMotion .waterTrace,.hasCooling .waterMotion .coilTrace,.hasCooling .waterMotion .jacketTrace{opacity:.85;animation-play-state:running}
.hasCollection .waterMotion .productTrace{opacity:.85;animation-play-state:running}
@media(prefers-reduced-motion:reduce){.waterMotion path{animation:none!important}}
/* BUSA_VIEWPORT_CSS_BEGIN */
html.viewportFit{height:100%;overflow:hidden;overscroll-behavior:none}
.viewportFit body{position:fixed;inset:0;width:100%;height:var(--app-height,100dvh);min-height:0!important;padding:0!important;overflow:hidden;overscroll-behavior:none;display:flex;flex-direction:column}
.viewportFit [hidden]{display:none!important}
.viewportFit .appHeader{order:0;position:relative!important;flex:0 0 auto;min-height:0}
.viewportFit .appHeader .netPill{white-space:nowrap;flex-shrink:0}
.viewportFit main{order:1;flex:1 1 0;width:100%;max-width:620px;min-height:0;padding:0 11px!important;overflow:hidden;display:flex;flex-direction:column;position:relative}
.viewportFit main>.page{flex:1 1 0;min-height:0;height:auto;max-height:100%;width:100%;margin:0!important;padding:0!important;overflow:hidden;position:relative}
.viewportFit .appTabs{order:2;position:relative!important;left:auto;right:auto;bottom:auto;flex:0 0 auto;margin:0 auto!important;width:100%}
.viewportFit section[data-section="0"]{display:flex;flex-direction:column}
.viewportFit section[data-section="0"] .relayIndicators{flex:0 0 auto;grid-template-columns:repeat(4,minmax(0,1fr));gap:4px;margin:0 0 4px!important}
.viewportFit section[data-section="0"] .approvedDashboard{flex:1 1 0;min-height:0;height:auto;max-width:none;width:100%;padding:0!important;margin:0!important;overflow:visible;position:relative}
.viewportFit section[data-section="0"] .approvedRig{position:absolute;left:var(--scene-left);top:var(--scene-top);width:var(--scene-w);height:var(--scene-h);min-height:0!important;margin:0!important}
.viewportFit .rigArtStage{width:100%;height:100%;aspect-ratio:auto;overflow:visible}
.viewportFit .approvedRigArt{width:100%;height:100%;object-fit:contain}
.viewportFit section[data-section="0"] .approvedTempGrid{inset:auto!important;left:var(--scene-left)!important;top:var(--scene-top)!important;width:var(--scene-w);height:var(--scene-h);padding:0!important}
.viewportFit section[data-section="0"] .temp1{left:calc(3px - var(--scene-left));top:calc(0px - var(--scene-top));width:calc(var(--panel-w)*.47)}
.viewportFit section[data-section="0"] .temp3{right:calc(3px - var(--scene-left));top:calc(0px - var(--scene-top));width:calc(var(--panel-w)*.47)}
.viewportFit section[data-section="0"] .approvedTempTitle{font-size:11px!important;line-height:1.2}
.viewportFit section[data-section="0"] .temp1 strong,.viewportFit section[data-section="0"] .temp3 strong{font-size:20px;line-height:1.1;margin:2px 0}
.viewportFit section[data-section="0"] .temp1 small,.viewportFit section[data-section="0"] .temp3 small{font-size:9px;line-height:1.15;gap:1px}
.viewportFit section[data-section="0"] .temp0{left:calc(2px - var(--scene-left));top:79%;width:35%;height:15%;padding:0}
.viewportFit section[data-section="0"] .temp0 strong{font-size:clamp(11px,calc((var(--scene-left) + var(--scene-w)*.13)/3.5),16px);line-height:1.1}
.viewportFit section[data-section="0"] .temp0 small{font-size:8px;line-height:1.15;gap:1px;top:92.5%;left:0}
.viewportFit section[data-section="0"] .temp2{left:31.5%;top:83.2%;width:22.5%;padding:1px 0}
.viewportFit section[data-section="0"] .temp2 .approvedTempTitle{font-size:8px!important;line-height:1.05}
.viewportFit section[data-section="0"] .temp2 strong{font-size:clamp(10px,calc(var(--scene-w)*.052),18px);line-height:1.1;margin:1px 0}
.viewportFit .cubeSetBelow,.viewportFit .collectionRateLabel{top:100%;height:13px;line-height:13px;font-size:clamp(8px,calc(var(--scene-w)*.033),10px)}
.viewportFit .cubeSetBelow{left:10%;width:65%;gap:9px}
.viewportFit .cubeSetBelow span{line-height:13px}
.viewportFit .collectionRateLabel{left:63%;width:35%;white-space:nowrap}
.viewportFit .flowOnPipe,.viewportFit .waterUsedOnPipe{right:calc(2px - var(--scene-left));width:max(65px,calc(var(--scene-w)*.2));text-align:right}
.viewportFit .flowOnPipe{top:52%}
.viewportFit .waterUsedOnPipe{top:64%}
.viewportFit .flowOnPipe span,.viewportFit .waterUsedOnPipe span{font-size:8px;line-height:1.15;margin-bottom:2px}
.viewportFit .flowOnPipe b,.viewportFit .waterUsedOnPipe b{font-size:clamp(10px,calc(var(--scene-w)*.045),14px);line-height:1.2}
.viewportFit .collectedOnGlass{top:88%;left:67.5%;width:18.8%}
.viewportFit .collectedOnGlass span{font-size:clamp(7px,calc(var(--scene-w)*.028),9px);line-height:1.05}
.viewportFit .collectedOnGlass b{font-size:clamp(10px,calc(var(--scene-w)*.05),16px);line-height:1.2}
.viewportFit .smallScene .cubeSetBelow{flex-direction:column;gap:0;font-size:8px;line-height:10px;height:20px}
.viewportFit .smallScene .cubeSetBelow span{line-height:10px}
.viewportFit .smallScene .collectionRateLabel{font-size:8px;line-height:10px}
.viewportFit .chartCard{flex:0 0 auto}
.viewportFit #chartCanvas{flex:none}
.viewportFit section[data-section="5"]{display:flex;flex-direction:column}
.viewportFit section[data-section="5"]>.approvedPageTitle,.viewportFit section[data-section="5"]>#settingsBack{flex:0 0 auto}
.viewportFit #settingsMenu{flex:0 0 auto;display:grid;grid-template-columns:repeat(var(--menu-columns,1),minmax(0,1fr));grid-auto-rows:var(--menu-row-height,53px);margin:0!important;padding:0!important}
.viewportFit #settingsMenu>.approvedMenuItem{height:100%;min-height:0!important;box-sizing:border-box;padding-top:clamp(0px,calc((var(--menu-row-height,53px) - 32px)/2),9px);padding-bottom:clamp(0px,calc((var(--menu-row-height,53px) - 32px)/2),9px)}
.viewportFit #settingsMenu .approvedMenuIcon{flex-shrink:0}
.viewportFit #settingsMenu>.approvedMenuItem>span:nth-child(2){overflow-wrap:anywhere}
.viewportFit .vpScroll{min-height:0;max-height:var(--content-room);overflow-y:auto!important;overflow-x:hidden;overscroll-behavior:contain;scrollbar-width:thin;flex:0 1 auto}
.viewportFit .vpScroll textarea[readonly]{pointer-events:auto;max-height:180px;overflow:auto;resize:none}
.viewportFit .page pre{white-space:pre-wrap!important;overflow-wrap:anywhere;word-break:break-word;max-width:100%}
.viewportFit section[data-section="1"]{display:flex;flex-direction:column}
.viewportFit section[data-section="1"]>.approvedPageTitle,.viewportFit section[data-section="1"]>.temperatureSaveRow{flex:0 0 auto}
.viewportFit section[data-section="1"] .approvedSensorList{display:flex;flex-direction:column;flex:1 1 0;min-height:0;overflow-y:auto;overscroll-behavior:contain;margin:4px 0 6px;gap:7px}
.viewportFit section[data-section="1"] .approvedSensor{flex:0 0 auto;min-height:0}
.viewportFit section[data-section="1"] .approvedSensorHead{min-height:var(--sensor-row-height,74px);height:var(--sensor-row-height,74px);box-sizing:border-box;padding-top:6px;padding-bottom:6px}
.viewportFit section[data-section="1"] .sensorEdit>summary{height:var(--sensor-row-height,74px)}
.viewportFit section[data-section="1"] .temperatureSaveRow{padding:4px 0;margin:0}
.viewportFit section[data-section="1"].wideContent .approvedSensorList{display:grid;grid-template-columns:1fr 1fr;align-content:start}
.viewportFit section[data-section="2"].compactContent .liquidPanel{padding:10px;margin:6px 0}
.viewportFit section[data-section="2"].compactContent .liquidPanelHeading{margin-bottom:8px}
.viewportFit section[data-section="2"].compactContent .liquidSummary{margin-top:8px;padding-top:7px;gap:6px}
.viewportFit section[data-section="2"].compactContent .liquidSummary strong{font-size:20px}
.viewportFit section[data-section="2"].wideContent{display:grid;grid-template-columns:1fr 1fr;gap:8px;align-content:start}
.viewportFit section[data-section="2"].wideContent>.approvedPageTitle{grid-column:1/-1}
.viewportFit section[data-section="2"].wideContent .liquidPanel{margin:0}
.viewportFit .temperatureCalibration,.viewportFit .wifiSettings{padding:10px!important;margin:0!important}
.viewportFit .temperatureCalibration h2,.viewportFit .wifiSettings h2{font-size:17px;line-height:1.25;margin:0 0 9px!important}
.viewportFit .temperatureCalibration>.hint{margin:0 0 7px;font-size:11px;line-height:1.3}
.viewportFit .calibrationFields{display:grid;gap:3px}
.viewportFit .calibrationFields .row{display:grid;grid-template-columns:76px minmax(0,1fr);align-items:center;gap:10px;padding:1px 0;margin:0}
.viewportFit .calibrationFields label{font-size:13px;line-height:1.2;margin:0;white-space:nowrap}
.viewportFit .calibrationFields input{width:100%;min-width:0;box-sizing:border-box;margin:0;padding:5px 10px;min-height:34px;font-size:16px;line-height:1.25}
.viewportFit .calibrationFields label[for="calOffset0"]{color:var(--water)}
.viewportFit .calibrationFields label[for="calOffset2"]{color:var(--cube)}
.viewportFit .calibrationFields label[for="calOffset1"]{color:var(--column)}
.viewportFit .calibrationFields label[for="calOffset3"]{color:var(--alarm)}
.viewportFit .temperatureCalibration #tempCalSave{margin:9px 0 0;min-height:38px;padding:8px;font-size:13px}
.viewportFit #tempCalResult:empty,.viewportFit #response:empty{display:none}
.viewportFit .temperatureCalibration #tempCalResult{margin:7px 0 0;font-size:11px;line-height:1.3}
.viewportFit .wifiSettings .busaWifiMode{border:0;padding:0;display:grid;gap:9px}
.viewportFit .wifiField{display:grid;gap:5px;min-width:0}
.viewportFit .wifiField label{font-size:13px;line-height:1.2;margin:0}
.viewportFit .wifiField input,.viewportFit .wifiField select{width:100%;box-sizing:border-box;min-width:0;margin:0;min-height:38px;padding:8px;font-size:16px;line-height:1.25}
.viewportFit .wifiSettings button{min-height:36px;padding:8px;font-size:13px;margin:0}
.viewportFit .wifiActions{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.viewportFit .wifiSettings #response{font-size:12px;line-height:1.3;margin:0;overflow-wrap:anywhere}
.viewportFit .shortFormViewport .calibrationFields{grid-template-columns:1fr 1fr;gap:4px 12px}
.viewportFit .shortFormViewport .calibrationFields .row{grid-template-columns:64px minmax(0,1fr);gap:5px;padding:3px 0}
.viewportFit .shortFormViewport .calibrationFields input{min-height:30px;padding:4px}
.viewportFit .shortFormViewport .temperatureCalibration>.hint{display:none}
.viewportFit .shortFormViewport .temperatureCalibration,.viewportFit .shortFormViewport .wifiSettings{padding:7px!important}
.viewportFit .shortFormViewport .temperatureCalibration h2,.viewportFit .shortFormViewport .wifiSettings h2{margin-bottom:5px!important}
.viewportFit .shortFormViewport .temperatureCalibration #tempCalSave{min-height:30px;margin-top:5px;padding:5px}
.viewportFit .shortFormViewport .wifiSettings .busaWifiMode{grid-template-columns:1fr 1fr;gap:5px 9px}
.viewportFit .shortFormViewport .wifiSettings #wifiRetry,.viewportFit .shortFormViewport .wifiSettings .wifiActions,.viewportFit .shortFormViewport .wifiSettings #response{grid-column:1/-1}
.viewportFit .shortFormViewport .wifiField input,.viewportFit .shortFormViewport .wifiField select{min-height:30px;padding:4px}
.viewportFit .shortFormViewport .wifiSettings button{min-height:28px;padding:5px}
@media(max-height:500px){.viewportFit .approvedPageTitle{display:none}}
/* BUSA_VIEWPORT_CSS_END */
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

<div class="relayIndicators" role="status" aria-label="Блок реле"><div class="relayBadge" id="relayBadge0" data-state="unknown"><span class="relayLamp"></span><span>ТЭН откл.</span><small id="soundStatus0">Нет связи</small></div><div class="relayBadge" id="relayBadge1" data-state="unknown"><span class="relayLamp"></span><span>Кран закрыт</span><small id="soundStatus1">Нет связи</small></div><div class="relayBadge" id="relayBadge2" data-state="unknown"><span class="relayLamp"></span><span>Кран открыт</span><small id="soundStatus2">Нет связи</small></div><div class="relayBadge" id="relayBadge3" data-state="unknown"><span class="relayLamp"></span><span>Авария</span><small id="soundStatus3">Нет связи</small></div></div>
<div class="approvedDashboard">
  <div class="approvedTempGrid"><div class="approvedTemp temp0"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon0" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><path d="M24 4C17 17 7 28 7 37a17 17 0 0034 0C41 28 31 17 24 4Z" fill="url(#busaIcon0)" stroke="#28cdff" stroke-width="2"/><path d="M15 35q0 12 12 13" fill="none" stroke="#dcfaff" stroke-width="2.5" stroke-linecap="round"/></svg></span><span>Вода</span></div><strong id="t0">Н/Д</strong><small id="homeSet0">ВКЛ — · ВЫКЛ —</small></div><div class="approvedTemp temp2"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon2" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="5" y="17" width="38" height="35" rx="7" stroke="#10b9f7" stroke-width="2.5" fill="#083357"/><path d="M8 31q8-5 16 0t16 0v18H8Z" fill="url(#busaIcon2)" opacity=".92"/><path d="M14 17V9h20v8" fill="none" stroke="#55d6ff" stroke-width="3"/></svg></span><span>Куб</span></div><strong id="t2">Н/Д</strong><small id="homeSet2">ВКЛ — · ВЫКЛ —</small></div><div class="approvedTemp temp1"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon1" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIcon1)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></span><span>Колонна</span></div><strong id="t1">Н/Д</strong><small id="homeSet1">ВКЛ — · ВЫКЛ —</small></div><div class="approvedTemp temp3"><div class="approvedTempTitle"><span class="approvedTempIcon"><svg class="busaRefTempIcon" viewBox="0 0 48 58" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"><defs><linearGradient id="busaIcon3" x1="0" x2="1" y1="0" y2="1"><stop stop-color="#d8faff"/><stop offset=".4" stop-color="#06caff"/><stop offset="1" stop-color="#0276e9"/></linearGradient></defs><rect x="9" y="6" width="30" height="6" rx="3" stroke="#c0eeff" fill="#8eeaff"/><rect x="15" y="13" width="18" height="35" rx="2" fill="#082b51" stroke="#6acbff" stroke-width="2"/><path d="M17 29h14v17H17Z" fill="url(#busaIcon3)" opacity=".84"/><path d="M17 23h14M17 38h14" stroke="#9deeff" stroke-width="2"/><rect x="9" y="47" width="30" height="6" rx="3" stroke="#b7e7ff" fill="#6cc2f0"/></svg></span><span>Авария</span></div><strong id="t3">Н/Д</strong><small id="homeSet3">ВКЛ — · ВЫКЛ —</small></div></div>
  <div class="approvedRig"><div class="rigArtStage" id="rigArtStage"><img class="approvedRigArt" src="data:image/webp;base64,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" alt="Аппарат с двумя холодильниками по согласованному рисунку"><div class="boilerBoil" aria-hidden="true"><span class="boilerSurface"></span><i style="--i:0;--x:12%;--d:2.4s"></i><i style="--i:1;--x:32%;--d:1.9s"></i><i style="--i:2;--x:54%;--d:2.7s"></i><i style="--i:3;--x:77%;--d:2.1s"></i><i style="--i:4;--x:22%;--d:3.1s"></i><i style="--i:5;--x:65%;--d:2.4s"></i><i style="--i:6;--x:43%;--d:2.2s"></i><i style="--i:7;--x:85%;--d:2.9s"></i></div><div class="collectionLiquid" aria-hidden="true"><span class="collectionSurface"></span><span class="collectionRipple"></span><i style="--i:0;--x:18%;--d:3.2s"></i><i style="--i:1;--x:48%;--d:4.1s"></i><i style="--i:2;--x:76%;--d:3.6s"></i></div><svg class="waterMotion" viewBox="0 0 1024 1536" aria-hidden="true"><path class="waterTrace inlet" d="M1016 920 H892 Q856 920 856 887 H840"/><path class="waterTrace transfer" d="M849 714 C830 374 620 303 310 308 H294"/><path class="waterTrace drain" d="M154 125 V1345 Q154 1385 111 1385 H14"/><path class="coilTrace" d="M729 897 C795 895 795 887.5 729 885.5 C723.72 882.5 723.72 876 729 874 C795 872 795 864.5 729 862.5 C723.72 859.5 723.72 853 729 851 C795 849 795 841.5 729 839.5 C723.72 836.5 723.72 830 729 828 C795 826 795 818.5 729 816.5 C723.72 813.5 723.72 807 729 805 C795 803 795 795.5 729 793.5 C723.72 790.5 723.72 784 729 782 C795 780 795 772.5 729 770.5 C723.72 767.5 723.72 761 729 759 C795 757 795 749.5 729 747.5 C723.72 744.5 723.72 738 729 736"/><path class="coilTrace" d="M201 310 C266 308 266 300.0 201 298.0 C195.8 295.0 195.8 288 201 286 C266 284 266 276.0 201 274.0 C195.8 271.0 195.8 264 201 262 C266 260 266 252.0 201 250.0 C195.8 247.0 195.8 240 201 238 C266 236 266 228.0 201 226.0 C195.8 223.0 195.8 216 201 214 C266 212 266 204.0 201 202.0 C195.8 199.0 195.8 192 201 190 C266 188 266 180.0 201 178.0 C195.8 175.0 195.8 168 201 166 C266 164 266 156.0 201 154.0 C195.8 151.0 195.8 144 201 142 C266 140 266 132.0 201 130.0 C195.8 127.0 195.8 120 201 118"/><path class="jacketTrace" d="M709 895 V735 M821 735 V895 M185 310 V120 M279 120 V310"/><path class="productTrace" d="M767 981 V1241"/></svg><div class="flowOnPipe"><span>Расход</span><b id="homeCoolingRate">— л/ч</b></div><div class="waterUsedOnPipe"><span>Израсходовано</span><b id="homeWaterUsed">— л</b></div><div class="collectedOnGlass"><span>Собрано</span><b></b></div><div class="collectionRateLabel"></div><span class="rigSheen" aria-hidden="true"></span><span class="rigCoolantGlow" aria-hidden="true"></span><span class="rigCoolantGlow rigUpperGlow" aria-hidden="true"></span><span class="rigDrop dropOne" aria-hidden="true"></span><span class="rigDrop dropTwo" aria-hidden="true"></span><span class="rigDrop dropThree" aria-hidden="true"></span></div></div>
</div>
<div class="approvedMetrics">
  <div class="approvedMetric"><span>◉</span><div><small>Отбор</small><strong id="homeCollectionRate">—</strong><small>мл/мин</small></div></div>
  <div class="approvedMetric"><span>▤</span><div><small>Собрано</small><strong id="homeCollectionTotal">—</strong><small id="homeCollectionUnit">мл</small></div></div>
</div>
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

<div class="card settingCard temperatureCalibration" data-stab="0"><h2>Калибровка температуры</h2>
<p class="hint">Поправка: −10…+10 °C · шаг 0,1 °C</p>
<div class="calibrationFields">
<div class="row"><label for="calOffset0">Вода</label><input id="calOffset0" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset2">Куб</label><input id="calOffset2" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset1">Колонна</label><input id="calOffset1" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
<div class="row"><label for="calOffset3">Авария</label><input id="calOffset3" type="number" min="-10" max="10" step="0.1" value="0.0"></div>
</div>
<button id="tempCalSave" type="button" class="full">Сохранить поправки</button>
<p id="tempCalResult" role="status" class="hint"></p></div>

<div class="card settingCard wifiSettings" data-stab="1" hidden><h2>Wi-Fi</h2>
<div class="busaWifiMode">
<div class="wifiField"><label for="net">Выберите сеть</label><select id="net"><option value="">Поиск сетей...</option></select></div>
<div class="wifiField"><label for="pass">Пароль</label><input id="pass" type="password" minlength="8" maxlength="63" autocomplete="current-password" placeholder="Введите пароль"></div>
<button type="button" id="wifiRetry" class="full">Обновить список</button>
<div class="wifiActions"><button type="button" class="full" id="save">Подключить</button>
<button type="button" class="full" id="disconnectWifi">Отключить</button></div>
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
const decimal1=new Intl.NumberFormat('ru-RU',{minimumFractionDigits:1,maximumFractionDigits:1});
let previousWater=null;
function measured(v){return typeof v==='number'&&Number.isFinite(v)&&v>=0;}
function formatVolumeMl(v){if(!measured(v))return '—';return v<1000?decimal1.format(v)+' мл':decimal1.format(v/1000)+' л';}
function formatNumeric(v){return typeof v==='number'&&Number.isFinite(v)?decimal1.format(v):'—';}
function formatRunTime(ms){if(!measured(ms))return '—';const minutes=Math.floor(ms/60000);return String(Math.floor(minutes/60)).padStart(2,'0')+':'+String(minutes%60).padStart(2,'0');}
function setMeasured(id,v){get(id).textContent=measured(v)?decimal1.format(v):'—';}
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
 get('homeCoolingRate').textContent=(measured(lph)?decimal1.format(lph):'—')+' л/ч';
 get('homeWaterUsed').textContent=(ready?decimal1.format(w.total_l_estimate):'—')+' л';
 get('coolingAvailability').textContent=!s?'Нет связи':ready?(w.calibrated?'Данные датчика':'Расчётно · без калибровки'):'Нет импульсов';
 get('rigArtStage').classList.toggle('hasCollection',measured(mlMin)&&mlMin>0);
 get('rigArtStage').classList.toggle('hasCooling',measured(lph)&&lph>0);
}

function renderHomeThresholds(i,on,off){
 const node=get('homeSet'+i);
 if(node.children.length!==2)node.replaceChildren(document.createElement('span'),document.createElement('span'));
 [['ВКЛ',on],['ОТКЛ',off]].forEach(([label,value],index)=>{
  const text=label+' '+formatNumeric(value)+'°',line=node.children[index];
  if(line.textContent!==text)line.textContent=text;
 });
}
function initHomeDrawing(){
 const stage=get('rigArtStage'),threshold=get('homeSet2');threshold.classList.add('cubeSetBelow');stage.append(threshold);
 const total=document.querySelector('.collectedOnGlass b');total.append(get('homeCollectionTotal'),document.createTextNode(' '),get('homeCollectionUnit'));
 document.querySelector('.collectionRateLabel').append(document.createTextNode('Отбор '),get('homeCollectionRate'),document.createTextNode(' мл/мин'));
 for(let i=0;i<4;i++)renderHomeThresholds(i,null,null);
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
 get('t'+i).textContent=typeof t.c==='number'&&Number.isFinite(t.c)?decimal1.format(t.c)+(i===0?'°C':' °C'):'Н/Д';
 get('tempLive'+i).textContent=get('t'+i).textContent;
 const a=tlog4[i];a.push(t.c===null?null:Number(t.c));if(a.length>80)a.shift();
 if(i===0){tlog.push(t.c===null?null:Number(t.c));if(tlog.length>80)tlog.shift();}
});
if(s.sound_alerts?.channels?.length===4 && s.sound_alerts?.buzzer?.sensors?.length===2){
 const c=s.sound_alerts.channels,b=s.sound_alerts.buzzer.sensors;
 const pairs=[[b[0].on_c,b[0].off_c],[b[1].on_c,b[1].off_c],[c[0].on_c,c[0].off_c],[c[3].on_c,c[3].off_c]];
 pairs.forEach((p,i)=>{
  if(!temperatureThresholdsInitialized){get('tempOn'+i).value=p[0]??'';get('tempOff'+i).value=p[1]??'';}
  renderHomeThresholds(i,p[0],p[1]);
   get('detailSet'+i).textContent='ВКЛ '+formatNumeric(p[0])+'° · ОТКЛ '+formatNumeric(p[1])+'°';
 });
 temperatureThresholdsInitialized=true;
}
updateRelayBadges(s.sound_alerts?.channels);
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?'НЕТ ДАННЫХ':formatNumeric(f);get('pulses').textContent=s.water.pulses;updateLiquidPanels(s);
flog.push(f);if(flog.length>80)flog.shift();
const currentPulse=Number(s.water.pulses);
if(Number.isFinite(currentPulse)){
 let delta=lastWaterPulse===null?null:Math.max(0,currentPulse-lastWaterPulse);
 lastWaterPulse=currentPulse;waterPulseLog.push(delta);
 if(waterPulseLog.length>80)waterPulseLog.shift();
}
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
     ' · ВКЛ '+formatNumeric(c.on_c)+' °C · ВЫКЛ '+formatNumeric(c.off_c)+
     ' °C').join('\n')+
     '\nЗуммер T1/T2: '+(s.sound_alerts?.buzzer?.sound_on?'ЗВУК':'ОЖИДАНИЕ');
   get('diagFlow').textContent='GPIO15 · '+d.water_flow?.status+
     '\nИмпульсы: '+d.water_flow?.pulses+
     '\nКалибровка: отсутствует'+
     '\nРасход (оценка): '+formatNumeric(s.water?.lpm)+' л/мин';
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
}catch(e){statusFailures++;if(statusFailures>=3){if(connected)message('Нет ответа ESP32: проверь локальный Wi-Fi');connected=false;networkLeds(null,'');updateRelayBadges(null);updateLiquidPanels(null);for(let i=0;i<4;i++){get('t'+i).textContent='Н/Д';get('tempLive'+i).textContent='Н/Д';}}}finally{pollBusy=false}}
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
function wifiScanMessage(j){
 if(!j.error||j.error==='no_networks'||j.error==='only_hidden_networks')return 'Сети не найдены. Обновите список.';
 return j.error==='scan_cancelled_ota'?'Поиск недоступен во время обновления.':'Не удалось найти сети. Повторите поиск.';
}
async function wifiRefreshNetworks(fresh=false){
 if(wifiListBusy||settingsActive!==1)return;
 wifiListBusy=true;get('wifiRetry').disabled=true;
 const sel=get('net'),selected=sel.value;
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
   if(unique.has(selected))sel.value=selected;
  get('response').textContent=unique.size?'':wifiScanMessage(j);
 }catch(e){
  if(settingsActive===1){sel.replaceChildren(new Option('Выберите сеть',''));get('response').textContent='Не удалось найти сети. Повторите поиск.';}
 }finally{
  wifiListBusy=false;get('wifiRetry').disabled=false;
  if(wifiRescanAfterDisconnect&&settingsActive===1){wifiRescanAfterDisconnect=false;await new Promise(resolve=>setTimeout(resolve,650));if(settingsActive===1)wifiRefreshNetworks(true);}
 }
}
get('net').onchange=()=>{get('response').textContent='';};
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
  let ssid=get('net').value,pass=get('pass').value;
 const encoder=new TextEncoder(),ssidBytes=encoder.encode(ssid).length,passBytes=encoder.encode(pass).length;
  if(!ssid||ssidBytes>32){get('response').textContent='Выберите сеть из списка.';return;}
  if(passBytes<8||passBytes>63){get('response').textContent='Пароль не подходит по длине.';return;}
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
initHomeDrawing();loadWebJournal();settingsTab(-1,false);
message('Веб-приложение запущено');
showPhoneClock();setInterval(showPhoneClock,1000);
 // No endless scan timer. Reopening Wi-Fi requests a new finite search.
syncClock();setInterval(syncClock,300000);
update();setInterval(update,3000);
// Graph cadence is fixed and independent of status polling and selected time range.
setInterval(()=>{if(active===3&&!document.hidden)refreshGraphHistory();},CHART_REFRESH_MS);
document.addEventListener('visibilitychange',()=>{if(!document.hidden&&active===3)refreshGraphHistory(true);});
</script><script src="/calculator.js?v=CALC1"></script><script>
/* BUSA_VIEWPORT_JS_BEGIN */
/* UI14: one-screen navigation; long content scrolls within its own panel. */
(()=>{
'use strict';
const $=s=>document.querySelector(s),main=$('main'),nav=$('.appTabs'),header=$('.appHeader');
document.body.append(nav);document.documentElement.classList.add('viewportFit');
// Copy the complete report, including text outside the visible report window.
$('#diagCopy')?.addEventListener('click',async e=>{
 if(!document.documentElement.classList.contains('viewportFit'))return;
 e.preventDefault();e.stopImmediatePropagation();
 const text=typeof lastDiagnosticReport==='string'?lastDiagnosticReport:'';
 if(!text){$('#diagCopyResult').textContent='Отчёт ещё не получен';return;}
 const fallback=()=>{const t=document.createElement('textarea');t.value=text;t.readOnly=true;t.style.cssText='position:fixed;left:0;top:0;width:1px;height:1px;opacity:0';document.body.append(t);t.focus();t.select();t.setSelectionRange(0,t.value.length);let ok=false;try{ok=document.execCommand('copy');}finally{t.remove();}return ok;};
 let ok=false;
 try{if(navigator.clipboard&&window.isSecureContext){await navigator.clipboard.writeText(text);ok=true;}else ok=fallback();}catch(_){try{ok=fallback();}catch(_){}}
 $('#diagCopyResult').textContent=ok?'Скопировано — вставь в чат':'Не удалось скопировать отчёт';
},true);

let pending=false,busy=false,initialHeight=innerHeight,lastChartSize='';
const visible=e=>!!e&&!e.hidden&&getComputedStyle(e).display!=='none';
const set=(e,n,v)=>{if(e.style.getPropertyValue(n)!==v)e.style.setProperty(n,v);};
function request(){if(!pending){pending=true;requestAnimationFrame(()=>{pending=false;fit();});}}
function size(){
 const v=window.visualViewport,h=v&&Math.abs(v.scale-1)<.01?v.height:innerHeight;
 set(document.documentElement,'--app-height',Math.round(h)+'px');
 document.body.classList.toggle('keyboardViewport',h<initialHeight*.65&&/INPUT|TEXTAREA|SELECT/.test(document.activeElement?.tagName||''));
 document.body.classList.toggle('shortFormViewport',main.clientHeight<280);
}
function fitHome(section){
 const panel=section.querySelector('.approvedDashboard'),w=panel.clientWidth,H=panel.clientHeight;
 let top=Math.min(61,Math.max(45,H*.14));const bottom=25;
 let h=Math.max(70,H-top-bottom),sw=h*1024/1536;
 if(sw>w*1.02){sw=w*1.02;h=sw*1536/1024;}
 top+=Math.max(0,H-top-bottom-h);
 const left=(w-sw)/2;
 for(const [n,v] of Object.entries({'--scene-w':sw+'px','--scene-h':h+'px','--scene-left':left+'px','--scene-top':top+'px','--panel-w':w+'px'}))set(panel,n,v);
 section.classList.toggle('smallScene',sw<220);
}
function fitGraph(){
 const c=$('#chartCanvas'),card=c.closest('.chartCard');
 card.style.zoom='';card.style.width='';
 c.style.removeProperty('height');c.style.removeProperty('aspect-ratio');
 const natural=c.getBoundingClientRect().height;
 const excess=card.getBoundingClientRect().bottom-main.getBoundingClientRect().bottom;
 if(excess>0){set(c,'height',Math.max(40,natural-excess-2)+'px');set(c,'aspect-ratio','auto');}
 if(card.getBoundingClientRect().bottom>main.getBoundingClientRect().bottom){const room=main.getBoundingClientRect().bottom-card.getBoundingClientRect().top-2;const zoom=Math.min(1,room/card.getBoundingClientRect().height);card.style.zoom=String(zoom);card.style.width=(100/zoom)+'%';}
 const r=c.getBoundingClientRect(),size=r.width+'x'+r.height;
 if(size!==lastChartSize&&typeof graphs==='function'){lastChartSize=size;graphs();}
}

function room(host){return Math.max(40,Math.floor(main.getBoundingClientRect().bottom-host.getBoundingClientRect().top-4));}
function fitMenu(menu){
 const items=[...menu.children],height=room(menu),columns=height<items.length*32?2:1;
 const rows=Math.ceil(items.length/columns),row=Math.min(53,Math.floor((height-2)/rows));
 set(menu,'--menu-columns',String(columns));set(menu,'--menu-row-height',row+'px');
}
function fitTemperatures(section){
 const list=section.querySelector('.approvedSensorList'),wide=section.clientWidth>=560&&main.clientHeight<320;
 section.classList.toggle('wideContent',wide);
 const count=wide?2:4,head=Math.max(48,Math.min(74,Math.floor((list.clientHeight-7*(count-1))/count)-2));
 set(section,'--sensor-row-height',head+'px');
}
function fitScroll(host){
 host.classList.add('vpScroll');set(host,'--content-room',room(host)+'px');
}
function focusInPanel(){
 const field=document.activeElement;if(!field||!/^(INPUT|SELECT|TEXTAREA)$/.test(field.tagName))return;
 const host=field.closest('.vpScroll,.approvedSensorList');if(!host)return;
 const f=field.getBoundingClientRect(),r=host.getBoundingClientRect();
 if(f.bottom>r.bottom-3)host.scrollTop+=f.bottom-r.bottom+6;
 else if(f.top<r.top+3)host.scrollTop-=r.top-f.top+6;
}
function fit(){
 if(busy)return;busy=true;
 try{
  const section=[...main.querySelectorAll(':scope>.page')].find(visible);if(!section)return;
  const id=Number(section.dataset.section);
  if(id===7){document.documentElement.classList.remove('viewportFit');document.documentElement.dataset.viewportMode='unchanged';return;}
  document.documentElement.classList.add('viewportFit');document.documentElement.dataset.viewportMode='fit';
  size();if(scrollY!==0)scrollTo(0,0);
  if(id===0){fitHome(section);return;}
  if(id===3){fitGraph();return;}
  if(id===1){fitTemperatures(section);focusInPanel();return;}
  if(id===2){section.classList.toggle('compactContent',main.clientHeight<500);section.classList.toggle('wideContent',section.clientWidth>=560&&main.clientHeight<320);return;}
  if(id===5){
   const menu=$('#settingsMenu');if(visible(menu)){fitMenu(menu);return;}
   const card=[...section.querySelectorAll('.settingCard')].find(visible);if(!card)return;
   if(card.dataset.stab!=='0'&&card.dataset.stab!=='1')fitScroll(card);
   focusInPanel();
  }
 }finally{busy=false;document.documentElement.dataset.viewportReady='1';}
}
document.addEventListener('click',request);
document.addEventListener('toggle',request,true);
document.addEventListener('focusin',request);
document.addEventListener('focusout',request);
if(typeof ResizeObserver!=='undefined'){
 const resize=new ResizeObserver(request);resize.observe(main);
 const dashboard=main.querySelector('.approvedDashboard');if(dashboard)resize.observe(dashboard);
}
new MutationObserver(rows=>{
 const home=main.querySelector('[data-section="0"]'),homeActive=home&&!home.hidden;
 if(rows.some(r=>r.type==='attributes'||!homeActive))request();
}).observe(main,{subtree:true,childList:true,attributes:true,attributeFilter:['hidden','open']});
window.addEventListener('resize',request);window.addEventListener('orientationchange',request);window.visualViewport?.addEventListener('resize',request);
window.addEventListener('pageshow',request);
request();
})();

/* BUSA_VIEWPORT_JS_END */
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
  if(otaRestart && elapsed(millis(),otaRestartAt,4000)){soundOutputsOff();ESP.restart();}
  // Avoid I2C/1-Wire/SPI scanning while Wi-Fi OTA flash chunks arrive.
  // Flash throughput remains hardware/network limited.
  if(otaActive){soundOutputsOff();delay(0);return;}
  if(tftRebootPending && elapsed(millis(),tftRebootAt,3000)){soundOutputsOff();ESP.restart();}
  if(googleRestart && elapsed(millis(),googleRestartAt,3500)){soundOutputsOff();ESP.restart();}
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
  busaGoogleSnapshotPoll(); // Main-loop-only capture; worker owns immutable JSON.
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
