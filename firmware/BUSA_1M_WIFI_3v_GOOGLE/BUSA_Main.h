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
#include <ArduinoJson.h>
#include <utility>
#include <WiFi.h>
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
#include <math.h>
#include "BUSA_WaterFlowMath.h"
#include <BUSA_LoopProfile.h>
#include <BUSA_KeyEvents.h>
#include "BUSA_DisplayLink.h"
#include "BUSA_LocalRequest.h"
#include "BUSA_HttpServer.h"
#define DECODE_NEC
#define DECODE_SAMSUNG
#define DECODE_SONY
#define DECODE_RC5
#define DECODE_RC6
#define NO_LED_FEEDBACK_CODE
#include <IRremote.hpp>
#include "BUSA_IrKeys.h"

constexpr uint8_t TPIN[4]={16,17,18,21};
constexpr uint8_t KPIN[4]={7,6,5,4}; // Four 12 V sound-annunciator relay driver inputs ONLY
constexpr uint8_t BUS_SDA=40,BUS_SCL=41; // Isolated PCF8575 keypad only
constexpr uint8_t IR_GPIO=42; // input only; proposed and approved for IR receiver
static_assert(IR_GPIO!=BUS_SDA&&IR_GPIO!=BUS_SCL&&IR_GPIO!=busa_uart::mainRx&&IR_GPIO!=busa_uart::mainTx,"IR GPIO conflict");
constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V
constexpr uint8_t COLLECTION_GPIO=1; // Input, verified 3.3 V logic required
static_assert(COLLECTION_GPIO==1 && COLLECTION_GPIO!=FLOW_GPIO,"Collection GPIO changed");
constexpr uint8_t BUSA_RESERVED_BUZZER_GPIO=2; // active 3.3V logic buzzer input, T1/T2
constexpr bool RELAY_PHYSICAL_OUTPUTS=true;
static_assert(RELAY_PHYSICAL_OUTPUTS,"K1-K4 sound-only relays; no PWM equipment outputs");
static_assert(BUSA_RESERVED_BUZZER_GPIO!=FLOW_GPIO && BUSA_RESERVED_BUZZER_GPIO!=BUS_SDA &&
  BUSA_RESERVED_BUZZER_GPIO!=BUS_SCL && BUSA_RESERVED_BUZZER_GPIO!=COLLECTION_GPIO &&
  BUSA_RESERVED_BUZZER_GPIO!=busa_uart::mainTx && BUSA_RESERVED_BUZZER_GPIO!=busa_uart::mainRx,
  "Buzzer pin conflicts with I2C, UART or flow pins");
static_assert(FLOW_GPIO==15 && TPIN[0]==16 && TPIN[1]==17 && TPIN[2]==18 && TPIN[3]==21 && BUS_SDA==40 && BUS_SCL==41,"GPIO changed");
const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-CALC4-WIFI1-UI16-CUBE1-20261121-GEOM1-MOTION1-20261122-YFS401-PPL1-20261123-FLOWSTATE1-20261124-WATERLIVE1-UART2-COLL1-DISPLAYOTA1-P1-20261125-HIST1-LIQCFG1-IR1-FAST1-RESET1-KEY1-FLOWUI1-SOUND30-PERF4-HW2-NAME1-TIME1";
// Diagnostic logging targets both the CH343 UART0 and native USB CDC (COM8).
uint32_t busaBootAt=0,busaSetupDoneMs=0,busaSlowLoops=0,busaMaxLoopMs=0;
static busa_perf::LoopProfile busaLoopProfile;
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
  // Each invocation performs at most one DallasTemperature bus operation.
  // The last conversion request gets the full 825 ms before any bus is read.
  enum Phase {Idle,Discover,Address,Resolution,Request,Wait,Read};
  static Phase phase=Idle;
  static uint8_t index=0;
  if(phase==Idle && (tempCycle==0||elapsed(now,tempCycle,2500))){
    tempCycle=now;tempConverting=true;index=0;phase=Discover;
  }
  if(phase==Wait && elapsed(now,tempRequestAt,825)){index=0;phase=Read;}
  switch(phase){
    case Discover:
      if(!tempInitialized[index]||(!tempPresent[index]&&elapsed(now,tempDiscoverAt[index],12000))){
        sensors[index]->begin();tempInitialized[index]=true;tempDiscoverAt[index]=millis();phase=Address;
      }else phase=Request;
      break;
    case Address:
      tempPresent[index]=sensors[index]->getAddress(cachedTempRom[index],0);
      cachedTempValid[index]=tempPresent[index];phase=tempPresent[index]?Resolution:Request;
      break;
    case Resolution:
      sensors[index]->setResolution(cachedTempRom[index],12);
      sensors[index]->setWaitForConversion(false);phase=Request;
      break;
    case Request:
      if(tempPresent[index])sensors[index]->requestTemperatures();
      else{temperature[index]=NAN;tempAt[index]=0;}
      if(++index==4){tempRequestAt=millis();phase=Wait;}else phase=Discover;
      break;
    case Read:{
      const int i=index;
      float v=cachedTempValid[i]?sensors[i]->getTempC(cachedTempRom[i]):DEVICE_DISCONNECTED_C;
      if(tempPresent[i] && isfinite(v) && v>=-55 && v<=125 && v!=DEVICE_DISCONNECTED_C){
        temperature[i]=v+tempOffset[i];tempAt[i]=millis();
      }else{
        temperature[i]=NAN;tempAt[i]=0;
        tempPresent[i]=false;cachedTempValid[i]=false;
      }
      if(++index==4){tempConverting=false;phase=Idle;}
      break;
    }
    default:break;
  }
  const uint32_t freshNow=millis();
  for(int i=0;i<4;i++)if(tempAt[i] && elapsed(freshNow,tempAt[i],7000)){
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
volatile uint32_t pulseCounter=0,flowRejectedCounter=0,flowLastAcceptedUs=0;
void IRAM_ATTR flowISR(){
  const uint32_t at=micros();
  portENTER_CRITICAL_ISR(&flowMux);
  if(!BusaWater::acceptEdge(flowLastAcceptedUs,at))++flowRejectedCounter;
  else{flowLastAcceptedUs=at;++pulseCounter;}
  portEXIT_CRITICAL_ISR(&flowMux);
}
uint32_t flowPulses(){
  uint32_t n;
  portENTER_CRITICAL(&flowMux);n=pulseCounter;portEXIT_CRITICAL(&flowMux);
  return n;
}
uint32_t flowRejectedEdges(){
  uint32_t n;
  portENTER_CRITICAL(&flowMux);n=flowRejectedCounter;portEXIT_CRITICAL(&flowMux);
  return n;
}
// Passport default: 98 Hz at 1 L/min = 5880 pulses/L; manual value stored in NVS.
float waterPulsesPerLitre=BusaWater::kPassportPulsesPerLitre;
bool waterCoefficientManual=false;
bool waterCalibrationVerified(){
  return waterCoefficientManual && BusaWater::verifiedBusa001Calibration(waterPulsesPerLitre);
}
uint32_t flowCycle=0,flowLastCount=0,flowLastRejected=0;
float flowLpm=0.0f,flowTrace[40]={};
uint32_t waterPulseHistory[40]={}; // filtered pulse counts, not calibrated
uint32_t waterPulseDelta=0,waterRejectDelta=0,waterSampleDurationMs=0;
bool waterSampleReady=false;
bool flowReady=false,flowSeen=false;
BusaWater::VolumeAccumulator waterVolume;
uint8_t flowTraceHead=0,waterSamplesRecorded=0;
void sampleFlow(uint32_t now){
  if(!flowCycle){flowCycle=now;return;}
  const uint32_t dt=now-flowCycle;
  if(dt<2000)return;
  uint32_t count,rejected;
  portENTER_CRITICAL(&flowMux);
  count=pulseCounter;rejected=flowRejectedCounter;
  portEXIT_CRITICAL(&flowMux);
  const uint32_t diff=count-flowLastCount,rejectedDiff=rejected-flowLastRejected;
  flowLastCount=count;flowLastRejected=rejected;flowCycle=now;
  waterRejectDelta=rejectedDiff;waterSampleDurationMs=dt;waterSampleReady=true;
  const float candidate=BusaWater::lpm(diff,dt,waterPulsesPerLitre);
  flowReady=BusaWater::credible(diff,dt,rejectedDiff,waterPulsesPerLitre);
  waterVolume.record(diff,rejectedDiff,flowReady,waterPulsesPerLitre);
  flowLpm=flowReady?candidate:0.0f;
  if(diff)flowSeen=true;
  waterPulseDelta=diff;
  waterPulseHistory[flowTraceHead]=diff;
  flowTrace[flowTraceHead]=flowReady?flowLpm:NAN;
  flowTraceHead=(flowTraceHead+1)%40;
  if(waterSamplesRecorded<40)waterSamplesRecorded++;
}
float totalWaterLitres(){return (float)waterVolume.countedLitres;}
#include "BUSA_Collection.h"
static BusaCollection::Meter collectionMeter;
static portMUX_TYPE collectionMux=portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t collectionPulseCount=0;
static void IRAM_ATTR collectionISR(){
 portENTER_CRITICAL_ISR(&collectionMux);++collectionPulseCount;portEXIT_CRITICAL_ISR(&collectionMux);
}
static uint64_t collectionPulses(){
 portENTER_CRITICAL(&collectionMux);const uint64_t count=collectionPulseCount;portEXIT_CRITICAL(&collectionMux);return count;
}
static String collectionNumber(double value){
 if(!isfinite(value))return "null";
 char buffer[32];snprintf(buffer,sizeof(buffer),"%.9g",value);return String(buffer);
}
static String collectionDisplay(double value){
 if(!isfinite(value))return "Н/Д";
 char buffer[24];
 for(int precision=4;precision>=1;--precision){
  const int n=snprintf(buffer,sizeof(buffer),"%.*g",precision,value);
  if(n>0&&n<=5)return String(buffer);
 }
 return "Н/Д";
}

// Isolated 3.3V I2C keypad PCF8575/PCF8575A only.
bool i2cOnline=false,pcfDetected=false;
uint8_t pcfAddr=0;uint32_t lastBusProbe=0;
uint8_t i2cFound[24]={0},i2cFoundCount=0;
uint16_t i2cAckCount=0;
uint32_t i2cFullScanAt=0;
// A probe is a sample, not a state change. Keep each device's most recent
// confirmed ACK, but report "online" only while confirmation is fresh.
uint32_t busaI2cLastPcfAckMs=0;
uint32_t busaI2cPcfAcks=0;
uint32_t busaI2cLastPcfNackMs=0;
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

bool pcfIdleReadOk=false;
uint16_t pcfIdleDirect=0xffff;
uint32_t keyCount=0,lastKeyScan=0,pendingAt=0;
int pendingPair=-1;bool keyHeld=false;
String keyMessage="NO DATA";
uint16_t pcfIdleMask=65535,pcfObservedLow=0;
uint8_t pcfPairCount=0,pcfScanError=0;
uint32_t pcfLastScanAt=0;
Preferences prefs;
static bool busaNvsReady=false;
static void loadCollectionCalibration(){
 if(busaNvsReady)collectionMeter.setCalibration(prefs.getFloat("collPulseMl",NAN));
}
static bool saveCollectionCalibration(float value){
 if(!busaNvsReady||!BusaCollection::Meter::validCalibration(value))return false;
 if(prefs.putFloat("collPulseMl",value)!=sizeof(float))return false;
 if(prefs.getFloat("collPulseMl",NAN)!=value)return false;
 return collectionMeter.setCalibration(value);
}
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
  pcfDetected=false;pcfAddr=0;
  // Only PCF8575 keypad is installed; no LED driver discovery or reads.
  for(uint8_t j=0;j<i2cFoundCount;j++){
    const uint8_t address=i2cFound[j];
    if(busaI2cIsPcfAddress(address) &&
       (!pcfDetected || address==0x20)){pcfDetected=true;pcfAddr=address;}
  }
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
// GUITION owns physical navigation and editing.
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
static busa_keys::Ring busaKeyEvents;
static uint32_t busaDisplayTestSeq=0;
static void publishPhysicalKey(uint8_t code){if(busa_display_link::keypadCapable())busaKeyEvents.push(code,millis());}
static busa_ir::Map irMap;
static busa_ir::Press irPress;
static const char* irMessage="Пульт не обучен";
static bool irPaused=false;
static void busaIrInit(){
  uint64_t saved[busa_ir::count]={};
  if(busaNvsReady&&prefs.getBytesLength("irKeys1")==sizeof(saved)&&prefs.getBytes("irKeys1",saved,sizeof(saved))==sizeof(saved)&&irMap.load(saved))irMessage="Пульт готов";
  IrReceiver.begin(IR_GPIO);
  pinMode(IR_GPIO,INPUT_PULLUP);
}
static void busaIrPause(){
  if(!irPaused){IrReceiver.stop();irPaused=true;}
  irPress.clear();
  if(irMap.learning()){irMap.cancel();irMessage="Обучение отменено: обновление";}
}
static void busaIrEmit(int key){
  if(key<0||!busa_display_link::keypadCapable()||tftDisplayOff||keypadLearning||irMap.learning())return;
  ++keyCount;tftDisplayActivityMs=millis();
  if(key==1){darkenTftDisplay();keyMessage="ИК: экран погашен";}
  else {if(key==busa_keys::deleteChar&&!busa_display_link::nav4Capable())key=0;publishPhysicalKey(uint8_t(key));keyMessage="ИК: клавиша принята";}
  tftDirty=true;
}
static void busaIrPoll(uint32_t now){
  if(irPaused){IrReceiver.start();pinMode(IR_GPIO,INPUT_PULLUP);irPaused=false;}
  if(irMap.expire(now)){irPress.clear();irMessage="Время обучения истекло";}
  if(!busa_display_link::keypadCapable()||tftDisplayOff||keypadLearning||irMap.learning())irPress.clear();
  if(!IrReceiver.decode()){irPress.expire(now,busaIrEmit);return;}
  const auto data=IrReceiver.decodedIRData;
  IrReceiver.resume();
  if(data.protocol==UNKNOWN||(data.flags&(IRDATA_FLAGS_PARITY_FAILED|IRDATA_FLAGS_WAS_OVERFLOW)))return;
  const uint64_t token=busa_ir::identity(uint8_t(data.protocol==NEC2?NEC:data.protocol),data.address,data.command);
  const bool repeat=(data.flags&(IRDATA_FLAGS_IS_REPEAT|IRDATA_FLAGS_IS_AUTO_REPEAT))!=0;
  if(irMap.learning()){
    const auto result=irMap.capture(token,repeat);
    if(result==busa_ir::LearnResult::duplicate)irMessage="Эта кнопка уже записана";
    else if(result==busa_ir::LearnResult::accepted)irMessage="Нажмите следующую кнопку";
    else if(result==busa_ir::LearnResult::complete){
      uint64_t check[busa_ir::count]={};
      const size_t bytes=sizeof(check);
      const bool saved=busaNvsReady&&prefs.putBytes("irKeys1",irMap.staged(),bytes)==bytes&&prefs.getBytes("irKeys1",check,bytes)==bytes&&!memcmp(check,irMap.staged(),bytes);
      irMessage=irMap.finish(saved)?"Пульт обучен и сохранён":"Ошибка сохранения; прежняя раскладка в работе";
    }
    irPress.suppress(token,now);return;
  }
  const int key=irMap.action(token);
  if(keypadLearning||!busa_display_link::keypadCapable()||key==busa_ir::none)return;
  if(tftDisplayOff){if(!repeat){wakeTftDisplay();keyMessage="ИК: экран пробуждён";}irPress.suppress(token,now);return;}
  irPress.frame(token,key,repeat,now,busaIrEmit);
}
String busaUartStatusJSON(){
  String s=statusJSON();
  if(busa_display_link::compactStatusCapable()){
    JsonDocument filter, compact;
    const char* fields[]={"device","firmware","uptime_ms","display_references","calibration_offsets","display_ota_supported","tft_theme_mode"};
    for(const auto* field:fields)filter[field]=true;
    filter["clock"]["datetime"]=true;
    filter["temperatures"][0]["c"]=true;
    filter["sound_alerts"]["channels"][0]["sound_on"]=true;
    filter["water"]["lpm"]=true;filter["water"]["total_l_estimate"]=true;filter["water"]["pulses_per_litre"]=true;
    filter["collection"]["ml_per_min"]=true;filter["collection"]["total_ml"]=true;filter["collection"]["pulses_per_ml"]=true;
    filter["collection"]["duration_ms"]=true;filter["collection"]["elapsed_ms"]=true;
    filter["station"]["connected"]=true;filter["station"]["ssid"]=true;
    filter["network_indicators"]["internet"]=true;
    filter["google"]["configured"]=true;filter["google"]["state"]=true;
    filter["modules"]["pcf8575"]=true;filter["keypad"]["message"]=true;filter["keypad"]["expected"]=true;
    filter["performance"]["i2c_errors"]=true;filter["performance"]["loop_worst_ms"]=true;filter["performance"]["psram_total_bytes"]=true;
    filter["diagnostics"]["water_flow"]["status"]=true;
    filter["diagnostics"]["esp32"]["free_heap_bytes"]=true;filter["diagnostics"]["wifi"]["sta_connected"]=true;
    filter["esp32"]["free_heap_bytes"]=true;filter["wifi"]["sta_connected"]=true;
    if(!filter.overflowed()&&!deserializeJson(compact,s,DeserializationOption::Filter(filter))&&!compact.overflowed()){
      String shortStatus;
      if(shortStatus.reserve(measureJson(compact)+1)&&serializeJson(compact,shortStatus)>0&&shortStatus.endsWith("}"))s=std::move(shortStatus);
    }
  }
  if(!busa_display_link::keypadCapable()||!s.endsWith("}"))return s;
  s.remove(s.length()-1);
  s+=",\"keypad_input\":{\"boot\":"+String(busaKeyEvents.boot())+",\"head\":"+String(busaKeyEvents.head());
  s+=",\"sleep\":"+String(tftDisplayOff?"true":"false")+",\"test_seq\":"+String(busaDisplayTestSeq)+",\"events\":[";
  const uint32_t now=millis();
  for(size_t i=0;i<busaKeyEvents.size();++i){const auto& e=busaKeyEvents.oldest(i);if(i)s+=",";
    s+="["+String(e.seq)+","+String(e.code)+","+String(uint32_t(now-e.time))+"]";}
  s+="]}}";return s;
}
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
    if(pair==-1 && keypadDeferredZero && keyHeld && !keypadLearning)
      publishPhysicalKey((uint32_t)(now-pendingAt)>2000?busa_keys::longZero:17);
    pendingPair=pair;pendingAt=now;keyHeld=false;keypadDeferredZero=false;
  }
  if(pair<0)return;
  if(keyHeld){
    if(keypadDeferredZero && (uint32_t)(now-pendingAt)>2000){
      keypadDeferredZero=false;
      publishPhysicalKey(busa_keys::longZero);
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
      if(!busa_display_link::keypadCapable()){keyMessage="Клавиатура: требуется совместимый GUITION";return;}
      keyMessage="Клавиша принята";
      if(k==1){darkenTftDisplay();keyMessage="Команда: погасить экран";}
      else if(k==17)keypadDeferredZero=true;
      else publishPhysicalKey(busa_display_link::nav5Capable()?busa_keys::matrixCode(uint8_t(k)):uint8_t(k));
    }
  }
  tftDirty=true;
}

// No SPI display is attached to the main ESP32.
bool tftInverted=false,tftThemeDark=false;
bool tftRebootPending=false;uint32_t tftRebootAt=0;
const char* pages[8]={"Главная","Температуры","Отбор","Графики","События","Настройки","Диагностика","Калькулятор"};
const uint8_t UI_ORDER[6]={0,1,2,3,7,5};
void navigatePage(uint8_t target);
uint8_t orderSlot(uint8_t page){if(page==4||page==6)return 5;for(uint8_t i=0;i<6;i++)if(UI_ORDER[i]==page)return i;return 0;}
void moveApprovedTab(int direction){int slot=(orderSlot(currentPage)+6+direction)%6;navigatePage(UI_ORDER[slot]);}
extern String stationSsid,otaMessage,nearbyNames[5];
extern uint8_t nearbyCount;
void wakeTftDisplay(){tftDisplayOff=false;tftDisplayActivityMs=millis();}
void darkenTftDisplay(){if(busa_display_link::keypadCapable())tftDisplayOff=true;}
bool busaSoundEnabled(uint8_t i);bool busaSoundActive(uint8_t i);
extern volatile bool googleEnabled;
extern volatile int googleInternet,googleHttp;
#include "BUSA_ChartHistory.h"
static BusaChart::History busaChartHistory;
static bool busaChartReady=false,busaChartAttempted=false;
static void busaCaptureChart(uint32_t now,uint32_t scheduledAt){
 if(!busaChartAttempted){
  busaChartAttempted=true;
  auto* memory=(BusaChart::Row*)heap_caps_malloc(sizeof(BusaChart::Row)*1801,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(memory){busaChartHistory.begin(memory,1801,0);busaChartReady=true;}
 }
 if(!busaChartReady)return;
 float values[4];for(int k=0;k<4;k++)values[k]=hasTemp(k,now)?temperature[k]:NAN;
 // Use the vetted two-second sample, not independent raw pulse differences.
 const float cooling=flowReady?flowLpm*60.0f:NAN;
 busaChartHistory.capture(scheduledAt,values,cooling,(float)collectionMeter.raw());
}
#include "BUSA_History.h"

BusaHttpServer server(80);
static esp_err_t busaOtaConfirmResult=ESP_ERR_NOT_FOUND;
static esp_err_t busaOtaSelectResult=ESP_ERR_NOT_SUPPORTED;
static String busaOtaRunning="unknown",busaOtaBoot="unknown";
bool otaActive=false,otaSuccess=false,otaRestart=false,otaWriteAllowed=false;
uint32_t otaBytes=0,otaRestartAt=0,otaTokenIssued=0,startedAt=0;
String otaToken="",otaMessage="Ожидание",nearbyNames[5];
uint8_t nearbyCount=0;
volatile bool busaWifiScanRunning=false;
uint32_t busaWifiScanStartedAt=0,busaWifiScanLastAt=0;

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
  s+=",\"loop_profile\":{\"last\":"+quoteJson(busa_perf::LoopProfile::name(busaLoopProfile.last()))+",\"max_ms\":{";
  for(size_t i=0;i<busa_perf::stageCount;++i){if(i)s+=",";s+=quoteJson(busa_perf::LoopProfile::name(i))+":"+String(busaLoopProfile.worst(i));}
  s+="},\"slow_count\":{";
  for(size_t i=0;i<busa_perf::stageCount;++i){if(i)s+=",";s+=quoteJson(busa_perf::LoopProfile::name(i))+":"+String(busaLoopProfile.slow(i));}
  s+="}}";
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
  s+=",\"http_worker_enabled\":"+(server.workerEnabled()?String("true"):String("false"));
  s+=",\"http_worker_worst_ms\":"+String(server.workerWorstMs());
  s+=",\"http_response_failures\":"+String(server.responseFailures());
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
  s+=",\"volume_complete\":";s+=(waterVolume.complete()?"true":"false");
  s+=",\"volume_excluded_intervals\":"+String(waterVolume.excludedIntervals);
  s+=",\"calibrated\":";s+=(waterCalibrationVerified()?"true":"false");
  s+=",\"seen_pulses\":"+(flowSeen?String("true"):String("false"));
   s+=",\"pulses_per_litre\":"+String(waterPulsesPerLitre,1);
   s+=",\"coefficient_source\":"+quoteJson(waterCoefficientManual?"manual":"passport");
  s+=",\"credible\":"+(flowReady?String("true"):String("false"));
  s+=",\"rejected_edges\":"+String(flowRejectedEdges());
  s+=",\"pulses_window\":"+String(waterPulseDelta);
  s+=",\"rejected_edges_window\":"+String(waterRejectDelta);
  s+=",\"sample_window_ms\":"+String(waterSampleDurationMs);
  s+=",\"sample_ready\":";s+=(waterSampleReady?"true":"false");s+="}";
  s+=",\"collection\":{\"gpio\":1,\"model\":\"YF-S401\",\"installed\":true,\"pulse_seen\":"+String(collectionMeter.seen()?"true":"false");
  // Historical field retained for older clients: evidence of pulses, not physical presence.
  s+=",\"sensor_present\":"+String(collectionMeter.seen()?"true":"false");
  char collectionCount[24];snprintf(collectionCount,sizeof(collectionCount),"%llu",(unsigned long long)collectionPulses());
  s+=",\"pulses\":"+String(collectionCount)+",\"calibrated\":"+String(collectionMeter.calibrated()?"true":"false");
  s+=",\"pulses_per_ml\":"+collectionNumber(collectionMeter.calibration());
  s+=",\"ml_per_min\":"+collectionNumber(collectionMeter.rate())+",\"total_ml\":"+collectionNumber(collectionMeter.total())+"}";
  s+=",\"modules\":{\"pcf8575\":"+(pcfDetected?String("true"):String("false"));
  s+=",\"tft_attempted\":"+(busa_display_link::lcdInitialized()?String("true"):String("false"));
  s+=",\"tft_rotation\":1";
  s+=",\"tft_boot_complete\":"+(busa_display_link::lcdInitialized()?String("true"):String("false"));
  s+=",\"tft_profile_id\":1,\"tft_profile\":\"GUITION\"";
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
  s+=",\"ir_remote\":{\"ready\":"+String(irMap.ready()?"true":"false")+",\"learning\":"+String(irMap.learning()?"true":"false");
  s+=",\"step\":"+String(unsigned(irMap.step()))+",\"expected\":"+quoteJson(irMap.learning()&&irMap.step()<busa_ir::count?String(busa_ir::bindings[irMap.step()].label):String(""));
  s+=",\"message\":"+quoteJson(String(irMessage))+"}";
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
  s+=",\"ack_fresh_ms\":"+String(BUSA_I2C_ACK_FRESH_MS);
  s+=",\"pcf_last_ack_ms\":"+String(busaI2cLastPcfAckMs);
  s+=",\"pcf_ack_total\":"+String(busaI2cPcfAcks);
  s+=",\"pcf_last_nack_ms\":"+String(busaI2cLastPcfNackMs);
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
  s+=",\"tft\":{\"profile\":\"GUITION\",\"model\":\"JC4827W543R_I\",\"width\":480,\"height\":272,\"rotation\":1";
  s+=",\"uart_tx\":"+String(busa_uart::mainTx)+",\"uart_rx\":"+String(busa_uart::mainRx);
  s+=",\"init_sent\":"+(busa_display_link::lcdInitialized()?String("true"):String("false"));
  s+=",\"boot_complete\":"+(busa_display_link::lcdInitialized()?String("true"):String("false"));
  s+=",\"pixel_feedback_available\":false}";
  s+=",\"water_flow\":{\"gpio\":15,\"input_level\":"+String(digitalRead(FLOW_GPIO));
  s+=",\"pulses\":"+String(flowPulses())+",\"rejected_edges\":"+String(flowRejectedEdges());
  s+=",\"pulse_seen\":"+(flowSeen?String("true"):String("false"));
  s+=",\"pulses_per_litre\":"+String(waterPulsesPerLitre,1);
  s+=",\"frequency_calibrated\":";s+=(waterCalibrationVerified()?"true":"false");
  s+=",\"status\":"+quoteJson(BusaWater::diagnosticStatus(waterSampleReady,flowReady,waterPulseDelta))+"}";
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
  s+=",\"display_diagnostics\":"+busa_display_link::diagnosticJSON();
  s+=",\"display_ota_supported\":true";
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
static busa_ota::Marker mainOtaMarker(busa_ota::mainFamily);
void uploadOTA(){
 HTTPUpload &u=server.upload();
 if(u.status==UPLOAD_FILE_START){
  otaWriteAllowed=false;otaActive=false;otaSuccess=false;otaBytes=0;mainOtaMarker.reset();
  if(busa_display_ota::busy()){otaMessage="Обновляется экран";return;}
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
  mainOtaMarker.feed(u.buf,u.currentSize);
  if(Update.write(u.buf,u.currentSize)!=u.currentSize){
    otaMessage="Ошибка записи";Update.printError(Serial0);
    Update.abort();otaActive=false;otaWriteAllowed=false;return;
  }
  otaBytes+=u.currentSize;
 }else if(u.status==UPLOAD_FILE_END){
  if(!otaWriteAllowed)return;
  if(!mainOtaMarker.found()){Update.abort();otaActive=false;otaWriteAllowed=false;otaMessage="BIN не для основной платы";return;}
  otaSuccess=Update.end(true);otaActive=false;otaWriteAllowed=false;
  otaMessage=otaSuccess?"Успешно, перезагрузка":"BIN не прошёл проверку";
  if(!otaSuccess)Update.printError(Serial0);
  if(otaSuccess){otaRestart=true;otaRestartAt=millis();}
 }else if(u.status==UPLOAD_FILE_ABORTED){
  if(otaWriteAllowed)Update.abort();
  otaWriteAllowed=false;otaActive=false;otaMessage="Отменено";
 }
}
static String displayOtaToken;
static uint32_t displayOtaTokenAt=0,displayOtaSize=0;
static bool displayUploadOpen=false,displayUploadOk=false;
static void uploadDisplayOTA(){
 HTTPUpload& u=server.upload();
 if(u.status==UPLOAD_FILE_START){
  displayUploadOpen=false;displayUploadOk=false;
  if(!apClient()||otaActive||otaRestart||busa_display_ota::busy()||!busa_display_link::otaCapable()||
     displayOtaToken.length()<16||uint32_t(millis()-displayOtaTokenAt)>90000||
     server.header("X-Busa-Token")!=displayOtaToken||!u.filename.endsWith(".bin"))return;
  displayOtaToken="";otaToken="";
  displayUploadOpen=busa_display_ota::begin(displayOtaSize);
 }else if(u.status==UPLOAD_FILE_WRITE){
  if(displayUploadOpen&&!busa_display_ota::append(u.buf,u.currentSize))displayUploadOpen=false;
 }else if(u.status==UPLOAD_FILE_END){
  if(displayUploadOpen)displayUploadOk=busa_display_ota::seal();
  displayUploadOpen=false;
 }else if(u.status==UPLOAD_FILE_ABORTED){
  if(displayUploadOpen)busa_display_ota::abort();
  displayUploadOpen=false;displayUploadOk=false;
 }
}
#include "BUSA_Google.h"
#include "BUSA_CalculatorWeb.h"
const char WEB_PAGE[] PROGMEM=R"HTMLBUSA(
<!doctype html><html lang="ru" class="busaDesign"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BUSA MONITOR</title>
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
/* BUSA_DESIGN_CSS_BEGIN */
/* UI15 foundation; UI16 holographic presentation is scoped below. */
html:root.busaDesign{color-scheme:dark;--ink:#e6f2f7;--muted:#9cb4c2;--paper:#061521;--surface:#0c2132;--line:#244253;--sea:#2087ed;--orange:#2087ed;--warn:#ff6473;--border:#36576c;--water:#38bdf8;--cube:#f4c84a;--column:#53d486;--alarm:#ff6473;--design-accent:#2087ed;--design-icon:#75d4f4;--design-selected:#fff;--design-green:#35d68c;--design-radius:14px;background:var(--paper);color:var(--ink)}
html:root.busaDesign[data-theme="light"]{color-scheme:light;--ink:#19384b;--muted:#536e7f;--paper:#f3f7fa;--surface:#fff;--line:#c9dce6;--border:#87a8bb;--design-icon:#087fae}
html:root.busaDesign body{background:var(--paper)!important;color:var(--ink);font-size:14px;-webkit-font-smoothing:antialiased}
html:root.busaDesign h1,html:root.busaDesign h2,html:root.busaDesign h3{color:var(--ink)}
html:root.busaDesign button,html:root.busaDesign input,html:root.busaDesign select,html:root.busaDesign textarea{background:var(--surface)!important;color:var(--ink)!important;border:1px solid var(--border);border-radius:var(--design-radius);box-shadow:none!important}
html:root.busaDesign input,html:root.busaDesign select,html:root.busaDesign textarea{font-size:16px}
html:root.busaDesign button:focus-visible,html:root.busaDesign input:focus-visible,html:root.busaDesign select:focus-visible{outline:2px solid var(--design-accent);outline-offset:2px}
html:root.busaDesign .appHeader{background:var(--paper)!important;border-bottom:1px solid var(--line)!important;box-shadow:none;padding:9px 12px 8px}
html:root.busaDesign .headerRow{max-width:596px;margin:0 auto}
html:root.busaDesign .appHeader h1{font-size:clamp(18px,5.1vw,22px);line-height:1.2;letter-spacing:.8px;font-weight:750;margin:0 0 5px}
html:root.busaDesign #clockDisplay{max-width:596px;margin:0 auto;color:var(--muted)!important;font-size:12px!important;line-height:1.3}
html:root.busaDesign .netIndicators{gap:8px;flex-wrap:nowrap;margin:7px auto 0;max-width:596px;font-size:12px;line-height:1.2}
html:root.busaDesign .appHeader .netPill{position:relative;flex:0 1 auto;gap:8px;min-width:0;white-space:nowrap;padding:5px 11px;background:transparent!important;border:1px solid var(--border)!important;border-radius:10px;color:var(--ink)!important;font-size:12px;font-weight:650;line-height:1.2}
html:root.busaDesign .netDot{width:9px;height:9px;flex-basis:9px;box-shadow:none}
/* Keep the live connection words available to assistive technology. */
html:root.busaDesign .netText{position:absolute;width:1px;height:1px;overflow:hidden;clip-path:inset(50%);white-space:nowrap;padding:0;margin:-1px}
html:root.busaDesign .appTabs{display:grid;grid-template-columns:repeat(6,minmax(0,1fr))!important;gap:3px;max-width:620px;background:var(--paper)!important;border-top:1px solid var(--line)!important;box-shadow:none!important;padding:6px 5px max(6px,env(safe-area-inset-bottom))!important;overflow:hidden}
html:root.busaDesign .appTabs button{display:flex;flex-direction:column;align-items:center;justify-content:center;min-width:0;min-height:58px;margin:0;padding:5px 1px!important;gap:3px;background:transparent!important;border:0!important;border-radius:14px!important;color:var(--muted)!important;font-size:clamp(7px,2.2vw,10px)!important;font-weight:500;white-space:normal;line-height:1.15}
html:root.busaDesign .appTabs button.on{background:var(--design-accent)!important;color:var(--design-selected)!important;font-weight:700;border:0!important;box-shadow:none!important}
html:root.busaDesign .appTabs button span{min-height:2.3em;display:flex;align-items:center;justify-content:center;line-height:1.15;max-width:100%}
html:root.busaDesign .appTabs .uiIcon{width:23px;height:23px;color:var(--design-icon)}
html:root.busaDesign .appTabs button.on .uiIcon{color:var(--design-selected)}
html:root.busaDesign .approvedPageTitle{font-size:18px;line-height:1.3;letter-spacing:.3px;gap:9px;margin:13px 0 12px}
html:root.busaDesign .approvedPageTitle .uiIcon{width:25px;height:25px;color:var(--design-icon)}
html:root.busaDesign main .card,html:root.busaDesign .chartCard,html:root.busaDesign .liquidPanel{background:var(--surface)!important;border:1px solid var(--line)!important;border-radius:var(--design-radius);box-shadow:none!important;color:var(--ink)}
html:root.busaDesign .hint{color:var(--muted)}
html:root.busaDesign .row{border-color:var(--line)}
html:root.busaDesign .approvedBack{background:var(--surface)!important;border-color:var(--line)!important;color:var(--ink);min-height:38px;padding:8px 11px;margin:0 0 9px;font-size:13px;border-radius:12px}
html:root.busaDesign #settingsMenu{background:transparent!important;border:0!important;border-radius:0;box-shadow:none!important}
html:root.busaDesign #settingsMenu .approvedMenuItem{height:100%;min-height:0;background:transparent!important;border-radius:0!important;border:0!important;border-bottom:1px solid var(--line)!important;padding:0 8px;gap:13px;color:var(--ink)!important;font-size:15px;font-weight:500}
html:root.busaDesign #settingsMenu .approvedMenuItem:last-child{border-bottom:0!important}
html:root.busaDesign .approvedMenuIcon{width:28px;height:28px;color:var(--design-icon)}
html:root.busaDesign .approvedMenuIcon .uiIcon{width:25px;height:25px}
html:root.busaDesign .approvedArrow{font-size:27px;color:var(--design-icon)}
html:root.busaDesign .settingsTabs button.on,html:root.busaDesign .chartTabs button.on,html:root.busaDesign .chartRanges button.on,html:root.busaDesign #temperatureSave,html:root.busaDesign #tempCalSave,html:root.busaDesign #save{background:var(--design-accent)!important;border-color:var(--design-accent)!important;color:var(--design-selected)!important;box-shadow:none!important}
/* Reuse the approved apparatus and all of its separate live animation layers. */
html:root.busaDesign section[data-section="0"]{--ink:inherit;--paper:inherit;--surface:inherit;--line:inherit;background:transparent!important;border:0!important;box-shadow:none!important}
html:root.busaDesign section[data-section="0"] .relayIndicators{gap:5px;margin:6px 0 7px!important}
html:root.busaDesign section[data-section="0"] .relayBadge{background:transparent;border:1px solid var(--line);border-radius:10px;padding:5px 4px;min-height:35px;font-size:clamp(8px,2.35vw,11px);line-height:1.2;box-shadow:none}
html:root.busaDesign section[data-section="0"] .relayBadge small{color:var(--muted);font-size:8px}
html:root.busaDesign section[data-section="0"] .relayBadge[data-state="on"]{background:#14382e;border-color:var(--design-green);color:#d0fbe7}
html:root.busaDesign section[data-section="0"] #relayBadge3[data-state="on"]{background:#42232d;border-color:var(--alarm);color:#ffb3bd}
html:root.busaDesign section[data-section="0"] .approvedTempTitle{font-weight:650}
html:root.busaDesign section[data-section="0"] .temp1 strong,html:root.busaDesign section[data-section="0"] .temp3 strong{font-size:clamp(20px,5.8vw,25px);letter-spacing:-.4px}
html:root.busaDesign section[data-section="0"] .approvedTemp small{color:var(--muted)!important}
html:root.busaDesign section[data-section="0"] .temp2 .approvedTempTitle{color:#e6f2f7}
html:root.busaDesign .cubeSetBelow,html:root.busaDesign .collectionRateLabel{color:var(--muted)}
html:root.busaDesign .flowOnPipe span,html:root.busaDesign .waterUsedOnPipe span{color:var(--ink)}
html:root.busaDesign section[data-section="1"] .approvedSensor{background:transparent!important;border:0!important;box-shadow:none!important}
html:root.busaDesign section[data-section="1"] .approvedSensorList{gap:7px}
html:root.busaDesign section[data-section="1"] .approvedSensorHead{gap:12px;padding:7px 25px 7px 6px}
html:root.busaDesign section[data-section="1"] .approvedSensorText strong{font-size:15px}
html:root.busaDesign section[data-section="1"] .approvedSensorText small{font-size:11px;color:var(--muted)}
html:root.busaDesign section[data-section="1"] .approvedSensorLive{font-size:clamp(21px,6.4vw,29px);font-weight:700;letter-spacing:-.6px}
html:root.busaDesign section[data-section="1"] .approvedSensorIcon{width:30px;flex-basis:30px}
html:root.busaDesign section[data-section="1"] .temp0 .approvedSensorIcon{color:var(--water)}
html:root.busaDesign section[data-section="1"] .temp2 .approvedSensorIcon{color:var(--cube)}
html:root.busaDesign section[data-section="1"] .temp1 .approvedSensorIcon{color:var(--column)}
html:root.busaDesign section[data-section="1"] .temp3 .approvedSensorIcon{color:var(--alarm)}
html:root.busaDesign section[data-section="1"] .busaRefTempIcon{filter:none!important;width:29px;height:36px}
html:root.busaDesign section[data-section="1"] .busaRefTempIcon path,html:root.busaDesign section[data-section="1"] .busaRefTempIcon rect,html:root.busaDesign section[data-section="1"] .busaRefTempIcon ellipse{fill:none;stroke:currentColor;stroke-width:2.5}
html:root.busaDesign #temperatureSave{min-height:43px;font-size:15px;font-weight:600;margin:0 0 8px;border-radius:13px}
html:root.busaDesign .liquidPanel{padding:16px;margin:0 0 12px}
html:root.busaDesign .liquidPanelHeading{margin-bottom:13px}
html:root.busaDesign .liquidPanelHeading h2{font-size:18px;color:var(--ink)}
html:root.busaDesign .liquidPanelHeading>span{position:absolute;width:1px;height:1px;overflow:hidden;clip-path:inset(50%)}
html:root.busaDesign .liquidRateLabel{font-size:13px;color:var(--muted);margin:0 0 9px}
html:root.busaDesign .liquidRateGrid{gap:13px}
html:root.busaDesign .liquidRateGrid>div{gap:5px}
html:root.busaDesign .liquidRateGrid small{color:var(--muted);font-size:12px}
html:root.busaDesign .liquidRateGrid strong{font-size:clamp(30px,8.4vw,39px);line-height:1.15;letter-spacing:-.5px}
html:root.busaDesign .liquidSummary{border-color:var(--line);margin-top:15px;padding-top:12px;gap:12px}
html:root.busaDesign .liquidSummary>div{font-size:13px;color:var(--muted)}
html:root.busaDesign .liquidSummary strong{font-size:24px}
html:root.busaDesign .chartCard{padding:12px;margin:0}
html:root.busaDesign .chartTabs,html:root.busaDesign .chartRanges{gap:6px}
html:root.busaDesign .chartTabs button,html:root.busaDesign .chartRanges button{min-width:0;min-height:38px;padding:7px 2px;font-size:12px;font-weight:500;border-radius:11px;box-shadow:none!important}
html:root.busaDesign .chartRanges button{min-height:35px;font-size:11px}
html:root.busaDesign .chartHeading{margin:12px 0 9px;font-size:13px;color:var(--ink)}
html:root.busaDesign .chartHeading span{color:var(--muted)}
html:root.busaDesign .chartLegend{gap:7px 12px;margin:9px 0 12px;color:var(--muted);font-size:11px}
html:root.busaDesign .chartLegend>span{display:flex;align-items:center;gap:5px;min-width:0}
html:root.busaDesign .chartLegend>span:before{flex:0 0 13px;margin:0}
html:root.busaDesign .chartLegend b{margin-left:auto;color:var(--ink);font-weight:650;white-space:nowrap;font-variant-numeric:tabular-nums}
html:root.busaDesign .chartCadence{margin:9px 0 0;text-align:center;font-size:11px;line-height:1.3;color:var(--muted)}
/* The calculator arithmetic, inputs and storage are unchanged. */
html:root.busaDesign section[data-section="7"]{display:flex;flex-direction:column}
html:root.busaDesign #busaCalculator{width:100%;max-width:none;margin:0;min-height:0;flex:1 1 0;overflow:auto;overscroll-behavior:contain;padding-bottom:10px}
html:root.busaDesign #busaCalculator>h2{font-size:18px;margin:13px 0 12px;letter-spacing:.3px}
html:root.busaDesign #busaCalculator .calcTabs{gap:6px;margin:0 0 10px}
html:root.busaDesign #busaCalculator .calcTabs button{min-height:47px;padding:7px 3px;font-size:11px;line-height:1.25;border-radius:12px}
html:root.busaDesign #busaCalculator button[aria-selected="true"],html:root.busaDesign #busaCalculator button[type="submit"]{background:var(--design-accent)!important;color:var(--design-selected)!important;border-color:var(--design-accent)!important}
html:root.busaDesign #busaCalculator .calcPanel{padding:16px;background:var(--surface)!important;border-color:var(--line)!important;border-radius:var(--design-radius)}
html:root.busaDesign #busaCalculator .calcPanel h3{font-size:17px;margin:0 0 16px}
html:root.busaDesign #busaCalculator .calcGrid{grid-template-columns:repeat(2,minmax(0,1fr));gap:13px}
html:root.busaDesign #busaCalculator .calcGrid>label:last-child:nth-child(odd){grid-column:1/-1}
html:root.busaDesign #busaCalculator label{margin:0;font-size:12px;gap:8px;color:var(--ink)}
html:root.busaDesign #busaCalculator input{min-height:45px;padding:9px 12px;background:var(--paper)!important;border-color:var(--border);border-radius:12px;font-size:22px}
html:root.busaDesign #busaCalculator .calcActions{gap:9px;margin:18px 0 0}
html:root.busaDesign #busaCalculator .calcActions button{min-height:42px;padding:9px;font-size:15px;border-radius:12px}
html:root.busaDesign #busaCalculator .calcNote{margin-top:13px;color:var(--muted)}
html:root.busaDesign #busaCalculator .calcNote summary{padding:6px 0}
html:root.busaDesign #busaCalculator .calcResult{background:var(--paper);border:1px solid var(--line);color:var(--ink);border-radius:12px}
html:root.busaDesign #busaCalculator .calcError:empty{display:none}
@media(max-width:350px){html:root.busaDesign .netIndicators{gap:6px}html:root.busaDesign .netPill{padding:5px 9px;font-size:11px}html:root.busaDesign #settingsMenu .approvedMenuItem{font-size:13px}html:root.busaDesign section[data-section="1"] .approvedSensorHead{gap:8px;padding-left:1px;padding-right:22px}html:root.busaDesign section[data-section="1"] .approvedSensorText small{font-size:9px}html:root.busaDesign section[data-section="1"] .approvedSensorLive{font-size:20px}html:root.busaDesign .chartTabs button{font-size:10px}html:root.busaDesign .chartLegend{font-size:10px}html:root.busaDesign #busaCalculator .calcTabs button{font-size:10px}html:root.busaDesign #busaCalculator .calcPanel{padding:12px}}
@media(max-height:650px){html:root.busaDesign .approvedPageTitle{margin:9px 0}html:root.busaDesign .liquidPanel{padding:11px;margin-bottom:8px}html:root.busaDesign .liquidPanelHeading{margin-bottom:9px}html:root.busaDesign .liquidSummary{margin-top:10px;padding-top:9px;gap:8px}html:root.busaDesign .liquidRateGrid strong{font-size:28px}html:root.busaDesign .liquidSummary strong{font-size:21px}html:root.busaDesign #busaCalculator .calcPanel{padding:12px}html:root.busaDesign #busaCalculator .calcPanel h3{margin-bottom:12px}html:root.busaDesign #busaCalculator .calcActions{margin-top:12px}}
@media(max-height:500px){html:root.busaDesign .appHeader{padding-top:5px;padding-bottom:5px}html:root.busaDesign .appHeader h1{font-size:16px;margin-bottom:2px}html:root.busaDesign #clockDisplay{font-size:10px!important}html:root.busaDesign .netIndicators{margin-top:4px}html:root.busaDesign .netPill{font-size:10px;padding:3px 8px}html:root.busaDesign .appTabs button{min-height:44px;padding:3px 1px!important;gap:1px}html:root.busaDesign .appTabs .uiIcon{width:18px;height:18px}html:root.busaDesign .approvedPageTitle{display:none}html:root.busaDesign .chartCadence{margin-top:4px}html:root.busaDesign .chartHeading{margin:5px 0}html:root.busaDesign .chartLegend{margin:5px 0}html:root.busaDesign .chartTabs button,html:root.busaDesign .chartRanges button{min-height:28px;padding:4px 2px}}

/* UI16: live holographic UI. All controls and readings remain DOM elements. */
html:root.busaDesign{--paper:#060e15;--surface:#0d2430;--line:#285064;--border:#47778d;--sea:#00a9e6;--orange:#009dde;--design-accent:#009fde;--design-icon:#91ecf4;--glass:linear-gradient(145deg,rgba(27,68,83,.72),rgba(8,24,34,.88) 58%,rgba(14,41,53,.8));--glass-edge:rgba(116,228,249,.5);--glass-shadow:inset 0 1px 0 rgba(171,246,255,.2),0 5px 18px rgba(0,0,0,.12)}
html:root.busaDesign[data-theme="light"]{--paper:#f0f6f8;--surface:#fff;--line:#b6d2df;--border:#7b9eaf;--design-icon:#087fae;--glass:linear-gradient(145deg,#fff,#e7f2f7);--glass-edge:#91bdcf;--glass-shadow:inset 0 1px #fff,0 4px 14px #17445c12}
html:root.busaDesign body{background:radial-gradient(ellipse at 48% 45%,#10303d30,transparent 68%),var(--paper)!important}
html:root.busaDesign .appHeader{background:transparent!important;border-color:transparent!important;padding:9px 13px 7px}
html:root.busaDesign .appHeader h1{display:flex;align-items:center;gap:9px;letter-spacing:.1px;font-weight:750}
html:root.busaDesign .appHeader h1:before{content:"";flex:0 0 19px;width:19px;height:27px;background:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 32'%3E%3Cpath d='M12 1C9 9 2 13 2 21a10 10 0 0020 0c0-5-2-8-5-11l-2 5c2 3 3 5 3 7a6 6 0 01-12 0c0-6 6-11 9-17Z' fill='%2300c9fa'/%3E%3Cpath d='M17 10l-2 5c2 3 3 5 3 7a6 6 0 01-7 6l-2 4c8 2 13-3 13-11 0-5-2-8-5-11Z' fill='%237ee345'/%3E%3C/svg%3E") center/contain no-repeat}
html:root.busaDesign #clockDisplay{font-size:11px!important;color:var(--muted)!important}
html:root.busaDesign .netIndicators{margin-top:7px;gap:8px}
html:root.busaDesign .appHeader .netPill{padding:5px 11px;border-color:var(--glass-edge)!important;border-radius:18px;background:linear-gradient(135deg,#153b4b66,#08182444)!important;box-shadow:inset 0 1px 0 #c5fbff12!important}
html:root.busaDesign .appTabs{width:calc(100% - 14px)!important;margin:0 auto max(5px,env(safe-area-inset-bottom))!important;padding:6px 4px!important;gap:2px;border:1px solid var(--glass-edge)!important;border-radius:21px!important;background:var(--glass)!important;box-shadow:var(--glass-shadow)!important}
html:root.busaDesign .appTabs button{min-height:56px;padding:4px 1px!important;border:1px solid transparent!important;border-radius:15px!important;font-family:Arial,sans-serif!important;font-size:clamp(7px,2.15vw,10px)!important;gap:3px}
html:root.busaDesign .appTabs button.on{background:linear-gradient(150deg,#00b8f659,#0782b855)!important;border:1px solid #4dd6ff66!important;box-shadow:inset 0 1px 0 #b5faff33!important}
html:root.busaDesign .appTabs .uiIcon{width:24px;height:24px}
html:root.busaDesign .appTabs button span{min-height:2.3em}
html:root.busaDesign main .card,html:root.busaDesign .chartCard,html:root.busaDesign .liquidPanel,html:root.busaDesign #busaCalculator .calcPanel{background:var(--glass)!important;border:1px solid var(--glass-edge)!important;border-radius:17px;box-shadow:var(--glass-shadow)!important}
html:root.busaDesign button,html:root.busaDesign input,html:root.busaDesign select,html:root.busaDesign textarea{border-color:var(--border)}
html:root.busaDesign .settingsTabs button.on,html:root.busaDesign .chartTabs button.on,html:root.busaDesign .chartRanges button.on,html:root.busaDesign #temperatureSave,html:root.busaDesign #tempCalSave,html:root.busaDesign #save,html:root.busaDesign #busaCalculator button[aria-selected="true"],html:root.busaDesign #busaCalculator button[type="submit"]{background:linear-gradient(145deg,#00c3f4,#0088d3)!important;border-color:#7aeaff!important;color:#fff!important;box-shadow:inset 0 1px 0 #e4fdff66,0 0 10px #00bfff18!important}
html:root.busaDesign .approvedPageTitle{margin:13px 2px 15px;font-weight:700;gap:10px}
html:root.busaDesign .approvedPageTitle .uiIcon{width:27px;height:27px}
html:root.busaDesign section[data-section="1"] .approvedSensorList{gap:11px}
html:root.busaDesign section[data-section="1"] .approvedSensor{background:var(--glass)!important;border:1px solid var(--glass-edge)!important;border-radius:17px!important;box-shadow:var(--glass-shadow)!important;overflow:hidden}
html:root.busaDesign section[data-section="1"] .temp2{border-color:#e2c66b8a!important}
html:root.busaDesign section[data-section="1"] .temp1{border-color:#55e6ae8a!important}
html:root.busaDesign section[data-section="1"] .temp3{border-color:#f88ca38a!important}
html:root.busaDesign section[data-section="1"] .approvedSensorHead{padding-left:13px;padding-right:28px;gap:12px}
html:root.busaDesign section[data-section="1"] .approvedSensorIcon{width:32px;flex-basis:32px}
html:root.busaDesign section[data-section="1"] .approvedSensorText strong{font-size:16px}
html:root.busaDesign section[data-section="1"] .approvedSensorLive{font-size:clamp(21px,6.2vw,29px);letter-spacing:-.5px}
html:root.busaDesign .sensorEdit>summary{background:transparent!important}
html:root.busaDesign .approvedFieldGrid{padding:0 12px 12px}
html:root.busaDesign #temperatureSave{border-radius:15px;min-height:44px}
html:root.busaDesign #settingsMenu{padding:2px!important;background:var(--glass)!important;border:1px solid var(--glass-edge)!important;border-radius:17px;box-shadow:var(--glass-shadow)!important;overflow:hidden}
html:root.busaDesign #settingsMenu .approvedMenuItem{padding:0 12px;gap:14px;border-bottom-color:var(--line)!important;background:transparent!important}
html:root.busaDesign .approvedMenuIcon{color:var(--design-icon);width:30px}
html:root.busaDesign .liquidPanel{padding:16px;margin-bottom:12px;position:relative;overflow:hidden}
html:root.busaDesign .liquidPanelHeading h2{font-size:19px}
html:root.busaDesign .collectionPanel .liquidRateGrid,html:root.busaDesign .collectionPanel .liquidSummary,html:root.busaDesign .collectionPanel .liquidRateLabel{width:64%}
html:root.busaDesign .collectionPanel .liquidRateGrid strong{font-size:clamp(28px,8.3vw,38px)}
html:root.busaDesign .collectionPanel .liquidSummary{gap:11px}
html:root.busaDesign .collectionPanel .liquidSummary>div{font-size:12px}
html:root.busaDesign .collectionPanel .liquidSummary strong{font-size:23px;white-space:nowrap}
.visualCollectionGlass{position:absolute;right:12px;top:76px;width:27%;height:calc(100% - 95px);min-height:112px;max-height:175px;pointer-events:none}
.visualCollectionGlass svg{width:100%;height:100%;overflow:visible}
.visualCollectionGlass .liquidFill{opacity:0;transition:opacity .2s}
.hasCollectionData .visualCollectionGlass .liquidFill{opacity:1}
.visualCollectionGlass .glassBubble{animation:glassBubbleRise 3s linear infinite;animation-play-state:paused;transform-origin:center;opacity:0}
.hasCollectionFlow .visualCollectionGlass .glassBubble{animation-play-state:running}
@keyframes glassBubbleRise{0%{transform:translateY(20px);opacity:0}15%{opacity:.5}85%{opacity:.5}100%{transform:translateY(-80px);opacity:0}}
.coolingPipe{height:34px;margin:18px 2px 1px;border:1px solid #70eaff6b;border-radius:15px;background:linear-gradient(180deg,#a2f5ff0d,#00a7dd3b 45%,#1dd1f024 55%,#061b24a6);position:relative;overflow:hidden;box-shadow:inset 0 3px 4px #c5ffff13,inset 0 -2px 5px #022735}
.coolingPipe:after{content:"";position:absolute;inset:0;background:repeating-linear-gradient(105deg,transparent 0 38px,#cefbff99 40px 44px,transparent 47px 90px);opacity:0;animation:coolingPipeFlow 4s linear infinite;animation-play-state:paused}
.hasCoolingFlow .coolingPipe:after{opacity:.45;animation-play-state:running}
@keyframes coolingPipeFlow{to{background-position:180px 0}}
html:root.busaDesign section[data-section="2"].compactContent .liquidPanel{padding:9px;margin:5px 0}
html:root.busaDesign section[data-section="2"].compactContent .liquidPanelHeading{margin-bottom:7px}
html:root.busaDesign section[data-section="2"].compactContent .liquidPanelHeading h2{font-size:16px}
html:root.busaDesign section[data-section="2"].compactContent .liquidRateLabel{font-size:11px;margin-bottom:5px}
html:root.busaDesign section[data-section="2"].compactContent .liquidRateGrid{gap:7px}
html:root.busaDesign section[data-section="2"].compactContent .liquidRateGrid>div{gap:3px}
html:root.busaDesign section[data-section="2"].compactContent .liquidRateGrid strong{font-size:26px}
html:root.busaDesign section[data-section="2"].compactContent .liquidRateGrid small{font-size:11px}
html:root.busaDesign section[data-section="2"].compactContent .liquidSummary{margin-top:8px;padding-top:7px;gap:6px}
html:root.busaDesign section[data-section="2"].compactContent .liquidSummary strong{font-size:19px}
html:root.busaDesign section[data-section="2"].compactContent .coolingPipe{height:21px;margin-top:9px}
html:root.busaDesign .chartCard{padding:13px}
html:root.busaDesign .chartTabs button,html:root.busaDesign .chartRanges button{border-radius:13px;background:linear-gradient(145deg,#23434b55,#061b2855)!important}
html:root.busaDesign .chartLegend{font-size:11px;gap:8px 12px}
html:root.busaDesign #chartCanvas{border:1px solid var(--line);border-radius:11px;background:#06131d}
html:root.busaDesign #busaCalculator .calcPanel{padding:17px}
html:root.busaDesign #busaCalculator .calcPanel h3{font-size:18px}
html:root.busaDesign #busaCalculator .calcTabs button{background:var(--glass)!important;border-color:var(--border);border-radius:14px}
html:root.busaDesign #busaCalculator input{background:#05121b66!important;border-color:#6da9be!important;box-shadow:inset 0 1px 3px #0002!important}
html:root.busaDesign[data-theme="light"] #busaCalculator input{background:#fff!important}
html:root.busaDesign #busaCalculator .calcActions button{border-radius:14px}
/* Home: the artwork contains no words. Real readouts stay in front of animation. */
html:root.busaDesign section.holoHome{display:flex;flex-direction:column;font-family:Arial,sans-serif}
html:root.busaDesign section.holoHome .approvedDashboard{order:1;min-height:0;overflow:visible}
html:root.busaDesign section.holoHome .relayIndicators{order:3;flex:0 0 auto;gap:5px;margin:6px 0 8px!important}
html:root.busaDesign section.holoHome .relayBadge{background:var(--glass);border:1px solid #6999bc70;border-radius:15px;padding:6px 5px;min-height:40px;box-shadow:inset 0 1px #ffffff12;font-size:clamp(8px,2.35vw,11px)}
html:root.busaDesign section.holoHome .relayBadge[data-state="on"]{background:linear-gradient(135deg,#095a2f,#073824);border-color:#35d68c;box-shadow:inset 0 1px #9affaf33}
html:root.busaDesign section.holoHome #relayBadge3[data-state="on"]{background:linear-gradient(135deg,#652635,#391823);border-color:var(--alarm)}
html:root.busaDesign section.holoHome .relayBadge small{font-size:9px;line-height:1.15}
html:root.busaDesign section.holoHome .approvedTemp small,html:root.busaDesign section.holoHome .cubeSetBelow{display:none!important}
html:root.busaDesign section.holoHome .approvedTempTitle{font-size:12px!important;font-weight:650}
html:root.busaDesign section.holoHome .approvedTemp strong{font-family:Arial,sans-serif!important;font-weight:750;text-shadow:0 1px 2px #0003;white-space:nowrap}
html:root.busaDesign section.holoHome .temp1,html:root.busaDesign section.holoHome .temp3{top:8%;width:var(--temp-label-width)!important}
html:root.busaDesign section.holoHome .temp1{left:calc(3px - var(--scene-left))}
html:root.busaDesign section.holoHome .temp3{right:calc(var(--scene-left) + var(--scene-w) - var(--panel-w) + 3px)}
html:root.busaDesign section.holoHome .temp1 strong,html:root.busaDesign section.holoHome .temp3 strong{font-size:var(--home-temp-size)!important;letter-spacing:-.4px;line-height:1.15}
html:root.busaDesign section.holoHome .temp0{left:calc(3px - var(--scene-left));top:55%;width:var(--temp-label-width)!important;height:auto}
html:root.busaDesign section.holoHome .temp0 strong{font-size:var(--home-temp-size)!important;margin-top:3px}
html:root.busaDesign section.holoHome .temp2{left:29%;top:79%;width:27%;padding:0;text-align:center}
html:root.busaDesign section.holoHome .temp2 .approvedTempTitle{font-size:clamp(8px,calc(var(--scene-w)*.033),12px)!important}
html:root.busaDesign section.holoHome .temp2 strong{font-size:clamp(11px,calc(var(--scene-w)*.068),22px)!important;margin-top:3px;letter-spacing:-.6px}
html:root.busaDesign section.holoHome .collectedOnGlass{left:67%;top:85.5%;width:19.5%;color:#fff}
html:root.busaDesign section.holoHome .collectedOnGlass span{color:var(--cube)!important;font-size:clamp(8px,calc(var(--scene-w)*.032),11px);font-weight:650}
html:root.busaDesign section.holoHome .collectedOnGlass b{font-family:Arial,sans-serif;font-size:clamp(11px,calc(var(--scene-w)*.056),19px);color:#fff!important}
html:root.busaDesign section.holoHome .flowOnPipe,html:root.busaDesign section.holoHome .waterUsedOnPipe,html:root.busaDesign section.holoHome .collectionRateLabel{display:none!important}
.holoMetrics{order:2;display:grid;grid-template-columns:1fr 1fr 1.12fr;gap:0;flex:0 0 auto;min-height:49px;margin:5px 0 0;border:1px solid var(--glass-edge);border-radius:15px;background:var(--glass);box-shadow:var(--glass-shadow);padding:9px 6px}
.holoMetric{min-width:0;display:flex;flex-direction:column;justify-content:center;gap:3px;padding:0 7px;border-right:1px solid var(--line)}
.holoMetric:last-child{border-right:0}
.holoMetric>span{font-size:9px;color:var(--ink);white-space:nowrap}
.holoMetric b{font-family:Arial,sans-serif;font-size:clamp(13px,4.1vw,18px);line-height:1.2;white-space:nowrap;color:#7de9f4;font-variant-numeric:tabular-nums}
.holoMetric .holoRate{font-size:clamp(12px,3.8vw,17px)}
.holoMetric .holoRate>span{font:inherit}
.holoMetric .holoRate strong{font:inherit}
.holoMetric .holoRate small{font-size:.66em;font-weight:500}
html:root.busaDesign section.holoHome .boilerBoil{left:30.5%;top:87.8%;width:25.5%;height:6.3%;z-index:2}
html:root.busaDesign section.holoHome .boilerSurface{display:none!important}
html:root.busaDesign section.holoHome .collectionLiquid{left:68%;top:84%;width:16.5%;height:11%;z-index:2}
html:root.busaDesign section.holoHome .waterMotion{z-index:1}
html:root.busaDesign section.holoHome .waterTrace{stroke-width:3;stroke-dasharray:13 110;filter:none}
html:root.busaDesign section.holoHome .coilTrace{stroke:#d3faff;stroke-width:2;stroke-dasharray:9 95;filter:none}
html:root.busaDesign section.holoHome .productTrace{stroke-width:3;stroke-dasharray:9 65}
html:root.busaDesign section.holoHome .jacketTrace,html:root.busaDesign section.holoHome .rigSheen,html:root.busaDesign section.holoHome .rigCoolantGlow,html:root.busaDesign section.holoHome .rigDrop{display:none!important}
html:root.busaDesign section.holoHome .boilerBoil i{animation-play-state:paused;opacity:0}
html:root.busaDesign section.holoHome .hasTemperature .boilerBoil i{animation-play-state:running}
html:root.busaDesign section.holoHome .approvedTempGrid{z-index:4;pointer-events:none}
@media(max-width:350px){html:root.busaDesign .appTabs{gap:0;width:calc(100% - 8px)!important;padding:5px 2px!important}html:root.busaDesign .appTabs button{font-size:7px!important;min-height:52px}html:root.busaDesign section[data-section="1"] .approvedSensorHead{padding-left:7px;gap:7px}html:root.busaDesign section[data-section="1"] .approvedSensorText strong{font-size:14px}html:root.busaDesign section[data-section="1"] .approvedSensorText small{font-size:9px}.holoMetric{padding:0 4px}.holoMetric>span{font-size:8px}.visualCollectionGlass{right:8px}.holoMetrics{padding:7px 4px}}
@media(max-height:670px){html:root.busaDesign .approvedPageTitle{margin:9px 2px 10px}html:root.busaDesign .coolingPipe{margin-top:12px;height:26px}html:root.busaDesign .liquidPanel{padding:12px;margin-bottom:9px}html:root.busaDesign .collectionPanel .liquidSummary{gap:8px;margin-top:10px;padding-top:9px}.visualCollectionGlass{top:68px;height:calc(100% - 83px);min-height:90px}html:root.busaDesign section.holoHome .relayBadge{min-height:34px;padding:4px}.holoMetrics{min-height:42px;padding:6px}.holoMetric>span{font-size:8px}}
@media(max-height:500px){html:root.busaDesign .appHeader{padding:4px 12px}html:root.busaDesign .appTabs button{min-height:43px}html:root.busaDesign .appTabs .uiIcon{width:20px;height:20px}html:root.busaDesign .appTabs{margin-bottom:3px!important;padding-top:3px!important;padding-bottom:3px!important}.holoMetrics{min-height:34px;padding:4px;margin-top:2px}html:root.busaDesign section.holoHome .relayIndicators{margin:3px 0!important}html:root.busaDesign section.holoHome .relayBadge{min-height:29px;font-size:9px}html:root.busaDesign .coolingPipe{display:none}.visualCollectionGlass{display:none}html:root.busaDesign .collectionPanel .liquidRateGrid,html:root.busaDesign .collectionPanel .liquidSummary,html:root.busaDesign .collectionPanel .liquidRateLabel{width:100%}}
@media(min-width:600px) and (max-height:500px){html:root.busaDesign section.holoHome .temp2{left:auto;right:calc(var(--scene-left) + var(--scene-w) - var(--panel-w) + 4px);top:52%;width:135px}html:root.busaDesign section.holoHome .temp2 .approvedTempTitle{font-size:11px!important}html:root.busaDesign section.holoHome .temp2 strong{font-size:20px!important}.holoMetrics{grid-template-columns:1fr 1fr 1fr}.holoMetric{flex-direction:row;align-items:center;gap:12px}.holoMetric>span{font-size:10px}}
@media(prefers-reduced-motion:reduce){.visualCollectionGlass .glassBubble,.coolingPipe:after{animation:none!important}.coolingPipe:after{opacity:0!important}}
@media(min-width:600px) and (max-height:500px){html:root.busaDesign section.holoHome .collectedOnGlass{left:auto;right:calc(var(--scene-left) + var(--scene-w) - var(--panel-w) + 4px);top:78%;width:135px;text-align:right}html:root.busaDesign section.holoHome .collectedOnGlass span{font-size:11px}html:root.busaDesign section.holoHome .collectedOnGlass b{font-size:19px}}

/* MOTION1 coordinates refer to the 1024 x 1536 approved equipment asset. */
@media(max-width:599px),(min-height:501px){
html:root.busaDesign section.holoHome .temp2{left:28%;top:76.5%;width:23.5%}
html:root.busaDesign section.holoHome .collectedOnGlass{left:64.5%;top:83.3%;width:19.5%}
}
html:root.busaDesign section.holoHome .boilerBoil{left:25.1%;top:85.6%;width:29.3%;height:7.4%;clip-path:polygon(0 0,100% 0,100% 73%,90% 100%,10% 100%,0 73%)}
html:root.busaDesign section.holoHome .collectionLiquid{left:66.4%;top:82%;width:15.9%;height:11.5%}
html:root.busaDesign section.holoHome .waterTrace{stroke-width:3;stroke-dasharray:14 100;animation:homeFluidRun 3s linear infinite paused}
html:root.busaDesign section.holoHome .coilTrace{stroke-width:2.5;stroke-dasharray:12 90;animation:homeCoilRun 2.8s linear infinite paused}
html:root.busaDesign section.holoHome .productTrace{animation:homeProductRun 1.5s linear infinite paused}
html:root.busaDesign section.holoHome .hasCooling .waterMotion .waterTrace,html:root.busaDesign section.holoHome .hasCooling .waterMotion .coilTrace{opacity:.85;animation-play-state:running}
html:root.busaDesign section.holoHome .hasCollection .waterMotion .productTrace{opacity:.85;animation-play-state:running}
.holoHome .waterMotion .columnFilm,.holoHome .waterMotion .capillaryDrop,.holoHome .waterMotion .cubeConvection,.holoHome .waterMotion .cubeSurfaceMotion{fill:none;stroke:#c5fffa;stroke-linecap:round;opacity:0;animation-play-state:paused}
.holoHome .waterMotion .columnFilm{stroke-width:1.4;stroke-dasharray:6 27;animation:homeFilmRun 2.4s linear infinite paused}
.holoHome .waterMotion .capillaryDrop{stroke-width:3;stroke-dasharray:4 23;animation:homeDropRun 1.4s linear infinite paused}
.holoHome .waterMotion .cubeConvection{stroke-width:2;stroke-dasharray:24 180;animation:homeConvectionRun 3.2s linear infinite paused}
.holoHome .waterMotion .cubeSurfaceMotion{stroke-width:1.6;animation:homeSurfaceRun 2s ease-in-out infinite alternate paused;transform-box:fill-box;transform-origin:center}
.holoHome .hasCollection .columnFilm,.holoHome .hasCollection .capillaryDrop{opacity:.7;animation-play-state:running}
.holoHome .hasTemperature .cubeConvection{opacity:.42;animation-play-state:running}
.holoHome .hasTemperature .cubeSurfaceMotion{opacity:.7;animation-play-state:running}
@keyframes homeFluidRun{to{stroke-dashoffset:-228}}
@keyframes homeCoilRun{to{stroke-dashoffset:-204}}
@keyframes homeProductRun{to{stroke-dashoffset:-148}}
@keyframes homeFilmRun{to{stroke-dashoffset:-66}}
@keyframes homeDropRun{to{stroke-dashoffset:-54}}
@keyframes homeConvectionRun{to{stroke-dashoffset:-204}}
@keyframes homeSurfaceRun{to{transform:scaleY(.65) translateY(2px)}}
html:root.busaDesign section.holoHome[hidden] .waterMotion path,html:root.busaDesign section.holoHome[hidden] .boilerBoil i,html:root.busaDesign section.holoHome[hidden] .collectionLiquid *{animation-play-state:paused!important}
html:root.busaDesign section.holoHome .boilerBoil,html:root.busaDesign section.holoHome .collectionLiquid{opacity:0}
html:root.busaDesign section.holoHome .hasTemperature .boilerBoil,html:root.busaDesign section.holoHome .hasCollection .collectionLiquid{opacity:1}
html:root.busaDesign section.holoHome .boilerBoil>*,html:root.busaDesign section.holoHome .collectionLiquid>*{animation-play-state:paused}
html:root.busaDesign section.holoHome .hasTemperature .boilerBoil>*,html:root.busaDesign section.holoHome .hasCollection .collectionLiquid>*{animation-play-state:running}
html:root.busaDesign section.holoHome .homeMotionPaused .waterMotion path,html:root.busaDesign section.holoHome .homeMotionPaused .boilerBoil>*,html:root.busaDesign section.holoHome .homeMotionPaused .collectionLiquid>*{animation-play-state:paused!important}
@media(prefers-reduced-motion:reduce){html:root.busaDesign section.holoHome .waterMotion path{animation:none!important}}

/* DESIGN-003 approved 27.11.2026: HOME ONLY. The existing rig image is unchanged. */
html:root.busaDesign section.holoHome.busaApprovedHome003 .relayIndicators{display:none!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedDashboard{position:relative;padding-bottom:52px!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTempTitle{font-size:14px!important;line-height:1.15!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp strong{font-size:clamp(18px,4.7vw,24px)!important;line-height:1.15!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp{outline:1px solid transparent;outline-offset:2px;border-radius:5px}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.busaAlert{outline-color:#ffc76d}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp3.busaAlert{outline-color:#ff6473}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp2{left:auto!important;right:calc(var(--scene-left) + var(--scene-w) - var(--panel-w) + 3px)!important;top:58%!important;width:var(--temp-label-width)!important;text-align:right!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp2 .approvedTempTitle{font-size:14px!important;color:var(--ink)!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp0{top:58%!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .collectedOnGlass{display:none!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .busaApprovedStatus003{position:absolute;bottom:7px;left:0;right:0;display:flex;justify-content:space-between;align-items:flex-end;gap:10px;pointer-events:none;color:#8cf5a9;font-size:14px;font-weight:700;line-height:1.25;z-index:6}
html:root.busaDesign section.holoHome.busaApprovedHome003 .busaApprovedStatus003 .busaStatusValve{text-align:right;margin-left:auto}
html:root.busaDesign section.holoHome.busaApprovedHome003 .busaApprovedStatus003 .busaStatusOff{color:#ff6976}
html:root.busaDesign section.holoHome.busaApprovedHome003 .busaApprovedStatus003 .busaStatusUnknown{color:var(--muted)}
html:root.busaDesign section.holoHome.busaApprovedHome003 .busaApprovedAlarm003{position:absolute;right:0;bottom:32px;border:1px solid #ff6473;border-radius:4px;color:#ff6473;font:700 14px/1.2 Arial,sans-serif;padding:3px 16px;pointer-events:none;z-index:6}
html:root.busaDesign section.holoHome.busaApprovedHome003 .busaApprovedAlarm003[hidden]{display:none!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetrics{display:grid!important;grid-template-columns:repeat(4,minmax(0,1fr))!important;min-height:40px;gap:0;margin-top:5px}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric{display:flex;flex-direction:row;align-items:center;justify-content:center;gap:4px;padding:3px 4px;min-width:0;white-space:nowrap}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>span{font-size:clamp(9px,2.3vw,12px);line-height:1.1;color:#d0eaf6}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>b,html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>strong{font:700 clamp(9px,2.3vw,12px)/1.1 Arial,sans-serif;color:#7de9f4;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric small{font-size:inherit}
/* DESIGN-003 mobile layout repair: keep the original apparatus image and navigation. */
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTempGrid{z-index:7!important;pointer-events:none}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp0{width:35%!important;z-index:7!important}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp2{
  width:35%!important;
  top:67%!important;
  left:auto!important;
  z-index:7!important;
  text-align:right!important;
}
html:root.busaDesign section.holoHome.busaApprovedHome003 .approvedTemp.temp2 .approvedTempTitle{
  justify-content:flex-end!important;
  text-align:right!important;
}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetrics{
  display:grid!important;
  grid-template-columns:minmax(0,1.12fr) minmax(0,.92fr) minmax(0,1.12fr) minmax(0,1fr)!important;
  padding:5px 3px!important;
  margin-top:5px!important;
  gap:0!important;
  min-height:42px!important;
  align-items:center!important;
}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric{
  display:flex!important;
  flex-direction:row!important;
  justify-content:center!important;
  align-items:center!important;
  gap:2px!important;
  padding:2px 2px!important;
  min-width:0!important;
  white-space:nowrap!important;
  overflow:visible!important;
}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>span{
  flex:0 0 auto!important;
  font:500 12px/1.1 'Arial Narrow',Arial,sans-serif!important;
  color:#cfe9f6!important;
}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>b{
  flex:0 0 auto!important;
  min-width:0!important;
  overflow:visible!important;
  text-overflow:clip!important;
  font:700 12px/1.1 'Arial Narrow',Arial,sans-serif!important;
  white-space:nowrap!important;
  color:#7de9f4!important;
}
html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>b *{
  display:inline!important;
  font:inherit!important;
  white-space:nowrap!important;
}
@media(max-width:365px){
  html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>span,
  html:root.busaDesign section.holoHome.busaApprovedHome003 .holoMetric>b{font-size:9.5px!important}
}

/* BUSA_DESIGN_CSS_END */
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
<button data-page="2"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 2C8 8 5 11 5 15a7 7 0 0 0 14 0c0-4-3-7-7-13z"/></svg><span>Вода и<br>отбор</span></button>
<button data-page="3"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 3v18h18"/><path d="m5 17 5-6 4 3 6-9"/></svg><span>Графики</span></button>
<button data-page="7"><svg class="uiIcon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7"><rect x="5" y="2" width="14" height="20" rx="2"/><path d="M8 6h8M8 10h2m4 0h2M8 14h2m4 0h2M8 18h2m4 0h2"/></svg><span>Калькулятор</span></button>
<button data-page="5"><svg class="uiIcon" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m9 3 .6-1h4.8l.6 3 2 .9 2.6-1 2.4 4.2-2 2.1v1.6l2 2.1-2.4 4.2-2.6-1-2 .9-.6 3H9.6L9 19l-2-.9-2.6 1L2 14.9l2-2.1v-1.6L2 9.1l2.4-4.2 2.6 1L9 5z"/><circle cx="12" cy="12" r="3"/></svg><span>Настройки</span></button></nav></header><main><section class="page" data-section="0">

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
  <div class="row">Расход воды <b id="flow">—</b></div>
  <p class="hint" id="waterCalibrationHint">Ожидание показаний расходомера</p>
  
  
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
<article class="liquidPanel collectionPanel"><div class="liquidPanelHeading"><h2>Отбор жидкости</h2><span id="collectionAvailability">Нет данных</span></div>
<p class="liquidRateLabel">Текущий отбор</p><div class="liquidRateGrid"><div><strong id="collectionMlMin">—</strong><small>мл/мин</small></div><div><strong id="collectionLph">—</strong><small>л/ч</small></div></div>
<div class="liquidSummary"><div><span>Собрано</span><strong id="collectionTotal">—</strong></div><div><span>Время отбора</span><strong id="collectionTime">—</strong></div></div><button id="collectionResetTotal" type="button" class="full">Сбросить собрано</button><p id="collectionResetResult" class="hint"></p></article>
<article class="liquidPanel coolingPanel"><div class="liquidPanelHeading"><h2>Охлаждение</h2><span id="coolingAvailability">Нет данных</span></div>
<div class="liquidRateGrid"><div><small>Расход воды</small><strong id="coolingLph">—</strong><small>л/ч</small></div><div><small>Израсходовано</small><strong id="coolingTotal">—</strong><small>л</small></div></div><button id="waterResetTotal" type="button" class="full">Сбросить израсходовано</button><p id="waterResetResult" class="hint"></p></article>
</section>
<section class="page" data-section="3" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 3v18h18"/><path d="m5 17 5-6 4 3 6-9"/></svg><strong>ГРАФИКИ</strong></div>
<div class="chartCard"><div class="chartTabs" role="tablist" aria-label="Вид графика">
<button type="button" data-chart="0" class="on" aria-selected="true">Температура</button><button type="button" data-chart="1" aria-selected="false">Отбор</button><button type="button" data-chart="2" aria-selected="false">Охлаждение</button></div>
<div class="chartHeading"><strong id="chartTitle">Температура, °C</strong><span id="chartState">Ожидание данных</span></div>
<div id="chartLegend" class="chartLegend"><span style="--series:#38bdf8">Вода<b data-legend-temp="0">Н/Д</b></span><span style="--series:#f4c84a">Куб<b data-legend-temp="2">Н/Д</b></span><span style="--series:#53d486">Колонна<b data-legend-temp="1">Н/Д</b></span><span style="--series:#ff6473">Авария<b data-legend-temp="3">Н/Д</b></span></div>
<canvas id="chartCanvas" width="640" height="440" role="img" aria-label="График реальных измерений"></canvas>
<div class="chartRanges" aria-label="Период"><button type="button" data-minutes="5" class="on" aria-pressed="true">5 мин</button><button type="button" data-minutes="10" aria-pressed="false">10 мин</button><button type="button" data-minutes="15" aria-pressed="false">15 мин</button><button type="button" data-minutes="30" aria-pressed="false">30 мин</button><button type="button" data-minutes="60" aria-pressed="false">1 ч</button></div><p class="chartCadence">Обновление 2 с</p>
</div></section>
<section class="page" data-section="5" hidden><div class="approvedPageTitle"><svg class="uiIcon" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m9 3 .6-1h4.8l.6 3 2 .9 2.6-1 2.4 4.2-2 2.1v1.6l2 2.1-2.4 4.2-2.6-1-2 .9-.6 3H9.6L9 19l-2-.9-2.6 1L2 14.9l2-2.1v-1.6L2 9.1l2.4-4.2 2.6 1L9 5z"/><circle cx="12" cy="12" r="3"/></svg><strong>НАСТРОЙКИ</strong></div>
<div class="card approvedSettingsMain" id="settingsMenu"><button type="button" data-stab-btn="0" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0z"/><path d="M12 10v8"/></svg></span><span>Температура</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="1" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M2 8c6-5 14-5 20 0M5 12c4-4 10-4 14 0M8 16c2-2 6-2 8 0"/><circle cx="12" cy="20" r="1" fill="currentColor"/></svg></span><span>Wi-Fi</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="2" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.2 2.2m9.8 9.8 2.2 2.2M4.9 19.1l2.2-2.2m9.8-9.8 2.2-2.2"/></svg></span><span>Экран</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="3" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M7 19h11a4 4 0 0 0 .5-8A6.5 6.5 0 0 0 6 11a4 4 0 0 0 1 8z"/></svg></span><span>Google</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="7" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="m7 7 10 10-5 4V3l5 4L7 17"/></svg></span><span>Bluetooth · RainMaker</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="5" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M12 3v12m-5-5 5 5 5-5M4 18v3h16v-3"/></svg></span><span>Обновление ПО</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="8" class="approvedMenuItem"><span class="approvedMenuIcon">▤</span><span>Журнал событий</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="6" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><path d="M3 18h4l3-8 4 6 3-4h4"/><circle cx="12" cy="12" r="10"/></svg></span><span>Диагностика</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="4" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" fill="none" aria-hidden="true"><circle cx="12" cy="12" r="10"/><path d="M12 11v6M12 7h.01"/></svg></span><span>О системе</span><span class="approvedArrow">›</span></button>
<button type="button" data-stab-btn="9" class="approvedMenuItem"><span class="approvedMenuIcon"><svg class="uiIcon" width="23" height="23" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.9"><path d="M12 2C8 8 5 11 5 15a7 7 0 0 0 14 0c0-4-3-7-7-13z"/></svg></span><span>Отбор и вода</span><span class="approvedArrow">›</span></button></div>
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
<div class="settingsTabs" role="tablist" aria-label="Плата для обновления"><button id="otaMainTab" type="button" class="on" role="tab" aria-selected="true">Основная плата</button><button id="otaScreenTab" type="button" role="tab" aria-selected="false">Обновление экрана</button></div>
<div id="otaMainPanel">
<div class="row">Установленная прошивка <b id="otaFirmware">—</b></div>
<p class="hint">Обновление только локально по Wi-Fi БУСА. Выбирайте APP_OTA.bin, настройки и раскладку клавиш не стирайте.</p>
<h3>Обновление прошивки</h3>
<p class="hint">Для OTA выбирай APP_OTA.bin, подключившись к Wi-Fi БУСА. Не стирай NVS — раскладка клавиатуры и настройки сохраняются.</p>
<input type="file" accept=".bin" id="firmwareFile">
<button type="button" class="full" id="manualUpdate">Выбрать и обновить</button>
<p id="updateResult" role="status">Ожидание подтверждения</p></div>
<div id="otaScreenPanel" hidden><h3>Обновление экрана GUITION N4R8</h3>
<div class="row">Прошивка экрана <b id="displayOtaVersion">—</b></div>
<p class="hint">Выбирайте только DISPLAY_OTA.bin. Передача выполняется через основную плату по UART.</p>
<input type="file" accept=".bin" id="displayFirmwareFile">
<button type="button" class="full" id="displayManualUpdate" disabled>Обновить экран</button>
<progress id="displayOtaProgress" max="100" value="0" style="width:100%"></progress>
<p id="displayUpdateResult" role="status">Проверка связи с экраном</p></div></div>
<div class="card settingCard" data-stab="9" hidden><h2>Отбор и вода</h2>
<div class="chartTabs" role="tablist" aria-label="Датчик"><button type="button" data-liquid-tab="collection" class="on" aria-selected="true">Отбор</button><button type="button" data-liquid-tab="water" aria-selected="false">Вода</button></div>
<div data-liquid-panel="collection"><h3>Коэффициент калибровки</h3><div class="wifiField"><label for="collectionPplInput">Импульсов на 1 мл</label><input id="collectionPplInput" type="number" min="0" step="any" inputmode="decimal"></div>
<button id="collectionPplSave" type="button" class="full">Сохранить коэффициент</button><p id="collectionPplResult" role="status" class="hint"></p></div>
<div data-liquid-panel="water" hidden><h3>Коэффициент калибровки</h3>
 <div class="wifiField"><label for="waterPplInput">Импульсов на 1 литр</label><input id="waterPplInput" type="number" min="1" max="100000" step="0.1"  inputmode="decimal"></div>
 <button id="waterPplSave" type="button" class="full">Сохранить коэффициент</button>
 <button id="waterPplPassport" type="button" class="full">Вернуть паспортные 5880</button>
 <p id="waterPplResult" role="status" class="hint"></p>
</div></div>
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
<div class="card" data-diag-panel="screen"><h2>Экран GUITION</h2>
<div class="row">Состояние <b id="displayState">—</b></div>
<div class="row">Контроллер <b id="tft">—</b></div>
<p class="hint">GUITION JC4827W543R_I · 480×272 · UART. Правильность изображения проверяется визуально.</p>
<div class="grid"><button id="screenSleep" type="button">Погасить экран</button><button id="screenWake" type="button">Разбудить экран</button></div>
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
<button id="keyCancel" type="button">Отмена</button></div>
<h2>ИК-пульт</h2><div class="row">Состояние <b id="irState">—</b></div>
<div class="row">Следующая кнопка <b id="irExpected">—</b></div>
<p id="irInfo" class="hint">—</p>
<button id="irStart" type="button" class="full">Обучить пульт — 17 кнопок</button>
<button id="irCancel" type="button" class="full">Отменить обучение пульта</button></div>
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
<p class="hint">Паспорт YF-S401: 5880 имп/л (98 Гц при 1 л/мин). Изменённый коэффициент не устраняет ошибки сигнала.</p>
</div>
</div>
</section>
<section class="page" data-section="7" hidden><div id="busaCalculator"></div></section></main>
<script>
const get=id=>document.getElementById(id),logs=[],tlog=[],flog=[],tlog4=[[],[],[],[]],waterPulseLog=[];
let busaGraphData={temperature:[[],[],[],[]],water_pulses:[]},busaGraphFetchedAt=0,busaGraphLoading=false;
let lastWaterPulse=null,themeInitialized=false,temperatureThresholdsInitialized=false,calInitialized=false;
let active=0,connected=false,pollBusy=false,statusFailures=0,settingsActive=-1,lastLocalNav=0,lastClockPush=0;
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
 const names={'/api/water/coefficient':'Коэффициент расходомера','/api/time/sync':'Синхронизация времени','/api/wifi':'Настройки Wi-Fi','/api/wifi/disconnect':'Отключение Wi-Fi','/api/temperature-alarms':'Пороги сигнализации','/api/temperature/calibration':'Поправки датчиков','/api/display/power':'Экран','/api/theme':'Тема экрана'};
 const label=names[url.split('?')[0]]||'Действие';
 try{const r=await nativeWebFetch(input,options);if(method==='POST')message(label+(r.ok?': выполнено':': ошибка'),r.ok?'system':'warn');return r;}
 catch(e){if(method==='POST')message(label+': нет связи','warn');throw e;}
};
document.addEventListener('click',e=>{
 const button=e.target.closest('button');if(!button||button.matches('.appTabs button,[data-stab-btn],[data-chart],[data-minutes],[data-liquid-tab],#settingsBack'))return;
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
 if(n===9&&previous!==9){waterPplInitialized=false;collectionPplInitialized=false;liquidTab('collection');}
 if(send&&n!==-1)message('Открыты настройки: '+({0:'Температура',1:'Wi-Fi',2:'Экран',3:'Google',4:'О системе',5:'Обновление ПО',6:'Диагностика',7:'Bluetooth',8:'Журнал событий',9:'Отбор и вода'}[n]||'Настройки'));
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
let waterPplInitialized=false,collectionPplInitialized=false;
function liquidTab(kind){
 document.querySelectorAll('[data-liquid-panel]').forEach(p=>p.hidden=p.dataset.liquidPanel!==kind);
 document.querySelectorAll('[data-liquid-tab]').forEach(b=>{const on=b.dataset.liquidTab===kind;b.classList.toggle('on',on);b.setAttribute('aria-selected',String(on));});
}
document.querySelectorAll('[data-liquid-tab]').forEach(b=>b.onclick=()=>liquidTab(b.dataset.liquidTab));
function measured(v){return typeof v==='number'&&Number.isFinite(v)&&v>=0;}
function formatVolumeMl(v){if(!measured(v))return '—';return v<1000?decimal1.format(v)+' мл':decimal1.format(v/1000)+' л';}
function formatNumeric(v){return typeof v==='number'&&Number.isFinite(v)?decimal1.format(v):'—';}
function formatRunTime(ms){if(!measured(ms))return '—';const minutes=Math.floor(ms/60000);return String(Math.floor(minutes/60)).padStart(2,'0')+':'+String(minutes%60).padStart(2,'0');}
function formatUptime(ms){if(!measured(ms))return '—';const minutes=Math.floor(ms/60000);return Math.floor(minutes/60)+'ч'+String(minutes%60).padStart(2,'0')+'мин';}
function setMeasured(id,v){get(id).textContent=measured(v)?decimal1.format(v):'—';}
function updateLiquidPanels(s){
 const c=s?.collection,collection=!!(c?.pulse_seen??c?.sensor_present);
 const mlMin=collection&&measured(c.ml_per_min)?c.ml_per_min:null,total=collection&&measured(c.total_ml)?c.total_ml:null;
 setMeasured('collectionMlMin',mlMin);setMeasured('collectionLph',measured(mlMin)?mlMin*.06:null);
 get('collectionTotal').textContent=formatVolumeMl(total);
 get('collectionTime').textContent=formatRunTime(collection?c.duration_ms??c.elapsed_ms:null);
 get('collectionAvailability').textContent=!s?'Нет связи':collection?(c.calibrated===false?'Нет калибровки':'Данные датчика'):'Нет данных';
 setMeasured('homeCollectionRate',mlMin);
 const volume=formatVolumeMl(total),split=volume.lastIndexOf(' ');
 get('homeCollectionTotal').textContent=split<0?volume:volume.slice(0,split);get('homeCollectionUnit').textContent=split<0?'мл':volume.slice(split+1);
 const w=s?.water,ready=w?.credible===true&&w?.seen_pulses===true&&measured(w?.total_l_estimate);
 get('waterCalibrationHint').textContent=!s?'Нет связи':w?.calibrated?'Калибровка по мерной ёмкости подтверждена':'Коэффициент не подтверждён мерной ёмкостью';
 const lph=measured(w?.lpm)?w.lpm*60:null;
 setMeasured('coolingLph',lph);setMeasured('coolingTotal',ready?w.total_l_estimate:null);
 get('homeCoolingRate').textContent=(measured(lph)?decimal1.format(lph):'—')+' л/ч';
 get('homeWaterUsed').textContent=(ready?decimal1.format(w.total_l_estimate):'—')+' л';
 get('coolingAvailability').textContent=!s?'Нет связи':w?.credible===false?'Недостоверные данные':ready?(w.volume_complete===false?'Частичный объём · были помехи':w.calibrated?'Данные датчика':'Расчётно · без калибровки'):'Нет данных';
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
  get('displayState').textContent=await resp.text();
 }catch(e){get('displayState').textContent='ОШИБКА: '+e.message}
}
get('screenSleep').onclick=()=>setScreenPower('sleep');
get('screenWake').onclick=()=>setScreenPower('wake');
get('keyStart').onclick=()=>{if(confirm('Обучить клавиатуру, начиная с F1?'))keyCommand('start')};
async function irCommand(action){
 try{const r=await fetch('/api/ir/'+action,{method:'POST',body:new URLSearchParams({confirm:'YES'})});
 get('irInfo').textContent=await r.text();}catch(e){get('irInfo').textContent='Ошибка: '+e.message;}
}
get('irStart').onclick=()=>{if(confirm('Обучить ИК-пульт?'))irCommand('start')};
get('irCancel').onclick=()=>irCommand('cancel');
get('keyNext').onclick=()=>keyCommand('next');
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
async function saveWaterPpl(value){
  const status=get('waterPplResult'),button=get('waterPplSave'),reset=get('waterPplPassport');
  const numeric=Number(value);
  if(!String(value).trim()||!Number.isFinite(numeric)||numeric<1||numeric>100000){
    status.textContent='Укажите от 1 до 100000 имп/л';return;
  }
  button.disabled=true;reset.disabled=true;
  try{
    const response=await fetch('/api/water/coefficient',{method:'POST',body:new URLSearchParams({confirm:'YES',pulses_per_litre:String(numeric)})});
    if(!response.ok)throw Error(await response.text());
    const data=await response.json();
    get('waterPplInput').value=Number(data.pulses_per_litre).toFixed(1);
    status.textContent='Сохранено: '+data.pulses_per_litre+' имп/л';
    waterPplInitialized=true;
  }catch(err){status.textContent='Ошибка: '+err.message;}
  finally{button.disabled=false;reset.disabled=false;}
}
get('waterPplSave').onclick=()=>saveWaterPpl(get('waterPplInput').value);
get('waterPplPassport').onclick=()=>saveWaterPpl('5880');
get('collectionPplSave').onclick=async()=>{
 const input=get('collectionPplInput'),status=get('collectionPplResult'),button=get('collectionPplSave');
 const raw=input.value.trim(),numeric=Number(raw);
 if(!raw||!Number.isFinite(numeric)||numeric<=0){status.textContent='Укажите положительный коэффициент';return;}
 button.disabled=true;
 try{
  const response=await fetch('/api/collection/calibration',{method:'POST',body:new URLSearchParams({confirm:'YES',pulses_per_ml:String(numeric)})});
  if(!response.ok)throw Error(await response.text());
  const data=await response.json();if(!Number.isFinite(data.pulses_per_ml)||data.pulses_per_ml<=0)throw Error('Некорректный ответ');
  input.value=String(data.pulses_per_ml);collectionPplInitialized=true;
  status.textContent='Сохранено';
 }catch(err){status.textContent='Ошибка: '+err.message;}
 finally{button.disabled=false;}
};
async function resetVolume(target,button,status){
 if(!window.confirm(target==='water'?'Сбросить израсходованный объём воды?':'Сбросить собранный объём?'))return;
 button.disabled=true;
 try{
  const response=await fetch('/api/volume/reset',{method:'POST',body:new URLSearchParams({confirm:'YES',target}),cache:'no-store'});
  if(!response.ok)throw Error(await response.text());
  if(!(await response.json()).ok)throw Error('Некорректный ответ');
  status.textContent='Объём сброшен';
 }catch(err){status.textContent='Ошибка: '+err.message;}
 finally{button.disabled=false;}
}
get('waterResetTotal').onclick=()=>resetVolume('water',get('waterResetTotal'),get('waterResetResult'));
get('collectionResetTotal').onclick=()=>resetVolume('collection',get('collectionResetTotal'),get('collectionResetResult'));
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
 const note=chartView===1&&!data.collection_sensor?'Нет импульсов отбора':points?'':chartView===2?'Нет импульсов охлаждения':'Ожидание измерений';
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
function displayOtaOwnsStatusPoll(){const p=get('otaScreenPanel');return document.busaOtaUploading||document.busaOtaStatusPolling||(document.busaOtaTransfer&&p&&!p.hidden&&!p.closest('[data-stab]').hidden);}
async function update(){if(pollBusy||document.hidden||displayOtaOwnsStatusPoll())return;pollBusy=true;try{let r=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(6000)});if(!r.ok)throw Error(r.status);let s=await r.json();if(!connected)message('Связь с ESP32 восстановлена');connected=true;statusFailures=0;
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
window.busaApprovedHomeUpdate?.(s);
if(!waterPplInitialized && Number.isFinite(s.water?.pulses_per_litre)){
 get('waterPplInput').value=Number(s.water.pulses_per_litre).toFixed(1);
 waterPplInitialized=true;
}
if(!collectionPplInitialized&&s.collection){
 get('collectionPplInput').value=Number.isFinite(s.collection.pulses_per_ml)?String(s.collection.pulses_per_ml):'';
 collectionPplInitialized=true;
}
let f=s.water.lpm===null?null:Number(s.water.lpm);get('flow').textContent=f===null?(s.water.sample_ready?'НЕДОСТОВЕРНО':'НЕТ ДАННЫХ'):formatNumeric(f);get('pulses').textContent=s.water.pulses;updateLiquidPanels(s);
flog.push(f);if(flog.length>80)flog.shift();
const currentPulse=Number(s.water.pulses);
if(Number.isFinite(currentPulse)){
 let delta=lastWaterPulse===null?null:Math.max(0,currentPulse-lastWaterPulse);
 lastWaterPulse=currentPulse;waterPulseLog.push(delta);
 if(waterPulseLog.length>80)waterPulseLog.shift();
}
// The browser's SSID picker is never automatically populated with old iPhone.
const cs=s.google;
get('displayState').textContent=s.display_diagnostics?.link==='CONNECTED'?(s.display_sleep?'Команда: погасить':'Связь есть'):'Нет подтверждённой связи';
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
const firmwareLabel=/-PERF4-HW2-NAME1(?:-TIME1)?$/.test(s.firmware)?'BUSA 1M AI 1v':s.firmware.replace(/^БУСА\s*1М\s*WiFi\s*/, '');
get('aboutFirmware').textContent=firmwareLabel;
get('otaFirmware').textContent=firmwareLabel;
get('aboutUptime').textContent=formatUptime(s.uptime_ms);
get('googleState').textContent=!cs.configured?'НЕ НАСТРОЕН':cs.state===5?'ОТВЕТ ПОЛУЧЕН':cs.state===4?'ПЕРЕДАНО, ПРОВЕРЬТЕ ТАБЛИЦУ':cs.state===3?'ОТПРАВКА':cs.state===2?'ВРЕМЯ':cs.state===1?'НЕТ ИНТЕРНЕТА':cs.state===6?'ОШИБКА':cs.state===7?'ОТКЛОНЕНО':'ОЖИДАНИЕ';
get('googleResult').textContent='HTTP '+cs.http+' · удачных подтверждений '+cs.sent+' · DNS '+(cs.dns??'?')+' · TLS '+(cs.tls_error??'?')+' · этап '+(cs.error_stage??'?');
get('updateResult').textContent=s.ota.message+(s.ota.active?' '+s.ota.bytes+' байт':'');
if(s.ir_remote){const q=s.ir_remote;
 get('irState').textContent=q.learning?('Обучение '+q.step+'/17'):(q.ready?'Сохранён':'Не обучен');
 get('irExpected').textContent=q.expected||'—';get('irInfo').textContent=q.message||'—';
}
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
     ' idle=0x'+d.pcf8575?.idle_pins_hex;
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
     '\nОтброшенные фронты: '+d.water_flow?.rejected_edges+
     '\nПоследние '+(s.water?.sample_window_ms??0)+' мс: '+(s.water?.pulses_window??0)+' имп / '+(s.water?.rejected_edges_window??0)+' отброшено'+
     '\nКоэффициент: '+(s.water?.pulses_per_litre??5880)+' имп/л ('+(s.water?.coefficient_source==='manual'?'ручной':'паспортный')+')'+
      '\nКалибровка по мерной ёмкости: '+(s.water?.calibrated?'подтверждена':'не подтверждена')+
      '\nУчтённый объём: '+formatNumeric(s.water?.total_l_estimate)+' л'+
      (s.water?.volume_complete===false?' (неполный, были помехи)':'')+
     '\nРасход (оценка): '+formatNumeric(s.water?.lpm)+' л/мин'+
     (s.water?.credible===false?'\nИЗМЕРЕНИЕ НЕДОСТОВЕРНО':'');
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
get('ver').textContent=s.firmware;get('uptime').textContent=formatUptime(s.uptime_ms);get('pcf').textContent=s.modules.pcf8575?'Найден':'Не найден';get('tft').textContent=s.display_diagnostics?.link==='CONNECTED'?(s.display_diagnostics.firmware||'GUITION'):'Нет актуальных данных';
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
document.addEventListener('visibilitychange',()=>{if(!document.hidden){update();if(active===3)refreshGraphHistory(true);}});
</script><script src="/calculator.js?v=CALC1"></script><script>
/* BUSA_VIEWPORT_JS_BEGIN */
/* One-screen navigation; long content scrolls within its own panel. */
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
 if(section.classList.contains('holoHome')){
  const h=Math.max(65,Math.min(H-5,w*1.48)),sw=h*2/3;
  const left=(w-sw)/2+Math.min(12,w*.03),top=Math.max(0,H-h);
  const label=Math.max(65,Math.min(w*.45,left+sw*.17-5));
  for(const [n,v] of Object.entries({'--scene-w':sw+'px','--scene-h':h+'px','--scene-left':left+'px','--scene-top':top+'px','--panel-w':w+'px','--temp-label-width':label+'px','--home-temp-size':Math.max(16,Math.min(24,label/3.6))+'px'}))set(panel,n,v);
  section.classList.toggle('smallScene',sw<220);return;
 }
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
 const count=wide?2:4,head=Math.max(48,Math.min(90,Math.floor((list.clientHeight-11*(count-1))/count)-2));
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
  document.documentElement.classList.add('viewportFit');document.documentElement.dataset.viewportMode='fit';
  size();if(scrollY!==0)scrollTo(0,0);
  if(id===7){document.documentElement.dataset.viewportMode='calculator';fitScroll($('#busaCalculator'));focusInPanel();return;}
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
</script><script>
/* BUSA_DESIGN_JS_BEGIN */
/* Read-only presentation bindings; controller API and calculator stay separate. */
(()=>{
 'use strict';
 const pairs=[0,2,1,3].map(i=>[document.getElementById('t'+i),document.querySelector('[data-legend-temp="'+i+'"]')]);
 const byId=id=>document.getElementById(id);
 const home=document.querySelector('section[data-section="0"]');
 const stage=byId('rigArtStage');
 home.classList.add('holoHome');
 const metricBar=document.createElement('div');metricBar.className='holoMetrics';metricBar.setAttribute('aria-label','Расход воды и отбор');
 const addMetric=(label,id,rate=false)=>{
  const box=document.createElement('div');box.className='holoMetric';
  const title=document.createElement('span');title.textContent=label;
  const value=rate?document.createElement('b'):byId(id);
  if(rate){value.className='holoRate';value.append(byId(id));const unit=document.createElement('small');unit.textContent=' мл/мин';value.append(unit);}
  box.append(title,value);metricBar.append(box);
 };
 addMetric('Расход','homeCoolingRate');addMetric('Израсходовано','homeWaterUsed');addMetric('Отбор','homeCollectionRate',true);
 home.append(metricBar,home.querySelector('.relayIndicators'));
 const waterSection=document.querySelector('section[data-section="2"]');
 const glass=document.createElement('div');glass.className='visualCollectionGlass';glass.setAttribute('aria-hidden','true');
 glass.innerHTML='<svg viewBox="0 0 110 180" fill="none"><defs><linearGradient id="holoGlass" x1="4" y1="0" x2="109" y2="0" gradientUnits="userSpaceOnUse"><stop stop-color="#d7fbff" stop-opacity=".38"/><stop offset=".16" stop-color="#8bebff" stop-opacity=".04"/><stop offset=".8" stop-color="#8bebff" stop-opacity=".03"/><stop offset="1" stop-color="#d7fbff" stop-opacity=".4"/></linearGradient><linearGradient id="holoLiquid" x1="0" y1="70" x2="0" y2="176" gradientUnits="userSpaceOnUse"><stop stop-color="#00ddff" stop-opacity=".48"/><stop offset="1" stop-color="#0083bc" stop-opacity=".13"/></linearGradient><clipPath id="holoGlassClip"><path d="M8 72H102V158Q102 174 55 174Q8 174 8 158Z"/></clipPath></defs><path d="M5 13V158C5 180 105 180 105 158V13" fill="url(#holoGlass)" stroke="#b9f6ff" stroke-opacity=".7" stroke-width="2"/><g class="liquidFill"><path d="M8 75H102V157C102 177 8 177 8 157Z" fill="url(#holoLiquid)"/><ellipse cx="55" cy="75" rx="47" ry="8" fill="#06c9ed" fill-opacity=".3" stroke="#8cfaff" stroke-opacity=".8"/><g clip-path="url(#holoGlassClip)" stroke="#c4fcff" stroke-width="1.1"><circle class="glassBubble" cx="24" cy="152" r="3"/><circle class="glassBubble" cx="47" cy="151" r="2.1" style="animation-delay:-1.2s"/><circle class="glassBubble" cx="75" cy="154" r="3.6" style="animation-delay:-.7s"/><circle class="glassBubble" cx="87" cy="148" r="2.2" style="animation-delay:-2.1s"/></g></g><ellipse cx="55" cy="13" rx="50" ry="10" fill="#08212c" fill-opacity=".15" stroke="#d1fbff" stroke-opacity=".8" stroke-width="2"/><ellipse cx="55" cy="162" rx="47" ry="10" stroke="#82ecff" stroke-opacity=".6"/></svg>';
 waterSection.querySelector('.collectionPanel').append(glass);
 const pipe=document.createElement('div');pipe.className='coolingPipe';pipe.setAttribute('aria-hidden','true');waterSection.querySelector('.coolingPanel').append(pipe);
 const numberFrom=id=>{const m=byId(id)?.textContent.match(/[-+]?\d+(?:[.,]\d+)?/);return m?Number(m[0].replace(',','.')):null;};
 const sync=()=>{
  for(const [source,target] of pairs)if(source&&target&&target.textContent!==source.textContent)target.textContent=source.textContent;
  stage.classList.toggle('hasTemperature',numberFrom('t2')!==null);
  waterSection.classList.toggle('hasCollectionData',numberFrom('collectionTotal')!==null);
  waterSection.classList.toggle('hasCollectionFlow',(numberFrom('collectionMlMin')||0)>0);
  waterSection.classList.toggle('hasCoolingFlow',stage.classList.contains('hasCooling'));
  for(let i=0;i<4;i++){
   const badge=byId('relayBadge'+i),state=badge.dataset.state,node=byId('soundStatus'+i);
   const text=state==='on'?'Вкл.':state==='off'?'Выкл.':'Нет связи';if(node.textContent!==text)node.textContent=text;
  }
 };
 const observer=new MutationObserver(sync);
 for(const [source] of pairs)if(source)observer.observe(source,{childList:true,characterData:true,subtree:true});
 for(const id of ['collectionTotal','collectionMlMin'])observer.observe(byId(id),{childList:true,characterData:true,subtree:true});
 observer.observe(stage,{attributes:true,attributeFilter:['class']});
 for(let i=0;i<4;i++)observer.observe(byId('relayBadge'+i),{attributes:true,attributeFilter:['data-state']});
 sync();
})();

/* MOTION1: passive presentation, using the existing measured-data classes. */
(()=>{
 'use strict';
 const stage=document.getElementById('rigArtStage');
 const svg=stage.querySelector('.waterMotion');
 svg.setAttribute('viewBox','0 0 1024 1536');
 svg.innerHTML=`<defs>
  <clipPath id="homeCubeLiquidClip"><path d="M255 1316H558V1392L529 1427H284L255 1392Z"/></clipPath>
  <clipPath id="homeColumnClip"><path d="M353 682H455V784H353ZM354 888H453V944H354Z"/></clipPath>
  <clipPath id="homeSmallGlassClip"><rect x="687" y="640" width="48" height="68"/></clipPath>
 </defs>
 <path class="waterTrace drain" data-motion="water-left" d="M18 1144H76Q142 1144 142 1076V331Q142 188 283 188H517"/>
 <path class="waterTrace transfer" data-motion="water-link" d="M684 367H878Q951 367 951 442V689Q951 759 881 759H787"/>
 <path class="waterTrace inlet" data-motion="water-out" d="M788 913H1003"/>
 <path class="coilTrace" data-motion="coil" d="M695 782C753 783 754 793 695 801S638 814 695 821S754 833 695 841S638 853 695 861S754 873 695 881S638 894 695 902S754 914 695 925"/>
 <path class="productTrace" d="M718 995L748 1256"/>
 <path class="columnFilm" data-motion="column" clip-path="url(#homeColumnClip)" d="M359 682V784M449 682V784M359 888V944M448 888V944"/>
 <path class="capillaryDrop" data-motion="small-glass" clip-path="url(#homeSmallGlassClip)" d="M711 640V708"/>
 <path class="cubeConvection" data-motion="cube-convection" clip-path="url(#homeCubeLiquidClip)" d="M298 1423C265 1380 307 1350 356 1320M515 1322C555 1360 514 1404 465 1425M383 1424C423 1390 358 1355 409 1320"/>
 <path class="cubeSurfaceMotion" data-motion="cube-surface" d="M268 1310C330 1299 477 1299 546 1310C478 1321 331 1321 268 1310Z"/>`;
 let coil='M688 783';
 for(let i=0;i<7;i++){
  const y=783+i*22;
  coil+='C751 '+(y-3)+' 751 '+(y+9)+' 688 '+(y+15)+'C672 '+(y+17)+' 672 '+(y+20)+' 688 '+(y+22);
 }
 svg.querySelector('.coilTrace').setAttribute('d',coil);
 stage.querySelector('.approvedRigArt').alt='БУСА: утверждённый рисунок главного экрана';
 stage.dataset.homeMotion='1';
 const syncVisibility=()=>stage.classList.toggle('homeMotionPaused',document.hidden);
 document.addEventListener('visibilitychange',syncVisibility);
 syncVisibility();
})();

/* DESIGN-003: preserve the original apparatus and live telemetry IDs on Home. */
(()=>{
 'use strict';
 const home=document.querySelector('section[data-section="0"]');
 if(!home || !home.querySelector('#rigArtStage'))return;
 const dashboard=home.querySelector('.approvedDashboard');
 const metrics=home.querySelector('.holoMetrics');
 if(!dashboard || !metrics)return;
 const existingArt=home.querySelector('#rigArtStage .approvedRigArt');
 const originalArtSource=existingArt?.getAttribute('src');
 home.classList.add('busaApprovedHome003');
 const statuses=document.createElement('div');
 statuses.className='busaApprovedStatus003';
 const heater=document.createElement('span');heater.className='busaStatusHeater';heater.textContent='ТЭН —';
 const valve=document.createElement('span');valve.className='busaStatusValve';valve.textContent='Кран —';
 statuses.append(heater,valve);dashboard.append(statuses);
 const alarm=document.createElement('span');alarm.className='busaApprovedAlarm003';alarm.textContent='АВАРИЯ';alarm.hidden=true;dashboard.append(alarm);
 const values=[['Расход','homeCoolingRate'],['Объём','homeWaterUsed'],['Отбор','homeCollectionRate'],['Собрано','homeCollectionTotal']];
 const readings=values.map(([,id])=>document.getElementById(id));
 const volumeUnit=document.getElementById('homeCollectionUnit');
 const cells=[];
 for(const [index,[name,id]] of values.entries()){
   const cell=document.createElement('div');cell.className='holoMetric';
   const title=document.createElement('span');title.textContent=name;
   const reading=document.createElement('b');
   if(readings[index])reading.append(readings[index]);else reading.textContent='—';
   if(id==='homeCollectionRate')reading.append(document.createTextNode(' мл/мин'));
   if(id==='homeCollectionTotal')reading.append(document.createTextNode(' '),volumeUnit||document.createTextNode('мл'));
   cell.append(title,reading);cells.push(cell);
 }
 metrics.replaceChildren(...cells);
 window.busaApprovedHomeUpdate=function(s){
   const sound=s?.sound_alerts;
   const cube=sound?.channels?.[0],top=sound?.channels?.[3];
   const cubeValid=!!(cube?.sample_valid&&Number.isFinite(cube.on_c)&&Number.isFinite(cube.off_c));
   const topValid=!!(top?.sample_valid&&Number.isFinite(top.on_c)&&Number.isFinite(top.off_c));
   const cubeOn=cubeValid&&cube.t3_condition===true;
   const topOn=topValid&&top.t4_condition===true;
   const known=cubeValid&&topValid;
   const normal=known&&cubeOn&&!topOn;
   const ordinaryOff=known&&!cubeOn&&!topOn&&Number(sound?.t3_off_events)>0;
   const marks=[sound?.buzzer?.sensors?.[0]?.condition_active===true,
                sound?.buzzer?.sensors?.[1]?.condition_active===true,ordinaryOff,topOn];
   for(let i=0;i<4;i++)home.querySelector('.approvedTemp.temp'+i)?.classList.toggle('busaAlert',marks[i]);
   heater.textContent=!known?'ТЭН —':normal?'ТЭН вкл.':'ТЭН откл.';
   valve.textContent=!known?'Кран —':normal?'Кран открыт':'Кран закрыт';
   for(const label of [heater,valve]){
     label.classList.toggle('busaStatusOff',known&&!normal);
     label.classList.toggle('busaStatusUnknown',!known);
   }
   alarm.hidden=!topOn;
   if(existingArt&&existingArt.getAttribute('src')!==originalArtSource)
     console.error('[DESIGN-003] Original local apparatus image unexpectedly changed');
 };
})();

/* BUSA_DESIGN_JS_END */
</script><script>
(()=>{
 const el=id=>document.getElementById(id),main=el('otaMainTab'),screen=el('otaScreenTab'),mainPanel=el('otaMainPanel'),screenPanel=el('otaScreenPanel');
 const button=el('displayManualUpdate'),message=el('displayUpdateResult'),progress=el('displayOtaProgress');
 let uploading=false,polling=false,uploadError='';
 const active=new Set(['receiving','ready','sending','waiting_boot']);
 const messages={idle:'Выберите OTA-файл экрана',receiving:'Загрузка файла на основную плату',ready:'Файл проверен. Ожидается экран',sending:'Передача по UART',waiting_boot:'Записано. Проверка запуска экрана',success:'Экран успешно обновлён',error:'Обновление не подтверждено. Проверьте связь и версию'};
 async function refresh(){
  if(uploading||polling||document.hidden||screenPanel.hidden||screenPanel.closest('[data-stab]').hidden||(typeof pollBusy!=='undefined'&&pollBusy))return;
  polling=true;document.busaOtaStatusPolling=true;
  try{
   const options={cache:'no-store'};
   if(typeof AbortSignal!=='undefined'&&typeof AbortSignal.timeout==='function')options.signal=AbortSignal.timeout(6000);
   const response=await fetch('/api/display-ota/status',options);
   if(!response.ok)throw Error('HTTP '+response.status);
   const data=await response.json();
   document.busaOtaTransfer=active.has(data.state);
   el('displayOtaVersion').textContent=data.firmware||'—';
   button.disabled=!data.capable||active.has(data.state);
   progress.value=data.size?Math.min(100,Math.floor(100*data.confirmed/data.size)):0;
   message.textContent=active.has(data.state)?messages[data.state]:uploadError||((data.state==='success'||data.state==='error')?messages[data.state]:data.capable?messages.idle:'Нет связи или экран не поддерживает OTA');
  }catch(e){button.disabled=true;message.textContent='Нет связи с основной платой. Результат не подтверждён';}
  finally{polling=false;document.busaOtaStatusPolling=false;}
 }
 function select(display){
  mainPanel.hidden=display;screenPanel.hidden=!display;
  main.classList.toggle('on',!display);screen.classList.toggle('on',display);
  main.setAttribute('aria-selected',String(!display));screen.setAttribute('aria-selected',String(display));
  if(display)refresh();
 }
 main.onclick=()=>select(false);screen.onclick=()=>select(true);
 button.onclick=async()=>{
  if(uploading)return;
  const file=el('displayFirmwareFile').files[0];
  if(!file||!file.name.endsWith('.bin')||file.size<60000||file.size>0x1c0000){message.textContent='Выберите подходящий OTA BIN экрана';return;}
  if(!confirm('Подтвердите обновление экрана GUITION N4R8. Не отключайте питание.'))return;
  uploading=true;document.busaOtaUploading=true;uploadError='';button.disabled=true;progress.value=0;
  try{
   const r=await fetch('/api/display-ota/prepare',{method:'POST',body:new URLSearchParams({confirm:'YES',size:String(file.size)})});
   if(!r.ok)throw Error(await r.text());
   const grant=await r.json(),body=new FormData();body.append('firmware',file,file.name);
   await new Promise((resolve,reject)=>{
    const xhr=new XMLHttpRequest();xhr.open('POST','/api/display-ota/upload',true);xhr.setRequestHeader('X-Busa-Token',grant.token);
    xhr.timeout=180000;xhr.upload.onprogress=e=>{if(e.lengthComputable)message.textContent='Wi-Fi: '+Math.round(e.loaded*100/e.total)+'%';};
    xhr.onload=()=>xhr.status===200?resolve():reject(Error(xhr.responseText||'HTTP '+xhr.status));
    xhr.onerror=()=>reject(Error('Связь прервана'));xhr.ontimeout=()=>reject(Error('Истекло время передачи'));xhr.send(body);
   });
   document.busaOtaTransfer=true;
   message.textContent='Файл принят. Ожидается подтверждение экрана';
  }catch(e){uploadError='Ошибка: '+e.message;message.textContent=uploadError;}
  finally{uploading=false;document.busaOtaUploading=false;refresh();}
 };
 setInterval(refresh,2000);
})();
</script></body></html>
)HTMLBUSA";

void handleCommand(String x){
  x.trim();x.toUpperCase();
  if(x=="STATUS") {Serial0.println(statusJSON());Serial.println(statusJSON());}
  else if(x.startsWith("PAGE ")){
    int n=x.substring(5).toInt();
    if(n>=1&&n<=6){navigatePage((uint8_t)(n-1));editingTemp=false;tftView=0;tftDirty=true;lastRender=0;Serial0.printf("PAGE=%d\n",n);}
  }
  else Serial0.println("Commands: STATUS | PAGE 1..6");
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


static void busaApi_waterCoefficient(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
     if(!apClient()||server.arg("confirm")!="YES"){
       server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;
     }
     if(!server.hasArg("pulses_per_litre")){
       server.send(400,"text/plain; charset=utf-8","Нет коэффициента");return;
     }
     String raw=server.arg("pulses_per_litre");raw.trim();
     char *end=nullptr;
     const float requested=strtof(raw.c_str(),&end);
     if(raw.isEmpty()||*end!='\0'||!isfinite(requested)||!BusaWater::validPulsesPerLitre(requested)){
       server.send(400,"text/plain; charset=utf-8","Коэффициент: 1–100000 имп/л");return;
     }
     if(!busaNvsReady||prefs.putFloat("waterPpl",requested)!=sizeof(float)){
       server.send(500,"text/plain; charset=utf-8","Ошибка сохранения NVS");return;
     }
     waterPulsesPerLitre=requested;
     waterCoefficientManual=fabsf(requested-BusaWater::kPassportPulsesPerLitre)>0.05f;
     flowReady=false;
     server.send(200,"application/json; charset=utf-8",
       String("{\"ok\":true,\"pulses_per_litre\":")+String(waterPulsesPerLitre,1)+"}");
}

static void busaApi_collectionCoefficient(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
  if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Только БУСА AP");return;}
  String value=server.arg("pulses_per_ml");value.replace(',','.');value.trim();char* end=nullptr;
  const float factor=strtof(value.c_str(),&end);
  if(value.isEmpty()||*end!='\0'||!BusaCollection::Meter::validCalibration(factor)){
    server.send(400,"text/plain; charset=utf-8","Неверный коэффициент");return;
  }
  if(!saveCollectionCalibration(factor)){server.send(500,"text/plain; charset=utf-8","Ошибка сохранения");return;}
  server.send(200,"application/json",String("{\"pulses_per_ml\":")+collectionNumber(factor)+"}");
}
static void busaApi_resetVolume(BusaLocalRequest& server){
  if(!server.local()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Confirmation required");return;}
  if(otaActive||busa_display_ota::busy()){server.send(409,"text/plain","Device busy");return;}
  const String target=server.arg("target");
  if(target=="water"){
    portENTER_CRITICAL(&flowMux);
    flowLastCount=pulseCounter;flowLastRejected=flowRejectedCounter;
    portEXIT_CRITICAL(&flowMux);
    flowCycle=millis();waterVolume=BusaWater::VolumeAccumulator();
  }else if(target=="collection"){
    collectionMeter.resetTotal(collectionPulses());
  }else{server.send(400,"text/plain","Invalid target");return;}
  busaWebRecord(target=="water"?"Сброс объёма воды":"Сброс собранного объёма");
  server.send(200,"application/json","{\"ok\":true}");
}
static void busaApi_temperatureAlarms(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_wifiScan(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_wifiDisconnect(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_wifiConnect(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    // Credentials may only be provisioned while connected to the protected local AP.
    if(!apClient()){
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
}

static void busaApi_theme(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Только БУСА AP");return;}
    if(!server.hasArg("mode")){server.send(400,"text/plain","Выбери тему");return;}
    int m=server.arg("mode").toInt();
    if(m<0||m>2||!saveThemeMode((uint8_t)m)){server.send(400,"text/plain","Тема не сохранена");return;}
    server.send(200,"text/plain; charset=utf-8",m==0?"Светлая тема":m==1?"Тёмная тема":"Системная тема");
}

static void busaApi_calibration(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_events(BusaLocalRequest& server){
   const uint32_t after=(uint32_t)strtoul(server.arg("after").c_str(),nullptr,10);
   String result="{\"boot_id\":"+String(busaWebBootId)+",\"uptime_ms\":"+String(millis())+",\"events\":[";bool comma=false;
   const uint16_t limit=server.uart()?min(uint16_t(35),busaWebJournal.size()):busaWebJournal.size();
   for(uint16_t i=0;i<limit;i++){
    const auto* e=busaWebJournal.latest(i);if(!e||e->seq<=after)continue;
    if(comma)result+=",";comma=true;
    result+="{\"id\":"+String(e->seq)+",\"uptime_ms\":"+String(e->uptime)+",\"epoch\":"+String(e->epoch)+",\"kind\":"+String(e->kind)+",\"text\":"+quoteJson(String(e->text))+"}";
   }
   result+="]}";server.send(200,"application/json; charset=utf-8",result);
}

static void busaApi_chartHistory(BusaLocalRequest& server){
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
    j+="],\"cooling_calibrated\":";j+=(waterCalibrationVerified()?"true":"false");
    j+=",\"collection_ml_min\":[";
    for(int i=0;i<bins;i++){if(i)j+=",";j+=collectionNumber(collectionMeter.convert(busaChartHistory.collectionValue(selected[i])));}
    j+="],\"collection_sensor\":"+String(collectionMeter.seen()?"true":"false")+",\"history_ready\":"+String(busaChartReady?"true":"false")+"}";
    server.send(200,"application/json; charset=utf-8",j);
}

static void busaApi_rainmakerEnable(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_rainmakerDisable(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_rainmakerStatus(BusaLocalRequest& server){
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
}

static void busaApi_keypadStart(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(irMap.learning()){server.send(409,"text/plain; charset=utf-8","Сначала завершите обучение пульта");return;}
    if(!pcfDetected){server.send(409,"text/plain","PCF8575 not found");return;}
    startKeyLearning();
    server.send(200,"text/plain; charset=utf-8","Обучение: нажми F1 и отпусти");
}

static void busaApi_keypadNext(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
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
}

static void busaApi_keypadUndo(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(!keypadLearning||learnStep==0){server.send(409,"text/plain","No previous step");return;}
    learnStep--;newPairs[learnStep]=255;
    pendingPair=-1;keyHeld=false;candidatePair=-1;candidateStep=255;
    keypadMessage="Повтори "+String(KEY_CAPS[learnStep]);
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
}

static void busaApi_keypadCancel(BusaLocalRequest& server){
  const auto apClient=[&server](){return server.local();};
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    keypadLearning=false;learnStep=0;candidatePair=-1;candidateStep=255;
    keypadMessage=keypadReady?"Сохранённая раскладка сохранена":"Обучение отменено";
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
}

static void busaApi_geometryRead(BusaLocalRequest& server){
    server.send(200,"application/json",String("{\"diameter\":")+quoteJson(busaCalcInputs[3][0])+",\"height\":"+quoteJson(busaCalcInputs[3][1])+"}");
}
static void busaApi_geometrySave(BusaLocalRequest& server){
    if(!server.local()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local only");return;}
    const String diameter=server.arg("diameter"),height=server.arg("height");
    double d=0,h=0;
    if(diameter.length()>12||height.length()>12||!busaCalcParse(diameter,d)||!busaCalcParse(height,h)||!BusaCalc::cylinder(d,h).ok){
        server.send(400,"text/plain","Invalid dimensions");return;
    }
    const String oldD=busaCalcInputs[3][0],oldH=busaCalcInputs[3][1];
    busaCalcInputs[3][0]=diameter;busaCalcInputs[3][1]=height;
    if(!busaCalcStoreGeometry()){
        busaCalcInputs[3][0]=oldD;busaCalcInputs[3][1]=oldH;
        server.send(500,"text/plain","Save failed");return;
    }
    server.send(200,"application/json","{\"ok\":true}");
}

static void busaApi_keyReference(BusaLocalRequest& server,bool calibration){
  if(!server.uart()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Physical display only");return;}
  const String si=server.arg("sensor"),field=server.arg("field"),op=server.arg("op");
  if(si.length()!=1||si[0]<'0'||si[0]>'3'||(!calibration&&(field.length()!=1||(field[0]!='0'&&field[0]!='1')))){
    server.send(400,"text/plain","Invalid field");return;}
  float v=NAN;
  if(op=="set"){
    String raw=server.arg("value");raw.replace(',','.');raw.trim();char* end=nullptr;v=strtof(raw.c_str(),&end);
    if(raw.isEmpty()||!end||*end||!isfinite(v)||(calibration?(v< -10||v>10||fabsf(v*10-roundf(v*10))>0.02f):(v<0||v>125))){
      server.send(400,"text/plain","Invalid value");return;}
  }else if(op=="clear"){if(calibration)v=0;}else{server.send(400,"text/plain","Invalid operation");return;}
  const int sensor=si[0]-'0';const bool ok=calibration?saveTempOffset(sensor,v):storeTempReference(sensor,field[0]-'0',v);
  server.send(ok?200:500,"text/plain; charset=utf-8",ok?"Сохранено":"Ошибка сохранения");
}
static void busaDispatchDisplayApi(const busa_uart::ApiRequest& request,int& code,String& body){
    BusaLocalRequest local(request);
    if(!busaNvsReady||otaActive){code=409;body="Device busy";return;}
    switch(request.route){
      case busa_uart::ApiRoute::waterCoefficient:busaApi_waterCoefficient(local);break;
      case busa_uart::ApiRoute::collectionCoefficient:busaApi_collectionCoefficient(local);break;
      case busa_uart::ApiRoute::resetVolume:busaApi_resetVolume(local);break;
      case busa_uart::ApiRoute::temperatureAlarms:busaApi_temperatureAlarms(local);break;
      case busa_uart::ApiRoute::wifiScan:busaApi_wifiScan(local);break;
      case busa_uart::ApiRoute::wifiDisconnect:busaApi_wifiDisconnect(local);break;
      case busa_uart::ApiRoute::wifiConnect:busaApi_wifiConnect(local);break;
      case busa_uart::ApiRoute::theme:busaApi_theme(local);break;
      case busa_uart::ApiRoute::calibration:busaApi_calibration(local);break;
      case busa_uart::ApiRoute::events:busaApi_events(local);break;
      case busa_uart::ApiRoute::chartHistory:busaApi_chartHistory(local);break;
      case busa_uart::ApiRoute::rainmakerEnable:busaApi_rainmakerEnable(local);break;
      case busa_uart::ApiRoute::rainmakerDisable:busaApi_rainmakerDisable(local);break;
      case busa_uart::ApiRoute::rainmakerStatus:busaApi_rainmakerStatus(local);break;
      case busa_uart::ApiRoute::googleConfig:busaApi_googleConfig(local);break;
      case busa_uart::ApiRoute::googleTest:busaApi_googleTest(local);break;
      case busa_uart::ApiRoute::googleOff:busaApi_googleOff(local);break;
      case busa_uart::ApiRoute::keypadStart:busaApi_keypadStart(local);break;
      case busa_uart::ApiRoute::keypadNext:busaApi_keypadNext(local);break;
      case busa_uart::ApiRoute::keypadUndo:busaApi_keypadUndo(local);break;
      case busa_uart::ApiRoute::keypadCancel:busaApi_keypadCancel(local);break;
      case busa_uart::ApiRoute::geometryRead:busaApi_geometryRead(local);break;
      case busa_uart::ApiRoute::geometrySave:busaApi_geometrySave(local);break;
      case busa_uart::ApiRoute::referenceEdit:busaApi_keyReference(local,false);break;
      case busa_uart::ApiRoute::calibrationEdit:busaApi_keyReference(local,true);break;
      default:code=404;body="Unknown request";return;
    }
    code=local.code();body=local.body();
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
  collectionMeter.begin(millis());
  pinMode(COLLECTION_GPIO,INPUT);
  attachInterrupt(digitalPinToInterrupt(COLLECTION_GPIO),collectionISR,FALLING);
  busaNvsReady=prefs.begin("busa-r04",false);
  loadCollectionCalibration();
   const float savedWaterPpl=busaNvsReady?prefs.getFloat("waterPpl",BusaWater::kPassportPulsesPerLitre):BusaWater::kPassportPulsesPerLitre;
   waterPulsesPerLitre=BusaWater::validPulsesPerLitre(savedWaterPpl)?savedWaterPpl:BusaWater::kPassportPulsesPerLitre;
   waterCoefficientManual=busaNvsReady && prefs.isKey("waterPpl") && BusaWater::validPulsesPerLitre(savedWaterPpl) && fabsf(savedWaterPpl-BusaWater::kPassportPulsesPerLitre)>0.05f;
   busaCalcLoadGeometry();
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
  // Retain the saved theme and all other NVS settings.
  uint8_t themeSaved=prefs.getUChar("themeMode",255);
  tftThemeMode=themeSaved<=2?themeSaved:(prefs.getBool("tftDarkColor",false)?1:2);
  tftThemeDark=tftThemeMode==1;
  for(int i=0;i<4;i++){
    String key="tCal"+String(i);float v=prefs.getFloat(key.c_str(),0);
    tempOffset[i]=isfinite(v)&&v>=-10&&v<=10?roundf(v*10)/10:0;
  }
  snprintf(busaRmService,sizeof(busaRmService),"PROV_BUSA_%04X",(unsigned)(ESP.getEfuseMac()&0xFFFF));
  startedAt=millis();
  busaKeyEvents.begin(esp_random());
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  wifiUiStaDisabled=prefs.getBool("staOff",false); // AP remains available even when STA is disabled
  loadKeymap();
  busaIrInit();
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
    server.send(200,"application/json",String("{\"datetime\":")+quoteJson(clockStamp(false))+"}");
  });
  server.on("/api/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",statusJSON());
  });
   // Local AP only. Manual coefficient does not override pulse credibility checks.
   server.on("/api/water/coefficient",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_waterCoefficient(request);
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
  server.on("/api/temperature-alarms",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_temperatureAlarms(request);
  });

  addGoogleRoutes();
  server.on("/api/display-ota/status",HTTP_GET,[]{
    if(!apClient()){server.send(403,"text/plain","Local AP only");return;}
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",busa_display_ota::status(busa_display_link::otaCapable(),busa_display_link::displayFirmware));
  });
  server.on("/api/display-ota/prepare",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Только локальная сеть БУСА");return;}
    if(otaActive||otaRestart||busa_display_ota::busy()||!busa_display_link::otaCapable()){
      server.send(409,"text/plain","Экран недоступен или выполняется обновление");return;
    }
    const String size=server.arg("size");char* stop=nullptr;
    const unsigned long bytes=strtoul(size.c_str(),&stop,10);
    if(size.isEmpty()||*stop||!busa_ota::validSize(bytes)){server.send(400,"text/plain","Неверный размер OTA");return;}
    char token[17];snprintf(token,sizeof(token),"%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random());
    displayOtaToken=token;displayOtaTokenAt=millis();displayOtaSize=bytes;
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"token\":")+quoteJson(displayOtaToken)+"}");
  });
  server.on("/api/display-ota/upload",HTTP_POST,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(displayUploadOk?200:400,"application/json",busa_display_ota::status(busa_display_link::otaCapable(),busa_display_link::displayFirmware));
    displayUploadOk=false;
  },uploadDisplayOTA);
  server.on("/api/ota/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",otaJSON());
  });
  server.on("/api/ota/prepare",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"||otaActive||otaRestart||busa_display_ota::busy()){
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
  server.on("/api/wifi/scan",HTTP_GET,[]{BusaLocalRequest request(server);busaApi_wifiScan(request);
  });
  // Phone-local Wi-Fi disconnect: never disable the BUSA access point.
  // The same saved 'staOff' flag is used by the existing physical disconnect.
  server.on("/api/wifi/disconnect",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_wifiDisconnect(request);
  });
  server.on("/api/wifi",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_wifiConnect(request);
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>7){server.send(400,"application/json","{\"error\":\"page 1..7\"}");return;}
    // Legacy browser callers must not alter TFT navigation.
    server.send(200,"application/json","{\"ok\":true,\"scope\":\"browser_only\"}");
  });
  server.on("/api/tft/profile",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local only");return;}
    server.send(410,"text/plain; charset=utf-8","Профиль GUITION 480×272 фиксирован");
  });
  server.on("/api/tft/screen",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;}
    if(!busa_display_link::keypadCapable()){server.send(409,"text/plain; charset=utf-8","Требуется совместимая версия GUITION");return;}
    String action=server.arg("action");
    if(action=="wake"){wakeTftDisplay();server.send(200,"text/plain; charset=utf-8","Команда: включить экран");}
    else if(action=="sleep"){darkenTftDisplay();server.send(200,"text/plain; charset=utf-8","Команда: погасить экран");}
    else server.send(400,"text/plain; charset=utf-8","Неверная команда");
  });
  server.on("/api/tft/test",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local only");return;}
    if(!busa_display_link::keypadCapable()){server.send(409,"text/plain; charset=utf-8","Требуется совместимая версия GUITION");return;}
    wakeTftDisplay();++busaDisplayTestSeq;
    server.send(200,"text/plain; charset=utf-8","Команда теста отправлена. Проверь изображение на GUITION.");
  });
  server.on("/api/tft/theme",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_theme(request);
  });
  server.on("/api/collection/calibration",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_collectionCoefficient(request);});
  server.on("/api/volume/reset",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_resetVolume(request);});
  server.on("/api/temperature/calibration",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_calibration(request);
  });
  server.on("/api/ir/start",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(!busaNvsReady||keypadLearning||otaActive||busa_display_ota::busy()){server.send(409,"text/plain; charset=utf-8","Устройство занято");return;}
    irPress.clear();IrReceiver.resume();irMap.start(millis());irMessage="Нажмите кнопку 1 и отпустите";
    server.send(200,"text/plain; charset=utf-8",irMessage);
  });
  server.on("/api/ir/cancel",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    irMap.cancel();irPress.clear();IrReceiver.resume();irMessage="Обучение отменено; прежняя раскладка сохранена";
    server.send(200,"text/plain; charset=utf-8",irMessage);
  });
  server.on("/api/keypad/start",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_keypadStart(request);
  });
  server.on("/api/keypad/next",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_keypadNext(request);
  });
  server.on("/api/keypad/undo",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_keypadUndo(request);
  });
  server.on("/api/keypad/cancel",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_keypadCancel(request);
  });
  // One shared history source for the physical TFT and local iPhone.
  // Collection and water pulse histories remain independent.
  server.on("/api/web-events",HTTP_GET,[]{BusaLocalRequest request(server);busaApi_events(request);
  });
  server.on("/api/chart-history",HTTP_GET,[]{BusaLocalRequest request(server);busaApi_chartHistory(request);
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
    j+="],\"water_interval_ms\":2000,\"water_calibrated\":";j+=(waterCalibrationVerified()?"true":"false");j+=",";
    j+="\"collection_sensor\":"+String(collectionMeter.seen()?"true":"false")+",\"time_ms\":"+String(now)+",";
    j+="\"temperature_samples\":"+String(v13HistoryCount)+"}";
    server.send(200,"application/json; charset=utf-8",j);
  });
  server.on("/api/graph-view",HTTP_GET,[]{
    int n=server.arg("view").toInt();
    if(n<0||n>2){server.send(400,"text/plain","Bad graph");return;}
    server.send(200,"application/json","{\"ok\":true,\"scope\":\"browser_only\"}");
  });
  server.on("/api/rainmaker/enable",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_rainmakerEnable(request);
  });
  server.on("/api/rainmaker/disable",HTTP_POST,[]{BusaLocalRequest request(server);busaApi_rainmakerDisable(request);
  });
  server.on("/api/rainmaker/status",HTTP_GET,[]{BusaLocalRequest request(server);busaApi_rainmakerStatus(request);
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
  static_assert(sizeof(WEB_PAGE)+2048<=BusaHttpServer::responseCapacity,"HTTP page exceeds response buffer");
  server.startWorker();
  Serial.printf("HTTP worker %s\n",server.workerEnabled()?"OK":"FALLBACK");
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
  busaReportStage("I2C_READY");
  busa_display_link::begin();
  launchGoogleCloud();
  busaSetupDoneMs=millis()-busaBootAt;
  busaReportStage("SETUP_DONE");
  Serial.printf("BUSA SETUP DONE ms=%lu K1-K4 sound-only\n",(unsigned long)busaSetupDoneMs);
  Serial0.println("BUSA CORE STABLE: GUITION + AP first; RainMaker explicit pairing; credentials hidden");
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
  busaLoopProfile.begin(tickStart);
  soundPoll(now);
  busaLoopProfile.mark(busa_perf::early,millis());
  if(!otaActive)busa_display_link::poll();
  busaLoopProfile.mark(busa_perf::uart,millis());
  {const uint32_t started=millis();server.pollApplication();busaObserve("HTTP",started,busaHttpWorstMs);}
  busaLoopProfile.mark(busa_perf::http,millis());
  // The application owns every API callback; network waits occur on the worker.
  now=millis();
  if(!otaActive)busa_display_link::poll();
  busaLoopProfile.mark(busa_perf::uart,millis());
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
  if(otaActive){busaIrPause();soundOutputsOff();delay(0);return;}
  if(busa_display_ota::busy())busaIrPause();else busaIrPoll(millis());
  if(tftRebootPending && elapsed(millis(),tftRebootAt,3000)){soundOutputsOff();ESP.restart();}
  if(googleRestart && elapsed(millis(),googleRestartAt,3500)){soundOutputsOff();ESP.restart();}
  busaRemoteAlarmApply(); // remote sound thresholds only; writes NVS in main loop
  busaLoopProfile.mark(busa_perf::services,millis());
  now=millis();
  {const uint32_t started=millis();sampleTemperatures(now);busaObserve("TEMP",started,busaTempWorstMs);}
  now=millis();
  {const uint32_t started=millis();probeOneWire(now);busaObserve("ONEWIRE",started,busaOneWireWorstMs);} // Diagnose on idle 1-Wire bus only
  // A missing sensor is a diagnostic error, not a reason to trap the UI on boot.
  // Wi-Fi AP and HTTP server initialize during the splash.
  // Missing sensors must not delay the user interface.
    sampleFlow(now);
  collectionMeter.sample(now,collectionPulses());
  busaLoopProfile.mark(busa_perf::peripherals,millis());
  busa_display_link::poll();
  busaLoopProfile.mark(busa_perf::uart,millis());
  busaGoogleSnapshotPoll(); // Main-loop-only capture; worker owns immutable JSON.
  busaLoopProfile.mark(busa_perf::snapshot,millis());
  busaRainMakerPoll(now);
  v13Collect(now);
  busaLoopProfile.mark(busa_perf::cloud,millis()); // Shared graph sampling, including when TFT screen is asleep.
  // No local physical display rendering.
  busaLoopProfile.mark(busa_perf::tft,millis());
  if(busa_display_ota::busy())busa_display_link::poll();
  busaLoopProfile.mark(busa_perf::uart,millis());
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
  busaLoopProfile.mark(busa_perf::services,millis());
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
  busaLoopProfile.mark(busa_perf::i2c,millis());
  if(elapsed(now,lastUart,5000)){
    lastUart=now;
    Serial.printf("BUSA ALIVE ms=%lu stage=%s AP=%d i2c=%d tft=%d worst_loop=%lu key_ms=%lu i2c_ms=%lu\n",
      (unsigned long)now,busaBootStage,(int)wifiRunning,(int)i2cOnline,
      (int)busa_display_link::lcdInitialized(),(unsigned long)busaMaxLoopMs,(unsigned long)busaLastKeyScanMs,
      (unsigned long)busaLastI2cStepMs);
    Serial0.printf("N16R8 HEARTBEAT ms=%lu wifi=%d i2c=%d tft=%d p=%lu flow=%.3f\n",
      (unsigned long)now,(int)wifiRunning,(int)i2cOnline,(int)busa_display_link::lcdInitialized(),
      (unsigned long)flowPulses(),(double)flowLpm);
  }
  busaLoopProfile.mark(busa_perf::logging,millis());
  const uint32_t loopMs=millis()-tickStart;
  busaLoopProfile.finish(loopMs);
  if(loopMs>busaMaxLoopMs)busaMaxLoopMs=loopMs;
  if(loopMs>100)busaSlowLoops++;
  delay(1); // Yield to Wi-Fi/RTOS each pass, with less idle latency than 2 ms.
}
