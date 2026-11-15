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
const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-CALC4-WIFI1-UI16-20261115";
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
html:root.busaDesign section.holoHome{display:flex;flex-direction:column}
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
<p class="liquidRateLabel">Текущий отбор</p><div class="liquidRateGrid"><div><strong id="collectionMlMin">—</strong><small>мл/мин</small></div><div><strong id="collectionLph">—</strong><small>л/ч</small></div></div>
<div class="liquidSummary"><div><span>Собрано</span><strong id="collectionTotal">—</strong></div><div><span>Время отбора</span><strong id="collectionTime">—</strong></div></div></article>
<article class="liquidPanel coolingPanel"><div class="liquidPanelHeading"><h2>Охлаждение</h2><span id="coolingAvailability">Нет данных</span></div>
<div class="liquidRateGrid"><div><small>Расход воды</small><strong id="coolingLph">—</strong><small>л/ч</small></div><div><small>Израсходовано воды</small><strong id="coolingTotal">—</strong><small>л</small></div></div></article>
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
 const motion=stage.querySelector('.waterMotion');
 const path=(selector,d)=>{const node=motion.querySelector(selector);if(node)node.setAttribute('d',d);};
 path('.inlet','M805 851H780');
 path('.transfer','M788 671C760 426 577 281 375 289');
 path('.drain','M195 122V1337Q195 1383 156 1383H106');
 const coil=(left,right,bottom,turns,step)=>{
  let d='M'+left+' '+bottom;
  for(let i=0;i<turns;i++){const y=bottom-i*step;d+='C'+(right+20)+' '+(y-4)+' '+(right+20)+' '+(y-step*.65)+' '+left+' '+(y-step*.8)+'C'+(left-12)+' '+(y-step*.84)+' '+(left-12)+' '+(y-step*.95)+' '+left+' '+(y-step);}
  return d;
 };
 const coils=motion.querySelectorAll('.coilTrace');if(coils[0])coils[0].setAttribute('d',coil(668,750,844,7,23));if(coils[1])coils[1].setAttribute('d',coil(240,315,303,8,25));
 path('.productTrace','M716 930C716 1043 721 1205 725 1278');
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

/* BUSA_DESIGN_JS_END */
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
