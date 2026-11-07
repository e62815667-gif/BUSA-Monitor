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
const char VERSION[]="БУСА 1М WiFi 14v9-N16R8-RM-CALC4-WIFI1-UI7-20261106";
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
.rigArtStage{position:relative;isolation:isolate;width:100%;aspect-ratio:820 / 1919;overflow:hidden}
.approvedRigArt{display:block;width:100%;height:auto;object-fit:contain;background:transparent}
.rigSheen{position:absolute;inset:0;pointer-events:none;background:linear-gradient(110deg,transparent 38%,#d6efff55 49%,transparent 60%);background-size:260% 100%;clip-path:polygon(15% 20%,29% 20%,29% 65%,61% 69%,62% 94%,2% 94%,2% 73%,15% 66%);animation:rigShine 6s ease-in-out infinite;opacity:.65}
.rigCoolantGlow{position:absolute;left:83%;top:28%;width:12%;height:30%;opacity:0;background:linear-gradient(0deg,transparent,#38bdf87a,transparent);border-radius:12px}
.hasCooling .rigCoolantGlow{opacity:.7;animation:coolantGlow 2.4s ease-in-out infinite}
.rigDrop{position:absolute;left:85%;top:68%;width:5px;height:8px;background:#40ceff;border-radius:70% 70% 70% 5%;opacity:0;transform:rotate(-45deg)}
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
  <div class="approvedRig"><div class="rigArtStage" id="rigArtStage"><img class="approvedRigArt" src="data:image/webp;base64,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" alt="Аппарат по утверждённому рисунку"><span class="rigSheen" aria-hidden="true"></span><span class="rigCoolantGlow" aria-hidden="true"></span><span class="rigDrop dropOne" aria-hidden="true"></span><span class="rigDrop dropTwo" aria-hidden="true"></span><span class="rigDrop dropThree" aria-hidden="true"></span></div></div>
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
