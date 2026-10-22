#pragma once
#include <Arduino.h>
#include <Wire.h>

// BUSA-1M #001 PCA9685 LED driver, 3.3V logic.
// This file does NOT turn on any output by itself.
// Before enabling hardware: verify each LED's current limiting and output load.
// OE hardware pin must be in its enabled state for output to be visible.
namespace BusaLed {
constexpr uint8_t kAddress=0x40;
constexpr uint8_t kCount=11;
enum Channel : uint8_t {
  WaterBlue=0, ColumnGreen=1, CubeYellow=2, TopRed=3,
  SystemGreen=4, SystemRed=5, ValveGreen=6, ValveRed=7,
  WifiGreen=8, WifiRed=9, AlarmRed=10
};
inline bool write4(uint8_t reg,uint8_t a,uint8_t b,uint8_t c,uint8_t d) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);Wire.write(a);Wire.write(b);Wire.write(c);Wire.write(d);
  return Wire.endTransmission()==0;
}
inline bool readReg(uint8_t reg,uint8_t &out) {
  Wire.beginTransmission(kAddress);Wire.write(reg);
  if(Wire.endTransmission(false)!=0)return false;
  if(Wire.requestFrom(kAddress,(uint8_t)1)!=(uint8_t)1)return false;
  out=Wire.read();return true;
}
inline bool writeReg(uint8_t reg,uint8_t value) {
  Wire.beginTransmission(kAddress);Wire.write(reg);Wire.write(value);
  return Wire.endTransmission()==0;
}
// PCA9685 supports full ON/OFF bit 4 in corresponding LEDn_ON_H/OFF_H.
inline bool set(uint8_t channel,bool on) {
  if(channel>=kCount)return false;
  const uint8_t base=0x06+4*channel;
  return write4(base,0,on?0x10:0,0,on?0:0x10);
}
inline bool allOff() {
  // ALL_LED_ON_L = 0xFA, ALL_LED_OFF_H = 0xFD.
  return write4(0xFA,0,0,0,0x10);
}
inline bool beginSafe() {
  uint8_t mode1=0,mode2=0;
  if(!readReg(0x00,mode1)||!readReg(0x01,mode2))return false;
  // Force outputs off before changing sleep state.
  if(!allOff())return false;
  // Clear SLEEP (bit 4); preserve other MODE1 bits.
  if(!writeReg(0x00,mode1 & (uint8_t)~0x10))return false;
  return allOff();
}
} // namespace BusaLed
