#pragma once
#include <Arduino.h>
// Virtual only: 8 lamp positions, 11 color channels. No physical outputs.
// Positions T1 blue,T2 green,T3 yellow,T4 red,unit red/green,
// valve red/green (both unknown/off),Wi-Fi red/green, alarm red.
struct PanelStatus {
  bool lamp[11]={false};
  void update(const bool valid[4],bool wifi) {
    for(int i=0;i<4;i++)lamp[i]=valid[i];
    lamp[4]=false; lamp[5]=true; // controller commissioning: not safe to assume machine ready
    lamp[6]=false; lamp[7]=false; // valve unverified; no state feedback
    lamp[8]=wifi; lamp[9]=!wifi;
    lamp[10]=!valid[0]||!valid[1]||!valid[2]||!valid[3];
  }
  String json() const {
    String j="[";
    for(int i=0;i<11;i++){if(i)j+=",";j+=lamp[i]?"true":"false";}
    return j+"]";
  }
};
