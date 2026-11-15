#pragma once
#include <cstdint>
// Timers only; official provisioning callbacks own successful BLE shutdown.
// Never reboot on CRED_SUCCESS: the phone still needs the provisioning result.
struct BusaBleBootRecovery {
  bool advertised=false,stopping=false,finishing=false;
  bool noAdvertisement=false,sessionTimeout=false,stopTimeout=false;
  uint32_t stoppedAt=0,finishedAt=0;
  struct Action {bool stop=false,restart=false;};
  Action tick(uint32_t now,uint32_t startedAt,bool advertising,bool ended,bool initError){
    Action action;
    if(advertising)advertised=true;
    if(!finishing && (ended||initError)){finishing=true;finishedAt=now;}
    if(!finishing&&!stopping){
      noAdvertisement=!advertised&&uint32_t(now-startedAt)>=30000U;
      sessionTimeout=uint32_t(now-startedAt)>=180000U;
      if(noAdvertisement||sessionTimeout){stopping=true;stoppedAt=now;action.stop=true;}
    }
    if(stopping&&!finishing&&uint32_t(now-stoppedAt)>=15000U){
      stopTimeout=true;finishing=true;finishedAt=now;
    }
    action.restart=finishing&&uint32_t(now-finishedAt)>=4000U;
    return action;
  }
};
