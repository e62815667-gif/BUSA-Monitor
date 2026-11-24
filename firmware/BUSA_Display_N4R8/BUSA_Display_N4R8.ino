#include "BUSA_GuitionHardware.h"
#include "BUSA_DisplaySession.h"
#include "BUSA_TouchUI.h"
static busa_guition::Hardware displayHardware;
static busa_guition::DisplaySession displaySession;
static busa_guition::Interface interface(displayHardware,displaySession);
static bool uiReady=false;
static uint32_t maxLoop=0,lastCapacity=0;
void setup(){
    Serial.begin(115200);
    displayHardware.begin();
    displaySession.begin();
    uiReady=interface.begin();
}
void loop(){
    const uint32_t started=millis();
    if(started-lastCapacity>=60000){lastCapacity=started;displayHardware.refreshCapacity();}
    uint32_t metrics[busa_uart::metricCount];
    for(auto& v:metrics)v=UINT32_MAX;
    displayHardware.metrics(metrics);metrics[busa_uart::wifiStatus]=0;
    displaySession.poll(metrics,"BUSA-DISPLAY-N4R8-OTA1-LCDFIX1-20261125",maxLoop,uiReady&&displayHardware.lcdReady()&&displayHardware.touchReady());
    if(uiReady)interface.poll();
    const uint32_t span=millis()-started;if(span>maxLoop)maxLoop=span;
    delay(2);
}
