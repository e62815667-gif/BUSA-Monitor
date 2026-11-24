// Internal compile-only check. This file is never a released firmware.
#include "BUSA_GuitionHardware.h"
#include "BUSA_DisplaySession.h"
busa_guition::Hardware board;
busa_guition::DisplaySession link;
void setup(){board.begin();link.begin();}
void loop(){
    uint32_t values[busa_uart::metricCount];
    for(auto& v:values)v=UINT32_MAX;
    board.metrics(values);
    link.poll(values,"BUSA-DISPLAY-COMPONENT",0);
    delay(1);
}
