#include "BUSA_GuitionHardware.h"
#include "BUSA_DisplaySession.h"
#include "BUSA_TouchUI.h"
static busa_guition::Hardware displayHardware;
static busa_guition::DisplaySession displaySession;
static busa_guition::Interface interface(displayHardware,displaySession);
static bool uiReady=false,uartReady=false;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static constexpr char displayVersion[]="BUSA-DISPLAY-N4R8-OTA1-BD2-I5-P1-UA-HW2";
#else
static constexpr char displayVersion[]="BUSA-DISPLAY-N4R8-OTA1-BD2-I4-P1-UA-HW2";
#endif
static_assert(sizeof(displayVersion)<=busa_uart::versionBytes,"Diagnostic version does not fit UART contract");
// The receiver, not the Arduino core, validates the new image after health checks.
extern "C" bool verifyRollbackLater(){return true;}
static uint32_t maxLoop=0,lastCapacity=0;
void setup(){
    Serial.begin(115200);
    Serial.printf("[BOOT] %s SDK=%s reset=%d\n",displayVersion,ESP.getSdkVersion(),int(esp_reset_reason()));
    displayHardware.beginDisplay();
    displayHardware.beginPeripherals();
    Serial.println("[BOOT] UART BEGIN");
    uartReady=displaySession.begin();
    Serial.printf("[BOOT] UART %s\n",uartReady?"OK":"FAIL");
    Serial.println("[BOOT] LVGL BEGIN");
    uiReady=interface.begin();
    Serial.printf("[BOOT] LVGL %s\n",uiReady?"OK":"FAIL");
}
void loop(){
    const uint32_t started=millis();
    static uint32_t lastBootReport=0;
    if(uint32_t(started-lastBootReport)>=5000){
        lastBootReport=started;
        Serial.printf("[BOOT] ALIVE lcd=%d touch=%d sd=%d uart=%d ui=%d psram=%u heap=%u\n",
                      int(displayHardware.lcdReady()),int(displayHardware.touchReady()),int(displayHardware.sdReady()),
                      int(uartReady),int(uiReady),unsigned(ESP.getFreePsram()),unsigned(ESP.getFreeHeap()));
    }
    if(started-lastCapacity>=60000){lastCapacity=started;displayHardware.refreshCapacity();}
    uint32_t metrics[busa_uart::metricCount];
    for(auto& v:metrics)v=UINT32_MAX;
    displayHardware.metrics(metrics);metrics[busa_uart::wifiStatus]=0;
    displaySession.poll(metrics,displayVersion,maxLoop,uiReady&&displayHardware.lcdReady()&&displayHardware.touchReady());
    if(uiReady)interface.poll();
    const uint32_t span=millis()-started;if(span>maxLoop)maxLoop=span;
    delay(2);
}
