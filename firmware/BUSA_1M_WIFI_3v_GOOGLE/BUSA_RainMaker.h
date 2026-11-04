#pragma once
// BUSA-1M ESP RainMaker bridge: read-only monitoring, no remote actuator control.
// Include from BUSA_Main.h just before setup(), after sensor and network state definitions.
#include <RMaker.h>
#include <WiFiProv.h>

static TemperatureSensor *busaRmTemps[4]={nullptr,nullptr,nullptr,nullptr};
static Device *busaRmHealth=nullptr,*busaRmWater=nullptr;
static bool busaRmStarted=false;
static uint32_t busaRmLastSent=0;
static const char * const busaRmSensorNames[4]={"T1 Temperature","T2 Temperature","T3 Temperature","T4 Temperature"};
static char busaRmService[24]={};
static char busaRmPoP[16]={};

static void busaRmProvisionEvent(arduino_event_t *event) {
  if(event->event_id==ARDUINO_EVENT_PROV_START) {
    Serial0.printf("BUSA RainMaker BLE: service=%s PoP=%s\n",busaRmService,busaRmPoP);
    printQR(busaRmService,busaRmPoP,"ble");
  }
}

static void busaRainMakerStart() {
  // Store a stable per-device proof-of-possession; do not put it in public firmware.
  char n[24];
  snprintf(n,sizeof(n),"BUSA_%04X",(unsigned)(ESP.getEfuseMac()&0xFFFF));
  strncpy(busaRmService,n,sizeof(busaRmService)-1);
  String pop=prefs.getString("rm-pop","");
  if(pop.length()!=8) {
    char generated[12];
    snprintf(generated,sizeof(generated),"%08lu",(unsigned long)(esp_random()%100000000UL));
    pop=generated;
    prefs.putString("rm-pop",pop);
  }
  strncpy(busaRmPoP,pop.c_str(),sizeof(busaRmPoP)-1);

  Node node=RMaker.initNode("BUSA-1M Monitor");
  if(!node.getNodeHandle()) {
    Serial0.println("BUSA RainMaker: initNode failed; local monitor remains running");
    return;
  }
  for(int i=0;i<4;i++) {
    busaRmTemps[i]=new TemperatureSensor(busaRmSensorNames[i],nullptr,0.0f);
    if(!busaRmTemps[i])continue;
    busaRmTemps[i]->addParam(Param("Present",nullptr,value(false),PROP_FLAG_READ));
    node.addDevice(*busaRmTemps[i]);
  }
  busaRmWater=new Device("Water Flow","esp.device.sensor");
  if(busaRmWater) {
    busaRmWater->addParam(Param("Rate L/min",nullptr,value(0.0f),PROP_FLAG_READ));
    busaRmWater->addParam(Param("Total L",nullptr,value(0.0f),PROP_FLAG_READ));
    busaRmWater->addParam(Param("Calibrated",nullptr,value(false),PROP_FLAG_READ));
    node.addDevice(*busaRmWater);
  }
  busaRmHealth=new Device("BUSA Health","esp.device.sensor");
  if(busaRmHealth) {
    busaRmHealth->addParam(Param("Uptime Seconds",nullptr,value(0),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Keypad Online",nullptr,value(false),PROP_FLAG_READ));
    node.addDevice(*busaRmHealth);
  }
  RMaker.enableOTA(OTA_USING_TOPICS);
  RMaker.enableTZService();
  esp_err_t ret=RMaker.start();
  if(ret!=ESP_OK) {
    Serial0.printf("BUSA RainMaker: start failed 0x%X; local monitor remains running\n",(unsigned)ret);
    return;
  }
  busaRmStarted=true;
  WiFi.onEvent(busaRmProvisionEvent);
  Serial0.printf("BUSA RainMaker: READY, BLE name %s, PoP %s (also shown in serial)\n",
                 busaRmService,busaRmPoP);
}
static void busaRainMakerProvision() {
  if(!busaRmStarted)return;
  // Official ESP RainMaker Arduino user-node association requires BLE provisioning.
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1,busaRmPoP,busaRmService);
}
static void busaRainMakerPoll(uint32_t now) {
  if(!busaRmStarted||WiFi.status()!=WL_CONNECTED||otaActive||
     busaGoogleNetworkBusy||busaWifiScanRunning||!elapsed(now,busaRmLastSent,30000))return;
  busaRmLastSent=now;
  for(int i=0;i<4;i++)if(busaRmTemps[i]) {
    const bool available=hasTemp(i,now);
    if(available)busaRmTemps[i]->updateAndReportParam(ESP_RMAKER_DEF_TEMPERATURE_NAME,temperature[i]);
    busaRmTemps[i]->updateAndReportParam("Present",available);
  }
  if(busaRmWater) {
    // Display only: this existing sensor has not been calibrated.
    busaRmWater->updateAndReportParam("Rate L/min",flowLpm);
    busaRmWater->updateAndReportParam("Total L",totalWaterLitres());
    busaRmWater->updateAndReportParam("Calibrated",false);
  }
  if(busaRmHealth) {
    busaRmHealth->updateAndReportParam("Uptime Seconds",(int)(now/1000u));
    busaRmHealth->updateAndReportParam("Keypad Online",pcfOnline);
  }
}
