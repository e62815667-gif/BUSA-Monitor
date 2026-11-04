#pragma once
// BUSA-1M ESP RainMaker bridge: T1-T4 measurement and full approved monitoring settings.
// All writes are queued and applied by the main Arduino loop, never the RainMaker task.
// No remote process outputs; K1-K4 remain low-voltage audible notification channels only.
#include <RMaker.h>
#include <WiFiProv.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

static const char * const busaRmTempNames[4]={"T1 Вода","T2 Колонна","T3 Куб","T4 Верх колонны"};
static const char * const RM_ON="ВКЛ °C";
static const char * const RM_OFF="ВЫКЛ °C";
static const char * const RM_CAL="Калибровка °C";
static const char * const RM_SET="Пороги заданы";
static const char * const RM_CLEAR="Удалить пороги";
static const char * const RM_BUZ="Зуммер";
static const char * const RM_K1="Звук К1";
static const char * const RM_K2="Звук К2";
static const char * const RM_K3="Звук К3";
static const char * const RM_K4="Звук К4";
static const char * const RM_SCREEN="Экран включен";
static const char * const RM_THEME="Тема (0/1/2)";
static const char * const RM_PAGE="Раздел (0-6)";
static const char * const RM_KEYLEARN="Обучить 20 клавиш";
static const char * const RM_KEYSTOP="Отмена обучения";

struct BusaRmRequest {
  uint8_t target; // 0..3=T1..T4, 4=screen, 5=keyboard
  uint8_t action; // 0=on,1=off,2=cal,3=clear,4=enable,5=theme,6=screen,7=page,8=learn,9=cancel
  uint8_t role;   // audible channel index for action 4, or 255 for buzzer
  float f;
  int i;
  bool b;
};
static QueueHandle_t busaRmQueue=nullptr;
static TemperatureSensor* busaRmTemps[4]={nullptr,nullptr,nullptr,nullptr};
static Device *busaRmHealth=nullptr,*busaRmWater=nullptr,*busaRmScreen=nullptr,*busaRmKeys=nullptr;
static bool busaRmStarted=false;
static uint32_t busaRmLastSent=0,busaRmLastSettingsSent=0;
static uint8_t busaRmSettingsCursor=0;
static char busaRmService[24]={};
static char busaRmPoP[16]={};

static void busaRmProvisionEvent(arduino_event_t *event){
  if(event->event_id==ARDUINO_EVENT_PROV_START){
    Serial0.printf("BUSA RainMaker pairing name=%s PoP=%s\n",busaRmService,busaRmPoP);
    printQR(busaRmService,busaRmPoP,"ble");
  }
}
static bool busaRmFindSensor(const char*name,int&out){
  for(int i=0;i<4;i++)if(strcmp(name,busaRmTempNames[i])==0){out=i;return true;}
  return false;
}
static void busaRmWrite(Device*device,Param*param,const param_val_t val,
                         void *priv_data,write_ctx_t*ctx){
  (void)priv_data;(void)ctx;
  if(!busaRmQueue||!device||!param)return;
  const char *d=device->getDeviceName(),*p=param->getParamName();
  if(!d||!p)return;
  BusaRmRequest q={};
  int sensor=-1;
  if(busaRmFindSensor(d,sensor)){
    q.target=sensor;
    if(strcmp(p,RM_ON)==0){q.action=0;q.f=val.val.f;}
    else if(strcmp(p,RM_OFF)==0){q.action=1;q.f=val.val.f;}
    else if(strcmp(p,RM_CAL)==0){q.action=2;q.f=val.val.f;}
    else if(strcmp(p,RM_CLEAR)==0){q.action=3;q.b=val.val.b;}
    else if(strcmp(p,RM_BUZ)==0 && sensor<2){q.action=4;q.role=255;q.b=val.val.b;}
    else if(sensor==2 && strcmp(p,RM_K1)==0){q.action=4;q.role=0;q.b=val.val.b;}
    else if(sensor==2 && strcmp(p,RM_K2)==0){q.action=4;q.role=1;q.b=val.val.b;}
    else if(sensor==2 && strcmp(p,RM_K3)==0){q.action=4;q.role=2;q.b=val.val.b;}
    else if(sensor==3 && strcmp(p,RM_K4)==0){q.action=4;q.role=3;q.b=val.val.b;}
    else return;
  }else if(strcmp(d,"Экран БУСА")==0){
    q.target=4;
    if(strcmp(p,RM_THEME)==0){q.action=5;q.i=val.val.i;}
    else if(strcmp(p,RM_SCREEN)==0){q.action=6;q.b=val.val.b;}
    else if(strcmp(p,RM_PAGE)==0){q.action=7;q.i=val.val.i;}
    else return;
  }else if(strcmp(d,"Клавиатура БУСА")==0){
    q.target=5;
    if(strcmp(p,RM_KEYLEARN)==0){q.action=8;q.b=val.val.b;}
    else if(strcmp(p,RM_KEYSTOP)==0){q.action=9;q.b=val.val.b;}
    else return;
  }else return;
  if(xQueueSend(busaRmQueue,&q,0)!=pdPASS)
    Serial0.println("BUSA RainMaker: command queue full; settings unchanged");
}
static void busaRmAddFloat(Device*d,const char*name,float initial,float lo,float hi,float step){
  if(!d)return;
  Param p(name,nullptr,value(initial),PROP_FLAG_READ|PROP_FLAG_WRITE);
  p.addBounds(value(lo),value(hi),value(step));
  d->addParam(p);
}
static void busaRmAddSwitch(Device*d,const char*name,bool initial){
  if(!d)return;
  Param p(name,nullptr,value(initial),PROP_FLAG_READ|PROP_FLAG_WRITE);
  p.addUIType(ESP_RMAKER_UI_TOGGLE);
  d->addParam(p);
}
static void busaRmAddInt(Device*d,const char*name,int initial,int lo,int hi){
  if(!d)return;
  Param p(name,nullptr,value(initial),PROP_FLAG_READ|PROP_FLAG_WRITE);
  p.addBounds(value(lo),value(hi),value(1));
  p.addUIType(ESP_RMAKER_UI_SLIDER);
  d->addParam(p);
}
static void busaRainMakerStart(){
  if(!busaRmQueue)busaRmQueue=xQueueCreate(20,sizeof(BusaRmRequest));
  if(!busaRmQueue){Serial0.println("BUSA RainMaker: no memory for settings queue");return;}
  snprintf(busaRmService,sizeof(busaRmService),"BUSA_%04X",(unsigned)(ESP.getEfuseMac()&0xFFFF));
  String pop=prefs.getString("rm-pop","");
  if(pop.length()!=8){
    char generated[12];
    snprintf(generated,sizeof(generated),"%08lu",(unsigned long)(esp_random()%100000000UL));
    pop=generated;
    prefs.putString("rm-pop",pop);
  }
  strncpy(busaRmPoP,pop.c_str(),sizeof(busaRmPoP)-1);
  Node node=RMaker.initNode("БУСА-1М Монитор");
  if(!node.getNodeHandle()){Serial0.println("BUSA RainMaker: initNode failed");return;}
  for(int i=0;i<4;i++){
    busaRmTemps[i]=new TemperatureSensor(busaRmTempNames[i],nullptr,0.0f);
    if(!busaRmTemps[i])continue;
    busaRmTemps[i]->addParam(Param("Датчик подключен",nullptr,value(false),PROP_FLAG_READ));
    busaRmTemps[i]->addParam(Param(RM_SET,nullptr,value(BusaSound::valid(refTemp[i],noticeTemp[i])),PROP_FLAG_READ));
    busaRmAddFloat(busaRmTemps[i],RM_ON,isfinite(refTemp[i])?refTemp[i]:0.0f,0.0f,125.0f,0.1f);
    busaRmAddFloat(busaRmTemps[i],RM_OFF,isfinite(noticeTemp[i])?noticeTemp[i]:0.0f,0.0f,125.0f,0.1f);
    busaRmAddFloat(busaRmTemps[i],RM_CAL,tempOffset[i],-10.0f,10.0f,0.1f);
    busaRmAddSwitch(busaRmTemps[i],RM_CLEAR,false);
    if(i<2)busaRmAddSwitch(busaRmTemps[i],RM_BUZ,buzzerEnabled[i]);
    if(i==2){
      busaRmAddSwitch(busaRmTemps[i],RM_K1,soundEnabled[0]);
      busaRmAddSwitch(busaRmTemps[i],RM_K2,soundEnabled[1]);
      busaRmAddSwitch(busaRmTemps[i],RM_K3,soundEnabled[2]);
    }
    if(i==3)busaRmAddSwitch(busaRmTemps[i],RM_K4,soundEnabled[3]);
    busaRmTemps[i]->addCb(busaRmWrite);
    node.addDevice(*busaRmTemps[i]);
  }
  busaRmWater=new Device("Расход воды","esp.device.sensor");
  if(busaRmWater){
    busaRmWater->addParam(Param("Расход л/мин",nullptr,value(0.0f),PROP_FLAG_READ));
    busaRmWater->addParam(Param("Объем л",nullptr,value(0.0f),PROP_FLAG_READ));
    busaRmWater->addParam(Param("Калиброван",nullptr,value(false),PROP_FLAG_READ));
    node.addDevice(*busaRmWater);
  }
  busaRmScreen=new Device("Экран БУСА","esp.device.sensor");
  if(busaRmScreen){
    busaRmAddInt(busaRmScreen,RM_THEME,(int)tftThemeMode,0,2);
    busaRmAddInt(busaRmScreen,RM_PAGE,(int)currentPage,0,6);
    busaRmAddSwitch(busaRmScreen,RM_SCREEN,!tftDisplayOff);
    busaRmScreen->addCb(busaRmWrite);
    node.addDevice(*busaRmScreen);
  }
  busaRmKeys=new Device("Клавиатура БУСА","esp.device.sensor");
  if(busaRmKeys){
    busaRmKeys->addParam(Param("PCF8575 подключен",nullptr,value(false),PROP_FLAG_READ));
    busaRmKeys->addParam(Param("Раскладка сохранена",nullptr,value(false),PROP_FLAG_READ));
    busaRmKeys->addParam(Param("Обучение идет",nullptr,value(false),PROP_FLAG_READ));
    busaRmKeys->addParam(Param("Следующая клавиша",nullptr,value(0),PROP_FLAG_READ));
    busaRmAddSwitch(busaRmKeys,RM_KEYLEARN,false);
    busaRmAddSwitch(busaRmKeys,RM_KEYSTOP,false);
    busaRmKeys->addCb(busaRmWrite);
    node.addDevice(*busaRmKeys);
  }
  busaRmHealth=new Device("Состояние БУСА","esp.device.sensor");
  if(busaRmHealth){
    busaRmHealth->addParam(Param("Работает секунд",nullptr,value(0),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Google настроен",nullptr,value(false),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Звук К1",nullptr,value(false),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Звук К2",nullptr,value(false),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Звук К3",nullptr,value(false),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Звук К4",nullptr,value(false),PROP_FLAG_READ));
    busaRmHealth->addParam(Param("Зуммер активен",nullptr,value(false),PROP_FLAG_READ));
    node.addDevice(*busaRmHealth);
  }
  RMaker.enableOTA(OTA_USING_TOPICS);
  RMaker.enableTZService();
  esp_err_t rc=RMaker.start();
  if(rc!=ESP_OK){Serial0.printf("BUSA RainMaker: start error 0x%X\n",(unsigned)rc);return;}
  busaRmStarted=true;
  WiFi.onEvent(busaRmProvisionEvent);
  Serial0.printf("BUSA RainMaker: device %s ready, pairing PoP %s\n",busaRmService,busaRmPoP);
}
static void busaRainMakerProvision(){
  if(!busaRmStarted)return;
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1,busaRmPoP,busaRmService);
}
// Apply only writes accepted by the signed-in RainMaker device owner.
// Never touch main TFT, NVS or GPIO from the RainMaker callback's task.
static void busaRainMakerApply(){
  if(!busaRmStarted||!busaRmQueue||otaActive)return;
  BusaRmRequest q;
  for(int attempt=0;attempt<5 && xQueueReceive(busaRmQueue,&q,0)==pdPASS;attempt++){
    bool ok=false;
    if(q.target<4){
      const int i=q.target;
      if(q.action==0||q.action==1){
        if(isfinite(q.f)&&q.f>=0.0f&&q.f<=125.0f){
          const float rounded=roundf(q.f*10.0f)/10.0f;
          if(fabsf(rounded-q.f)<0.051f)
            ok=storeTempReference(i,(int)q.action,rounded);
        }
      }else if(q.action==2){
        if(isfinite(q.f)&&q.f>=-10.0f&&q.f<=10.0f){
          const float rounded=roundf(q.f*10.0f)/10.0f;
          if(fabsf(rounded-q.f)<0.051f)ok=saveTempOffset((uint8_t)i,rounded);
        }
      }else if(q.action==3 && q.b){
        ok=storeTempReference(i,0,NAN)&&storeTempReference(i,1,NAN);
      }else if(q.action==4){
        if(q.role==255 && i<2){
          const String key=String("buzE")+i;
          ok=prefs.putBool(key.c_str(),q.b);
          if(ok)buzzerEnabled[i]=q.b;
        }else if(q.role<4 && ((i==2 && q.role<3)||(i==3&&q.role==3))){
          const String key=String("sndRoleEnable")+q.role;
          ok=prefs.putBool(key.c_str(),q.b);
          if(ok){soundEnabled[q.role]=q.b;soundResetState();soundOutputsOff();}
        }
      }
    }else if(q.target==4){
      if(q.action==5&&q.i>=0&&q.i<=2)ok=saveThemeMode((uint8_t)q.i);
      else if(q.action==6){
        if(q.b){wakeTftDisplay();ok=true;}
        else {darkenTftDisplay();ok=true;}
      }else if(q.action==7&&q.i>=0&&q.i<7){navigatePage((uint8_t)q.i);ok=true;}
    }else if(q.target==5){
      if(q.action==8&&q.b){
        if(pcfDetected){startKeyLearning();ok=true;}
      }else if(q.action==9&&q.b){
        keypadLearning=false;learnStep=0;candidatePair=-1;candidateStep=255;
        keypadMessage=keypadReady?"Сохранённая раскладка сохранена":"Обучение отменено";
        tftDirty=true;ok=true;
      }
    }
    if(!ok)Serial0.printf("BUSA RainMaker: rejected target=%u action=%u\n",(unsigned)q.target,(unsigned)q.action);
    // Publish the actual local values after each attempted write, not the untrusted requested value.
    busaRmSettingsCursor=(uint8_t)q.target;
    busaRmLastSettingsSent=0;
  }
}
static void busaRainMakerPoll(uint32_t now){
  if(!busaRmStarted||WiFi.status()!=WL_CONNECTED||otaActive||
     busaGoogleNetworkBusy||busaWifiScanRunning)return;
  // Settings sync also reflects changes made via keyboard or original local web UI.
  // One device per interval avoids flooding MQTT and stalling local TFT/I2C.
  if(!busaRmLastSettingsSent||elapsed(now,busaRmLastSettingsSent,2500)){
    busaRmLastSettingsSent=now;
    const uint8_t slot=busaRmSettingsCursor;
    busaRmSettingsCursor=(busaRmSettingsCursor+1)%6;
    if(slot<4&&busaRmTemps[slot]){
      const int i=slot;
      const float on=refTemp[i],off=noticeTemp[i];
      busaRmTemps[i]->updateAndReportParam(RM_ON,isfinite(on)?on:0.0f);
      busaRmTemps[i]->updateAndReportParam(RM_OFF,isfinite(off)?off:0.0f);
      busaRmTemps[i]->updateAndReportParam(RM_CAL,tempOffset[i]);
      busaRmTemps[i]->updateAndReportParam(RM_SET,BusaSound::valid(on,off));
      busaRmTemps[i]->updateAndReportParam(RM_CLEAR,false);
      if(i<2)busaRmTemps[i]->updateAndReportParam(RM_BUZ,buzzerEnabled[i]);
      if(i==2){
        busaRmTemps[i]->updateAndReportParam(RM_K1,soundEnabled[0]);
        busaRmTemps[i]->updateAndReportParam(RM_K2,soundEnabled[1]);
        busaRmTemps[i]->updateAndReportParam(RM_K3,soundEnabled[2]);
      }
      if(i==3)busaRmTemps[i]->updateAndReportParam(RM_K4,soundEnabled[3]);
    }
    if(slot==4&&busaRmScreen){
      busaRmScreen->updateAndReportParam(RM_THEME,(int)tftThemeMode);
      busaRmScreen->updateAndReportParam(RM_SCREEN,!tftDisplayOff);
      busaRmScreen->updateAndReportParam(RM_PAGE,(int)currentPage);
    }
    if(slot==5&&busaRmKeys){
      busaRmKeys->updateAndReportParam("Раскладка сохранена",keypadReady);
      busaRmKeys->updateAndReportParam("Обучение идет",keypadLearning);
      busaRmKeys->updateAndReportParam("Следующая клавиша",keypadLearning?(int)learnStep+1:0);
      busaRmKeys->updateAndReportParam(RM_KEYLEARN,false);
      busaRmKeys->updateAndReportParam(RM_KEYSTOP,false);
    }
  }
  if(busaRmLastSent && !elapsed(now,busaRmLastSent,30000))return;
  busaRmLastSent=now;
  for(int i=0;i<4;i++)if(busaRmTemps[i]){
    const bool present=hasTemp(i,now);
    if(present)busaRmTemps[i]->updateAndReportParam(ESP_RMAKER_DEF_TEMPERATURE_NAME,temperature[i]);
    busaRmTemps[i]->updateAndReportParam("Датчик подключен",present);
  }
  if(busaRmWater){
    busaRmWater->updateAndReportParam("Расход л/мин",flowLpm);
    busaRmWater->updateAndReportParam("Объем л",totalWaterLitres());
    busaRmWater->updateAndReportParam("Калиброван",false);
  }
  if(busaRmHealth){
    busaRmHealth->updateAndReportParam("Работает секунд",(int)(now/1000u));
    busaRmHealth->updateAndReportParam("Google настроен",(bool)googleEnabled);
    busaRmHealth->updateAndReportParam("Звук К1",soundChannels[0].on);
    busaRmHealth->updateAndReportParam("Звук К2",soundChannels[1].on);
    busaRmHealth->updateAndReportParam("Звук К3",soundChannels[2].on);
    busaRmHealth->updateAndReportParam("Звук К4",soundChannels[3].on);
    busaRmHealth->updateAndReportParam("Зуммер активен",buzzerSoundOn);
  }
  if(busaRmKeys)busaRmKeys->updateAndReportParam("PCF8575 подключен",pcfDetected);
}
