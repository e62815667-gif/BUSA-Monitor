static void busaRainMakerStart(){
  if(!busaRmQueue)busaRmQueue=xQueueCreate(20,sizeof(BusaRmRequest));
  if(!busaRmQueue){Serial0.println("BUSA RainMaker: no memory for settings queue");return;}
  snprintf(busaRmService,sizeof(busaRmService),"PROV_BUSA_%04X",(unsigned)(ESP.getEfuseMac()&0xFFFF));
  // Approved fixed RainMaker BLE proof-of-possession (nine ASCII digits).
  // Do not read previously generated eight-digit PoP from NVS.
  constexpr const char *BUSA_RM_FIXED_POP="123456789";
  strncpy(busaRmPoP,BUSA_RM_FIXED_POP,sizeof(busaRmPoP)-1);
  busaRmPoP[sizeof(busaRmPoP)-1]='\0';
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
  esp_err_t rc=RMaker.start();
  if(rc!=ESP_OK){Serial0.printf("BUSA RainMaker: start error 0x%X\n",(unsigned)rc);return;}
  busaRmStarted=true;
  WiFi.onEvent(busaRmProvisionEvent);
  Serial0.printf("BUSA RainMaker: device %s ready\n",busaRmService);
}
