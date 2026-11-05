// Unmodified FIX3 poll: budget-starvation regression fixture.
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
