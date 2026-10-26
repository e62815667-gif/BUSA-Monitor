#pragma once
// V15 boot and cooperative runtime. Included after all component declarations.
void setup(){
  alarmInitPins(); // GPIO7/6/5/4 inactive before radio/display boot
  relaySimStop("BOOT_SAFE");
  ledSimStop("BOOT_LOCKED");
  Serial0.begin(115200);
  Serial0.println("\nBUSA 1M WiFi V15.1: independent four-channel 12V sound alerts");
  Serial.begin(115200);
  delay(400);
  pinMode(FLOW_GPIO,INPUT); // requires an electrically verified 3.3 V logic signal
  attachInterrupt(digitalPinToInterrupt(FLOW_GPIO),flowISR,FALLING);
  Serial0.println("WATER GPIO15 interrupt attached (input only)");
  prefs.begin("busa-r04",false);
  TempBindingRecord bindings={};
  if(prefs.getBytesLength("temp_bind")==sizeof(bindings) &&
     prefs.getBytes("temp_bind",&bindings,sizeof(bindings))==sizeof(bindings) && tempBindingRecordValid(bindings)){
    memcpy(tempBindings,bindings.bindings,sizeof(tempBindings));
  }
  waterPulsesPerLiter=prefs.getFloat("wf_ppl",NAN);
  waterCalibrated=prefs.getBool("wf_ready",false) && isfinite(waterPulsesPerLiter) && waterPulsesPerLiter>=10.0f && waterPulsesPerLiter<=500000.0f;
  if(!waterCalibrated)waterPulsesPerLiter=NAN;
  // User-approved ST7789 320x240 rotation 1 and software-drawn light theme.
  if(!prefs.getBool("v14lightSt",false)){
    prefs.putUChar("tft",1);
    prefs.putUChar("tftRotate",1);
    prefs.putBool("tftInvert",false);
    prefs.putBool("v14lightSt",true);
  }
  tftProfile=1;tftRotation=1;tftInverted=false;
  tftThemeDark=prefs.getBool("tftDarkColor",false);
  startedAt=millis();
  tftProfile=1; // ST7789 only
  // Initialize TFT first so splash actually covers Wi-Fi and sensor initialization.
  screenAttempted=true;
  startScreen();
  stationSsid=prefs.getString("ssid","");
  stationPass=prefs.getString("pass","");
  loadKeymap();
  clockZoneKnown=prefs.getBool("clkZone",false);
  clockUtcOffsetMinutes=prefs.getInt("clkTz",0);
  if(clockUtcOffsetMinutes<(-12*60)||clockUtcOffsetMinutes>14*60)clockZoneKnown=false;
  for(int i=0;i<4;i++){
    String k=String("rf")+String(i),j=String("nt")+String(i);
    refTemp[i]=prefs.getFloat(k.c_str(),NAN);
    noticeTemp[i]=prefs.getFloat(j.c_str(),NAN);
    if(!isfinite(refTemp[i])||refTemp[i]<0||refTemp[i]>125)refTemp[i]=NAN;
    if(!isfinite(noticeTemp[i])||noticeTemp[i]<0||noticeTemp[i]>125)noticeTemp[i]=NAN;
  }
  alarmLoadSettings();
  setupGoogleCloud();
  WiFi.persistent(false); // Credentials live only in application Preferences.
  WiFi.setAutoReconnect(false); // Only wifiRecoveryTick owns STA attempts.
  WiFi.onEvent(wifiRecoveryEvent);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false); // Favor interactive local response over power saving
  configTime(0,0,"pool.ntp.org","time.google.com"); // NTP UTC; local offset from paired phone
  clockNtpStarted=true;
  // Private STA password must be provisioned into NVS after public GitHub build;
  // no STA password is embedded in this public source. Local AP fallback remains.
  {
    uint64_t mac=ESP.getEfuseMac();
    char pwd[24];
    snprintf(pwd,sizeof(pwd),"BUSA%016llX",(unsigned long long)mac);
    apPassword=String(pwd);
  }
  wifiRunning=WiFi.softAP("BUSA-1M-001-R04",apPassword.c_str(),BUSA_AP_CHANNEL,false,4);
  esp_wifi_set_bandwidth(WIFI_IF_AP,WIFI_BW_HT20);
  updateBootStage(1);
  Serial0.printf("AP=%d IP=%s SSID=BUSA-1M-001-R04\n",
                 (int)wifiRunning,WiFi.softAPIP().toString().c_str());
  wifiStartPolicy(millis()); // Brief startup grace; local clients do not block internet association.
  server.on("/",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store, max-age=0");
    server.send_P(200,"text/html; charset=utf-8",WEB_PAGE);
  });
  // Minimal independent endpoints for diagnosing an unreachable main app.
  server.on("/api/ping",HTTP_GET,[]{
    String out="{\"device\":\"BUSA-1M-001\",\"version\":"+quoteJson(String(VERSION));
    out+=",\"uptime_ms\":"+String(millis());
    out+=",\"ap_running\":"+(wifiRunning?String("true"):String("false"));
    out+=",\"ap_clients\":"+String(WiFi.softAPgetStationNum());
    out+=",\"sta_connected\":"+(WiFi.status()==WL_CONNECTED?String("true"):String("false"));
    out+=",\"free_heap\":"+String(ESP.getFreeHeap())+"}";
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",out);
  });
  server.on("/recovery",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"text/html; charset=utf-8",
      String("<!doctype html><html lang='ru'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>")
      +"<title>БУСА · проверка связи</title><body style='font:17px system-ui;padding:16px'>"
      +"<h2>БУСА-1М · Проверка связи</h2><p>Версия: "+String(VERSION)+"</p>"
      +"<p>Локальная сеть: "+(wifiRunning?String("работает"):String("нет"))+"</p>"
      +"<p>Клиентов: "+String(WiFi.softAPgetStationNum())+"</p>"
      +"<p>Подключение к интернету: "+(WiFi.status()==WL_CONNECTED?String("да"):String("нет"))+"</p>"
      +"<p><a href='/api/ping'>Ответ ESP32 (JSON)</a></p>"
      +"<p><a href='/'>Открыть приложение</a></p></body></html>");
  });
  server.on("/api/time/sync",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная сеть БУСА");return;
    }
    String e=server.arg("epoch"),tz=server.arg("tz");
    char *end=nullptr,*tzEnd=nullptr;
    long long sec=strtoll(e.c_str(),&end,10);
    long offset=strtol(tz.c_str(),&tzEnd,10);
    if(e.isEmpty()||!end||*end||tz.isEmpty()||!tzEnd||*tzEnd||
       sec<1609459200LL||sec>4102444800LL||offset<-720||offset>840){
       server.send(400,"text/plain","Неверные дата/время/часовой пояс");return;
    }
    timeval tv;tv.tv_sec=(time_t)sec;tv.tv_usec=0;
    if(settimeofday(&tv,nullptr)!=0){
      server.send(500,"text/plain","Не удалось синхронизировать часы");return;
    }
    clockUtcOffsetMinutes=(int32_t)offset;clockZoneKnown=true;
    prefs.putInt("clkTz",clockUtcOffsetMinutes);
    prefs.putBool("clkZone",true);
    Serial0.println("CLOCK: synchronized from phone, UTC offset stored");
    // V14.6 live clock refreshes its own header strip; no full-screen flash.
    if(screen && screen->width()<300)tftDirty=true;
    server.send(200,"application/json",String("{\"datetime\":")+quoteJson(clockStamp(false))+"}");
  });
  server.on("/api/temperature/bind",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="BIND_TEMPERATURES"||otaActive||otaRestart){
      server.send(403,"text/plain; charset=utf-8","Нужны локальная сеть и подтверждение привязки");return;
    }
    TempBindingRecord record={};record.magic=0x42544D50U;record.version=1;
    for(int i=0;i<4;i++){
      if(!tempParseSelection(server.arg(String("t")+String(i)),record.bindings[i])){
        server.send(400,"text/plain; charset=utf-8","Неверный GPIO или ROM датчика");return;
      }
    }
    if(!tempValidateBindings(record.bindings)){
      server.send(409,"text/plain; charset=utf-8","Датчик не найден или один ROM назначен двум каналам. Обнови список.");return;
    }
    record.crc=OneWire::crc8((const uint8_t*)&record,offsetof(TempBindingRecord,crc));
    if(prefs.putBytes("temp_bind",&record,sizeof(record))!=sizeof(record)){
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить привязку");return;
    }
    memcpy(tempBindings,record.bindings,sizeof(tempBindings));tempSyncLogical(millis());tftDirty=true;
    server.send(200,"application/json","{\"saved\":true}");
  });
  server.on("/api/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",statusJSON());
  });
  // Local Wi-Fi-only in-memory simulation. No direct GPIO, coil or PWM actuation.
  // Sound-only K1..K4 are intentionally independent of the legacy virtual test.
  server.on("/api/alarm/config",HTTP_POST,[]{
    if(!apClient() || server.arg("confirm")!="SOUND_ALARMS_ONLY" || otaActive || otaRestart){
      server.send(403,"text/plain; charset=utf-8","Настройка звуковой сигнализации доступна только из локальной сети БУСА");
      return;
    }
    bool requested[4]={false,false,false,false};
    for(int i=0;i<4;i++){
      const String value=server.arg(String("enable")+String(i));
      if(value!="0" && value!="1"){
        server.send(400,"text/plain; charset=utf-8","Нужен выбор каждого из четырёх каналов");return;
      }
      requested[i]=value=="1";
      if(requested[i] && !BusaAlarm::validPair(refTemp[i],noticeTemp[i])){
        server.send(422,"text/plain; charset=utf-8",
          String("К")+String(i+1)+": сначала укажи Твкл и Тоткл с разницей от 0,5 °C");
        return;
      }
    }
    // Conservative on settings update: cut all outputs before saving.
    alarmOutputsOff();
    for(int i=0;i<4;i++){
      alarmEnabled[i]=false;
      const String key=String("aen")+String(i);
      if(!prefs.putBool(key.c_str(),requested[i])){
        alarmOutputsOff();
        server.send(500,"text/plain; charset=utf-8","Не удалось сохранить настройки");return;
      }
      alarmEnabled[i]=requested[i];
    }
    alarmLastPoll=0;
    server.send(200,"application/json; charset=utf-8",alarmStatusJSON(millis()));
  });
  server.on("/api/relay/sim",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Только защищённая локальная сеть");
      return;
    }
    const String action=server.arg("action");
    if(action=="stop"){
      relaySimStop("EMERGENCY_STOP");
      server.send(200,"application/json",relaySimJSON());return;
    }
    if(action!="single"||server.arg("confirm")!="DRY_RUN_ONLY"||otaActive){
      server.send(403,"text/plain; charset=utf-8","Только подтверждённая симуляция без физических сигналов");return;
    }
    const String ch=server.arg("channel");
    if(ch.length()!=1||ch[0]<'1'||ch[0]>'4'||relaySimChannel){
      server.send(409,"text/plain; charset=utf-8","Только один виртуальный канал за раз");return;
    }
    relaySimChannel=(uint8_t)(ch[0]-'0');
    relaySimStart=millis();relaySimUntil=relaySimStart+RELAY_SIM_MAX_MS;
    relaySimReason="DRY_RUN_ONLY";
    server.send(200,"application/json",relaySimJSON());
  });
  server.on("/api/led/sim",HTTP_POST,[]{
    if(!apClient()){
      server.send(403,"text/plain; charset=utf-8","Требуется защищённая локальная сеть БУСА");return;
    }
    const String action=server.arg("action");
    if(action=="stop"){
      ledSimStop("STOP_REQUESTED");
      server.send(200,"application/json",ledSimJSON());return;
    }
    if(action!="test"||server.arg("confirm")!="DRY_RUN_ONLY"||otaActive){
      server.send(403,"text/plain; charset=utf-8","Разрешена только виртуальная проверка LED");return;
    }
    const String arg=server.arg("channel");
    if(arg.isEmpty()||arg.length()>2){
      server.send(400,"text/plain","Неверный канал");return;
    }
    for(unsigned i=0;i<arg.length();i++)if(arg[i]<'0'||arg[i]>'9'){
      server.send(400,"text/plain","Неверный номер");return;
    }
    const int ch=arg.toInt();
    if(ch<0||ch>=BusaLed::kCount||ledSimChannel>=0){
      server.send(409,"text/plain","Тест занят или канал недоступен");return;
    }
    ledSimChannel=(int8_t)ch;ledSimUntil=millis()+LED_SIM_MAX_MS;
    ledSimReason="DRY_RUN_ONLY";
    server.send(200,"application/json",ledSimJSON());
  });
  // All calibration actions are low-voltage, read-only monitoring settings.
  // Never expose these write operations through the public Internet.
  server.on("/api/water/calibration/start",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    waterCalStartPulses=flowPulses();waterCalSession=true;
    server.send(200,"text/plain; charset=utf-8","Старт записан. Измеренные импульсы считаются, пока активна проба.");
  });
  server.on("/api/water/calibration/finish",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    if(!waterCalSession){
      server.send(409,"text/plain; charset=utf-8","Сначала начните пробу калибровки");return;
    }
    const String input=server.arg("volume_ml");
    const float measuredMl=input.toFloat();
    if(input.isEmpty()||!isfinite(measuredMl)||measuredMl<100.0f||measuredMl>50000.0f){
      server.send(400,"text/plain; charset=utf-8","Укажите реальный измеренный объём воды: 100–50000 мл");return;
    }
    const uint32_t pulseDelta=flowPulses()-waterCalStartPulses;
    if(pulseDelta<10){
      server.send(400,"text/plain; charset=utf-8","Импульсов недостаточно для калибровки");return;
    }
    const float factor=(float)pulseDelta*1000.0f/measuredMl;
    if(!isfinite(factor)||factor<10.0f||factor>500000.0f){
      server.send(400,"text/plain; charset=utf-8","Получен неправдоподобный коэффициент. Проверьте объём и импульсы.");return;
    }
    if(prefs.putFloat("wf_ppl",factor)==0||prefs.putBool("wf_ready",true)==0){
      server.send(500,"text/plain; charset=utf-8","Не удалось сохранить калибровку");return;
    }
    waterPulsesPerLiter=factor;waterCalibrated=true;waterCalSession=false;
    waterCalLastPulses=pulseDelta;waterCalLastMl=measuredMl;
    server.send(200,"text/plain; charset=utf-8",
      String("Калибровка сохранена: ")+String(pulseDelta)+" имп. / "+
      String(measuredMl,1)+" мл; "+String(factor,2)+" имп./л");
  });
  server.on("/api/water/calibration/reset",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    prefs.remove("wf_ready");prefs.remove("wf_ppl");
    waterCalibrated=false;waterCalSession=false;waterPulsesPerLiter=NAN;flowLpm=NAN;
    server.send(200,"text/plain; charset=utf-8","Калибровка сброшена; отображаются только импульсы.");
  });

  addGoogleRoutes();
  server.on("/api/ota/status",HTTP_GET,[]{
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",otaJSON());
  });
  server.on("/api/ota/prepare",HTTP_POST,[]{
    alarmOutputsOff();
    relaySimStop("OTA_PREPARE");
    ledSimStop("OTA_PREPARE");
    if(!apClient()||server.arg("confirm")!="YES"||otaActive||otaRestart||wifiScanBusy()){
      server.send(403,"text/plain","Требуется локальная сеть БУСА и согласие");return;
    }
    const String sizeText=server.arg("size");
    char *sizeEnd=nullptr;
    const unsigned long size=strtoul(sizeText.c_str(),&sizeEnd,10);
    const esp_partition_t* slot=esp_ota_get_next_update_partition(nullptr);
    if(sizeText.isEmpty()||!sizeEnd||*sizeEnd||!slot||size<60000||size>slot->size){
      server.send(400,"text/plain","APP size / OTA partition invalid");return;
    }
    otaExpectedBytes=size;otaSuccess=false;otaUploadCompleted=false;
    char token[17];
    snprintf(token,sizeof(token),"%08lx%08lx",
      (unsigned long)esp_random(),(unsigned long)esp_random());
    otaToken=token;otaTokenIssued=millis();
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"token\":")+quoteJson(otaToken)+"}");
  });
  server.on("/api/ota/upload",HTTP_POST,[]{
    server.sendHeader("Cache-Control","no-store");
    const bool completed=otaUploadCompleted;
    if(!completed){server.send(400,"application/json","{\"success\":false,\"message\":\"No completed upload\"}");return;}
    server.send(otaSuccess?200:400,"application/json",otaJSON());
    otaUploadCompleted=false;
  },uploadOTA);
  server.on("/api/wifi/scan",HTTP_GET,[]{
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"application/json","{\"error\":\"Only protected local AP\"}");return;
    }
    if(otaActive || otaRestart){
      server.send(409,"application/json","{\"error\":\"ota_busy\"}");return;
    }
    if(server.hasArg("start") && server.arg("start")=="1")wifiRequestScan(millis());
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json; charset=utf-8",wifiScanJSON());
  });
  server.on("/api/wifi",HTTP_POST,[]{
    // Credentials may only be provisioned while connected to the protected local AP.
    if(server.client().localIP()!=WiFi.softAPIP()){
      server.send(403,"text/plain; charset=utf-8","Настройка Wi-Fi доступна только через сеть БУСА.");return;
    }
    if(otaActive||otaRestart){server.send(409,"text/plain","OTA busy");return;}
    String ssid=server.arg("ssid"),pass=server.arg("pass");
    ssid.trim();
    if(ssid.isEmpty()||ssid.length()>32||pass.length()<8||pass.length()>63){
      server.send(400,"text/plain; charset=utf-8","Недопустимое имя сети или длина пароля (8–63).");return;
    }
    // NVS is encrypted only if board flash encryption is enabled; keep local AP secure.
    bool ok=(prefs.putString("ssid",ssid)>0 && prefs.putString("pass",pass)>0);
    if(!ok){server.send(500,"text/plain; charset=utf-8","Не удалось сохранить Wi-Fi настройки");return;}
    stationSsid=ssid;stationPass=pass;
    wifiCancelRequested=true;
    wifiScheduleConnect(millis());
    server.send(202,"text/plain; charset=utf-8","Wi-Fi сохранён. БУСА подключится к интернету автоматически; оставайся в локальной сети. При смене радиоканала возможна краткая пауза, приложение повторит запрос.");
  });
  server.on("/api/page",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<1||n>6){server.send(400,"application/json","{\"error\":\"page 1..6\"}");return;}
    navigatePage((uint8_t)(n-1));editingTemp=false;tftView=0;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/tft/profile",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная сеть БУСА");return;
    }
    if(server.arg("profile")!="1"||server.arg("rotation")!="1"){
      server.send(400,"text/plain; charset=utf-8","Зафиксированы ST7789 и поворот 1");return;
    }
    prefs.putUChar("tft",1);prefs.putUChar("tftRotate",1);
    server.send(200,"text/plain; charset=utf-8","ST7789 320×240 сохранён");
  });
  server.on("/api/tft/screen",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;}
    String action=server.arg("action");
    if(action=="wake"){
      wakeTftDisplay();
      server.send(200,"text/plain; charset=utf-8","TFT включён");
    }else if(action=="sleep"){
      darkenTftDisplay();
      server.send(200,"text/plain; charset=utf-8","TFT погашен; ESP32 продолжает мониторинг");
    }else server.send(400,"text/plain; charset=utf-8","Неверная команда");
  });
  server.on("/api/tft/test",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Нужна локальная Wi-Fi сеть БУСА");return;
    }
    if(!tftStarted||!screen){
      server.send(409,"text/plain; charset=utf-8","Дисплей ещё не запускался");return;
    }
    drawBootSplash(5000);
    lastRender=0;
    server.send(200,"text/plain; charset=utf-8",
      "Тест отправлен: 5 секунд чёрного фона и белой надписи BUSA 1M AI.");
  });
  server.on("/api/tft/theme",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain; charset=utf-8","Только локальная сеть БУСА");return;
    }
    const String theme=server.arg("theme");
    if(theme!="dark"&&theme!="light"){
      server.send(400,"application/json","{\"error\":\"use explicit theme dark or light\"}");return;
    }
    const bool dark=theme=="dark";
    if(dark!=tftThemeDark && prefs.putBool("tftDarkColor",dark)!=1){
      server.send(500,"application/json","{\"error\":\"theme persist failed\"}");return;
    }
    tftThemeDark=dark;
    tftDirty=true;lastRender=0;
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",
      String("{\"theme\":")+(dark?"\"dark\"":"\"light\"")+",\"saved\":true}");
  });
  server.on("/api/keypad/start",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(!pcfDetected){server.send(409,"text/plain","PCF8575 not found");return;}
    startKeyLearning();
    server.send(200,"text/plain; charset=utf-8","Обучение: нажми F1 и отпусти");
  });
  server.on("/api/keypad/next",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){
      server.send(403,"text/plain","Local AP only");return;
    }
    if(!keypadLearning){
      server.send(409,"text/plain; charset=utf-8","Обучение сейчас не запущено");return;
    }
    // Try a fresh scan if the key is still held; otherwise use the last pair
    // observed at the CURRENT step, only within the last 10 seconds.
    int detected=scanKeyPair(),pair=-1;
    if(detected>=0)pair=detected;
    else if(candidatePair>=0&&candidateStep==learnStep &&
            !elapsed(millis(),candidateAt,10000))pair=candidatePair;
    if(pair<0){
      keypadMessage=detected==-2?
        "Несколько линий замкнуты: отпусти другие клавиши":
        "Сигнала нет: F1 или текущая клавиша не распознана. Повтори нажатие.";
      server.send(409,"text/plain; charset=utf-8",keypadMessage);return;
    }
    if(!learnCommitPair(pair)){
      server.send(409,"text/plain; charset=utf-8",keypadMessage);return;
    }
    keyHeld=true; // Avoid double submission of the same held key
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
  });
  server.on("/api/keypad/undo",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    if(!keypadLearning||learnStep==0){server.send(409,"text/plain","No previous step");return;}
    learnStep--;newPairs[learnStep]=255;
    pendingPair=-1;keyHeld=false;candidatePair=-1;candidateStep=255;
    keypadMessage="Повтори "+String(KEY_CAPS[learnStep]);
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
  });
  server.on("/api/keypad/cancel",HTTP_POST,[]{
    if(!apClient()||server.arg("confirm")!="YES"){server.send(403,"text/plain","Local AP only");return;}
    keypadLearning=false;learnStep=0;candidatePair=-1;candidateStep=255;
    keypadMessage=keypadReady?"Сохранённая раскладка сохранена":"Обучение отменено";
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8",keypadMessage);
  });
  server.on("/api/display-refs",HTTP_POST,[]{
    if(!apClient()){server.send(403,"text/plain; charset=utf-8","Нужна локальная точка доступа БУСА");return;}
    float target[4],notice[4];
    for(int i=0;i<4;i++){
      String a=server.arg(String("ref")+i),b=server.arg(String("nt")+i);
      target[i]=a.length()?a.toFloat():NAN;
      notice[i]=b.length()?b.toFloat():NAN;
      if((isfinite(target[i])&&(target[i]<0||target[i]>125))||
         (isfinite(notice[i])&&(notice[i]<0||notice[i]>125))){
         server.send(400,"text/plain","Диапазон 0..125");return;
      }
    }
    for(int i=0;i<4;i++){
      String k=String("rf")+i,j=String("nt")+i;
      if(isfinite(target[i]))storeTempReference(i,0,target[i]);
      else{prefs.remove(k.c_str());refTemp[i]=NAN;}
      if(isfinite(notice[i]))storeTempReference(i,1,notice[i]);
      else{prefs.remove(j.c_str());noticeTemp[i]=NAN;}
    }
    alarmOutputsOff(); // no outdated setpoints can keep a buzzer energized
    alarmLastPoll=0;
    tftDirty=true;
    server.send(200,"text/plain; charset=utf-8","Пороги звуковой сигнализации сохранены.");
  });
  server.on("/api/graph-view",HTTP_GET,[]{
    int n=server.arg("view").toInt();
    if(n<0||n>2){server.send(400,"text/plain","Bad graph");return;}
    graphSubView=(uint8_t)n;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/settings-tab",HTTP_GET,[]{
    int n=server.arg("n").toInt();
    if(n<0||n>3){server.send(400,"text/plain","bad subtab");return;}
    settingsSubTab=(uint8_t)n;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  server.on("/api/tft/scroll",HTTP_GET,[]{
    String dir=server.arg("dir");
    if(dir!="up"&&dir!="down"){server.send(400,"text/plain","bad dir");return;}
    tftView=(dir=="down")?1:0;tftDirty=true;lastRender=0;
    server.send(200,"application/json","{\"ok\":true}");
  });
  const char *headers[]={"X-Busa-Token"};
  server.collectHeaders(headers,1);
  server.begin();
  Serial0.println("WEB started; temperature scan & I2C next");
  i2cOnline=Wire.begin(BUS_SDA,BUS_SCL,100000);
  if(i2cOnline){Wire.setTimeOut(25);probeI2c(millis());}
  updateBootStage(2);
  Serial0.printf("I2C=%d PCF=%d PCA=%d\n",(int)i2cOnline,(int)pcfDetected,(int)pcaDetected);
  updateBootStage(3);
  launchGoogleCloud();
  Serial0.println("SETUP DONE: K1-K4 only for 12V sound annunciators; no equipment control");
}
void loop(){
  uint32_t now=millis();
  alarmPoll(now);
  relaySimTick(now);
  ledSimTick(now);
  if(lastLoopStart){const uint32_t gap=now-lastLoopStart;if(gap>loopMaxGapMs)loopMaxGapMs=gap;}
  lastLoopStart=now;
  wifiRecoveryTick(now);
  const uint32_t webStart=millis();
  server.handleClient();
  const uint32_t webElapsed=millis()-webStart;
  if(webElapsed>webMaxServiceMs)webMaxServiceMs=webElapsed;
  readConsole();
  if(otaRestart && elapsed(now,otaRestartAt,4000))ESP.restart();
  // Avoid I2C/1-Wire/SPI scanning while Wi-Fi OTA flash chunks arrive.
  // Flash throughput remains hardware/network limited.
  if(otaActive){alarmOutputsOff();relaySimStop("OTA_TRANSFER");ledSimStop("OTA_TRANSFER");delay(0);return;}
  if(tftRebootPending && elapsed(now,tftRebootAt,3000))ESP.restart();
  if(googleRestart && elapsed(now,googleRestartAt,3500))ESP.restart();
  publishGoogleSnapshot(now);
  temperatureTick(millis()); // One bounded temperature bus operation per iteration
  // A missing sensor is a diagnostic error, not a reason to trap the UI on boot.
  // Wi-Fi AP and HTTP server initialize during the splash.
  // Missing sensors must not delay the user interface.
  if(!bootSplashComplete && elapsed(now,splashStartedAt,8000)){
    bootSplashComplete=true;
    tftDirty=true;
  }
    sampleFlow(now);
  if(i2cOnline){
    if(elapsed(now,lastBusProbe,6000))probeI2c(now);
    pollKeys(now);
  }
  // Landscape V14.6: rebuild background ONLY on explicit view/theme change.
  // A 2.5 s telemetry tick refreshes changed text without wiping the TFT.
  if(tftStarted && !tftDisplayOff && !otaActive && bootSplashComplete &&
     elapsed(now,splashStartedAt,splashHoldMs) &&
     (tftDirty||lastRender==0 || elapsed(now,lastRender,2500))){
    const bool wholeFrame=tftDirty||lastRender==0;
    lastRender=now;
    if(screen->width()>=300 && !wholeFrame)v14RefreshLive(now);
    else renderScreen(now);
  }
  if(elapsed(now,lastUart,5000)){
    lastUart=now;
    Serial0.printf("N8R2 HEARTBEAT ms=%lu wifi=%d i2c=%d tft=%d p=%lu flow=%.3f\n",
      (unsigned long)now,(int)wifiRunning,(int)i2cOnline,(int)tftStarted,
      (unsigned long)flowPulses(),(double)flowLpm);
  }
  delay(2);
}
