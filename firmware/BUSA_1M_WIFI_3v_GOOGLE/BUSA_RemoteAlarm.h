#pragma once
// BUSA-1M: remote configuration of T1-T4 SOUND NOTIFICATION thresholds only.
// Never permits GPIO / valve / heater / relay control commands.
struct BusaRemoteAlarmCommand {
  char id[25]={0};
  float on[4]={}, off[4]={};
};
static portMUX_TYPE busaRemoteAlarmMux = portMUX_INITIALIZER_UNLOCKED;
static BusaRemoteAlarmCommand busaRemoteInbox;
static bool busaRemoteInboxReady=false;
static char busaRemoteAckId[25]={0};
static uint8_t busaRemoteAckState=0; // 0 idle, 1 applied, 2 failed

static bool busaRemoteValidId(const char *id) {
  if(!id || strlen(id)!=24)return false;
  for(int i=0;i<24;i++)if(!isxdigit((unsigned char)id[i]))return false;
  return true;
}
static bool busaRemoteValidPair(float on,float off) {
  if(!isfinite(on)||!isfinite(off)||on<0||on>125||off<0||off>125)return false;
  if(fabsf(on-off)<0.099f)return false;
  return fabsf(on*10.0f-roundf(on*10.0f))<0.02f &&
         fabsf(off*10.0f-roundf(off*10.0f))<0.02f;
}
static void busaRemoteAlarmInit() {
  String last=prefs.getString("remoteAlarmId","");
  if(last.length()==24 && busaRemoteValidId(last.c_str())){
    portENTER_CRITICAL(&busaRemoteAlarmMux);
    memcpy(busaRemoteAckId,last.c_str(),25);
    busaRemoteAckState=1;
    portEXIT_CRITICAL(&busaRemoteAlarmMux);
  }
}
static void busaRemoteAck(const char *id,uint8_t state) {
  portENTER_CRITICAL(&busaRemoteAlarmMux);
  memcpy(busaRemoteAckId,id,25);
  busaRemoteAckState=state;
  portEXIT_CRITICAL(&busaRemoteAlarmMux);
}
static String busaRemoteAlarmJSON() {
  char id[25]={0};
  uint8_t state=0;
  portENTER_CRITICAL(&busaRemoteAlarmMux);
  memcpy(id,busaRemoteAckId,sizeof(id));
  state=busaRemoteAckState;
  portEXIT_CRITICAL(&busaRemoteAlarmMux);
  return String("{\"id\":")+(state?quoteJson(String(id)):String("null"))+
         ",\"state\":\""+(state==1?"applied":state==2?"failed":"idle")+"\"}";
}
// Called only from the HTTPS worker; NEVER writes flash or changes physical pins.
static void busaRemoteEnqueue(const BusaRemoteAlarmCommand &cmd) {
  if(!busaRemoteValidId(cmd.id))return;
  for(int i=0;i<4;i++)if(!busaRemoteValidPair(cmd.on[i],cmd.off[i]))return;
  portENTER_CRITICAL(&busaRemoteAlarmMux);
  if(!busaRemoteInboxReady && !(busaRemoteAckState==1 && strcmp(cmd.id,busaRemoteAckId)==0)){
    busaRemoteInbox=cmd;
    busaRemoteInboxReady=true;
  }
  portEXIT_CRITICAL(&busaRemoteAlarmMux);
}
// Runs on Arduino main loop: all Preferences writes occur in this context.
static void busaRemoteAlarmApply() {
  BusaRemoteAlarmCommand cmd;
  bool available=false;
  portENTER_CRITICAL(&busaRemoteAlarmMux);
  if(busaRemoteInboxReady) {
    cmd=busaRemoteInbox;
    busaRemoteInboxReady=false;
    available=true;
  }
  portEXIT_CRITICAL(&busaRemoteAlarmMux);
  if(!available)return;
  for(int i=0;i<4;i++)if(!busaRemoteValidPair(cmd.on[i],cmd.off[i])){
    busaRemoteAck(cmd.id,2);return;
  }
  const char *onKeys[4]={"buzOn0","buzOn1","sndT3On","sndT4On"};
  const char *offKeys[4]={"buzOff0","buzOff1","sndT3Off","sndT4Off"};
  float priorOn[4],priorOff[4];
  for(int i=0;i<4;i++){
    priorOn[i]=prefs.getFloat(onKeys[i],NAN);
    priorOff[i]=prefs.getFloat(offKeys[i],NAN);
  }
  bool written=true;
  // All values were validated before changing the sound notification configuration.
  for(int i=0;i<4;i++){
    if(prefs.putFloat(onKeys[i],cmd.on[i])!=sizeof(float))written=false;
    if(prefs.putFloat(offKeys[i],cmd.off[i])!=sizeof(float))written=false;
  }
  for(int i=0;i<2;i++)if(!prefs.putBool((String("buzE")+i).c_str(),true))written=false;
  for(int i=0;i<4;i++)if(!prefs.putBool((String("sndRoleEnable")+i).c_str(),true))written=false;
  for(int i=0;i<4;i++){
    if(fabsf(prefs.getFloat(onKeys[i],NAN)-cmd.on[i])>0.051f||
       fabsf(prefs.getFloat(offKeys[i],NAN)-cmd.off[i])>0.051f)written=false;
  }
  if(!written){
    // Roll back the threshold keys on failure instead of claiming saved.
    for(int i=0;i<4;i++){
      if(isfinite(priorOn[i]))prefs.putFloat(onKeys[i],priorOn[i]);
      else prefs.remove(onKeys[i]);
      if(isfinite(priorOff[i]))prefs.putFloat(offKeys[i],priorOff[i]);
      else prefs.remove(offKeys[i]);
    }
    busaRemoteAck(cmd.id,2);
    return;
  }
  soundOutputsOff();
  soundResetState();
  for(int i=0;i<2;i++){
    buzzerOnC[i]=cmd.on[i];buzzerOffC[i]=cmd.off[i];
    buzzerEnabled[i]=true;buzzerTempChannels[i]=BusaSound::Channel();
  }
  for(int i=0;i<3;i++){
    soundOnC[i]=cmd.on[2];soundOffC[i]=cmd.off[2];
  }
  soundOnC[3]=cmd.on[3];soundOffC[3]=cmd.off[3];
  for(int i=0;i<4;i++){
    soundEnabled[i]=true;
    refTemp[i]=cmd.on[i];noticeTemp[i]=cmd.off[i];
  }
  // A successful NVS record avoids re-applying a command after reboot.
  if(prefs.putString("remoteAlarmId",cmd.id)!=24){
    busaRemoteAck(cmd.id,2);
    return;
  }
  tftDirty=true;lastRender=0;
  busaRemoteAck(cmd.id,1);
}
