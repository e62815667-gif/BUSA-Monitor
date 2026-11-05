#pragma once
// SDK callback only queues bounded commands; no math/network/NVS/GPIO in callback.
static void busaRmCalcWrite(Device* device,Param* param,const param_val_t val,void*,write_ctx_t*){
 if(!busaRmQueue||!device||!param)return;
 // Arduino creates a temporary Device wrapper for every callback. Match the
 // stable SDK handle, never the address of that temporary C++ wrapper.
 const auto* handle=device->getDeviceHandle();int mode=-1;
 for(int i=0;i<3;i++)if(handle&&busaRmCalcDevices[i]&&handle==busaRmCalcDevices[i]->getDeviceHandle()){mode=i;break;}
 if(mode<0)return;
 const char* name=param->getParamName();if(!name)return;
 BusaRmRequest q={};q.target=6+mode;bool found=false;
 for(int i=0;i<busaRmCalcInputCounts[mode];i++)if(strcmp(name,busaRmCalcInputNames[mode][i])==0){
  if(val.type!=RMAKER_VAL_TYPE_FLOAT)return;q.action=10;q.role=i;q.f=val.val.f;found=true;break;
 }
 if(!found){
  if(val.type!=RMAKER_VAL_TYPE_BOOLEAN)return;
  if(strcmp(name,"Рассчитать")==0)q.action=11;
  else if(strcmp(name,"Сбросить")==0)q.action=12;
  else return;
  q.b=val.val.b;
 }
 if(xQueueSend(busaRmQueue,&q,0)!=pdPASS)++busaRmWriteQueueDrops;
}
static bool busaRmCalcAddParam(Device* d,const char* name,param_val_t val,uint8_t flags,bool toggle=false){
 Param param(name,nullptr,val,flags);if(!param.getParamHandle())return false;
 if(toggle)param.addUIType(ESP_RMAKER_UI_TOGGLE);
 else param.addUIType(ESP_RMAKER_UI_TEXT);
 return d->addParam(param)==ESP_OK;
}
static void busaRmAddCalculators(Node& node){
 for(int mode=0;mode<3;mode++){
  busaRmCalcReset(mode);
  auto* device=new Device(busaRmCalcNames[mode],"esp.device.other");busaRmCalcDevices[mode]=device;
  if(!device)continue;bool ok=true;
  for(int field=0;field<busaRmCalcInputCounts[mode];field++)
   ok=busaRmCalcAddParam(device,busaRmCalcInputNames[mode][field],value(busaRmCalculators[mode].inputs[field]),PROP_FLAG_READ|PROP_FLAG_WRITE)&&ok;
  ok=busaRmCalcAddParam(device,"Рассчитать",value(false),PROP_FLAG_READ|PROP_FLAG_WRITE,true)&&ok;
  ok=busaRmCalcAddParam(device,"Сбросить",value(false),PROP_FLAG_READ|PROP_FLAG_WRITE,true)&&ok;
  ok=busaRmCalcAddParam(device,"Статус",value(busaRmCalculators[mode].status),PROP_FLAG_READ)&&ok;
  for(int field=0;field<busaRmCalcOutputCounts[mode];field++)
   ok=busaRmCalcAddParam(device,busaRmCalcOutputNames[mode][field],value(busaRmCalculators[mode].outputs[field]),PROP_FLAG_READ)&&ok;
  ok=busaRmCalcAddParam(device,"Примечание",value(busaRmCalcNotes[mode]),PROP_FLAG_READ)&&ok;
  device->addCb(busaRmCalcWrite);
  const esp_err_t rc=node.addDevice(*device);busaRmCalcReady[mode]=ok&&rc==ESP_OK;
  if(!busaRmCalcReady[mode])Serial0.printf("BUSA RainMaker calculator %d: model init failed\n",mode);
 }
}
