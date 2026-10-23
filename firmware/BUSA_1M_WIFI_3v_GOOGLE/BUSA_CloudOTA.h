#pragma once
#include <mbedtls/sha256.h>
#include "BUSA_CloudCA.h"
#include "BUSA_CloudPolicy.h"
// Firmware source is fixed to this repository. Google telemetry cannot issue commands.
const char CLOUD_MANIFEST_URL[]="https://raw.githubusercontent.com/e62815667-gif/BUSA-Monitor/lab/busa-v14-14-google-readonly-audit-20261023/releases/cloud/latest.json";
// Stages: 0 idle, 1 checking, 2 current, 3 available, 4 downloading,
// 5 verified/reboot, 6 error. All shared compound state protected by cloudMux.
portMUX_TYPE cloudMux=portMUX_INITIALIZER_UNLOCKED;
struct CloudCandidate{char url[256]={0},sha[65]={0},version[32]={0};uint32_t size=0,build=0;};
CloudCandidate cloudCandidate;
uint8_t cloudPhase=0,cloudCommand=0;
uint32_t cloudBytes=0;
int cloudError=0;
bool cloudTaskReady=false;
String cloudGrant="",cloudGrantSha="";
uint32_t cloudGrantAt=0;
void cloudResult(uint8_t phase,int error=0,uint32_t bytes=0){
  portENTER_CRITICAL(&cloudMux);cloudPhase=phase;cloudError=error;cloudBytes=bytes;portEXIT_CRITICAL(&cloudMux);
}
String cloudOtaJSON(){
  CloudCandidate c;uint8_t phase;uint32_t bytes;int error;
  portENTER_CRITICAL(&cloudMux);c=cloudCandidate;phase=cloudPhase;bytes=cloudBytes;error=cloudError;portEXIT_CRITICAL(&cloudMux);
  return String("{\"enabled\":")+(cloudTaskReady?"true":"false")+",\"phase\":"+String(phase)+
    ",\"error\":"+String(error)+",\"bytes\":"+String(bytes)+",\"size\":"+String(c.size)+
    ",\"build\":"+String(c.build)+",\"current_build\":"+String(BUSA_BUILD)+
    ",\"version\":"+quoteJson(c.version)+",\"sha256\":"+quoteJson(c.sha)+"}";
}
bool cloudPartitionOK(uint32_t size){
  const esp_partition_t* run=esp_ota_get_running_partition();
  const esp_partition_t* next=esp_ota_get_next_update_partition(nullptr);
  return run&&next&&run->address!=next->address && ESP.getFlashChipSize()==8388608UL &&
    busaSlotOK(run->address,run->size) && busaSlotOK(next->address,next->size) && size<=next->size;
}
void cloudHttpSetup(WiFiClientSecure& secure,HTTPClient& request){
  secure.setCACert(BUSA_CLOUD_CA);secure.setTimeout(15);secure.setHandshakeTimeout(15);
  request.setConnectTimeout(15000);request.setTimeout(15000);request.setReuse(false);
  request.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
}
void cloudCheck(){
  portENTER_CRITICAL(&cloudMux);cloudCandidate=CloudCandidate{};portEXIT_CRITICAL(&cloudMux);
  if(WiFi.status()!=WL_CONNECTED){cloudResult(6,1);return;}
  if(time(nullptr)<1700000000){cloudResult(6,2);return;}
  WiFiClientSecure secure;HTTPClient request;cloudHttpSetup(secure,request);
  if(!request.begin(secure,CLOUD_MANIFEST_URL)){cloudResult(6,3);return;}
  request.addHeader("Accept-Encoding","identity");request.addHeader("Cache-Control","no-cache");
  int code=request.GET();
  if(code!=200){request.end();cloudResult(6,3);return;}
  GoogleReplyBuffer body;
  if(request.getSize()>2048 || request.writeToStream(&body)<0 || body.overflow || !body.used){request.end();cloudResult(6,4);return;}
  request.end();
  StaticJsonDocument<1536> doc;
  if(deserializeJson(doc,body.data,body.used)||!doc.is<JsonObject>()){cloudResult(6,4);return;}
  const char* url=doc["url"]|"";const char* sha=doc["sha256"]|"";const char* version=doc["version"]|"";
  if(!doc["size"].is<uint32_t>()||!doc["build"].is<uint32_t>()||
     !doc["flash_bytes"].is<uint32_t>()||doc["flash_bytes"].as<uint32_t>()!=8388608UL||
     strcmp(doc["device"]|"","BUSA-1M-001")||strcmp(doc["target"]|"","ESP32-S3")||
     strcmp(doc["partition_scheme"]|"","default_8MB")||
     !busaCloudMetaOK(url,sha,version,doc["size"].as<uint32_t>(),doc["build"].as<uint32_t>())){cloudResult(6,4);return;}
  CloudCandidate c;strcpy(c.url,url);strcpy(c.sha,sha);strcpy(c.version,version);
  c.size=doc["size"].as<uint32_t>();c.build=doc["build"].as<uint32_t>();
  if(!cloudPartitionOK(c.size)){cloudResult(6,5);return;}
  portENTER_CRITICAL(&cloudMux);cloudCandidate=c;portEXIT_CRITICAL(&cloudMux);
  cloudResult(c.build>BUSA_BUILD?3:2);
}
void cloudInstall(){
  CloudCandidate c;portENTER_CRITICAL(&cloudMux);c=cloudCandidate;portEXIT_CRITICAL(&cloudMux);
  if(!busaCloudMetaOK(c.url,c.sha,c.version,c.size,c.build)||c.build<=BUSA_BUILD||!cloudPartitionOK(c.size)){cloudResult(6,5);return;}
  if(WiFi.status()!=WL_CONNECTED||time(nullptr)<1700000000){cloudResult(6,1);return;}
  WiFiClientSecure secure;HTTPClient request;cloudHttpSetup(secure,request);
  if(!request.begin(secure,c.url)){cloudResult(6,3);return;}
  request.addHeader("Accept-Encoding","identity");
  if(request.GET()!=200||request.getSize()!=(int)c.size){request.end();cloudResult(6,6);return;}
  WiFiClient* stream=request.getStreamPtr();
  uint8_t buffer[2048],prefix[36];size_t prefixUsed=0;
  uint32_t received=0,lastData=millis(),started=lastData;bool begun=false;int error=0;
  mbedtls_sha256_context hash;mbedtls_sha256_init(&hash);
  if(mbedtls_sha256_starts_ret(&hash,0)!=0)error=10;
  while(!error && received<c.size){
    uint32_t now=millis();
    if(WiFi.status()!=WL_CONNECTED || (uint32_t)(now-lastData)>15000 || (uint32_t)(now-started)>300000){error=7;break;}
    int available=stream->available();
    if(available<=0){if(!request.connected()){error=7;break;}vTaskDelay(pdMS_TO_TICKS(2));continue;}
    size_t count=(size_t)available;if(count>sizeof(buffer))count=sizeof(buffer);if(count>c.size-received)count=c.size-received;
    int n=stream->read(buffer,count);if(n<=0){vTaskDelay(pdMS_TO_TICKS(2));continue;}
    lastData=now;
    if(mbedtls_sha256_update_ret(&hash,buffer,n)!=0){error=10;break;}
    size_t used=0;
    if(prefixUsed<sizeof(prefix)){
      used=sizeof(prefix)-prefixUsed;if(used>(size_t)n)used=n;
      memcpy(prefix+prefixUsed,buffer,used);prefixUsed+=used;
      if(prefixUsed==sizeof(prefix)){
        if(!busaAppHeaderValid(prefix,sizeof(prefix))){error=8;break;}
        if(!Update.begin(c.size,U_FLASH)){error=9;break;}begun=true;
        if(Update.write(prefix,sizeof(prefix))!=sizeof(prefix)){error=9;break;}
      }
    }
    if((size_t)n>used && Update.write(buffer+used,n-used)!=(size_t)n-used){error=9;break;}
    received+=n;cloudResult(4,0,received);vTaskDelay(pdMS_TO_TICKS(1));
  }
  uint8_t digest[32]={0};
  if(!error && mbedtls_sha256_finish_ret(&hash,digest)!=0)error=10;
  mbedtls_sha256_free(&hash);request.end();secure.stop();
  if(!error&&!busaDigestMatches(digest,c.sha))error=10;
  // Only activate the inactive slot AFTER SHA256 over every original byte agrees.
  if(!error && (!begun||received!=c.size||!Update.end(false)))error=11;
  if(error){if(begun)Update.abort();cloudResult(6,error,received);return;}
  cloudResult(5,0,received);
}
void cloudOtaWorker(void*){
  for(;;){
    uint8_t command;
    portENTER_CRITICAL(&cloudMux);command=cloudCommand;cloudCommand=0;portEXIT_CRITICAL(&cloudMux);
    if(command==1)cloudCheck();else if(command==2)cloudInstall();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void cloudOtaTick(){
  if(!cloudBusy)return;
  uint8_t phase;portENTER_CRITICAL(&cloudMux);phase=cloudPhase;portEXIT_CRITICAL(&cloudMux);
  if(phase==2||phase==3||phase==6){cloudBusy=false;otaActive=false;}
  else if(phase==5&&!otaRestart){otaRestart=true;otaRestartAt=millis();}
}
void addCloudOtaRoutes(){
  server.on("/api/cloud/status",HTTP_GET,[]{server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",cloudOtaJSON());});
  server.on("/api/cloud/check",HTTP_POST,[]{
    if(!apClient()||!cloudTaskReady||otaActive||otaRestart||cloudBusy||wifiScanBusy()){
      server.send(409,"application/json","{\"error\":\"local_or_busy\"}");return;
    }
    cloudGrant="";cloudBusy=true;
    portENTER_CRITICAL(&cloudMux);cloudPhase=1;cloudError=0;cloudBytes=0;cloudCommand=1;portEXIT_CRITICAL(&cloudMux);
    server.send(202,"application/json","{\"accepted\":true}");
  });
  server.on("/api/cloud/prepare",HTTP_POST,[]{
    uint8_t phase;CloudCandidate c;portENTER_CRITICAL(&cloudMux);phase=cloudPhase;c=cloudCandidate;portEXIT_CRITICAL(&cloudMux);
    if(!apClient()||server.arg("confirm")!="YES"||phase!=3||cloudBusy||otaActive||otaRestart||wifiScanBusy()||!cloudPartitionOK(c.size)){
      server.send(409,"text/plain","No approved update or device busy");return;
    }
    char token[33];snprintf(token,sizeof(token),"%08lx%08lx%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random());
    cloudGrant=token;cloudGrantSha=c.sha;cloudGrantAt=millis();
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"token\":")+quoteJson(cloudGrant)+"}");
  });
  server.on("/api/cloud/install",HTTP_POST,[]{
    uint8_t phase;CloudCandidate c;portENTER_CRITICAL(&cloudMux);phase=cloudPhase;c=cloudCandidate;portEXIT_CRITICAL(&cloudMux);
    if(!apClient()||server.arg("confirm")!="YES"||cloudGrant.length()!=32||server.header("X-Busa-Token")!=cloudGrant||
       (uint32_t)(millis()-cloudGrantAt)>90000||cloudGrantSha!=String(c.sha)||phase!=3||cloudBusy||otaActive||otaRestart||wifiScanBusy()){
      server.send(403,"text/plain","Confirmation expired or device busy");return;
    }
    cloudGrant="";otaToken="";cloudBusy=true;otaActive=true;
    relaySimStop("CLOUD_OTA");ledSimStop("CLOUD_OTA");
    portENTER_CRITICAL(&cloudMux);cloudPhase=4;cloudBytes=0;cloudError=0;cloudCommand=2;portEXIT_CRITICAL(&cloudMux);
    server.send(202,"application/json","{\"accepted\":true}");
  });
}
void launchCloudOta(){
  cloudTaskReady=xTaskCreatePinnedToCore(cloudOtaWorker,"busa-cloud-ota",12288,nullptr,1,nullptr,0)==pdPASS;
  if(!cloudTaskReady)cloudResult(6,12);
}
