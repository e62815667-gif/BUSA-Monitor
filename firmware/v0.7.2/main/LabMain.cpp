// Real monitoring only. NO relay, keypad, PCA9685 or TFT output writes.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <cJSON.h>
#include "TemperatureBus.h"
#include "LabPage.h"
constexpr bool PHYSICAL_OUTPUTS=false;
static_assert(!PHYSICAL_OUTPUTS,"Actuators forbidden");
const char *VERSION="v0.7.2-lab.1";
WebServer web(80); Preferences prefs;
String streamId,token,ssid,password;
bool armed=false,storage=false;
uint32_t sequence=0,revision=0,lastReconnect=0;
float warnings[4]={NAN,NAN,NAN,NAN},limits[4]={NAN,NAN,NAN,NAN},volume=NAN;
String randomHex(){char b[33];snprintf(b,sizeof(b),"%08lx%08lx%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random());return b;}
String quote(const String &s){
 String o="\"";for(size_t i=0;i<s.length();i++){uint8_t c=s[i];
 if(c=='"'||c=='\\'){o+='\\';o+=(char)c;}else if(c<32){char b[7];snprintf(b,7,"\\u%04x",c);o+=b;}else o+=(char)c;}return o+"\"";
}
String num(float f){return isfinite(f)?String(f,2):String("null");}
void json(int code,const String &s){web.sendHeader("Cache-Control","no-store");web.sendHeader("X-Content-Type-Options","nosniff");web.send(code,"application/json; charset=utf-8",s);}
String settings(){
 String j="{\"revision\":"+String(revision)+",\"warning\":[";
 for(int i=0;i<4;i++){if(i)j+=",";j+=num(warnings[i]);}j+="],\"automation\":[";
 for(int i=0;i<4;i++){if(i)j+=",";j+=num(limits[i]);}
 return j+"],\"volume_l\":"+num(volume)+",\"physical_outputs\":false}";
}
bool nullable(cJSON *p,float &v,float lo,float hi){
 if(cJSON_IsNull(p)){v=NAN;return true;}
 if(!cJSON_IsNumber(p)||!isfinite(p->valuedouble)||p->valuedouble<lo||p->valuedouble>hi)return false;
 v=p->valuedouble;return true;
}
bool parseSettings(cJSON *r,float *w,float *a,float &v){
 if(!cJSON_IsObject(r))return false;
 auto *wa=cJSON_GetObjectItemCaseSensitive(r,"warning"),*aa=cJSON_GetObjectItemCaseSensitive(r,"automation");
 if(!cJSON_IsArray(wa)||!cJSON_IsArray(aa)||cJSON_GetArraySize(wa)!=4||cJSON_GetArraySize(aa)!=4)return false;
 for(int i=0;i<4;i++)if(!nullable(cJSON_GetArrayItem(wa,i),w[i],-55,125)||!nullable(cJSON_GetArrayItem(aa,i),a[i],-55,125))return false;
 return nullable(cJSON_GetObjectItemCaseSensitive(r,"volume_l"),v,0,100);
}
void saveSettings(){
 if(web.header("X-BUSA-Token")!=token){json(403,"{\"error\":\"pairing_required\"}");return;}
 if(web.header("Content-Type")!="application/json"){json(415,"{\"error\":\"json_required\"}");return;}
 String b=web.arg("plain");if(b.length()>1024){json(413,"{\"error\":\"body_too_large\"}");return;}
 cJSON *r=cJSON_ParseWithLength(b.c_str(),b.length());float w[4],a[4],v;
 auto *rev=r?cJSON_GetObjectItemCaseSensitive(r,"revision"):nullptr;
 if(!parseSettings(r,w,a,v)||!cJSON_IsNumber(rev)||floor(rev->valuedouble)!=rev->valuedouble||rev->valuedouble<0){cJSON_Delete(r);json(400,"{\"error\":\"invalid_settings\"}");return;}
 if(rev->valuedouble!=revision){cJSON_Delete(r);json(409,"{\"error\":\"revision_conflict\"}");return;}cJSON_Delete(r);
 if(!storage){json(503,"{\"error\":\"nvs_unavailable\"}");return;}
 float ow[4],oa[4],ov=volume;uint32_t oldRev=revision;
 for(int i=0;i<4;i++){ow[i]=warnings[i];oa[i]=limits[i];warnings[i]=w[i];limits[i]=a[i];}volume=v;++revision;
 String saved=settings();
 if(prefs.putString("settings",saved)!=saved.length()){
 for(int i=0;i<4;i++){warnings[i]=ow[i];limits[i]=oa[i];}volume=ov;revision=oldRev;json(503,"{\"error\":\"save_failed\"}");return;}
 json(200,saved);
}
void capabilities(){
 json(200,"{\"device\":\"BUSA-1M-001\",\"version\":"+quote(VERSION)+
 ",\"api_read_only\":true,\"settings_write\":true,\"physical_outputs\":false,\"temperature_channels\":4,"
 "\"keyboard\":\"4x4 PCF8575\",\"keyboard_enabled\":false,\"display\":false,\"pca9685_enabled\":false,"
 "\"automation_implemented\":false,\"level_mode\":\"manual\",\"simulation\":false}");
}
void telemetry(){
 uint32_t now=millis();String j="{\"device\":\"BUSA-1M-001\",\"version\":"+quote(VERSION)+
 ",\"mode\":\"MONITOR_ONLY\",\"outputs_enabled\":false,\"simulation\":false,\"uptime_ms\":"+String(now)+
 ",\"sequence\":"+String(++sequence)+",\"bridge_online\":"+String(bridgeFound?"true":"false")+
 ",\"sensors_enabled\":"+String(armed?"true":"false")+",\"bridge_address\":"+String(DS2482_ADDR)+
 ",\"keypad_online\":false,\"last_key\":null,\"key_seq\":0,\"key_stream_id\":"+quote(streamId)+",\"temperatures\":[";
 for(int i=0;i<4;i++){if(i)j+=",";j+=bridgeFound&&fresh(sensors[i],now,3000)?num(sensors[i].value):String("null");}j+="],\"channels\":[";
 for(int i=0;i<4;i++){
 if(i){j+=",";}
 auto &s=sensors[i];bool valid=bridgeFound&&fresh(s,now,3000);String rom;
 for(uint8_t v:s.rom){char b[3];snprintf(b,3,"%02X",v);rom+=b;}
 auto cmp=[&](float f){return !valid||!isfinite(f)?String("null"):String(s.value>=f?"true":"false");};
 j+="{\"id\":"+quote(s.id)+",\"name\":"+quote(s.name)+",\"rom\":"+quote(rom)+",\"valid\":"+String(valid?"true":"false")+
 ",\"age_ms\":"+(s.hasSample?String(uint32_t(now-s.updated)):String("null"))+",\"errors\":"+String(s.errors)+
 ",\"fault\":"+quote(!armed?"hardware_not_armed":s.valid&&!valid?"stale":s.fault)+
 ",\"warning_reached\":"+cmp(warnings[i])+",\"automation_threshold_reached\":"+cmp(limits[i])+"}";
 }
 j+="],\"level_mode\":\"manual\",\"volume_l\":"+num(volume)+",\"level_mm\":null,\"level_sensor\":null,"
 "\"settings_revision\":"+String(revision)+",\"automation_implemented\":false,\"k_states\":{\"K1\":false,\"K2\":false,\"K3\":false,\"K4\":false},"
 "\"wifi_connected\":"+String(WiFi.status()==WL_CONNECTED?"true":"false")+
 ",\"station_ip\":"+quote(WiFi.localIP().toString())+",\"ap_ip\":"+quote(WiFi.softAPIP().toString())+"}";json(200,j);
}
void command(const String &line){
 cJSON *r=cJSON_ParseWithLength(line.c_str(),line.length());auto *c=r?cJSON_GetObjectItemCaseSensitive(r,"cmd"):nullptr;
 if(!cJSON_IsString(c)){Serial0.println("ERR json/cmd");cJSON_Delete(r);return;}String cmd=c->valuestring;
 if(cmd=="wifi"){
 auto *s=cJSON_GetObjectItemCaseSensitive(r,"ssid"),*p=cJSON_GetObjectItemCaseSensitive(r,"password");
 if(cJSON_IsString(s)&&cJSON_IsString(p)&&strlen(s->valuestring)>0&&strlen(s->valuestring)<=32&&strlen(p->valuestring)<=63&&
 (strlen(p->valuestring)==0||strlen(p->valuestring)>=8)&&storage){
 String cfg="{\"ssid\":"+quote(s->valuestring)+",\"password\":"+quote(p->valuestring)+"}";
 if(prefs.putString("wifi",cfg)==cfg.length()){ssid=s->valuestring;password=p->valuestring;WiFi.begin(ssid.c_str(),password.c_str());lastReconnect=millis();Serial0.println("OK wifi saved");}
 else Serial0.println("ERR wifi save");}else Serial0.println("ERR wifi parameters/storage");
 }else if(cmd=="sensors"){
 auto *a=cJSON_GetObjectItemCaseSensitive(r,"address"),*confirm=cJSON_GetObjectItemCaseSensitive(r,"confirm");
 if(cJSON_IsTrue(confirm)&&cJSON_IsNumber(a)&&a->valuedouble>=24&&a->valuedouble<=31&&floor(a->valuedouble)==a->valuedouble){
 bridgeOffline("restart");if(busReady)Wire.end();DS2482_ADDR=a->valueint;
 busReady=Wire.begin(8,7,100000);Wire.setTimeOut(20);armed=busReady;Serial0.println(busReady?"OK sensors armed for this boot":"ERR I2C init");
 }else Serial0.println("ERR confirm wiring and address 24..31");
 }else if(cmd=="sensors_off"){armed=false;bridgeOffline("hardware_not_armed");if(busReady)Wire.end();busReady=false;Serial0.println("OK sensors off");
 }else if(cmd=="status"){Serial0.printf("IP=%s sensors=%d bridge=%d flash=%lu psram=%lu\n",WiFi.localIP().toString().c_str(),armed,bridgeFound,(unsigned long)ESP.getFlashChipSize(),(unsigned long)ESP.getPsramSize());
 }else Serial0.println("ERR unknown command");cJSON_Delete(r);
}
void serialPoll(){
 static String line;static bool overflow=false;
 while(Serial0.available()){char c=Serial0.read();if(c=='\n'){
 if(!overflow&&line.length())command(line);else if(overflow)Serial0.println("ERR command too long");line="";overflow=false;
 }else if(c!='\r'){if(line.length()<512&&!overflow)line+=c;else{line="";overflow=true;}}}
}
extern "C" void app_main(){
 initArduino();Serial0.begin(115200);streamId=randomHex();token=randomHex();
 storage=nvs_flash_init()==ESP_OK&&prefs.begin("busa-lab",false); // Never erase existing NVS.
 if(storage){
 String saved=prefs.getString("settings","");auto *r=cJSON_Parse(saved.c_str());float w[4],a[4],v;
 if(parseSettings(r,w,a,v)){for(int i=0;i<4;i++){warnings[i]=w[i];limits[i]=a[i];}volume=v;
 auto *rev=cJSON_GetObjectItemCaseSensitive(r,"revision");if(cJSON_IsNumber(rev)&&rev->valuedouble>=0&&rev->valuedouble<=UINT32_MAX)revision=(uint32_t)rev->valuedouble;}cJSON_Delete(r);
 saved=prefs.getString("wifi","");r=cJSON_Parse(saved.c_str());auto *s=r?cJSON_GetObjectItemCaseSensitive(r,"ssid"):nullptr,*p=r?cJSON_GetObjectItemCaseSensitive(r,"password"):nullptr;
 if(cJSON_IsString(s)&&cJSON_IsString(p)){ssid=s->valuestring;password=p->valuestring;}cJSON_Delete(r);
 }
 WiFi.persistent(false);WiFi.mode(WIFI_AP_STA);String apPassword=randomHex().substring(0,16);
 bool ap=WiFi.softAP("BUSA-1M-001",apPassword.c_str());
 Serial0.println("\nBUSA v0.7.2-lab.1 NO ACTUATOR OUTPUTS");
 if(ap){Serial0.print("AP password (this boot): ");Serial0.println(apPassword);Serial0.print("URL http://");Serial0.println(WiFi.softAPIP());}
 else Serial0.println("ERR AP start");
 Serial0.print("Settings token (this boot): ");Serial0.println(token);
 Serial0.println("Sensors DISARMED. Confirm electrical checks before sensors command.");
 if(ssid.length())WiFi.begin(ssid.c_str(),password.c_str());
 const char *headers[]={"X-BUSA-Token","Content-Type"};web.collectHeaders(headers,2);
 web.on("/",HTTP_GET,[]{web.sendHeader("Cache-Control","no-cache");web.send_P(200,"text/html; charset=utf-8",LAB_PAGE);});
 web.on("/index.html",HTTP_GET,[]{web.send_P(200,"text/html; charset=utf-8",LAB_PAGE);});
 web.on("/api/capabilities",HTTP_GET,capabilities);web.on("/api/telemetry",HTTP_GET,telemetry);
 web.on("/api/keys",HTTP_GET,[]{json(200,"{\"stream_id\":"+quote(streamId)+",\"events\":[]}");});
 web.on("/api/settings",HTTP_GET,[]{json(200,settings());});web.on("/api/settings",HTTP_POST,saveSettings);
 web.onNotFound([]{json(404,"{\"error\":\"not_found\"}");});web.begin();
 uint32_t lastLog=0;
 while(true){
 serialPoll();web.handleClient();
 if(armed){if(!bridgeFound&&uint32_t(millis()-lastBridgeAttempt)>=2000){lastBridgeAttempt=millis();initBridge();}pollSensors();}
 if(ssid.length()&&WiFi.status()!=WL_CONNECTED&&uint32_t(millis()-lastReconnect)>30000){lastReconnect=millis();WiFi.disconnect(false,false);WiFi.begin(ssid.c_str(),password.c_str());}
 if(uint32_t(millis()-lastLog)>10000){lastLog=millis();Serial0.printf("STA=%s IP=%s bridge=%d\n",WiFi.status()==WL_CONNECTED?"connected":"offline",WiFi.localIP().toString().c_str(),bridgeFound);}
 delay(2);
 }
}

