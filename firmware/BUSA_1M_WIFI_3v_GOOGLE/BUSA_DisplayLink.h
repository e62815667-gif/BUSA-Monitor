#pragma once
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <BUSA_DisplayProtocol.h>

String statusJSON();
namespace busa_display_link {
using namespace busa_uart;
static uint8_t rx[headerSize+diagnosticBytes+crcSize];
static Decoder decoder(rx,sizeof(rx));
static uint8_t* raw=nullptr;
static uint8_t* wire=nullptr;
static size_t wireSize=0, wireAt=0;
static uint32_t lastDiagnostic=0, tx=0, busyDrops=0;
static uint32_t metrics[metricCount]={};
static char displayFirmware[versionBytes]={};
static bool ready=false, haveDiagnostic=false;
static void begin(){
    raw=static_cast<uint8_t*>(heap_caps_malloc(maxRaw,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    wire=static_cast<uint8_t*>(heap_caps_malloc(maxWire,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    if(!raw||!wire){free(raw);free(wire);raw=nullptr;wire=nullptr;return;}
    Serial1.setRxBufferSize(1024);
    Serial1.begin(baud,SERIAL_8N1,mainRx,mainTx);
    ready=true;
}
static bool enqueue(uint8_t type,uint32_t seq,const uint8_t* data,size_t length){
    if(wireAt<wireSize){++busyDrops;return false;}
    wireAt=0;
    wireSize=encode(type,seq,data,length,raw,maxRaw,wire,maxWire);
    return wireSize!=0;
}
static void poll(){
    if(!ready)return;
    Frame f{};
    // Bounded byte budget: UART traffic must not starve other loop work.
    for(unsigned i=0;i<256&&Serial1.available();++i){
        int c=Serial1.read();
        if(c<0||!decoder.push(uint8_t(c),f))continue;
        if(f.kind==requestStatus&&f.size==0){
            if(wireAt<wireSize){++busyDrops;continue;}
            String snapshot=statusJSON();
            if(snapshot.length()>0&&snapshot.length()<=maxPayload)
                enqueue(status,f.seq,reinterpret_cast<const uint8_t*>(snapshot.c_str()),snapshot.length());
        }else if(decodeMetrics(f,metrics,displayFirmware)){
            lastDiagnostic=millis();haveDiagnostic=true;
            enqueue(diagnosticsAck,f.seq,nullptr,0);
        }
    }
    if(wireAt<wireSize){
        int avail=Serial1.availableForWrite();
        if(avail>0){
            size_t count=wireSize-wireAt;
            if(count>size_t(avail))count=size_t(avail);
            if(count>256)count=256;
            wireAt+=Serial1.write(wire+wireAt,count);
            if(wireAt==wireSize)++tx;
        }
    }
}
static String metricJson(Metric m){
    return !haveDiagnostic||metrics[m]==UINT32_MAX?String("null"):String(metrics[m]);
}
static String bytesJson(Metric lo,Metric hi){
    if(!haveDiagnostic||(metrics[lo]==UINT32_MAX&&metrics[hi]==UINT32_MAX))return "null";
    char buf[24];snprintf(buf,sizeof(buf),"%llu",(unsigned long long)combineBytes(metrics[lo],metrics[hi]));
    return String(buf);
}
static String diagnosticJSON(){
    const uint32_t age=millis()-lastDiagnostic;
    String s="{\"link\":\"";
    s+=!ready?"INIT_FAILED":!haveDiagnostic?"WAITING":age>staleMs?"STALE":"CONNECTED";
    s+="\",\"age_ms\":"+ (haveDiagnostic?String(age):String("null"));
    s+=",\"firmware\":"+(haveDiagnostic?String("\"")+displayFirmware+"\"":String("null"));
    s+=",\"main_uart_rx\":"+String(decoder.counters.rx);
    s+=",\"main_uart_tx\":"+String(tx);
    s+=",\"main_uart_crc_errors\":"+String(decoder.counters.crcErrors);
    s+=",\"main_uart_format_errors\":"+String(decoder.counters.formatErrors);
    s+=",\"main_uart_overflow\":"+String(decoder.counters.overflow);
    s+=",\"main_uart_busy_drops\":"+String(busyDrops);
    const char* names[]={"uptime_ms","reset_reason","cpu_mhz","free_ram","min_ram","psram_total","psram_free","max_loop_ms","uart_rx","uart_tx","uart_crc","uart_lost","uart_timeout","lcd_init","render_last_ms","render_max_ms","fps_milli","touch_init","touch_errors","sd_status"};
    for(size_t i=0;i<sizeof(names)/sizeof(names[0]);++i){s+=",\"";s+=names[i];s+="\":";s+=metricJson(Metric(i));}
    s+=",\"sd_total_bytes\":"+bytesJson(sdTotalLow,sdTotalHigh);
    s+=",\"sd_free_bytes\":"+bytesJson(sdFreeLow,sdFreeHigh);
    const char* extra[]={"sd_read_errors","sd_write_errors","asset_errors","wifi_status","last_error"};
    for(size_t i=0;i<sizeof(extra)/sizeof(extra[0]);++i){s+=",\"";s+=extra[i];s+="\":";s+=metricJson(Metric(sdReadErrors+i));}
    s+="}";return s;
}
} // namespace busa_display_link
