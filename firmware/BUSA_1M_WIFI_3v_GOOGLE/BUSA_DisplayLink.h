#pragma once
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <driver/uart.h>
#include <BUSA_UartTxMailbox.h>
#include <BUSA_DisplayProtocol.h>
#include "BUSA_DisplayOtaMain.h"

String statusJSON();
static void busaDispatchDisplayApi(const busa_uart::ApiRequest&,int&,String&);
namespace busa_display_link {
using namespace busa_uart;
static uint8_t rx[headerSize+apiMaxRequest+crcSize];
static Decoder decoder(rx,sizeof(rx));
static uint8_t* raw=nullptr;
static uint8_t* wire=nullptr;
static uint32_t lastDiagnostic=0, tx=0, busyDrops=0;
static TxMailbox txMailbox;
static TaskHandle_t txTask=nullptr;
static uint32_t txWorstMs=0,txErrors=0,jsonWorstMs=0,encodeWorstMs=0,apiWorstMs=0;
static void txWorker(void*){
    for(;;){
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        const size_t n=txMailbox.take();
        if(!n)continue;
        const uint32_t started=millis();
        // Only this task writes UART1. No Arduino Serial mutex is held.
        const int written=uart_write_bytes(UART_NUM_1,wire,n);
        txMailbox.complete(written,millis()-started);
    }
}
static void drainTx(){
    bool success=false;uint32_t elapsed=0;
    if(!txMailbox.reap(success,elapsed))return;
    if(elapsed>txWorstMs)txWorstMs=elapsed;
    if(success)++tx;else ++txErrors;
}
static uint32_t metrics[metricCount]={};
static char displayFirmware[versionBytes]={};
static bool ready=false, haveDiagnostic=false;
static void begin(){
    raw=static_cast<uint8_t*>(heap_caps_malloc(maxRaw,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    wire=static_cast<uint8_t*>(heap_caps_malloc(maxWire,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    if(!raw||!wire){free(raw);free(wire);raw=nullptr;wire=nullptr;return;}
    if(!Serial1.setRxBufferSize(4096)||Serial1.setTxBufferSize(mainTxBuffer)!=mainTxBuffer){free(raw);free(wire);raw=nullptr;wire=nullptr;return;}
    Serial1.begin(baud,SERIAL_8N1,mainRx,mainTx);
    if(!uart_is_driver_installed(UART_NUM_1)||xTaskCreatePinnedToCore(txWorker,"busa-uart-tx",3072,nullptr,2,&txTask,1)!=pdPASS){Serial1.end();free(raw);free(wire);raw=nullptr;wire=nullptr;return;}
    ready=true;
}
static bool enqueue(uint8_t type,uint32_t seq,const uint8_t* data,size_t length){
    drainTx();
    if(txMailbox.busy()){++busyDrops;return false;}
    const uint32_t started=millis();
    const size_t size=encode(type,seq,data,length,raw,maxRaw,wire,maxWire);
    const uint32_t elapsed=millis()-started;if(elapsed>encodeWorstMs)encodeWorstMs=elapsed;
    if(!txMailbox.publish(size))return false;
    xTaskNotifyGive(txTask);return true;
}
static void handleApi(const Frame& f){
    drainTx();
    if(txMailbox.busy()){++busyDrops;return;}
    const uint32_t started=millis();
    ApiRequest req{};
    int code=400;String body="Invalid request";
    if(parseApiRequest(f.data,f.size,req)){
        if(!haveDiagnostic||uint32_t(millis()-lastDiagnostic)>staleMs){code=409;body="Display session not ready";}
        else busaDispatchDisplayApi(req,code,body);
    }
    if(body.length()>maxPayload-2){code=413;body="Response exceeds UART limit";}
    uint8_t* payload=raw+headerSize;
    payload[0]=uint8_t(code);payload[1]=uint8_t(code>>8);
    memcpy(payload+2,body.c_str(),body.length());
    enqueue(apiResponse,f.seq,payload,body.length()+2);
    const uint32_t elapsed=millis()-started;if(elapsed>apiWorstMs)apiWorstMs=elapsed;
    memset(rx,0,sizeof(rx));
}
static bool lcdReady(){return ready&&haveDiagnostic&&uint32_t(millis()-lastDiagnostic)<=staleMs&&metrics[lcdInit]==1;}
static bool supportsBacklightRequest(){return lcdReady()&&strstr(displayFirmware,"-HW1")!=nullptr;}
static bool otaCapable(){return ready&&haveDiagnostic&&uint32_t(millis()-lastDiagnostic)<=staleMs&&strncmp(displayFirmware,busa_ota::displayFamily,strlen(busa_ota::displayFamily))==0;}
static void poll(){
    busa_display_ota::tick();
    if(!ready)return;
    // Finish queuing an existing frame before accepting the next request.
    drainTx();
    Frame f{};
    // One bounded, zero-wait IDF read; no per-byte Arduino wait or TX mutex.
    uint8_t bytes[256];
    const int count=uart_read_bytes(UART_NUM_1,bytes,sizeof(bytes),0);
    for(int i=0;i<count;++i){
        if(!decoder.push(bytes[i],f))continue;
        if(busa_display_ota::handle(f,enqueue))continue;
        if(f.kind==requestStatus&&f.size==0){
            drainTx();
            if(txMailbox.busy()){++busyDrops;continue;}
            const uint32_t started=millis();
            String snapshot=statusJSON();
            const uint32_t elapsed=millis()-started;if(elapsed>jsonWorstMs)jsonWorstMs=elapsed;
            if(snapshot.length()>0&&snapshot.length()<=maxPayload)
                enqueue(status,f.seq,reinterpret_cast<const uint8_t*>(snapshot.c_str()),snapshot.length());
        }else if(f.kind==apiRequest){
            handleApi(f);
        }else if(decodeMetrics(f,metrics,displayFirmware)){
            lastDiagnostic=millis();haveDiagnostic=true;
            enqueue(diagnosticsAck,f.seq,nullptr,0);
        }
    }
    drainTx();
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
    s+=",\"main_uart_tx_worker_max_ms\":"+String(txWorstMs);
    s+=",\"main_uart_tx_write_errors\":"+String(txErrors);
    s+=",\"main_uart_tx_pending\":"+String(txMailbox.busy()?"true":"false");
    s+=",\"main_uart_json_max_ms\":"+String(jsonWorstMs);
    s+=",\"main_uart_encode_max_ms\":"+String(encodeWorstMs);
    s+=",\"main_uart_api_max_ms\":"+String(apiWorstMs);
    const char* names[]={"uptime_ms","reset_reason","cpu_mhz","free_ram","min_ram","psram_total","psram_free","max_loop_ms","uart_rx","uart_tx","uart_crc","uart_lost","uart_timeout","lcd_init","render_last_ms","render_max_ms","fps_milli","touch_init","touch_errors","sd_status"};
    for(size_t i=0;i<sizeof(names)/sizeof(names[0]);++i){s+=",\"";s+=names[i];s+="\":";s+=metricJson(Metric(i));}
    s+=",\"sd_total_bytes\":"+bytesJson(sdTotalLow,sdTotalHigh);
    s+=",\"sd_free_bytes\":"+bytesJson(sdFreeLow,sdFreeHigh);
    const char* extra[]={"sd_read_errors","sd_write_errors","asset_errors","wifi_status","last_error"};
    for(size_t i=0;i<sizeof(extra)/sizeof(extra[0]);++i){s+=",\"";s+=extra[i];s+="\":";s+=metricJson(Metric(sdReadErrors+i));}
    s+="}";return s;
}
} // namespace busa_display_link
