#pragma once
#include <Arduino.h>
#include <new>
#include <utility>
#include <cstring>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <BUSA_DisplayProtocol.h>

namespace busa_guition {
using namespace busa_uart;
class DisplaySession {
public:
    bool begin(){
        auto* storage=static_cast<uint8_t*>(heap_caps_malloc(maxRaw,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
        if(!storage)return false;
        decoder_=new(std::nothrow) Decoder(storage,maxRaw);
        if(!decoder_){free(storage);return false;}
        rxStorage_=storage;
        if(!Serial1.setRxBufferSize(8192)){
            delete decoder_;decoder_=nullptr;free(rxStorage_);rxStorage_=nullptr;return false;
        }
        Serial1.begin(baud,SERIAL_8N1,displayRx,displayTx);
        return true;
    }
    bool connected()const{return haveData_&&uint32_t(millis()-lastData_)<=staleMs;}
    const JsonDocument& snapshot()const{return model_;}
    uint32_t snapshotRevision()const{return revision_;}
    void poll(const uint32_t* boardMetrics,const char* firmware,uint32_t loopMax){
        if(!decoder_)return;
        Frame f{};
        // Drain bytes in bounded slices; caller must service poll between
        // rendering stripes rather than blocking on a whole screen redraw.
        for(unsigned i=0;i<1024&&Serial1.available();++i){
            const int c=Serial1.read();
            if(c>=0&&decoder_->push(uint8_t(c),f))handle(f);
        }
        const uint32_t now=millis();
        if(pending_&&uint32_t(now-requestAt_)>=3500){
            ++timeouts_;++lost_;pending_=0;wireSize_=wireAt_=0;
        }
        if(!pending_){
            if(!diagnosticSent_||uint32_t(now-lastDiag_)>=1500)
                sendDiagnostics(boardMetrics,firmware,loopMax,now);
            else if(!requestSent_||uint32_t(now-lastRequest_)>=250){
                lastRequest_=now;requestSent_=true;
                start(requestStatus,nullptr,0,status,now);
            }
        }
        if(wireAt_<wireSize_){
            const int avail=Serial1.availableForWrite();
            if(avail>0){
                size_t n=wireSize_-wireAt_;
                if(n>size_t(avail))n=size_t(avail);
                wireAt_+=Serial1.write(wire_+wireAt_,n);
                if(wireAt_==wireSize_)++tx_;
            }
        }
    }
private:
    void start(uint8_t kind,const uint8_t* data,size_t size,uint8_t expected,uint32_t now){
        expectedSeq_=++sequence_;wireAt_=0;
        wireSize_=encode(kind,expectedSeq_,data,size,raw_,sizeof(raw_),wire_,sizeof(wire_));
        if(!wireSize_)return;
        requestAt_=now;pending_=expected;
    }
    void handle(const Frame& f){
        if(!pending_||f.seq!=expectedSeq_||f.kind!=pending_){++lost_;return;}
        if(f.kind==diagnosticsAck&&f.size==0){pending_=0;return;}
        if(f.kind!=status)return;
        JsonDocument candidate;
        const DeserializationError error=deserializeJson(candidate,reinterpret_cast<const char*>(f.data),f.size);
        const char* device=candidate["device"]|"";
        if(error||strcmp(device,"BUSA-1M-001")!=0||!candidate["temperatures"].is<JsonArray>()||
           candidate["temperatures"].size()!=4){
            ++lost_;pending_=0;return;
        }
        model_=std::move(candidate);
        if(model_.overflowed()){++lost_;haveData_=false;pending_=0;return;}
        haveData_=true;lastData_=millis();++revision_;pending_=0;
    }
    void sendDiagnostics(const uint32_t* board,const char* firmware,uint32_t loopMax,uint32_t now){
        if(!board||!firmware||strlen(firmware)>=versionBytes)return;
        uint32_t m[metricCount];
        for(size_t i=0;i<metricCount;++i)m[i]=board[i];
        m[uptime]=now;m[resetReason]=uint32_t(esp_reset_reason());
        m[cpuMhz]=ESP.getCpuFreqMHz();m[freeRam]=ESP.getFreeHeap();m[minRam]=ESP.getMinFreeHeap();
        m[totalPsram]=ESP.getPsramSize();m[freePsram]=ESP.getFreePsram();
        m[maxLoopMs]=loopMax;m[uartRx]=decoder_->counters.rx;m[uartTx]=tx_;
        m[uartCrc]=decoder_->counters.crcErrors;m[uartLost]=lost_+decoder_->counters.overflow+decoder_->counters.formatErrors;
        m[uartTimeout]=timeouts_;
        uint8_t packet[diagnosticBytes]={};
        for(size_t i=0;i<metricCount;++i)write32(packet+4*i,m[i]);
        memcpy(packet+metricBytes,firmware,strlen(firmware));
        start(displayDiagnostics,packet,sizeof(packet),diagnosticsAck,now);
        diagnosticSent_=true;lastDiag_=now;
    }
    Decoder* decoder_=nullptr;
    uint8_t* rxStorage_=nullptr;
    uint8_t raw_[headerSize+diagnosticBytes+crcSize]={};
    uint8_t wire_[2*sizeof(raw_)+2]={};
    size_t wireSize_=0,wireAt_=0;
    uint8_t pending_=0;
    uint32_t expectedSeq_=0,sequence_=0,requestAt_=0,tx_=0,lost_=0,timeouts_=0;
    uint32_t lastData_=0,revision_=0,lastDiag_=0,lastRequest_=0;
    bool haveData_=false,diagnosticSent_=false,requestSent_=false;
    JsonDocument model_;
};
} // namespace busa_guition
