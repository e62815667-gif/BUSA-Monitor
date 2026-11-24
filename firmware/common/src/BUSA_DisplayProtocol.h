#pragma once
#include "BUSA_UartFrame.h"
#include "BUSA_DisplayApi.h"
namespace busa_uart {
// Versioned link messages. Read-only snapshot requests are idempotent.
constexpr uint8_t requestStatus=1, status=2;
constexpr uint8_t displayDiagnostics=3, diagnosticsAck=4;
constexpr size_t maxPayload=12288;
constexpr size_t maxRaw=headerSize+maxPayload+crcSize;
constexpr size_t maxWire=2*maxRaw+2;
// P1: yellow TXD -> main RX GPIO8; blue RXD <- main TX GPIO9.
constexpr int mainTx=9, mainRx=8, displayTx=43, displayRx=44;
// Shared software default, not a hardware pin assignment.
constexpr uint32_t baud=115200;
constexpr uint32_t staleMs=5000;
// Diagnostic schema v1, little-endian 32-bit words; unknown=UINT32_MAX.
// No text fields or credentials are accepted from the wire.
enum Metric : size_t {
    uptime, resetReason, cpuMhz, freeRam, minRam, totalPsram, freePsram,
    maxLoopMs, uartRx, uartTx, uartCrc, uartLost, uartTimeout,
    lcdInit, renderLast, renderMax, fpsMilli, touchInit, touchErrors,
    sdStatus, sdTotalLow, sdTotalHigh, sdFreeLow, sdFreeHigh,
    sdReadErrors, sdWriteErrors, assetErrors, wifiStatus, lastError, metricCount
};
constexpr size_t metricBytes=metricCount*4;
constexpr size_t versionBytes=40;
constexpr size_t diagnosticBytes=metricBytes+versionBytes;
inline bool decodeMetrics(const Frame& f,uint32_t* out,char* firmware) {
    if(f.kind!=displayDiagnostics||f.size!=diagnosticBytes||!f.data||!out||!firmware)return false;
    const uint8_t* v=f.data+metricBytes;
    bool ended=false;
    if(v[0]!='B'||v[1]!='U'||v[2]!='S'||v[3]!='A')return false;
    for(size_t i=0;i<versionBytes;++i){
        const uint8_t c=v[i];
        if(c==0){ended=true;continue;}
        if(ended||!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.'))return false;
    }
    if(!ended)return false;
    for(size_t i=0;i<metricCount;++i)out[i]=read32(f.data+4*i);
    for(size_t i=0;i<versionBytes;++i)firmware[i]=char(v[i]);
    return true;
}
inline uint64_t combineBytes(uint32_t lo,uint32_t hi) {return uint64_t(lo)|(uint64_t(hi)<<32);}
}
