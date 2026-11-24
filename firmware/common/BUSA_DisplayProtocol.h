#pragma once
#include "BUSA_UartFrame.h"
namespace busa_uart {
// Versioned link messages. Read-only snapshot requests are idempotent.
constexpr uint8_t requestStatus=1, status=2;
constexpr uint8_t displayDiagnostics=3, diagnosticsAck=4;
constexpr size_t maxPayload=12288;
constexpr size_t maxRaw=headerSize+maxPayload+crcSize;
constexpr size_t maxWire=2*maxRaw+2;
// Approved physical wiring.
constexpr int mainTx=8, mainRx=9, displayTx=17, displayRx=18;
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
constexpr size_t diagnosticBytes=metricCount*4;
inline bool decodeMetrics(const Frame& f,uint32_t* out) {
    if(f.kind!=displayDiagnostics||f.size!=diagnosticBytes||!out)return false;
    for(size_t i=0;i<metricCount;++i)out[i]=read32(f.data+4*i);
    return true;
}
inline uint64_t combineBytes(uint32_t lo,uint32_t hi) {return uint64_t(lo)|(uint64_t(hi)<<32);}
}
