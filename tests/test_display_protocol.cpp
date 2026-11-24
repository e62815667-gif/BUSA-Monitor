#include "../firmware/common/src/BUSA_DisplayProtocol.h"
#include <assert.h>
#include <string.h>
using namespace busa_uart;
int main() {
    uint8_t packet[diagnosticBytes]={};
    uint32_t decoded[metricCount]={};
    char firmware[versionBytes]={};
    for(size_t i=0;i<metricCount;++i)write32(packet+4*i,uint32_t(i));
    memcpy(packet+metricBytes,"BUSA-DISPLAY-1",15);
    Frame f{displayDiagnostics,1,packet,sizeof(packet)};
    assert(decodeMetrics(f,decoded,firmware));
    assert(strcmp(firmware,"BUSA-DISPLAY-1")==0);
    for(size_t i=0;i<metricCount;++i)assert(decoded[i]==i);
    f.size--;assert(!decodeMetrics(f,decoded,firmware));f.size++;
    packet[metricBytes+4]='"';assert(!decodeMetrics(f,decoded,firmware));
    assert(strcmp(firmware,"BUSA-DISPLAY-1")==0);
    packet[metricBytes+4]='-';
    packet[diagnosticBytes-1]='x';assert(!decodeMetrics(f,decoded,firmware));
    assert(combineBytes(0x12345678,1)==0x112345678ULL);
}
