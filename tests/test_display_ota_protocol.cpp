#include "BUSA_DisplayOtaProtocol.h"
#include <cassert>
#include <cstring>
using namespace busa_ota;
int main(){
 assert(validSize(60000)&&validSize(slotSize)&&!validSize(59999)&&!validSize(slotSize+1));
 uint8_t image[36]={};image[0]=0xe9;image[12]=9;image[32]=0x32;image[33]=0x54;image[34]=0xcd;image[35]=0xab;
 assert(validHeader(image,sizeof(image)));image[12]=0;assert(!validHeader(image,sizeof(image)));assert(!validHeader(image,35));
 Marker mainMarker("14v9-N16R8-");const char* a="foo14v9-N";const char* b="16R8-bar";
 mainMarker.feed((const uint8_t*)a,strlen(a));assert(!mainMarker.found());mainMarker.feed((const uint8_t*)b,strlen(b));assert(mainMarker.found());mainMarker.reset();assert(!mainMarker.found());
 Marker displayMarker(displayFamily);const char* c="BUSA-DISPLAY-N4R8-UART2";displayMarker.feed((const uint8_t*)c,strlen(c));assert(!displayMarker.found());
 displayMarker.feed((const uint8_t*)displayFamily,strlen(displayFamily));assert(displayMarker.found());
 assert(chunkLength(60000,0)==1024&&chunkLength(60000,59999)==1&&chunkLength(60000,60000)==0&&chunkLength(60000,60001)==0);
 assert(chunkLength(0,0)==0);assert(chunkLength(slotSize+1,0)==0);
}
