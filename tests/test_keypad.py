"""Execute shipping scanKeyPair against electrical connectivity and I/O fault models."""
from pathlib import Path
import subprocess,tempfile
source=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
scanner=source[source.index('int scanKeyPair(){'):source.index('String pairName(')]
stub=r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>
#include <utility>
#include <cstring>
using std::min;using std::max;
bool pcfDetected=true,keypadReady=false;
uint32_t pcfLastScanAt=0,pcfError6Total=0,pcfIdleLowTotal=0,pcfValidPairTotal=0;
uint8_t pcfScanError=0,pcfPairCount=0,keyPairs[20]={};int8_t pcfConflictDrive=-1;
uint16_t pcfObservedLow=0,pcfConflictMask=0,pcfIdleMask=65535;
uint16_t driven=65535,idleLow=0;int reads=0,writes=0,failReadAt=-1,failWriteAt=-1;
bool failRelease=false;
std::vector<std::pair<int,int>> edges;
uint32_t millis(){return 1;}void delayMicroseconds(unsigned){}
bool pcfWrite(uint16_t value){++writes;if(writes==failWriteAt||(failRelease&&value==65535&&writes>1))return false;driven=value;return true;}
bool pcfRead(uint16_t& value){
 ++reads;if(reads==failReadAt)return false;
 uint16_t low=(uint16_t)~driven|idleLow;
 for(int n=0;n<16;n++)for(auto edge:edges){if(low&(1u<<edge.first))low|=1u<<edge.second;if(low&(1u<<edge.second))low|=1u<<edge.first;}
 value=(uint16_t)~low;return true;
}
void reset(){pcfDetected=true;keypadReady=false;driven=65535;idleLow=0;reads=writes=0;failReadAt=failWriteAt=-1;failRelease=false;edges.clear();}
'''
tests=r'''
int main(){
 reset();assert(scanKeyPair()==-1&&pcfScanError==0&&driven==65535);
 reset();edges={{3,9}};assert(scanKeyPair()==((3<<4)|9)&&pcfScanError==0&&driven==65535);
 // Field report conflict c0: two lines P6/P7 low when P0 is driven. Do not invent a key.
 reset();edges={{0,6},{0,7}};assert(scanKeyPair()==-2&&pcfScanError==6);
 assert(pcfConflictDrive==0&&pcfConflictMask==0xc0&&driven==65535);
 assert(std::strcmp(keyScanErrorText(6),"AMBIGUOUS_MATRIX")==0);
 reset();edges={{0,6},{0,7}};failReadAt=3;assert(scanKeyPair()==-2&&pcfScanError==5&&driven==65535);
 reset();idleLow=0xc0;assert(scanKeyPair()==-2&&pcfScanError==9&&pcfConflictMask==0xc0);
 reset();failWriteAt=1;assert(scanKeyPair()==-2&&pcfScanError==2&&writes==2);
 reset();failReadAt=1;assert(scanKeyPair()==-2&&pcfScanError==3&&writes==2);
 reset();failWriteAt=2;assert(scanKeyPair()==-2&&pcfScanError==4&&driven==65535);
 reset();failReadAt=2;assert(scanKeyPair()==-2&&pcfScanError==5&&driven==65535);
 reset();edges={{0,1},{2,3}};assert(scanKeyPair()==-2&&pcfScanError==7);
 reset();edges={{0,1}};failRelease=true;assert(scanKeyPair()==-2&&pcfScanError==8);
 reset();pcfDetected=false;assert(scanKeyPair()==-1&&pcfScanError==1&&writes==0);
 // Learned 5x4 mapping, arbitrary physical rows/columns: no fixed wire order.
 reset();keypadReady=true;int k=0;for(int row:{1,3,5,7,9})for(int col:{0,2,4,6})keyPairs[k++]=(min(row,col)<<4)|max(row,col);
 edges={{5,4}};assert(scanKeyPair()==((4<<4)|5)&&pcfScanError==0&&driven==65535);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'keypad.cpp';p.write_text(stub+scanner+tests);exe=Path(d)/'keypad'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS real PCF8575 scanner: idle/single/multiple contacts, c0 conflict, settling-read failure, all I/O failures, release, missing chip and arbitrary learned 5x4 wiring')
