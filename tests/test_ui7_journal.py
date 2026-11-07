"""Production journal ring: ordering, bounded capacity and safe UTF-8 truncation."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];fw=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
code=r'''
#include <cassert>
#include <string>
#include "BUSA_WebJournal.h"
int main(){
 BusaWebJournal::Ring r;r.add(1,0,0,"before storage");assert(r.size()==0);
 BusaWebJournal::Entry rows[3];r.begin(rows,3);
 for(int i=0;i<10;i++){std::string s="event "+std::to_string(i);r.add(i,100+i,i==9?2:0,s.c_str());}
 assert(r.size()==3);assert(r.latest(0)->seq==10&&r.latest(0)->kind==2);assert(r.latest(2)->seq==8);assert(!r.latest(3));
 assert(std::string(r.latest(0)->text)=="event 9");
 std::string unicode;for(int i=0;i<100;i++)unicode+="Я";r.add(50,200,0,unicode.c_str());
 std::string truncated=r.latest(0)->text;assert(truncated.size()<=111&&truncated.size()%2==0);
 for(unsigned i=0;i<truncated.size();i+=2)assert((unsigned char)truncated[i]==0xd0&&(unsigned char)truncated[i+1]==0xaf);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'journal.cpp';p.write_text(code);exe=Path(d)/'journal';subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(fw),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
main=(fw/'BUSA_Main.h').read_text();a=main.index('static void busaWebEventsPoll(');b=main.index('bool busaSoundEnabled(uint8_t i){',a);observer=main[a:b]
for forbidden in ['temperature[','sampleTemperatures(', 'digitalWrite(', 'prefs.put','soundOutputsOff(', 'WiFi.begin(', 'ESP.restart(']:assert forbidden not in observer,forbidden
print('PASS production device journal: reverse order/capacity, sequence IDs, UTF-8 bounds; observer contains no temperature reads, GPIO/NVS/radio writes')
