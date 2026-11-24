#include <cassert>
#include <cstdint>
#include <vector>
#include "../firmware/BUSA_Display_N4R8/BUSA_GuitionFactoryInit.h"
struct Event {int op,a,b;};
struct Bus {
 std::vector<Event> events;
 void sendCommand(uint8_t c){events.push_back({1,c,0});}
 void beginWrite(){events.push_back({0,0,0});}
 void writeC8D8(uint8_t a,uint8_t b){events.push_back({7,a,b});}
 void endWrite(){events.push_back({11,0,0});}
};
int main(){
 Bus bus;
 busa_guition::applyFactoryInit(bus,[&](uint8_t ms){bus.events.push_back({12,ms,0});});
 const auto& e=bus.events;
 assert(e.size()==busa_guition::factoryInitCount+2);
 assert(e[0].op==1 && e[0].a==1);
 assert(e[1].op==12 && e[1].a>=120);
 for(size_t i=0;i<busa_guition::factoryInitCount;++i){
   const auto& step=busa_guition::factoryInit[i];const auto& actual=e[i+2];
   assert(step.op==actual.op);
   if(step.op==12){assert(actual.a>=(step.a==40?120:200));}
   else {assert(step.a==actual.a && step.b==actual.b);}
 }
}
