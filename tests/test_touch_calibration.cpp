#include "../firmware/BUSA_Display_N4R8/BUSA_TouchCalibration.h"
#include <cassert>
using namespace busa_touch;
int main(){
 // Independent timing expectations: no cached samples, no catch-up burst, wrap-safe.
 uint32_t last=100;
 assert(!sampleDue(102,last)&&last==100);
 assert(!sampleDue(109,last)&&last==100);
 assert(sampleDue(110,last)&&last==110);
 assert(!sampleDue(110,last));
 assert(sampleDue(151,last)&&last==151);
 assert(!sampleDue(152,last));
 last=UINT32_MAX-4;
 assert(!sampleDue(4,last));
 assert(sampleDue(5,last)&&last==5);
 Point raw[3]={{300,3700},{3700,3700},{300,300}},screen[3]={{20,20},{460,20},{20,252}};Transform t{};
 assert(fit(raw,screen,t));Point p=apply(t,{2000,2000});assert(p.x==240&&p.y==136);
 Point deg[3]={{1,1},{2,2},{3,3}};assert(!fit(deg,screen,t));
 Point swap[3]={{3700,300},{3700,3700},{300,300}};assert(fit(swap,screen,t));p=apply(t,{2000,2000});assert(p.x==240&&p.y==136);
}
