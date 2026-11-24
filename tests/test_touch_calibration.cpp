#include "../firmware/BUSA_Display_N4R8/BUSA_TouchCalibration.h"
#include <cassert>
using namespace busa_touch;
int main(){
 Point raw[3]={{300,3700},{3700,3700},{300,300}},screen[3]={{20,20},{460,20},{20,252}};Transform t{};
 assert(fit(raw,screen,t));Point p=apply(t,{2000,2000});assert(p.x==240&&p.y==136);
 Point deg[3]={{1,1},{2,2},{3,3}};assert(!fit(deg,screen,t));
 Point swap[3]={{3700,300},{3700,3700},{300,300}};assert(fit(swap,screen,t));p=apply(t,{2000,2000});assert(p.x==240&&p.y==136);
}
