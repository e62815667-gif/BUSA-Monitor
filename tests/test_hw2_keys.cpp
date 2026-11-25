#include "../firmware/common/src/BUSA_KeyEvents.h"
#include "../firmware/common/src/BUSA_DisplayApi.h"
#include <assert.h>
using namespace busa_keys;
int main(){
 Ring r;r.begin(77);r.push(2,1);r.push(3,1);r.push(1,1);assert(r.size()==0);
 Cursor c;unsigned calls=0;auto take=[&](const Event& e){++calls;assert(valid(e.code));};
 assert(c.consume(77,0,nullptr,0,take)==Result::baseline);
 r.push(16,100);r.push(19,200);Event batch[]={{1,100,16},{2,0,19}};
 assert(c.consume(r.boot(),r.head(),batch,2,take)==Result::applied&&calls==2);
 assert(c.consume(r.boot(),r.head(),batch,2,take)==Result::applied&&calls==2);
 Event bad[]={{3,10,4},{4,maxAgeMs+1,19}};assert(c.consume(77,4,bad,2,take)==Result::lost&&calls==2);
 Event missing[]={{6,0,19}};assert(c.consume(77,6,missing,1,take)==Result::lost&&calls==2);
 c.reset();assert(c.consume(77,6,missing,1,take)==Result::baseline&&calls==2);
 assert(c.consume(88,100,missing,1,take)==Result::baseline&&calls==2);
 Event fresh[]={{101,0,longZero}};assert(c.consume(88,101,fresh,1,take)==Result::applied&&calls==3);
 for(int i=0;i<25;++i){r.push(4,i);}
 assert(r.size()==capacity&&r.oldest(0).seq==r.head()-capacity+1);
 c.reset();assert(c.consume(88,UINT32_MAX,nullptr,0,take)==Result::baseline);
 Event wrap[]={{0,0,17}};assert(c.consume(88,0,wrap,1,take)==Result::applied&&calls==4);
 assert(busa_uart::validRoute(22)&&busa_uart::validRoute(23)&&busa_uart::validRoute(24)&&!busa_uart::validRoute(25));
}
