#include "../firmware/BUSA_Display_N4R8/BUSA_GraphView.h"
#include <cassert>
#include <cstring>
#include <cstdio>
using namespace busa_graph;
int main(){
    JsonDocument d;d["ok"]=true;d["minutes"]=5;d["collection_sensor"]=false;d["cooling_calibrated"]=false;
    auto ts=d["temperature"].to<JsonArray>();for(int k=0;k<4;++k){auto a=ts.add<JsonArray>();for(int i=0;i<150;++i){if(i==0)a.add(nullptr);else a.add(20+k);}}
    auto c=d["collection_ml_min"].to<JsonArray>();auto w=d["cooling_lph"].to<JsonArray>();
    for(int i=0;i<150;++i){c.add(nullptr);w.add(i==0?0.0:35.5);}
    auto j=d.as<JsonObjectConst>();Scale s;
    assert(analyze(j,0,5,s)&&s.points==596&&s.high==125&&spec(0).series==4);
    assert(samples(j,0,2)[1].as<int>()==22);
    assert(analyze(j,1,5,s)&&s.points==0&&s.high==10&&spec(1).series==1);
    assert(!strcmp(state(j,1,s),"Нет импульсов отбора")&&samples(j,1,1).isNull());
    assert(analyze(j,2,5,s)&&s.points==150&&s.high==40&&samples(j,2,0)[1].as<double>()==35.5);
    assert(!strcmp(state(j,2,s),"Расчётно · без калибровки"));
    assert(coordinate(35.5,s.high)==888&&coordinate(0,s.high)==0&&coordinate(-1,s.high)==0);
    d["collection_sensor"]=true;c[1]=6400.0;
    assert(analyze(j,1,5,s)&&s.points==1&&s.high==6400&&coordinate(6400,s.high)==1000);
    assert(!analyze(j,1,10,s));c.remove(0);assert(!analyze(j,1,5,s));
    d["ok"]=false;assert(!analyze(j,0,5,s));
    std::puts("PASS real JSON graph mapping, null samples, no sensor, zero flow, scaling and stale periods");
}
