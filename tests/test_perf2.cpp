#include "../firmware/BUSA_Display_N4R8/BUSA_DirtyRows.h"
#include "../firmware/common/src/BUSA_LoopProfile.h"
#include <cassert>
#include <cstring>
int main(){
    using busa_guition::DirtyRows;
    DirtyRows r(272);int16_t y=0,n=0;
    assert(!r.take(y,n)&&n==0);
    r.note(30,53);r.note(12,15);r.note(70,70);
    assert(r.take(y,n)&&y==12&&n==59);
    assert(!r.take(y,n));
    r.note(-8,1);r.note(271,300);
    assert(r.take(y,n)&&y==0&&n==272);
    r.note(9,4);r.note(-8,-1);r.note(272,300);
    assert(!r.take(y,n));
    r.note(100,100);assert(r.take(y,n)&&y==100&&n==1);
    using namespace busa_perf;
    LoopProfile p;
    p.begin(100);p.mark(early,101);p.mark(http,105);p.mark(uart,160);
    p.mark(snapshot,162);p.mark(uart,205);p.mark(tft,215);p.finish(115);
    assert(p.worst(uart)==98&&p.worst(http)==4&&p.slow(uart)==1&&p.last()==uart);
    p.begin(300);p.mark(http,390);p.mark(tft,400);p.finish(100);
    assert(p.worst(http)==90&&p.slow(http)==0&&p.slow(uart)==1);
    p.begin(UINT32_MAX-10);p.mark(http,100);p.finish(111);
    assert(p.worst(http)==111&&p.slow(http)==1&&p.last()==http);
    assert(std::strcmp(LoopProfile::name(http),"http")==0);
}
