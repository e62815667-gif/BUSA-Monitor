#include "../firmware/BUSA_Display_N4R8/BUSA_DirtyRows.h"
#include <cassert>
int main(){
    busa_guition::DirtyRows rows(272); int16_t start=0,count=0;
    rows.note(0,9); rows.note(100,109); rows.note(105,115); rows.note(270,290);
    assert(rows.takeNext(start,count)&&start==0&&count==10);
    assert(rows.takeNext(start,count)&&start==100&&count==16);
    assert(rows.takeNext(start,count)&&start==270&&count==2);
    assert(!rows.takeNext(start,count));
    rows.note(20,22);rows.note(21,24);assert(rows.takeNext(start,count)&&start==20&&count==5);
    assert(!rows.takeNext(start,count));
    rows.note(-10,300);assert(rows.takeNext(start,count)&&start==0&&count==272);
    assert(!rows.takeNext(start,count));
}
