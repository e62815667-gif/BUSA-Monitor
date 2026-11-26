#include "../firmware/BUSA_Display_N4R8/BUSA_FocusOrder.h"
#include <cassert>
using namespace busa_focus;
int main(){
    Rect settings[]={{5,35,235,26},{243,35,235,26},{5,64,235,26},{243,64,235,26},{5,93,235,26},{243,93,235,26},{5,122,235,26},{243,122,235,26},{123,151,235,26}};
    unsigned order[64]={};orderControls(settings,9,order);
    const unsigned expected[]={0,2,4,6,1,3,5,7,8};for(unsigned i=0;i<9;i++)assert(order[i]==expected[i]);
    Rect temps[]={{4,75,110,24},{120,75,110,24},{245,75,110,24},{361,75,110,24},{4,145,110,24},{120,145,110,24},{245,145,110,24},{361,145,110,24},{4,181,470,24}};
    orderControls(temps,9,order);const unsigned t[]={0,4,1,5,2,6,3,7,8};for(unsigned i=0;i<9;i++)assert(order[i]==t[i]);
    Rect calc[]={{5,33,115,25},{123,33,115,25},{241,33,115,25},{359,33,115,25},{5,75,153,23},{163,75,153,23},{321,75,153,23},{5,113,153,23},{163,113,153,23},{5,151,470,25}};
    orderControls(calc,10,order);for(unsigned i=0;i<10;i++)assert(order[i]==i);
    Rect scrambled[]={{245,60,230,24},{4,32,230,24},{4,60,230,24},{245,32,230,24}};
    orderControls(scrambled,4,order);const unsigned z[]={1,2,3,0};for(unsigned i=0;i<4;i++)assert(order[i]==z[i]);
    orderControls(nullptr,0,order);
    const Rect grid[]={{4,30,100,20},{120,30,100,20},{4,60,100,20},{120,60,100,20}};
    assert(neighbor(grid,4,0,Direction::right)==1);
    assert(neighbor(grid,4,1,Direction::left)==0);
    assert(neighbor(grid,4,0,Direction::down)==2);
    assert(neighbor(grid,4,3,Direction::up)==1);
    assert(neighbor(grid,4,2,Direction::left)==2);
    assert(neighbor(grid,4,1,Direction::up)==1);
    assert(neighbor(grid,4,-1,Direction::right)==0);
    assert(neighbor(nullptr,0,-1,Direction::down)==-1);
    assert(neighbor(scrambled,4,1,Direction::right)==3);
    assert(neighbor(scrambled,4,1,Direction::down)==2);
    assert(neighbor(temps,9,4,Direction::right)==5);
    assert(neighbor(temps,9,7,Direction::down)==8);
    assert(route(16,false,false)==Nav::previousSection);
    assert(route(18,false,false)==Nav::nextSection);
    assert(route(7,false,false)==Nav::none);
    assert(route(11,false,false)==Nav::none);
    assert(route(19,false,false)==Nav::enter);
    assert(route(16,true,false)==Nav::left);
    assert(route(18,true,false)==Nav::right);
    assert(route(7,true,false)==Nav::up);
    assert(route(11,true,false)==Nav::down);
    assert(route(19,true,false)==Nav::activate);
    assert(route(15,true,true)==Nav::backChild);
    assert(route(15,true,false)==Nav::leave);
    assert(route(15,false,false)==Nav::none);
    assert(route(0,true,false)==Nav::none);
}
