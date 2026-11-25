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
}
