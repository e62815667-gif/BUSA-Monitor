#include "../firmware/common/src/BUSA_DisplayProtocol.h"
#include <cassert>
int main(){
    using namespace busa_uart;
    assert(baud==1152000);
    assert(mainTxBuffer==4096);
    assert(txWriteBudget(1000,-1)==0);
    assert(txWriteBudget(1000,0)==0);
    assert(txWriteBudget(1000,128)==0);
    assert(txWriteBudget(1000,129)==1);
    assert(txWriteBudget(1000,256)==128);
    assert(txWriteBudget(10000,4096)==2048);
    assert(txWriteBudget(17,4096)==17);
    assert(txWriteBudget(0,4096)==0);
    assert(txWriteBudget(2074,4096)==2048);
}
