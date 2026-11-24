#include "../firmware/common/src/BUSA_UartTxMailbox.h"
#include <cassert>
#include <thread>
#include <array>
int main(){
    busa_uart::TxMailbox box;
    bool success=false;uint32_t elapsed=0;
    assert(!box.busy()&&!box.take()&&!box.publish(0));
    assert(!box.reap(success,elapsed)&&!box.complete(1,1));
    assert(box.publish(10)&&box.busy()&&box.take()==10);
    assert(!box.publish(20)&&!box.reap(success,elapsed));
    assert(box.complete(9,321)&&box.busy()&&!box.take());
    assert(box.reap(success,elapsed)&&!success&&elapsed==321&&!box.busy());
    assert(box.publish(10)&&box.complete(-1,42));
    assert(box.reap(success,elapsed)&&!success&&elapsed==42);
    std::array<unsigned,64> wire{};
    std::thread worker([&]{
        for(unsigned i=1;i<=10000;++i){
            size_t n;while(!(n=box.take()))std::this_thread::yield();
            assert(n==wire.size());
            for(auto x:wire)assert(x==i);
            assert(box.complete(int(n),i));
        }
    });
    for(unsigned i=1;i<=10000;++i){
        assert(!box.busy());wire.fill(i);assert(box.publish(wire.size()));
        while(!box.reap(success,elapsed))std::this_thread::yield();
        assert(success&&elapsed==i);
    }
    worker.join();
    assert(!box.busy());
}
