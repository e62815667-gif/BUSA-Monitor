#pragma once
#include <atomic>
#include <stddef.h>
#include <stdint.h>
namespace busa_uart {
// One producer (main loop), one consumer (UART TX task). The producer
// owns the external frame buffer until publish, then cannot modify it
// until the worker has completed and the producer has reaped the result.
class TxMailbox {
public:
    bool busy() const{return state_.load(std::memory_order_acquire)!=idle;}
    bool publish(size_t size){
        if(!size||busy())return false;
        size_=size;
        state_.store(pending,std::memory_order_release);
        return true;
    }
    size_t take() const{
        return state_.load(std::memory_order_acquire)==pending?size_:0;
    }
    bool complete(int written,uint32_t ms){
        if(state_.load(std::memory_order_acquire)!=pending)return false;
        success_=written>=0&&size_t(written)==size_;
        elapsed_=ms;
        state_.store(done,std::memory_order_release);
        return true;
    }
    bool reap(bool& success,uint32_t& ms){
        if(state_.load(std::memory_order_acquire)!=done)return false;
        success=success_;ms=elapsed_;
        state_.store(idle,std::memory_order_release);
        return true;
    }
private:
    enum : uint32_t {idle,pending,done};
    std::atomic<uint32_t> state_{idle};
    size_t size_=0;
    bool success_=false;
    uint32_t elapsed_=0;
};
}
