#pragma once
#include <cstdint>
// Caller supplies a short critical section. No network, allocation or SDK calls.
struct BusaRmPublishTracker {
  struct Pending {int id=0;uint32_t at=0,seq=0,uptime=0;};
  struct EarlyAck {int id=0;uint32_t at=0;};
  Pending pending[16];EarlyAck early[16];uint8_t earlyCursor=0;
  uint32_t generation=0,connectedEvents=0,disconnectedEvents=0,publishedEvents=0;
  uint32_t enqueued=0,acked=0,unconfirmed=0,deleted=0;
  uint32_t lastEnqueuedSeq=0,lastAckSeq=0,lastAckUptime=0,lastAckAt=0;
  int lastMessageId=0;
  void disconnected(){
    ++disconnectedEvents;
    for(auto& p:pending){if(p.id)++unconfirmed;p=Pending();}
    for(auto& a:early)a=EarlyAck();
  }
  void connected(){++connectedEvents;++generation;}
  void acknowledge(const Pending& p,uint32_t now){
    ++acked;lastAckAt=now;
    // An out-of-order ACK must not move the latest acknowledged snapshot backwards.
    if(!lastAckSeq||int32_t(p.seq-lastAckSeq)>=0){lastAckSeq=p.seq;lastAckUptime=p.uptime;}
  }
  void published(int id,uint32_t now){
    ++publishedEvents;if(id<=0)return;
    for(auto& p:pending)if(p.id==id){acknowledge(p,now);p=Pending();return;}
    early[earlyCursor].id=id;early[earlyCursor].at=now;earlyCursor=(earlyCursor+1)%16;
  }
  void submitted(int id,uint32_t started,uint32_t now,uint32_t seq,uint32_t uptime){
    if(id<=0){++unconfirmed;return;}
    ++enqueued;lastEnqueuedSeq=seq;lastMessageId=id;
    Pending value;value.id=id;value.at=now;value.seq=seq;value.uptime=uptime;
    for(auto& a:early)if(a.id==id&&int32_t(a.at-started)>=0&&uint32_t(now-a.at)<=30000U){
      acknowledge(value,a.at);a=EarlyAck();return;
    }
    for(auto& p:pending)if(!p.id){p=value;return;}
    ++unconfirmed; // Bounded bookkeeping, never invent an ACK if capacity is exhausted.
  }
  void messageDeleted(int id){
    ++deleted;
    for(auto& p:pending)if(p.id==id){++unconfirmed;p=Pending();return;}
  }
  bool expire(uint32_t now){
    bool lost=false;
    for(auto& p:pending)if(p.id&&uint32_t(now-p.at)>=60000U){++unconfirmed;p=Pending();lost=true;}
    return lost;
  }
  unsigned outstanding()const{unsigned n=0;for(const auto& p:pending)if(p.id)++n;return n;}
};
