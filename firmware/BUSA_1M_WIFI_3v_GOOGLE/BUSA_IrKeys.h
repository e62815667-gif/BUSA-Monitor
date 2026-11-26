#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace busa_ir {
constexpr size_t count=17;
constexpr int none=-1;
constexpr uint32_t releaseMs=350,holdMs=2000,learnTimeoutMs=300000;
struct Binding{const char* label;uint8_t code;};
static constexpr Binding bindings[count]={
 {"1",4},{"2",5},{"3",6},{"4",8},{"5",9},{"6",10},{"7",12},{"8",13},{"9",14},{"0",17},
 {"UP",7},{"DOWN",11},{"LEFT",16},{"RIGHT",18},{"OK",19},{"*",15},{"#",21}
};
inline uint64_t identity(uint8_t protocol,uint16_t address,uint16_t command){
 return protocol?(uint64_t(protocol)<<32)|(uint64_t(address)<<16)|command:0;
}
inline bool validMap(const uint64_t* p){
 for(size_t i=0;i<count;++i){
  if(!p[i]||p[i]>>40||!(p[i]>>32))return false;
  for(size_t j=0;j<i;++j)if(p[i]==p[j])return false;
 }
 return true;
}
enum class LearnResult{ignored,duplicate,accepted,complete};
class Map{
public:
 bool load(const uint64_t* p){if(!validMap(p))return false;memcpy(saved_,p,sizeof(saved_));ready_=true;return true;}
 void start(uint32_t now){step_=0;learning_=true;started_=now;memset(staged_,0,sizeof(staged_));}
 void cancel(){learning_=false;step_=0;}
 bool expire(uint32_t now){if(learning_&&uint32_t(now-started_)>=learnTimeoutMs){cancel();return true;}return false;}
 LearnResult capture(uint64_t code,bool repeat){
  if(!learning_||repeat||!code||code>>40||!(code>>32)||step_>=count)return LearnResult::ignored;
  for(size_t i=0;i<step_;++i)if(staged_[i]==code)return LearnResult::duplicate;
  staged_[step_++]=code;
  return step_==count?LearnResult::complete:LearnResult::accepted;
 }
 bool finish(bool persisted){const bool ok=persisted&&step_==count&&load(staged_);cancel();return ok;}
 bool ready()const{return ready_;}
 bool learning()const{return learning_;}
 size_t step()const{return step_;}
 const uint64_t* staged()const{return staged_;}
 int action(uint64_t code)const{
  if(!ready_||!code)return none;
  for(size_t i=0;i<count;++i)if(saved_[i]==code)return bindings[i].code;
  return none;
 }
private:
 uint64_t saved_[count]={},staged_[count]={};
 bool ready_=false,learning_=false;size_t step_=0;uint32_t started_=0;
};
class Press{
public:
 void clear(){active_=false;pending_=none;}
 void suppress(uint64_t token,uint32_t now){active_=true;token_=token;first_=last_=now;pending_=none;}
 template<class Emit> void expire(uint32_t now,Emit emit){
  if(active_&&uint32_t(now-last_)>releaseMs){int p=pending_;clear();if(p!=none)emit(p);}
 }
 template<class Emit> void frame(uint64_t token,int key,bool repeat,uint32_t now,Emit emit){
  expire(now,emit);
  if(repeat){
   if(!active_||token!=token_)return;
   last_=now;
   if(pending_!=none&&uint32_t(now-first_)>=holdMs){pending_=none;emit(20);}
   return;
  }
  if(active_&&pending_!=none){int p=pending_;pending_=none;emit(p);}
  clear();
  if(key==none)return;
  suppress(token,now);
  if(key==17)pending_=key;else emit(key);
 }
private:
 bool active_=false;uint64_t token_=0;uint32_t first_=0,last_=0;int pending_=none;
};
}
