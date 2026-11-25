#pragma once
#include <stdint.h>
#include <stddef.h>
namespace busa_keys {
constexpr size_t capacity=16;
constexpr uint32_t maxAgeMs=2500;
constexpr uint8_t longZero=20;
inline bool valid(uint8_t code){return code<=longZero&&code!=1&&code!=2&&code!=3;}
struct Key {uint32_t seq,time;uint8_t code;};
class Ring {
public:
 void begin(uint32_t boot){boot_=boot?boot:1;seq_=0;count_=0;}
 void push(uint8_t code,uint32_t now){if(!valid(code))return;++seq_;rows_[seq_%capacity]={seq_,now,code};if(count_<capacity)++count_;}
 uint32_t boot()const{return boot_;}
 uint32_t head()const{return seq_;}
 size_t size()const{return count_;}
 const Key& oldest(size_t i)const{return rows_[(seq_-uint32_t(count_)+1+uint32_t(i))%capacity];}
private:
 Key rows_[capacity]={};uint32_t boot_=1,seq_=0;size_t count_=0;
};
struct Event {uint32_t seq,age;uint8_t code;};
enum class Result {baseline,applied,lost};
class Cursor {
public:
 void reset(){ready_=false;}
 template<class F> Result consume(uint32_t boot,uint32_t head,const Event* events,size_t count,F emit){
  if(!boot||count>capacity||(count&&!events)){reset();return Result::lost;}
  if(!ready_||boot_!=boot){ready_=true;boot_=boot;last_=head;return Result::baseline;}
  const uint32_t n=head-last_;
  if(!n)return Result::applied;
  const uint32_t previous=last_;last_=head;
  if(n>count||n>capacity)return Result::lost;
  const size_t start=count-n;
  for(size_t i=0;i<n;++i){const Event& e=events[start+i];if(e.seq!=previous+uint32_t(i)+1||e.age>maxAgeMs||!valid(e.code))return Result::lost;}
  for(size_t i=0;i<n;++i)emit(events[start+i]);
  return Result::applied;
 }
private:
 bool ready_=false;uint32_t boot_=0,last_=0;
};
}
