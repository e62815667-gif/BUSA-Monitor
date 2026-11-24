#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
namespace busa_ota {
constexpr uint8_t poll=7,offer=8,request=9,chunk=10,result=11,ack=12;
constexpr uint8_t written=1,failed=2,bootOk=3,bootFailed=4;
constexpr uint32_t slotSize=0x1c0000;
constexpr size_t blockSize=1024,offerSize=40,requestSize=8,resultSize=5;
constexpr char displayFamily[]="BUSA-DISPLAY-N4R8-OTA1";
constexpr char mainFamily[]="14v9-N16R8-";
inline bool validSize(size_t size){return size>=60000&&size<=slotSize;}
inline bool validHeader(const uint8_t* p,size_t n){return p&&n>=36&&p[0]==0xe9&&p[12]==9&&p[13]==0&&p[32]==0x32&&p[33]==0x54&&p[34]==0xcd&&p[35]==0xab;}
inline size_t chunkLength(uint32_t size,uint32_t offset){if(!validSize(size)||offset>=size)return 0;return size-offset>blockSize?blockSize:size-offset;}
class Marker {
public:
 explicit Marker(const char* pattern):pattern_(pattern),len_(pattern?strlen(pattern):0){}
 void reset(){matched_=0;found_=false;}
 bool found()const{return found_;}
 void feed(const uint8_t* p,size_t n){
  if(found_||!p||!len_)return;
  for(size_t i=0;i<n&&!found_;++i){
   while(matched_&&p[i]!=uint8_t(pattern_[matched_])){
    size_t prefix=matched_-1;
    while(prefix&&memcmp(pattern_,pattern_+matched_-prefix,prefix)!=0)--prefix;
    matched_=prefix;
   }
   if(p[i]==uint8_t(pattern_[matched_]))++matched_;
   if(matched_==len_)found_=true;
  }
 }
private:
 const char* pattern_;size_t len_,matched_=0;bool found_=false;
};
}
