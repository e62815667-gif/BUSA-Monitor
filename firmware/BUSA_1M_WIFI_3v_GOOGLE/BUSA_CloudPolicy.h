#pragma once
#include <stdint.h>
#include <string.h>
constexpr uint32_t BUSA_BUILD=141901;
inline bool busaSlotOK(uint32_t address,uint32_t size){return (address==0x10000||address==0x340000)&&size==0x330000;}
inline bool busaHexLower(const char* s,size_t n){
  if(!s||strlen(s)!=n)return false;
  for(size_t i=0;i<n;++i)if(!((s[i]>='0'&&s[i]<='9')||(s[i]>='a'&&s[i]<='f')))return false;
  return true;
}
inline bool busaCloudMetaOK(const char* url,const char* sha,const char* version,uint32_t size,uint32_t build){
  const char* prefix="https://raw.githubusercontent.com/e62815667-gif/BUSA-Monitor/";
  if(!url||!version||strlen(url)>=256||strncmp(url,prefix,strlen(prefix))||!busaHexLower(sha,64)||size<60000||size>0x330000||build<141601)return false;
  const char* revision=url+strlen(prefix);
  if(strlen(revision)<40)return false;
  char commit[41];memcpy(commit,revision,40);commit[40]=0;if(!busaHexLower(commit,40))return false;
  const char* path=revision+40;const char* dir="/releases/cloud/";
  if(strncmp(path,dir,strlen(dir)))return false;
  path+=strlen(dir);size_t n=strlen(path);
  if(n<5||strcmp(path+n-4,".bin"))return false;
  for(size_t i=0;i<n-4;++i)if(!((path[i]>='A'&&path[i]<='Z')||(path[i]>='a'&&path[i]<='z')||(path[i]>='0'&&path[i]<='9')||path[i]=='_'||path[i]=='-'))return false;
  if(strlen(version)<5||strlen(version)>24||strncmp(version,"V14.",4))return false;
  for(size_t i=4;version[i];++i)if(version[i]<'0'||version[i]>'9')return false;
  return true;
}
inline bool busaDigestMatches(const uint8_t digest[32],const char* hex){
  if(!busaHexLower(hex,64))return false;
  const char* digits="0123456789abcdef";unsigned difference=0;
  for(size_t i=0;i<32;++i){difference|=(unsigned)(digits[digest[i]>>4]^hex[i*2]);difference|=(unsigned)(digits[digest[i]&15]^hex[i*2+1]);}
  return difference==0;
}
