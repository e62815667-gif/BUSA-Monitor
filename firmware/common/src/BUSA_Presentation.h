#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
namespace busa_presentation {
inline bool endsWith(const char* s,const char* suffix){
    if(!s)return false;
    const size_t n=std::strlen(s),m=std::strlen(suffix);
    return n>=m&&std::strcmp(s+n-m,suffix)==0;
}
inline const char* firmwareLabel(const char* version){
    return endsWith(version,"-PERF4-HW2-NAME1")||endsWith(version,"-PERF4-HW2-NAME1-TIME1")?"BUSA 1M AI 1v":version;
}
inline void duration(char* out,size_t size,uint32_t ms){
    const unsigned long minutes=ms/60000u;
    std::snprintf(out,size,"%luч%02luмин",minutes/60u,minutes%60u);
}
inline size_t clockLength(const char* stamp){
    const size_t n=std::strlen(stamp);
    return n==19&&stamp[2]=='.'&&stamp[5]=='.'&&stamp[10]==' '&&stamp[13]==':'&&stamp[16]==':'?16:n;
}
}
