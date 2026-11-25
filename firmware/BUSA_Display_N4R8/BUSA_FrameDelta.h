#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace busa_guition {
struct FrameDelta {
    int x=0,y=0,w=0,h=0;
    bool valid()const{return w>0&&h>0;}
};
inline FrameDelta changedBounds(const uint16_t* now,const uint16_t* sent,int width,int height,int first,int count){
    FrameDelta result;
    if(!now||!sent||width<=0||height<=0||first<0||first>=height||count<=0||count>height-first)return result;
    int left=width,right=-1,top=height,bottom=-1;
    for(int y=first;y<first+count;++y){
        const auto* a=now+size_t(y)*width;const auto* b=sent+size_t(y)*width;
        if(memcmp(a,b,size_t(width)*sizeof(uint16_t))==0)continue;
        int l=0,r=width-1;while(l<width&&a[l]==b[l])++l;while(r>l&&a[r]==b[r])--r;
        if(l<left)left=l;
        if(r>right)right=r;
        if(y<top)top=y;
        bottom=y;
    }
    if(right>=left){result.x=left;result.y=top;result.w=right-left+1;result.h=bottom-top+1;}
    return result;
}
inline void packDelta(const uint16_t* now,uint16_t* packed,int width,const FrameDelta& r){
    for(int y=0;y<r.h;++y)memcpy(packed+size_t(y)*r.w,now+size_t(r.y+y)*width+r.x,size_t(r.w)*sizeof(uint16_t));
}
inline void rememberDelta(const uint16_t* now,uint16_t* sent,int width,const FrameDelta& r){
    for(int y=0;y<r.h;++y){size_t i=size_t(r.y+y)*width+r.x;memcpy(sent+i,now+i,size_t(r.w)*sizeof(uint16_t));}
}
inline bool canReserveGraphics(size_t total,size_t free,size_t bytes){
    return total>0&&free>=total/2&&bytes<=free-total/2;
}
}
