#pragma once
#include <stddef.h>
#include <stdint.h>
namespace busa_focus {
struct Rect {int x,y,w,h;};
inline int absDiff(int a,int b){return a>b?a-b:b-a;}
enum class Direction {up,down,left,right};
enum class Nav {none,previousSection,nextSection,enter,leave,backChild,up,down,left,right,activate};
inline Nav route(uint8_t k,bool entered,bool child){
    switch(k){
        case 15:return child?Nav::backChild:entered?Nav::leave:Nav::none;
        case 19:return entered?Nav::activate:Nav::enter;
        case 16:return entered?Nav::left:Nav::previousSection;
        case 18:return entered?Nav::right:Nav::nextSection;
        case 7:return entered?Nav::up:Nav::none;
        case 11:return entered?Nav::down:Nav::none;
        default:return Nav::none;
    }
}
inline int neighbor(const Rect* r,unsigned n,int current,Direction d){
    if(!r||!n||n>64)return -1;
    if(current<0||unsigned(current)>=n)return 0;
    const Rect& a=r[current];
    const bool horizontal=d==Direction::left||d==Direction::right;
    const int sign=(d==Direction::left||d==Direction::up)?-1:1;
    int best=current,bestBand=2,bestScore=0,bestPrimary=0;
    for(unsigned i=0;i<n;++i){
        if(int(i)==current)continue;
        const Rect& b=r[i];
        const int dx=2*(b.x-a.x)+b.w-a.w,dy=2*(b.y-a.y)+b.h-a.h;
        const int primary=sign*(horizontal?dx:dy);
        if(primary<=0)continue;
        const int offset=absDiff(horizontal?dy:dx,0);
        const int band=offset<(horizontal?a.h+b.h:a.w+b.w)?0:1;
        const int score=primary+2*offset;
        if(band<bestBand||(band==bestBand&&(score<bestScore||(score==bestScore&&primary<bestPrimary)))){
            best=int(i);bestBand=band;bestScore=score;bestPrimary=primary;
        }
    }
    return best;
}
// Group aligned visual rows, then traverse repeated grids column by column.
// Single rows (including horizontal tab bars) retain left-to-right order.
inline void orderControls(const Rect* r,unsigned n,unsigned* out){
    if(!n||n>64)return;
    unsigned sorted[64],starts[65];
    for(unsigned i=0;i<n;++i){unsigned k=i;while(k&&r[sorted[k-1]].y>r[i].y){sorted[k]=sorted[k-1];--k;}sorted[k]=i;}
    unsigned rows=0;
    for(unsigned i=0;i<n;){starts[rows++]=i;unsigned j=i+1;while(j<n&&absDiff(r[sorted[j]].y,r[sorted[i]].y)<=6)++j;
        for(unsigned a=i+1;a<j;++a){unsigned v=sorted[a],k=a;while(k>i&&r[sorted[k-1]].x>r[v].x){sorted[k]=sorted[k-1];--k;}sorted[k]=v;}i=j;}
    starts[rows]=n;unsigned dst=0;
    for(unsigned a=0;a<rows;){const unsigned cols=starts[a+1]-starts[a];unsigned b=a+1;
        if(cols>1)while(b<rows&&starts[b+1]-starts[b]==cols){bool aligned=true;for(unsigned c=0;c<cols;++c)if(absDiff(r[sorted[starts[a]+c]].x,r[sorted[starts[b]+c]].x)>8)aligned=false;if(!aligned)break;++b;}
        for(unsigned c=0;c<cols;++c)for(unsigned row=a;row<b;++row)out[dst++]=sorted[starts[row]+c];
        a=b;
    }
}
}
