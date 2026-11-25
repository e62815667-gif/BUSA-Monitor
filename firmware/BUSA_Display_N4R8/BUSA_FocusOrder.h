#pragma once
#include <stddef.h>
#include <stdint.h>
namespace busa_focus {
struct Rect {int x,y,w,h;};
inline int absDiff(int a,int b){return a>b?a-b:b-a;}
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
