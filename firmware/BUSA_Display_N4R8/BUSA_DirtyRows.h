#pragma once
#include <stdint.h>
#include <string.h>
namespace busa_guition {
// Track dirty full-width rows in the existing canvas; no new pixel buffer.
class DirtyRows {
public:
    explicit DirtyRows(int16_t height):height_(height>maxRows?maxRows:(height<0?0:height)),first_(height_){}
    void note(int16_t y1,int16_t y2){
        if(y1>y2||y2<0||y1>=height_)return;
        if(y1<0)y1=0;
        if(y2>=height_)y2=height_-1;
        if(y1<first_)first_=y1;
        if(y2>last_)last_=y2;
        for(int16_t row=y1;row<=y2;++row)dirty_[row]=true;
    }
    bool take(int16_t& first,int16_t& count){
        const bool valid=last_>=first_;
        first=valid?first_:0;
        count=valid?last_-first_+1:0;
        clear();
        return valid;
    }
    // Return each disjoint run, skipping untouched rows instead of sending
    // the entire span between the clock and a lower changing label.
    bool takeNext(int16_t& first,int16_t& count){
        while(first_<height_&&!dirty_[first_])++first_;
        if(first_>=height_||first_>last_){first=0;count=0;clear();return false;}
        first=first_;
        while(first_<height_&&dirty_[first_]){dirty_[first_]=false;++first_;}
        count=first_-first;
        if(first_>last_)clear();
        return true;
    }
    void clear(){memset(dirty_,0,sizeof(dirty_));first_=height_;last_=-1;}
private:
    static constexpr int16_t maxRows=272;
    int16_t height_,first_,last_=-1;
    bool dirty_[maxRows]={};
};
}
