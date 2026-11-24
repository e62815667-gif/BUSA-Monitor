#pragma once
#include <stdint.h>
namespace busa_guition {
// Contiguous full-width rows in the existing canvas; no new pixel buffer.
class DirtyRows {
public:
    explicit DirtyRows(int16_t height):height_(height),first_(height){}
    void note(int16_t y1,int16_t y2){
        if(y1>y2||y2<0||y1>=height_)return;
        if(y1<0)y1=0;
        if(y2>=height_)y2=height_-1;
        if(y1<first_)first_=y1;
        if(y2>last_)last_=y2;
    }
    bool take(int16_t& first,int16_t& count){
        const bool valid=last_>=first_;
        first=valid?first_:0;
        count=valid?last_-first_+1:0;
        first_=height_;last_=-1;
        return valid;
    }
private:
    int16_t height_,first_,last_=-1;
};
}
