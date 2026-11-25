#include "../firmware/BUSA_Display_N4R8/BUSA_FrameDelta.h"
#include <cassert>
#include <cstdio>
#include <vector>
#include <random>
#include <algorithm>
using namespace busa_guition;
int main(){
 constexpr int W=480,H=272;std::vector<uint16_t> a(W*H),b(a),p(a),display(a);
 assert(!changedBounds(a.data(),b.data(),W,H,0,H).valid());
 assert(!changedBounds(a.data(),b.data(),W,H,-1,H).valid());
 assert(!changedBounds(a.data(),b.data(),W,H,1,H).valid());
 for(int y=70;y<73;++y)for(int x=219;x<222;++x)a[y*W+x]=0x1234;
 auto r=changedBounds(a.data(),b.data(),W,H,69,5);assert(r.x==219&&r.y==70&&r.w==3&&r.h==3);
 packDelta(a.data(),p.data(),W,r);for(int i=0;i<9;++i)assert(p[i]==0x1234);
 rememberDelta(a.data(),b.data(),W,r);assert(a==b);
 std::fill(b.begin(),b.end(),0);std::fill(a.begin(),a.end(),0xffff);
 r=changedBounds(a.data(),b.data(),W,H,0,H);assert(r.x==0&&r.y==0&&r.w==W&&r.h==H);
 rememberDelta(a.data(),b.data(),W,r);assert(a==b);display=b;
 std::mt19937 rng(42);
 for(int trial=0;trial<150;++trial){
  int start=rng()%H,count=1+rng()%(H-start);
  for(int j=0;j<100;++j){int x=rng()%W,y=start+rng()%count;a[y*W+x]=uint16_t(rng());}
  r=changedBounds(a.data(),b.data(),W,H,start,count);
  if(r.valid()){
   packDelta(a.data(),p.data(),W,r);
   for(int y=0;y<r.h;++y)for(int x=0;x<r.w;++x)display[(r.y+y)*W+r.x+x]=p[y*r.w+x];
   rememberDelta(a.data(),b.data(),W,r);
  }
  assert(a==b&&a==display);
 }
 assert(canReserveGraphics(8388608,8067328,522240));
 assert(!canReserveGraphics(8388608,4194304,1));
 assert(!canReserveGraphics(8388608,4194303,0));
 assert(!canReserveGraphics(0,0,0));
 std::puts("PASS delta: no redundant frames, 9 instead of 2400 pixels for isolated change, full frame and 150 randomized reconstruction trials, PSRAM limit");
}
