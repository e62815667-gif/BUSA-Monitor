"""Compile the production subclasses with Arduino_G's inherited dimension names.
Host test of constructor arguments and allocation behavior, not a hardware test.
"""
from pathlib import Path
import subprocess
import tempfile

base = Path(__file__).resolve().parents[1] / 'firmware/BUSA_Display_N4R8'
h = (base / 'BUSA_GuitionHardware.h').read_text()
classes = h[h.index('class FactoryPanel final'):h.index('class Hardware {')]
prelude = r'''
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <cstdio>
constexpr int GFX_NOT_DEFINED=-1, GFX_SKIP_OUTPUT_BEGIN=-2;
constexpr unsigned MALLOC_CAP_SPIRAM=1, MALLOC_CAP_8BIT=2;
static size_t requested=0;
static int aligned_calls=0, malloc_calls=0, base_calls=0;
static bool fail_aligned=false, fail_malloc=false, fail_base=false;
static void* heap_caps_aligned_alloc(size_t alignment,size_t size,unsigned caps){
    assert(alignment==16 && caps==(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    ++aligned_calls;requested=size;return size && !fail_aligned ? std::malloc(size):nullptr;
}
static void* heap_caps_malloc(size_t size,unsigned caps){
    assert(caps==(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    ++malloc_calls;requested=size;return size && !fail_malloc ? std::malloc(size):nullptr;
}
struct Logger {template<class... T> void printf(const char*,T...){} void println(const char*){}} Serial;
struct Arduino_DataBus {};
static void delay(uint8_t){}
template<class T> static void applyFactoryInit(Arduino_DataBus&,T){}
class Arduino_G {
protected:
    const int16_t WIDTH,HEIGHT;
public:
    Arduino_G(int16_t w,int16_t h):WIDTH(w),HEIGHT(h){}
    int16_t width() const{return WIDTH;}
    int16_t height() const{return HEIGHT;}
};
class Arduino_NV3041A : public Arduino_G {
protected:
    Arduino_DataBus* _bus;
    virtual void tftInit(){}
    void invertDisplay(bool){}
public:
    Arduino_NV3041A(Arduino_DataBus* b,int,int,bool,int16_t w,int16_t h):Arduino_G(w,h),_bus(b){}
};
class Arduino_Canvas : public Arduino_G {
protected:
    uint16_t* _framebuffer=nullptr;
public:
    Arduino_Canvas(int16_t w,int16_t h,Arduino_G*):Arduino_G(w,h){}
    virtual ~Arduino_Canvas(){std::free(_framebuffer);}
    virtual bool begin(int32_t speed){++base_calls;assert(speed==GFX_SKIP_OUTPUT_BEGIN);return _framebuffer&&!fail_base;}
};
namespace busa_guition {
constexpr int WIDTH=480,HEIGHT=272;
'''
checks = r'''
}
using namespace busa_guition;
static Arduino_DataBus bus;
static FactoryPanel panel(&bus);
static PsramCanvas canvas(&panel);
static PsramCanvas fallback(&panel);
static PsramCanvas failed(&panel);
static PsramCanvas base_failed(&panel);
int main(){
    assert(panel.width()==480 && panel.height()==272 && "LCD inherited dimensions shadow board constants");
    assert(canvas.width()==480 && canvas.height()==272 && "Canvas inherited dimensions shadow board constants");
    assert(canvas.begin(GFX_SKIP_OUTPUT_BEGIN));
    assert(requested==261120 && aligned_calls==1 && malloc_calls==0 && base_calls==1);
    assert(canvas.begin(GFX_SKIP_OUTPUT_BEGIN) && aligned_calls==1);
    fail_aligned=true;
    assert(fallback.begin(GFX_SKIP_OUTPUT_BEGIN) && requested==261120 && malloc_calls==1);
    fail_malloc=true;
    const int before=base_calls;
    assert(!failed.begin(GFX_SKIP_OUTPUT_BEGIN) && base_calls==before);
    fail_aligned=false;fail_malloc=false;fail_base=true;
    assert(!base_failed.begin(GFX_SKIP_OUTPUT_BEGIN));
}
'''
with tempfile.TemporaryDirectory() as tmp:
    src=Path(tmp)/'geometry.cpp'
    exe=Path(tmp)/'geometry'
    src.write_text(prelude+classes+checks)
    subprocess.run(['g++','-std=c++11',str(src),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: LCD/canvas geometry, exact PSRAM request, fallback and failure handling')
