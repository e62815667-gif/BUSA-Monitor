#pragma once
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <esp_heap_caps.h>
#include <SPI.h>
#include <SD.h>
#include <XPT2046_Touchscreen.h>
#include <BUSA_DisplayProtocol.h>
#include "BUSA_GuitionPins.h"
#include "BUSA_GuitionFactoryInit.h"
#include "BUSA_DirtyRows.h"
#include "BUSA_FrameDelta.h"

namespace busa_guition {
// The original device uses Arduino_GFX QSPI and the exact extracted register
// sequence. The compatible window-address implementation is from Arduino_GFX.
// Do not replace the factory table with its different stock initialization.
class FactoryPanel final : public Arduino_NV3041A {
public:
    explicit FactoryPanel(Arduino_DataBus* bus)
        : Arduino_NV3041A(bus,GFX_NOT_DEFINED,0,true,busa_guition::WIDTH,busa_guition::HEIGHT) {}
protected:
    void tftInit() override {
        applyFactoryInit(*_bus,[](uint8_t ms){delay(ms);});
        // Arduino_GFX 1.5.0 uses (_ips ^ i); IPS=true requires true for INVOFF.
        // Field photograph shows complementary colors with the previous INVON.
        invertDisplay(true);
    }
};

// Use the factory demo full-frame path; allocate explicitly in external RAM.
class PsramCanvas final : public Arduino_Canvas {
public:
    explicit PsramCanvas(Arduino_G* output):Arduino_Canvas(busa_guition::WIDTH,busa_guition::HEIGHT,output) {}
    bool begin(int32_t speed) override {
        constexpr size_t bytes=size_t(busa_guition::WIDTH)*busa_guition::HEIGHT*sizeof(uint16_t);
        static_assert(bytes==261120,"Unexpected canvas size");
        if(width()!=busa_guition::WIDTH||height()!=busa_guition::HEIGHT){
            Serial.println("[BOOT] CANVAS GEOMETRY FAIL");return false;
        }
        if(!_framebuffer){
            _framebuffer=static_cast<uint16_t*>(heap_caps_aligned_alloc(16,bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
            Serial.printf("[BOOT] CANVAS ALIGNED bytes=%u %s\n",unsigned(bytes),_framebuffer?"OK":"FAIL");
            if(!_framebuffer){
                // QSPI copies RGB565 pixels into its own DMA buffer.
                _framebuffer=static_cast<uint16_t*>(heap_caps_malloc(bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
                Serial.printf("[BOOT] CANVAS MALLOC bytes=%u %s\n",unsigned(bytes),_framebuffer?"OK":"FAIL");
            }
        }
        if(!_framebuffer)return false;
        const bool ready=Arduino_Canvas::begin(speed);
        Serial.printf("[BOOT] CANVAS BASE %s\n",ready?"OK":"FAIL");
        return ready;
    }
};

class Hardware {
public:
    Hardware():qspi_(LCD_CS,LCD_SCK,LCD_D0,LCD_D1,LCD_D2,LCD_D3),
               lcd_(&qspi_),canvas_(&lcd_),shared_(HSPI),touch_(TOUCH_CS) {}
    void beginDisplay() {
        pinMode(BACKLIGHT,OUTPUT);digitalWrite(BACKLIGHT,LOW);
        pinMode(TOUCH_CS,OUTPUT);digitalWrite(TOUCH_CS,HIGH);
        pinMode(SD_CS,OUTPUT);digitalWrite(SD_CS,HIGH);
        Serial.println("[BOOT] LCD BEGIN");
        Serial.printf("[BOOT] LCD GEOMETRY %dx%d\n",int(lcd_.width()),int(lcd_.height()));
        panelReady_=lcd_.width()==busa_guition::WIDTH&&lcd_.height()==busa_guition::HEIGHT&&lcd_.begin(LCD_CLOCK_HZ);
        Serial.printf("[BOOT] LCD %s\n",panelReady_?"OK":"FAIL");
        if(panelReady_){
            lcd_.fillScreen(BLACK);
            backlight(true);
            Serial.println("[BOOT] BACKLIGHT HIGH");
        }
        const bool ramReady=psramFound();
        Serial.printf("[BOOT] PSRAM %s total=%u free=%u largest=%u\n",ramReady?"OK":"FAIL",
                      unsigned(ESP.getPsramSize()),unsigned(ESP.getFreePsram()),
                      unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT)));
        lcdReady_=panelReady_&&ramReady&&canvas_.begin(GFX_SKIP_OUTPUT_BEGIN);
        Serial.printf("[BOOT] CANVAS %s\n",lcdReady_?"OK":"FAIL");
        if(lcdReady_){
            canvas_.fillScreen(BLACK);canvas_.flush();
            constexpr size_t bytes=size_t(WIDTH)*HEIGHT*sizeof(uint16_t);
            if(canReserveGraphics(ESP.getPsramSize(),ESP.getFreePsram(),2*bytes)){
                sent_=static_cast<uint16_t*>(heap_caps_malloc(bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
                packed_=static_cast<uint16_t*>(heap_caps_malloc(bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
                if(!sent_||!packed_){free(sent_);free(packed_);sent_=packed_=nullptr;}
            }
            Serial.printf("[BOOT] GRAPHICS DELTA %s bytes=%u\n",sent_?"ON":"FALLBACK",sent_?unsigned(2*bytes):0u);
        }
        if(!lcdReady_)lastError_=1;
    }
    void beginPeripherals() {
        Serial.println("[BOOT] TOUCH BEGIN");
        shared_.begin(SHARED_SPI_SCK,SHARED_SPI_MISO,SHARED_SPI_MOSI,-1);
        touchReady_=touch_.begin(shared_);
        touch_.setRotation(0);
        Serial.printf("[BOOT] TOUCH %s\n",touchReady_?"OK":"FAIL");
        Serial.println("[BOOT] SD BEGIN");
        sdReady_=SD.begin(SD_CS,shared_,4000000,"/sd",5,false) && SD.cardType()!=CARD_NONE;
        Serial.printf("[BOOT] SD %s\n",sdReady_?"OK":"UNAVAILABLE");
        if(sdReady_)refreshCapacity();
        if(!lcdReady_)lastError_=1;
        else if(!sdReady_)lastError_=2;
    }
    bool lcdReady() const{return lcdReady_;}
    // A successfully configured library is not proof of a physical touch.
    bool touchReady() const{return touchReady_;}
    bool sdReady() const{return sdReady_;}
    Arduino_GFX& panel(){return canvas_;}
    void noteDirtyRows(int16_t y1,int16_t y2){dirtyRows_.note(y1,y2);}
    void present(){
        int16_t first=0,count=0;
        if(firstFrame_){canvas_.flush();firstFrame_=false;dirtyRows_.clear();
            if(sent_)memcpy(sent_,canvas_.getFramebuffer(),size_t(WIDTH)*HEIGHT*sizeof(uint16_t));
            return;}
        // The existing full-frame canvas remains authoritative; send each
        // dirty run once, not the clean rows between distant UI changes.
        while(dirtyRows_.takeNext(first,count)){
            if(!sent_||!packed_){
                lcd_.draw16bitRGBBitmap(0,first,canvas_.getFramebuffer()+size_t(first)*busa_guition::WIDTH,busa_guition::WIDTH,count);
                continue;
            }
            auto* pixels=canvas_.getFramebuffer();
            const auto r=changedBounds(pixels,sent_,WIDTH,HEIGHT,first,count);
            if(!r.valid())continue;
            if(r.w==WIDTH)lcd_.draw16bitRGBBitmap(0,r.y,pixels+size_t(r.y)*WIDTH,r.w,r.h);
            else {packDelta(pixels,packed_,WIDTH,r);lcd_.draw16bitRGBBitmap(r.x,r.y,packed_,r.w,r.h);}
            rememberDelta(pixels,sent_,WIDTH,r);
        }
    }
    void backlight(bool on){digitalWrite(BACKLIGHT,on&&panelReady_?HIGH:LOW);}
    bool readRawTouch(TS_Point& p){
        if(!touchReady_||!touch_.touched())return false;
        p=touch_.getPoint();
        if(p.x<0||p.x>4095||p.y<0||p.y>4095){++touchErrors_;return false;}
        return true;
    }
    // The caller owns calibration and pixel mapping; no guessed factory
    // calibration is silently written into NVS.
    bool readAsset(const char* path,uint8_t* output,size_t size){
        if(!sdReady_||!path||!output||!size){++assetErrors_;return false;}
        File f=SD.open(path,FILE_READ);
        if(!f||f.isDirectory()||f.size()!=size){
            ++assetErrors_;++readErrors_;lastError_=3;return false;
        }
        const size_t actual=f.read(output,size);
        f.close();
        if(actual!=size){++assetErrors_;++readErrors_;lastError_=3;return false;}
        return true;
    }
    void invalidAsset(){++assetErrors_;lastError_=3;}
    void refreshCapacity(){
        if(!sdReady_)return;
        total_=SD.totalBytes();
        const uint64_t used=SD.usedBytes();
        free_=used<=total_?total_-used:0;
        capacityKnown_=total_>0&&used<=total_;
    }
    void frameComplete(uint32_t started){
        const uint32_t now=millis();
        renderLast_=now-started;
        if(renderMax_==UINT32_MAX||renderLast_>renderMax_)renderMax_=renderLast_;
        if(!frames_){Serial.println("[BOOT] FIRST_FRAME OK");frameWindow_=now;frames_=1;return;}
        ++frames_;
        const uint32_t span=now-frameWindow_;
        if(span>=1000){
            fpsMilli_=uint32_t(uint64_t(frames_-1)*1000000u/span);
            frameWindow_=now;frames_=1;
        }
    }
    void metrics(uint32_t* out) const{
        using namespace busa_uart;
        out[lcdInit]=lcdReady_?1:0;
        out[touchInit]=touchReady_?1:0;
        out[touchErrors]=touchErrors_;
        out[sdStatus]=sdReady_?1:0;
        if(capacityKnown_){
            out[sdTotalLow]=uint32_t(total_);out[sdTotalHigh]=uint32_t(total_>>32);
            out[sdFreeLow]=uint32_t(free_);out[sdFreeHigh]=uint32_t(free_>>32);
        }
        out[sdReadErrors]=readErrors_;out[assetErrors]=assetErrors_;
        out[renderLast]=renderLast_;out[renderMax]=renderMax_;
        out[fpsMilli]=fpsMilli_;out[lastError]=lastError_;
        // This interface never writes or formats the SD card.
        out[sdWriteErrors]=0;
    }
private:
    Arduino_ESP32QSPI qspi_;
    FactoryPanel lcd_;
    PsramCanvas canvas_;
    DirtyRows dirtyRows_{busa_guition::HEIGHT};
    bool firstFrame_=true;
    uint16_t *sent_=nullptr,*packed_=nullptr;
    SPIClass shared_;
    XPT2046_Touchscreen touch_;
    bool panelReady_=false,lcdReady_=false,touchReady_=false,sdReady_=false,capacityKnown_=false;
    uint64_t total_=0,free_=0;
    uint32_t touchErrors_=0,readErrors_=0,assetErrors_=0,lastError_=0;
    uint32_t renderLast_=UINT32_MAX,renderMax_=UINT32_MAX,fpsMilli_=UINT32_MAX;
    uint32_t frameWindow_=0,frames_=0;
};
} // namespace busa_guition
