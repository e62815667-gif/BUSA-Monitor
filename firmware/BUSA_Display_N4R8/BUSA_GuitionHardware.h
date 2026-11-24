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

namespace busa_guition {
// The original device uses Arduino_GFX QSPI and the exact extracted register
// sequence. The compatible window-address implementation is from Arduino_GFX.
// Do not replace the factory table with its different stock initialization.
class FactoryPanel final : public Arduino_NV3041A {
public:
    explicit FactoryPanel(Arduino_DataBus* bus)
        : Arduino_NV3041A(bus,GFX_NOT_DEFINED,0,true,WIDTH,HEIGHT) {}
protected:
    void tftInit() override {
        applyFactoryInit(*_bus,[](uint8_t ms){delay(ms);});
        invertDisplay(false);
    }
};

// Use the factory demo full-frame path; allocate explicitly in external RAM.
class PsramCanvas final : public Arduino_Canvas {
public:
    explicit PsramCanvas(Arduino_G* output):Arduino_Canvas(WIDTH,HEIGHT,output) {}
    bool begin(int32_t speed) override {
        if(!_framebuffer) _framebuffer=static_cast<uint16_t*>(heap_caps_aligned_alloc(16,size_t(WIDTH)*HEIGHT*sizeof(uint16_t),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
        return _framebuffer && Arduino_Canvas::begin(speed);
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
        panelReady_=lcd_.begin(LCD_CLOCK_HZ);
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
        if(lcdReady_){canvas_.fillScreen(BLACK);canvas_.flush();}
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
    void present(){canvas_.flush();}
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
    SPIClass shared_;
    XPT2046_Touchscreen touch_;
    bool panelReady_=false,lcdReady_=false,touchReady_=false,sdReady_=false,capacityKnown_=false;
    uint64_t total_=0,free_=0;
    uint32_t touchErrors_=0,readErrors_=0,assetErrors_=0,lastError_=0;
    uint32_t renderLast_=UINT32_MAX,renderMax_=UINT32_MAX,fpsMilli_=UINT32_MAX;
    uint32_t frameWindow_=0,frames_=0;
};
} // namespace busa_guition
