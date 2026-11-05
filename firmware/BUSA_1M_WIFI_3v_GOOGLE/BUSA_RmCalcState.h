#pragma once
#include "BUSA_Calculator.h"
#include <cstdio>
#include <cstring>
// Independent RAM-only RainMaker calculator inputs. Main loop owns this state.
struct BusaRmCalcState {
  float inputs[5]={};uint32_t revision=1;
  char outputs[4][64]={{}};char status[96]={};
};
static BusaRmCalcState busaRmCalculators[3];
static Device* busaRmCalcDevices[3]={};
static bool busaRmCalcReady[3]={};
static const char* const busaRmCalcNames[3]={"Калькулятор · Разбавление","Калькулятор · Ареометр","Калькулятор · Перегонка"};
static const uint8_t busaRmCalcInputCounts[3]={3,2,5},busaRmCalcOutputCounts[3]={2,1,4};
static const char* const busaRmCalcInputNames[3][5]={
 {"Объём спирта, л","Начальная крепость, %","Нужная крепость, %","",""},
 {"Температура, °C","Показание, %","","",""},
 {"Объём сырца, л","Крепость сырца, %","Крепость продукта, %","Головы от АС, %","Хвосты от АС, %"}
};
static const char* const busaRmCalcOutputNames[3][4]={
 {"Долить воды","Объём смеси","",""},
 {"Крепость при 20 °C","","",""},
 {"Абсолютный спирт","Головы","Хвосты","Выход продукта"}
};
static const char* const busaRmCalcNotes[3]={
 "Объёмы при 20 °C; учтена контракция.",
 "0…40 °C; стеклянный спиртомер на 20 °C; раствор без сахара. Оценка.",
 "Головы/хвосты в л АС, не физические объёмы фракций. Выход без доп. потерь."
};
static void busaRmCalcClearResult(BusaRmCalcState& s,const char* status){
 for(auto& out:s.outputs)snprintf(out,sizeof(out),"—");
 snprintf(s.status,sizeof(s.status),"%s",status);
}
static void busaRmCalcReset(uint8_t mode){
 auto& s=busaRmCalculators[mode];for(auto& v:s.inputs)v=0;
 if(mode==1)s.inputs[0]=20;
 busaRmCalcClearResult(s,"Введите значения; затем Рассчитать");++s.revision;
}
static bool busaRmCalcInputAllowed(uint8_t mode,uint8_t field,float v){
 if(mode>=3||field>=busaRmCalcInputCounts[mode]||!isfinite(v)||v<0)return false;
 if((mode==0&&field==0)||(mode==2&&field==0))return v<=100000;
 if(mode==1&&field==0)return v<=40;
 return v<=100;
}
static void busaRmCalcFormat(char* out,size_t size,double value,const char* unit){
 snprintf(out,size,"%.2f",value);for(char* p=out;*p;p++)if(*p=='.')*p=',';
 size_t used=strlen(out);if(used<size)snprintf(out+used,size-used," %s",unit);
}
static bool busaRmCalcApply(uint8_t mode,uint8_t action,uint8_t field,float input,bool pressed){
 if(mode>=3)return false;auto& s=busaRmCalculators[mode];
 if(action==10){
  if(!busaRmCalcInputAllowed(mode,field,input)){
   busaRmCalcClearResult(s,"Неверное значение поля");++s.revision;return false;
  }
  s.inputs[field]=input;busaRmCalcClearResult(s,"Параметры изменены; Рассчитать");++s.revision;return true;
 }
 if(action!=11&&action!=12)return false;
 if(!pressed)return true;
 if(action==12){busaRmCalcReset(mode);return true;}
 const auto* v=s.inputs;BusaCalc::Result r;
 if(mode==0)r=BusaCalc::dilute(v[0],v[1],v[2]);
 else if(mode==1)r=BusaCalc::correct(v[0],v[1]);
 else r=BusaCalc::fraction(v[0],v[1],v[2],v[3],v[4]);
 ++s.revision;
 if(!r.ok){busaRmCalcClearResult(s,"Проверьте диапазоны и доли");return false;}
 busaRmCalcClearResult(s,"Готово");
 if(mode==0){busaRmCalcFormat(s.outputs[0],64,r.water,"л");busaRmCalcFormat(s.outputs[1],64,r.total,"л");}
 else if(mode==1)busaRmCalcFormat(s.outputs[0],64,r.strength,"% об.");
 else{
  busaRmCalcFormat(s.outputs[0],64,r.absolute,"л АС");busaRmCalcFormat(s.outputs[1],64,r.heads,"л АС");
  busaRmCalcFormat(s.outputs[2],64,r.tails,"л АС");busaRmCalcFormat(s.outputs[3],64,r.product,"л");
 }
 return true;
}
