#pragma once
#include "BUSA_Calculator.h"
#include <cstdint>
#include <cstdlib>
// Calculator state is RAM-only and independent from sensor/threshold/Wi-Fi settings.
static uint8_t busaCalcMode=0,busaCalcField=0;
static bool busaCalcEditing=false,busaCalcComplete=false;
static String busaCalcInputs[3][5],busaCalcEdit="";
static uint8_t busaCalcCursor=0;
static BusaCalc::Result busaCalcResult;
static const char* const busaCalcNames[3]={"Разбавление","Ареометр","Вторая перегонка"};
static const uint8_t busaCalcCounts[3]={3,2,5};
static const char* const busaCalcLabels[3][5]={
 {"Объём, л","Крепость, %","Нужная, %","",""},
 {"Температура, °C","Показание, %","","",""},
 {"Объём, л","Крепость, %","На выходе, %","Головы, % АС","Хвосты, % АС"}
};
static bool busaCalcParse(String text,double& out){
 text.trim();text.replace(',','.');if(!text.length())return false;
 bool digit=false,dot=false;for(unsigned i=0;i<text.length();i++){
  char c=text[i];if(c>='0'&&c<='9')digit=true;else if(c=='.'&&!dot)dot=true;else return false;
 }
 if(!digit)return false;
 char* end=nullptr;out=strtod(text.c_str(),&end);return end&&!*end&&isfinite(out);
}
static void busaCalcRecalculate(){
 double v[5]={};busaCalcComplete=false;busaCalcResult=BusaCalc::Result();
 for(unsigned i=0;i<busaCalcCounts[busaCalcMode];i++)if(!busaCalcParse(busaCalcInputs[busaCalcMode][i],v[i]))return;
 busaCalcComplete=true;
 if(busaCalcMode==0)busaCalcResult=BusaCalc::dilute(v[0],v[1],v[2]);
 else if(busaCalcMode==1)busaCalcResult=BusaCalc::correct(v[0],v[1]);
 else busaCalcResult=BusaCalc::fraction(v[0],v[1],v[2],v[3],v[4]);
}
static void busaCalcDecimal(){
 if(busaCalcEditing&&busaCalcEdit.indexOf(',')<0&&busaCalcEdit.length()<12){
  busaCalcEdit=busaCalcEdit.substring(0,busaCalcCursor)+","+busaCalcEdit.substring(busaCalcCursor);++busaCalcCursor;
 }
 tftDirty=true;
}
static bool busaCalcKey(int k){
 if(currentPage!=7)return false;
 if(busaCalcEditing){
  if(k==15){busaCalcEditing=false;busaCalcEdit="";}
  else if(k==0){busaCalcEdit="";busaCalcCursor=0;}
  else if(k==16){if(busaCalcCursor)--busaCalcCursor;}
  else if(k==18){if(busaCalcCursor<busaCalcEdit.length())++busaCalcCursor;}
  else if(k==7)busaCalcCursor=0;
  else if(k==11)busaCalcCursor=busaCalcEdit.length();
  else if(k==19){double value=0;if(busaCalcParse(busaCalcEdit,value)){
    busaCalcInputs[busaCalcMode][busaCalcField]=busaCalcEdit;busaCalcEditing=false;busaCalcRecalculate();
  }else keyMessage="Введите число или ESC";}
  else {const int digitKeys[10]={17,4,5,6,8,9,10,12,13,14};for(int d=0;d<10;d++)if(k==digitKeys[d]&&busaCalcEdit.length()<12){
    busaCalcEdit=busaCalcEdit.substring(0,busaCalcCursor)+String(d)+busaCalcEdit.substring(busaCalcCursor);++busaCalcCursor;break;
  }}
 }else if(k==7||k==11){
  if(!tftView)busaCalcMode=(busaCalcMode+(k==7?2:1))%3;
  else busaCalcField=(busaCalcField+(k==7?busaCalcCounts[busaCalcMode]-1:1))%busaCalcCounts[busaCalcMode];
  busaCalcRecalculate();
 }else if(k==19){
  if(!tftView){tftView=1;busaCalcField=0;busaCalcRecalculate();}
  else{busaCalcEditing=true;busaCalcEdit=busaCalcInputs[busaCalcMode][busaCalcField];busaCalcCursor=busaCalcEdit.length();}
 }else if(k==15&&tftView)tftView=0;
 else if(k==0&&tftView){for(auto& s:busaCalcInputs[busaCalcMode])s="";busaCalcRecalculate();}
 else return false; // Global LEFT/RIGHT, ESC/back and F2 keep existing behavior.
 tftDirty=true;return true;
}
