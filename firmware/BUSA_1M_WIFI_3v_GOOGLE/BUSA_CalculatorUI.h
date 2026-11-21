#pragma once
#include "BUSA_Calculator.h"
#include <cstdint>
#include <cstdlib>
// Calculator values stay in RAM; only cylindrical vessel geometry is stored in NVS.
static uint8_t busaCalcMode=0,busaCalcField=0;
static bool busaCalcEditing=false,busaCalcComplete=false;
static String busaCalcInputs[4][5],busaCalcEdit="";
static uint8_t busaCalcCursor=0;
static BusaCalc::Result busaCalcResult;
static const char* const busaCalcNames[4]={"Разбавление","Ареометр","Вторая перегонка","Объём куба"};
static const uint8_t busaCalcCounts[4]={3,2,5,3};
static const char* const busaCalcLabels[4][5]={
 {"Объём, л","Крепость, %","Нужная, %","",""},
 {"Температура, °C","Показание, %","","",""},
 {"Объём, л","Крепость, %","На выходе, %","Головы, % АС","Хвосты, % АС"},
 {"Диаметр внутр., см","Высота внутр., см","Уровень, см","",""}
};
static bool busaCalcParse(String text,double& out){
 text.trim();text.replace(',','.');if(!text.length())return false;
 bool digit=false,dot=false;for(unsigned i=0;i<text.length();i++){
  char c=text[i];if(c>='0'&&c<='9')digit=true;else if(c=='.'&&!dot)dot=true;else return false;
 }
 if(!digit)return false;
 char* end=nullptr;out=strtod(text.c_str(),&end);return end&&!*end&&isfinite(out);
}
// Geometry is a single NVS record; level is deliberately session-only.
static bool busaCalcGeometryValid(){
 double diameter=0,height=0;
 return busaCalcParse(busaCalcInputs[3][0],diameter)&&busaCalcParse(busaCalcInputs[3][1],height)
   &&BusaCalc::cylinder(diameter,height).ok;
}
static inline void busaCalcLoadGeometry(){
 if(!busaNvsReady)return;
 const String record=prefs.getString("cubeDims","");
 if(record.length()>25)return;
 const int split=record.indexOf(';');
 if(split<=0||split>=int(record.length())-1)return;
 busaCalcInputs[3][0]=record.substring(0,split);
 busaCalcInputs[3][1]=record.substring(split+1);
 if(!busaCalcGeometryValid()){busaCalcInputs[3][0]="";busaCalcInputs[3][1]="";}
}
static bool busaCalcStoreGeometry(){
 if(!busaCalcGeometryValid()||!busaNvsReady)return false;
 const String record=busaCalcInputs[3][0]+";"+busaCalcInputs[3][1];
 return prefs.putString("cubeDims",record)==record.length();
}
static void busaCalcRecalculate(){
 double v[5]={};busaCalcComplete=false;busaCalcResult=BusaCalc::Result();
 if(busaCalcMode==3){
  if(!busaCalcParse(busaCalcInputs[3][0],v[0])||!busaCalcParse(busaCalcInputs[3][1],v[1]))return;
  if(!busaCalcInputs[3][2].length())busaCalcResult=BusaCalc::cylinder(v[0],v[1]);
  else if(busaCalcParse(busaCalcInputs[3][2],v[2]))busaCalcResult=BusaCalc::cylinder(v[0],v[1],v[2]);
  busaCalcComplete=true;return;
 }
 for(unsigned i=0;i<busaCalcCounts[busaCalcMode];i++)if(!busaCalcParse(busaCalcInputs[busaCalcMode][i],v[i]))return;
 busaCalcComplete=true;
 if(busaCalcMode==0)busaCalcResult=BusaCalc::dilute(v[0],v[1],v[2]);
 else if(busaCalcMode==1)busaCalcResult=BusaCalc::correct(v[0],v[1]);
 else if(busaCalcMode==2)busaCalcResult=BusaCalc::fraction(v[0],v[1],v[2],v[3],v[4]);
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
  else if(k==19){double value=0;if(busaCalcParse(busaCalcEdit,value)
    &&(busaCalcMode!=3||busaCalcField==2||value>0)){
    busaCalcInputs[busaCalcMode][busaCalcField]=busaCalcEdit;busaCalcEditing=false;busaCalcRecalculate();
    if(busaCalcMode==3&&busaCalcField<2&&busaCalcGeometryValid()&&!busaCalcStoreGeometry())
      keyMessage="Ошибка сохранения размеров";
  }else keyMessage="Введите корректное число или ESC";}
  else {const int digitKeys[10]={17,4,5,6,8,9,10,12,13,14};for(int d=0;d<10;d++)if(k==digitKeys[d]&&busaCalcEdit.length()<12){
    busaCalcEdit=busaCalcEdit.substring(0,busaCalcCursor)+String(d)+busaCalcEdit.substring(busaCalcCursor);++busaCalcCursor;break;
  }}
 }else if(k==7||k==11){
  if(!tftView)busaCalcMode=(busaCalcMode+(k==7?3:1))%4;
  else busaCalcField=(busaCalcField+(k==7?busaCalcCounts[busaCalcMode]-1:1))%busaCalcCounts[busaCalcMode];
  busaCalcRecalculate();
 }else if(k==19){
  if(!tftView){tftView=1;busaCalcField=(busaCalcMode==3&&busaCalcGeometryValid())?2:0;busaCalcRecalculate();}
  else{busaCalcEditing=true;busaCalcEdit=busaCalcInputs[busaCalcMode][busaCalcField];busaCalcCursor=busaCalcEdit.length();}
 }else if(k==15&&tftView)tftView=0;
 else if(k==0&&tftView){if(busaCalcMode==3)busaCalcInputs[3][2]="";else for(auto& s:busaCalcInputs[busaCalcMode])s="";busaCalcRecalculate();}
 else return false; // Global LEFT/RIGHT, ESC/back and F2 keep existing behavior.
 tftDirty=true;return true;
}
