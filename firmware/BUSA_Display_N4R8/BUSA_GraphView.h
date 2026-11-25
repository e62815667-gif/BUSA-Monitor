#pragma once
#include <ArduinoJson.h>
#include <cmath>
#include <cstdint>
namespace busa_graph {
struct Spec {const char* title;const char* key;unsigned series;uint32_t color;};
inline Spec spec(unsigned kind){
    switch(kind){
      case 1:return {"Отбор, мл/мин","collection_ml_min",1,0xf4c84a};
      case 2:return {"Охлаждение, л/ч","cooling_lph",1,0x38bdf8};
      default:return {"Температура, °C","temperature",4,0x38bdf8};
    }
}
inline JsonArrayConst samples(JsonObjectConst doc,unsigned kind,unsigned index){
    if(index>=spec(kind).series)return JsonArrayConst();
    return kind==0?doc["temperature"][index].as<JsonArrayConst>():doc[spec(kind).key].as<JsonArrayConst>();
}
inline bool valid(JsonVariantConst v){
    return v.is<double>()&&std::isfinite(v.as<double>());
}
struct Scale {unsigned count=0,points=0;double high=125;};
inline bool analyze(JsonObjectConst doc,unsigned kind,int minutes,Scale& s){
    s=Scale();s.high=kind==0?125:10;
    if(!(doc["ok"]|false)||(doc["minutes"]|0)!=minutes||!doc["temperature"].is<JsonArrayConst>())return false;
    unsigned expected=minutes==5?150:160;
    for(unsigned k=0;k<spec(kind).series;++k){
        auto a=samples(doc,kind,k);
        if(a.isNull()||a.size()!=expected)return false;
        for(auto v:a)if(valid(v)){++s.points;if(kind!=0&&v.as<double>()>s.high)s.high=v.as<double>();}
    }
    s.count=expected;
    if(kind!=0)s.high=std::ceil(s.high/10.0)*10;
    return std::isfinite(s.high)&&s.high>0;
}
inline int coordinate(double value,double high){
    if(value<=0)return 0;if(value>=high)return 1000;
    return int(std::lround(value/high*1000));
}
inline const char* state(JsonObjectConst doc,unsigned kind,const Scale& s){
    if(kind==1&&!(doc["collection_sensor"]|false))return "Нет импульсов отбора";
    if(!s.points)return kind==2?"Нет импульсов охлаждения":"Ожидание измерений";
    if(kind==2&&!(doc["cooling_calibrated"]|false))return "Расчётно · без калибровки";
    return "Данные с включения";
}
}
