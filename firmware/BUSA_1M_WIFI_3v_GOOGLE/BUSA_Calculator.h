#pragma once
#include <cmath>
#include "BUSA_CalcCoefficients.h"
namespace BusaCalc {
struct Result {bool ok=false;double water=0,total=0,strength=0,absolute=0,heads=0,tails=0,product=0;};
inline double density(double p,double t){double r=0;for(const auto& row:BUSA_CALC_COEFFICIENTS){double v=0;for(double c:row)v=v*(t-20)+c;r=r*p+v;}return r;}
inline double abvFromMass(double p){return 100*p*density(p,20)/density(1,20);}
inline double massFromAbv(double a){double lo=0,hi=1;for(int k=0;k<48;k++){double p=(lo+hi)/2;if(abvFromMass(p)<a)lo=p;else hi=p;}return (lo+hi)/2;}
inline Result dilute(double v,double from,double to){
 Result r;if(!std::isfinite(v)||!std::isfinite(from)||!std::isfinite(to)||v<=0||v>100000||from<=0||from>100||to<=0||to>from)return r;
 r.ok=true;if(to==from){r.total=v;return r;}
 double p=massFromAbv(from),q=massFromAbv(to),mass=v*density(p,20),finalMass=mass*p/q;
 r.water=(finalMass-mass)/density(0,20);r.total=finalMass/density(q,20);return r;
}
inline Result correct(double t,double reading){
 Result r;if(!std::isfinite(t)||!std::isfinite(reading)||t<0||t>40||reading<0||reading>100)return r;
 if(t==20){r.ok=true;r.strength=reading;return r;}
 double target=density(massFromAbv(reading),20)/(1+0.000025*(t-20));
 if(target>density(0,t)+1e-8||target<density(1,t)-1e-8)return r;
 double lo=0,hi=1;for(int k=0;k<48;k++){double p=(lo+hi)/2;if(density(p,t)>target)lo=p;else hi=p;}
 r.ok=true;r.strength=abvFromMass((lo+hi)/2);return r;
}
inline Result fraction(double v,double strength,double out,double head,double tail){
 Result r;if(!std::isfinite(v)||!std::isfinite(strength)||!std::isfinite(out)||!std::isfinite(head)||!std::isfinite(tail)||v<=0||v>100000||strength<=0||strength>100||out<=0||out>100||head<0||tail<0||head+tail>=100)return r;
 r.ok=true;r.absolute=v*strength/100;r.heads=r.absolute*head/100;r.tails=r.absolute*tail/100;r.product=(r.absolute-r.heads-r.tails)*100/out;return r;
}
}
