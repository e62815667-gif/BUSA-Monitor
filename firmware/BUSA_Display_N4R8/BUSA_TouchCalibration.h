#pragma once
#include <stdint.h>
#include <cmath>
namespace busa_touch {
struct Point {int x,y;};
struct Transform {double a,b,c,d,e,f;};
inline bool fit(const Point* r,const Point* s,Transform& t){
    const double x1=r[1].x-r[0].x,y1=r[1].y-r[0].y,x2=r[2].x-r[0].x,y2=r[2].y-r[0].y;
    const double det=x1*y2-x2*y1;
    if(std::fabs(det)<10000)return false;
    const double u1=s[1].x-s[0].x,u2=s[2].x-s[0].x,v1=s[1].y-s[0].y,v2=s[2].y-s[0].y;
    Transform q{};
    q.a=(u1*y2-u2*y1)/det;q.b=(x1*u2-x2*u1)/det;q.c=s[0].x-q.a*r[0].x-q.b*r[0].y;
    q.d=(v1*y2-v2*y1)/det;q.e=(x1*v2-x2*v1)/det;q.f=s[0].y-q.d*r[0].x-q.e*r[0].y;
    if(!std::isfinite(q.a)||!std::isfinite(q.b)||!std::isfinite(q.c)||!std::isfinite(q.d)||!std::isfinite(q.e)||!std::isfinite(q.f))return false;
    if(std::fabs(q.a*q.e-q.b*q.d)<0.0001)return false;
    t=q;return true;
}
inline Point apply(const Transform& t,Point p){
    return {int(std::lround(t.a*p.x+t.b*p.y+t.c)),int(std::lround(t.d*p.x+t.e*p.y+t.f))};
}
}
