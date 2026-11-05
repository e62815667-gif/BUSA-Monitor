"""Execute the shipping RAM-only calculator keyboard controller against String stubs."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
code=r'''
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cassert>
using std::isfinite;
class String:public std::string{
 public:using std::string::string;using std::string::operator=;
 String(const std::string& s):std::string(s){}String(int n):std::string(std::to_string(n)){}
 void trim(){auto a=find_first_not_of(" \r\n\t");auto b=find_last_not_of(" \r\n\t");assign(a==npos?"":substr(a,b-a+1));}
 void replace(char a,char b){std::replace(begin(),end(),a,b);}
 int indexOf(char c)const{auto p=find(c);return p==npos?-1:int(p);}
 String substring(size_t a,size_t b=std::string::npos)const{return substr(a,b==std::string::npos?b:b-a);}
};
int currentPage=7,tftView=0;bool tftDirty=false;String keyMessage;
#include "BUSA_CalculatorUI.h"
void enter(int digit){int keys[10]={17,4,5,6,8,9,10,12,13,14};assert(busaCalcKey(keys[digit]));}
void editNumber(const char* s){busaCalcKey(19);for(const char* p=s;*p;p++){if(*p==',')busaCalcDecimal();else enter(*p-'0');}busaCalcKey(19);}
int main(){
 assert(!busaCalcParse("",busaCalcResult.water));assert(!busaCalcParse("1,2,3",busaCalcResult.water));
 assert(!busaCalcParse("nan",busaCalcResult.water));assert(!busaCalcParse("1e9",busaCalcResult.water));
 busaCalcKey(19);assert(tftView==1);editNumber("1");busaCalcKey(11);editNumber("96");busaCalcKey(11);editNumber("40");
 assert(busaCalcComplete&&busaCalcResult.ok&&fabs(busaCalcResult.water-1.4705338867)<1e-8);
 busaCalcKey(19);busaCalcKey(0);enter(5);busaCalcKey(15);assert(busaCalcInputs[0][2]=="40");
 busaCalcKey(19);busaCalcKey(0);enter(3);busaCalcDecimal();busaCalcDecimal();enter(5);assert(busaCalcEdit=="3,5");
 busaCalcKey(19);assert(busaCalcInputs[0][2]=="3,5");
 assert(!busaCalcKey(18));assert(!busaCalcKey(1)); // retain global section navigation / F2
 busaCalcKey(0);assert(!busaCalcComplete&&!busaCalcResult.ok);
 busaCalcKey(15);assert(tftView==0);busaCalcKey(11);assert(busaCalcMode==1);
 busaCalcKey(19);editNumber("20");busaCalcKey(11);editNumber("40");assert(busaCalcResult.ok&&busaCalcResult.strength==40);
 currentPage=0;assert(!busaCalcKey(19));
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'key.cpp';p.write_text(code);exe=Path(d)/'key'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(sketch),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS actual calculator keypad: menu/fields, digits, long-zero comma, save/cancel/clear, invalid input, global navigation/F2 retained; no ESP32 tested')
