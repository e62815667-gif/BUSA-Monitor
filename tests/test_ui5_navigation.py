"""Execute the real physical keypad controller with hardware calls stubbed."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE';main=(sketch/'BUSA_Main.h').read_text()
def function(signature):
 start=main.index(signature);pos=main.index('{',start);depth=1;end=pos+1
 while depth:
  if main[end]=='{':depth+=1
  elif main[end]=='}':depth-=1
  end+=1
 return main[start:end]
prefix=(root/'tests/test_calculator_keypad.py').read_text().split("code=r'''",1)[1].split('int currentPage=7',1)[0]
prefix=prefix.replace('String(int n):','String(double v,int n){char b[64];snprintf(b,sizeof(b),"%.*f",n,v);assign(b);}\n bool isEmpty()const{return empty();} float toFloat()const{return strtof(c_str(),nullptr);}\n String(int n):')
code='#include <cstdio>\n#include <cstdint>\n'+prefix+r'''
uint8_t currentPage=0,tftView=0,previousMainPage=0,approvedSettingsItem=0,settingsSubTab=0;
uint8_t graphMenuCursor=0,graphSubView=0,diagModule=0,calSensor=0,themeDraft=2,tftThemeMode=2;
uint8_t wifiUiMenuIndex=0,editCursor=0;
int wifiUiStage=0;const int WIFIUI_STATUS=0;
int selectedTemp=0,selectedField=0;
bool tftDirty=false,editingTemp=false,deleteConfirmPending=false,calEditing=false;
float calDraft=0,refTemp[4]={NAN,NAN,NAN,NAN},noticeTemp[4]={NAN,NAN,NAN,NAN},tempOffset[4]={};
String keyMessage,editTempBuffer;
#include "BUSA_CalculatorUI.h"
int wifiLeaves=0,scans=0,displayOff=0;
void wifiUiLeave(){++wifiLeaves;}
bool wifiUiKey(int){return false;}
void wifiUiBeginScan(){++scans;}
int wifiUiDigitFromKey(int k){return k==17?0:k>=4&&k<=14&&k!=7&&k!=11?1:-1;}
bool saveTempOffset(uint8_t,float){return true;}bool saveThemeMode(uint8_t){return true;}
bool storeTempReference(int,int,float){return true;}
void darkenTftDisplay(){++displayOff;}void deleteSelectedReference(){}
float constrain(float v,float lo,float hi){return std::max(lo,std::min(hi,v));}
void navigatePage(uint8_t);void moveApprovedTab(int);
'''
code+=function('void navigatePage(uint8_t target)')+'\n'
code+=re.search(r'const uint8_t UI_ORDER\[8\]=.*?;',main).group()+'\n'
code+=function('uint8_t orderSlot(uint8_t page)')+'\n'+function('void moveApprovedTab(int direction){')+'\n'+function('void keypadAction(int k)')
code+=r'''
int main(){ busaCalcDecimal();
 for(int i=1;i<=8;i++){keypadAction(18);assert(currentPage==i%8);}
 for(int i=7;i>=0;i--){keypadAction(16);assert(currentPage==i);}
 for(int page:{1,3,5,6,7}){
  navigatePage(page);int count=page==1?4:page==3?2:page==5?5:page==6?7:3;
  for(int i=0;i<count*2;i++){
   int selected=page==1?selectedTemp:page==3?graphMenuCursor:page==5?approvedSettingsItem:page==6?diagModule:busaCalcMode;
   assert(selected==i%count);keypadAction(11);
  }
  keypadAction(7);
  assert((page==1?selectedTemp:page==3?graphMenuCursor:page==5?approvedSettingsItem:page==6?diagModule:busaCalcMode)==count-1);
  keypadAction(19);assert(tftView==1);keypadAction(15);assert(tftView==0&&currentPage==page);
 }
 navigatePage(5);approvedSettingsItem=0;
 for(int i=0;i<5;i++){
  assert(approvedSettingsItem==i);keypadAction(19);assert(tftView==1);
  keypadAction(15);assert(tftView==0&&approvedSettingsItem==i);keypadAction(11);
 }
 assert(scans==1);
 navigatePage(1);selectedTemp=2;keypadAction(19);assert(tftView==1&&selectedField==0);
 keypadAction(11);assert(selectedField==1);keypadAction(7);assert(selectedField==0);
 keypadAction(15);assert(!tftView&&selectedTemp==2);
 navigatePage(0);navigatePage(1);assert(selectedTemp==0);
 navigatePage(3);graphMenuCursor=1;keypadAction(19);assert(graphSubView==1&&tftView==1);keypadAction(15);assert(!tftView);
 navigatePage(0);keypadAction(4);assert(currentPage==0);keypadAction(1);assert(displayOff==1);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'nav.cpp';p.write_text(code);exe=Path(d)/'nav'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(sketch),str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('PASS physical keypad: all 8 sections forward/reverse; all submenu rows and wrap; ENT/ESC; Settings behavior retained; temperature fields, graph choice, first-row entry, F2 retained. Hardware mocked.')
