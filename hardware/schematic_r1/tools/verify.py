"""Verify EXPORTED KiCad connectivity against firmware and independent design constraints."""
from pathlib import Path
import json,re,xml.etree.ElementTree as ET,csv,math,subprocess,os,tempfile,shutil
root=Path(__file__).resolve().parents[1];repo=root.parents[1];out=root/'output'
s=(repo/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
x=ET.parse(out/'NETLIST.xml').getroot();nets={};pin_net={};values={c.attrib['ref']:c.findtext('value') for c in x.find('components')}
for net in x.find('nets'):
 name=net.attrib['name']
 if name.startswith('unconnected-'):continue
 nodes={(n.attrib['ref'],n.attrib['pin']) for n in net.findall('node')};nets[name]=nodes
 for node in nodes:
  assert node not in pin_net,('pin appears in two nets',node);pin_net[node]=name
checks=[]
def same(*nodes):
 names={pin_net[n] for n in nodes};assert len(names)==1,(nodes,names)
def diff(*nodes):assert len({pin_net[n] for n in nodes})==len(nodes)
def add(t):checks.append(t)
assert len(x.find('components'))==71
assert re.search(r'TPIN\[4\]=\{16,17,18,21\}',s)
assert re.search(r'KPIN\[4\]=\{7,6,5,4\}',s)
assert 'BUS_SDA=40,BUS_SCL=41' in s and 'FLOW_GPIO=15' in s
# Source GPIO -> actual board header -> connector, including independent pull-ups.
for gpio,pin,j,res in [(16,'R14','J6','R30'),(18,'R12','J7','R31'),(17,'R13','J8','R32'),(21,'L05','J9','R33')]:
 same(('U1',pin),(j,'2'),(res,'2'));same((j,'1'),('U1','R21'),(res,'1'));same((j,'3'),('J1','2'))
 assert values[res]=='4.7k'
add('Four temperature sensors direct to GPIO16/18/17/21; separate 4.7k pullups to 3.3V; 3-pin VDD/DQ/GND connectors')
assert len([p for p in pin_net if p[0]=='J1'])==2;same(('J1','1'),('F1','1'));same(('F1','2'),('D1','A'));same(('D1','K'),('U2','1'),('JP2','1'))
same(('U2','3'),('JP1','1'),('J4','6'));same(('JP1','2'),('U1','R02'))
for n in [('U2','2'),('U1','L01'),('U1','L02'),('U1','L22'),('U1','R01')]:same(n,('J1','2'))
diff(('J1','1'),('J1','2'),('U2','3'),('U1','R21'),('U1','R02'))
add('Exactly two DC input pins; protected 12V,5V,3.3V,GND separate; ESP external 5V has removable JP1')
# TFT exact firmware CS/DC/RST/MOSI/SCK=10/11/12/13/14
assert 'TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14' in s
for jpin,esp in [('1','R01'),('2','R21'),('3','R03'),('4','R04'),('5','R05'),('6','R06'),('7','R07')]:same(('J2',jpin),('U1',esp))
add('Display GND/VCC3.3/SCK14/MOSI13/RST12/DC11/CS10 matches UI12 and user-confirmed VCC')
same(('U3','VCC'),('U1','R21'));same(('U3','GND'),('J1','2'));same(('U3','SDA'),('U1','L15'));same(('U3','SCL'),('U1','L16'))
for i in range(1,10):same(('J3',str(i)),('U3','P0'+str(i-1) if i<9 else 'P10'))
assert ('U3','INT') not in pin_net
for i in range(1,8):assert ('U3','P1'+str(i)) not in pin_net
assert set(pin_net[('J3',str(i))] for i in range(1,10)).isdisjoint({'V3V3','V5','GND'})
add('PCF8575 module: 3.3V, SDA40/SCL41, INT NC; 9 matrix signals P00..P07/P10; no supply in keypad ribbon')
for i,(esp,a,y) in enumerate(zip(['R16','R17','R18','R19'],['1','3','5','9'],['2','4','6','8']),1):
 same(('U1',esp),('U4',a),('R'+str(10+i),'2'));same(('U4',y),('J4',str(i+1)),('R'+str(20+i),'2'))
 assert pin_net[('U1',esp)]!=pin_net[('J4',str(i+1))]
 same(('R'+str(10+i),'1'),('U1','R21'));same(('R'+str(20+i),'1'),('U2','3'))
add('Four active-low outputs preserve GPIO7/6/5/4 through noninverting open-drain buffer; boot pullups and isolated 5V pullups')
same(('JP2','2'),('J5','1'));same(('J5','2'),('J1','2'));assert pin_net[('J5','1')]!=pin_net[('J4','6')]
add('Relay VCC5 and JD12 separate; removable JD supply link; shared low-voltage ground')
for prefix,j,q,resbase,respull,uA,uY,esp in [('COOL','J10','Q1','R42','R44','1','6','R15'),('COL','J11','Q2','R52','R54','3','4','L19')]:
 same((j,'3'),(resbase,'1'));same((resbase,'2'),(q,'1'));same((q,'2'),('J1','2'));same((q,'3'),(respull,'2'),('U5',uA));same((respull,'1'),('U1','R21'))
 same(('U5',uY),('R45' if prefix=='COOL' else 'R55','1'));same(('R45' if prefix=='COOL' else 'R55','2'),('U1',esp))
 assert pin_net[(j,'3')]!=pin_net[('U1',esp)]
 same((j,'2'),('J1','2'))
add('Cooling and collection independent; NPN+Schmitt level conversion; coolingGPIO15, collectionGPIO1 reserve only')
for esp in ['L03','L04','L06','L07','L08','L09','L10','L11','L12','R08','R09','R10','R11']:
 assert ('U1',esp) not in pin_net
add('USB19/20, PSRAM35/36/37, boot pins and unused pins left NC, no GPIO collision')
# Nominal/1% resistor margin, deliberately stated as a calculation not a bench test.
rail_hi=3.3*1.05;raw_lo=4.5;vbe=.85
base_min=(raw_lo-vbe)/(22000*1.01+10000*1.01)-vbe/(100000*.99)
collector_max=(rail_hi-.2)/(4700*.99)
assert base_min>0 and collector_max/base_min<10
assert 4700*1.01*1e-9*1.1<6e-6
add('Pulse input calculation: forced beta <10 at 4.5V source with pullup; RC tau <6us versus 588Hz max nominal YF-S401 cadence')
# KiCad must really reject a conflict; do not merely report a clean report from passives.
env={**os.environ,'XDG_CONFIG_HOME':os.environ.get('XDG_CONFIG_HOME',str(root/'.runtime/config')),'XDG_CACHE_HOME':os.environ.get('XDG_CACHE_HOME',str(root/'.runtime/cache')),'XDG_DATA_HOME':os.environ.get('XDG_DATA_HOME',str(root/'.runtime/data'))}
with tempfile.TemporaryDirectory() as d:
 d=Path(d)
 for name in ['BUSA_R1.kicad_pro','sym-lib-table','BUSA.kicad_sym']:shutil.copy(root/name,d/name)
 bad=(root/'BUSA_R1.kicad_sch').read_text().replace('(label "K1_GPIO7"','(label "TFT_SCK"',1);(d/'BUSA_R1.kicad_sch').write_text(bad)
 r=subprocess.run(['kicad-cli','sch','erc','--format','json','--exit-code-violations','-o',str(d/'fault.json'),str(d/'BUSA_R1.kicad_sch')],env=env,capture_output=True,text=True)
 j=json.loads((d/'fault.json').read_text());v=[v for sh in j['sheets'] for v in sh['violations']]
 assert r.returncode!=0 and any(v['severity']=='error' and v['type']=='pin_to_pin' for v in v),v
 (out/'ERC_negative_control.json').write_text(json.dumps(j,ensure_ascii=False,indent=2))
add('Negative control: intentional short between GPIO7 and GPIO14 is rejected by KiCad ERC (output conflict)')
j=json.loads((out/'ERC.json').read_text());assert not [v for sh in j['sheets'] for v in sh['violations']]
report={'result':'PASS_DESIGN_CHECKS_NOT_BENCH_APPROVAL','checks':checks,'component_count':71,'calculations':{'base_min_A':base_min,'collector_max_A':collector_max,'forced_beta':collector_max/base_min},'outstanding':['Relay module JD-VCC/header return continuity and 3.3V rail current measurement','Physical footprint dimensions and connector orientation','Exact collection sensor and firmware implementation']}
(out/'CHECKS.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
with (out/'PIN_NETS.csv').open('w') as f:
 w=csv.writer(f,lineterminator="\n");w.writerow(['Reference','Pin','Net'])
 for (ref,pin),net in sorted(pin_net.items()):w.writerow([ref,pin,net])
print('PASS',len(checks),'independent netlist checks; KiCad ERC 0; deliberate conflict detected')
