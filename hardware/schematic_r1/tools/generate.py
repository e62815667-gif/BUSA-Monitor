from draw import *
note('БУСА R1 — электрическая схема модульной несущей платы',15,15,3)
note('СВЕРКА С UI12 / ERC / ПОДГОТОВКА К МАКЕТУ. На устройстве не испытано.',15,23,2)
note('Назначения выводов по маркировке; посадочные места и обратные стороны модулей ещё требуется проверить.',15,30)
note('1. Питание: 12 В, два контакта',15,44,2)
conn('J1','DC INPUT / 12V ONLY',[('+12V','VIN12'),('GND','GND')],35,65)
flag('PF1','VIN12',105,60);flag('PF2','GND',105,80)
two('F1','1A slow / 32V','VIN12','VIN12_FUSED',40,103)
part('D1','B340A-13-F / SMA',110,103,[('A','ANODE','passive','L','VIN12_FUSED'),('K','CATHODE','passive','R','V12_PROT')],width=18,url='https://www.diodes.com/datasheet/download/B340A.pdf',desc='Exact SMA Schottky choice; do not substitute a vendor SS34 in SMC')
flag('PF3','V12_PROT',175,103)
part('U2','R-78E5.0-1.0',75,135,[('1','VIN','power_in','L','V12_PROT'),('2','GND','power_in','L','GND'),('3','5V OUT','power_out','R','V5')],width=32,url='https://recom-power.com/pdf/Innoline/R-78E-1.0.pdf',desc='New selected regulator option, not claimed existing user component')
cap('C1','10uF / 35V','V12_PROT',40,167);cap('C2','100nF / 50V','V12_PROT',130,167)
cap('C3','10uF / 10V','V5',40,192);cap('C4','100nF','V5',130,192)
flag('PF4','ESP_5V',170,220)
two('JP1','ESP PWR LINK / OPEN','V5','ESP_5V',75,220,'Open before either ESP32 USB port is powered. Do not connect external 5V and USB concurrently.')
note('JP1 снять перед питанием/прошивкой от USB.',15,243)
note('3,3 В — от ESP32. Ток с экраном измерить.',15,251)
note('12 В не подавать на VCC логики или GPIO.',15,259)
# Actual archived 44-pin header arrangement, USB at top, front face toward user.
note('2. ESP32-S3 N16R8: 44 контакта',230,44,2)
L=['GND','GND','GPIO19','GPIO20','GPIO21','GPIO47','GPIO48','GPIO45','GPIO0','GPIO35','GPIO36','GPIO37','GPIO38','GPIO39','GPIO40','GPIO41','GPIO42','GPIO2','GPIO1','RX','TX','GND']
R=['GND','5V','GPIO14','GPIO13','GPIO12','GPIO11','GPIO10','GPIO9','GPIO46','GPIO3','GPIO8','GPIO18','GPIO17','GPIO16','GPIO15','GPIO7','GPIO6','GPIO5','GPIO4','RST','3V3','3V3']
used={'GPIO21':('TEMP_ALARM','bidirectional'),'GPIO40':('I2C_SDA','bidirectional'),'GPIO41':('I2C_SCL','bidirectional'),'GPIO2':('BUZZER_SIG','output'),'GPIO1':('COLLECT_GPIO1','input'),'GPIO14':('TFT_SCK','output'),'GPIO13':('TFT_MOSI','output'),'GPIO12':('TFT_RST','output'),'GPIO11':('TFT_DC','output'),'GPIO10':('TFT_CS','output'),'GPIO18':('TEMP_CUBE','bidirectional'),'GPIO17':('TEMP_COLUMN','bidirectional'),'GPIO16':('TEMP_WATER','bidirectional'),'GPIO15':('COOL_GPIO15','input'),'GPIO7':('K1_GPIO7','output'),'GPIO6':('K2_GPIO6','output'),'GPIO5':('K3_GPIO5','output'),'GPIO4':('K4_GPIO4','output')}
p=[]
for side,rows in [('L',L),('R',R)]:
 for i,name in enumerate(rows):
  net,typ=used.get(name,(None,'passive'))
  if name=='GND':net,typ='GND','power_in'
  if name=='5V':net,typ='ESP_5V','power_in'
  if name=='3V3':net,typ='V3V3',('power_out' if i==20 else 'passive')
  p.append((side+str(i+1).zfill(2),name,typ,side,net))
part('U1','ESP32-S3 / user 44PIN',295,117,p,width=42,desc='L/R numbered top-to-bottom with both USB at top. Actual socket geometry still to measure.')
note('GPIO1 — резерв. UI12 ещё не считывает отбор.',230,179)
note('USB 19/20, PSRAM 35–37 и boot-пины оставлены свободными.',230,187,1.4)
conn('J12','BUZZER LOGIC ONLY',[('GPIO2','BUZZER_SIG'),('GND','GND')],270,214,'Logic control only; not direct supply for a bare buzzer')
# Display
note('3. Экран GMT028-05 V1.1',450,44,2)
conn('J2','TFT / VCC=3.3V',[('GND','GND'),('VCC','V3V3'),('SCK','TFT_SCK'),('SDA_MOSI','TFT_MOSI'),('RST','TFT_RST'),('DC','TFT_DC'),('CS','TFT_CS')],495,82,'New carrier pin order matches photographed display legends; confirm cable face/orientation')
cap('C5','100nF','V3V3',470,123);cap('C6','10uF / 6.3V','V3V3',560,123)
note('VCC=3,3 В подтверждено владельцем.',450,148)
note('GPIO взяты из UI12, не из старых схем октября.',450,156,1.4)
# Keypad expander module
note('4. PCF8575 — готовый съёмный модуль',230,278,2)
p=[('VCC','VCC','power_in','L','V3V3'),('GND','GND','power_in','L','GND'),('SDA','SDA','bidirectional','L','I2C_SDA'),('SCL','SCL','input','L','I2C_SCL'),('INT','INT','open_collector','L',None)]
for i in range(8):p.append(('P0'+str(i),'P0'+str(i),'bidirectional','R','KEY'+str(i+1)))
p.append(('P10','P10','bidirectional','R','KEY9'))
for i in range(1,8):p.append(('P1'+str(i),'P1'+str(i),'bidirectional','R',None))
part('U3','PCF8575 MODULE / 8+8+5',280,330,p,width=33,desc='Pin numbers are actual printed terminal names, not invented physical positions. INT and 7 unused I/O are NC.')
conn('J3','KEYPAD 9-WIRE',[(str(i),'KEY'+str(i)) for i in range(1,10)],380,316,'Harness 1..8 -> P00..P07; 9 -> P10. Preserve installed harness mapping or retrain.')
resistor('R1','4.7k OPTIONAL','V3V3','I2C_SDA',250,380,'Measure existing module pullups first; effective target 2.2k..10k to 3.3V only')
resistor('R2','4.7k OPTIONAL','V3V3','I2C_SCL',355,380,'Measure existing module pullups first; no 5V pullup')
cap('C7','100nF','V3V3',250,405);cap('C8','4.7uF','V3V3',355,405)
note('Шлейф клавиатуры: девять сигнальных линий.',230,428)
note('Нумерацию и ориентацию шлейфа сверить. Питания в нём нет.',230,436,1.4)
# Direct temperature probes
note('5. Температуры: напрямую к ESP32',15,278,2)
for i,(name,net) in enumerate([('WATER','TEMP_WATER'),('CUBE','TEMP_CUBE'),('COLUMN','TEMP_COLUMN'),('ALARM','TEMP_ALARM')]):
 x=40 if i<2 else 140;y=310+(i%2)*72
 conn('J'+str(6+i),'DS18B20 '+name,[('VDD','V3V3'),('DQ',net),('GND','GND')],x,y,'Carrier standard: 1=VDD,2=DQ,3=GND; identify actual cable conductors before crimping')
 resistor('R'+str(30+i),'4.7k','V3V3',net,x,y+25);cap('C'+str(10+i),'100nF','V3V3',x,y+48)
note('Четыре независимые линии DQ, четыре подтяжки.',15,450,1.5)
note('3-проводное питание, не паразитный режим.',15,458,1.5)
note('Цвета жил датчиков не заменяют проверку распиновки.',15,466,1.4)
# Relay signals
note('6. Реле: буфер сохраняет активный LOW',645,44,2)
p=[]
for i,(a,y) in enumerate(zip([1,3,5,9,11,13],[2,4,6,8,10,12]),1):
 p.extend([(str(a),str(i)+'A','input','L',f'K{i}_GPIO{8-i}' if i<=4 else 'GND'),(str(y),str(i)+'Y OD','open_collector','R',f'RELAY_IN{i}' if i<=4 else None)])
p.extend([('14','VCC','power_in','L','V3V3'),('7','GND','power_in','L','GND')])
part('U4','SN74LVC07AD / SOIC14',685,103,p,width=30,url='https://www.ti.com/lit/ds/symlink/sn74lvc07a.pdf',desc='Noninverting open-drain; outputs tolerate 5.5V; Ioff; GPIO low => sink relay IN low')
conn('J4','RELAY CTRL / 6',[('GND','GND'),('IN1','RELAY_IN1'),('IN2','RELAY_IN2'),('IN3','RELAY_IN3'),('IN4','RELAY_IN4'),('VCC','V5')],790,92,'Matches photographed legends GND IN1 IN2 IN3 IN4 VCC. Check physical orientation.')
for i in range(1,5):
 x=665 if i<=2 else 775;y=160+((i-1)%2)*24
 resistor('R'+str(10+i),'10k','V3V3',f'K{i}_GPIO{8-i}',x,y,'Reset pullup: relay OFF')
 resistor('R'+str(20+i),'4.7k','V5',f'RELAY_IN{i}',x,y+65,'Logic pullup only; not coil supply')
cap('C9','100nF','V3V3',665,280)
two('JP2','JD12 LINK / OPEN','V12_PROT','RELAY_JD12',775,280,'Identify actual JD-VCC and remove module VCC/JD jumper BEFORE closing')
conn('J5','COIL SUPPLY / 2',[('JD_VCC','RELAY_JD12'),('GND_RETURN','GND')],720,315,'New carrier connector; actual relay-board JD header pin order/return requires measurement')
note('J4 — 6 контактов логики. J5 — 2 линии питания катушек.',645,343,1.4)
note('На модуле снять перемычку VCC/JD-VCC. Нельзя замкнуть 12 В с 5 В.',645,351,1.4)
note('JP2 по умолчанию разомкнут до проверки маркировки и питания.',645,359,1.4)
# Pulse conditioning; two independent channels; no native firmware added for collection.
note('7. Два импульсных входа: защита уровня + триггер Шмитта',450,278,1.8)
# Place two channels in lower half with one dual Schmitt in middle.
for ch,ref,x in [('COOL','J10',465),('COL','J11',655)]:
 y=401
 conn(ref,ch+' PULSE / 3',[('V_SENSOR',ch+'_PWR'),('GND','GND'),('PULSE',ch+'_RAW')],x,y,'Carrier order 1=power,2=GND,3=pulse. Cooling: YF-S401. Collection model unspecified.')
 two('JP3' if ch=='COOL' else 'JP4',ch+' PWR LINK / OPEN','V5',ch+'_PWR',x,437,'Populate only after verifying sensor supply; collection model remains open')
 resistor('R41' if ch=='COOL' else 'R51','10k',ch+'_PWR',ch+'_RAW',x,461,'Pullup supports open-collector sensor; inspect push-pull compatibility')
 resistor('R42' if ch=='COOL' else 'R52','22k',ch+'_RAW',ch+'_BASE',x,485)
 part('Q1' if ch=='COOL' else 'Q2','BC847B / SOT23',x,516,[('1','B','passive','L',ch+'_BASE'),('2','E','passive','L','GND'),('3','C','passive','R',ch+'_COL')],width=20,url='https://assets.nexperia.com/documents/data-sheet/BC847X_SER.pdf')
 resistor('R43' if ch=='COOL' else 'R53','100k',ch+'_BASE','GND',x,550)
 resistor('R44' if ch=='COOL' else 'R54','4.7k','V3V3',ch+'_COL',x+90,460)
 cap('C14' if ch=='COOL' else 'C15','1nF',ch+'_COL',x+90,484)
 # antiparallel B-E diode, cathode at base, anode ground
 part('D2' if ch=='COOL' else 'D3','1N4148W',x+90,511,[('A','ANODE','passive','L','GND'),('K','CATHODE','passive','R',ch+'_BASE')],width=20)
part('U5','SN74LVC2G14DBV / SOT23-6',290,505,[('1','1A','input','L','COOL_COL'),('3','2A','input','L','COL_COL'),('5','VCC','power_in','L','V3V3'),('2','GND','power_in','L','GND'),('6','1Y','output','R','COOL_BUF'),('4','2Y','output','R','COL_BUF')],width=35,url='https://www.ti.com/lit/ds/symlink/sn74lvc2g14.pdf')
resistor('R45','330','COOL_BUF','COOL_GPIO15',310,541);resistor('R55','330','COL_BUF','COLLECT_GPIO1',310,565)
cap('C16','100nF','V3V3',195,531)
note('GPIO15: охлаждение. GPIO1: аппаратный резерв отбора.',15,501,1.5)
note('У входа два инвертора: Q + U5. Полярность импульса сохраняется.',15,510,1.4)
note('Отбор UI12 не поддерживает; нужна отдельная доработка ПО.',15,519,1.5)
note('Все земли общие. Гальваническая развязка не заявлена.',15,570,1.6)
note('R1: до разводки PCB подтвердить размеры модулей, распиновку JD-VCC, ток 3,3 В и датчик отбора.',15,581,1.5)
write()
