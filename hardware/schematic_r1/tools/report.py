from pathlib import Path
import json,csv,fitz
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,PageBreak
from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.pagesizes import A4
from xml.sax.saxutils import escape
root=Path(__file__).resolve().parents[1];out=root/'output';parts=json.loads((root/'parts.json').read_text());checks=json.loads((out/'CHECKS.json').read_text())
for n,f in [('DJ','DejaVuSans.ttf'),('DJB','DejaVuSans-Bold.ttf')]:pdfmetrics.registerFont(TTFont(n,'/usr/share/fonts/truetype/dejavu/'+f))
style=getSampleStyleSheet();style.add(ParagraphStyle(name='BodyRU',fontName='DJ',fontSize=9,leading=13,spaceAfter=7));style.add(ParagraphStyle(name='TitleRU',fontName='DJB',fontSize=21,leading=26,textColor=colors.HexColor('#16436b'),spaceAfter=10));style.add(ParagraphStyle(name='HeadRU',fontName='DJB',fontSize=12,leading=17,spaceBefore=10,spaceAfter=6));style.add(ParagraphStyle(name='CellRU',fontName='DJ',fontSize=8,leading=11));style.add(ParagraphStyle(name='SmallRU',fontName='DJ',fontSize=8,leading=11,spaceAfter=5))
p=lambda t:Paragraph(t,style['BodyRU'])
h=lambda t:Paragraph(t,style['HeadRU'])
cell=lambda t:Paragraph(escape(str(t)).replace('\n','<br/>'),style['CellRU'])
def table(rows,widths):
 t=Table([[cell(v) for v in row] for row in rows],colWidths=widths,repeatRows=1,hAlign='LEFT');t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e7eff6')),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#9eb5c9')),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]));return t
story=[Paragraph('БУСА — электрическая схема R1',style['TitleRU']),p('08.11.2026 • Модульная несущая плата • Совместимость с UI12'),h('Что проверено'),p('<b>KiCad ERC: 0 ошибок, 0 предупреждений.</b> По экспортированному списку цепей прошли 10 отдельных проверок. Намеренное соединение двух GPIO-выходов в тестовой копии обнаруживается ERC.'),p('Съёмные ESP32 и PCF8575, отдельный четырёхканальный звуковой релейный модуль. Датчики температуры напрямую к ESP32. Удалённые PCA9685 и DS2482 не включены.'),h('Основные сигналы — по текущей прошивке'),table([
['Назначение','GPIO','Вывод вашей ESP32 / разъём'],['Вода','16','R14 → J6.2'],['Куб','18','R12 → J7.2'],['Колонна','17','R13 → J8.2'],['Авария','21','L05 → J9.2'],['Клавиатура через PCF8575','SDA40 / SCL41','L15 / L16 → U3.SDA / SCL'],['TFT: CS / DC / RST / MOSI / SCK','10 / 11 / 12 / 13 / 14','R07 / R06 / R05 / R04 / R03'],['Реле IN1 / IN2 / IN3 / IN4','7 / 6 / 5 / 4','R16 / R17 / R18 / R19 → буфер U4 → J4'],['Охлаждение YF-S401','15','J10 → Q1 → U5 → R15'],['Отбор: аппаратный резерв','1','J11 → Q2 → U5 → L19; UI12 не опрашивает'],['Зуммер: логический резерв','2','L18 → J12.1, не питание излучателя']],[151,110,264]),h('Нумерация ESP32'),p('Вид на компоненты, оба USB расположены сверху. L01…L22 — левый ряд сверху вниз, R01…R22 — правый. Это ваша 44-контактная плата, не универсальная распиновка всех ESP32-S3.'),h('Статус проверки'),p('<b>Проверено программно на уровне схемы и связей.</b> Физическая прозвонка, токи и нагрев на устройстве не проверены. До изготовления PCB остаются геометрия колодок, ориентация разъёмов и проверка питания конкретного релейного модуля.'),PageBreak(),Paragraph('Разъёмы и питание',style['TitleRU']),table([
['Разъём','Контакты несущей платы','Назначение / условия'],['J1 — вход DC','1: +12 В; 2: GND','Только два провода. Регулируемый источник 12 В.'],['J2 — экран','1:GND, 2:3V3, 3:SCK, 4:MOSI, 5:RST, 6:DC, 7:CS','Питание экрана 3,3 В подтверждено владельцем.'],['J3 — клавиатура','1…8: P00…P07; 9: P10','9 сигнальных проводов; сохранить текущий порядок шлейфа или обучить заново.'],['J4 — реле, логика','1:GND; 2:IN1; 3:IN2; 4:IN3; 5:IN4; 6:VCC5','Соответствие шести подписанным выводам платы реле.'],['J5 — катушки реле','1:JD12; 2:GND','Отдельно от VCC5. Реальное расположение JD-VCC и возврата проверить.'],['J6…J9 — температуры','1:3V3; 2:DQ; 3:GND','По каналу: Вода, Куб, Колонна, Авария. На каждом DQ 4,7 кОм к 3,3 В.'],['J10 — охлаждение','1:питание; 2:GND; 3:импульс','Вход через преобразователь уровня. Питание через JP3 после сверки датчика.'],['J11 — отбор','1:питание; 2:GND; 3:импульс','Модель датчика ещё не выбрана; JP4 разомкнут.'],['J12 — зуммер','1:GPIO2; 2:GND','Логический сигнал для совместимого внешнего драйвера.']],[93,225,207]),h('Перемычки'),p('<b>JP1:</b> внешние 5 В на ESP32. Разомкнуть перед питанием от любого USB.<br/><b>JP2:</b> питание JD-VCC. Разомкнута до проверки модуля; перемычка VCC/JD-VCC на самом реле должна быть снята.<br/><b>JP3/JP4:</b> питание датчиков импульсов. Устанавливать по допустимому питанию конкретного датчика.'),h('Предлагаемые новые цепи несущей платы'),p('Защита полярности и предохранитель; стабилизатор R-78E5.0-1.0 (5 В / 1 А); неинвертирующий буфер реле SN74LVC07A; два преобразователя BC847B + SN74LVC2G14 для импульсных входов. Это новые проектные компоненты, их наличие у владельца не предполагается.'),h('Что ещё требуется подтвердить измерением'),p('1. Связи JD-VCC/VCC/GND и фактические уровни входов именно вашей релейной платы.<br/>2. Ток шины 3,3 В с включённой подсветкой и нагрев стабилизатора ESP32.<br/>3. Размеры и ориентация всех съёмных колодок.<br/>4. Модель, диапазон и коэффициент импульсов датчика отбора.'),p('ERC не проверяет неизвестные соединения внутри готового модуля. Монтажные размеры не берутся с иллюстрации. Полная таблица цепей, BOM и ссылки на документацию сохранены в проекте.')]
summary=out/'SUMMARY.pdf';SimpleDocTemplate(str(summary),pagesize=A4,leftMargin=35,rightMargin=35,topMargin=30,bottomMargin=30).build(story)
# Detailed schematic panels for readable phone viewing. Same vector source, no redraw.
d=fitz.open(out/'BUSA_R1.pdf');panels=fitz.open();regions=[('01_power_esp',0,0,430,270),('02_display_relays',430,0,841,370),('03_temp_keypad',0,270,430,480),('04_pulse_inputs',0,370,841,594)]
for name,x0,y0,x1,y1 in regions:
 clip=fitz.Rect(x0*72/25.4,y0*72/25.4,x1*72/25.4,y1*72/25.4);pg=panels.new_page(width=clip.width,height=clip.height);pg.show_pdf_page(pg.rect,d,0,clip=clip);pg.get_pixmap(matrix=fitz.Matrix(1.3,1.3)).save(out/(name+'.png'))
panels.save(out/'BUSA_R1_readable_sections.pdf');allpages=fitz.open(summary);allpages.insert_pdf(panels);allpages.save(out/'BUSA_R1_review.pdf');allpages[0].get_pixmap(matrix=fitz.Matrix(2,2)).save(out/'PINMAP_PREVIEW.png')
d[0].get_pixmap(matrix=fitz.Matrix(1.5,1.5)).save(out/'SCHEMATIC_OVERVIEW.png')
with (out/'BOM.csv').open('w') as f:
 w=csv.writer(f,lineterminator="\n");w.writerow(['Ref','Value','Kind','Datasheet','Notes'])
 for r in parts:
  if r['ref'].startswith('PF'):continue
  kind='Existing user module' if r['ref'] in ['U1','U3'] else 'Proposed carrier part/connector'
  if 'OPTIONAL' in r['value']:kind='Optional - check module pullups before fitting'
  if r['ref'].startswith('JP'):kind='Open by default; see commissioning conditions'
  w.writerow([r['ref'],r['value'],kind,r['datasheet'],r['description']])
u1=next(r for r in parts if r['ref']=='U1')
with (out/'HEADER_44.csv').open('w') as f:
 w=csv.writer(f,lineterminator="\n");w.writerow(['Position USB-at-top','Silkscreen function','Net','Direction'])
 for r in u1['pins']:w.writerow([r['number'],r['name'],r['net'] or 'NC',r['type']])
print('Built review PDF:',len(allpages),'pages; BOM, 44-pin header table and inline preview')
