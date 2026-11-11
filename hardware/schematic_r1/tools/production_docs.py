"""Create honest review BOM, harness table, 1:1 template and readable PDF.

Stock, substitutions, package-specific assembly rotations and external-module
mechanics remain explicit open items; none are inferred from a clean ERC.
"""
from pathlib import Path
import json,csv,collections,hashlib
from xml.sax.saxutils import escape
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import SimpleDocTemplate,Paragraph,Table,TableStyle,PageBreak,Image,Spacer
from reportlab.pdfgen import canvas
import fitz
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output'
parts=json.loads((ROOT/'parts.json').read_text());fps=json.loads((ROOT/'footprints.json').read_text())
for n,f in [('DJ','DejaVuSans.ttf'),('DJB','DejaVuSans-Bold.ttf')]:pdfmetrics.registerFont(TTFont(n,'/usr/share/fonts/truetype/dejavu/'+f))

bom=[];harness=[]
for part in parts:
    ref=part['ref'];value=part['value']
    if ref.startswith('PF'):continue
    mpn='';maker='';fit='FIT';note='Выбор для несущей платы; наличие и замены согласовать'
    if ref=='U1':mpn='Имеющийся ESP32-S3 N16R8 44PIN / 2 USB';maker='Модуль пользователя';note='Гнёзда 2×22; междурядье 25,50 мм по фото; корпус и USB проверить'
    elif ref=='U3':mpn='Имеющийся модуль PCF8575 8+8+5';maker='Модуль пользователя';note='Гнёзда 2×8 + 1×5; положение колодок НЕ подтверждено'
    elif ref=='U2':mpn='R-78E5.0-1.0';maker='RECOM'
    elif ref=='U4':mpn='SN74LVC07AD';maker='Texas Instruments'
    elif ref=='U5':mpn='SN74LVC2G14DBVR';maker='Texas Instruments'
    elif ref.startswith('Q'):mpn='BC847B,215';maker='Nexperia'
    elif ref.startswith('R'):
        base=value.split()[0];mpn='RC0805FR-07'+{'330':'330RL','4.7k':'4K7L','10k':'10KL','22k':'22KL','100k':'100KL'}[base];maker='Yageo';note='0805, 1%, ≥0,125 Вт; MPN-кандидат, сверить закупочный лист'
        if ref in ['R1','R2']:fit='DNP';note='Не ставить до измерения подтяжек на PCF8575; подтяжка только к 3,3 В'
    elif ref in ['C1','C3']:mpn='GRM32ER7YA106KA12L';maker='Murata';note='1210, 10 мкФ, 35 В, X7R; проверить ёмкость под DC-смещением'
    elif ref=='C6':mpn='GRM21BR60J106ME19L';maker='Murata';note='0805, 10 мкФ, 6,3 В, X5R; шина 3,3 В'
    elif ref=='C8':mpn='GRM21BR71C475KA73L';maker='Murata';note='0805, 4,7 мкФ, 16 В, X7R'
    elif ref in ['C14','C15']:mpn='TBD: 0805 / 1 нФ / 50 В / X7R или C0G';maker='TBD';note='Точный MPN ещё выбрать; не закупочная позиция'
    elif ref.startswith('C'):mpn='GRM21BR71H104KA01L';maker='Murata';note='0805, 100 нФ, 50 В, X7R'
    elif ref=='D1':mpn='B340A-13-F';maker='Diodes Incorporated';note='3 А / 40 В, SMA, катод с полосой; MPN и корпус проверены по документации производителя'
    elif ref.startswith('D'):mpn='1N4148W-E3-18';maker='Vishay';note='SOD-123; MPN и корпус проверены по документации производителя, катод к BASE'
    elif ref=='F1':mpn='0454001.MR (кандидат)';maker='Littelfuse';note='1 А, time-lag; окончательно сверить чертёж корпуса, DC-рейтинг, I²t и ток платы'
    elif ref.startswith('JP'):mpn='TBD: 1×2, 2,54 мм, вертикальный';maker='TBD';fit='HEADER / NO SHUNT';note='Штыри установить, перемычку НЕ устанавливать до соответствующей проверки'
    elif ref.startswith('J'):
        count=len(part['pins']);mpn=f'B{count}B-XH-A(LF)(SN)';maker='JST';note='Новая вертикальная колодка XH, шаг 2,50 мм; не прямое совпадение со старым жгутом'
        for pin in part['pins']:harness.append([ref,pin['number'],pin['name'],pin['net'],f'XHP-{count}',f'Новая проводка; {part["description"]}'.rstrip()])
    bom.append([ref,value,fps[ref],1,maker,mpn,fit,note])
with (OUT/'BOM_REVIEW.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(['Reference','Value','Footprint','Qty_per_board','Manufacturer','Candidate_MPN_NOT_PURCHASE_RELEASE','Fit','Notes']);w.writerows(bom)
with (OUT/'HARNESS.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(['Carrier_connector','Pin','Signal','Net','Candidate_mating_housing','Notes']);w.writerows(harness)
accessories=[['U1 sockets','2','1×22 female, 2.54mm','MPN/height TBD; user ESP module fits on top'],['U3 sockets','2 + 1','1×8 female ×2; 1×5 female ×1; 2.54mm','MPN/height and relative geometry TBD'],['Cable contacts','48','JST SXH-001T-P0.6 candidate','Wire gauge and contact/tool applicability to verify'],['Mounting','4','M3 screws/standoffs','Length according to enclosure, metal must clear copper'],['Module relay','1','Existing 4-channel 12V sound relay','External assembly; not soldered to carrier'],['Display','1','Existing GMT028-05 V1.1','External; 3.3V confirmed'],['Keypad','1','Existing light membrane 5×4','9 signal wires; current key map'],['Temperature','4','Existing three-wire DS18B20 probes','New harness order VDD / DQ / GND'],['Cooling flowmeter','1','Existing white YF-S401','Supply/output and calibration to verify'],['Collection flowmeter','0','Future model not selected','Reserved connector only, not a purchase item']]
with (OUT/'ACCESSORIES.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(['Item','Qty','Description','Notes']);w.writerows(accessories)

# A true physical-size print for checking sockets; no raster scaling involved.
c=canvas.Canvas(str(OUT/'CONTROL_TEMPLATE_1TO1.pdf'),pagesize=A4)
c.setFont('DJB',16);c.drawString(20*mm,280*mm,'БУСА — проверочный отпечаток 1:1')
c.setFont('DJ',10)
for i,t in enumerate(['Печатать 100%, без «вписать в страницу».','Сначала проверить линейкой отрезок 50 мм внизу.','ESP: шаг 2,54 мм; междурядье 25,50 мм по фото.','PCF: геометрия предварительная. Это НЕ монтажный шаблон.']):c.drawString(20*mm,(270-6*i)*mm,t)
c.saveState();c.translate(45*mm,75*mm);c.scale(mm,mm)
c.setLineWidth(.2);c.rect(0,0,120,100)
for x,y in [(4,4),(116,4),(4,96),(116,96)]:c.circle(x,y,1.6)
placements=list(csv.DictReader((OUT/'PLACEMENT_REVIEW.csv').open()))
# Header coordinates from parameters, which match the board audit.
for x in [50,50+25.5]:
    for i in range(22):c.circle(x,100-(30+i*2.54),.5)
pcf=par=json.loads((ROOT/'pcb_parameters.json').read_text())['pcf8575']
c.setStrokeColor(colors.HexColor('#b32828'))
for i in range(8):
    for y in [16,16+pcf['io_row_spacing_mm']]:c.circle(11+i*2.54,100-y,.5)
for i in range(5):c.circle(11+pcf['i2c_x_mm'],100-(16+pcf['i2c_y_mm']+i*2.54),.5)
c.restoreState();c.setStrokeColor(colors.black);c.line(30*mm,40*mm,80*mm,40*mm);c.line(30*mm,38*mm,30*mm,42*mm);c.line(80*mm,38*mm,80*mm,42*mm);c.setFont('DJ',10);c.drawString(45*mm,34*mm,'50,00 мм')
c.setFont('DJ',9);c.drawString(20*mm,20*mm,'Плата 120 × 100 мм. Контроль посадки до выпуска Gerber.');c.save()

style=getSampleStyleSheet()
for name,size,leading,bold in [('BodyRU',9,13,False),('TitleRU',19,24,True),('HeadRU',12,17,True),('CellRU',7.5,10,False)]:style.add(ParagraphStyle(name=name,fontName='DJB' if bold else 'DJ',fontSize=size,leading=leading,spaceAfter=7))
P=lambda t:Paragraph(t,style['BodyRU'])
H=lambda t:Paragraph(t,style['HeadRU'])
T=lambda t:Paragraph(t,style['TitleRU'])
def table(rows,widths):
    t=Table([[Paragraph(escape(str(v)).replace('\n','<br/>'),style['CellRU']) for v in r] for r in rows],colWidths=widths,repeatRows=1)
    t.setStyle(TableStyle([('GRID',(0,0),(-1,-1),.4,colors.HexColor('#a4b7c9')),('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e5edf5')),('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]));return t
checks=json.loads((OUT/'PCB_CHECKS.json').read_text());drc=json.loads((OUT/'PCB_DRC.json').read_text())
violations=len(drc['violations']);missing=len(drc['unconnected_items'])
story=[T('БУСА R1 — проект несущей платы'),P('08.11.2026 • Для подготовки к изготовлению в НПП «Принт-Д», Челябинск'),H('Статус: инженерная проверка, выпуск ещё не выполнен'),P('Схема и физический проект KiCad приложены. Размеры колодок PCF8575, габариты ESP/USB/антенны и питание внешних модулей остаются открытыми. Эти сведения нельзя получить из ERC/DRC.'),table([['Параметр','Результат'],['Габарит / толщина / слои','120 × 100 мм / 1,6 мм / 2'],['ESP32','44 PIN, 2 USB; шаг 2,54 мм, междурядье 25,50 мм по фото'],['Конструктив','Съёмные ESP32 и PCF8575; внешний блок 4 звуковых реле'],['Кабельные колодки','12 вертикальных разъёмов сверху; выбран JST XH 2,50 мм'],['Схема','KiCad ERC: 0; 10 проверок соединений с UI12'],['Плата: зазоры / незавершённые связи',f'{violations} нарушений / {missing} незавершённых соединений'],['Независимая сверка',f'{checks["expected_connected_pads"]} подключённых площадок сверены с XML-схемой'],['Разводка',f'{checks["tracks"]} сегментов, {checks["vias"]} переходов'],['Физические испытания','НЕ выполнены. Измерения не подменены расчётами.']],[176,349]),H('Изготовитель'),P('ООО НПП «Принт-Д», Челябинск, Новороссийская, 30. Контакты из справочников: +7 (351) 231-45-60, print_d@mail.ru. Сайт printd74.ru прочитать не удалось. Принимаемые форматы, монтаж, цена, срок и минимальная партия не подтверждены. Заказ и письма не отправлялись.'),H('Что требуется до окончательного выпуска'),P('PCF8575: A/B/C и ориентация колодок; ESP: корпус/USB/антенна и отпечаток 1:1. Прозвонить JD-VCC релейной платы, проверить ток и нагрев 3,3 В. Закрыть TBD в BOM и согласовать технологию/покрытие/тираж с изготовителем.'),PageBreak(),T('Размещение и дорожки'),P('Изображение построено из реального .kicad_pcb. Прозрачные очертания модулей — предварительный габарит, а не измеренная 3D-модель.'),Image(str(OUT/'PCB_PREVIEW.png'),width=525,height=437.5),P('Все кабельные разъёмы вставляются сверху перпендикулярно плате. Релейный модуль не расположен на несущей плате; J4/J5 соединяют его отдельными жгутами. Запас под антенну не заменяет проверку чертежа вашей ESP32.'),PageBreak(),T('Размеры и контроль посадки'),Image(str(OUT/'MEASUREMENT_GUIDE.png'),width=525,height=262.5),P('Для PCF8575 нужны A/B/C между центрами отверстий и читаемый порядок подписей. Числа в параметрах модели PCF являются предварительными. Использовать CONTROL_TEMPLATE_1TO1.pdf при печати без масштабирования.'),P('25,50 мм ESP взято с маркировки предоставленной переходной платы; это не результат измерения штангенциркулем. Шаг 2,54 мм указан пользователем. Положение корпусов и разъёмов ещё сверяется.'),H('Механические параметры'),P('Предварительно: 120 × 100 мм, четыре отверстия Ø3,2 мм на расстоянии 4 мм от краёв. Гнёзда съёмных модулей — 2,54 мм, новые кабельные колодки XH — 2,50 мм. Эти два шага не взаимозаменяемы.'),PageBreak(),T('Разъёмы и жгуты'),table([['Разъём','Контакты','Примечание']]+[[ref,'; '.join(f'{p[1]}={p[2]}' for p in harness if p[0]==ref),next(x['value'] for x in parts if x['ref']==ref)] for ref in [f'J{i}' for i in range(1,13)]],[51,306,168]),H('Перемычки — поставляются открытыми'),P('JP1: внешние 5 В на ESP, снять до любого USB. JP2: питание катушек, до прозвонки внешнего реле не замыкать. JP3/JP4: питание импульсных датчиков, до проверки допустимого напряжения не замыкать. R1/R2 I²C по умолчанию DNP до измерения подтяжек.'),P('Новые жгуты должны соответствовать таблице. Номер разъёма или цвет провода сам по себе не доказывает совместимость со старой проводкой.'),PageBreak(),T('Перечень деталей для согласования'),P('BOM_REVIEW.csv содержит все 67 электрических позиций. Точные MPN и кандидаты различены. DNP и позиции TBD не являются разрешением на автоматическую закупку или монтаж. Съёмные гнёзда, кабели и внешние модули перечислены отдельно в ACCESSORIES.csv.'),table([['Позиции','Номинал / MPN-кандидат','Монтаж / условие']]+[[r[0],r[1]+'\n'+r[5],r[6]+'\n'+r[7]] for r in bom],[43,226,256]),PageBreak(),T('Комплект для изготовителя'),P('После закрытия замечаний FAB включает Gerber меди/маски/шелкографии/контура, Excellon PTH/NPTH, чертёж платы, параметры слоёв, IPC-D-356 и SHA-256. Для сборки дополнительно: BOM, согласованные XY/углы/сторона, сборочные виды, полярность, DNP, трафаретная паста и жгуты.'),P('Пока не сформирован разрешённый FAB/ASSEMBLY. Файлы в архиве ENGINEERING_REVIEW предназначены для проверки проекта. Координатная ведомость не является проверенным машинным файлом конкретного монтажника.'),H('Проверка первого образца'),P('1. Без модулей проверить полярность и сопротивления; все перемычки открыты. Ограничить ток источника и проверить 12/5 В.<br/>2. Поэтапно подключить ESP, PCF, экран; измерить токи 3,3 В и нагрев. Перед USB отключить JP1.<br/>3. Проверить четыре температурных канала и обрыв каждого датчика.<br/>4. Проверить I²C и текущую раскладку клавиатуры.<br/>5. Прозвонить реле отдельно, исключить соединение VCC5/JD12, проверить активный LOW и состояния на запуске/сбросе.<br/>6. Проверить импульсные уровни/счётчик и калибровать YF-S401 проливом. Отбор оставить резервом до отдельной реализации.<br/>7. Проверить повторные включения, обрывы и помехи длинных кабелей; записать результаты.'),H('Границы проекта'),P('Общая низковольтная земля, гальваническая развязка не заявлена. Выходы реле — звуковые сигналы. Цепи сетевого питания не входят в плату. Калькулятор, Wi-Fi и остальная прошивка этой работой не изменены. Полные ссылки и история решений — в SOURCES.md и PRODUCTION.md.')]
def footer(c,doc):
    c.setFont('DJ',7);c.setFillColor(colors.HexColor('#8b3232'));c.drawString(35,17,'ИНЖЕНЕРНЫЙ ПРОЕКТ • НЕ ВЫПУЩЕН В ПРОИЗВОДСТВО');c.drawRightString(A4[0]-35,17,str(doc.page))
SimpleDocTemplate(str(OUT/'BUSA_R1_PCB_REVIEW.pdf'),pagesize=A4,leftMargin=35,rightMargin=35,topMargin=30,bottomMargin=30).build(story,onFirstPage=footer,onLaterPages=footer)
doc=fitz.open(OUT/'BUSA_R1_PCB_REVIEW.pdf');assert len(doc)>=6
for page in doc:assert 'ИНЖЕНЕРНЫЙ ПРОЕКТ' in page.get_text()
doc[0].get_pixmap(matrix=fitz.Matrix(1.5,1.5)).save(OUT/'PCB_REVIEW_COVER.png')
print('Saved review PDF:',len(doc),'pages; BOM 67 rows; harness',len(harness),'pins; true-size socket template')
