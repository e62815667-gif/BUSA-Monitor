"""Generate a review-only physical KiCad board from the checked R1 netlist.

Module geometry is parametric and explicitly unverified. Do not infer measured
hardware or production readiness from successful DRC or generated artwork.
Run with the system /usr/bin/python3 (KiCad's pcbnew binding).
"""
import os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for k, d in [('XDG_CONFIG_HOME','config'), ('XDG_CACHE_HOME','cache'), ('XDG_DATA_HOME','data')]:
    os.environ.setdefault(k, str(ROOT/'.runtime'/d))
import pcbnew as p
import json, uuid, re, csv
import xml.etree.ElementTree as ET

PAR = json.loads((ROOT/'pcb_parameters.json').read_text())
PARTS = json.loads((ROOT/'parts.json').read_text())
LIB = Path('/usr/share/kicad/footprints')
LOCAL = ROOT/'BUSA.pretty'
LOCAL.mkdir(exist_ok=True)
OUT = ROOT/'output'
OUT.mkdir(exist_ok=True)
U = lambda s: str(uuid.uuid5(uuid.NAMESPACE_URL, 'busa-r1:'+s))
V = lambda x,y: p.VECTOR2I(p.FromMM(x),p.FromMM(y))
def layers(*items):
    result=p.LSET()
    for item in items:result.AddLayer(item)
    return result
board = p.BOARD()
board.SetFileName(str(ROOT/'BUSA_R1.kicad_pcb'))
settings = board.GetDesignSettings()
settings.SetCopperLayerCount(2)
settings.SetBoardThickness(p.FromMM(1.6))
settings.m_MinClearance = p.FromMM(.25)
settings.m_TrackMinWidth = p.FromMM(.25)
settings.m_ViasMinSize = p.FromMM(.8)
settings.m_ViasMinAnnularWidth = p.FromMM(.15)
settings.m_CopperEdgeClearance = p.FromMM(.5)
settings.m_HoleClearance = p.FromMM(.25)
settings.m_SolderMaskMinWidth = p.FromMM(.1)
settings.m_SilkClearance = p.FromMM(.15)
settings.m_SolderMaskExpansion = p.FromMM(.05)
settings.SetAuxOrigin(V(50,150))
default_class=settings.m_NetSettings.GetDefaultNetclass()
default_class.SetClearance(p.FromMM(.25));default_class.SetTrackWidth(p.FromMM(.25))
default_class.SetViaDiameter(p.FromMM(.8));default_class.SetViaDrill(p.FromMM(.4))
board.GetTitleBlock().SetTitle('BUSA modular carrier — ENGINEERING DRAFT')
board.GetTitleBlock().SetRevision('R1 PCB REVIEW / NOT RELEASED')
board.GetTitleBlock().SetDate('2026-11-08')
board.GetTitleBlock().SetComment(0, 'Module dimensions unverified; do not manufacture')

# Use the KiCad-exported XML, not just generator parts, as electrical authority.
xml = ET.parse(OUT/'NETLIST.xml').getroot()
nets = {}; pin_net = {}
for n in xml.find('nets'):
    name = n.attrib['name']
    item = p.NETINFO_ITEM(board,name); board.Add(item); nets[name]=item
    for node in n.findall('node'): pin_net[node.attrib['ref'],node.attrib['pin']]=name

power_class=p.NETCLASS('Power');power_class.SetClearance(p.FromMM(.25));power_class.SetTrackWidth(p.FromMM(.8))
power_class.SetViaDiameter(p.FromMM(1.1));power_class.SetViaDrill(p.FromMM(.5))
settings.m_NetSettings.SetNetclass('Power',power_class)
power_nets=['/'+n for n in ['VIN12','VIN12_FUSED','V12_PROT','V5','ESP_5V','V3V3','GND','RELAY_JD12']]
for name in power_nets:settings.m_NetSettings.SetNetclassPatternAssignment(name,'Power')
board.SynchronizeNetsAndNetClasses(False)

def fp_line(fp, x1,y1,x2,y2,layer=p.F_Fab,width=.15):
    line = p.PCB_SHAPE(fp); line.SetShape(p.SHAPE_T_SEGMENT)
    line.SetStart(V(x1,y1));line.SetEnd(V(x2,y2));line.SetLayer(layer);line.SetWidth(p.FromMM(width));fp.Add(line)

def fp_rect(fp,x1,y1,x2,y2,layer=p.F_Fab):
    for a,b,c,d in [(x1,y1,x2,y1),(x2,y1,x2,y2),(x2,y2,x1,y2),(x1,y2,x1,y1)]: fp_line(fp,a,b,c,d,layer,.05 if layer==p.F_CrtYd else .15)

def add_pad(fp,num,x,y):
    pad=p.PAD(fp);pad.SetNumber(num);pad.SetAttribute(p.PAD_ATTRIB_PTH)
    pad.SetShape(p.PAD_SHAPE_RECT if num in ['L01','P07'] else p.PAD_SHAPE_OVAL)
    pad.SetSize(V(1.7,1.7));pad.SetDrillSize(V(1,1));pad.SetLayerSet(layers(p.F_Cu,p.B_Cu,p.F_Mask,p.B_Mask));pad.SetPosition(V(x,y));fp.Add(pad)

def module_fp(kind):
    fp=p.FOOTPRINT(board)
    if kind=='esp32':
        geo=PAR[kind];w=geo['row_spacing_mm'];pitch=geo['pitch_mm']
        for side,x in [('L',0),('R',w)]:
            for i in range(22):add_pad(fp,side+f'{i+1:02}',x,i*pitch)
        fp_rect(fp,-1.5,-6,w+1.5,66)
        # Two removable 1x22 sockets; module outline is fabrication annotation.
        for x in [0,w]:fp_rect(fp,x-1.5,-1.5,x+1.5,21*pitch+1.5,p.F_CrtYd)
        name='ESP32S3_44PIN_SOCKET_DRAFT'
    else:
        geo=PAR['pcf8575'];pitch=geo['pitch_mm'];dy=geo['io_row_spacing_mm']
        for i in range(8):
            add_pad(fp,'P0'+str(7-i),i*pitch,0)
            add_pad(fp,'P1'+str(i),i*pitch,dy)
        for i,num in enumerate(['INT','SCL','SDA','VCC','GND']):add_pad(fp,num,geo['i2c_x_mm'],geo['i2c_y_mm']+i*pitch)
        fp_rect(fp,-1.5,-1.5,geo['i2c_x_mm']+1.5,dy+1.5)
        for y in [0,dy]:fp_rect(fp,-1.5,y-1.5,7*pitch+1.5,y+1.5,p.F_CrtYd)
        fp_rect(fp,geo['i2c_x_mm']-1.5,geo['i2c_y_mm']-1.5,geo['i2c_x_mm']+1.5,geo['i2c_y_mm']+4*pitch+1.5,p.F_CrtYd)
        name='PCF8575_8_8_5_SOCKET_DRAFT'
    fp.SetFPID(p.LIB_ID('BUSA',name));fp.SetAttributes(p.FP_THROUGH_HOLE)
    p.PCB_IO_MGR.PluginFind(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(LOCAL),fp)
    return fp

def load_lib(lib,name,local_name=None,renumber=None):
    fp=p.FootprintLoad(str(LIB/(lib+'.pretty')),name)
    if not fp:raise ValueError((lib,name))
    # Vendor-independent library geometry is copied into project-local library.
    if renumber:
        for pad in fp.Pads():pad.SetNumber(renumber.get(pad.GetNumber(),pad.GetNumber()))
    name=local_name or name
    fp.SetFPID(p.LIB_ID('BUSA',name));p.PCB_IO_MGR.PluginFind(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(LOCAL),fp)
    return fp

positions={
 'U1':(100,80,0),'U3':(61,66,0),
 'J1':(61,140,0),'F1':(72,142,0),'D1':(81,142,0),'U2':(89,141,0),
 'C1':(78,132,0),'C2':(84,130,90),'C3':(88,122,0),'C4':(88,127,90),'JP1':(91,113,0),
 'J2':(101,56,0),'C5':(121,62,0),'C6':(121,66,0),
 'J3':(62,56,0),'R1':(87,87,0),'R2':(87,91,0),'C7':(89,80,90),'C8':(89,84,90),
 'J4':(139,57,0),'J5':(160,65,90),'JP2':(159,76,0),
 'U4':(143,80,0),'C9':(149,75,90),
 'R11':(135,77,90),'R12':(135,81,90),'R13':(135,85,90),'R14':(135,89,90),
 'R21':(139,67,90),'R22':(143,67,90),'R23':(147,67,90),'R24':(151,67,90),
 'J6':(57,94,90),'R30':(67,90,90),'C10':(71,90,90),
 'J7':(57,107,90),'R31':(67,103,90),'C11':(71,103,90),
 'J8':(57,120,90),'R32':(67,116,90),'C12':(71,116,90),
 'J9':(80,106,90),'R33':(89,101,90),'C13':(89,105,90),
 'J10':(163,100,90),'JP3':(156,97,0),'R41':(155,103,90),'R42':(155,107,90),
 'Q1':(148,103,0),'R43':(142,103,90),'R44':(142,97,90),'C14':(147,97,90),'D2':(152,112,0),
 'J11':(163,125,90),'JP4':(156,122,0),'R51':(155,128,90),'R52':(155,132,90),
 'Q2':(148,128,0),'R53':(142,128,90),'R54':(142,122,90),'C15':(147,122,90),'D3':(152,137,0),
 'U5':(142,113,0),'R45':(135,108,90),'R55':(135,116,90),'C16':(147,115,90),
 'J12':(83,115,0)
}
footprints={};mapping={}
for part in PARTS:
    ref=part['ref']
    if ref.startswith('PF'):continue
    if ref=='U1':fp=module_fp('esp32')
    elif ref=='U3':fp=module_fp('pcf8575')
    elif ref.startswith('JP'):fp=load_lib('Connector_PinHeader_2.54mm','PinHeader_1x02_P2.54mm_Vertical')
    elif ref.startswith('J'):
        count=len(part['pins']);fp=load_lib('Connector_JST',f'JST_XH_B{count}B-XH-A_1x{count:02}_P2.50mm_Vertical')
    elif ref.startswith('R'):fp=load_lib('Resistor_SMD','R_0805_2012Metric')
    elif ref in ['C1','C3']:fp=load_lib('Capacitor_SMD','C_1210_3225Metric')
    elif ref.startswith('C'):fp=load_lib('Capacitor_SMD','C_0805_2012Metric')
    elif ref=='D1':fp=load_lib('Diode_SMD','D_SMA','D_SMA_LOGICAL_A_K',{'1':'K','2':'A'})
    elif ref.startswith('D'):fp=load_lib('Diode_SMD','D_SOD-123','1N4148W_SOD123_LOGICAL_A_K',{'1':'K','2':'A'})
    elif ref.startswith('Q'):fp=load_lib('Package_TO_SOT_SMD','SOT-23')
    elif ref=='U2':fp=load_lib('Converter_DCDC','Converter_DCDC_RECOM_R-78E-0.5_THT','RECOM_R78E10_11.6x8.5_P2.54')
    elif ref=='U4':fp=load_lib('Package_SO','SOIC-14_3.9x8.7mm_P1.27mm')
    elif ref=='U5':fp=load_lib('Package_TO_SOT_SMD','SOT-23-6')
    elif ref=='F1':fp=load_lib('Fuse','Fuse_Littelfuse-NANO2-451_453','Fuse_Littelfuse_NANO2_454_DRAFT')
    else:raise ValueError(ref)
    fp.SetReference(ref);fp.SetValue(part['value']);fp.SetPath(p.KIID_PATH('/'+U('root')+'/'+U(ref)))
    if ref in ['R1','R2']:fp.SetDNP(True)
    board.Add(fp)
    x,y,a=positions[ref];fp.SetPosition(V(x,y));fp.SetOrientationDegrees(a)
    fp.Reference().SetTextSize(V(.8,.8));fp.Reference().SetTextThickness(p.FromMM(.12))
    fp.Reference().SetLayer(p.F_Fab)
    fp.Value().SetVisible(False)
    for pad in fp.Pads():
        key=ref,pad.GetNumber()
        if key in pin_net:pad.SetNet(nets[pin_net[key]])
    assert {pad.GetNumber() for pad in fp.Pads()}=={v['number'] for v in part['pins']},ref
    footprints[ref]=fp;mapping[ref]=str(fp.GetFPID().GetLibNickname())+':'+str(fp.GetFPID().GetLibItemName())

def line(x1,y1,x2,y2,layer=p.Edge_Cuts,width=.05):
    item=p.PCB_SHAPE(board);item.SetShape(p.SHAPE_T_SEGMENT);item.SetStart(V(x1,y1));item.SetEnd(V(x2,y2));item.SetLayer(layer);item.SetWidth(p.FromMM(width));board.Add(item)

for x1,y1,x2,y2 in [(50,50,170,50),(170,50,170,150),(170,150,50,150),(50,150,50,50)]:line(x1,y1,x2,y2)
for i,(x,y) in enumerate([(54,54),(166,54),(54,146),(166,146)],1):
    fp=load_lib('MountingHole','MountingHole_3.2mm_M3');fp.SetReference('H'+str(i));fp.SetPosition(V(x,y));fp.SetAttributes(p.FP_BOARD_ONLY|p.FP_EXCLUDE_FROM_BOM|p.FP_EXCLUDE_FROM_POS_FILES);fp.Reference().SetVisible(False);fp.Value().SetVisible(False);board.Add(fp)

def text(t,x,y,size=1,layer=p.F_SilkS):
    it=p.PCB_TEXT(board);it.SetText(t);it.SetPosition(V(x,y));it.SetLayer(layer);it.SetTextSize(V(size,size));it.SetTextThickness(p.FromMM(.15));board.Add(it)

text('BUSA / R1 ENGINEERING DRAFT',110,146,1.25)
text('MODULE DIMENSIONS NOT VERIFIED',110,143.5,.8,p.Dwgs_User)
text('12V  GND',62,147,.85)
text('TFT / 3V3',108,63,.9)
text('KEYPAD / 9 SIGNALS',70,61,.85)
text('SOUND RELAY CTRL / 5V LOGIC',144,62,.8)
for name,x,y in [('WATER',59,98.5),('CUBE',59,111.5),('COLUMN',59,124.5),('ALARM',81,110.5),('COOLING',161,105.5),('COLLECTION',160,130.5),('JD12',160,71),('BUZZER SIG',81,121)]:text(name,x,y,.8)
text('USB x2',112.7,72,1,p.Dwgs_User)
text('NO COPPER / ESP ANTENNA',112.5,140,.75,p.Dwgs_User)
# Conservative reserved antenna zone, BOTH copper layers. Module drawing needed.
zone=p.ZONE(board);zone.SetIsRuleArea(True);zone.SetLayerSet(layers(p.F_Cu,p.B_Cu));zone.SetDoNotAllowCopperPour(True);zone.SetDoNotAllowTracks(True);zone.SetDoNotAllowVias(True)
zone.Outline().NewOutline()
for x,y in [(97,135),(129,135),(129,150),(97,150)]:zone.Outline().Append(int(p.FromMM(x)),int(p.FromMM(y)))
board.Add(zone)
# Narrow, short escapes are necessary at the SOT23-6 power pins; the main
# supply routing uses 0.8 mm. These are routing constraints, not ampacity proof.
for pin,x in [('2',139.1),('5',145.0)]:
    pad=next(pad for pad in footprints['U5'].Pads() if pad.GetNumber()==pin)
    tr=p.PCB_TRACK(board);tr.SetStart(pad.GetPosition());tr.SetEnd(V(x,113));tr.SetLayer(p.F_Cu);tr.SetWidth(p.FromMM(.25));tr.SetNet(pad.GetNet());tr.SetLocked(True);board.Add(tr)
    via=p.PCB_VIA(board);via.SetPosition(V(x,113));via.SetFrontWidth(p.FromMM(1.1));via.SetDrill(p.FromMM(.5));via.SetViaType(p.VIATYPE_THROUGH);via.SetLayerPair(p.F_Cu,p.B_Cu);via.SetNet(pad.GetNet());via.SetLocked(True);board.Add(via)
board.BuildConnectivity()
p.SaveBoard(str(ROOT/'BUSA_R1.kicad_pcb'),board)
(ROOT/'footprints.json').write_text(json.dumps(mapping,indent=2)+'\n')
(ROOT/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "BUSA") (type "KiCad") (uri "${KIPRJMOD}/BUSA.pretty") (options "") (descr "BUSA project-local review footprints")))\n')
with (OUT/'PLACEMENT_REVIEW.csv').open('w') as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(['Ref','Footprint','X_mm_absolute','Y_mm_absolute','Rotation_deg','Side','Status'])
    for ref,fp in sorted(footprints.items()):w.writerow([ref,mapping[ref],*positions[ref],'Top','DRAFT; NOT MACHINE-APPROVED'])
print('Saved engineering draft:',len(footprints),'electrical footprints + 4 mounting holes')
assert p.ExportSpecctraDSN(board,str(OUT/'BUSA_R1.dsn'))
print('Exported DSN for local, offline routing')
