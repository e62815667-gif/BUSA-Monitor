from pathlib import Path
import json,uuid
root=Path(__file__).resolve().parents[1]
U=lambda s:str(uuid.uuid5(uuid.NAMESPACE_URL,'busa-r1:'+s))
Q=lambda s:json.dumps(str(s),ensure_ascii=False)
G=lambda v:round(v/1.27)*1.27
parts=[];items=[];libs=[];rid=U('root')
def note(t,x,y,size=1.6):items.append(f'(text {Q(t)} (at {x} {y} 0) (effects (font (size {size} {size})) (justify left)) (uuid {Q(U(t+str(x)+str(y)))}))')
def part(ref,value,x,y,pins,desc='',width=25,spacing=3.81,footprint='',url=''):
 x,y=G(x),G(y);width=G(width);half=G((max(sum(p[3]==z for p in pins) for z in 'LR')+1)*spacing/2)
 positions={}
 for side in 'LR':
  for i,p in enumerate([p for p in pins if p[3]==side]):positions[p[0]]=(-width/2-5.08 if side=='L' else width/2+5.08,half-3.81-i*spacing,0 if side=='L' else 180)
 # Snap connection points relative to centre to 1.27 mm grid.
 positions={n:(G(a),G(b),c) for n,(a,b,c) in positions.items()}
 name='BUSA:'+ref
 prop=lambda key,val,a,b,hide=False:f'(property {Q(key)} {Q(val)} (at {a} {b} 0) (effects (font (size 1.2 1.2))'+(' (hide yes)' if hide else '')+'))'
 # Conventional IEC two-terminal symbols; module/IC blocks retain named pins.
 graphics=f'(rectangle (start {-width/2} {half}) (end {width/2} {-half}) (stroke (width .254) (type default)) (fill (type background)))'
 def ln(x1,y1,x2,y2):return f'(polyline (pts (xy {x1} {y1}) (xy {x2} {y2})) (stroke (width .254) (type default)) (fill (type none)))'
 if len(pins)==2 and ref.startswith(('R','C','F','JP')):
  # Both pins share the same row; all generated two-terminal elements are horizontal.
  syy=positions[pins[0][0]][1];edge=abs(positions[pins[0][0]][0])-5.08
  if ref.startswith('C'):
   graphics=ln(-1.27,syy-2.54,-1.27,syy+2.54)+ln(1.27,syy-2.54,1.27,syy+2.54)+ln(-edge,syy,-1.27,syy)+ln(1.27,syy,edge,syy)
  elif ref.startswith('JP'):
   graphics=ln(-edge,syy,-1.27,syy)+ln(1.27,syy,edge,syy)+f'(circle (center -1.27 {syy}) (radius .508) (stroke (width .254) (type default)) (fill (type none)))'+f'(circle (center 1.27 {syy}) (radius .508) (stroke (width .254) (type default)) (fill (type none)))'
  else:
   graphics=f'(rectangle (start -3.81 {syy+1.27}) (end 3.81 {syy-1.27}) (stroke (width .254) (type default)) (fill (type none)))'+ln(-edge,syy,-3.81,syy)+ln(3.81,syy,edge,syy)
   if ref.startswith('F'):graphics+=ln(-3.81,syy,3.81,syy)
 lib=f'(symbol {Q(name)} (pin_names (offset .5)) (exclude_from_sim no) (in_bom yes) (on_board yes) '+prop('Reference',ref,0,half+5.08)+prop('Value',value,0,half+2.54)+f'(symbol {Q(ref+"_0_1")} {graphics}) (symbol {Q(ref+"_1_1")} '
 for n,caption,typ,side,net in pins:
  a,b,angle=positions[n];lib+=f'(pin {typ} line (at {a} {b} {angle}) (length 5.08) (name {Q(caption)} (effects (font (size 1 1)))) (number {Q(n)} (effects (font (size .8 .8))))) '
 libs.append(lib+'))')
 sy=f'(symbol (lib_id {Q(name)}) (at {x} {y} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {Q(U(ref))}) '
 for key,val,dy,hide in [('Reference',ref,-half-5.08,False),('Value',value,-half-2.54,False),('Footprint',footprint,0,True),('Datasheet',url,0,True),('Description',desc,0,True)]:sy+=prop(key,val,x,y+dy,hide)
 for n,*_ in pins:sy+=f'(pin {Q(n)} (uuid {Q(U(ref+"."+n))})) '
 sy+=f'(instances (project "BUSA_R1" (path "/{rid}" (reference {Q(ref)}) (unit 1)))))';items.append(sy)
 for n,caption,typ,side,net in pins:
  a,b,angle=positions[n];a,b=G(x+a),G(y-b)
  if net is None:items.append(f'(no_connect (at {a} {b}) (uuid {Q(U(ref+n+"NC"))}))');continue
  c=G(a+(-10.16 if side=='L' else 10.16))
  items.append(f'(wire (pts (xy {a} {b}) (xy {c} {b})) (stroke (width 0) (type default)) (uuid {Q(U(ref+n+"W"))}))')
  items.append(f'(label {Q(net)} (at {c} {b} {0 if side=="L" else 180}) (effects (font (size 1 1)) (justify left bottom)) (uuid {Q(U(ref+n+"L"))}))')
 parts.append(dict(ref=ref,value=value,pins=[dict(number=n,name=c,type=t,side=z,net=net) for n,c,t,z,net in pins],description=desc,footprint=footprint,datasheet=url))
def two(ref,value,a,b,x,y,desc=''):part(ref,value,x,y,[('1','1','passive','L',a),('2','2','passive','R',b)],desc,width=12)
def resistor(ref,value,a,b,x,y,desc=''):two(ref,value,a,b,x,y,desc)
def cap(ref,value,net,x,y):two(ref,value,net,'GND',x,y)
def conn(ref,value,nets,x,y,desc=''):part(ref,value,x,y,[(str(i+1),n,'passive','R',net) for i,(n,net) in enumerate(nets)],desc,width=29)
def flag(ref,net,x,y):part(ref,'POWER_FLAG',x,y,[('1',net,'power_out','R',net)],'External supply after passive protection',width=12)
def write():
 (root/'BUSA_R1.kicad_sch').write_text(f'(kicad_sch (version 20250114) (generator "busa_r1") (uuid {Q(rid)}) (paper "A1") (title_block (title "BUSA carrier — electrical schematic") (date "2026-11-08") (rev "R1 REVIEW") (comment 1 "ERC and netlist verified; bench/footprints pending"))\n(lib_symbols\n'+ '\n'.join(libs)+')\n'+'\n'.join(items)+'\n(embedded_fonts no))\n')
 (root/'BUSA.kicad_sym').write_text('(kicad_symbol_lib (version 20241209) (generator busa_r1)\n'+'\n'.join(t.replace('\"BUSA:', '\"', 1) for t in libs)+')\n')
 (root/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name \"BUSA\") (type \"KiCad\") (uri \"${KIPRJMOD}/BUSA.kicad_sym\") (options \"\") (descr \"BUSA R1 functional module symbols\")))\n')
 (root/'parts.json').write_text(json.dumps(parts,ensure_ascii=False,indent=2)+'\n')
 print('Generated',len(parts),'symbols')
