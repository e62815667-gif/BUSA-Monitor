"""Prove KiCad catches a real copper short, in a disposable copy only."""
import os,subprocess,tempfile,shutil,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for k,d in [('XDG_CONFIG_HOME','config'),('XDG_CACHE_HOME','cache'),('XDG_DATA_HOME','data')]:os.environ.setdefault(k,str(ROOT/'.runtime'/d))
import pcbnew as p
before=(ROOT/'BUSA_R1.kicad_pcb').read_bytes()
with tempfile.TemporaryDirectory() as tmp:
    tmp=Path(tmp)
    for name in ['BUSA_R1.kicad_pro','BUSA_R1.kicad_sch','BUSA_R1.kicad_pcb','BUSA.kicad_sym','sym-lib-table','fp-lib-table']:shutil.copy2(ROOT/name,tmp/name)
    shutil.copytree(ROOT/'BUSA.pretty',tmp/'BUSA.pretty')
    b=p.LoadBoard(str(tmp/'BUSA_R1.kicad_pcb'));u=next(f for f in b.GetFootprints() if f.GetReference()=='U1')
    a=u.FindPadByNumber('R16');z=u.FindPadByNumber('R17');assert a.GetNetname()!=z.GetNetname()
    tr=p.PCB_TRACK(b);tr.SetStart(a.GetPosition());tr.SetEnd(z.GetPosition());tr.SetWidth(p.FromMM(.5));tr.SetLayer(p.F_Cu);tr.SetNet(a.GetNet());b.Add(tr)
    p.SaveBoard(str(tmp/'BUSA_R1.kicad_pcb'),b)
    r=subprocess.run(['kicad-cli','pcb','drc','--format','json','--exit-code-violations','-o',str(tmp/'fault.json'),str(tmp/'BUSA_R1.kicad_pcb')],capture_output=True,text=True)
    report=json.loads((tmp/'fault.json').read_text())
    assert r.returncode!=0 and any(v['severity']=='error' and v['type'] in ['shorting_items','clearance'] for v in report['violations']),report
    (ROOT/'output/PCB_DRC_negative_control.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
assert (ROOT/'BUSA_R1.kicad_pcb').read_bytes()==before
print('PASS: injected copper short GPIO7 → GPIO6 rejected; actual PCB unchanged')
