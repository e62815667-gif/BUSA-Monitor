"""Import a local Specctra result without treating it as production approval."""
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for k,d in [('XDG_CONFIG_HOME','config'),('XDG_CACHE_HOME','cache'),('XDG_DATA_HOME','data')]:os.environ.setdefault(k,str(ROOT/'.runtime'/d))
import pcbnew as p
b=p.LoadBoard(str(ROOT/'BUSA_R1.kicad_pcb'))
assert p.ImportSpecctraSES(b,str(ROOT/'output/BUSA_R1.ses'))
b.BuildConnectivity()
p.SaveBoard(str(ROOT/'BUSA_R1.kicad_pcb'),b)
print('Imported local routes:',len(b.GetTracks()),'segments/vias')
