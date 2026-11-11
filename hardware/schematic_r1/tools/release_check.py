"""Fail closed on missing engineering evidence; produce a readable status file.

Run after ERC, full DRC with schematic parity, PCB audit and BOM generation.
This checker does not manufacture, order, send email, or invent verification.
"""
from pathlib import Path
import json,hashlib,csv,sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output'
par=json.loads((ROOT/'pcb_parameters.json').read_text());audit=json.loads((OUT/'PCB_CHECKS.json').read_text());drc=json.loads((OUT/'PCB_DRC.json').read_text());erc=json.loads((OUT/'ERC.json').read_text())
blocked=[k for k,v in par['release_gates'].items() if v is not True]
if hashlib.sha256((ROOT/'BUSA_R1.kicad_pcb').read_bytes()).hexdigest()!=audit['pcb_sha256']:blocked.append('pcb_audit_is_stale')
if drc['violations'] or drc['unconnected_items'] or drc['schematic_parity']:blocked.append('drc_or_parity_failed')
if any(s['violations'] for s in erc['sheets']):blocked.append('erc_failed')
rows=list(csv.DictReader((OUT/'BOM_REVIEW.csv').open(encoding='utf-8-sig')))
tbd=[r['Reference'] for r in rows if 'TBD' in r['Candidate_MPN_NOT_PURCHASE_RELEASE']]
if tbd:blocked.append('exact_mpn_selection_incomplete')
status={'status':'NOT_RELEASED' if blocked else 'READY_FOR_RELEASE_REVIEW','manufacturer':par['manufacturer'],'board_mm':[120,100,1.6],'erc_violations':sum(len(s['violations']) for s in erc['sheets']),'drc_violations':len(drc['violations']),'unconnected_items':len(drc['unconnected_items']),'schematic_parity_issues':len(drc['schematic_parity']),'connected_pads_audited':audit['expected_connected_pads'],'blocking_items':blocked,'mpn_tbd_references':tbd,'fabrication_files_created':False,'assembly_files_released':False,'order_submitted':False,'note':'An engineering review archive is not a production order or a validated assembly package.'}
(OUT/'RELEASE_STATUS.json').write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n')
print(status['status']+': '+', '.join(blocked))
sys.exit(2 if blocked else 0)
