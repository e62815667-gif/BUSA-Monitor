"""Make browser-test failure diagnostics visible in public Actions annotations."""
from pathlib import Path
import sys,os
text='\n'.join(Path(sys.argv[1]).read_text(errors='replace').splitlines()[-80:])
summary=os.environ.get('GITHUB_STEP_SUMMARY')
if summary:
 with open(summary,'a') as f:f.write('### Calculator browser checks\n\n~~~text\n'+text+'\n~~~\n')
if int(sys.argv[2]):
 escaped=text.replace('%','%25').replace('\r','%0D').replace('\n','%0A')
 print('::error title=Calculator browser diagnostics::'+escaped)
