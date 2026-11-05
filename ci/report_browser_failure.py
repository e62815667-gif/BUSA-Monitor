"""Make browser-test failure diagnostics visible in public Actions annotations."""
from pathlib import Path
import sys,os
lines=Path(sys.argv[1]).read_text(errors='replace').splitlines()
# GitHub truncates annotations at ~4 KiB: retain the exception, not progress bars.
lines=[line for line in lines if line.strip() and not line.lstrip().startswith('|')]
text='\n'.join(lines[-45:]).encode()[-3000:].decode(errors='replace')
summary=os.environ.get('GITHUB_STEP_SUMMARY')
if summary:
 with open(summary,'a') as f:f.write('### Calculator browser checks\n\n~~~text\n'+text+'\n~~~\n')
if int(sys.argv[2]):
 escaped=text.replace('%','%25').replace('\r','%0D').replace('\n','%0A')
 print('::error title=Calculator browser diagnostics::'+escaped)
