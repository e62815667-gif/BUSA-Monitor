"""Embed the local viewport presentation module; --check is read-only."""
from pathlib import Path
import re,sys
root=Path(__file__).resolve().parents[1];p=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h';s=p.read_text()
css=(root/'web/viewport.css').read_text();js=(root/'web/viewport.js').read_text()
def scoped(text):
    text=re.sub(r'/\*.*?\*/','',text,flags=re.S)
    out=[];pos=0
    while pos<len(text):
        opening=text.find('{',pos)
        if opening<0:break
        head=text[pos:opening].strip();depth=1;end=opening+1
        while end<len(text) and depth:
            if text[end]=='{':depth+=1
            elif text[end]=='}':depth-=1
            end+=1
        assert depth==0,'Unbalanced viewport CSS'
        inner=text[opening+1:end-1]
        if head.startswith('@media'):out.append(head+'{'+scoped(inner)+'}')
        else:
            selectors=[]
            for selector in head.split(','):
                selector=selector.strip()
                selectors.append('html.viewportFit'+selector[4:] if selector.startswith('html') else '.viewportFit '+selector)
            out.append(','.join(selectors)+'{'+inner+'}')
        pos=end
    return '\n'.join(out)
css=scoped(css)
start='/* BUSA_VIEWPORT_CSS_BEGIN */';end='/* BUSA_VIEWPORT_CSS_END */'
jstart='/* BUSA_VIEWPORT_JS_BEGIN */';jend='/* BUSA_VIEWPORT_JS_END */'
if start not in s:
 s=s.replace('</style>',start+'\n'+css+'\n'+end+'\n</style>',1)
 s=s.replace('<script src="/calculator.js?v=CALC1"></script>', '<script src="/calculator.js?v=CALC1"></script><script>\n'+jstart+'\n'+js+'\n'+jend+'\n</script>',1)
else:
 s=re.sub(re.escape(start)+'.*?'+re.escape(end),lambda _:start+'\n'+css+'\n'+end,s,flags=re.S)
 s=re.sub(re.escape(jstart)+'.*?'+re.escape(jend),lambda _:jstart+'\n'+js+'\n'+jend,s,flags=re.S)
if '--check' in sys.argv:assert p.read_text()==s,'Viewport assets are stale'
else:p.write_text(s)
print('PASS viewport presentation assets'+(' (read-only)' if '--check' in sys.argv else ' embedded'))
