/* UI14: one-screen navigation; long content scrolls within its own panel. */
(()=>{
'use strict';
const $=s=>document.querySelector(s),main=$('main'),nav=$('.appTabs'),header=$('.appHeader');
document.body.append(nav);document.documentElement.classList.add('viewportFit');
// Copy the complete report, including text outside the visible report window.
$('#diagCopy')?.addEventListener('click',async e=>{
 if(!document.documentElement.classList.contains('viewportFit'))return;
 e.preventDefault();e.stopImmediatePropagation();
 const text=typeof lastDiagnosticReport==='string'?lastDiagnosticReport:'';
 if(!text){$('#diagCopyResult').textContent='Отчёт ещё не получен';return;}
 const fallback=()=>{const t=document.createElement('textarea');t.value=text;t.readOnly=true;t.style.cssText='position:fixed;left:0;top:0;width:1px;height:1px;opacity:0';document.body.append(t);t.focus();t.select();t.setSelectionRange(0,t.value.length);let ok=false;try{ok=document.execCommand('copy');}finally{t.remove();}return ok;};
 let ok=false;
 try{if(navigator.clipboard&&window.isSecureContext){await navigator.clipboard.writeText(text);ok=true;}else ok=fallback();}catch(_){try{ok=fallback();}catch(_){}}
 $('#diagCopyResult').textContent=ok?'Скопировано — вставь в чат':'Не удалось скопировать отчёт';
},true);

let pending=false,busy=false,initialHeight=innerHeight,lastChartSize='';
const visible=e=>!!e&&!e.hidden&&getComputedStyle(e).display!=='none';
const set=(e,n,v)=>{if(e.style.getPropertyValue(n)!==v)e.style.setProperty(n,v);};
function request(){if(!pending){pending=true;requestAnimationFrame(()=>{pending=false;fit();});}}
function size(){
 const v=window.visualViewport,h=v&&Math.abs(v.scale-1)<.01?v.height:innerHeight;
 set(document.documentElement,'--app-height',Math.round(h)+'px');
 document.body.classList.toggle('keyboardViewport',h<initialHeight*.65&&/INPUT|TEXTAREA|SELECT/.test(document.activeElement?.tagName||''));
 document.body.classList.toggle('shortFormViewport',main.clientHeight<280);
}
function fitHome(section){
 const panel=section.querySelector('.approvedDashboard'),w=panel.clientWidth,H=panel.clientHeight;
 let top=Math.min(61,Math.max(45,H*.14));const bottom=25;
 let h=Math.max(70,H-top-bottom),sw=h*1024/1536;
 if(sw>w*1.02){sw=w*1.02;h=sw*1536/1024;}
 top+=Math.max(0,H-top-bottom-h);
 const left=(w-sw)/2;
 for(const [n,v] of Object.entries({'--scene-w':sw+'px','--scene-h':h+'px','--scene-left':left+'px','--scene-top':top+'px','--panel-w':w+'px'}))set(panel,n,v);
 section.classList.toggle('smallScene',sw<220);
}
function fitGraph(){
 const c=$('#chartCanvas'),card=c.closest('.chartCard');
 card.style.zoom='';card.style.width='';
 c.style.removeProperty('height');c.style.removeProperty('aspect-ratio');
 const natural=c.getBoundingClientRect().height;
 const excess=card.getBoundingClientRect().bottom-main.getBoundingClientRect().bottom;
 if(excess>0){set(c,'height',Math.max(40,natural-excess-2)+'px');set(c,'aspect-ratio','auto');}
 if(card.getBoundingClientRect().bottom>main.getBoundingClientRect().bottom){const room=main.getBoundingClientRect().bottom-card.getBoundingClientRect().top-2;const zoom=Math.min(1,room/card.getBoundingClientRect().height);card.style.zoom=String(zoom);card.style.width=(100/zoom)+'%';}
 const r=c.getBoundingClientRect(),size=r.width+'x'+r.height;
 if(size!==lastChartSize&&typeof graphs==='function'){lastChartSize=size;graphs();}
}

function room(host){return Math.max(40,Math.floor(main.getBoundingClientRect().bottom-host.getBoundingClientRect().top-4));}
function fitMenu(menu){
 const items=[...menu.children],height=room(menu),columns=height<items.length*32?2:1;
 const rows=Math.ceil(items.length/columns),row=Math.min(53,Math.floor((height-2)/rows));
 set(menu,'--menu-columns',String(columns));set(menu,'--menu-row-height',row+'px');
}
function fitTemperatures(section){
 const list=section.querySelector('.approvedSensorList'),wide=section.clientWidth>=560&&main.clientHeight<320;
 section.classList.toggle('wideContent',wide);
 const count=wide?2:4,head=Math.max(48,Math.min(74,Math.floor((list.clientHeight-7*(count-1))/count)-2));
 set(section,'--sensor-row-height',head+'px');
}
function fitScroll(host){
 host.classList.add('vpScroll');set(host,'--content-room',room(host)+'px');
}
function focusInPanel(){
 const field=document.activeElement;if(!field||!/^(INPUT|SELECT|TEXTAREA)$/.test(field.tagName))return;
 const host=field.closest('.vpScroll,.approvedSensorList');if(!host)return;
 const f=field.getBoundingClientRect(),r=host.getBoundingClientRect();
 if(f.bottom>r.bottom-3)host.scrollTop+=f.bottom-r.bottom+6;
 else if(f.top<r.top+3)host.scrollTop-=r.top-f.top+6;
}
function fit(){
 if(busy)return;busy=true;
 try{
  const section=[...main.querySelectorAll(':scope>.page')].find(visible);if(!section)return;
  const id=Number(section.dataset.section);
  if(id===7){document.documentElement.classList.remove('viewportFit');document.documentElement.dataset.viewportMode='unchanged';return;}
  document.documentElement.classList.add('viewportFit');document.documentElement.dataset.viewportMode='fit';
  size();if(scrollY!==0)scrollTo(0,0);
  if(id===0){fitHome(section);return;}
  if(id===3){fitGraph();return;}
  if(id===1){fitTemperatures(section);focusInPanel();return;}
  if(id===2){section.classList.toggle('compactContent',main.clientHeight<500);section.classList.toggle('wideContent',section.clientWidth>=560&&main.clientHeight<320);return;}
  if(id===5){
   const menu=$('#settingsMenu');if(visible(menu)){fitMenu(menu);return;}
   const card=[...section.querySelectorAll('.settingCard')].find(visible);if(!card)return;
   if(card.dataset.stab!=='0'&&card.dataset.stab!=='1')fitScroll(card);
   focusInPanel();
  }
 }finally{busy=false;document.documentElement.dataset.viewportReady='1';}
}
document.addEventListener('click',request);
document.addEventListener('toggle',request,true);
document.addEventListener('focusin',request);
document.addEventListener('focusout',request);
if(typeof ResizeObserver!=='undefined'){
 const resize=new ResizeObserver(request);resize.observe(main);
 const dashboard=main.querySelector('.approvedDashboard');if(dashboard)resize.observe(dashboard);
}
new MutationObserver(rows=>{
 const home=main.querySelector('[data-section="0"]'),homeActive=home&&!home.hidden;
 if(rows.some(r=>r.type==='attributes'||!homeActive))request();
}).observe(main,{subtree:true,childList:true,attributes:true,attributeFilter:['hidden','open']});
window.addEventListener('resize',request);window.addEventListener('orientationchange',request);window.visualViewport?.addEventListener('resize',request);
window.addEventListener('pageshow',request);
request();
})();
