/* One-screen local interface. Pagination changes presentation only, never device APIs. */
(()=>{
'use strict';
const $=s=>document.querySelector(s),main=$('main'),nav=$('.appTabs'),header=$('.appHeader');
document.body.append(nav);document.documentElement.classList.add('viewportFit');
const pager=document.createElement('div');pager.className='vpPager';pager.hidden=true;
const prev=document.createElement('button'),next=document.createElement('button'),count=document.createElement('span');
prev.type=next.type='button';prev.textContent='‹ Назад';next.textContent='Далее ›';pager.append(prev,count,next);pager.addEventListener('click',e=>e.stopPropagation());main.append(pager);
// Copy the complete report even when its text is on another presentation page.
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
let pending=false,busy=false,currentHost=null,currentPage=0,pages=[],focusUnit=null,initialHeight=innerHeight,lastChartSize="";
const visible=e=>!!e&&!e.hidden&&getComputedStyle(e).display!=='none';
const set=(e,n,v)=>{if(e.style.getPropertyValue(n)!==v)e.style.setProperty(n,v);};
function request(){if(!pending){pending=true;requestAnimationFrame(()=>{pending=false;fit();});}}
function size(){
 const v=window.visualViewport,h=v&&Math.abs(v.scale-1)<.01?v.height:innerHeight;
 set(document.documentElement,'--app-height',Math.round(h)+'px');
 document.body.classList.toggle('keyboardViewport',h<initialHeight*.65&&/INPUT|TEXTAREA|SELECT/.test(document.activeElement?.tagName||''));
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
function view(){
 const section=Array.from(main.querySelectorAll(':scope>.page')).find(visible);if(!section)return null;
 const id=Number(section.dataset.section);
 if(id===0||id===3)return {section,id};
 if(id===1)return {section,id,host:section,units:[...section.querySelectorAll('.approvedSensor'),section.querySelector('.temperatureSaveRow')],key:'temperature'};
 if(id===2)return {section,id,host:section,units:[...section.querySelectorAll('.liquidPanel')],key:'liquids'};
 if(id===7)return {section,id,unchanged:true,key:'calculator'};
 if(id===5){
  const menu=$('#settingsMenu');if(visible(menu))return {section,id,host:menu,units:[...menu.children],key:'settings-menu'};
  const card=Array.from(section.querySelectorAll('.settingCard')).find(visible);if(!card)return null;
  if(card.dataset.stab==='1')return {section,id,unchanged:true,key:'wifi'};
  if(card.dataset.stab==='6'){
   const host=Array.from(card.querySelectorAll('[data-diag-panel]')).find(visible);if(!host)return null;
   return {section,id,host,units:Array.from(host.children).filter(e=>e.tagName!=='H2'),key:'diag-'+host.dataset.diagPanel};
  }
  if(card.dataset.stab==='8'){
   const host=$('#events'),rows=[...host.children];return {section,id,host,units:rows.length?rows:[],key:'events'};
  }
  return {section,id,host:card,units:Array.from(card.children).filter(e=>e.tagName!=='H2'),key:'setting-'+card.dataset.stab};
 }
 return null;
}
function clearUnits(){
 document.querySelectorAll('.vpHidden').forEach(e=>e.classList.remove('vpHidden'));
 document.querySelectorAll('.vpLongWindow').forEach(e=>{e.style.height='auto';const p=e.firstElementChild;if(p&&p.tagName!=='TEXTAREA')p.style.transform='';});
}
function wrapper(e){
 if(e.parentElement?.classList.contains('vpLongWindow'))return e.parentElement;
 const w=document.createElement('div');w.className='vpLongWindow';e.before(w);w.append(e);return w;
}
function unitsOf(v){
 // Keep a label with its following input/select; every field stays reachable.
 const rows=[],list=v.units.filter(e=>e&&visible(e));
 for(let i=0;i<list.length;i++){
  const e=list[i];
  if(e.tagName==='LABEL'&&list[i+1]&&/^(INPUT|SELECT)$/.test(list[i+1].tagName)){
   const w=document.createElement('div');w.className='vpField';e.before(w);w.append(e,list[++i]);rows.push(w);
  }else rows.push(e);
 }
 // Newly-created wrappers replace those original direct children on subsequent fits.
 return rows;
}
function measure(v,reserved){
 const host=v.host;
 set(host,'height','auto');set(host,'max-height','none');set(host,'flex','0 0 auto');
 const top=host.getBoundingClientRect().top;
 const height=Math.max(50,main.getBoundingClientRect().bottom-top-reserved-3);
 set(host,'height',height+'px');set(host,'box-sizing','border-box');
 let units=unitsOf(v);const records=[];
 let first=units.find(e=>e.getBoundingClientRect().height>0);
 let head=first?Math.max(0,first.getBoundingClientRect().top-top):0;
 const pad=parseFloat(getComputedStyle(host).paddingBottom)||0,budget=Math.max(35,height-head-pad-14);

 for(let e of units){
  if(!visible(e))continue;
  const style=getComputedStyle(e),h=e.getBoundingClientRect().height+(parseFloat(style.marginTop)||0)+(parseFloat(style.marginBottom)||0);
  if(h<=.5)continue;
  if((h>budget+1||(e.tagName==='TEXTAREA'&&e.scrollHeight>e.clientHeight+1))&&!e.classList.contains('vpLongWindow')&&(/^(PRE|TEXTAREA)$/.test(e.tagName)||!e.querySelector('input,select,button,textarea'))){
   const original=e,w=wrapper(e),natural=original.tagName==='TEXTAREA'?Math.max(original.scrollHeight,h):h;
   const slices=Math.max(1,Math.ceil(natural/budget));
   for(let i=0;i<slices;i++)records.push({el:w,original,slice:i,step:budget,h:Math.min(budget,natural-i*budget),exclusive:true});
  }else if(e.classList.contains('vpLongWindow')){
   const original=e.firstElementChild,natural=original.tagName==='TEXTAREA'?Math.max(original.scrollHeight,h):original.getBoundingClientRect().height+(parseFloat(getComputedStyle(original).marginTop)||0)+(parseFloat(getComputedStyle(original).marginBottom)||0);
   for(let i=0;i<Math.ceil(natural/budget);i++)records.push({el:e,original,slice:i,step:budget,h:Math.min(budget,natural-i*budget),exclusive:true});
  }else records.push({el:e,h,exclusive:false});
 }
 const result=[];let page=[],used=0;
 for(const r of records){
  if(page.length&&(used+r.h>budget||r.exclusive)){result.push(page);page=[];used=0;}
  page.push(r);used+=r.h+3;
  if(r.exclusive){result.push(page);page=[];used=0;}
 }
 if(page.length)result.push(page);
 return result.length?result:[[]];
}
function show(v){
 const all=new Set(pages.flat().map(x=>x.el));for(const e of all)e.classList.add('vpHidden');
 for(const r of pages[currentPage]||[]){
  r.el.classList.remove('vpHidden');
  if(r.original){
   r.el.style.height=r.h+'px';
   if(r.original.tagName==='TEXTAREA'){r.original.style.maxHeight='none';r.original.style.minHeight='0';r.original.style.height=r.h+'px';r.original.scrollTop=r.slice*r.step;}
   else r.original.style.transform='translateY(-'+(r.slice*r.step)+'px)';
  }
 }
 const text=(currentPage+1)+' / '+pages.length;if(count.textContent!==text)count.textContent=text;
 prev.disabled=currentPage===0;next.disabled=currentPage===pages.length-1;
 pager.hidden=pages.length<=1;
 if(focusUnit&&pages[currentPage]?.some(r=>r.el===focusUnit||r.el.contains(focusUnit)))focusUnit=null;
}
function fit(){
 if(busy)return;busy=true;
 try{
  const initial=view();if(!initial)return;
  if(initial.unchanged){document.documentElement.classList.remove('viewportFit');document.documentElement.dataset.viewportMode='unchanged';pager.hidden=true;clearUnits();currentHost=null;return;}
  document.documentElement.classList.add('viewportFit');document.documentElement.dataset.viewportMode='fit';
  size();if(scrollY!==0)scrollTo(0,0);clearUnits();pager.hidden=true;
  const v=view();if(!v)return;
  if(v.id===0){currentHost=null;fitHome(v.section);return;}
  if(v.id===3){currentHost=null;const c=$('#chartCanvas'),r=c.getBoundingClientRect(),size=r.width+'x'+r.height;if(size!==lastChartSize&&typeof graphs==='function'){lastChartSize=size;graphs();}return;}
  if(!v.host)return;
  if(currentHost!==v.key){currentHost=v.key;currentPage=0;}
  pages=measure(v,0);
  if(pages.length>1){pager.hidden=false;clearUnits();pages=measure(view(),32);}
  if(focusUnit){const i=pages.findIndex(p=>p.some(r=>r.el===focusUnit||r.el.contains(focusUnit)));if(i>=0)currentPage=i;}
  currentPage=Math.min(Math.max(currentPage,0),pages.length-1);show(v);
 }finally{busy=false;document.documentElement.dataset.viewportReady='1';}
}
prev.onclick=()=>{if(currentPage>0){currentPage--;show(view());}};
next.onclick=()=>{if(currentPage<pages.length-1){currentPage++;show(view());}};
document.addEventListener('click',e=>{if(!e.target.closest('.vpPager'))request();});
document.addEventListener('toggle',e=>{focusUnit=e.target.closest('.approvedSensor')||e.target;request();},true);
document.addEventListener('focusin',e=>{focusUnit=e.target;request();});
document.addEventListener('focusout',request);
if(typeof ResizeObserver!=='undefined')new ResizeObserver(request).observe(main);
new MutationObserver(rows=>{if(rows.some(r=>!r.target.closest?.('.vpPager')))request();}).observe(main,{subtree:true,childList:true,attributes:true,attributeFilter:['hidden','open']});
window.addEventListener('resize',request);window.addEventListener('orientationchange',request);window.visualViewport?.addEventListener('resize',request);
window.addEventListener('pageshow',request);
request();
})();
