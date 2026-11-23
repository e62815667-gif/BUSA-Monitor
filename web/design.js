/* Read-only presentation bindings; controller API and calculator stay separate. */
(()=>{
 'use strict';
 const pairs=[0,2,1,3].map(i=>[document.getElementById('t'+i),document.querySelector('[data-legend-temp="'+i+'"]')]);
 const byId=id=>document.getElementById(id);
 const home=document.querySelector('section[data-section="0"]');
 const stage=byId('rigArtStage');
 home.classList.add('holoHome');
 const metricBar=document.createElement('div');metricBar.className='holoMetrics';metricBar.setAttribute('aria-label','Расход воды и отбор');
 const addMetric=(label,id,rate=false)=>{
  const box=document.createElement('div');box.className='holoMetric';
  const title=document.createElement('span');title.textContent=label;
  const value=rate?document.createElement('b'):byId(id);
  if(rate){value.className='holoRate';value.append(byId(id));const unit=document.createElement('small');unit.textContent=' мл/мин';value.append(unit);}
  box.append(title,value);metricBar.append(box);
 };
 addMetric('Расход','homeCoolingRate');addMetric('Израсходовано','homeWaterUsed');addMetric('Отбор','homeCollectionRate',true);
 home.append(metricBar,home.querySelector('.relayIndicators'));
 const waterSection=document.querySelector('section[data-section="2"]');
 const glass=document.createElement('div');glass.className='visualCollectionGlass';glass.setAttribute('aria-hidden','true');
 glass.innerHTML='<svg viewBox="0 0 110 180" fill="none"><defs><linearGradient id="holoGlass" x1="4" y1="0" x2="109" y2="0" gradientUnits="userSpaceOnUse"><stop stop-color="#d7fbff" stop-opacity=".38"/><stop offset=".16" stop-color="#8bebff" stop-opacity=".04"/><stop offset=".8" stop-color="#8bebff" stop-opacity=".03"/><stop offset="1" stop-color="#d7fbff" stop-opacity=".4"/></linearGradient><linearGradient id="holoLiquid" x1="0" y1="70" x2="0" y2="176" gradientUnits="userSpaceOnUse"><stop stop-color="#00ddff" stop-opacity=".48"/><stop offset="1" stop-color="#0083bc" stop-opacity=".13"/></linearGradient><clipPath id="holoGlassClip"><path d="M8 72H102V158Q102 174 55 174Q8 174 8 158Z"/></clipPath></defs><path d="M5 13V158C5 180 105 180 105 158V13" fill="url(#holoGlass)" stroke="#b9f6ff" stroke-opacity=".7" stroke-width="2"/><g class="liquidFill"><path d="M8 75H102V157C102 177 8 177 8 157Z" fill="url(#holoLiquid)"/><ellipse cx="55" cy="75" rx="47" ry="8" fill="#06c9ed" fill-opacity=".3" stroke="#8cfaff" stroke-opacity=".8"/><g clip-path="url(#holoGlassClip)" stroke="#c4fcff" stroke-width="1.1"><circle class="glassBubble" cx="24" cy="152" r="3"/><circle class="glassBubble" cx="47" cy="151" r="2.1" style="animation-delay:-1.2s"/><circle class="glassBubble" cx="75" cy="154" r="3.6" style="animation-delay:-.7s"/><circle class="glassBubble" cx="87" cy="148" r="2.2" style="animation-delay:-2.1s"/></g></g><ellipse cx="55" cy="13" rx="50" ry="10" fill="#08212c" fill-opacity=".15" stroke="#d1fbff" stroke-opacity=".8" stroke-width="2"/><ellipse cx="55" cy="162" rx="47" ry="10" stroke="#82ecff" stroke-opacity=".6"/></svg>';
 waterSection.querySelector('.collectionPanel').append(glass);
 const pipe=document.createElement('div');pipe.className='coolingPipe';pipe.setAttribute('aria-hidden','true');waterSection.querySelector('.coolingPanel').append(pipe);
 const numberFrom=id=>{const m=byId(id)?.textContent.match(/[-+]?\d+(?:[.,]\d+)?/);return m?Number(m[0].replace(',','.')):null;};
 const sync=()=>{
  for(const [source,target] of pairs)if(source&&target&&target.textContent!==source.textContent)target.textContent=source.textContent;
  stage.classList.toggle('hasTemperature',numberFrom('t2')!==null);
  waterSection.classList.toggle('hasCollectionData',numberFrom('collectionTotal')!==null);
  waterSection.classList.toggle('hasCollectionFlow',(numberFrom('collectionMlMin')||0)>0);
  waterSection.classList.toggle('hasCoolingFlow',stage.classList.contains('hasCooling'));
  for(let i=0;i<4;i++){
   const badge=byId('relayBadge'+i),state=badge.dataset.state,node=byId('soundStatus'+i);
   const text=state==='on'?'Вкл.':state==='off'?'Выкл.':'Нет связи';if(node.textContent!==text)node.textContent=text;
  }
 };
 const observer=new MutationObserver(sync);
 for(const [source] of pairs)if(source)observer.observe(source,{childList:true,characterData:true,subtree:true});
 for(const id of ['collectionTotal','collectionMlMin'])observer.observe(byId(id),{childList:true,characterData:true,subtree:true});
 observer.observe(stage,{attributes:true,attributeFilter:['class']});
 for(let i=0;i<4;i++)observer.observe(byId('relayBadge'+i),{attributes:true,attributeFilter:['data-state']});
 sync();
})();
