(()=>{
 'use strict';
 const C=window.BusaCore,$=id=>document.getElementById(id),text=(id,v)=>{$(id).textContent=v;};
 let generation=0,mode='offline',timer=null,watchdog=null,lastGood=0,current=null,settingsRevision=null,base='',caps=null;
 const pending=new Set();let history=[];
 try{const h=JSON.parse(localStorage.getItem('busa-real-history')||'[]');if(Array.isArray(h))history=h.slice(-1000);}catch{}
 function storeHistory(){try{localStorage.setItem('busa-real-history',JSON.stringify(history));}catch{text('history-status','Хранилище недоступно: история только до закрытия страницы');}}
 function showHistory(){
  text('history-log',history.slice(-100).reverse().map(h=>h.time+' · '+h.temperatures.map(v=>Number.isFinite(v)?v.toFixed(1):'—').join(' / ')+' °C').join('\n')||'История пуста');
 }
 function draw(d,demo=false){
  $('summary').replaceChildren();$('channels').replaceChildren();
  C.names.forEach((name,i)=>{
   const c=d?.channels?.[i],v=c?.valid?d.temperatures[i]:null;
   const tile=document.createElement('div');tile.className='tile';
   const label=document.createElement('small');label.textContent='T'+(i+1)+' · '+name;
   const value=document.createElement('b');value.textContent=Number.isFinite(v)?v.toFixed(1)+' °C':'—';
   const info=document.createElement('small');info.textContent=demo?'Демо':Number.isFinite(v)?'Измерение':c?.fault||'Нет данных';
   tile.append(label,value,info);$('summary').append(tile);
   const row=document.createElement('div');row.className='line';
   row.textContent=name+': '+(Number.isFinite(v)?v.toFixed(2)+' °C':'Нет данных')+
    (c?.warning_reached===true?' · предупреждение':'')+
    (c?.automation_threshold_reached===true?' · достигнут порог автоматики (без управления)':'');
   $('channels').append(row);
  });
  text('dev',demo?'Демо':d?.device||'Нет связи');
  text('bridge',demo?'Демо':d?.bridge_online?'На связи':d?.sensors_enabled===false?'Опрос выключен до проверки':'Нет данных');
  text('keypad',d?.keypad_online?'На связи':'Отключена / не проверена');
  text('level-value',d?.level_mode==='manual'&&Number.isFinite(d.volume_l)?d.volume_l+' л · введено вручную':'Нет данных');
  text('lastkey',d?.last_key||'—');text('key2',d?.last_key||'—');text('keyseq',d?.key_seq??'—');
 }
 function stop(){
  ++generation;clearTimeout(timer);clearInterval(watchdog);for(const c of pending)c.abort();pending.clear();
  lastGood=0;current=null;caps=null;settingsRevision=null;$('save-settings').disabled=true;
  text('firmware-version','Не подключена');text('keyhistory','Нет данных');text('keycount','0');
 }
 function offline(reason){
  current=null;lastGood=0;caps=null;$('save-settings').disabled=true;draw(null);
  text('badge','Нет связи');text('connection-status',reason);text('notice','Свежих данных нет. История не является текущим измерением. Выходы отключены.');
  text('keyhistory','Нет связи');text('keycount','0');
 }
 function address(){
  const u=new URL($('url').value.trim());
  if(!['http:','https:'].includes(u.protocol)||u.username||u.password||u.search||u.hash||u.pathname!=='/')throw Error('Укажите http(s)://адрес без пути и пароля');
  return u.origin;
 }
 async function request(path,gen,options={}){
  const c=new AbortController();pending.add(c);let timeout;
  try{return await Promise.race([
   (async()=>{const r=await fetch(base+path,{cache:'no-store',signal:c.signal,...options});if(!r.ok)throw Error('HTTP '+r.status);const d=await r.json();if(gen!==generation)throw Error('Устаревший запрос');return d;})(),
   new Promise((_,reject)=>{timeout=setTimeout(()=>{c.abort();reject(Error('Тайм-аут'));},4000);})
  ]);}finally{clearTimeout(timeout);pending.delete(c);}
 }
 function fillSettings(s){
  C.settings(s);settingsRevision=s.revision;
  for(let i=0;i<4;i++){$('warning-'+i).value=s.warning[i]??'';$('automation-'+i).value=s.automation[i]??'';}
  $('volume').value=s.volume_l??'';text('settings-status','Загружена ревизия '+s.revision+'. Пусто = порог не задан.');
 }
 async function poll(gen){
  if(gen!==generation||mode!=='real')return;
  try{
   const c=C.capabilities(await request('/api/capabilities',gen));
   const d=C.telemetry(await request('/api/telemetry',gen));if(d.version!==c.version)throw Error('Прошивка сменилась');
   if(gen!==generation)return;caps=c;current=d;lastGood=Date.now();draw(d);
   text('badge','Реальная телеметрия');text('connection-status','Контроллер отвечает · '+c.version);
   text('firmware-version',c.version);text('notice','Лабораторный мониторинг. Реле K1–K4 физически отключены.');text('display-state',c.display?'Включён':'Отключён / не проверен');
   $('save-settings').disabled=c.settings_write!==true;
   // Record real frames once, including channel errors, never demo.
   const last=history.at(-1);
   if(!last||last.stream!==d.key_stream_id||last.sequence!==d.sequence){
    history.push({time:new Date().toISOString(),stream:d.key_stream_id,sequence:d.sequence,temperatures:d.temperatures.slice(),faults:d.channels.map(x=>x.fault||null),volume_l:d.volume_l??null});
    history=history.slice(-1000);storeHistory();showHistory();
   }
   if(c.settings_write===true&&settingsRevision===null)fillSettings(await request('/api/settings',gen));
   try{const k=await request('/api/keys',gen);
    if(k.stream_id!==d.key_stream_id||!Array.isArray(k.events)||k.events.length>64||!k.events.every(e=>Number.isInteger(e.seq)&&typeof e.key==='string'))throw Error('Неверный журнал');
    if(gen!==generation||!lastGood)return;text('keycount',k.events.length);text('keyhistory',k.events.map(e=>e.seq+': '+e.key).join('\n')||'Нет нажатий; клавиатура может быть отключена');
   }catch(e){if(gen===generation){text('keyhistory','Журнал недоступен');text('keycount','0');}}
  }catch(e){if(gen===generation)offline(e.message);}
  finally{if(gen===generation&&mode==='real')timer=setTimeout(()=>poll(gen),1500);}
 }
 function connect(){
  stop();mode='real';draw(null);text('badge','Подключение…');
  try{base=address();if(location.protocol==='https:'&&base.startsWith('http:'))throw Error('Откройте локальную страницу ESP32: HTTPS→HTTP блокируется браузером');
   const gen=generation;watchdog=setInterval(()=>{if(lastGood&&Date.now()-lastGood>6000)offline('Показания устарели');},500);poll(gen);
  }catch(e){offline(e.message);}
 }
 $('connect').onclick=connect;
 $('demo').onclick=()=>{
  stop();mode='demo';text('badge','ДЕМО — не измерения');text('notice','Условные значения. В реальную историю не записываются. Настройки не отправляются.');
  text('connection-status','Демо без оборудования');draw({temperatures:[22,23,24,25],channels:C.names.map(()=>({valid:true}))},true);
 };
 $('local').onclick=()=>{try{location.href=address();}catch(e){text('connection-status',e.message);}};
 $('load-settings').onclick=async()=>{try{if(mode!=='real'||!lastGood)throw Error('Нет связи');const g=generation;fillSettings(await request('/api/settings',g));}catch(e){text('settings-status',e.message);}};
 $('save-settings').onclick=async()=>{
  try{
   if(mode!=='real'||!lastGood||Date.now()-lastGood>6000||!caps?.settings_write||settingsRevision===null)throw Error('Сначала подключитесь и загрузите настройки');
   const token=$('pair-token').value.trim();if(!/^[a-f0-9]{32}$/.test(token))throw Error('Введите токен из USB-журнала этой загрузки');
   const s=C.settings({revision:settingsRevision,warning:C.names.map((_,i)=>C.parseInput($('warning-'+i).value,-55,125)),
    automation:C.names.map((_,i)=>C.parseInput($('automation-'+i).value,-55,125)),volume_l:C.parseInput($('volume').value,0,100),physical_outputs:false});
   const g=generation;fillSettings(await request('/api/settings',g,{method:'POST',headers:{'Content-Type':'application/json','X-BUSA-Token':token},body:JSON.stringify(s)}));
   text('settings-status','Сохранено на ESP32. Реле не управляются.');
  }catch(e){text('settings-status',e.message+' · При HTTP 409 загрузите настройки заново.');}
 };
 $('export-history').onclick=()=>{
  const b=new Blob([JSON.stringify(history,null,2)],{type:'application/json'}),u=URL.createObjectURL(b),a=document.createElement('a');
  a.href=u;a.download='BUSA-real-history.json';a.click();setTimeout(()=>URL.revokeObjectURL(u),1000);
 };
 document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('main section').forEach(s=>s.classList.add('hidden'));$(b.dataset.tab).classList.remove('hidden');
  document.querySelectorAll('nav button').forEach(x=>x.classList.toggle('active',x===b));
 });
 showHistory();draw(null);text('badge','Нет связи');$('save-settings').disabled=true;
 const local=location.protocol==='http:'&&location.hostname!=='e62815667-gif.github.io';
 if(local){$('url').value=location.origin;connect();}
 if(location.protocol==='https:'&&'serviceWorker'in navigator)navigator.serviceWorker.register('./sw.js').catch(()=>{});
})();

