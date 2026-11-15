/* Remote presentation uses the same UI and real telemetry; no credentials in JS. */
(function(){
 if(window.busaCloudInstalled)return;window.busaCloudInstalled=true;
 const localMessage=message;
 message=function(text,kind){
  if(text==='Нет ответа ESP32: проверь локальный Wi-Fi')text='Нет свежих данных Google. Проверь интернет и отправку БУСА.';
  if(text==='Связь с ESP32 восстановлена')text='Получены свежие показания БУСА через интернет';
  localMessage(text,kind);
 };
 wifiRefreshNetworks=async function(){
  const select=get('net');select.disabled=true;
  try{
   const r=await fetch('/api/status');if(!r.ok)throw Error();const s=await r.json();
   const name=s.station?.ssid||'Нет подключения';select.replaceChildren(new Option(name,name));
   get('response').textContent=s.station?.connected?'Wi-Fi прибора: '+name:'ESP32 не подключён к Wi-Fi';
  }catch(e){get('response').textContent='Нет свежих данных о Wi-Fi прибора';}
 };
 networkLeds=function(info,station){
  const fresh=info?.cloud_fresh===true;
  get('internetLed').classList.toggle('internetOn',fresh&&station?.connected===true);
  get('googleLed').classList.toggle('googleOn',fresh);
  get('localLed').classList.toggle('localOn',fresh);
  get('internetLabel').textContent=fresh&&station?.connected===true?'ПОДКЛ':'ОТКЛ';
  get('googleLabel').textContent=fresh?'ПОДКЛ':'ОТКЛ';
  get('localLabel').textContent=fresh?'ДАННЫЕ':'НЕТ ДАННЫХ';
 };
 // Google is a telemetry channel. Device setup and uploading firmware require
 // a direct connection; these controls must not falsely report a remote save.
 for(const id of ['temperatureSave','tempCalSave','save','disconnectWifi','wifiRetry',
  'googleSave','googleTest','googleStop','manualUpdate','firmwareFile',
  'keyStart','keyNext','keyUndo','keyCancel','tftSave','tftTest','tftTheme',
  'screenSleep','screenWake','rmStart','rmStop','rmCheck']){
  const element=get(id);if(element)element.disabled=true;
 }
 for(const id of ['net','pass','googleUrl','googleToken','tftDriver','tftRotation','themeModeSelect']){
  const element=get(id);if(element)element.disabled=true;
 }
 for(const index of [0,1,2,3,5,7]){
  const panel=document.querySelector('[data-stab="'+index+'"]');
  if(panel&&!panel.querySelector('.cloudNote')){
   const note=document.createElement('p');note.className='hint cloudNote';
   note.textContent='Изменение настроек прибора доступно при подключении к Wi-Fi БУСА. Через интернет показываются данные из Google.';
   panel.prepend(note);
  }
 }
 const connectMenu=document.createElement('button');connectMenu.type='button';connectMenu.textContent='Подключение через интернет';
 connectMenu.className='btn';connectMenu.onclick=()=>{location.href='busa://connection';};
 const google=get('googleResult');if(google)google.after(connectMenu);
})();
