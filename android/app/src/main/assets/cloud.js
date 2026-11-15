/* Remote presentation uses the same UI and real telemetry; no credentials in JS. */
(function(){
 if(window.busaCloudInstalled)return;window.busaCloudInstalled=true;
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
