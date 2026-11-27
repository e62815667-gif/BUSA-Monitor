/* DESIGN-003: preserve the original apparatus and live telemetry IDs on Home. */
(()=>{
 'use strict';
 const home=document.querySelector('section[data-section="0"]');
 if(!home || !home.querySelector('#rigArtStage'))return;
 const dashboard=home.querySelector('.approvedDashboard');
 const metrics=home.querySelector('.holoMetrics');
 if(!dashboard || !metrics)return;
 const existingArt=home.querySelector('#rigArtStage .approvedRigArt');
 const originalArtSource=existingArt?.getAttribute('src');
 home.classList.add('busaApprovedHome003');
 const statuses=document.createElement('div');
 statuses.className='busaApprovedStatus003';
 const heater=document.createElement('span');heater.className='busaStatusHeater';heater.textContent='ТЭН —';
 const valve=document.createElement('span');valve.className='busaStatusValve';valve.textContent='Кран —';
 statuses.append(heater,valve);dashboard.append(statuses);
 const alarm=document.createElement('span');alarm.className='busaApprovedAlarm003';alarm.textContent='АВАРИЯ';alarm.hidden=true;dashboard.append(alarm);
 const values=[['Расход','homeCoolingRate'],['Объём','homeWaterUsed'],['Отбор','homeCollectionRate'],['Собрано','homeCollectionTotal']];
 const readings=values.map(([,id])=>document.getElementById(id));
 const volumeUnit=document.getElementById('homeCollectionUnit');
 const cells=[];
 for(const [index,[name,id]] of values.entries()){
   const cell=document.createElement('div');cell.className='holoMetric';
   const title=document.createElement('span');title.textContent=name;
   const reading=document.createElement('b');
   if(readings[index])reading.append(readings[index]);else reading.textContent='—';
   if(id==='homeCollectionRate')reading.append(document.createTextNode(' мл/мин'));
   if(id==='homeCollectionTotal')reading.append(document.createTextNode(' '),volumeUnit||document.createTextNode('мл'));
   cell.append(title,reading);cells.push(cell);
 }
 metrics.replaceChildren(...cells);
 window.busaApprovedHomeUpdate=function(s){
   const sound=s?.sound_alerts;
   const channels=Array.isArray(sound?.channels)&&sound.channels.length===4?sound.channels:null;
   const known=!!(channels&&channels.every(channel=>typeof channel?.sound_on==='boolean'));
   // K1 sounds continuously in normal mode. K2/K3 notify a transition, K4 reports T4 warning.
   const k2=known&&channels[1].sound_on;
   const k3=known&&channels[2].sound_on;
   const k4=known&&channels[3].sound_on;
   const alarmOn=k4||channels?.[3]?.t4_condition===true;
   const triggered=k2||k3||alarmOn;
   const normal=known&&!triggered;
   const marks=[sound?.buzzer?.sensors?.[0]?.condition_active===true,
                sound?.buzzer?.sensors?.[1]?.condition_active===true,k3,alarmOn];
   for(let i=0;i<4;i++)home.querySelector('.approvedTemp.temp'+i)?.classList.toggle('busaAlert',marks[i]);
   heater.textContent=!known?'ТЭН —':normal?'ТЭН вкл.':'ТЭН откл.';
   valve.textContent=!known?'Кран —':normal?'Кран открыт':'Кран закрыт';
   for(const label of [heater,valve]){
     label.classList.toggle('busaStatusOff',known&&!normal);
     label.classList.toggle('busaStatusUnknown',!known);
   }
   alarm.hidden=!alarmOn;
   if(existingArt&&existingArt.getAttribute('src')!==originalArtSource)
     console.error('[DESIGN-003] Original local apparatus image unexpectedly changed');
 };
})();
