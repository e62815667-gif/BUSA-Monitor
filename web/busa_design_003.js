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
   const cube=sound?.channels?.[0],top=sound?.channels?.[3];
   const cubeValid=!!(cube?.sample_valid&&Number.isFinite(cube.on_c)&&Number.isFinite(cube.off_c));
   const topValid=!!(top?.sample_valid&&Number.isFinite(top.on_c)&&Number.isFinite(top.off_c));
   const cubeOn=cubeValid&&cube.t3_condition===true;
   const topOn=topValid&&top.t4_condition===true;
   const known=cubeValid&&topValid;
   const normal=known&&cubeOn&&!topOn;
   const ordinaryOff=known&&!cubeOn&&!topOn&&Number(sound?.t3_off_events)>0;
   const marks=[sound?.buzzer?.sensors?.[0]?.condition_active===true,
                sound?.buzzer?.sensors?.[1]?.condition_active===true,ordinaryOff,topOn];
   for(let i=0;i<4;i++)home.querySelector('.approvedTemp.temp'+i)?.classList.toggle('busaAlert',marks[i]);
   heater.textContent=!known?'ТЭН —':normal?'ТЭН вкл.':'ТЭН откл.';
   valve.textContent=!known?'Кран —':normal?'Кран открыт':'Кран закрыт';
   for(const label of [heater,valve]){
     label.classList.toggle('busaStatusOff',known&&!normal);
     label.classList.toggle('busaStatusUnknown',!known);
   }
   alarm.hidden=!topOn;
   if(existingArt&&existingArt.getAttribute('src')!==originalArtSource)
     console.error('[DESIGN-003] Original local apparatus image unexpectedly changed');
 };
})();
