/* Pure validation shared by browser and node tests. Never creates fake samples. */
(function(root){
 'use strict';
 const names=['Вода','Колонна','Куб','Верхняя точка колонны'];
 const finite=(v,lo,hi)=>typeof v==='number'&&Number.isFinite(v)&&v>=lo&&v<=hi;
 function telemetry(d){
  if(d?.device!=='BUSA-1M-001'||d.mode!=='MONITOR_ONLY'||d.outputs_enabled!==false||d.simulation===true||
   typeof d.bridge_online!=='boolean'||typeof d.keypad_online!=='boolean'||
   !Array.isArray(d.temperatures)||d.temperatures.length!==4||!Array.isArray(d.channels)||d.channels.length!==4||
   typeof d.key_stream_id!=='string')throw Error('Несовместимая телеметрия');
  d.channels.forEach((c,i)=>{
   if(typeof c?.valid!=='boolean')throw Error('Нет достоверности');
   if(c.valid&&(!d.bridge_online||!finite(d.temperatures[i],-55,125)||!finite(c.age_ms,0,3000)))throw Error('Недостоверная температура');
   if(!c.valid&&d.temperatures[i]!==null)throw Error('Ошибка канала должна быть null');
  });
  return d;
 }
 function capabilities(c){
  if(c?.device!=='BUSA-1M-001'||c.api_read_only!==true||c.physical_outputs!==false||c.temperature_channels!==4||
   !['4x4 PCF8575','5x4 PCF8575'].includes(c.keyboard))throw Error('Несовместимая прошивка');
  return c;
 }
 function settings(s){
  if(!Number.isInteger(s?.revision)||s.revision<0||s.physical_outputs!==false)throw Error('Неверная версия настроек');
  for(const k of ['warning','automation'])if(!Array.isArray(s[k])||s[k].length!==4||!s[k].every(v=>v===null||finite(v,-55,125)))throw Error('Пороги должны быть от −55 до 125 °C или пустыми');
  if(s.volume_l!==null&&!finite(s.volume_l,0,100))throw Error('Объём должен быть от 0 до 100 л');
  return s;
 }
 function parseInput(s,lo,hi){
  if(s.trim()==='')return null;
  const value=Number(s.trim().replace(',','.'));
  if(!finite(value,lo,hi))throw Error('Число вне диапазона '+lo+'…'+hi);
  return value;
 }
 const api={names,telemetry,capabilities,settings,parseInput};
 if(typeof module==='object')module.exports=api;else root.BusaCore=api;
})(typeof window==='undefined'?globalThis:window);

