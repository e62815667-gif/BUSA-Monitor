/* Coefficients adapted from alcoholometry-core (MIT), David Platt.
MIT License

Copyright (c) 2023 David Platt <david.platt@protonmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
const BUSA_CALC_COEFFICIENTS=[[1285.617841998974,-39032.85426],[7.442971530188783,-7420.201433430137,223446.0334],[-0.5411227621436812,-44.14153236817392,18523.73922069467,-547846.1354],[2.458043105903461,108.043594285623,-26055.62982188164,747017.2998],[-4.672147440794683,-142.2753946421155,22486.46550400788,-613838.1234],[4.810060584300675,109.635566657757,-12101.64659068747,306287.4042],[-2.895696483903638,-50.29988758547014,3924.090430035045,-88292.78388],[-1.51578483698721e-06,1.02299296671922,13.53034988843029,-704.7478054272792,13522.15441],[6.515031360099368e-06,-0.2002561813734156,-2.170575700536993,71.96353469546523,-1668.103923],[1.345612883493354e-08,-8.76305857347111e-06,0.01876837790289664,0.2517399633803461,-10.46914743455169,389.1238958],[-2.788074354782409e-08,4.075376675622027e-06,-0.0006802995733503803,-0.0119301300505701,0.1693433461530087,-192.9769495],[-9.9739231e-11,7.169354e-09,-3.8957702e-07,3.6130013e-05,-0.0052682542,-0.20618513,998.20123]];
// BUSA calculator arithmetic. All strengths are ABV at 20 °C; inputs are manual.
const BusaCalc=(()=>{
 const finite=(...v)=>v.every(Number.isFinite);
 function density(p,t){let r=0;for(const row of BUSA_CALC_COEFFICIENTS){let v=0;for(const c of row)v=v*(t-20)+c;r=r*p+v;}return r;}
 function abvFromMass(p){return 100*p*density(p,20)/density(1,20);}
 function massFromAbv(a){let lo=0,hi=1;for(let k=0;k<48;k++){const p=(lo+hi)/2;if(abvFromMass(p)<a)lo=p;else hi=p;}return (lo+hi)/2;}
 function dilute(v,from,to){
  if(!finite(v,from,to)||v<=0||v>100000||from<=0||from>100||to<=0||to>from)throw Error('Объём > 0; крепость 0 < требуемая ≤ начальная ≤ 100 %.');
  if(to===from)return {water:0,total:v};
  const p=massFromAbv(from),q=massFromAbv(to),mass=v*density(p,20),finalMass=mass*p/q;
  return {water:(finalMass-mass)/density(0,20),total:finalMass/density(q,20)};
 }
 function correct(t,reading){
  if(!finite(t,reading)||t<0||t>40||reading<0||reading>100)throw Error('Температура 0…40 °C; показание 0…100 %.');
  if(t===20)return {strength:reading};
  // Nominal glass alcohol hydrometer, volumetric expansion 25e-6 / °C.
  const target=density(massFromAbv(reading),20)/(1+0.000025*(t-20));
  if(target>density(0,t)+1e-8||target<density(1,t)-1e-8)throw Error('Показание вне диапазона модели при этой температуре.');
  let lo=0,hi=1;for(let k=0;k<48;k++){const p=(lo+hi)/2;if(density(p,t)>target)lo=p;else hi=p;}
  return {strength:abvFromMass((lo+hi)/2)};
 }
 function fraction(v,strength,out,head,tail){
  if(!finite(v,strength,out,head,tail)||v<=0||v>100000||strength<=0||strength>100||out<=0||out>100||head<0||tail<0||head+tail>=100)throw Error('Проверьте объём, крепость 0…100 % и сумму долей < 100 %.');
  const absolute=v*strength/100,heads=absolute*head/100,tails=absolute*tail/100;
  return {absolute,heads,tails,product:(absolute-heads-tails)*100/out};
 }
 function cylinder(diameter,height,level){
  if(!finite(diameter,height)||diameter<=0||height<=0)throw Error('Внутренний диаметр и высота должны быть больше нуля.');
  const area=Math.PI*diameter*diameter/4000;
  const capacity=area*height;
  if(!Number.isFinite(capacity)||capacity<=0)throw Error('Размеры вне диапазона расчёта.');
  if(level===undefined)return {capacity};
  if(!finite(level)||level<0||level>height)throw Error('Уровень жидкости должен быть от 0 до внутренней высоты.');
  const liquid=area*level;
  if(!Number.isFinite(liquid))throw Error('Размеры вне диапазона расчёта.');
  return {capacity,liquid};
 }
 function parse(text){const s=String(text).trim();if(!/^(?:\d+(?:[.,]\d*)?|[.,]\d+)$/.test(s))throw Error('Введите число; дробную часть можно вводить через запятую.');const n=Number(s.replace(',','.'));if(!Number.isFinite(n))throw Error('Недопустимое число.');return n;}
 return {density,abvFromMass,massFromAbv,dilute,correct,fraction,cylinder,parse};
})();
if(typeof module!=='undefined')module.exports=BusaCalc;

// Isolated browser calculator: no controller API, telemetry or automatic sensor input.
if(typeof document!=='undefined')(()=>{
 const host=document.getElementById('busaCalculator');if(!host)return;
 const specs=[
 {name:'Разбавление водой',fields:[['Объём спирта, л','v'],['Начальная крепость, %','from'],['Требуемая крепость, %','to']],note:'Все объёмы и крепость — при 20 °C. Учтено сжатие смеси.',run:a=>BusaCalc.dilute(...a),results:[['Необходимо долить воды','water','л'],['Итоговый объём','total','л']]},
 {name:'Коррекция ареометра',fields:[['Температура спирта, °C','t'],['Показание ареометра, %','reading']],note:'Для водно-спиртового раствора без сахара, 0…40 °C. Стеклянный спиртомер, калибровка 20 °C; номинальное расширение стекла. Оценка, не поверка прибора.',run:a=>BusaCalc.correct(...a),results:[['Крепость при 20 °C','strength','% об.']]},
 {name:'Вторая перегонка',fields:[['Объём спирта-сырца, л','v'],['Крепость спирта-сырца, %','strength'],['Крепость на выходе, %','out'],['Доля голов от АС, %','head'],['Доля хвостов от АС, %','tail']],note:'Доли голов и хвостов — от абсолютного спирта (АС). Литры АС не равны фактическим объёмам фракций. Выход — расчётная оценка без дополнительных потерь.',run:a=>BusaCalc.fraction(...a),results:[['Абсолютный спирт','absolute','л АС'],['Головы','heads','л АС'],['Хвосты','tails','л АС'],['Выход продукта','product','л']]},
 {name:'Объём куба',fields:[['Внутренний диаметр, см','diameter'],['Внутренняя высота, см','height'],['Уровень жидкости, см','level']],note:'Вертикальный цилиндр с плоским дном. Все размеры внутренние; уровень измеряется от дна. Диаметр и высота сохраняются до их изменения.',optionalLast:true,run:a=>BusaCalc.cylinder(...a),results:[['Полный объём','capacity','л'],['Объём жидкости','liquid','л']]}
 ];
 const title=document.createElement('h2');title.textContent='Калькулятор';host.append(title);
 const tabs=document.createElement('div');tabs.className='calcTabs';tabs.setAttribute('role','tablist');tabs.setAttribute('aria-label','Вид расчёта');host.append(tabs);
 const panels=[];const buttons=[];
 const fmt=n=>n.toLocaleString('ru-RU',{minimumFractionDigits:2,maximumFractionDigits:2});
 specs.forEach((spec,i)=>{
  const button=document.createElement('button');button.type='button';button.textContent=spec.name;button.id='calc-tab-'+i;button.setAttribute('role','tab');button.setAttribute('aria-controls','calc-panel-'+i);tabs.append(button);buttons.push(button);
  const panel=document.createElement('form');panel.id='calc-panel-'+i;panel.className='calcPanel';panel.setAttribute('role','tabpanel');panel.setAttribute('aria-labelledby',button.id);panel.noValidate=true;panels.push(panel);host.append(panel);
  const heading=document.createElement('h3');heading.textContent=spec.name;panel.append(heading);
  const grid=document.createElement('div');grid.className='calcGrid';panel.append(grid);
  const inputs=spec.fields.map(([label,key])=>{
   const wrap=document.createElement('label');wrap.textContent=label;
   const input=document.createElement('input');input.type='text';input.inputMode='decimal';input.name=key;input.id='calc-'+i+'-'+key;input.placeholder='0,00';input.autocomplete='off';input.maxLength=16;wrap.append(input);grid.append(wrap);return input;
  });
  // The two vessel dimensions are one-time settings. Keep the measured level visible.
  let dimensions=null,dimensionSummary=null;
  if(i===3){
   dimensions=document.createElement('details');dimensions.className='calcNote';
   dimensionSummary=document.createElement('summary');dimensions.append(dimensionSummary);
   const dimensionsGrid=document.createElement('div');dimensionsGrid.className='calcGrid';dimensions.append(dimensionsGrid);
   dimensionsGrid.append(inputs[0].parentElement,inputs[1].parentElement);
   panel.insertBefore(dimensions,grid);
  }
  const actions=document.createElement('div');actions.className='calcActions';
  const calculate=document.createElement('button');calculate.type='submit';calculate.textContent='Рассчитать';
  const reset=document.createElement('button');reset.type='reset';reset.textContent=i===3?'Очистить уровень':'Очистить';actions.append(calculate,reset);panel.append(actions);
  const error=document.createElement('p');error.className='calcError';error.setAttribute('role','alert');panel.append(error);
  const result=document.createElement('div');result.className='calcResult';result.setAttribute('role','status');result.setAttribute('aria-live','polite');panel.append(result);
  const note=document.createElement('details');note.className='calcNote';const summary=document.createElement('summary');summary.textContent='Условия расчёта';const noteText=document.createElement('p');noteText.textContent=spec.note;note.append(summary,noteText);panel.append(note);
  let saved={};try{saved=JSON.parse(localStorage.getItem('busa-calculator-v1-'+i)||'{}');}catch(e){}
  inputs.forEach(input=>{if(typeof saved[input.name]==='string')input.value=saved[input.name].slice(0,16);});
  function dimensionsValid(){
   if(i!==3)return false;
   try{return Number.isFinite(BusaCalc.cylinder(BusaCalc.parse(inputs[0].value),BusaCalc.parse(inputs[1].value)).capacity);}catch(e){return false;}
  }
  if(dimensions)dimensions.open=!dimensionsValid();
  function persist(){
   try{
    const key='busa-calculator-v1-'+i;
    if(i!==3){localStorage.setItem(key,JSON.stringify(Object.fromEntries(inputs.map(x=>[x.name,x.value]))));return;}
    let values={};try{values=JSON.parse(localStorage.getItem(key)||'{}')||{};}catch(e){}
    if(dimensionsValid()){
     values.diameter=inputs[0].value;values.height=inputs[1].value;
    }
    values.level=inputs[2].value;
    localStorage.setItem(key,JSON.stringify(values));
   }catch(e){}
  }
  function update(explicit=false){
   error.textContent='';result.replaceChildren();inputs.forEach(x=>x.removeAttribute('aria-invalid'));
   if(dimensions){
    dimensionSummary.textContent=dimensionsValid()?('Размеры куба: '+inputs[0].value+' × '+inputs[1].value+' см · изменить'):'Введите внутренние размеры куба';
   }
   if(!explicit&&inputs.some((x,n)=>!x.value.trim()&&(!spec.optionalLast||n!==inputs.length-1)))return;
   try{
    const args=inputs.map((input,n)=>{if(spec.optionalLast&&n===inputs.length-1&&!input.value.trim())return undefined;try{return BusaCalc.parse(input.value);}catch(e){input.setAttribute('aria-invalid','true');throw e;}});
    const values=spec.run(args);
    for(const [label,key,unit] of spec.results){if(values[key]===undefined)continue;const row=document.createElement('div');const text=document.createElement('span');text.textContent=label;const strong=document.createElement('strong');strong.textContent=fmt(values[key])+' '+unit;strong.dataset.value=key;row.append(text,strong);result.append(row);}
   }catch(e){error.textContent=e.message;}
  }
  panel.addEventListener('submit',e=>{e.preventDefault();persist();update(true);if(dimensions&&dimensionsValid())dimensions.open=false;});
  panel.addEventListener('input',()=>{persist();update();});
  panel.addEventListener('reset',e=>{
   if(dimensions){e.preventDefault();inputs[2].value='';persist();update();return;}
   setTimeout(()=>{inputs.forEach(x=>{x.value='';x.removeAttribute('aria-invalid');});persist();error.textContent='';result.replaceChildren();},0);
  });
  update();
  button.addEventListener('click',()=>{panels.forEach((p,n)=>{p.hidden=n!==i;buttons[n].setAttribute('aria-selected',String(n===i));});});
 });
 panels.forEach((p,i)=>{p.hidden=i!==0;buttons[i].setAttribute('aria-selected',String(i===0));});
})();
