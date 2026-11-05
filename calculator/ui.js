// Isolated browser calculator: no controller API, telemetry or automatic sensor input.
if(typeof document!=='undefined')(()=>{
 const host=document.getElementById('busaCalculator');if(!host)return;
 const specs=[
 {name:'Разбавление водой',fields:[['Объём спирта, л','v'],['Начальная крепость, %','from'],['Требуемая крепость, %','to']],note:'Все объёмы и крепость — при 20 °C. Учтено сжатие смеси.',run:a=>BusaCalc.dilute(...a),results:[['Необходимо долить воды','water','л'],['Итоговый объём','total','л']]},
 {name:'Коррекция ареометра',fields:[['Температура спирта, °C','t'],['Показание ареометра, %','reading']],note:'Для водно-спиртового раствора без сахара, 0…40 °C. Стеклянный спиртомер, калибровка 20 °C; номинальное расширение стекла. Оценка, не поверка прибора.',run:a=>BusaCalc.correct(...a),results:[['Крепость при 20 °C','strength','% об.']]},
 {name:'Вторая перегонка',fields:[['Объём спирта-сырца, л','v'],['Крепость спирта-сырца, %','strength'],['Крепость на выходе, %','out'],['Доля голов от АС, %','head'],['Доля хвостов от АС, %','tail']],note:'Доли голов и хвостов — от абсолютного спирта (АС). Литры АС не равны фактическим объёмам фракций. Выход — расчётная оценка без дополнительных потерь.',run:a=>BusaCalc.fraction(...a),results:[['Абсолютный спирт','absolute','л АС'],['Головы','heads','л АС'],['Хвосты','tails','л АС'],['Выход продукта','product','л']]}
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
  const actions=document.createElement('div');actions.className='calcActions';
  const calculate=document.createElement('button');calculate.type='submit';calculate.textContent='Рассчитать';
  const reset=document.createElement('button');reset.type='reset';reset.textContent='Очистить';actions.append(calculate,reset);panel.append(actions);
  const error=document.createElement('p');error.className='calcError';error.setAttribute('role','alert');panel.append(error);
  const result=document.createElement('div');result.className='calcResult';result.setAttribute('role','status');result.setAttribute('aria-live','polite');panel.append(result);
  const note=document.createElement('p');note.className='calcNote';note.textContent=spec.note;panel.append(note);
  let saved={};try{saved=JSON.parse(localStorage.getItem('busa-calculator-v1-'+i)||'{}');}catch(e){}
  inputs.forEach(input=>{if(typeof saved[input.name]==='string')input.value=saved[input.name].slice(0,16);});
  function persist(){try{localStorage.setItem('busa-calculator-v1-'+i,JSON.stringify(Object.fromEntries(inputs.map(x=>[x.name,x.value]))));}catch(e){}}
  function update(explicit=false){
   error.textContent='';result.replaceChildren();inputs.forEach(x=>x.removeAttribute('aria-invalid'));
   if(!explicit&&inputs.some(x=>!x.value.trim()))return;
   try{
    const args=inputs.map(input=>{try{return BusaCalc.parse(input.value);}catch(e){input.setAttribute('aria-invalid','true');throw e;}});
    const values=spec.run(args);
    for(const [label,key,unit] of spec.results){const row=document.createElement('div');const text=document.createElement('span');text.textContent=label;const strong=document.createElement('strong');strong.textContent=fmt(values[key])+' '+unit;strong.dataset.value=key;row.append(text,strong);result.append(row);}
   }catch(e){error.textContent=e.message;}
  }
  panel.addEventListener('submit',e=>{e.preventDefault();persist();update(true);});
  panel.addEventListener('input',()=>{persist();update();});
  panel.addEventListener('reset',()=>{setTimeout(()=>{inputs.forEach(x=>{x.value='';x.removeAttribute('aria-invalid');});persist();error.textContent='';result.replaceChildren();},0);});
  update();
  button.addEventListener('click',()=>{panels.forEach((p,n)=>{p.hidden=n!==i;buttons[n].setAttribute('aria-selected',String(n===i));});});
 });
 panels.forEach((p,i)=>{p.hidden=i!==0;buttons[i].setAttribute('aria-selected',String(i===0));});
})();
