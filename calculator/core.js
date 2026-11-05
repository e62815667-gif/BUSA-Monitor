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
 function parse(text){const s=String(text).trim();if(!/^(?:\d+(?:[.,]\d*)?|[.,]\d+)$/.test(s))throw Error('Введите число; дробную часть можно вводить через запятую.');const n=Number(s.replace(',','.'));if(!Number.isFinite(n))throw Error('Недопустимое число.');return n;}
 return {density,abvFromMass,massFromAbv,dilute,correct,fraction,parse};
})();
if(typeof module!=='undefined')module.exports=BusaCalc;
