// Execute real local HOME presentation against minimal DOM fixtures.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
function node(){
  const classes=new Set(),attrs={src:'original-rig'};
  return {
    textContent:'',hidden:false,children:[],
    classList:{add(x){classes.add(x)},toggle(x,on){if(on)classes.add(x);else classes.delete(x)},contains(x){return classes.has(x)}},
    append(...v){this.children.push(...v)},replaceChildren(...v){this.children=v},
    getAttribute(k){return attrs[k]},setAttribute(k,v){attrs[k]=v},
  };
}
const dashboard=node(),metrics=node(),rig=node(),temps=Array.from({length:4},()=>node());
const elements=Object.fromEntries(['homeCoolingRate','homeWaterUsed','homeCollectionRate','homeCollectionTotal','homeCollectionUnit'].map(id=>[id,node()]));
const home=node();
home.querySelector=selector=>({
  '#rigArtStage':node(),
  '.approvedDashboard':dashboard,
  '.holoMetrics':metrics,
  '#rigArtStage .approvedRigArt':rig,
  '.approvedTemp.temp0':temps[0],
  '.approvedTemp.temp1':temps[1],
  '.approvedTemp.temp2':temps[2],
  '.approvedTemp.temp3':temps[3],
})[selector]||null;
const document={querySelector:()=>home,createElement:()=>node(),createTextNode:text=>({textContent:text}),getElementById:id=>elements[id]||null};
const window={};
vm.runInNewContext(fs.readFileSync('web/busa_design_003.js','utf8'),{document,window,console},{filename:'web/busa_design_003.js'});
const update=window.busaApprovedHomeUpdate;
assert.strictEqual(typeof update,'function');
assert.strictEqual(metrics.children.length,4);
const state=(active=[],topCondition=false,buzzer=[false,false])=>({
  sound_alerts:{
    channels:[0,1,2,3].map(i=>({sound_on:active.includes(i),t4_condition:i===3&&topCondition})),
    buzzer:{sensors:buzzer.map(condition_active=>({condition_active}))}
  }
});
const check=(s,expectedHeater,expectedValve,alarm)=>{
  update(s);
  const a=dashboard.children[0],heater=a.children[0],valve=a.children[1],red=dashboard.children[1];
  assert.strictEqual(heater.textContent,expectedHeater);
  assert.strictEqual(valve.textContent,expectedValve);
  assert.strictEqual(red.hidden,!alarm);
};
check(state(),'ТЭН вкл.','Кран открыт',false);
check(state([0]),'ТЭН вкл.','Кран открыт',false); // K1 = ordinary steady acoustic state
check(state([1]),'ТЭН откл.','Кран закрыт',false);
check(state([2]),'ТЭН откл.','Кран закрыт',false);
assert(temps[2].classList.contains('busaAlert'));
check(state([3]),'ТЭН откл.','Кран закрыт',true);
assert(temps[3].classList.contains('busaAlert'));
check(state([],true),'ТЭН откл.','Кран закрыт',true);
check(state([0]),'ТЭН вкл.','Кран открыт',false);
assert(!temps[3].classList.contains('busaAlert'));
update(state([],false,[true,true]));
assert(temps[0].classList.contains('busaAlert')&&temps[1].classList.contains('busaAlert'));
check(null,'ТЭН —','Кран —',false);
assert.strictEqual(rig.getAttribute('src'),'original-rig');
console.log('PASS local HOME statuses: quiet, K1 normal, K2/K3 transition, K4/T4 alarm, return, buzzer, disconnect, original image');
