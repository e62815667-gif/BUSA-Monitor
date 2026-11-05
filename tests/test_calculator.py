"""Compare real C++ and browser math, reference density points and invalid inputs."""
from pathlib import Path
import subprocess,tempfile,json,random,math
root=Path(__file__).resolve().parents[1];sketch=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
subprocess.run(['python3','-B',str(root/'ci/generate_calculator.py'),'--check'],check=True)
rng=random.Random(1701);vectors=[]
for _ in range(100):
 a=rng.uniform(5,99);b=rng.uniform(1,a);v=rng.uniform(.01,100)
 vectors.append(['dilute',[v,a,b]])
 vectors.append(['correct',[rng.uniform(1,39),rng.uniform(20,85)]])
 vectors.append(['fraction',[v,a,rng.uniform(10,96),rng.uniform(0,20),rng.uniform(0,20)]])
vectors.extend([['dilute',[1,96,40]],['correct',[20,40]],['correct',[30,40]],['correct',[10,40]],['fraction',[10,40,80,10,10]],['dilute',[1,40,40]]])
script='''const c=require(process.argv[1]);const vectors=JSON.parse(process.argv[2]);
const assert=require('assert');assert(Math.abs(c.density(.31,13)-956.42)<.005);assert(Math.abs(c.density(.88,-10)-848.57)<.005);
assert(Math.abs(c.density(0,20)-998.20123)<1e-8);assert(Math.abs(c.density(1,20)-789.2391233)<.01);
assert.strictEqual(c.parse(' 35,7 '),35.7);assert.strictEqual(c.parse('.5'),.5);
for(const s of ['',' ','1,2,3','1abc','Infinity','-1','<script>','1e4'])assert.throws(()=>c.parse(s));
for(const a of [[0,40,20],[1,40,0],[1,20,40],[1,101,40],[NaN,40,20]])assert.throws(()=>c.dilute(...a));
for(const a of [[-1,40],[41,40],[20,101],[NaN,40]])assert.throws(()=>c.correct(...a));
for(const a of [[10,40,0,10,10],[10,40,80,60,40],[0,40,80,10,10],[10,40,80,-1,10]])assert.throws(()=>c.fraction(...a));
const d=c.dilute(1,96,40);assert(d.water>1.4&&d.water<1.5);assert(Math.abs(d.total-2.4)<1e-9);assert(1+d.water>d.total);
assert(c.correct(30,40).strength<40);assert(c.correct(10,40).strength>40);
const f=c.fraction(10,40,80,10,10);assert.deepStrictEqual(f,{absolute:4,heads:.4,tails:.4,product:4});
console.log(JSON.stringify(vectors.map(([fn,args])=>{try{return c[fn](...args);}catch(e){return null;}})));
'''
expected=json.loads(subprocess.check_output(['node','-e',script,str(root/'calculator.js'),json.dumps(vectors)],text=True))
code='''#include "BUSA_Calculator.h"
#include <cassert>
#include <cstdio>
int main(){
assert(!BusaCalc::dilute(0,40,20).ok);assert(!BusaCalc::dilute(1,40,0).ok);
assert(!BusaCalc::correct(41,40).ok);assert(!BusaCalc::correct(20,101).ok);
assert(!BusaCalc::fraction(10,40,0,10,10).ok);assert(!BusaCalc::fraction(10,40,80,60,40).ok);
'''
for (fn,args),result in zip(vectors,expected):
 call='BusaCalc::'+fn+'('+','.join(format(x,'.17g') for x in args)+')'
 code+='{auto r='+call+';'
 if result is None:code+='assert(!r.ok);'
 else:
  code+='assert(r.ok);'
  for key,value in result.items():code+=f'assert(std::fabs(r.{key}-({value:.17g}))<1e-7);'
 code+='}\n'
code+='puts("PASS calculator: 306 C++/JS scenarios, OIML reference points, contraction, AS balance, ranges and parsing");}\n'
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'calc.cpp';p.write_text(code);exe=Path(d)/'calc'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(sketch),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
# No controller writes/credentials/network calls in calculator code.
for name in ['BUSA_Calculator.h','BUSA_CalculatorUI.h','BUSA_CalculatorTft.h']:
 s=(sketch/name).read_text()
 for forbidden in ['digitalWrite(', 'analogWrite(', 'ledcWrite(', 'prefs.', 'WiFi.', 'HTTPClient','server.']:
  assert forbidden not in s,(name,forbidden)
s=(root/'calculator.js').read_text();assert 'fetch(' not in s and 'XMLHttpRequest' not in s
print('PASS calculator isolation: no GPIO/NVS/network writes')
