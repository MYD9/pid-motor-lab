'use strict';
// Run with Node.js. Executes only the pure simulation core extracted from the offline page.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)];
assert.equal(scripts.length, 1);
const sandbox = {module: {exports: {}}};
vm.runInNewContext(scripts[0][1], sandbox, {timeout: 5000});
const {LabCore: C, MODEL_DEFAULTS} = sandbox.module.exports;
const initial = C.defaults();
let cases = 0;
function test(name, fn) {fn();cases++;console.log(`PASS ${name}`);}
test('page contains no network or hardware APIs', () => {
  assert.ok(!/\b(?:fetch|XMLHttpRequest|WebSocket|requestPort|SerialPort)\s*\(/.test(html));
  assert.ok(!/\bsrc\s*=\s*["']https?:/i.test(html));
  for (const p of Object.values(MODEL_DEFAULTS)) assert.ok(typeof p.provenance === 'string' && p.provenance.length > 5);
});
test('default teaching controller matches measured PI and runtime acceleration', () => {
  for(const ch of ['A','B']){
    assert.equal(initial[ch].kp,.03); assert.equal(initial[ch].ki,.08); assert.equal(initial[ch].kd,0);
    assert.equal(initial[ch].target,600);assert.equal(initial[ch].accel,1000);assert.equal(initial[ch].cap,30);
  }
});
test('target follows each selected acceleration limit through 1500 cps', () => {
  for(const accel of [10,100,1000,1500])for(const target of [-1500,1500]){
    const p={...initial.A,target,accel};const data=C.simulate(p,initial.options);
    for(let i=1;i<data.length;i++)assert.ok(Math.abs(data[i].target-data[i-1].target)<=accel*C.DT+1e-8);
    const expected=Math.sign(target)*Math.min(Math.abs(target),accel*(C.END-C.START+C.DT));
    assert.ok(Math.abs(data.at(-1).target-expected)<1e-6);
    assert.equal(C.metrics(data,p,initial.options).ready,C.START+Math.abs(target)/accel);
  }
});
test('zero command never energizes the simulated actuator', () => {
  const data=C.simulate({...initial.A,target:0},initial.options);
  assert.ok(data.every(r=>r.output===0&&r.actual===0&&r.i===0));
});
test('PWM is bounded and respects configured slew rate', () => {
  for (const target of [-1500,1500]) {
    const data=C.simulate({...initial.A,target,kp:1,ki:2,kd:.05,cap:17},initial.options);
    assert.ok(data.every(r=>Math.abs(r.output)<=17&&Math.abs(r.i)<=17));
    for(let i=1;i<data.length;i++)assert.ok(Math.abs(data[i].output-data[i-1].output)<=30*C.DT+1e-9);
    assert.ok(data.every(r=>target*r.output>=0));
  }
});
test('zero PWM cap gives a stationary plant', () => {
  assert.ok(C.simulate({...initial.A,cap:0},initial.options).every(r=>r.output===0&&r.actual===0));
});
test('large integral increments make progress through the slew envelope', () => {
  // Fixed slow plant isolates the anti-windup regression from later fitted model updates.
  const data=C.simulate({...initial.A,gain:35,deadzone:8,tau:.12,kp:0,ki:2,kd:0,target:100},{...initial.options,ramp:false});
  assert.ok(data[100].output>10,'partial integral acceptance must not freeze at startup');
  assert.ok(data[100].actual>0,'the simulated motor must cross its deadzone');
});
test('positive and negative plant responses are symmetric', () => {
  const positive=C.simulate(initial.A,initial.options);
  const negative=C.simulate({...initial.A,target:-initial.A.target},initial.options);
  positive.forEach((r,i)=>assert.ok(Math.abs(r.actual+negative[i].actual)<1e-8));
});
test('deterministic encoder quantization preserves reproducibility', () => {
  const first=C.simulate(initial.A,initial.options);
  const second=C.simulate(initial.A,initial.options);
  assert.equal(JSON.stringify(first),JSON.stringify(second));
  assert.ok(first.every(r=>Math.abs(r.raw/100-Math.round(r.raw/100))<1e-8));
});
test('PI reduces the lasting error compared with proportional-only control', () => {
  for(const ch of ['A','B']){
    const proportional={...initial[ch],kp:.03,ki:0,kd:0};
    const pi={...initial[ch],kp:.03,ki:.08,kd:0};
    const p=C.metrics(C.simulate(proportional,initial.options),proportional,initial.options);
    const i=C.metrics(C.simulate(pi,initial.options),pi,initial.options);
    assert.ok(Math.abs(i.error)<Math.abs(p.error)*.2,`${ch}: P ${p.error}, PI ${i.error}`);
    console.log(`${ch} model-only metrics: P error=${p.error.toFixed(2)}, PI error=${i.error.toFixed(2)}, PI overshoot=${i.overshoot.toFixed(2)}%, settling=${i.settled}`);
  }
});
test('all classroom presets remain finite with both model defaults', () => {
  for(const preset of C.presets)for(const ch of ['A','B']){
    const [kp,ki,kd]=preset.pid;
    const p={...initial[ch],kp,ki,kd,target:preset.target??600,cap:preset.cap??30};
    const options={...initial.options,...preset.opts};
    const data=C.simulate(p,options);
    assert.equal(data.length,1601);
    assert.ok(data.every(r=>Object.values(r).every(Number.isFinite)));
    assert.ok(data.every(r=>Math.abs(r.output)<=p.cap));
  }
});
test('step-down and load stimuli begin at their documented times', () => {
  const data=C.simulate(initial.A,{...initial.options,stepdown:true,load:true});
  assert.equal(data[49].request,0);assert.equal(data[50].request,600);
  assert.equal(data[799].request,600);assert.equal(data[800].request,300);
  const m=C.metrics(data,initial.A,{...initial.options,stepdown:true,load:true});
  assert.equal(m.end,7);
});
test('manual loading preserves history and unloading restores PI speed',()=>{
  const opt={...initial.options,loadEvents:[{t:5,value:4},{t:9,value:0}]};
  const data=C.simulate(initial.A,opt),plain=C.simulate(initial.A,initial.options);
  assert.deepEqual(data.slice(0,500),plain.slice(0,500));
  assert.equal(data[500].load,4);assert.equal(data[899].load,4);assert.equal(data[900].load,0);
  assert.ok(data[520].actual<plain[520].actual-20);
  assert.ok(Math.abs(data.at(-1).actual-initial.A.target)<15);
  assert.ok(data.every(r=>Number.isFinite(r.position)));
  assert.ok(data.at(-1).position>data[100].position);
  const back=C.simulate({...initial.A,target:-600},opt);
  assert.ok(back.at(-1).position<0);
});
const P=sandbox.module.exports.PracticeCore;
test('practice uses fixed conditions and does not mutate submitted state',()=>{
  const s=C.defaults();s.A.cap=0;s.A.target=0;s.A.gain=999;s.options.quantized=false;
  const before=JSON.stringify(s),r=P.evaluate(s);
  assert.equal(JSON.stringify(s),before);
  assert.equal(r.state.A.cap,30);assert.equal(r.state.A.target,600);
  assert.equal(r.state.A.gain,MODEL_DEFAULTS.A.gain);assert.equal(r.state.options.quantized,true);
  assert.equal(r.motors.A.data[499].load,0);assert.equal(r.motors.A.data[500].load,4);
  assert.equal(r.motors.A.data[1000].load,0);
});
test('practice rewards PI improvement and evaluates A and B independently',()=>{
  const good=C.defaults(),weak=C.defaults();for(const ch of ['A','B']){weak[ch].kp=.015;weak[ch].ki=0;}
  const a=P.evaluate(good),b=P.evaluate(weak);
  assert.ok(a.score>b.score+40);assert.ok(a.score>=75&&a.score<100);
  assert.equal(b.motors.A.values.recovery,null);
  good.B.ki=0;const changed=P.evaluate(good);
  assert.equal(a.motors.A.score,changed.motors.A.score);assert.ok(changed.motors.B.score<a.motors.B.score);
  assert.equal(JSON.stringify(a),JSON.stringify(P.evaluate(C.defaults())));
});
test('practice rejects invalid parameters and keeps extreme grades finite',()=>{
  for(const value of [NaN,Infinity,-1,2]){const s=C.defaults();s.A.kp=value;assert.throws(()=>P.evaluate(s));}
  for(const pid of [[0,0,0],[1,2,.05]]){const s=C.defaults();for(const ch of ['A','B'])[s[ch].kp,s[ch].ki,s[ch].kd]=pid;
    const r=P.evaluate(s);assert.ok(Number.isFinite(r.score)&&r.score>=0&&r.score<=100);
    if(pid[0]===0){assert.ok(r.score<60);assert.equal(r.motors.A.values.error,100);}
  }
});
console.log(`${cases} simulation tests passed. These do not validate a real motor or browser rendering.`);
