// Headless runner: node tests/run_tests.js  (runs the protocol self-test, static checks and every flight test)
const path = require('path');
for (const f of ['util', 'protocol', 'hardware', 'firmware', 'link', 'physics', 'analysis', 'tests']) require(path.join(__dirname, '..', 'js', f + '.js'));
const S = globalThis.Sim;
const hw = S.Hardware.defaults(), fw = S.Firmware.defaults();
const only = process.argv.slice(2);
const icon = { pass: 'PASS', warn: 'WARN', fail: 'FAIL', info: 'INFO' };
let bad = 0;
console.log('== Protocol core (JS port vs C vector)');
for (const r of S.Protocol.selfTest()) { console.log(`  ${r.ok ? 'PASS' : 'FAIL'}  ${r.name}  ${r.detail || ''}`); if (!r.ok) bad++; }
if (!only.length) {
  const a = S.Analysis.analyze(hw, fw);
  for (const s of a.sections) { console.log('== ' + s.title); for (const i of s.items) console.log(`  ${icon[i.status]}  ${i.name}: ${i.value}${i.fix ? '   -> ' + i.fix : ''}`); }
  console.log('Static summary', a.summary);
}
(async () => {
  console.log('== Flight tests');
  const t0 = Date.now();
  const res = await S.Tests.runAll(hw, fw, null, only.length ? only : null);
  for (const r of res) {
    console.log(`  ${icon[r.status]}  ${r.name}: ${r.summary}`);
    for (const [k, v] of r.metrics) console.log(`        ${k}: ${v}`);
    for (const n of r.notes || []) console.log(`        note: ${n}`);
    if (r.status === 'fail') bad++;
  }
  console.log(`done in ${((Date.now() - t0) / 1000).toFixed(1)} s`);
  process.exitCode = bad ? 1 : 0;
})();
