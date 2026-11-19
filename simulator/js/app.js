// Browser app: wires the simulator modules to the page.
(function (S) {
  'use strict';
  const U = S.U, $ = (id) => document.getElementById(id);
  const STORE = 'bicopterSim.v1';

  // ------------------------------------------------------------------ state
  const app = {
    hw: null, fw: null, world: null, running: false, speed: 1, pending: false,
    analysis: null, testResults: [], replay: null,
    sticks: { thr: 0, yaw: 0, pitch: 0, roll: 0 }, arm: false, angle: true,
    keys: new Set(), gpArmPrev: false, lastFrame: performance.now(),
  };

  function load() {
    try {
      const raw = localStorage.getItem(STORE);
      if (raw) {
        const o = JSON.parse(raw);
        app.hw = mergeDefaults(S.Hardware.defaults(), o.hw);
        app.fw = mergeDefaults(S.Firmware.defaults(), o.fw);
        return;
      }
    } catch (e) { /* storage unavailable or corrupt */ }
    app.hw = S.Hardware.defaults(); app.fw = S.Firmware.defaults();
  }
  function save() { try { localStorage.setItem(STORE, JSON.stringify({ hw: app.hw, fw: app.fw })); } catch (e) { /* ignore */ } }
  function mergeDefaults(d, o) {
    if (!o || typeof o !== 'object') return d;
    if (Array.isArray(d)) return Array.isArray(o) ? o : d;
    const out = Object.assign({}, d);
    for (const k in o) out[k] = (d[k] && typeof d[k] === 'object' && !Array.isArray(d[k])) ? mergeDefaults(d[k], o[k]) : o[k];
    return out;
  }

  // ------------------------------------------------------------------ theme from CSS tokens
  let theme = {};
  function readTheme() {
    const cs = getComputedStyle(document.documentElement), v = (n) => cs.getPropertyValue(n).trim();
    theme = { bg: v('--card'), text: v('--text'), muted: v('--muted'), sky1: v('--sky1'), sky2: v('--sky2'), grid: v('--grid'), gridMajor: v('--grid-major'),
      frame: v('--frame'), arm: v('--arm'), motor: v('--motor'), battery: v('--battery'), elec: v('--elec'), servo: v('--servo'), edge: v('--edge'),
      thrust: v('--thrust'), cg: v('--cg'), propRgb: v('--prop'), blade: v('--blade'), trail: v('--trail'), shadow: v('--shadow'), pilot: v('--pilot'), nose: v('--nose'),
      chartGrid: v('--chart-grid'), chartZero: v('--chart-zero'), c: ['--c1', '--c2', '--c3', '--c4', '--c5', '--c6'].map(v), line: v('--line'), accent: v('--accent'), fail: v('--fail'), warn: v('--warn'), pass: v('--pass') };
  }

  // ------------------------------------------------------------------ world
  function newWorld() {
    app.world = new S.Physics.World(U.deepCopy(app.hw), U.deepCopy(app.fw), { seed: (Math.random() * 1e9) | 0, noise: true, record: true });
    app.pilot = new S.Tests.Pilot();
    app.world.scenario = { update: applyInputs };
    app.altTarget = 0;
    setPending(false);
    renderLog(true);
  }
  function setPending(v) { app.pending = v; $('pending').hidden = !v; }
  function changed() { save(); setPending(true); scheduleAnalysis(); }

  function applyInputs(w) {
    const i = w.handset.inputs, s = app.sticks;
    i.aux[0] = app.arm ? 1900 : 1000;
    i.aux[1] = app.angle ? 1900 : 1000;
    const altAssist = $('assistAlt').checked, posAssist = $('assistPos').checked;
    if (altAssist || posAssist) {
      const p = app.pilot;
      if (!p._init) { p._init = true; p.alt = Math.max(0.2, w.altitude()); const c = w.cgWorld(); p.x = c[0]; p.y = c[1]; p.yaw = U.qToEuler(w.q)[2]; p.iAlt = 0; const hov = S.Analysis.hoverPoint(w.hw, w.fw); const P = w.fw.params; const mu = w.hw.esc.calLow + hov.duty * (w.hw.esc.calHigh - w.hw.esc.calLow); const lo = 1000 + P.motor_idle / 10, hi = 1000 + 10 * P.motor_output_limit; p.thrBase = P.min_check + U.clamp((mu - lo) / (hi - lo), 0, 1) * (2000 - P.min_check); }
      p.active = true; p.override = {};
      if (altAssist) { p.alt = U.clamp(p.alt + (s.thr - 0.5) * 2 * 2.0 * w.dt, 0, 60); if (s.thr < 0.05 && w.altitude() < 0.25) p.override.throttle = 1000; }
      else p.override.throttle = 1000 + 1000 * s.thr;
      const sticksIdle = Math.abs(s.pitch) < 0.03 && Math.abs(s.roll) < 0.03;
      p.hold = posAssist && sticksIdle;
      if (!p.hold) { p.override.pitch = s.pitch; p.override.roll = s.roll; const c = w.cgWorld(); p.x = c[0]; p.y = c[1]; }
      if (Math.abs(s.yaw) > 0.03 || !posAssist) { p.override.yaw = s.yaw; p.yaw = U.qToEuler(w.q)[2]; }
      p.yawHold = posAssist;
      if (!app.arm) p.iAlt = 0;
      p.apply(w, w.dt);
    } else {
      app.pilot._init = false;
      i.throttle = 1000 + 1000 * s.thr; i.yaw = 1500 + 500 * s.yaw; i.pitch = 1500 + 500 * s.pitch; i.roll = 1500 + 500 * s.roll;
    }
  }

  // ------------------------------------------------------------------ sticks
  function makeStick(canvas, axes) {
    const st = { x: 0, y: 0, drag: false };
    const toVal = (e) => { const r = canvas.getBoundingClientRect(); return [U.clamp(((e.clientX - r.left) / r.width) * 2 - 1, -1, 1), U.clamp(1 - ((e.clientY - r.top) / r.height) * 2, -1, 1)]; };
    canvas.addEventListener('pointerdown', (e) => { st.drag = true; canvas.setPointerCapture(e.pointerId); [st.x, st.y] = toVal(e); push(); });
    canvas.addEventListener('pointermove', (e) => { if (!st.drag) return; [st.x, st.y] = toVal(e); push(); });
    canvas.addEventListener('pointerup', () => { st.drag = false; if (axes.springX) st.x = 0; if (axes.springY) st.y = 0; push(); });
    function push() { axes.set(st.x, st.y); }
    st.draw = (x, y) => {
      const d = window.devicePixelRatio || 1, r = canvas.getBoundingClientRect();
      if (canvas.width !== Math.round(r.width * d)) { canvas.width = Math.round(r.width * d); canvas.height = Math.round(r.height * d); }
      const g = canvas.getContext('2d'), W = canvas.width, H = canvas.height;
      g.clearRect(0, 0, W, H);
      g.strokeStyle = theme.line; g.lineWidth = d; g.strokeRect(1, 1, W - 2, H - 2);
      g.beginPath(); g.moveTo(W / 2, 4); g.lineTo(W / 2, H - 4); g.moveTo(4, H / 2); g.lineTo(W - 4, H / 2); g.stroke();
      const px = (x + 1) / 2 * W, py = (1 - y) / 2 * H;
      g.fillStyle = theme.accent; g.beginPath(); g.arc(px, py, 9 * d, 0, 7); g.fill();
    };
    return st;
  }

  function updateKeysAndGamepad(dt) {
    const k = app.keys, s = app.sticks;
    if (k.has('KeyW')) s.thr = U.clamp(s.thr + 0.45 * dt, 0, 1);
    if (k.has('KeyS')) s.thr = U.clamp(s.thr - 0.6 * dt, 0, 1);
    const keyAxis = (neg, pos, mag) => (k.has(pos) ? mag : 0) - (k.has(neg) ? mag : 0);
    if (k.has('KeyA') || k.has('KeyD')) s.yaw = keyAxis('KeyA', 'KeyD', 0.5); else if (app._kbYaw) s.yaw = 0;
    app._kbYaw = k.has('KeyA') || k.has('KeyD');
    if (k.has('ArrowUp') || k.has('ArrowDown')) s.pitch = keyAxis('ArrowDown', 'ArrowUp', 0.35); else if (app._kbPitch) s.pitch = 0;
    app._kbPitch = k.has('ArrowUp') || k.has('ArrowDown');
    if (k.has('ArrowLeft') || k.has('ArrowRight')) s.roll = keyAxis('ArrowLeft', 'ArrowRight', 0.35); else if (app._kbRoll) s.roll = 0;
    app._kbRoll = k.has('ArrowLeft') || k.has('ArrowRight');
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = gps && Array.from(gps).find((g) => g && g.connected);
    if (gp) {
      $('gamepadInfo').textContent = 'Gamepad: ' + gp.id.slice(0, 28);
      const a = gp.axes, dz = (v) => (Math.abs(v) < 0.06 ? 0 : v);
      if (a.length >= 4) { s.yaw = dz(a[0]); s.thr = U.clamp((1 - a[1]) / 2, 0, 1); s.roll = dz(a[2]); s.pitch = -dz(a[3]); }
      const b = gp.buttons[0] && gp.buttons[0].pressed;
      if (b && !app.gpArmPrev) setArm(!app.arm);
      app.gpArmPrev = b;
    }
  }
  function setArm(v) { app.arm = v; $('swArm').classList.toggle('on', v); }
  function setAngle(v) { app.angle = v; const b = $('swMode'); b.classList.toggle('on', v); b.firstChild.textContent = v ? 'ANGLE ' : 'ACRO '; }

  // ------------------------------------------------------------------ main loop
  function frame(now) {
    const dtReal = Math.min(0.25, (now - app.lastFrame) / 1000); app.lastFrame = now;
    updateKeysAndGamepad(dtReal);
    const w = app.world;
    if (app.replay) stepReplay(dtReal);
    else if (app.running) {
      const n = Math.min(1000, Math.round(dtReal * app.speed / w.dt));
      for (let i = 0; i < n; i++) w.step();
    }
    const target = app.replay ? app.replay.view : w;
    app.view.showVectors = $('optVectors').checked; app.view.follow = $('optFollow').checked; app.view.showCG = $('optCG').checked;
    app.view.draw(target, theme);
    app.stickL.draw(app.sticks.yaw, app.sticks.thr * 2 - 1); app.stickR.draw(app.sticks.roll, app.sticks.pitch);
    if (!app._slow || now - app._slow > 100) { app._slow = now; updateHud(); updateNumbers(); drawCharts(); renderLog(); }
    requestAnimationFrame(frame);
  }

  // ------------------------------------------------------------------ HUD / chips / numbers
  function updateHud() {
    const w = app.replay ? null : app.world;
    if (app.replay) { const r = app.replay.cur; if (!r) return;
      $('chips').innerHTML = [['Replay', 'info'], [r.armed ? 'ARMED' : 'disarmed', r.armed ? 'fail' : 'info'], [`LQ ${r.lq.toFixed(0)} %`, r.lq > 80 ? 'pass' : 'warn'], [`${r.vbat.toFixed(2)} V`, 'info'], [`${r.Ibatt.toFixed(1)} A`, 'info']].concat(r.fs !== 'IDLE' ? [['Failsafe ' + r.fs, 'fail']] : []).map(([t, c]) => `<span class="chip ${c}">${esc(t)}</span>`).join('');
      $('hud').innerHTML = `t ${r.t.toFixed(2)} s · REPLAY<br>alt ${r.alt.toFixed(2)} m · roll ${r.roll.toFixed(1)}° pitch ${r.pitch.toFixed(1)}°<br>tilt ${r.tilt[0].toFixed(1)}° / ${r.tilt[1].toFixed(1)}° · ${r.armed ? 'ARMED' : 'disarmed'} · FS ${r.fs}`; return; }
    const e = U.qToEuler(w.q).map((x) => x * U.RAD), fc = w.fc;
    $('hud').innerHTML = `t ${w.t.toFixed(2)} s · ${fc.armed ? '<b>ARMED</b>' : 'disarmed'} · ${app.angle ? 'ANGLE' : 'ACRO'}<br>` +
      `alt ${w.altitude().toFixed(2)} m · v ${U.norm(w.vel).toFixed(1)} m/s<br>roll ${e[0].toFixed(1)}° pitch ${e[1].toFixed(1)}° yaw ${e[2].toFixed(0)}°<br>` +
      `thr ${(app.sticks.thr * 100).toFixed(0)} % · tilt ${(w.tilt[0] * U.RAD).toFixed(1)}° / ${(w.tilt[1] * U.RAD).toFixed(1)}°` + (w.crashed ? `<br><b style="color:var(--fail)">CRASHED: ${w.crashReason}</b>` : '');
    const chips = [];
    const blocked = [...fc.flags];
    chips.push(fc.armed ? ['ARMED', 'fail'] : [blocked.length ? 'Blocked: ' + blocked.join(', ') : 'Ready to arm', blocked.length ? 'warn' : 'pass']);
    if (fc.fs.phase !== 'IDLE') chips.push(['Failsafe ' + fc.fs.phase, 'fail']);
    const lq = w.receiver.linkQuality(Math.round(w.t * 1e6));
    chips.push([`LQ ${lq.toFixed(0)} %`, lq > 80 ? 'pass' : lq > 40 ? 'warn' : 'fail']);
    const cell = w.Vbatt / w.hw.battery.cells;
    chips.push([`${w.Vbatt.toFixed(2)} V (${cell.toFixed(2)}/cell)`, cell > 3.6 ? 'pass' : cell > 3.4 ? 'warn' : 'fail']);
    chips.push([`${w.Ibatt.toFixed(1)} A`, Math.max(...w.motorIavg) > w.hw.motor.maxCurrent ? 'warn' : 'info']);
    if (!w.mcuOn) chips.push(['Pico OFF', 'fail']);
    if (w.crashed) chips.push(['CRASH', 'fail']);
    $('chips').innerHTML = chips.map(([t, c]) => `<span class="chip ${c}">${esc(t)}</span>`).join('');
  }

  function updateNumbers() {
    if (app.replay) return;
    const w = app.world, fc = w.fc, h = w.hw;
    const fosArm = w.armMat.yield / Math.max(1, Math.max(w.armVm0 || 0, w.armVm1 || 0));
    const stall = h.servo.stallTorque48;
    const n = [
      ['Altitude', `${w.altitude().toFixed(2)} m`], ['Speed', `${U.norm(w.vel).toFixed(2)} m/s`],
      ['Motor L / R', `${(w.rotor[0] * 9.549).toFixed(0)} / ${(w.rotor[1] * 9.549).toFixed(0)} rpm`],
      ['Thrust L / R', `${(w.thrust[0] / U.G * 1000).toFixed(0)} / ${(w.thrust[1] / U.G * 1000).toFixed(0)} g`],
      ['Motor current', `${w.motorI[0].toFixed(1)} / ${w.motorI[1].toFixed(1)} A`, Math.max(...w.motorIavg) > h.motor.maxCurrent ? 'warn' : ''],
      ['Battery', `${w.Vbatt.toFixed(2)} V · ${w.Ibatt.toFixed(1)} A`], ['Used', `${w.mAh.toFixed(0)} mAh (${(w.soc * 100).toFixed(0)} %)`],
      ['BF sensed vbat', `${(fc.vbatFilt || 0).toFixed(2)} V ${fc.batWarn}`, fc.batWarn ? 'warn' : ''],
      ['Pico 5 V', `${w.rails.v5.toFixed(2)} V · ${(w.rails.i5 * 1000).toFixed(0)} mA`, w.rails.v5 < 4.5 ? 'fail' : ''],
      ['Servo rail', `${w.rails.vServo.toFixed(2)} V · ${w.rails.iServo.toFixed(2)} A`, w.rails.vServo < 4.5 ? 'warn' : ''],
      ['Radio 3V3', `${(w.rails.v3radio - w.rails.radioDip).toFixed(2)} V min`],
      ['Tilt L / R', `${(w.tilt[0] * U.RAD).toFixed(1)}° / ${(w.tilt[1] * U.RAD).toFixed(1)}°`],
      ['Servo load', `${(w.servoLoad[0] * 100 / U.G).toFixed(2)} / ${(w.servoLoad[1] * 100 / U.G).toFixed(2)} kg·cm`, Math.max(Math.abs(w.servoLoad[0]), Math.abs(w.servoLoad[1])) > stall ? 'fail' : ''],
      ['Arm stress', `${(Math.max(w.armVm0 || 0, w.armVm1 || 0) / 1e6).toFixed(1)} MPa (FoS ${fosArm > 99 ? '>99' : fosArm.toFixed(1)})`, fosArm < 2 ? 'warn' : ''],
      ['Temps M / E', `${Math.max(...w.temp.motor).toFixed(0)} / ${Math.max(...w.temp.esc).toFixed(0)} °C`, Math.max(...w.temp.motor) > h.motor.tmax - 15 ? 'warn' : ''],
      ['Link', `${w.receiver.linkQuality(Math.round(w.t * 1e6)).toFixed(0)} % · ${w.radio.last.prx.toFixed(0)} dBm`],
      ['RX accepted', `${w.receiver.stats.accepted} (${w.receiver.hop.state ? 'synced' : 'search'})`],
      ['PID sum R/P/Y', fc.pid.map((p) => p.sum.toFixed(0)).join(' / ')],
      ['Motor / servo out', `${w.lastOut.motorUs.map((x) => x.toFixed(0)).join('/')} · ${w.lastOut.servoUs.map((x) => x.toFixed(0)).join('/')}`],
    ];
    $('numgrid').innerHTML = n.map(([k, v, c]) => `<div class="num ${c || ''}"><div class="k">${k}</div><div class="v">${esc(v)}</div></div>`).join('');
  }

  // ------------------------------------------------------------------ charts
  const CHARTS = [
    { title: 'Attitude', unit: '°', series: [['roll', (r) => r.roll, 0], ['pitch', (r) => r.pitch, 1]] },
    { title: 'Rates vs setpoint', unit: '°/s', series: [['roll', (r) => r.rate[0], 0], ['sp', (r) => r.sp[0], 0, [4, 3]], ['pitch', (r) => r.rate[1], 1], ['sp', (r) => r.sp[1], 1, [4, 3]], ['yaw', (r) => r.rate[2], 2], ['sp', (r) => r.sp[2], 2, [4, 3]]] },
    { title: 'Outputs', unit: 'us', series: [['motor L', (r) => r.motorUs[0], 0], ['motor R', (r) => r.motorUs[1], 1], ['servo L', (r) => r.servoUs[0], 2], ['servo R', (r) => r.servoUs[1], 3]] },
    { title: 'Nacelle tilt', unit: '°', series: [['left', (r) => r.tilt[0], 0], ['right', (r) => r.tilt[1], 1]] },
    { title: 'Servo load', unit: 'kg·cm', series: [['left', (r) => r.servoLoad[0] * 100 / U.G, 0], ['right', (r) => r.servoLoad[1] * 100 / U.G, 1]] },
    { title: 'Current', unit: 'A', series: [['battery', (r) => r.Ibatt, 4], ['motor L', (r) => r.motorI[0], 0], ['motor R', (r) => r.motorI[1], 1]] },
    { title: 'Voltage', unit: 'V', series: [['battery', (r) => r.vbat, 4], ['Pico 5V', (r) => r.v5, 2], ['servo', (r) => r.vServo, 3]] },
    { title: 'Altitude & throttle', unit: 'm / %', series: [['alt', (r) => r.alt, 0], ['thr/10', (r) => r.throttle * 10, 1]] },
    { title: 'Link', unit: '% / dBm', series: [['LQ', (r) => r.lq, 2], ['Prx', (r) => r.prx, 5]] },
    { title: 'Temperature', unit: '°C', series: [['motor L', (r) => r.tempMotor[0], 0], ['motor R', (r) => r.tempMotor[1], 1], ['ESC L', (r) => r.tempEsc[0], 2], ['ESC R', (r) => r.tempEsc[1], 3]] },
  ];
  function buildCharts() {
    const host = $('charts'); host.innerHTML = '';
    app.charts = CHARTS.map((c) => {
      const cv = document.createElement('canvas'); host.appendChild(cv);
      return new S.Chart(cv, { title: c.title, unit: c.unit, window: 10, series: c.series.map(([label, key, ci, dash]) => ({ label, key, color: theme.c[ci], dash })) });
    });
  }
  function drawCharts() {
    if (!$('tab-telemetry').classList.contains('active')) return;
    const rec = app.replay ? app.replay.rec : app.world.record;
    const tEnd = app.replay ? (app.replay.cur ? app.replay.cur.t : 0) : app.world.t;
    for (const c of app.charts) c.o.series.forEach((s, i) => { s.color = theme.c[CHARTS[app.charts.indexOf(c)].series[i][2]]; });
    for (const c of app.charts) c.draw(rec, Math.max(tEnd, 10), theme, Math.max(0, Math.max(tEnd, 10) - 10));
  }

  let logCount = -1;
  function renderLog(force) {
    const ev = app.replay ? app.replay.events : app.world.events;
    if (!force && ev.length === logCount) return;
    logCount = ev.length;
    $('log').innerHTML = ev.slice(-150).reverse().map((e) => `<div class="${e.level}">${e.t.toFixed(3)} s  ${esc(e.msg)}</div>`).join('') || '<span class="muted">No events yet. Press Run, then arm (Space or the ARM switch).</span>';
  }

  // ------------------------------------------------------------------ firmware tab
  function cliPrint(text) { const o = $('cliOut'); o.textContent += text + '\n'; o.scrollTop = o.scrollHeight; }
  function runCli(text) {
    const out = S.Firmware.cli(app.fw, text);
    cliPrint(out);
    buildParamForm(); syncCustom();
    if (/^\s*(save|defaults)\b/im.test(text)) { save(); newWorld(); cliPrint('# rebooting (simulator reset)'); }
    else changed();
  }
  function buildParamForm() {
    const P = S.Firmware.PARAMS, groups = {}, d = S.Firmware.defaults().params;
    for (const k in P) (groups[P[k][3]] = groups[P[k][3]] || []).push(k);
    const open = new Set([...document.querySelectorAll('#paramForm details[open]')].map((x) => x.dataset.g));
    $('paramForm').innerHTML = Object.entries(groups).map(([g, keys]) => `<details class="group" data-g="${g}" ${open.has(g) ? 'open' : ''}><summary>${g}</summary><div class="fields">` + keys.map((k) => {
      const m = P[k], v = app.fw.params[k], ch = v !== d[k] ? ' changed' : '';
      const ctl = m[5] ? `<select data-p="${k}">${m[5].map((o) => `<option ${o === v ? 'selected' : ''}>${o}</option>`).join('')}</select>`
        : typeof m[0] === 'string' ? `<input type="text" data-p="${k}" value="${esc(String(v))}">`
        : `<input type="number" data-p="${k}" value="${v}" min="${m[1]}" max="${m[2]}">`;
      return `<label class="field${ch}" title="${esc(m[4])}"><span>${k}</span>${ctl}</label>`;
    }).join('') + '</div></details>').join('');
    $('paramForm').querySelectorAll('[data-p]').forEach((el) => el.addEventListener('change', () => {
      const r = S.Firmware.setParam(app.fw, el.dataset.p, el.value);
      cliPrint(`# set ${el.dataset.p} = ${el.value}\n${r}`);
      buildParamForm(); changed();
    }));
  }
  function syncCustom() { $('customCode').value = app.fw.customCode; $('customEnabled').checked = !!app.fw.customEnabled; checkCustom(); }
  function checkCustom() {
    try { new Function('io', 'Math', $('customCode').value); $('customStatus').textContent = app.fw.customEnabled ? 'Compiles. Active after Apply & reboot.' : 'Compiles (disabled).'; $('customStatus').style.color = ''; }
    catch (e) { $('customStatus').textContent = 'Syntax error: ' + e.message; $('customStatus').style.color = 'var(--fail)'; }
  }

  const PROJECT_CLI = `# HANDOFF.md section 11
set rx_spi_protocol = NRF24_BICOPTER
set motor_pwm_protocol = PWM
set motor_pwm_rate = 200
set failsafe_procedure = DROP
set failsafe_delay = 5
set debug_mode = RX_NRF24_BICOPTER
aux 0 0 0 1700 2100 0 0
# simulator addition: ANGLE mode on AUX2 (not in the project CLI)
aux 1 1 1 1300 2100 0 0`;
  const TUNE_CLI = `# Suggested bicopter starting tune (found with this simulator; verify on hardware)
set yaw_srate = 25
set p_yaw = 12
set i_yaw = 30
set f_yaw = 15
set roll_srate = 30
set pitch_srate = 30
set angle_limit = 30
aux 1 1 1 1300 2100 0 0`;

  // ------------------------------------------------------------------ hardware tab
  const O = (pairs) => pairs.map(([v, l]) => ({ v, l }));
  const HW_FIELDS = [
    ['Frame & structure', [['frame.armLength', 'Arm length to tilt axis', 'm'], ['frame.hubHalfWidth', 'Hub half width', 'm'],
      ['frame.armMaterial', 'Arm material', O(Object.entries(S.Hardware.MATERIALS).map(([k, m]) => [k, m.name]))],
      ['frame.armSection.type', 'Arm section', O([['round_tube', 'Round tube'], ['round_solid', 'Round solid'], ['square_tube', 'Square tube'], ['rect', 'Solid rectangle']])],
      ['frame.armSection.od', 'Tube OD / side', 'm'], ['frame.armSection.wall', 'Wall', 'm'], ['frame.armSection.width', 'Rect width', 'm'], ['frame.armSection.height', 'Rect height', 'm'],
      ['frame.propAboveTilt', 'Prop plane above tilt axis', 'm'], ['frame.motorAxisOffsetX', 'Motor axis ahead of tilt axis', 'm'], ['frame.nacelleCgAboveTilt', 'Nacelle CG above tilt axis', 'm'],
      ['frame.tiltLimitDeg', 'Nacelle mechanical stop', '°'], ['frame.gearHeight', 'Gear below tilt axis', 'm'], ['frame.gearSpanX', 'Gear span fore-aft', 'm'], ['frame.gearSpanY', 'Gear span left-right', 'm'],
      ['frame.gearStroke', 'Gear compliance (drop)', 'm'], ['frame.bodyDragArea', 'Body drag Cd·A', 'm²'], ['frame.dropHeight', 'Hard-landing drop height', 'm'], ['frame.maneuverLoadFactor', 'Manoeuvre load factor', 'g'], ['frame.targetSafetyFactor', 'Target safety factor', '']]],
    ['Motors', [['motor.name', 'Model', 'text'], ['motor.kv', 'Kv', 'rpm/V'], ['motor.rm', 'Winding resistance', 'Ω'], ['motor.i0', 'No-load current', 'A'], ['motor.maxCurrent', 'Max current', 'A'], ['motor.maxCurrentTime', 'for', 's'],
      ['motor.rotorInertia', 'Rotor inertia', 'kg·m²'], ['motor.cth', 'Thermal mass', 'J/K'], ['motor.rth', 'Thermal resistance', 'K/W'], ['motor.tmax', 'Max temperature', '°C'],
      ['motor.spinL', 'Left prop rotation', O([[-1, 'CW from above'], [1, 'CCW from above']])], ['motor.spinR', 'Right prop rotation', O([[-1, 'CW from above'], [1, 'CCW from above']])]]],
    ['Propellers', [['prop.name', 'Model', 'text'], ['prop.diameter', 'Diameter', 'm'], ['prop.pitch', 'Pitch', 'm'], ['prop.ct', 'Thrust coefficient Ct', ''], ['prop.cp', 'Power coefficient Cp', ''], ['prop.mass', 'Mass', 'kg']]],
    ['ESCs', [['esc.name', 'Model', 'text'], ['esc.contCurrent', 'Continuous', 'A'], ['esc.burstCurrent', 'Burst', 'A'], ['esc.rOn', 'FET path resistance', 'Ω'], ['esc.calLow', 'Calibrated stop', 'us'], ['esc.calHigh', 'Calibrated full', 'us'],
      ['esc.startDuty', 'Start threshold (duty)', ''], ['esc.maxRateHz', 'Max input rate', 'Hz'], ['esc.dampedLight', 'Damped light (active braking)', 'bool'], ['esc.tmax', 'Max temperature', '°C']]],
    ['Battery', [['battery.name', 'Model', 'text'], ['battery.cells', 'Cells (S)', ''], ['battery.capacity', 'Capacity', 'Ah'], ['battery.cRating', 'C rating', 'C'], ['battery.rCell', 'Resistance per cell', 'Ω'], ['battery.soc0', 'Start charge', '0-1']]],
    ['Tilt servos', [['servo.name', 'Model', 'text'], ['servo.stallTorque48', 'Stall torque @4.8 V', 'N·m'], ['servo.stallTorque60', 'Stall torque @6 V', 'N·m'], ['servo.speed48', 'Speed @4.8 V', 's/60°'], ['servo.speed60', 'Speed @6 V', 's/60°'],
      ['servo.degPerUs', 'Servo travel', '°/us'], ['servo.linkRatio', 'Linkage ratio (servo°/nacelle°)', ''], ['servo.gearStripTorque', 'Gear-strip estimate', 'N·m'], ['servo.iStall48', 'Stall current @4.8 V', 'A'],
      ['servo.dirL', 'Left: PWM ↑ tilts nacelle', O([[1, 'forward'], [-1, 'backward']])], ['servo.dirR', 'Right: PWM ↑ tilts nacelle', O([[1, 'forward'], [-1, 'backward']])],
      ['servo.trimDegL', 'Left mechanical trim', '°'], ['servo.trimDegR', 'Right mechanical trim', '°']]],
    ['Power distribution', [['power.battLead.awg', 'Battery lead', O(Object.keys(S.Hardware.AWG).map((k) => [Number(k), k + ' AWG']))], ['power.battLead.length', 'Battery lead length', 'm'], ['power.battLead.connector', 'Battery connector', O(Object.keys(S.Hardware.CONNECTORS).map((k) => [k, k]))],
      ['power.escLead.awg', 'ESC power leads', O(Object.keys(S.Hardware.AWG).map((k) => [Number(k), k + ' AWG']))], ['power.motorLead.awg', 'Motor phase wires', O(Object.keys(S.Hardware.AWG).map((k) => [Number(k), k + ' AWG']))],
      ['power.fc5v.source', 'Pico 5 V from', O([['escbec', 'ESC BEC'], ['pdb5v', 'PDB 5 V regulator'], ['ubec', 'Dedicated UBEC']])], ['power.fc5v.type', 'Regulator type', O([['linear', 'Linear'], ['buck', 'Switching (buck)']])],
      ['power.fc5v.imax', 'Pico 5 V capacity', 'A'], ['power.fc5v.dropout', 'Linear dropout', 'V'],
      ['power.servo.source', 'Servos powered from', O([['ubec', 'Separate UBEC'], ['fc5v', 'Pico 5 V rail (shared)']])], ['power.servo.v', 'Servo UBEC voltage', 'V'], ['power.servo.imax', 'Servo UBEC capacity', 'A'],
      ['power.radio.source', 'Radio 3.3 V from', O([['ldo', 'Dedicated LDO (from 5 V)'], ['pico3v3', 'Pico 3V3(OUT) pin']])], ['power.radio.capUF', 'Radio decoupling', 'µF'], ['power.radio.dropout', 'Radio LDO dropout', 'V'],
      ['power.commonGround', 'Common ground', 'bool'], ['power.mpuBreakout', 'MPU6500 breakout', O([['ldo', 'Has regulator (3.3-5 V)'], ['bare', '3.3 V only']])], ['power.mpuSupply', 'MPU6500 fed from', O([['pico3v3', 'Pico 3V3'], ['fc5v', '5 V']])],
      ['power.vbatTop', 'VBAT divider top', 'Ω'], ['power.vbatBottom', 'VBAT divider bottom', 'Ω']]],
    ['Electronics loads', [['loads.picoA', 'Pico 2 (VSYS)', 'A'], ['loads.imuA', 'MPU6500', 'A'], ['loads.radioRxA', 'Radio RX', 'A'], ['loads.radioTxPeakA', 'Radio TX/ACK peak', 'A'], ['loads.radioTxDuty', 'TX duty', '0-1']]],
    ['Radio link (handset)', [['link.linkId', 'HANDSET_LINK_ID', 'text'], ['link.rfPwrBits', 'HANDSET_RF_PWR_BITS', O([[0, '0x00 (−18 dBm)'], [2, '0x02 (−12 dBm)'], [4, '0x04 (−6 dBm)'], [6, '0x06 (0 dBm)']])],
      ['link.paGainDb', 'PA gain', 'dB'], ['link.rxSensDbm', 'RX sensitivity (with LNA)', 'dBm'], ['link.lnaSatDbm', 'LNA saturation', 'dBm'], ['link.pathLossExp', 'Path-loss exponent', ''], ['link.fadeSigmaDb', 'Fading σ', 'dB'],
      ['link.wifiChannel', 'Nearby Wi-Fi channel (0 = none)', ''], ['link.wifiLevelDb', 'Wi-Fi desense', 'dB'], ['link.pilotPos.0', 'Pilot x', 'm'], ['link.pilotPos.1', 'Pilot y', 'm']]],
    ['IMU mounting', [['imuMount.roll', 'Physical roll', '°'], ['imuMount.pitch', 'Physical pitch', '°'], ['imuMount.yaw', 'Physical yaw', '°']]],
    ['Environment', [['environment.altitudeAmsl', 'Field elevation', 'm'], ['environment.tempC', 'Air temperature', '°C']]],
  ];
  const getPath = (o, p) => p.split('.').reduce((a, k) => (a == null ? a : a[k]), o);
  const setPath = (o, p, v) => { const ks = p.split('.'); const last = ks.pop(); const t = ks.reduce((a, k) => a[k], o); t[last] = v; };

  function buildHwForm() {
    const open = new Set([...document.querySelectorAll('#hwForm details[open]')].map((x) => x.dataset.g));
    const d = S.Hardware.defaults();
    let html = `<details class="group" data-g="components" ${open.has('components') || !open.size ? 'open' : ''}><summary>Components (mass & position)</summary><div style="padding:4px 8px 10px;overflow:auto">
      <table class="comp"><thead><tr><th>Part</th><th>g</th><th>x mm</th><th>y mm</th><th>z mm</th><th>tilts</th><th></th></tr></thead><tbody>` +
      app.hw.components.map((c, i) => `<tr><td><input class="name" data-c="${i}" data-k="name" value="${esc(c.name)}"></td>` +
        (c.auto ? `<td colspan="5" class="muted">auto from frame (${(S.Hardware.resolveComponents(app.hw)[i].mass * 1000).toFixed(1)} g)</td>` :
        `<td><input type="number" step="0.1" data-c="${i}" data-k="mass" value="${U.round(c.mass * 1000, 1)}"></td>` +
        [0, 1, 2].map((j) => `<td><input type="number" step="1" data-c="${i}" data-k="pos${j}" value="${U.round(c.pos[j] * 1000, 1)}" ${c.tilts && j === 1 ? 'disabled title="set by arm length"' : ''}></td>`).join('') +
        `<td><select data-c="${i}" data-k="tilts"><option value="" ${!c.tilts ? 'selected' : ''}>—</option><option ${c.tilts === 'L' ? 'selected' : ''}>L</option><option ${c.tilts === 'R' ? 'selected' : ''}>R</option></select></td>`) +
        `<td>${c.auto ? '' : `<button class="small" data-del="${i}" title="Remove">×</button>`}</td></tr>`).join('') +
      `</tbody></table><div class="row" style="margin-top:6px"><button class="small" id="addComp">Add part</button><span class="muted tiny">x forward, y left, z up, from the midpoint between the tilt axes. Tilting parts rotate with the nacelle.</span></div></div></details>`;
    for (const [g, fields] of HW_FIELDS) {
      html += `<details class="group" data-g="${g}" ${open.has(g) ? 'open' : ''}><summary>${g}</summary><div class="fields">` + fields.map(([path, label, unit]) => {
        const v = getPath(app.hw, path), dv = getPath(d, path), ch = JSON.stringify(v) !== JSON.stringify(dv) ? ' changed' : '';
        let ctl;
        if (Array.isArray(unit)) ctl = `<select data-h="${path}">${unit.map((o) => `<option value="${o.v}" ${String(o.v) === String(v) ? 'selected' : ''}>${esc(o.l)}</option>`).join('')}</select>`;
        else if (unit === 'bool') ctl = `<select data-h="${path}" data-bool="1"><option value="true" ${v ? 'selected' : ''}>yes</option><option value="false" ${!v ? 'selected' : ''}>no</option></select>`;
        else if (unit === 'text') ctl = `<input type="text" data-h="${path}" value="${esc(String(v))}">`;
        else ctl = `<input type="number" step="any" data-h="${path}" value="${v}">`;
        return `<label class="field${ch}"><span>${label}${unit && !Array.isArray(unit) && unit !== 'bool' && unit !== 'text' ? ` (${unit})` : ''}</span>${ctl}</label>`;
      }).join('') + '</div></details>';
    }
    $('hwForm').innerHTML = html;
    $('hwForm').querySelectorAll('[data-h]').forEach((el) => el.addEventListener('change', () => {
      const path = el.dataset.h, old = getPath(app.hw, path);
      let v = el.value;
      if (el.dataset.bool) v = v === 'true';
      else if (typeof old === 'number') { v = Number(v); if (!Number.isFinite(v)) return; }
      setPath(app.hw, path, v); buildHwForm(); changed();
    }));
    $('hwForm').querySelectorAll('[data-c]').forEach((el) => el.addEventListener('change', () => {
      const c = app.hw.components[Number(el.dataset.c)], k = el.dataset.k;
      if (k === 'name') c.name = el.value;
      else if (k === 'mass') c.mass = Math.max(0, Number(el.value) / 1000);
      else if (k === 'tilts') { c.tilts = el.value || undefined; if (!c.tilts) delete c.tilts; }
      else c.pos[Number(k.slice(3))] = Number(el.value) / 1000;
      buildHwForm(); changed();
    }));
    $('hwForm').querySelectorAll('[data-del]').forEach((el) => el.addEventListener('click', () => { app.hw.components.splice(Number(el.dataset.del), 1); buildHwForm(); changed(); }));
    $('addComp').addEventListener('click', () => { app.hw.components.push({ id: 'extra' + Date.now(), name: 'New part (e.g. ESP32-CAM)', mass: 0.01, pos: [0.05, 0, -0.02], size: [0.03, 0.03, 0.01] }); buildHwForm(); changed(); });
    drawMass();
  }

  function drawMass() {
    const mp = S.Hardware.massProperties(app.hw), cg = mp.cg;
    $('massSummary').innerHTML = `<b>${(mp.mass * 1000).toFixed(0)} g</b> all-up · CG x ${(cg[0] * 1000).toFixed(1)} mm, y ${(cg[1] * 1000).toFixed(1)} mm, z ${(cg[2] * 1000).toFixed(1)} mm (below tilt axis by ${(-cg[2] * 1000).toFixed(1)} mm)`;
    const cv = $('cgView'), d = window.devicePixelRatio || 1, r = cv.getBoundingClientRect();
    cv.width = Math.round(r.width * d) || 600; cv.height = Math.round(r.height * d) || 190;
    const g = cv.getContext('2d'), W = cv.width, H = cv.height;
    g.clearRect(0, 0, W, H);
    const L = app.hw.frame.armLength, sc = Math.min((W / 2 - 20 * d) / (2 * L + 0.15), (H - 30 * d) / 0.45);
    const panel = (ox, title, proj) => {
      g.fillStyle = theme.muted; g.font = `${11 * d}px system-ui`; g.fillText(title, ox + 6 * d, 14 * d);
      const cx = ox + W / 4, cy = H / 2 + 6 * d;
      g.strokeStyle = theme.line; g.beginPath(); g.moveTo(ox + 8 * d, cy); g.lineTo(ox + W / 2 - 8 * d, cy); g.moveTo(cx, 20 * d); g.lineTo(cx, H - 6 * d); g.stroke();
      for (const c of mp.placed) {
        const [a, b] = proj(c.p); const rr = Math.max(2 * d, Math.sqrt(c.mass * 1000) * 1.1 * d);
        g.fillStyle = /batt/.test(c.id) ? theme.battery : /mot|prop/.test(c.id) ? theme.motor : /servo/.test(c.id) ? theme.servo : /esc|pdb|pico|imu|radio|ubec|ldo/.test(c.id) ? theme.elec : theme.frame;
        g.globalAlpha = 0.75; g.beginPath(); g.arc(cx + a * sc, cy - b * sc, rr, 0, 7); g.fill(); g.globalAlpha = 1;
      }
      const [a, b] = proj(cg);
      g.strokeStyle = theme.cg; g.lineWidth = 2 * d; g.beginPath(); g.moveTo(cx + a * sc - 7 * d, cy - b * sc); g.lineTo(cx + a * sc + 7 * d, cy - b * sc); g.moveTo(cx + a * sc, cy - b * sc - 7 * d); g.lineTo(cx + a * sc, cy - b * sc + 7 * d); g.stroke(); g.lineWidth = d;
      return { cx, cy };
    };
    panel(0, 'Rear view (aircraft left on the left, z ↑)', (p) => [-p[1], p[2]]);
    const s = panel(W / 2, 'Side view (nose to the right) · red + = CG, origin = tilt axis', (p) => [p[0], p[2]]);
    g.strokeStyle = theme.thrust; g.setLineDash([4 * d, 3 * d]); g.beginPath(); g.moveTo(s.cx, s.cy - 0.12 * sc); g.lineTo(s.cx, s.cy + 0.15 * sc); g.stroke(); g.setLineDash([]);
  }

  // ------------------------------------------------------------------ checks tab
  let anaTimer = null;
  function scheduleAnalysis() { clearTimeout(anaTimer); anaTimer = setTimeout(runAnalysis, 150); }
  const FIX_CMD = /^(set |servo |smix |aux |mmix |rxfail |resource )/;
  function runAnalysis() {
    let a;
    try { a = S.Analysis.analyze(app.hw, app.fw); } catch (e) { $('checksList').innerHTML = `<p class="status fail">Analysis error: ${esc(e.message)}</p>`; return; }
    app.analysis = a;
    const sm = a.summary;
    $('checksSummary').innerHTML = ['fail', 'warn', 'pass', 'info'].map((k) => `<span class="chip ${k === 'info' ? '' : k}">${sm[k]} ${k}</span>`).join('');
    const b = $('checksBadge'); b.textContent = sm.fail ? sm.fail : sm.warn || ''; b.className = 'badge ' + (sm.fail ? 'fail' : sm.warn ? 'warn' : '');
    const order = { fail: 0, warn: 1, info: 2, pass: 3 };
    $('checksList').innerHTML = a.sections.map((s) => `<h3>${s.title}</h3>` + s.items.slice().sort((x, y) => order[x.status] - order[y.status]).map((i) => {
      const cmd = FIX_CMD.test(i.fix || '') ? i.fix.split(/ — | \(| or |, |; /)[0].split(' and ').map((x) => x.trim()).join('\n') : '';
      const fixHtml = i.fix ? `<div class="fix">Fix: ${esc(i.fix)}${cmd ? ` <button class="small" data-fix="${esc(cmd)}" title="Run in the CLI: ${esc(cmd)}">Apply</button>` : ''}</div>` : '';
      return `<div class="check"><span class="status ${i.status}">${i.status.toUpperCase()}</span><div><span class="name">${esc(i.name)}</span> — <span class="val">${esc(String(i.value))}</span></div>${i.detail ? `<div class="detail">${esc(i.detail)}</div>` : ''}${fixHtml}</div>`;
    }).join('')).join('');
    $('checksList').querySelectorAll('[data-fix]').forEach((el) => el.addEventListener('click', () => { runCli(el.dataset.fix); }));
    // hardware-dependent views
    drawMass();
  }

  // ------------------------------------------------------------------ tests tab
  function renderTests(progress) {
    const res = app.testResults, byId = Object.fromEntries(res.map((r) => [r.id, r]));
    $('testsList').innerHTML = S.Tests.TESTS.map((t) => {
      const r = byId[t.id], st = r ? r.status : (progress === t.id ? 'running' : '');
      return `<div class="testcard"><div class="head"><span class="status ${r ? r.status : 'info'}" style="width:52px">${st ? st.toUpperCase() : '—'}</span><span class="name">${esc(t.name)}</span><span class="muted tiny">${t.group}</span>
        <button class="small" data-run="${t.id}">Run</button>${r && r.world ? `<button class="small" data-replay="${t.id}">Replay</button>` : ''}</div>
        <div class="muted tiny">${esc(t.desc)}</div>
        ${r ? `<div style="margin-top:4px"><b>${esc(r.summary)}</b></div><table>${r.metrics.map(([k, v]) => `<tr><td>${esc(k)}</td><td>${esc(String(v))}</td></tr>`).join('')}</table>${(r.notes || []).map((n) => `<div class="muted tiny">${esc(n)}</div>`).join('')}` : ''}</div>`;
    }).join('');
    $('testsList').querySelectorAll('[data-run]').forEach((el) => el.addEventListener('click', () => runTests([el.dataset.run])));
    $('testsList').querySelectorAll('[data-replay]').forEach((el) => el.addEventListener('click', () => startReplay(byId[el.dataset.replay])));
    const sm = { pass: 0, warn: 0, fail: 0 }; res.forEach((r) => { if (sm[r.status] !== undefined) sm[r.status]++; });
    const b = $('testsBadge'); b.textContent = res.length ? (sm.fail || sm.warn || sm.pass) : ''; b.className = 'badge ' + (sm.fail ? 'fail' : sm.warn ? 'warn' : res.length ? 'pass' : '');
  }
  async function runTests(only) {
    $('btnRunTests').disabled = true;
    const t0 = performance.now();
    const results = await S.Tests.runAll(app.hw, app.fw, (t, r) => {
      $('testsProgress').textContent = r ? '' : `Running: ${t.name}…`;
      if (r) { app.testResults = app.testResults.filter((x) => x.id !== r.id).concat([r]); app.testResults.sort((a, b) => S.Tests.TESTS.findIndex((t) => t.id === a.id) - S.Tests.TESTS.findIndex((t) => t.id === b.id)); }
      renderTests(r ? null : t.id);
    }, only);
    $('testsProgress').textContent = `${results.length} test${results.length > 1 ? 's' : ''} in ${((performance.now() - t0) / 1000).toFixed(1)} s`;
    $('btnRunTests').disabled = false;
  }

  // ------------------------------------------------------------------ replay
  function startReplay(r) {
    const w = r.world; if (!w || !w.record.length) return;
    app.running = false; $('btnRun').textContent = 'Run';
    const mpCg = w.mp.cg;
    const view = { hw: w.hw, pos: [0, 0, 0], q: [1, 0, 0, 0], tilt: [0, 0], rotor: [0, 0], thrust: [0, 0], trail: [], vibPhase: [0, 0], cgWorld() { return this._cg; }, _cg: [0, 0, 0] };
    app.replay = { rec: w.record, events: w.events, idx: 0, playing: true, view, name: r.name, mpCg, cur: null };
    $('replaybar').hidden = false; $('replayName').textContent = 'Replay: ' + r.name; $('replayPlay').textContent = 'Pause';
    $('replaySlider').max = w.record.length - 1;
    if (r.window) { const i = w.record.findIndex((x) => x.t >= r.window[0]); app.replay.idx = Math.max(0, i); }
    setReplayIdx(app.replay.idx);
    renderLog(true);
    document.querySelector('[data-tab="telemetry"]').click();
  }
  function setReplayIdx(i) {
    const R = app.replay; R.idx = U.clamp(Math.round(i), 0, R.rec.length - 1);
    const r = R.rec[R.idx], v = R.view; R.cur = r;
    v.q = U.qFromEuler(r.roll * U.DEG, r.pitch * U.DEG, r.yaw * U.DEG);
    v._cg = r.pos.slice(); v.pos = U.sub(r.pos, U.qRot(v.q, R.mpCg));
    v.tilt = r.tilt.map((x) => x * U.DEG); v.rotor = r.rpm.map((x) => x * 2 * Math.PI / 60); v.thrust = r.thrust.slice();
    v.vibPhase = v.vibPhase.map((p, k) => p + v.rotor[k] * 0.01);
    v.trail = R.rec.slice(Math.max(0, R.idx - 600), R.idx + 1).filter((_, k) => k % 5 === 0).map((x) => x.pos);
    $('replaySlider').value = R.idx;
  }
  function stepReplay(dt) { const R = app.replay; if (!R.playing) return; const next = R.idx + dt * app.speed / 0.01; if (next >= R.rec.length - 1) { R.playing = false; $('replayPlay').textContent = 'Play'; } setReplayIdx(next); }
  function exitReplay() { app.replay = null; $('replaybar').hidden = true; renderLog(true); }

  // ------------------------------------------------------------------ faults
  function fault(kind) {
    if (app.replay) exitReplay();
    const w = app.world, now = Math.round(w.t * 1e6);
    const drop = (ms) => { w.radio.dropouts.push({ from: now, to: now + ms * 1000 }); w.log(`Injected RF dropout ${ms} ms`, 'warn'); };
    switch (kind) {
      case 'drop100': drop(100); break;
      case 'drop400': drop(400); break;
      case 'drop3000': drop(3000); break;
      case 'hsToggle': if (w.handset.on) { w.handset.on = false; w.log('Handset switched OFF', 'warn'); } else { w.handset.on = true; w.handset.boot(now); w.log('Handset switched ON (new session → restart announcement)', 'warn'); } break;
      case 'reboot': w.handset.boot(now); w.log('Handset reboot injected', 'warn'); break;
      case 'inputFault': w.handset.faults.inputFaultUntil = now + 100000; w.log('Handset input fault for 100 ms', 'warn'); break;
      case 'servoL': w.faults.servoDead[0] = true; w.log('Left servo failed (limp)', 'error'); break;
      case 'motorR': w.faults.escDead[1] = true; w.log('Right ESC failed', 'error'); break;
      case 'brownout': w.faults.brownUntil = w.t + 0.2; w.log('Pico supply interrupted for 200 ms', 'error'); break;
      case 'clearHw': w.faults.servoDead = [false, false]; w.faults.escDead = [false, false]; w.log('Hardware faults cleared'); break;
    }
  }

  // ------------------------------------------------------------------ report
  function report() {
    const a = app.analysis || S.Analysis.analyze(app.hw, app.fw);
    const lines = [`# Bicopter simulator report`, '', `Generated ${new Date().toISOString()}`, '', `Hardware: ${app.hw.name}; all-up ${(a.mp.mass * 1000).toFixed(0)} g`, '', '## Static checks', ''];
    for (const s of a.sections) { lines.push(`### ${s.title}`, ''); for (const i of s.items) lines.push(`- **${i.status.toUpperCase()}** ${i.name}: ${i.value}${i.detail ? ` — ${i.detail}` : ''}${i.fix ? ` *Fix:* ${i.fix}` : ''}`); lines.push(''); }
    lines.push('## Flight tests', '');
    if (!app.testResults.length) lines.push('_Not run._');
    for (const r of app.testResults) { lines.push(`### ${r.status.toUpperCase()} — ${r.name}`, '', r.summary, ''); for (const [k, v] of r.metrics) lines.push(`- ${k}: ${v}`); for (const n of r.notes || []) lines.push(`- note: ${n}`); lines.push(''); }
    lines.push('## Firmware diff', '', '```', S.Firmware.dump(app.fw, true), '```', '', '_Simulation results are model predictions. Verify on hardware with props off first (HANDOFF.md section 13)._');
    download('bicopter-sim-report.md', lines.join('\n'), 'text/markdown');
  }
  function download(name, text, type) {
    const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type })); a.download = name; document.body.appendChild(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 500);
  }
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  // ------------------------------------------------------------------ help
  const HELP = `<div class="help">
  <h3>What this simulates</h3>
  <p>The full control chain of your bicopter, step by step every 1 ms: the STM32 handset (session, sample sequence, restart announcement), the nRF24 link (path loss, PA/LNA saturation, Wi-Fi overlap, frequency hopping), the receiver adapter from <code>rx/nrf24_bicopter.c</code> (freshness, arm interlock, restart → immediate disarm), a Betaflight-style flight controller (RX watchdog, rxfail, failsafe stages, arming flags, ACTUAL rates, angle mode, PID with Betaflight scaling, TPA, BI mixer, servo mixer, 200 Hz motor / 50 Hz servo outputs), SimonK ESCs, A2212 motors with a prop model, battery sag, power rails (BEC, UBEC, LDO, Pico 3V3, brown-out), MG90S tilt servos with torque and speed limits, gyroscopic loads of the spinning props, ground contact and live arm stresses.</p>
  <p>The protocol core is a line-by-line port of <code>nrf24_bicopter_link.c</code>; it is checked against the same known packet vector as the C tests.</p>
  <h3>Flying</h3>
  <ul><li>Press <b>Run</b>. The handset announces its restart for ~1 s; then flip <b>ARM</b> (Space) with throttle low.</li>
  <li>Mode 2 sticks: left = throttle/yaw (<kbd>W</kbd>/<kbd>S</kbd>, <kbd>A</kbd>/<kbd>D</kbd>), right = pitch/roll (arrow keys). Drag the on-screen sticks or use a gamepad (button A toggles ARM).</li>
  <li><b>altitude assist</b> turns the throttle stick into a climb-rate command; <b>position assist</b> holds position when the right stick is centred. Both act like a careful pilot; the firmware itself has no altitude hold.</li>
  <li><kbd>M</kbd> toggles ANGLE/ACRO (AUX2), <kbd>P</kbd> pauses, <kbd>R</kbd> resets. Drag the 3D view to orbit, scroll to zoom.</li></ul>
  <h3>Customising the firmware</h3>
  <p>Use the CLI exactly like Betaflight's (<code>set</code>, <code>get</code>, <code>diff</code>, <code>dump</code>, <code>aux</code>, <code>servo</code>, <code>smix reverse</code>, <code>mmix</code>, <code>rxfail</code>, <code>resource</code>, <code>save</code>), the parameter form, or the custom JavaScript hook. Board values from <code>config.h</code> (link ID, nRF24 SPI clock, pins) are exposed as parameters too. <b>Download diff</b> exports the changes for Betaflight Configurator.</p>
  <h3>Conventions</h3>
  <ul><li>Body axes: x forward, y left, z up. Roll + = right side down, pitch + = nose down, yaw + = nose left (Betaflight's frame; the yaw stick is negated, so stick right = nose right).</li>
  <li>Tilt + = nacelle tilted forward (thrust toward the nose). Set each servo's real direction in Hardware → Tilt servos after watching it in Configurator's servo tab.</li>
  <li>On the BI mixer Betaflight drives the tilt servos with <code>servo 4</code> (left) and <code>servo 5</code> (right) settings.</li></ul>
  <h3>Limits of the model</h3>
  <ul><li>Masses, motor/prop coefficients, servo torque and rail capacities are catalogue estimates: weigh and measure your parts and enter them.</li>
  <li>The flight controller is a model of Betaflight's behaviour, not the compiled C. Filters are simplified (no dynamic notch or RPM filter), the IMU estimator is a plain Mahony filter, and stage-1 failsafe follows rxfail AUTO semantics. Confirm on the bench.</li>
  <li>Aerodynamics are quasi-static (no vortex ring state, simple inflow and ground effect). Structural loads use beam theory at the arm root; real joints and glue lines are usually weaker.</li>
  <li>Passing here is necessary, not sufficient: follow the props-off checklist in HANDOFF.md section 13 before any flight.</li></ul>
  </div>`;

  // ------------------------------------------------------------------ init
  function init() {
    load(); readTheme();
    try { window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { readTheme(); buildHwForm(); }); } catch (e) { /* old browsers */ }
    app.view = new S.View3D($('view'));
    app.stickL = makeStick($('stickL'), { springX: true, springY: false, set: (x, y) => { app.sticks.yaw = x; app.sticks.thr = (y + 1) / 2; } });
    app.stickR = makeStick($('stickR'), { springX: true, springY: true, set: (x, y) => { app.sticks.roll = x; app.sticks.pitch = y; } });
    newWorld();
    buildCharts(); buildParamForm(); buildHwForm(); syncCustom(); runAnalysis(); renderTests();
    $('helpText').innerHTML = HELP;
    cliPrint('# Betaflight CLI (simulated). Type help.\n# Firmware: 2026.6.2 + NRF24_BICOPTER v2, target BICOPTER_PICO2');

    document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
      document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
      document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
      if (b.dataset.tab === 'hardware') drawMass();
    }));
    $('btnRun').addEventListener('click', () => { if (app.replay) exitReplay(); app.running = !app.running; $('btnRun').textContent = app.running ? 'Pause' : 'Run'; });
    $('btnReset').addEventListener('click', () => { if (app.replay) exitReplay(); newWorld(); setArm(false); app.sticks.thr = 0; });
    $('btnApply').addEventListener('click', () => { newWorld(); setArm(false); app.sticks.thr = 0; });
    $('speed').addEventListener('change', (e) => { app.speed = Number(e.target.value); });
    $('swArm').classList.add('arm');
    $('swArm').addEventListener('click', () => setArm(!app.arm));
    $('swMode').addEventListener('click', () => setAngle(!app.angle));
    document.querySelectorAll('[data-fault]').forEach((b) => b.addEventListener('click', () => fault(b.dataset.fault)));
    $('lossSlider').addEventListener('input', (e) => { app.world.radio.extraLoss = Number(e.target.value) / 100; $('lossVal').textContent = e.target.value + ' %'; });
    $('windSlider').addEventListener('input', (e) => { app.world.wind[0] = Number(e.target.value); $('windVal').textContent = e.target.value + ' m/s'; });
    $('gustSlider').addEventListener('input', (e) => { app.world.hw.environment.gust = Number(e.target.value); $('gustVal').textContent = e.target.value; });
    // CLI
    $('cliRun').addEventListener('click', () => { const t = $('cliIn').value; if (t.trim()) { runCli(t); $('cliIn').value = ''; } });
    $('cliIn').addEventListener('keydown', (e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); $('cliRun').click(); } });
    $('presetProject').addEventListener('click', () => { const code = app.fw.customCode; app.fw = S.Firmware.defaults(); app.fw.customCode = code; runCli(PROJECT_CLI); });
    $('presetTune').addEventListener('click', () => runCli(TUNE_CLI));
    $('btnDiff').addEventListener('click', () => download('bicopter-diff.txt', S.Firmware.dump(app.fw, true), 'text/plain'));
    $('btnDump').addEventListener('click', () => cliPrint(S.Firmware.dump(app.fw, false)));
    $('customCode').addEventListener('input', () => { app.fw.customCode = $('customCode').value; checkCustom(); changed(); });
    $('customEnabled').addEventListener('change', () => { app.fw.customEnabled = $('customEnabled').checked; checkCustom(); changed(); });
    // hardware
    $('btnExport').addEventListener('click', () => download('bicopter-project.json', JSON.stringify({ hw: app.hw, fw: app.fw }, null, 2), 'application/json'));
    $('fileImport').addEventListener('change', async (e) => {
      const f = e.target.files[0]; if (!f) return;
      try { const o = JSON.parse(await f.text()); app.hw = mergeDefaults(S.Hardware.defaults(), o.hw); app.fw = mergeDefaults(S.Firmware.defaults(), o.fw); buildHwForm(); buildParamForm(); syncCustom(); changed(); newWorld(); }
      catch (err) { alert('Could not read that project file: ' + err.message); }
      e.target.value = '';
    });
    $('btnHwDefaults').addEventListener('click', () => { if (confirm('Replace all hardware values with the defaults?')) { app.hw = S.Hardware.defaults(); buildHwForm(); changed(); } });
    // tests
    $('btnRunTests').addEventListener('click', () => runTests());
    $('btnReport').addEventListener('click', report);
    $('replaySlider').addEventListener('input', (e) => { if (app.replay) { app.replay.playing = false; $('replayPlay').textContent = 'Play'; setReplayIdx(Number(e.target.value)); } });
    $('replayPlay').addEventListener('click', () => { const R = app.replay; if (!R) return; if (R.idx >= R.rec.length - 1) R.idx = 0; R.playing = !R.playing; $('replayPlay').textContent = R.playing ? 'Pause' : 'Play'; });
    $('replayExit').addEventListener('click', exitReplay);
    // keyboard
    const typing = (e) => /INPUT|TEXTAREA|SELECT/.test(e.target.tagName);
    window.addEventListener('keydown', (e) => {
      if (typing(e)) return;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) e.preventDefault();
      if (e.repeat) return;
      if (e.code === 'Space') setArm(!app.arm);
      else if (e.code === 'KeyM') setAngle(!app.angle);
      else if (e.code === 'KeyP') $('btnRun').click();
      else if (e.code === 'KeyR') $('btnReset').click();
      app.keys.add(e.code);
    });
    window.addEventListener('keyup', (e) => app.keys.delete(e.code));
    window.addEventListener('blur', () => app.keys.clear());
    window.addEventListener('resize', () => drawMass());
    requestAnimationFrame(frame);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();
})(window.Sim);
