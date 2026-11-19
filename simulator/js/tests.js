// Dynamic test suite: scripted flights through the full chain
// handset -> radio -> receiver adapter -> flight controller -> ESC/servos -> airframe.
// A virtual pilot (altitude + position hold using true state, like a good human pilot)
// flies each manoeuvre so the result reflects the firmware and hardware, not stick skill.
(function (S) {
  'use strict';
  const U = S.U;

  class Pilot {
    constructor() { this.alt = 1.5; this.x = 0; this.y = 0; this.yaw = 0; this.iAlt = 0; this.hold = true; this.yawHold = true; this.thrBase = 1480; this.override = {}; this.active = false; }
    apply(w, dt) {
      const i = w.handset.inputs;
      if (!this.active) return;
      const cg = w.cgWorld(), v = w.vel, e = U.qToEuler(w.q);
      const ea = this.alt - cg[2];
      this.iAlt = U.clamp(this.iAlt + ea * dt, -3, 3);
      let thr = this.thrBase + 160 * ea - 120 * v[2] + 90 * this.iAlt;
      // position hold in the heading frame (angle mode: stick = angle demand)
      const cy = Math.cos(e[2]), sy = Math.sin(e[2]);
      const ex = this.x - cg[0], ey = this.y - cg[1];
      const exb = cy * ex + sy * ey, eyb = -sy * ex + cy * ey, vxb = cy * v[0] + sy * v[1], vyb = -sy * v[0] + cy * v[1];
      let pitch = this.hold ? U.clamp(0.08 * exb - 0.12 * vxb, -0.25, 0.25) : 0;
      let roll = this.hold ? U.clamp(-(0.08 * eyb - 0.12 * vyb), -0.25, 0.25) : 0;
      let yawS = this.yawHold ? U.clamp(1.2 * (((e[2] - this.yaw + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI), -0.4, 0.4) : 0;
      const o = this.override;
      if (o.pitch !== undefined) pitch = o.pitch;
      if (o.roll !== undefined) roll = o.roll;
      if (o.yaw !== undefined) yawS = o.yaw;
      if (o.throttle !== undefined) thr = o.throttle;
      i.throttle = U.clamp(thr, 1000, 2000);
      i.pitch = 1500 + 500 * pitch; i.roll = 1500 + 500 * roll; i.yaw = 1500 + 500 * yawS;
    }
  }

  // Standard prelude: boot, ARM at 1.5 s, take off at 2 s, settle in hover by ~6 s.
  function makeWorld(hw, fw, extra = {}) {
    const h = U.deepCopy(hw), f = U.deepCopy(fw);
    if (extra.hw) extra.hw(h);
    if (extra.fw) extra.fw(f);
    const w = new S.Physics.World(h, f, { seed: extra.seed || 42, noise: true, record: true });
    const pilot = new Pilot();
    const hov = S.Analysis.hoverPoint(h, f);
    const p = f.params;
    // throttle stick that gives hover duty (inverse of the mixer + ESC mapping)
    const motorUs = h.esc.calLow + hov.duty * (h.esc.calHigh - h.esc.calLow);
    const low = 1000 + p.motor_idle / 10, high = 1000 + 10 * p.motor_output_limit;
    const thr = U.clamp((motorUs - low) / (high - low) / (p.throttle_limit_percent / 100), 0, 1);
    pilot.thrBase = p.min_check + thr * (2000 - p.min_check);
    w.pilot = pilot;
    const steps = [];
    w.scenario = { update(w, t) {
      const i = w.handset.inputs;
      if (t < 1.5) { i.throttle = 1000; i.aux[0] = 1000; }
      else if (t < 2.0) { i.aux[0] = 1900; }
      else if (!pilot.disabled) pilot.active = true;
      for (const s of steps) if (!s.done && t >= s.t) { s.done = true; s.fn(w, t); }
      pilot.apply(w, w.dt);
    } };
    w.at = (t, fn) => steps.push({ t, fn, done: false });
    return w;
  }

  const stat = (arr) => { const n = arr.length || 1, m = arr.reduce((a, b) => a + b, 0) / n; return { mean: m, rms: Math.sqrt(arr.reduce((a, b) => a + b * b, 0) / n), max: Math.max(0, ...arr.map(Math.abs)) }; };
  const between = (rec, a, b) => rec.filter((r) => r.t >= a && r.t <= b);
  const firstEvent = (w, re, after = 0) => { const e = w.events.find((x) => x.t >= after && re.test(x.msg)); return e ? e.t : null; };
  const res = (status, summary, metrics, notes = []) => ({ status, summary, metrics, notes });

  const TESTS = [
    {
      id: 'arming', name: 'Boot & arming interlock', group: 'Firmware',
      desc: 'Handset boots with ARM on and throttle up. Nothing may arm until ARM is switched off with throttle low, then deliberately on.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.scenario = { update(w, t) { const i = w.handset.inputs; if (t < 2.5) { i.aux[0] = 1900; i.throttle = 1600; } else if (t < 3.0) { i.aux[0] = 1000; i.throttle = 1000; } else { i.aux[0] = 1900; i.throttle = 1000; } } };
        let armedEarly = false, armedLate = false;
        for (let k = 0; k < 3500; k++) { w.step(); if (w.t < 2.5 && w.fc.armed) armedEarly = true; if (w.t > 3.0 && w.fc.armed) armedLate = true; }
        const ok = !armedEarly && armedLate && w.rotor[0] < 600;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Interlock held during boot; deliberate re-arm worked.' : armedEarly ? 'Armed with the switch already on at boot!' : 'Did not arm after a proper OFF→ON sequence.',
          [['Armed during boot', armedEarly ? 'YES' : 'no'], ['Armed after OFF→ON', armedLate ? 'yes' : 'NO'], ['Receiver accepted frames', w.receiver.stats.accepted], ['Restart packets', w.receiver.stats.txFault]],
          w.events.filter((e) => /block|ARMED|RX/.test(e.msg)).slice(0, 6).map((e) => `${e.t.toFixed(2)} s ${e.msg}`)), { world: w });
      },
    },
    {
      id: 'hover', name: 'Take-off & hover stability', group: 'Flight',
      desc: 'Angle mode, 1.5 m hover with light position hold. Checks attitude noise, oscillation and hover current.',
      run(hw, fw) {
        const w = makeWorld(hw, fw); w.run(12);
        const r = between(w.record, 6, 12);
        const att = stat(r.flatMap((x) => [x.roll, x.pitch])), rate = stat(r.flatMap((x) => [x.rate[0], x.rate[1]]));
        const alt = stat(r.map((x) => x.alt - 1.5)), I = stat(r.map((x) => x.Ibatt));
        const drift = Math.hypot(...w.cgWorld().slice(0, 2));
        const fail = w.crashed || !w.fc.armed || att.max > 20 || rate.rms > 60;
        const warn = att.rms > 2.5 || rate.rms > 15 || alt.rms > 0.3;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', fail ? (w.crashed ? 'Crashed: ' + w.crashReason : 'Unstable hover.') : warn ? 'Flies, but with oscillation/noise: tune PID or filters.' : 'Stable hover.',
          [['Attitude RMS / max', `${att.rms.toFixed(2)}° / ${att.max.toFixed(1)}°`], ['Roll/pitch rate RMS', `${rate.rms.toFixed(1)} °/s`], ['Altitude error RMS', `${alt.rms.toFixed(2)} m`],
           ['Hover battery current', `${I.mean.toFixed(1)} A`], ['Hover motor', `${(r.reduce((a, x) => a + x.motorUs[0] + x.motorUs[1], 0) / (2 * r.length || 1)).toFixed(0)} us`], ['Position drift', `${drift.toFixed(2)} m`]]), { world: w, window: [6, 12] });
      },
    },
    {
      id: 'tv_pitch', name: 'Thrust vectoring: pitch', group: 'Thrust vectoring',
      desc: 'From hover, push pitch forward 30 % for 1.5 s. Both nacelles must tilt forward, the nose must go down and the aircraft must accelerate forward.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, () => { w.pilot.hold = false; w.pilot.override.pitch = 0.3; });
        w.at(7.5, () => { w.pilot.override = {}; });
        w.run(10);
        const r = between(w.record, 6.2, 7.5);
        const tl = stat(r.map((x) => x.tilt[0])).mean, tr = stat(r.map((x) => x.tilt[1])).mean, pitch = stat(r.map((x) => x.pitch)).mean;
        const vx = (r[r.length - 1] || { vel: [0] }).vel[0];
        const maxAtt = stat(between(w.record, 6, 10).flatMap((x) => [x.roll, x.pitch])).max;
        const ok = tl > 0.3 && tr > 0.3 && pitch > 2 && vx > 0.3 && !w.crashed && maxAtt < 45;
        const reversed = pitch < -2 || vx < -0.3;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Pitch thrust vectoring works in the right direction.' : reversed ? 'Pitch response REVERSED.' : w.crashed ? 'Crashed: ' + w.crashReason : 'Pitch response wrong or too weak.',
          [['Mean tilt L / R', `${tl.toFixed(1)}° / ${tr.toFixed(1)}°`], ['Mean body pitch', `${pitch.toFixed(1)}° (+ = nose down)`], ['Forward speed after 1.5 s', `${vx.toFixed(2)} m/s`], ['Max attitude', `${maxAtt.toFixed(1)}°`]],
          ok ? [] : ['Check the "Thrust vectoring: +pitch" item in Checks and the servo direction settings in Hardware.']), { world: w, window: [5, 10] });
      },
    },
    {
      id: 'tv_yaw', name: 'Thrust vectoring: yaw', group: 'Thrust vectoring',
      desc: 'From hover, yaw stick right 50 % for 1.5 s. The left nacelle must tilt forward and the right back, turning the nose right.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, () => { w.pilot.yawHold = false; w.pilot.override.yaw = 0.5; });
        w.at(7.5, () => { w.pilot.override = {}; w.pilot.yawHold = false; });
        w.run(9);
        const r = between(w.record, 6.2, 7.5);
        const diff = Math.max(...between(w.record, 6.0, 6.3).map((x) => x.tilt[0] - x.tilt[1]));
        const y0 = between(w.record, 5.9, 6.0)[0]?.yaw ?? 0, y1 = r[r.length - 1]?.yaw ?? 0;
        let dyaw = y1 - y0; dyaw = ((dyaw + 540) % 360) - 180;
        const rate = stat(r.map((x) => x.rate[2])).mean;
        const maxAtt = stat(between(w.record, 6, 9).flatMap((x) => [x.roll, x.pitch])).max;
        const ok = dyaw < -15 && diff > 0.5 && !w.crashed && maxAtt < 30;
        const reversed = dyaw > 15 && diff < 0;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Yaw thrust vectoring turns the nose the commanded way.' : reversed ? 'Yaw REVERSED: nose turns left for a right command (or spins).' : w.crashed ? 'Crashed: ' + w.crashReason : maxAtt >= 30 ? 'Yaw works but couples into pitch/roll and loses control (gyroscopic coupling of the tilted props + yaw overshoot). Lower yaw rate/P.' : 'Yaw response wrong or too weak.',
          [['Heading change', `${dyaw.toFixed(0)}° (− = nose right)`], ['Mean yaw rate', `${rate.toFixed(0)} °/s`], ['Peak tilt L − R (first 300 ms)', `${diff.toFixed(1)}° (+ = left forward)`], ['Max roll/pitch', `${maxAtt.toFixed(1)}°`]]), { world: w, window: [5, 9] });
      },
    },
    {
      id: 'roll', name: 'Roll by differential thrust', group: 'Thrust vectoring',
      desc: 'Roll stick right 40 % for 0.8 s. The left motor must speed up and the aircraft roll right.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, () => { w.pilot.hold = false; w.pilot.override.roll = 0.4; });
        w.at(6.8, () => { w.pilot.override = {}; });
        w.run(9);
        const r = between(w.record, 6.0, 6.15), r2 = between(w.record, 6.3, 6.8);
        const dm = Math.max(...r.map((x) => x.motorUs[0] - x.motorUs[1])), roll = stat(r2.map((x) => x.roll)).mean;
        const ok = dm > 5 && roll > 5 && !w.crashed;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Roll works: left motor up, right side down.' : roll < -5 ? 'Roll REVERSED.' : 'Roll response wrong or too weak.',
          [['Peak motor L − R (first 150 ms)', `${dm.toFixed(0)} us`], ['Mean roll angle', `${roll.toFixed(1)}° (+ = right down)`]]), { world: w, window: [5, 9] });
      },
    },
    {
      id: 'gust', name: 'Gust rejection', group: 'Flight',
      desc: 'Hover in 3 m/s steady wind with 2 m/s gusts for 8 s.',
      run(hw, fw) {
        const w = makeWorld(hw, fw, { hw: (h) => { h.environment.windX = 3; h.environment.gust = 2; } });
        w.run(14);
        const r = between(w.record, 6, 14), att = stat(r.flatMap((x) => [x.roll, x.pitch])), alt = stat(r.map((x) => x.alt - 1.5));
        const fail = w.crashed || att.max > 35, warn = att.max > 15 || alt.max > 0.8;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', fail ? 'Lost control in wind.' : warn ? 'Holds, with large excursions.' : 'Rejects gusts well.',
          [['Max attitude', `${att.max.toFixed(1)}°`], ['Max altitude error', `${alt.max.toFixed(2)} m`], ['Max tilt used', `${Math.max(...r.map((x) => Math.max(Math.abs(x.tilt[0]), Math.abs(x.tilt[1])))).toFixed(1)}°`]]), { world: w, window: [6, 14] });
      },
    },
    {
      id: 'punch', name: 'Full-throttle punch (overcurrent)', group: 'Power',
      desc: '1.5 s at full throttle from hover: peak motor/ESC/battery current, sag and Betaflight voltage warnings.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, () => { w.pilot.override.throttle = 2000; });
        w.at(7.5, () => { w.pilot.override = {}; w.pilot.alt = w.altitude(); });
        w.run(11);
        const pk = w.peaks, h = w.hw;
        const mI = Math.max(...pk.motorIavg), bI = pk.battIavg, mSpike = Math.max(...pk.motorI);
        const fail = mI > h.esc.burstCurrent || bI > h.battery.capacity * h.battery.cRating || w.crashed;
        const warn = mI > h.motor.maxCurrent || mI > h.esc.contCurrent * 0.85 || pk.minCell < 3.3;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', fail ? 'Overcurrent beyond ratings.' : warn ? 'Exceeds a continuous rating during the punch.' : 'Within all ratings.',
          [['Peak motor/ESC current (50 ms avg)', `${mI.toFixed(1)} A (motor ${h.motor.maxCurrent} A, ESC ${h.esc.contCurrent} A)`], ['Step transient (instantaneous)', `${mSpike.toFixed(0)} A for a few ms while the prop spins up`], ['Peak battery current (50 ms avg)', `${bI.toFixed(1)} A (${(h.battery.capacity * h.battery.cRating).toFixed(0)} A max)`],
           ['Min battery voltage', `${pk.minVbatt.toFixed(2)} V (${pk.minCell.toFixed(2)} V/cell)`], ['Peak rpm', `${Math.max(...pk.rpm).toFixed(0)}`], ['Time above motor rating', `${Math.max(...w.overTime.motorI).toFixed(2)} s`], ['Max climb altitude', `${Math.max(...w.record.map((x) => x.alt)).toFixed(1)} m`]],
          ['The step transient depends on any acceleration limiting in the ESC firmware, which this model does not include; the steady full-throttle current is the reliable figure.']), { world: w, window: [5, 11] });
      },
    },
    {
      id: 'servo_load', name: 'Aggressive roll: servo gyroscopic load', group: 'Structure',
      desc: 'Roll stick reversals at ±60 % at 1.5 Hz for 3 s. Spinning props resist being turned: the tilt servos see the gyroscopic H×ω torque.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, () => { w.pilot.hold = false; });
        for (let k = 0; k < 9; k++) w.at(6 + k / 3, () => { w.pilot.override.roll = k % 2 ? -0.6 : 0.6; });
        w.at(9, () => { w.pilot.override = {}; w.pilot.hold = true; });
        w.run(12);
        const pk = w.peaks, st = w.hw.servo.stallTorque48;
        const ld = Math.max(...pk.servoLoad), over = Math.max(...pk.servoOverload);
        const fail = w.crashed || ld > w.hw.servo.gearStripTorque, warn = ld > st * 0.7 || over > 0;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', fail ? (w.crashed ? 'Crashed: ' + w.crashReason : 'Servo load beyond the gear-strip estimate.') : warn ? 'Servo load near/over stall: nacelles get pushed off target during fast rolls.' : 'Servos keep up.',
          [['Peak servo load', `${(ld * 100 / U.G).toFixed(2)} kg·cm (stall ${(st * 100 / U.G).toFixed(2)} @4.8 V)`], ['Time overpowered', `${over.toFixed(2)} s`], ['Peak roll rate', `${Math.max(...w.record.map((x) => Math.abs(x.rate[0]))).toFixed(0)} °/s`],
           ['Peak arm stress', `${(Math.max(...pk.vonMises) / 1e6).toFixed(0)} MPa`]]), { world: w, window: [5, 12] });
      },
    },
    {
      id: 'servo_rail', name: 'Servo rail under fast stick work', group: 'Power',
      desc: 'Pitch and yaw square waves at 3 Hz: servo current, servo-rail sag, BEC temperature, MCU brown-out.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, () => { w.pilot.hold = false; w.pilot.yawHold = false; });
        for (let k = 0; k < 18; k++) w.at(6 + k / 6, () => { w.pilot.override.pitch = k % 2 ? -0.5 : 0.5; w.pilot.override.yaw = k % 2 ? 0.6 : -0.6; });
        w.at(9, () => { w.pilot.override = {}; w.pilot.hold = true; w.pilot.yawHold = true; w.pilot.yaw = U.qToEuler(w.q)[2]; w.pilot.x = w.cgWorld()[0]; w.pilot.y = w.cgWorld()[1]; });
        w.run(11);
        const pk = w.peaks, r = between(w.record, 6, 9);
        const iS = Math.max(...r.map((x) => x.servoI[0] + x.servoI[1]));
        const brown = w.events.some((e) => /brown-out|thermal/.test(e.msg));
        const fail = brown || pk.minServoV < 4.0, warn = pk.minServoV < 4.6 || pk.minV5 < 4.75;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', brown ? 'Brown-out / BEC shutdown during the test.' : fail ? 'Servo rail collapses.' : warn ? 'Rails sag noticeably.' : 'Rails hold.',
          [['Peak servo current (both)', `${iS.toFixed(2)} A`], ['Min servo rail', `${pk.minServoV.toFixed(2)} V`], ['Min Pico 5 V', `${pk.minV5.toFixed(2)} V`], ['Peak BEC temp', `${pk.maxTemp.bec.toFixed(0)} °C`]]), { world: w, window: [5, 11] });
      },
    },
    {
      id: 'drop100', name: 'RF dropout 100 ms', group: 'Failsafe',
      desc: 'Shorter than the 150 ms RX watchdog: Betaflight must not even see RXLOSS.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(7, (w) => w.radio.dropouts.push({ from: Math.round(w.t * 1e6), to: Math.round((w.t + 0.1) * 1e6) }));
        w.run(10);
        const loss = firstEvent(w, /RXLOSS/, 6.9), armed = w.fc.armed;
        const ok = !loss && armed && !w.crashed;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Bridged by the watchdog; no failsafe.' : 'Unexpected failsafe on a 100 ms gap.',
          [['RXLOSS', loss ? `${loss.toFixed(2)} s` : 'none'], ['Still armed', armed ? 'yes' : 'NO'], ['Hop resync', `${w.receiver.hop.state ? 'synced' : 'searching'}`]]), { world: w, window: [6, 10] });
      },
    },
    {
      id: 'drop400', name: 'RF dropout 400 ms (stage 1)', group: 'Failsafe',
      desc: 'Longer than the watchdog, shorter than stage 1: control must resume without disarming. Measures height lost to the stage-1 rxfail throttle.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(7, (w) => w.radio.dropouts.push({ from: Math.round(w.t * 1e6), to: Math.round((w.t + 0.4) * 1e6) }));
        w.run(11);
        const loss = firstEvent(w, /RXLOSS/, 6.9), back = firstEvent(w, /RX signal OK/, 7.0);
        const altPre = between(w.record, 6.9, 7.0)[0]?.alt ?? 1.5, altMin = Math.min(...between(w.record, 7, 9).map((x) => x.alt));
        const armed = w.fc.armed, stage2 = w.events.some((e) => e.t > 7 && /stage 2/.test(e.msg));
        const fail = !armed || w.crashed || stage2, warn = altPre - altMin > 0.5;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', fail ? (w.crashed ? 'Crashed: ' + w.crashReason : 'Disarmed/stage 2 on a 400 ms gap.') : warn ? `Recovered, but lost ${(altPre - altMin).toFixed(2)} m during stage 1.` : 'Recovered inside stage 1.',
          [['RXLOSS at', loss ? `+${((loss - 7) * 1000).toFixed(0)} ms` : 'none'], ['Signal back at', back ? `+${((back - 7) * 1000).toFixed(0)} ms` : '—'], ['Altitude lost', `${(altPre - altMin).toFixed(2)} m`], ['Still armed', armed ? 'yes' : 'NO']],
          warn ? ['Stage 1 applies rxfail: AUTO throttle = rx_min_usec (idle). See the Stage-1 throttle check.'] : []), { world: w, window: [6, 11] });
      },
    },
    {
      id: 'drop3s', name: 'RF loss 3 s (DROP)', group: 'Failsafe',
      desc: 'Handset off. Expect RXLOSS after ~150 ms, stage 2 after failsafe_delay, then disarm.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(7, (w) => { w.handset.on = false; });
        w.run(11);
        const dis = firstEvent(w, /DISARMED/, 7), loss = firstEvent(w, /RXLOSS/, 7);
        const exp = w.fw.params.rx_frame_timeout_ms / 1000 + w.fw.params.failsafe_delay / 10;
        const lat = dis !== null ? dis - 7 : null;
        const imp = Math.max(...between(w.record, 7, 11).map((x) => Math.hypot(...x.vel)));
        const ok = lat !== null && Math.abs(lat - exp) < 0.1;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? `Disarmed ${lat.toFixed(2)} s after signal loss, as configured.` : 'Failsafe timing not as configured (or no disarm).',
          [['RXLOSS after', loss ? `${((loss - 7) * 1000).toFixed(0)} ms` : '—'], ['Disarm after', lat !== null ? `${lat.toFixed(2)} s (expected ${exp.toFixed(2)})` : 'never'], ['Peak fall speed', `${imp.toFixed(1)} m/s`], ['Crash', w.crashed ? w.crashReason : 'no']]), { world: w, window: [6, 11] });
      },
    },
    {
      id: 'reboot', name: 'Handset reboot in flight', group: 'Failsafe',
      desc: 'The handset resets mid-flight (new session + restart announcement). The receiver must call failsafeOnReceiverReset() and disarm at once, and refuse to re-arm until ARM is cycled.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        let bootAt = 7;
        w.at(7, (w) => { w.handset.boot(Math.round(w.t * 1e6)); });
        w.run(10);
        const dis = firstEvent(w, /DISARMED/, 7);
        const lat = dis !== null ? dis - bootAt : null;
        const rearmed = w.record.some((x) => x.t > (dis || 7) + 0.05 && x.armed);
        const ok = lat !== null && lat < 0.06 && !rearmed;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? `Disarmed ${(lat * 1000).toFixed(0)} ms after the reboot (first restart packet ~20 ms); no re-arm with ARM still on.` : rearmed ? 'Re-armed without an ARM toggle!' : 'Did not disarm promptly.',
          [['Disarm latency', lat !== null ? `${(lat * 1000).toFixed(0)} ms` : 'never'], ['Re-armed with ARM held', rearmed ? 'YES' : 'no'], ['Receiver resets', w.receiver.stats.resets]],
          ['Expected behaviour of protocol v2. The aircraft falls: a handset reboot in flight is a loss of the aircraft by design.']), { world: w, window: [6, 10] });
      },
    },
    {
      id: 'fault', name: 'Handset input-fault burst', group: 'Failsafe',
      desc: '100 ms of TX_INPUT_FAULT packets (e.g. one bad ADC read). Must not disarm or re-engage the interlock.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(7, (w) => { w.handset.faults.inputFaultUntil = Math.round((w.t + 0.1) * 1e6); });
        w.run(10);
        const ok = w.fc.armed && !w.crashed && !w.events.some((e) => e.t > 7 && /DISARM/.test(e.msg));
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Fault packets dropped; control continued.' : 'Disarmed on an input fault.',
          [['Fault packets dropped', w.receiver.stats.txFault], ['RXLOSS', firstEvent(w, /RXLOSS/, 7) ? 'yes' : 'no'], ['Still armed', w.fc.armed ? 'yes' : 'NO']]), { world: w, window: [6, 10] });
      },
    },
    {
      id: 'loss30', name: '30 % random packet loss', group: 'Failsafe',
      desc: 'Hover with 30 % of packets lost (interference). Link quality ~70 %, no RXLOSS, attitude unaffected.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.at(6, (w) => { w.radio.extraLoss = 0.3; });
        w.run(12);
        const r = between(w.record, 7, 12), lq = stat(r.map((x) => x.lq)).mean, att = stat(r.flatMap((x) => [x.roll, x.pitch]));
        const loss = firstEvent(w, /RXLOSS/, 6);
        const ok = !loss && w.fc.armed && att.max < 10;
        return Object.assign(res(ok ? 'pass' : 'fail', ok ? 'Link tolerant of heavy loss.' : 'Loss caused failsafe or instability.',
          [['Mean link quality', `${lq.toFixed(0)} %`], ['RXLOSS', loss ? 'yes' : 'no'], ['Max attitude', `${att.max.toFixed(1)}°`], ['Off-channel / resync', `${w.receiver.stats.offChannel} / ${w.receiver.stats.resync}`]]), { world: w, window: [6, 12] });
      },
    },
    {
      id: 'droptest', name: 'Hard landing (disarm at 0.8 m)', group: 'Structure',
      desc: 'Disarm in hover at 0.8 m and fall onto the gear: impact g-load, arm stress and safety factor.',
      run(hw, fw) {
        const w = makeWorld(hw, fw);
        w.pilot.alt = 0.8;
        w.at(7, (w) => { w.pilot.disabled = true; w.pilot.active = false; w.handset.inputs.aux[0] = 1000; w.handset.inputs.throttle = 1000; });
        w.run(9);
        const pk = w.peaks, y = w.armMat.yield, vm = Math.max(...pk.vonMises), fos = y / Math.max(1, vm);
        const fail = fos < 1.2 || w.crashed, warn = fos < w.hw.frame.targetSafetyFactor;
        return Object.assign(res(fail ? 'fail' : warn ? 'warn' : 'pass', w.crashed ? 'Crash on landing: ' + w.crashReason : fail ? 'Arm would yield.' : warn ? 'Survives with low margin.' : 'Survives with margin.',
          [['Impact', `${pk.contactG.toFixed(1)} g`], ['Peak arm stress (von Mises)', `${(vm / 1e6).toFixed(0)} MPa`], ['Safety factor', fos.toFixed(1)], ['Peak tip deflection', `${(Math.max(...pk.deflection) * 1000).toFixed(1)} mm`]],
          ['Contact stiffness is a model: real impact loads depend on the gear and the surface.']), { world: w, window: [6, 9] });
      },
    },
    {
      id: 'lowbatt', name: 'Low battery behaviour', group: 'Power',
      desc: 'Hover on a nearly empty pack (12 %): sag under load, Betaflight warnings, sensed vs true voltage.',
      run(hw, fw) {
        const w = makeWorld(hw, fw, { hw: (h) => { h.battery.soc0 = 0.12; } });
        w.run(12);
        const warnT = firstEvent(w, /Battery (LOW|CRITICAL)/), r = between(w.record, 6, 12);
        const vTrue = stat(r.map((x) => x.vbat)).mean, vSensed = w.fc.vbatFilt || 0;
        const cell = vTrue / w.hw.battery.cells;
        return Object.assign(res(warnT ? 'pass' : cell < w.fw.params.vbat_warning_cell_voltage / 100 ? 'fail' : 'info', warnT ? 'Betaflight raised the battery warning.' : cell < w.fw.params.vbat_warning_cell_voltage / 100 ? 'Cells are below the warning level but no warning was raised (check vbat_scale / battery_meter).' : 'Pack still above the warning level.',
          [['True pack voltage (hover)', `${vTrue.toFixed(2)} V (${cell.toFixed(2)} V/cell)`], ['Betaflight sensed', `${vSensed.toFixed(2)} V`], ['Warning at', warnT ? `${warnT.toFixed(1)} s` : 'none'], ['Detected cells', w.fc.cells]]), { world: w, window: [2, 12] });
      },
    },
  ];

  async function runAll(hw, fw, onProgress, only) {
    const out = [];
    for (const t of TESTS) {
      if (only && !only.includes(t.id)) continue;
      if (onProgress) onProgress(t, null);
      await new Promise((r) => setTimeout(r, 0));
      let r;
      try { r = t.run(hw, fw); } catch (e) { r = res('fail', 'Test error: ' + e.message, [], [String(e.stack || e)]); }
      r.id = t.id; r.name = t.name; r.group = t.group; r.desc = t.desc;
      out.push(r);
      if (onProgress) onProgress(t, r);
    }
    return out;
  }

  S.Tests = { TESTS, runAll, makeWorld, Pilot };
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
