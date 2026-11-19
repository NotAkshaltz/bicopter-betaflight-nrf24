// World simulation: rigid-body 6-DOF airframe with two tilting rotors, motor/ESC/battery
// electrics, servo actuators with torque/speed limits, power rails (BEC/LDO/brownout),
// thermal models, ground contact and live structural loads. One step = one PID loop.
(function (S) {
  'use strict';
  const U = S.U, H = S.Hardware, PR = S.Protocol, L = S.Link, FW = S.Firmware;

  const parseId = (s) => (typeof s === 'number' ? s : parseInt(String(s), 16)) >>> 0;

  class World {
    constructor(hw, fw, opts = {}) {
      this.hw = hw; this.fw = fw; this.opts = Object.assign({ seed: 1234, noise: true, record: true }, opts);
      this.reset();
    }

    reset() {
      const hw = this.hw, fw = this.fw;
      this.rng = U.rng(this.opts.seed);
      this.dt = 1 / fw.params.pid_loop_hz;
      this.t = 0;
      this.rho = H.airDensity(hw.environment.altitudeAmsl, hw.environment.tempC);
      this.kT = H.Prop.kT(hw.prop, this.rho); this.kQ = H.Prop.kQ(hw.prop, this.rho);
      this.Jr = H.Prop.inertia(hw.prop) + hw.motor.rotorInertia;
      this.kvr = hw.motor.kv * 2 * Math.PI / 60; this.kt = 1 / this.kvr;
      this.mp = H.massProperties(hw, 0, 0);
      this.tipMass = { L: H.tipMass(hw, 'L'), R: H.tipMass(hw, 'R') };
      const sec = H.section(hw.frame.armSection), mat = H.MATERIALS[hw.frame.armMaterial];
      this.armSec = sec; this.armMat = mat;
      const restZ = hw.frame.gearHeight; // tilt axis height when sitting on the gear
      this.pos = [0, 0, restZ + 0.0005 - this.mp.cg[2] * 0];
      this.posRef = 'origin'; // position = body origin (tilt axis midpoint) in world
      this.vel = [0, 0, 0]; this.q = [1, 0, 0, 0]; this.w = [0, 0, 0];
      this.rotor = [0, 0]; this.rotorDot = [0, 0];
      this.tilt = [0, 0]; this.tiltRate = [0, 0]; this.servoLoad = [0, 0]; this.servoI = [0, 0];
      this.motorI = [0, 0]; this.escI = [0, 0]; this.duty = [0, 0]; this.thrust = [0, 0]; this.motorTorque = [0, 0];
      this.temp = { motor: [hw.environment.tempC, hw.environment.tempC], esc: [hw.environment.tempC, hw.environment.tempC], batt: hw.environment.tempC, bec: hw.environment.tempC };
      this.soc = hw.battery.soc0; this.mAh = 0; this.Ibatt = 0; this.Vbatt = H.batteryOcv(hw, this.soc);
      this.rails = { v5: 5, vServo: 5, v3radio: 3.3, v3pico: 3.3, i5: 0, iServo: 0, i3radio: 0, i3pico: 0, radioDip: 0, becPdiss: 0 };
      this.mcuOn = true; this.mcuOffSince = null; this.becShutdown = false;
      this.faults = { servoDead: [false, false], escDead: [false, false], brownUntil: -1 };
      this.crashed = false; this.crashReason = '';
      this.contact = false; this.contactForce = 0; this.peakContactForce = 0;
      this.motorIavg = [0, 0]; this.battIavg = 0;
      this.peaks = { motorIavg: [0, 0], battIavg: 0, armStress: [0, 0], armTorsion: [0, 0], vonMises: [0, 0], deflection: [0, 0], servoLoad: [0, 0], motorI: [0, 0], escI: [0, 0], battI: 0,
                     minVbatt: 99, minCell: 9, minV5: 9, minServoV: 9, minRadioV: 9, maxTemp: { motor: 0, esc: 0, batt: 0, bec: 0 }, contactG: 0, servoOverload: [0, 0], rpm: [0, 0] };
      this.overTime = { motorI: [0, 0], escBurst: [0, 0] };
      this.events = [];
      // firmware + link
      this.fc = new FW.FlightController(fw);
      this.fc.reset(0);
      const linkId = parseId(fw.params.link_id), hsId = parseId(hw.link.linkId);
      this.rfChannels = PR.deriveRfChannels(hsId);
      this.handset = new L.Handset({ linkId: hsId }, this.rng);
      this.radio = new L.RadioChannel(hw.link, this.rng);
      this.receiver = new L.Receiver(linkId);
      this.inflight = [];
      this.record = []; this.recordEvery = 0.01; this.nextRecord = 0;
      this.scenario = null;
      this.mountR = FW.rotEuler(hw.imuMount.roll, hw.imuMount.pitch, hw.imuMount.yaw);
      this.vibPhase = [0, 0];
      this.lastOut = { motorUs: [1000, 1000], servoUs: [1500, 1500] };
      this.trail = [];
      this.nacAngle = [0, 0];
      this.wind = [hw.environment.windX, hw.environment.windY, 0]; this.gust = [0, 0, 0];
    }

    log(msg, level = 'info') { this.events.push({ t: this.t, msg, level }); if (this.events.length > 500) this.events.shift(); }

    cgWorld() { return U.add(this.pos, U.qRot(this.q, this.mp.cg)); }
    altitude() { return this.cgWorld()[2]; }

    // ---------------------------------------------------------------- electrics
    _rails(dt) {
      const hw = this.hw, p = hw.power, ld = hw.loads, R = this.rails;
      const extra = (rail) => p.extraLoads.filter((e) => e.rail === rail).reduce((a, e) => a + (e.avg || 0), 0);
      const servoTotal = this.servoI[0] + this.servoI[1];
      // Pico 3V3(OUT): Pico itself + MPU (if on 3V3) + radio (if on Pico 3V3)
      const radioAvg = ld.radioRxA + ld.radioTxPeakA * ld.radioTxDuty;
      let i3pico = ld.imuA * (p.mpuSupply === 'pico3v3' ? 1 : 0) + ld.ledA + extra('pico3v3') + (p.radio.source === 'pico3v3' ? radioAvg : 0);
      // 5 V rail loads (Pico VSYS draws through its buck-boost: ~ (3.3 V * I) / (0.9 * 5 V))
      let i5 = ld.picoA + (i3pico * 3.3) / (0.9 * 5) + (p.radio.source === 'ldo' ? radioAvg : 0) + (p.mpuSupply === 'fc5v' ? ld.imuA : 0) + extra('fc5v');
      let iServoRail = servoTotal + extra('servo');
      if (p.servo.source === 'fc5v') i5 += iServoRail;
      // FC 5 V source
      const src = p.fc5v;
      let v5 = src.v;
      if (src.type === 'linear') v5 = Math.min(src.v, this.Vbatt - src.dropout);
      if (i5 > src.imax) v5 *= src.imax / i5; // current-limit fold-back
      v5 -= i5 * src.rOut;
      if (this.becShutdown) v5 = 0;
      if (this.faults.brownUntil > this.t) v5 = Math.min(v5, 0.8); // injected supply interruption
      const pd = src.type === 'linear' ? Math.max(0, this.Vbatt - v5) * i5 : i5 * v5 * (1 / (src.eff || 0.88) - 1);
      // BEC thermal (small SMD regulator, ~40 K/W, 3 J/K)
      this.temp.bec += (pd - (this.temp.bec - hw.environment.tempC) / 40) / 3 * dt;
      if (!this.becShutdown && this.temp.bec > 150) { this.becShutdown = true; this.log('5 V BEC thermal shutdown (' + pd.toFixed(1) + ' W dissipated)', 'error'); }
      if (this.becShutdown && this.temp.bec < 120) { this.becShutdown = false; this.log('5 V BEC recovered from thermal shutdown', 'warn'); }
      // servo rail
      let vServo;
      if (p.servo.source === 'fc5v') vServo = v5;
      else {
        vServo = Math.min(p.servo.v, this.Vbatt - 0.5);
        if (iServoRail > p.servo.imax) vServo *= p.servo.imax / iServoRail;
        vServo -= iServoRail * p.servo.rOut;
      }
      // radio 3V3
      let v3radio, v3pico = v5 > 1.9 ? 3.3 : Math.max(0, v5 * 0.9); // Pico buck-boost holds 3.3 V down to ~1.8 V VSYS
      if (i3pico > p.pico3v3Limit) v3pico *= p.pico3v3Limit / i3pico;
      let tResp, capF = (p.radio.capUF + 1) * 1e-6;
      if (p.radio.source === 'ldo') { v3radio = Math.min(p.radio.v, v5 - p.radio.dropout); tResp = 20e-6; }
      else { v3radio = v3pico; tResp = 50e-6; }
      const dip = ld.radioTxPeakA * tResp / capF;
      Object.assign(R, { v5: Math.max(0, v5), vServo: Math.max(0, vServo), v3radio: Math.max(0, v3radio), v3pico: Math.max(0, v3pico),
                         i5, iServo: iServoRail, i3radio: radioAvg, i3pico, radioDip: dip, becPdiss: pd });
      // MCU brown-out / reboot
      if (this.mcuOn && R.v5 < 2.0) { this.mcuOn = false; this.mcuOffSince = this.t; this.log(`MCU brown-out: Pico VSYS ${R.v5.toFixed(2)} V`, 'error'); }
      if (!this.mcuOn && R.v5 > 3.0 && this.t - this.mcuOffSince > 0.05) {
        if (!this._rebootAt) this._rebootAt = this.t + 1.0;
        if (this.t >= this._rebootAt) { this.mcuOn = true; this._rebootAt = null; this.fc.reset(this.t); this.receiver.init(Math.round(this.t * 1e6)); this.log('Pico rebooted after brown-out (arming needs ARM switch toggle)', 'warn'); }
      }
      const pk = this.peaks;
      pk.minV5 = Math.min(pk.minV5, R.v5); pk.minServoV = Math.min(pk.minServoV, R.vServo); pk.minRadioV = Math.min(pk.minRadioV, R.v3radio - dip);
      return R;
    }

    // ---------------------------------------------------------------- servos
    _servo(i, us, dt) {
      const hw = this.hw, sv = hw.servo, R = this.rails;
      const V = R.vServo, f = U.clamp((V - 4.8) / 1.2, -1, 1.5);
      const vScale = U.clamp(V / 4.8, 0, 1.3);
      const tauStall = Math.max(0, (V < 4.8 ? sv.stallTorque48 * U.clamp((V - 3.0) / 1.8, 0, 1) : sv.stallTorque48 + (sv.stallTorque60 - sv.stallTorque48) * f));
      const speed = V < 4.8 ? sv.speed48 / Math.max(0.2, vScale) : sv.speed48 + (sv.speed60 - sv.speed48) * f; // s per 60 deg
      const wNl = (60 * U.DEG) / speed / sv.linkRatio;
      const lim = hw.frame.tiltLimitDeg * U.DEG;
      const dir = i === 0 ? sv.dirL : sv.dirR, trim = (i === 0 ? sv.trimDegL : sv.trimDegR) * U.DEG;
      const powered = this.mcuOn && us > 0 && V > 3.0 && !this.faults.servoDead[i];
      const load = this.servoLoad[i]; // N*m about tilt axis, + tends to increase tilt
      const loadAtServo = load / sv.linkRatio;
      let rate;
      if (!powered) {
        rate = U.clamp(load / 0.02, -wNl, wNl); // limp: back-driven against gear friction
        this.servoI[i] = 0.005;
      } else {
        const cmd = U.clamp(dir * (us - 1500) * sv.degPerUs * U.DEG / sv.linkRatio + trim, -lim, lim);
        const err = cmd - this.tilt[i];
        const db = 0.25 * U.DEG;
        let want = Math.abs(err) < db ? 0 : U.clamp(err * 35, -wNl, wNl);
        const opp = Math.abs(loadAtServo) / Math.max(1e-6, tauStall);
        if (opp >= 1) {
          rate = Math.sign(load) * Math.min(wNl, (opp - 1) * 2.0); // back-driven
          this.peaks.servoOverload[i] += dt;
        } else if (want !== 0 && Math.sign(want) !== Math.sign(load) && load !== 0) {
          rate = U.clamp(want, -wNl * (1 - opp), wNl * (1 - opp));
        } else rate = want;
        const iSt = sv.iStall48 + (sv.iStall60 - sv.iStall48) * U.clamp(f, 0, 1);
        this.servoI[i] = sv.iIdle + iSt * U.clamp(opp + 0.35 * Math.abs(rate) / wNl, 0, 1);
      }
      this.tiltRate[i] = rate;
      this.tilt[i] = U.clamp(this.tilt[i] + rate * dt, -lim, lim);
      if (Math.abs(this.tilt[i]) >= lim - 1e-9) this.tiltRate[i] = 0;
      this.peaks.servoLoad[i] = Math.max(this.peaks.servoLoad[i], Math.abs(loadAtServo));
      if (Math.abs(loadAtServo) > sv.gearStripTorque && !this['_stripLogged' + i]) { this['_stripLogged' + i] = true; this.log(`${i ? 'Right' : 'Left'} servo load ${(loadAtServo * 100 / 9.81).toFixed(1)} kg*cm exceeds gear-strip estimate`, 'error'); }
    }

    // ---------------------------------------------------------------- main step
    step() {
      const hw = this.hw, dt = this.dt, fr = hw.frame;
      this.t += dt;
      const nowUs = Math.round(this.t * 1e6);
      if (this.scenario && this.scenario.update) this.scenario.update(this, this.t);

      // ---- radio link
      const pkt = this.handset.tick(nowUs);
      if (pkt) {
        const pilot = hw.link.pilotPos, dist = U.norm(U.sub(this.cgWorld(), pilot));
        let ok = this.radio.deliver(pkt, nowUs, dist, hw.link.rfPwrBits, this.rfChannels);
        if (ok && this.rails.v3radio - this.rails.radioDip < 1.9 && this.rng() < 0.5) { ok = false; this._radioBrown = (this._radioBrown || 0) + 1; }
        if (ok && this.mcuOn) this.inflight.push({ pkt, at: nowUs + 1100 });
      }
      if (this.mcuOn) {
        this.receiver.tick(nowUs);
        while (this.inflight.length && this.inflight[0].at <= nowUs) this.receiver.onAir(this.inflight.shift().pkt, nowUs, this.fc);
      } else this.inflight = [];

      // ---- sensors (true body rates / specific force -> sensor frame through the physical mount)
      const mountT = [this.mountR[0], this.mountR[3], this.mountR[6], this.mountR[1], this.mountR[4], this.mountR[7], this.mountR[2], this.mountR[5], this.mountR[8]];
      const n = this.opts.noise ? 1 : 0;
      for (let i = 0; i < 2; i++) this.vibPhase[i] += this.rotor[i] * dt;
      const vib = 0.05 * n * (Math.sin(this.vibPhase[0]) * this.rotor[0] + Math.sin(this.vibPhase[1] * 1.01) * this.rotor[1]) / 800;
      const gyroTrue = U.add(this.w, [vib + 0.004 * n * this.rng.gauss(), 0.6 * vib + 0.004 * n * this.rng.gauss(), 0.002 * n * this.rng.gauss()]);
      const sf = this._specificForce || [0, 0, U.G];
      const accTrue = U.add(sf, [0.05 * n * this.rng.gauss() + 3 * vib, 0.05 * n * this.rng.gauss(), 0.05 * n * this.rng.gauss()]);
      const vAdc = U.clamp(this.Vbatt * hw.power.vbatBottom / (hw.power.vbatTop + hw.power.vbatBottom), 0, 3.3);
      let out;
      if (this.mcuOn) {
        out = this.fc.step(this.t, { gyro: U.mat3MulV(mountT, gyroTrue), acc: U.mat3MulV(mountT, accTrue), vbatAdc: vAdc, tiltDeg: this.tilt.map((x) => x * U.RAD) });
        if (this.fc.events.length) { for (const e of this.fc.events) this.events.push(Object.assign({ src: 'fc' }, e)); this.fc.events = []; if (this.events.length > 500) this.events.splice(0, this.events.length - 500); }
      } else out = { motorUs: [0, 0], servoUs: [0, 0] };
      this.lastOut = out;

      // ---- electrics: battery, ESCs, motors
      this._rails(dt);
      const esc = hw.esc, mot = hw.motor;
      const Rb = H.batteryResistance(hw);
      const ocv = H.batteryOcv(hw, this.soc);
      this.Vbatt = Math.max(0, ocv - this.Ibatt * Rb);
      let Itot = 0;
      for (let i = 0; i < 2; i++) {
        const us = out.motorUs[i];
        let duty = us >= esc.calLow ? U.clamp((us - esc.calLow) / (esc.calHigh - esc.calLow), 0, 1) : 0;
        if (this.rotor[i] < 20 && duty < esc.startDuty) duty = 0;
        if (this.crashed && this._propStrike) duty = 0;
        if (this.faults.escDead[i]) duty = 0;
        const vEsc = this.Vbatt - this.escI[i] * H.AWG[hw.power.escLead.awg].r * hw.power.escLead.length * 2;
        const Vm = duty * vEsc;
        let I = (Vm - this.rotor[i] / this.kvr) / (mot.rm + esc.rOn + H.AWG[hw.power.motorLead.awg].r * hw.power.motorLead.length * 2);
        if (duty === 0 || (!esc.dampedLight && I < 0)) I = 0;
        const tauM = this.kt * (I - (this.rotor[i] > 1 ? mot.i0 : 0));
        const tauQ = this.kQ * this.rotor[i] * this.rotor[i];
        let wd = (tauM - tauQ) / this.Jr;
        this.rotor[i] = Math.max(0, this.rotor[i] + wd * dt);
        if (this.rotor[i] === 0) wd = 0;
        this.rotorDot[i] = wd; this.duty[i] = duty; this.motorI[i] = I; this.motorTorque[i] = tauM;
        const iEsc = duty * I + (duty > 0 ? 0.03 : 0.01);
        this.escI[i] = iEsc; Itot += iEsc;
        // thermal
        const flow = 1 + this.rotor[i] / 400;
        const pMot = I * I * mot.rm + mot.i0 * Math.abs(Vm), pEsc = I * I * esc.rOn + 0.015 * Math.abs(Vm * I) + 0.05;
        this.temp.motor[i] += (pMot - (this.temp.motor[i] - hw.environment.tempC) * flow / mot.rth) / mot.cth * dt;
        this.temp.esc[i] += (pEsc - (this.temp.esc[i] - hw.environment.tempC) * flow / esc.rth) / esc.cth * dt;
        // current ratings
        const pk = this.peaks;
        this.motorIavg[i] += (Math.abs(I) - this.motorIavg[i]) * dt / 0.05; pk.motorIavg[i] = Math.max(pk.motorIavg[i], this.motorIavg[i]);
        pk.motorI[i] = Math.max(pk.motorI[i], Math.abs(I)); pk.escI[i] = Math.max(pk.escI[i], Math.abs(I)); pk.rpm[i] = Math.max(pk.rpm[i], this.rotor[i] * 60 / (2 * Math.PI));
        if (Math.abs(I) > mot.maxCurrent) this.overTime.motorI[i] += dt;
        if (Math.abs(I) > esc.contCurrent) this.overTime.escBurst[i] += dt;
      }
      // regulators draw from the battery too
      const pRails = this.rails.v5 * this.rails.i5 / (hw.power.fc5v.type === 'linear' ? this.rails.v5 / Math.max(1, this.Vbatt) : (hw.power.fc5v.eff || 0.88));
      const pServo = hw.power.servo.source === 'ubec' ? this.rails.vServo * this.rails.iServo / (hw.power.servo.eff || 0.88) : 0;
      Itot += (pRails + pServo) / Math.max(1, this.Vbatt) + hw.power.extraLoads.filter((e) => e.rail === 'batt').reduce((a, e) => a + (e.avg || 0), 0);
      this.Ibatt = Itot;
      this.mAh += Itot * dt / 3.6;
      this.soc = Math.max(0, this.soc - Itot * dt / 3600 / hw.battery.capacity);
      this.temp.batt += (Itot * Itot * hw.battery.cells * hw.battery.rCell - (this.temp.batt - hw.environment.tempC) / hw.battery.rth) / hw.battery.cth * dt;
      {
        const pk = this.peaks; pk.battI = Math.max(pk.battI, Itot); this.battIavg += (Itot - this.battIavg) * dt / 0.05; pk.battIavg = Math.max(pk.battIavg, this.battIavg); pk.minVbatt = Math.min(pk.minVbatt, this.Vbatt); pk.minCell = Math.min(pk.minCell, this.Vbatt / hw.battery.cells);
        pk.maxTemp.motor = Math.max(pk.maxTemp.motor, ...this.temp.motor); pk.maxTemp.esc = Math.max(pk.maxTemp.esc, ...this.temp.esc);
        pk.maxTemp.batt = Math.max(pk.maxTemp.batt, this.temp.batt); pk.maxTemp.bec = Math.max(pk.maxTemp.bec, this.temp.bec);
      }

      // ---- servos / nacelles
      for (let i = 0; i < 2; i++) this._servo(i, out.servoUs[i], dt);

      // ---- mass properties (tilting parts move the CG a little)
      if (!this._mpCount || ++this._mpCount > 20) { this.mp = H.massProperties(hw, this.tilt[0], this.tilt[1]); this._mpCount = 1; }
      const m = this.mp.mass, cg = this.mp.cg;

      // ---- aerodynamics and forces (body frame)
      const gWorld = [0, 0, -U.G * m];
      // gusts: first-order random process
      const ge = hw.environment.gust;
      for (let k = 0; k < 3; k++) this.gust[k] += (-this.gust[k] * 0.5 + ge * 1.0 * this.rng.gauss() * Math.sqrt(1 / dt) * 0.3) * dt;
      const windW = U.add(this.wind, this.gust);
      const vAirW = U.sub(this.vel, windW);
      const vAirB = U.qRotInv(this.q, vAirW);
      let F = [0, 0, 0], T = [0, 0, 0];
      const H_rot = [];
      const L_ = fr.armLength, spins = [hw.motor.spinL, hw.motor.spinR];
      const axisB = [], hubB = [];
      for (let i = 0; i < 2; i++) {
        const d = this.tilt[i], side = i === 0 ? 1 : -1;
        const ax = [Math.sin(d), 0, Math.cos(d)];
        const hub = [fr.motorAxisOffsetX * Math.cos(d) + fr.propAboveTilt * Math.sin(d), side * L_, -fr.motorAxisOffsetX * Math.sin(d) + fr.propAboveTilt * Math.cos(d)];
        axisB.push(ax); hubB.push(hub);
        const w = this.rotor[i];
        const nrev = w / (2 * Math.PI);
        const vAx = U.dot(vAirB, ax);
        let Tr = this.kT * w * w * U.clamp(1 - vAx / Math.max(1, nrev * hw.prop.pitch), 0, 1.25);
        // ground effect
        const hubW = U.add(this.pos, U.qRot(this.q, hub));
        const z = Math.max(hw.prop.diameter / 4, hubW[2]), Rp = hw.prop.diameter / 2;
        Tr *= 1 / (1 - Math.pow(Rp / (4 * z), 2));
        this.thrust[i] = Tr;
        const Fi = U.scale(ax, Tr);
        // rotor H-force (in-plane drag) opposing in-plane airflow
        const vIn = U.sub(vAirB, U.scale(ax, vAx));
        const Fh = U.scale(vIn, -2.5e-5 * w);
        const Ftot = U.add(Fi, Fh);
        F = U.add(F, Ftot);
        const r = U.sub(hub, cg);
        T = U.add(T, U.cross(r, Ftot));
        // reaction torque of the motor on the body (opposite to the rotor's spin)
        T = U.add(T, U.scale(ax, -spins[i] * this.motorTorque[i]));
        const Hi = U.scale(ax, spins[i] * this.Jr * w);
        H_rot.push(Hi);
        // gyroscopic: body rotation and nacelle tilt rate turning the rotor momentum
        T = U.add(T, U.scale(U.cross(this.w, Hi), -1));
        T = U.add(T, U.scale(U.cross([0, this.tiltRate[i], 0], Hi), -1));
      }
      // servo load about each tilt axis: +torque increases tilt (thrust toward +x)
      {
        const fB = this._specificForce || [0, 0, U.G];
        for (let i = 0; i < 2; i++) {
          const d = this.tilt[i];
          // thrust offset from the tilt axis
          let tau = this.thrust[i] * fr.motorAxisOffsetX * -1;
          // nacelle mass above the tilt axis behaves like an inverted pendulum in the specific-force field
          const mN = this.mp.nacelleMass, hN = fr.nacelleCgAboveTilt;
          const rN = [hN * Math.sin(d), 0, hN * Math.cos(d)];
          tau += U.cross(rN, U.scale(fB, -mN))[1]; // y component of r x (-m f): + when tilted (tips further)
          // gyroscopic precession: turning the rotor momentum with the body needs (w x H); its y part
          // must come from the servo, so it appears as a load of the opposite sign
          tau -= U.cross(this.w, H_rot[i])[1];
          this.servoLoad[i] = tau;
        }
      }
      // body drag + angular damping
      const vA = U.norm(vAirB);
      F = U.add(F, U.scale(vAirB, -0.5 * this.rho * fr.bodyDragArea * vA));
      T = U.add(T, U.scale(this.w, -0.003));
      // to world
      let Fw = U.add(U.qRot(this.q, F), gWorld);
      let Tb = T;

      // ---- ground contact (gear points + prop tips + arm ends)
      const contacts = [];
      const gx = fr.gearSpanX / 2, gy = fr.gearSpanY / 2, gz = -fr.gearHeight;
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) contacts.push({ p: [sx * gx, sy * gy, gz], k: 4000, c: 40, kind: 'gear' });
      for (let i = 0; i < 2; i++) for (let a = 0; a < 8; a++) {
        const ang = a * Math.PI / 4, Rp = hw.prop.diameter / 2, d = this.tilt[i];
        const ex = [Math.cos(d), 0, -Math.sin(d)], ey = [0, 1, 0];
        contacts.push({ p: U.add(hubB[i], U.add(U.scale(ex, Rp * Math.cos(ang)), U.scale(ey, Rp * Math.sin(ang)))), k: 2500, c: 20, kind: 'prop', rotor: i });
      }
      let cf = 0; this.contact = false;
      for (const c of contacts) {
        const r = U.sub(c.p, cg);
        const pw = U.add(this.pos, U.qRot(this.q, c.p));
        if (pw[2] >= 0) continue;
        const vPoint = U.add(this.vel, U.qRot(this.q, U.cross(this.w, r)));
        if (c.kind === 'prop' && this.rotor[c.rotor] > 50 && !this._propStrike) {
          this._propStrike = true; this.crashed = true; this.crashReason = 'Prop strike (' + (c.rotor ? 'right' : 'left') + ')'; this.log('CRASH: prop strike on the ' + (c.rotor ? 'right' : 'left') + ' rotor', 'error');
        }
        const pen = -pw[2];
        let fn = Math.max(0, c.k * pen - c.c * vPoint[2]);
        const vt = [vPoint[0], vPoint[1], 0], vtn = U.norm(vt);
        const ft = U.scale(vt, -0.7 * fn / (vtn + 0.05));
        const fwc = [ft[0], ft[1], fn];
        Fw = U.add(Fw, fwc);
        Tb = U.add(Tb, U.cross(r, U.qRotInv(this.q, fwc)));
        cf += fn; this.contact = true;
      }
      this.contactForce = cf;
      this.peaks.contactG = Math.max(this.peaks.contactG, cf / (m * U.G));
      const speed = U.norm(this.vel);
      if (this.contact && !this.crashed && this.t > 0.5) {
        const e = U.qToEuler(this.q);
        if (speed > 4.0) { this.crashed = true; this.crashReason = `Hit the ground at ${speed.toFixed(1)} m/s`; this.log('CRASH: ' + this.crashReason, 'error'); }
        else if (Math.abs(e[0]) > 70 * U.DEG || Math.abs(e[1]) > 70 * U.DEG) { this.crashed = true; this.crashReason = 'Tipped over on the ground'; this.log('CRASH: tipped over', 'error'); }
      }

      // ---- integrate
      const acc = U.scale(Fw, 1 / m);
      this.vel = U.add(this.vel, U.scale(acc, dt));
      // integrate the CG, then convert back to the body origin
      const cgW0 = this.cgWorld();
      const cgW = U.add(cgW0, U.scale(this.vel, dt));
      const Iw = U.mat3MulV(this.mp.I, this.w);
      const wdot = U.mat3MulV(this.mp.Iinv, U.sub(Tb, U.cross(this.w, Iw)));
      this.w = U.add(this.w, U.scale(wdot, dt));
      if (U.norm(this.w) > 60) this.w = U.scale(U.unit(this.w), 60);
      this.q = U.qIntegrate(this.q, this.w, dt);
      this.pos = U.sub(cgW, U.qRot(this.q, cg));
      this._specificForce = U.qRotInv(this.q, U.sub(acc, [0, 0, -U.G]));

      // ---- structural loads on each arm (root moment from tip loads)
      this._armLoads(axisB, hubB, H_rot, spins);

      // ---- record
      if (this.opts.record && this.t >= this.nextRecord) { this.nextRecord += this.recordEvery; this._record(); }
    }

    _armLoads(axisB, hubB, H_rot, spins) {
      const hw = this.hw, fr = hw.frame, f = this._specificForce || [0, 0, U.G];
      const free = Math.max(0.01, fr.armLength - fr.hubHalfWidth);
      for (let i = 0; i < 2; i++) {
        const mt = this.tipMass[i === 0 ? 'L' : 'R'];
        // force the arm applies to the tip = m*f - thrust  -> force tip applies to arm = thrust - m*f
        const Ftip = U.sub(U.scale(axisB[i], this.thrust[i]), U.scale(f, mt));
        const side = i === 0 ? 1 : -1;
        const rTip = [0, side * free, 0];
        // torques from the nacelle onto the arm: motor reaction + gyroscopic (taken by the bearings) + servo torque
        const tauReact = U.scale(axisB[i], -spins[i] * this.motorTorque[i]);
        const tauGyro = U.scale(U.cross(this.w, H_rot[i]), -1);
        const M = U.add(U.add(U.cross(rTip, Ftip), tauReact), tauGyro);
        const Mb = Math.hypot(M[0], M[2]), Mt = Math.abs(M[1]) + Math.abs(this.servoLoad[i]);
        const s = this.armSec;
        const sigV = Math.abs(M[0]) * s.cv / s.Iv, sigH = Math.abs(M[2]) * s.ch / s.Ih;
        // round sections: resultant moment; rectangular: worst corner (both stresses add)
        const sig = s.Iv === s.Ih && this.hw.frame.armSection.type.startsWith('round') ? Mb * s.cv / s.Iv : sigV + sigH;
        const tau = Mt * s.ct / s.J;
        const vm = Math.sqrt(sig * sig + 3 * tau * tau);
        const defl = Math.hypot(Ftip[0], Ftip[2]) * free ** 3 / (3 * this.armMat.E * s.Iv);
        const pk = this.peaks;
        pk.armStress[i] = Math.max(pk.armStress[i], sig); pk.armTorsion[i] = Math.max(pk.armTorsion[i], tau);
        pk.vonMises[i] = Math.max(pk.vonMises[i], vm); pk.deflection[i] = Math.max(pk.deflection[i], defl);
        this['armVm' + i] = vm;
      }
    }

    _record() {
      const e = U.qToEuler(this.q).map((x) => x * U.RAD), fc = this.fc;
      this.record.push({
        t: this.t, alt: this.altitude(), pos: this.cgWorld(), vel: this.vel.slice(),
        roll: e[0], pitch: e[1], yaw: e[2],
        rate: this.w.map((x) => x * U.RAD), sp: fc.setpoint.slice(), estAtt: fc.att.slice(),
        motorUs: this.lastOut.motorUs.slice(), servoUs: this.lastOut.servoUs.slice(), tilt: this.tilt.map((x) => x * U.RAD),
        rpm: this.rotor.map((x) => x * 60 / (2 * Math.PI)), thrust: this.thrust.slice(), motorI: this.motorI.slice(), Ibatt: this.Ibatt,
        vbat: this.Vbatt, v5: this.rails.v5, vServo: this.rails.vServo, servoLoad: this.servoLoad.slice(), servoI: this.servoI.slice(),
        tempMotor: this.temp.motor.slice(), tempEsc: this.temp.esc.slice(), lq: this.receiver.linkQuality(Math.round(this.t * 1e6)), prx: this.radio.last.prx,
        armed: fc.armed, fs: fc.fs.phase, rxOk: fc.rxSignal, pid: fc.pid.map((p) => p.sum), armVm: [this.armVm0 || 0, this.armVm1 || 0], throttle: fc.throttle,
        mcuOn: this.mcuOn,
      });
      if (this.record.length > 200000) this.record.splice(0, 50000);
      if (this.trail.length === 0 || U.norm(U.sub(this.trail[this.trail.length - 1], this.cgWorld())) > 0.05) { this.trail.push(this.cgWorld()); if (this.trail.length > 600) this.trail.shift(); }
    }

    run(seconds) { const n = Math.round(seconds / this.dt); for (let i = 0; i < n; i++) this.step(); }
  }

  S.Physics = { World, parseId };
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
