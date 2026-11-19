// Static engineering checks. Each check returns
//   {name, value, status: 'pass'|'warn'|'fail'|'info', detail, fix}
// grouped into sections. Nothing here runs the time simulation (see tests.js for that).
(function (S) {
  'use strict';
  const U = S.U, H = S.Hardware, FW = S.Firmware, PR = S.Protocol;

  const item = (name, value, status, detail = '', fix = '') => ({ name, value, status, detail, fix });
  const pin = (s) => (s && /^A\d+$/.test(s) ? Number(s.slice(1)) : null);

  // RP2350 SPI divider: prescale even 2..254, postdiv 1..256, result rounds DOWN to <= request.
  function spiActualHz(reqHz, clk = 150e6) {
    let best = 0;
    for (let pre = 2; pre <= 254; pre += 2) {
      const post = Math.ceil(clk / (pre * reqHz));
      if (post < 1 || post > 256) continue;
      const f = clk / (pre * post);
      if (f <= reqHz && f > best) best = f;
    }
    return best;
  }

  // Physical response of the nacelles to the firmware's servo mixer for a unit stabilised input.
  function tiltResponse(hw, fw, inputIdx) {
    const p = fw.params;
    const rules = p.mixer === 'CUSTOM' ? fw.smix : FW.BI_SMIX;
    const out = [0, 0];
    [4, 5].forEach((target, k) => {
      const s = fw.servos[target];
      let v = 0;
      for (const r of rules) if (r.target === target && r.input === inputIdx) v += (s.reversed.includes(r.input) ? -1 : 1) * r.rate / 100;
      const us = s.rate / 100 * v * 100;       // us per 100 units of input
      const dir = k === 0 ? hw.servo.dirL : hw.servo.dirR;
      out[k] = dir * us * hw.servo.degPerUs / hw.servo.linkRatio; // deg of nacelle tilt per 100 input units
    });
    return out;
  }

  function hoverPoint(hw, fw) {
    const mp = H.massProperties(hw), rho = H.airDensity(hw.environment.altitudeAmsl, hw.environment.tempC);
    const W = mp.mass * U.G, Vb = hw.battery.cells * 3.75 - 6 * H.batteryResistance(hw);
    let lo = 0, hi = 1;
    for (let i = 0; i < 40; i++) { const d = (lo + hi) / 2; if (2 * H.motorSteady(hw, d, Vb, rho).thrust > W) hi = d; else lo = d; }
    const hov = H.motorSteady(hw, hi, Vb, rho);
    return { mp, rho, W, duty: hi, hov, Vb };
  }

  function analyze(hw, fw) {
    const sections = [];
    const p = fw.params, fr = hw.frame;
    const { mp, rho, W, duty: hoverDuty, hov } = hoverPoint(hw, fw);
    const cg = mp.cg, L = fr.armLength;
    const Rb = H.batteryResistance(hw);

    // ---------------------------------------------------------------- weight & balance
    {
      const it = [];
      it.push(item('All-up weight', `${(mp.mass * 1000).toFixed(0)} g`, 'info', `${hw.components.length} components. Weigh the real parts and edit them in the Hardware tab.`));
      const latPct = Math.abs(cg[1]) / L * 100;
      it.push(item('Lateral CG offset', `${(cg[1] * 1000).toFixed(1)} mm (${latPct.toFixed(1)} % of arm)`, latPct > 8 ? 'fail' : latPct > 2 ? 'warn' : 'pass',
        `Holding it level needs ${(latPct).toFixed(1)} % more thrust on the ${cg[1] > 0 ? 'left' : 'right'} motor, taken from the roll authority.`,
        latPct > 2 ? `Move ${(Math.abs(cg[1]) * mp.mass / 0.185 * 1000).toFixed(0)} mm of battery (or equivalent mass) toward the ${cg[1] > 0 ? 'right' : 'left'}.` : ''));
      const h = -cg[2];
      if (h <= 0.002) {
        it.push(item('CG height vs tilt axis', `${(h * 1000).toFixed(1)} mm below`, 'fail',
          'Pitch is controlled by tilting the thrust line around the CG. With the CG at or above the tilt axis the pitch moment is ~zero or reversed: the pitch loop has no authority or positive feedback.',
          'Mount the battery lower (hang it under the frame) so the CG sits at least 20-40 mm below the tilt axis.'));
      } else {
        const theta = Math.atan2(cg[0], h) * U.RAD;
        it.push(item('Longitudinal CG offset', `${(cg[0] * 1000).toFixed(1)} mm ${cg[0] >= 0 ? 'ahead of' : 'behind'} the tilt axis`, Math.abs(theta) > 10 ? 'fail' : Math.abs(theta) > 3 ? 'warn' : 'pass',
          `Hover equilibrium needs the thrust line through the CG: the body settles ${Math.abs(theta).toFixed(1)}° nose-${cg[0] > 0 ? 'down' : 'up'} with nacelles tilted ${Math.abs(theta).toFixed(1)}° the other way. In angle mode (level demanded) it drifts ${cg[0] > 0 ? 'forward' : 'backward'} until the I-term trims it.`,
          Math.abs(theta) > 3 ? `Move the battery ${(Math.abs(cg[0]) * mp.mass / 0.185 * 1000).toFixed(0)} mm ${cg[0] > 0 ? 'backward' : 'forward'}.` : ''));
        const lim = Math.min(fr.tiltLimitDeg, 20) * U.DEG;
        const alpha = 2 * (W / 2) * Math.sin(lim) * h / mp.I[4] * U.RAD;
        it.push(item('Pitch authority', `${alpha.toFixed(0)} °/s² at ${(lim * U.RAD).toFixed(0)}° tilt`, alpha < 300 ? 'warn' : 'pass',
          `Pitch moment = thrust × sin(tilt) × CG depth (${(h * 1000).toFixed(0)} mm). Compare roll authority ≈ ${(2 * 0.25 * W * L / mp.I[0] * U.RAD).toFixed(0)} °/s² at ±25 % differential thrust.`,
          alpha < 300 ? 'Lower the CG (battery further below the tilt axis) for more pitch authority.' : ''));
      }
      it.push(item('Inertia Ixx / Iyy / Izz', `${(mp.I[0] * 1e3).toFixed(2)} / ${(mp.I[4] * 1e3).toFixed(2)} / ${(mp.I[8] * 1e3).toFixed(2)} g·m²`, 'info', 'Roll and yaw inertia are dominated by the nacelles at the arm tips.'));
      const tauPend = mp.nacelleMass * U.G * fr.nacelleCgAboveTilt * Math.sin(fr.tiltLimitDeg * U.DEG);
      it.push(item('Nacelle balance (tilting mass above the axis)', `${(fr.nacelleCgAboveTilt * 1000).toFixed(0)} mm, ${(tauPend * 100 / U.G).toFixed(2)} kg·cm at the stop`, tauPend > 0.3 * hw.servo.stallTorque48 ? 'warn' : 'pass',
        'Mass above the tilt axis acts like an inverted pendulum the servo must hold, worse under g-load.', tauPend > 0.3 * hw.servo.stallTorque48 ? 'Lower the motor mount so the tilting parts balance on the axis.' : ''));
      sections.push({ id: 'balance', title: 'Weight & balance', items: it });
    }

    // ---------------------------------------------------------------- propulsion
    {
      const it = [];
      const Vfull = hw.battery.cells * 4.2;
      // full-throttle point with battery sag (iterate)
      let Vb = Vfull, full;
      for (let k = 0; k < 6; k++) { full = H.motorSteady(hw, p.motor_output_limit / 100, Vb, rho); Vb = Vfull - 2 * full.battCurrent * Rb; }
      const Vnom = hw.battery.cells * 3.7;
      let Vb2 = Vnom, fullN;
      for (let k = 0; k < 6; k++) { fullN = H.motorSteady(hw, p.motor_output_limit / 100, Vb2, rho); Vb2 = Vnom - 2 * fullN.battCurrent * Rb; }
      const twr = 2 * fullN.thrust / W;
      it.push(item('Thrust-to-weight (3.7 V/cell, loaded)', twr.toFixed(2), twr < 1.4 ? 'fail' : twr < 2 ? 'warn' : 'pass',
        `Max thrust ${(2 * fullN.thrust / U.G * 1000).toFixed(0)} g vs weight ${(mp.mass * 1000).toFixed(0)} g. Under 2:1 leaves little margin for control and gusts.`));
      it.push(item('Hover throttle', `${(hoverDuty * 100).toFixed(0)} % duty, ${hov.rpm.toFixed(0)} rpm`, hoverDuty > 0.7 ? 'fail' : hoverDuty > 0.55 ? 'warn' : 'pass',
        `Hover: ${(hov.current).toFixed(1)} A per motor, ${(2 * hov.battCurrent).toFixed(1)} A from the battery, ${(2 * hov.thrust / U.G / (2 * hov.pElec) * 1000).toFixed(1)} g/W.`));
      const motOver = full.current > hw.motor.maxCurrent;
      // output limit that keeps the motor at its rating
      let lo = 0, hi = 1; for (let i = 0; i < 30; i++) { const d = (lo + hi) / 2; if (H.motorSteady(hw, d, Vfull - 2 * 10 * Rb, rho).current > hw.motor.maxCurrent) hi = d; else lo = d; }
      it.push(item('Motor current at full throttle (fresh pack)', `${full.current.toFixed(1)} A (rating ${hw.motor.maxCurrent} A / ${hw.motor.maxCurrentTime} s)`, motOver ? (full.current > 1.4 * hw.motor.maxCurrent ? 'fail' : 'warn') : 'pass',
        `${hw.prop.name} on ${hw.battery.cells}S at full throttle. Short bursts are tolerated; sustained full throttle overheats the windings and magnets.`,
        motOver ? `set motor_output_limit = ${Math.floor(lo * 100)} — keeps the motor ≤ ${hw.motor.maxCurrent} A; or fit a 9x4.7 / 8x4.5 prop.` : ''));
      it.push(item('ESC current at full throttle', `${full.current.toFixed(1)} A of ${hw.esc.contCurrent} A continuous`, full.current > hw.esc.burstCurrent ? 'fail' : full.current > hw.esc.contCurrent ? 'warn' : full.current > 0.85 * hw.esc.contCurrent ? 'warn' : 'pass',
        'SimonK has no current limiting: overload ends in burnt FETs.'));
      const battMax = 2 * full.battCurrent + 0.5;
      const battRating = hw.battery.capacity * hw.battery.cRating;
      it.push(item('Battery current at full throttle', `${battMax.toFixed(1)} A of ${battRating.toFixed(0)} A (${hw.battery.cRating}C)`, battMax > battRating ? 'fail' : battMax > 0.7 * battRating ? 'warn' : 'pass',
        `Sag at full throttle: ${(Vb / hw.battery.cells).toFixed(2)} V/cell fresh, ${(Vb2 / hw.battery.cells).toFixed(2)} V/cell at nominal (R pack+leads ${(Rb * 1000).toFixed(0)} mΩ).`));
      const cellNomSag = Vb2 / hw.battery.cells;
      it.push(item('Voltage sag vs warning', `${cellNomSag.toFixed(2)} V/cell under full load at 50 % charge`, cellNomSag < p.vbat_min_cell_voltage / 100 ? 'warn' : 'pass',
        `Betaflight warns at ${p.vbat_warning_cell_voltage / 100} V/cell and is critical at ${p.vbat_min_cell_voltage / 100} V/cell. Punches mid-pack will trigger them briefly.`));
      const usable = hw.battery.capacity * 0.8, hoverA = 2 * hov.battCurrent + 0.3;
      it.push(item('Hover endurance (80 % of capacity)', `${(usable / hoverA * 60).toFixed(1)} min`, 'info', `${hoverA.toFixed(1)} A average including electronics.`));
      const tip = Math.PI * hw.prop.diameter * full.rpm / 60, mach = tip / 343;
      it.push(item('Prop tip speed at full rpm', `${tip.toFixed(0)} m/s (Mach ${mach.toFixed(2)})`, mach > 0.6 ? 'fail' : mach > 0.5 ? 'warn' : 'pass', `${full.rpm.toFixed(0)} rpm max.`));
      const tMot = hw.environment.tempC + (hov.current ** 2 * hw.motor.rm + hw.motor.i0 * hov.current * hw.motor.rm * 0 + hw.motor.i0 * hoverDuty * Vnom) * hw.motor.rth / (1 + hov.w / 400);
      it.push(item('Motor temperature, steady hover', `${tMot.toFixed(0)} °C`, tMot > hw.motor.tmax ? 'fail' : tMot > hw.motor.tmax - 25 ? 'warn' : 'pass', 'Copper + iron loss with prop-wash cooling (rough model).'));
      // protocol / ESC compatibility
      const proto = p.motor_pwm_protocol;
      it.push(item('Motor protocol vs ESC', `${proto} → ${hw.esc.name}`, proto.startsWith('DSHOT') ? 'fail' : proto === 'ONESHOT125' ? 'warn' : 'pass',
        proto.startsWith('DSHOT') ? 'SimonK firmware does not decode DShot; the motors will not start.' : proto === 'ONESHOT125' ? 'SimonK does not officially support OneShot125; behaviour depends on the build.' : 'Standard 1000-2000 us PWM.',
        proto !== 'PWM' ? 'set motor_pwm_protocol = PWM' : ''));
      const rate = Math.min(480, p.motor_pwm_rate), period = 1e6 / rate;
      it.push(item('Motor PWM rate', `${p.motor_pwm_rate} Hz${p.motor_pwm_rate > 480 ? ' (driver caps at 480)' : ''}, period ${period.toFixed(0)} us`,
        period < 2100 ? 'fail' : p.motor_pwm_rate > hw.esc.maxRateHz ? 'fail' : p.motor_pwm_rate > 400 ? 'warn' : 'pass',
        `A 2000 us pulse needs a period > 2000 us plus gap. SimonK accepts up to ~${hw.esc.maxRateHz} Hz. Higher rate = lower output latency (${(period / 1000).toFixed(1)} ms now).`,
        period < 2100 ? 'set motor_pwm_rate = 400' : ''));
      const idleUs = 1000 + p.motor_idle / 10, highUs = 1000 + 10 * p.motor_output_limit;
      const idleDuty = (idleUs - hw.esc.calLow) / (hw.esc.calHigh - hw.esc.calLow);
      it.push(item('Disarmed pulse vs ESC calibration', `min_command ${p.min_command} us, ESC stop ≤ ${hw.esc.calLow} us`, p.min_command > hw.esc.calLow + 5 ? 'fail' : 'pass',
        p.min_command > hw.esc.calLow + 5 ? 'Motors would spin while DISARMED.' : 'Motors stay stopped while disarmed.', p.min_command > hw.esc.calLow ? `set min_command = ${hw.esc.calLow}` : ''));
      it.push(item('Idle vs ESC start threshold', `idle ${idleUs.toFixed(0)} us = ${(idleDuty * 100).toFixed(1)} % ESC duty`, idleDuty < hw.esc.startDuty ? 'warn' : 'pass',
        idleDuty < hw.esc.startDuty ? 'Motors stop at idle when armed: no air-mode authority at zero throttle and a restart delay.' : 'Motors keep spinning at idle when armed.',
        idleDuty < hw.esc.startDuty ? `Raise motor_idle to ${Math.ceil((hw.esc.calLow + hw.esc.startDuty * 1.3 * (hw.esc.calHigh - hw.esc.calLow) - 1000) * 10)} or recalibrate the ESCs.` : ''));
      it.push(item('Max output vs ESC full scale', `${highUs.toFixed(0)} us vs ESC full ${hw.esc.calHigh} us`, highUs < hw.esc.calHigh - 50 ? 'info' : 'pass',
        highUs < hw.esc.calHigh - 50 ? 'Full ESC power is not reachable (fine if intentional via motor_output_limit).' : ''));
      it.push(item('Prop rotation', `left ${hw.motor.spinL > 0 ? 'CCW' : 'CW'}, right ${hw.motor.spinR > 0 ? 'CCW' : 'CW'} (from above)`, hw.motor.spinL === hw.motor.spinR ? 'fail' : 'pass',
        hw.motor.spinL === hw.motor.spinR ? 'Same direction: the reaction torques add and the tilt servos must hold a constant yaw correction.' : 'Counter-rotating: reaction torques cancel in hover.'));
      sections.push({ id: 'propulsion', title: 'Propulsion & battery', items: it, data: { full, fullN, hov, hoverDuty } });
    }

    // ---------------------------------------------------------------- power distribution
    {
      const it = [], pw = hw.power, ld = hw.loads;
      const fullI = sections[1].data.full.current, battI = 2 * sections[1].data.full.battCurrent + 0.5;
      const wire = (label, w, I) => {
        const a = H.AWG[w.awg], con = H.CONNECTORS[w.connector] || H.CONNECTORS.Soldered;
        const drop = I * (a.r * w.length * 2 + con.r);
        it.push(item(`${label}: ${w.awg} AWG, ${w.connector}`, `${I.toFixed(1)} A of ${a.amp} A wire / ${con.rating} A connector`, I > a.amp * 1.3 || I > con.rating ? 'fail' : I > a.amp ? 'warn' : 'pass',
          `Drop ${(drop * 1000).toFixed(0)} mV, ${(I * I * (a.r * w.length * 2 + con.r)).toFixed(2)} W heat at full throttle.`, I > a.amp ? `Use ${Object.keys(H.AWG).map(Number).filter((g) => H.AWG[g].amp >= I).sort((x, y) => y - x)[0] || 10} AWG or heavier.` : ''));
      };
      wire('Battery lead', pw.battLead, battI);
      wire('ESC power leads', pw.escLead, sections[1].data.full.battCurrent);
      wire('Motor phase wires', pw.motorLead, fullI * 0.82);
      const servoPeak = 2 * hw.servo.iStall48 * (pw.servo.v >= 6 ? hw.servo.iStall60 / hw.servo.iStall48 : 1);
      const extra = (rail, key = 'peak') => pw.extraLoads.filter((e) => e.rail === rail).reduce((a, e) => a + (e[key] || e.avg || 0), 0);
      const radioAvg = ld.radioRxA + ld.radioTxPeakA * ld.radioTxDuty;
      const i3 = (pw.mpuSupply === 'pico3v3' ? ld.imuA : 0) + ld.ledA + (pw.radio.source === 'pico3v3' ? ld.radioTxPeakA : 0) + extra('pico3v3');
      let i5 = ld.picoA + i3 * 3.3 / (0.9 * 5) + (pw.radio.source === 'ldo' ? ld.radioTxPeakA : 0) + extra('fc5v');
      if (pw.servo.source === 'fc5v') i5 += servoPeak + extra('servo');
      const src = pw.fc5v;
      it.push(item(`FC 5 V rail (${src.source === 'escbec' ? 'ESC BEC' : src.source}, ${src.type})`, `${(i5 * 1000).toFixed(0)} mA peak of ${(src.imax * 1000).toFixed(0)} mA`, i5 > src.imax ? 'fail' : i5 > 0.7 * src.imax ? 'warn' : 'pass',
        'Pico VSYS, the radio regulator and any extra 5 V loads.' + (pw.servo.source === 'fc5v' ? ' Servos are on this rail too.' : '')));
      if (src.type === 'linear') {
        const iAvg = ld.picoA + i3 * 0.7 + (pw.radio.source === 'ldo' ? radioAvg : 0) + extra('fc5v', 'avg') + (pw.servo.source === 'fc5v' ? 0.3 : 0);
        const pd = (hw.battery.cells * 4.2 - src.v) * iAvg;
        it.push(item('Linear BEC dissipation (full pack)', `${pd.toFixed(2)} W`, pd > 2 ? 'fail' : pd > 1 ? 'warn' : 'pass',
          `(${(hw.battery.cells * 4.2).toFixed(1)} V − ${src.v} V) × ${(iAvg * 1000).toFixed(0)} mA average. Small ESC linear BECs overheat above ~1-2 W and shut down, rebooting the Pico in flight.`,
          pd > 1 ? 'Feed the Pico from a switching 5 V regulator on the PDB instead of the ESC BEC.' : ''));
      }
      if (pw.servo.source === 'fc5v') it.push(item('Servo supply', 'shared with the Pico 5 V rail', 'fail', 'Servo stall currents and motor noise go straight into VSYS: risk of brown-out resets in flight. The project plan is a separate servo supply.', 'Power the servos from their own UBEC (common ground, signal only from the Pico).'));
      else it.push(item('Servo UBEC', `${servoPeak.toFixed(2)} A both stalled of ${pw.servo.imax} A`, servoPeak > pw.servo.imax ? 'fail' : servoPeak > 0.8 * pw.servo.imax ? 'warn' : 'pass', `${pw.servo.v} V. MG90S stall ~${hw.servo.iStall48} A at 4.8 V, ~${hw.servo.iStall60} A at 6 V.`));
      it.push(item('Pico 3V3(OUT) load', `${(i3 * 1000).toFixed(0)} mA peak of ${(pw.pico3v3Limit * 1000).toFixed(0)} mA`, i3 > pw.pico3v3Limit ? 'fail' : i3 > 0.6 * pw.pico3v3Limit ? 'warn' : 'pass',
        pw.radio.source === 'pico3v3' ? 'The PA/LNA radio is powered from the Pico 3V3 pin; its TX peaks load the Pico regulator.' : 'MPU6500 and small loads only.'));
      const capF = (pw.radio.capUF + 1) * 1e-6, tResp = pw.radio.source === 'ldo' ? 20e-6 : 50e-6, dip = ld.radioTxPeakA * tResp / capF;
      it.push(item('Radio supply dip at TX/ACK peaks', `${(dip * 1000).toFixed(0)} mV with ${pw.radio.capUF} µF`, dip > 0.8 ? 'fail' : dip > 0.3 ? 'warn' : 'pass',
        `PA/LNA modules draw ~${(ld.radioTxPeakA * 1000).toFixed(0)} mA when transmitting (auto-ACK). Undersized decoupling causes resets and lost packets.`,
        dip > 0.3 ? 'Fit 10-100 µF low-ESR right at the module VCC/GND pins.' : ''));
      if (pw.radio.source === 'ldo') {
        const head = src.v - pw.radio.dropout;
        it.push(item('Radio LDO headroom', `${head.toFixed(2)} V in for 3.3 V out`, head < 3.3 ? 'fail' : 'pass', `Dropout ${pw.radio.dropout} V (AMS1117 ≈ 1.1 V).`, head < 3.3 ? 'Use a low-dropout regulator (e.g. MCP1700 / AP2112) or a 5 V input.' : ''));
      }
      it.push(item('MPU6500 breakout supply', `${pw.mpuBreakout === 'ldo' ? 'breakout has regulator' : 'bare 3.3 V part'}, fed from ${pw.mpuSupply === 'pico3v3' ? 'Pico 3V3' : '5 V'}`,
        pw.mpuBreakout === 'bare' && pw.mpuSupply === 'fc5v' ? 'fail' : pw.mpuBreakout === 'ldo' && pw.mpuSupply === 'pico3v3' ? 'warn' : 'pass',
        pw.mpuBreakout === 'bare' && pw.mpuSupply === 'fc5v' ? '5 V on a 3.3 V-only MPU6500 destroys it.' : pw.mpuBreakout === 'ldo' && pw.mpuSupply === 'pico3v3' ? 'A breakout regulator fed with 3.3 V may drop the sensor to ~3.0 V; usually works but check the board.' : 'OK.'));
      it.push(item('Common ground', pw.commonGround ? 'yes' : 'NO', pw.commonGround ? 'pass' : 'fail', pw.commonGround ? 'All supplies share ground with the Pico.' : 'PWM signals have no reference: servos/ESCs will jitter or ignore commands.', pw.commonGround ? '' : 'Tie all supply grounds to the Pico GND.'));
      // VBAT divider
      const ratio = pw.vbatBottom / (pw.vbatTop + pw.vbatBottom), vMax = hw.battery.cells * 4.35, adcMax = vMax * ratio;
      const scaleNeeded = Math.round(10 / ratio);
      it.push(item('VBAT divider (GP26)', `${(adcMax).toFixed(2)} V at ${vMax.toFixed(1)} V pack`, adcMax > 3.3 ? 'fail' : adcMax < 0.8 ? 'warn' : 'pass',
        `Divider ${pw.vbatTop / 1000}k / ${pw.vbatBottom / 1000}k = 1:${(1 / ratio).toFixed(1)}; ${(1 / ratio * 3.3 / 4096 * 1000).toFixed(1)} mV per ADC step.`, adcMax > 3.3 ? 'Increase the top resistor: the ADC pin is not 5 V tolerant above 3.3 V.' : ''));
      if (p.battery_meter === 'ADC') it.push(item('vbat_scale vs divider', `${p.vbat_scale} (divider needs ${scaleNeeded})`, Math.abs(p.vbat_scale - scaleNeeded) / scaleNeeded > 0.03 ? 'warn' : 'pass',
        `Betaflight would read ${(hw.battery.cells * 3.8 * ratio * p.vbat_scale / 10).toFixed(2)} V for a real ${(hw.battery.cells * 3.8).toFixed(2)} V.`, Math.abs(p.vbat_scale - scaleNeeded) / scaleNeeded > 0.03 ? `set vbat_scale = ${scaleNeeded}` : ''));
      it.push(item('PWM signal levels', '3.3 V logic to ESC and servo inputs', 'info', 'SimonK and MG90S normally accept 3.3 V PWM; confirm on the bench with the ESC beeps/servo response.'));
      sections.push({ id: 'power', title: 'Power distribution & electronics', items: it });
    }

    // ---------------------------------------------------------------- pins & buses
    {
      const it = [], res = fw.resources;
      const used = {};
      for (const [k, v] of Object.entries(res)) { const g = pin(v); if (g === null) continue; (used[g] = used[g] || []).push(k); }
      const dups = Object.entries(used).filter(([, ks]) => ks.length > 1);
      it.push(item('Pin conflicts', dups.length ? dups.map(([g, ks]) => `GP${g}: ${ks.join(' + ')}`).join('; ') : 'none', dups.length ? 'fail' : 'pass', 'Each GPIO can carry one function.'));
      const reserved = { 23: 'Pico SMPS mode', 24: 'VBUS sense', 29: 'VSYS/3 ADC' };
      const resHits = Object.keys(used).filter((g) => reserved[g]);
      it.push(item('Pico 2 internal pins', resHits.length ? resHits.map((g) => `GP${g} (${reserved[g]})`).join(', ') : 'not used', resHits.length ? 'fail' : 'pass', 'GP23/24/29 are wired on the Pico 2 board (GP25 = LED).'));
      const SPI = { 0: { sck: [2, 6, 18, 22], tx: [3, 7, 19], rx: [0, 4, 16, 20] }, 1: { sck: [10, 14, 26], tx: [11, 15, 27], rx: [8, 12, 28] } };
      for (const bfBus of [1, 2]) {
        const hwBus = bfBus - 1, sck = pin(res[`SPI_SCK ${bfBus}`]), sdo = pin(res[`SPI_SDO ${bfBus}`]), sdi = pin(res[`SPI_SDI ${bfBus}`]);
        if (sck === null && sdo === null && sdi === null) continue;
        const ok = SPI[hwBus].sck.includes(sck) && SPI[hwBus].tx.includes(sdo) && SPI[hwBus].rx.includes(sdi);
        it.push(item(`Betaflight SPI${bfBus} = RP2350 SPI${hwBus} pins`, `SCK GP${sck}, SDO GP${sdo}, SDI GP${sdi}`, ok ? 'pass' : 'fail',
          `Valid: SCK ${SPI[hwBus].sck.map((x) => 'GP' + x).join('/')}, SDO ${SPI[hwBus].tx.map((x) => 'GP' + x).join('/')}, SDI ${SPI[hwBus].rx.map((x) => 'GP' + x).join('/')}.`));
      }
      it.push(item('Bus assignment', `gyro on SPI${p.gyro_1_spibus}, NRF24 on SPI${p.rx_spi_bus}`, p.gyro_1_spibus === p.rx_spi_bus ? 'warn' : (p.rx_spi_bus === 2 ? 'pass' : 'warn'),
        p.gyro_1_spibus === p.rx_spi_bus ? 'Sharing one bus between the gyro and the radio adds latency and CS contention.' : 'rx_spi_bus must read 2 (SPI1) on this target, per the handoff.'));
      const slice = (g) => (g >> 1) & 7;
      const outs = ['MOTOR 1', 'MOTOR 2', 'SERVO 1', 'SERVO 2'].map((k) => ({ k, g: pin(res[k]) })).filter((o) => o.g !== null);
      const bySlice = {};
      for (const o of outs) (bySlice[slice(o.g)] = bySlice[slice(o.g)] || []).push(o);
      const clash = Object.entries(bySlice).filter(([, os]) => os.some((o) => o.k.startsWith('MOTOR')) && os.some((o) => o.k.startsWith('SERVO')));
      it.push(item('PWM slices', outs.map((o) => `${o.k}=GP${o.g}→slice ${slice(o.g)}${o.g & 1 ? 'B' : 'A'}`).join(', '), clash.length ? 'fail' : 'pass',
        clash.length ? 'A motor and a servo share a PWM slice, so they share one frequency: 50 Hz servos would force the motors to 50 Hz (or the reverse).' : 'Motors and servos are on separate slices, so 200 Hz and 50 Hz are independent.'));
      const adc = pin(res['ADC_BATT 1']);
      if (adc !== null) it.push(item('VBAT ADC pin', `GP${adc}`, [26, 27, 28].includes(adc) ? 'pass' : 'fail', 'Only GP26-GP28 are ADC inputs on the Pico 2 header.'));
      const nrfHz = spiActualHz(p.nrf24_spi_clk_hz);
      it.push(item('nRF24 SPI clock', `${(p.nrf24_spi_clk_hz / 1e6).toFixed(2)} MHz requested → ${(nrfHz / 1e6).toFixed(3)} MHz actual`, p.nrf24_spi_clk_hz > 10e6 ? 'fail' : nrfHz > 8e6 ? 'warn' : 'pass',
        'The nRF24L01+ is rated to 10 MHz; the patch #errors above that. Long jumper wires prefer ≤ 4 MHz.'));
      const gHz = spiActualHz(p.gyro_spi_clk_hz);
      it.push(item('MPU6500 SPI clock', `${(gHz / 1e6).toFixed(2)} MHz actual`, gHz > 20e6 ? 'fail' : 'pass', 'MPU6500: 1 MHz for configuration, 20 MHz max for sensor reads.'));
      const fwId = S.Physics.parseId(p.link_id), hsId = S.Physics.parseId(hw.link.linkId);
      it.push(item('Link ID (aircraft vs handset)', `0x${fwId.toString(16).toUpperCase()} vs 0x${hsId.toString(16).toUpperCase()}`, fwId === hsId ? 'pass' : 'fail',
        fwId === hsId ? `Address ${PR.deriveAddress(fwId).map((b) => b.toString(16).padStart(2, '0').toUpperCase()).join(' ')}, channels ${PR.deriveRfChannels(fwId).join('/')}.` : 'Different IDs give different addresses, channels, tag and CRC seed: every packet is rejected.',
        fwId === hsId ? '' : 'Make BICOPTER_NRF24_LINK_ID equal HANDSET_LINK_ID and rebuild both.'));
      if (p.rx_spi_protocol !== 'NRF24_BICOPTER') it.push(item('RX protocol', p.rx_spi_protocol, 'fail', 'No receiver: arming is impossible.', 'set rx_spi_protocol = NRF24_BICOPTER'));
      if (p.servo_pwm_rate !== 50) it.push(item('servo_pwm_rate', `${p.servo_pwm_rate} Hz`, 'info', 'Ignored on the Pico target: the servo driver is fixed at 50 Hz (handoff finding 12).'));
      sections.push({ id: 'pins', title: 'Pins, buses & radio link', items: it });
    }

    // ---------------------------------------------------------------- structures
    {
      const it = [], mat = H.MATERIALS[fr.armMaterial], sec = H.section(fr.armSection);
      const free = Math.max(0.01, L - fr.hubHalfWidth);
      const mTip = H.tipMass(hw, 'L');
      const armMass = mat.density * sec.A * free;
      const full = sections[1].data.full;
      const cases = [];
      // 1: full thrust climb
      const fClimb = 2 * full.thrust / mp.mass;
      cases.push({ name: 'Full-thrust climb', F: full.thrust - mTip * fClimb, T: full.torque });
      // 2: hard landing from dropHeight
      const v = Math.sqrt(2 * U.G * fr.dropHeight), nLand = 1 + v * v / (2 * U.G * Math.max(0.002, fr.gearStroke));
      cases.push({ name: `Hard landing (${fr.dropHeight} m drop, ${(fr.gearStroke * 1000).toFixed(0)} mm stroke)`, F: mTip * U.G * nLand, T: 0, n: nLand });
      // 3: maneuver
      cases.push({ name: `Manoeuvre ${fr.maneuverLoadFactor} g at hover thrust`, F: (W / 2) * fr.maneuverLoadFactor - mTip * U.G * fr.maneuverLoadFactor, T: hov.torque });
      const servoT = hw.servo.stallTorque60;
      for (const c of cases) {
        const M = Math.abs(c.F) * free, sig = M * sec.cv / sec.Iv;
        const tau = (Math.abs(c.T) + servoT) * sec.ct / sec.J;
        const vm = Math.sqrt(sig * sig + 3 * tau * tau), fos = mat.yield / vm;
        it.push(item(`Arm root: ${c.name}`, `${(vm / 1e6).toFixed(0)} MPa, FoS ${fos.toFixed(1)}`, fos < 1.2 ? 'fail' : fos < fr.targetSafetyFactor ? 'warn' : 'pass',
          `Bending ${(sig / 1e6).toFixed(0)} MPa + torsion ${(tau / 1e6).toFixed(1)} MPa (von Mises) in ${mat.name}, yield ${(mat.yield / 1e6).toFixed(0)} MPa. Tip load ${Math.abs(c.F).toFixed(1)} N${c.n ? ` (${c.n.toFixed(0)} g impact)` : ''}.`,
          fos < fr.targetSafetyFactor ? 'Use a larger/thicker section or stiffer material, or add gear compliance.' : ''));
      }
      const Fh = full.thrust - mTip * fClimb;
      const defl = Fh * free ** 3 / (3 * mat.E * sec.Iv), slope = Fh * free * free / (2 * mat.E * sec.Iv) * U.RAD;
      it.push(item('Arm tip deflection at full thrust', `${(defl * 1000).toFixed(2)} mm, ${slope.toFixed(2)}° slope`, slope > 2 ? 'fail' : slope > 0.8 ? 'warn' : 'pass',
        'Arm bending tilts the thrust axis outward/inward, coupling roll and yaw and wasting thrust.'));
      const k = 3 * mat.E * sec.Iv / free ** 3, fn = Math.sqrt(k / (mTip + 0.24 * armMass)) / (2 * Math.PI);
      const fRot = hov.rpm / 60, fBlade = 2 * fRot, fRotMax = full.rpm / 60;
      const near = (a, b) => Math.abs(a - b) / b < 0.2;
      const res = near(fn, fRot) || near(fn, fBlade);
      it.push(item('Arm first bending mode', `${fn.toFixed(0)} Hz`, res ? 'warn' : fn < fRotMax ? 'warn' : 'pass',
        `Hover rotor ${fRot.toFixed(0)} Hz, blade pass ${fBlade.toFixed(0)} Hz, max rotor ${fRotMax.toFixed(0)} Hz. ${res ? 'Resonance near hover: vibration reaches the gyro and fatigues the arm.' : fn < fRotMax ? 'The rotor sweeps through this mode while throttling up: expect a vibration band.' : 'Mode above the rotor range.'}`,
        res || fn < fRotMax ? `Stiffen the arm, or put a gyro notch near ${fn.toFixed(0)} Hz; gyro_lpf1 is ${p.gyro_lpf1_static_hz} Hz.` : ''));
      // servo torque requirement
      const Jr = H.Prop.inertia(hw.prop) + hw.motor.rotorInertia;
      const maxRoll = p.roll_srate * 10 * U.DEG;
      const Hh = Jr * hov.w, Hf = Jr * full.w;
      const tGyroH = Hh * maxRoll, tGyroF = Hf * maxRoll * 0.5;
      const tPend = mp.nacelleMass * U.G * fr.nacelleCgAboveTilt * Math.sin(fr.tiltLimitDeg * U.DEG) * 2;
      const tOff = full.thrust * Math.abs(fr.motorAxisOffsetX);
      const need = Math.max(tGyroH, tGyroF) + tPend + tOff;
      const servoV = hw.power.servo.source === 'fc5v' ? hw.power.fc5v.v : hw.power.servo.v;
      const stall = (servoV >= 6 ? hw.servo.stallTorque60 : hw.servo.stallTorque48 + (hw.servo.stallTorque60 - hw.servo.stallTorque48) * U.clamp((servoV - 4.8) / 1.2, 0, 1)) * hw.servo.linkRatio;
      const fosS = stall / need;
      it.push(item('Tilt servo torque margin', `need ${(need * 100 / U.G).toFixed(2)} kg·cm, have ${(stall * 100 / U.G).toFixed(2)} kg·cm (FoS ${fosS.toFixed(1)})`, fosS < 1 ? 'fail' : fosS < 1.5 ? 'warn' : 'pass',
        `Gyroscopic load H×ω at the max roll rate ${(maxRoll * U.RAD).toFixed(0)}°/s: ${(tGyroH * 100 / U.G).toFixed(2)} kg·cm at hover rpm (prop + rotor inertia ${(Jr * 1e7).toFixed(0)} g·cm²). Plus the 2 g nacelle pendulum ${(tPend * 100 / U.G).toFixed(2)} kg·cm and thrust offset ${(tOff * 100 / U.G).toFixed(2)} kg·cm.`,
        fosS < 1.5 ? `Lower roll_srate (servos hold up to ~${(stall / 1.5 / Math.max(1e-6, Hh) * U.RAD).toFixed(0)}°/s at hover rpm), use a stronger servo, or gear the tilt (linkRatio > 1).` : ''));
      const sp60 = (servoV >= 6 ? hw.servo.speed60 : hw.servo.speed48) * hw.servo.linkRatio;
      it.push(item('Tilt actuator speed', `${(60 / sp60).toFixed(0)}°/s no-load`, 60 / sp60 < 300 ? 'warn' : 'pass', `A 10° correction takes ≥ ${(10 / (60 / sp60) * 1000).toFixed(0)} ms plus the 20 ms servo frame: this bounds pitch and yaw bandwidth.`));
      // clearances
      const R = hw.prop.diameter / 2, lim = fr.tiltLimitDeg * U.DEG;
      const latClr = L - R - fr.hubHalfWidth;
      it.push(item('Prop to centre-body clearance', `${(latClr * 1000).toFixed(0)} mm`, latClr < 0 ? 'fail' : latClr < 0.01 ? 'warn' : 'pass', ''));
      const lowZ = fr.propAboveTilt * Math.cos(lim) - R * Math.sin(lim), gClr = lowZ + fr.gearHeight;
      it.push(item('Prop to ground at full tilt (landed)', `${(gClr * 1000).toFixed(0)} mm`, gClr < 0 ? 'fail' : gClr < 0.02 ? 'warn' : 'pass', `Lowest blade point ${(lowZ * 1000).toFixed(0)} mm relative to the tilt axis at ${fr.tiltLimitDeg}°.`, gClr < 0.02 ? 'Lengthen the landing gear or reduce the tilt limit (servo min/max).' : ''));
      const armClr = fr.propAboveTilt - (fr.armSection.od || fr.armSection.height || 0.012) / 2 - 0.015;
      it.push(item('Prop to arm/servo clearance', `${(armClr * 1000).toFixed(0)} mm above arm top`, armClr < 0 ? 'fail' : armClr < 0.01 ? 'warn' : 'pass', 'Approximate: prop plane height above the tilt axis minus arm radius and a 15 mm servo/horn allowance.'));
      sections.push({ id: 'structure', title: 'Structure, servos & clearances', items: it });
    }

    // ---------------------------------------------------------------- firmware & thrust-vectoring logic
    {
      const it = [];
      const pitchT = tiltResponse(hw, fw, 1), yawT = tiltResponse(hw, fw, 2), rollT = tiltResponse(hw, fw, 0);
      const h = -cg[2];
      const pitchOk = pitchT[0] > 0 && pitchT[1] > 0 && h > 0;
      const pitchRev = pitchT[0] < 0 && pitchT[1] < 0;
      it.push(item('Thrust vectoring: +pitch (nose down)', `left ${pitchT[0].toFixed(1)}°, right ${pitchT[1].toFixed(1)}° per 100 PID units`,
        pitchOk ? 'pass' : 'fail',
        pitchOk ? 'Both nacelles tilt forward together: forward thrust above the CG pitches the nose down. Correct.' : pitchRev ? 'Both nacelles tilt BACKWARD for a nose-down command: the pitch loop is reversed and will diverge.' : 'The nacelles tilt in opposite directions (or not at all) for pitch: pitch becomes yaw.',
        pitchOk ? '' : pitchRev ? 'Reverse the pitch input on both servos: smix reverse 4 1 r and smix reverse 5 1 r (or flip servo 4/5 rate signs if yaw is also reversed).' : 'One servo is mirrored relative to the firmware: reverse the pitch input on one servo (smix reverse 4 1 r or smix reverse 5 1 r), then recheck yaw.'));
      const yawOk = yawT[0] < 0 && yawT[1] > 0, yawRev = yawT[0] > 0 && yawT[1] < 0;
      it.push(item('Thrust vectoring: +yaw (nose left, CCW)', `left ${yawT[0].toFixed(1)}°, right ${yawT[1].toFixed(1)}°`,
        yawOk ? 'pass' : 'fail',
        yawOk ? 'Right nacelle forward, left back: the aircraft turns nose-left. Correct.' : yawRev ? 'Yaw is reversed: the nacelles push the nose the wrong way, the yaw loop diverges (spins up).' : 'Both nacelles move together for yaw: yaw becomes pitch.',
        yawOk ? '' : yawRev ? 'Reverse the yaw input on both servos: smix reverse 4 2 r and smix reverse 5 2 r.' : 'Reverse the yaw input on one servo (smix reverse 4 2 r or 5 2 r).'));
      if (Math.abs(rollT[0]) + Math.abs(rollT[1]) > 0.01) it.push(item('Roll → servos', `left ${rollT[0].toFixed(1)}°, right ${rollT[1].toFixed(1)}°`, 'warn', 'Roll is fed to the tilt servos: unusual for a bicopter.'));
      const mm = p.mixer === 'CUSTOM' ? fw.mmix : FW.BI_MMIX;
      const rollOk = mm[0] && mm[1] && mm[0].roll > 0 && mm[1].roll < 0;
      it.push(item('Roll: differential thrust', mm[0] && mm[1] ? `left ${mm[0].roll}, right ${mm[1].roll}` : 'motor mix incomplete', rollOk ? 'pass' : 'fail',
        rollOk ? '+roll (right side down) speeds up the left motor. Correct.' : 'Roll mixing is reversed or missing: the aircraft flips on take-off.', rollOk ? '' : 'mmix 0 1 1 0 0 and mmix 1 1 -1 0 0, or mixer BICOPTER.'));
      if (p.mixer !== 'CUSTOM' && fw.smix.some((r, i) => JSON.stringify(r) !== JSON.stringify(Object.assign({ speed: 0, min: 0, max: 100, box: 0 }, FW.BI_SMIX[i] || {}))))
        it.push(item('Custom smix rules', `${fw.smix.length} rules`, 'info', 'mixer BICOPTER uses the built-in BI servo rules; your smix table only applies with mixer CUSTOM. Direction fixes on BI go through servo rate signs or smix reverse.'));
      // servo throw vs mechanical stop
      for (const [idx, side] of [[4, 'left'], [5, 'right']]) {
        const s = fw.servos[idx], perUs = hw.servo.degPerUs / hw.servo.linkRatio;
        const reach = Math.max(s.max - 1500, 1500 - s.min) * perUs;
        const trim = s.mid - 1500;
        it.push(item(`Servo ${idx} (${side}) travel`, `${s.min}-${s.max} us, mid ${s.mid}, rate ${s.rate} → ±${reach.toFixed(0)}° nacelle`, reach > fr.tiltLimitDeg + 1 ? 'warn' : 'pass',
          reach > fr.tiltLimitDeg + 1 ? `The servo can drive past the ${fr.tiltLimitDeg}° mechanical stop: it will stall against it (heat, current, stripped gears).` : 'Travel stays inside the mechanical stop.',
          reach > fr.tiltLimitDeg + 1 ? `servo ${idx} ${Math.round(1500 - fr.tiltLimitDeg / perUs)} ${Math.round(1500 + fr.tiltLimitDeg / perUs)} ${s.mid} ${s.rate} -1` : ''));
        if (Math.abs(trim) > 0) it.push(item(`Servo ${idx} middle trim`, `${trim > 0 ? '+' : ''}${trim} us = ${(trim * perUs * (side === 'left' ? hw.servo.dirL : hw.servo.dirR)).toFixed(1)}° tilt`, Math.abs(trim * perUs) > 8 ? 'warn' : 'info', 'Use the middle value to level the nacelles mechanically before flight.'));
      }
      // arming / interlock
      const arm = fw.aux.find((a) => a.mode === 0);
      if (!arm) it.push(item('ARM mode', 'not assigned', 'fail', 'No aux range for ARM: the aircraft cannot arm.', 'aux 0 0 0 1700 2100 0 0'));
      else {
        it.push(item('ARM on AUX1 (channel 5)', `AUX${arm.ch + 1} ${arm.lo}-${arm.hi}`, arm.ch === 0 ? 'pass' : 'fail', arm.ch === 0 ? 'Matches the protocol interlock (BL_ARM_CHANNEL = 4).' : 'The NRF24 restart interlock watches channel 5; ARM on another channel defeats it.', arm.ch === 0 ? '' : 'aux 0 0 0 1700 2100 0 0'));
        it.push(item('ARM range vs interlock "ARM low" (≤ 1250 us)', `${arm.lo}-${arm.hi}`, arm.lo <= PR.ARM_LOW_US ? 'fail' : 'pass', arm.lo <= PR.ARM_LOW_US ? 'A switch position the interlock treats as ARM OFF would arm Betaflight.' : 'No overlap.'));
      }
      const im = hw.imuMount, al = [p.align_board_roll, p.align_board_pitch, p.align_board_yaw];
      const imuOk = [im.roll, im.pitch, im.yaw].every((v, k) => ((v - al[k]) % 360 + 360) % 360 < 1 || ((v - al[k]) % 360 + 360) % 360 > 359);
      it.push(item('IMU orientation vs board alignment', `mounted ${im.roll}/${im.pitch}/${im.yaw}°, align_board ${al.join('/')}°`, imuOk ? 'pass' : 'fail',
        imuOk ? 'Gyro axes match the airframe.' : 'The flight controller would read roll as pitch (or reversed): the first correction makes things worse and it flips on take-off.',
        imuOk ? '' : `set align_board_roll = ${im.roll} and set align_board_pitch = ${im.pitch} and set align_board_yaw = ${im.yaw}`));
      const angle = fw.aux.find((a) => a.mode === 1);
      it.push(item('Self-levelling (ANGLE mode)', angle ? `AUX${angle.ch + 1} ${angle.lo}-${angle.hi}` : 'not assigned', angle ? 'info' : 'warn',
        angle ? 'The project CLI (HANDOFF.md §11) only assigns ARM; this ANGLE range is a simulator default. Add the same aux line on the real board, or it flies in ACRO.' : 'Without ANGLE the aircraft flies in ACRO (rate) mode: very hard on a bicopter.',
        angle ? '' : 'aux 1 1 1 1300 2100 0 0'));
      it.push(item('min_check vs interlock throttle-low (≤ 1050 us)', `${p.min_check} us`, p.min_check < PR.THROTTLE_LOW_US ? 'warn' : 'pass', 'Arming needs throttle < min_check; the receiver releases its interlock at ≤ 1050 us.'));
      // failsafe analysis
      const thrMode = fw.rxfail[2].m, thrVal = thrMode === 'a' ? p.rx_min_usec : thrMode === 's' ? fw.rxfail[2].v : null;
      const t1 = p.failsafe_delay / 10, tHold = p.rx_frame_timeout_ms / 1000;
      if (thrVal !== null && thrVal < 1100) {
        const dh = 0.5 * U.G * t1 * t1;
        it.push(item('Stage-1 throttle (rxfail ch 2)', `${thrMode === 'a' ? 'auto' : 'set'} → ${thrVal} us (idle)`, t1 > 0.3 ? 'warn' : 'info',
          `During any dropout longer than ${p.rx_frame_timeout_ms} ms Betaflight applies rxfail: throttle goes to idle for up to ${t1.toFixed(1)} s before ${p.failsafe_procedure}. A bicopter falls ≈ ${dh.toFixed(1)} m in that time even if the link returns. (Model of Betaflight's AUTO rxfail; confirm with 'rxfail' on the real board.)`,
          'If short dropouts are expected, consider rxfail 2 h (hold) or a lower failsafe_delay; test with the 400 ms dropout scenario.'));
      }
      it.push(item('Signal loss to motor stop (DROP)', `${(tHold + t1).toFixed(2)} s`, p.failsafe_procedure === 'DROP' ? 'info' : 'info',
        `${p.rx_frame_timeout_ms} ms RXLOSS detection + ${t1.toFixed(1)} s stage 1, then ${p.failsafe_procedure}. A handset reboot skips this and disarms at once (failsafeOnReceiverReset).`));
      it.push(item('Air mode', p.airmode, p.airmode === 'ON' ? 'pass' : 'warn', p.airmode === 'ON' ? 'PID keeps authority at zero throttle (needed for the stage-1 idle throttle and descents).' : 'At zero throttle the PID is weakened and the I-term reset: attitude control is poor during stage-1 failsafe and fast descents.'));
      if (fw.customEnabled) {
        let ok = true, msg = 'Compiles.';
        try { new Function('io', 'Math', fw.customCode); } catch (e) { ok = false; msg = e.message; }
        it.push(item('Custom firmware hook', ok ? 'enabled' : 'compile error', ok ? 'info' : 'fail', msg));
      }
      const ptests = PR.selfTest();
      const bad = ptests.filter((t) => !t.ok);
      it.push(item('Protocol v2 core (JS port vs C test vector)', `${ptests.length - bad.length}/${ptests.length} checks`, bad.length ? 'fail' : 'pass', bad.length ? bad.map((b) => b.name).join(', ') : 'Known packet B10234095A…2A89, tag, CRC seed, address, channels, freshness rules.'));
      sections.push({ id: 'firmware', title: 'Firmware logic & thrust vectoring', items: it });
    }

    // ---------------------------------------------------------------- link budget
    {
      const it = [], lk = hw.link;
      const ptx = (S.Link.RF_PWR_DBM[lk.rfPwrBits] ?? 0) + lk.paGainDb;
      const budget = ptx + lk.txAntDbi + lk.rxAntDbi - lk.rxSensDbm - 3 - 2 * lk.fadeSigmaDb;
      const range = Math.pow(10, (budget - 40.2) / (10 * lk.pathLossExp));
      it.push(item('Estimated reliable range', `${range.toFixed(0)} m`, range < 100 ? 'warn' : 'pass', `TX ${ptx} dBm (RF_PWR 0x${lk.rfPwrBits.toString(16).padStart(2, '0')} + PA ${lk.paGainDb} dB), RX sensitivity ${lk.rxSensDbm} dBm, path-loss exponent ${lk.pathLossExp}, ${2 * lk.fadeSigmaDb} dB fade margin.`,
        range < 100 ? 'Raise HANDSET_RF_PWR_BITS (0x06 = max) for field use; keep 0x00 only for bench tests.' : ''));
      const sat = Math.pow(10, (ptx + lk.txAntDbi + lk.rxAntDbi - lk.lnaSatDbm - 40.2) / 20);
      it.push(item('Too-close distance (LNA saturation)', `< ${sat.toFixed(1)} m`, sat > 1.5 ? 'warn' : 'pass', 'PA/LNA modules at high power overload each other at short range (packet loss on the bench).'));
      if (lk.wifiChannel > 0) {
        const f = 2412 + 5 * (lk.wifiChannel - 1), hit = PR.deriveRfChannels(S.Physics.parseId(hw.link.linkId)).filter((c) => Math.abs(2400 + c - f) <= 11);
        it.push(item(`Wi-Fi channel ${lk.wifiChannel} overlap`, hit.length ? `RF_CH ${hit.join(', ')}` : 'none', hit.length ? 'warn' : 'pass', hit.length ? `${hit.length} of 4 hop channels lose ${lk.wifiLevelDb} dB of sensitivity; the hop scheme still delivers ${4 - hit.length}/4 slots.` : ''));
      }
      sections.push({ id: 'link', title: 'Radio link budget', items: it });
    }

    const summary = { pass: 0, warn: 0, fail: 0, info: 0 };
    for (const s of sections) for (const i of s.items) summary[i.status]++;
    return { sections, summary, hover: { duty: hoverDuty, ...hov }, mp };
  }

  S.Analysis = { analyze, spiActualHz, tiltResponse, hoverPoint };
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
