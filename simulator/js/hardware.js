// Hardware definition of the aircraft: components (mass + position), frame/arm structure,
// propulsion parts, power distribution and electronics. Defaults follow the project
// (A2212 1000KV, SimonK 30A, MG90S, 3S, Pico 2, MPU6500, nRF24 PA/LNA). Masses are typical
// catalogue values: weigh your own parts and edit them in the Airframe tab.
//
// Body frame: x forward, y left, z up. Origin = midpoint between the two nacelle tilt axes.
(function (S) {
  'use strict';
  const U = S.U;

  const MATERIALS = {
    cf_tube:   { name: 'Carbon fibre (pultruded)', E: 70e9,  G: 5e9,   yield: 600e6, density: 1600 },
    cf_roll:   { name: 'Carbon fibre (roll-wrapped)', E: 55e9, G: 20e9, yield: 450e6, density: 1550 },
    al6061:    { name: 'Aluminium 6061-T6',        E: 69e9,  G: 26e9,  yield: 276e6, density: 2700 },
    al7075:    { name: 'Aluminium 7075-T6',        E: 72e9,  G: 27e9,  yield: 503e6, density: 2810 },
    plywood:   { name: 'Birch plywood',            E: 10e9,  G: 0.6e9, yield: 40e6,  density: 680 },
    pine:      { name: 'Pine / spruce stick',      E: 9e9,   G: 0.5e9, yield: 35e6,  density: 450 },
    pla:       { name: 'PLA (3D printed, 100%)',   E: 3.5e9, G: 1.2e9, yield: 45e6,  density: 1240 },
    petg:      { name: 'PETG (3D printed, 100%)',  E: 2.1e9, G: 0.8e9, yield: 40e6,  density: 1270 },
    g10:       { name: 'G10 / FR4 fibreglass',     E: 18e9,  G: 4e9,   yield: 260e6, density: 1850 },
  };

  // Section properties: area A, second moments Iy (vertical bending), Iz (horizontal bending),
  // polar/torsion constant J, extreme fibre distances cz (vertical) and cy (horizontal).
  function section(sec) {
    if (sec.type === 'round_tube' || sec.type === 'round_solid') {
      const ro = sec.od / 2, ri = sec.type === 'round_solid' ? 0 : Math.max(0, ro - sec.wall);
      const A = Math.PI * (ro * ro - ri * ri), I = Math.PI / 4 * (ro ** 4 - ri ** 4);
      return { A, Iv: I, Ih: I, J: 2 * I, cv: ro, ch: ro, ct: ro };
    }
    if (sec.type === 'square_tube') {
      const b = sec.od, bi = Math.max(0, b - 2 * sec.wall);
      const A = b * b - bi * bi, I = (b ** 4 - bi ** 4) / 12;
      const t = sec.wall, bm = b - t; // Bredt thin-wall torsion
      return { A, Iv: I, Ih: I, J: 4 * (bm * bm) ** 2 * t / (4 * bm), cv: b / 2, ch: b / 2, ct: b / 2 };
    }
    // solid rectangle: width (horizontal) x height (vertical)
    const w = sec.width, h = sec.height;
    const a = Math.max(w, h), b = Math.min(w, h);
    const J = a * b ** 3 * (1 / 3 - 0.21 * b / a * (1 - b ** 4 / (12 * a ** 4)));
    return { A: w * h, Iv: w * h ** 3 / 12, Ih: h * w ** 3 / 12, J, cv: h / 2, ch: w / 2, ct: b / 2 };
  }

  // 3S LiPo open-circuit voltage per cell vs state of charge.
  const LIPO_OCV = [[0, 3.27], [0.05, 3.50], [0.1, 3.61], [0.2, 3.69], [0.3, 3.73], [0.4, 3.77], [0.5, 3.81],
                    [0.6, 3.86], [0.7, 3.93], [0.8, 4.00], [0.9, 4.09], [1.0, 4.20]];

  // American wire gauge: resistance (ohm/m, copper 20 C) and a conservative continuous
  // ampacity for short, free-air chassis runs with silicone insulation.
  const AWG = {
    10: { r: 0.00328, amp: 55 }, 12: { r: 0.00521, amp: 41 }, 14: { r: 0.00829, amp: 32 },
    16: { r: 0.0132, amp: 22 }, 18: { r: 0.0210, amp: 16 }, 20: { r: 0.0333, amp: 11 },
    22: { r: 0.0530, amp: 7 }, 24: { r: 0.0842, amp: 3.5 }, 26: { r: 0.134, amp: 2.2 }, 28: { r: 0.213, amp: 1.4 },
  };

  const CONNECTORS = { XT60: { rating: 60, r: 0.0006 }, XT30: { rating: 30, r: 0.0010 }, XT90: { rating: 90, r: 0.0003 },
    Deans: { rating: 50, r: 0.0008 }, 'Bullet 3.5mm': { rating: 70, r: 0.0004 }, 'Bullet 2mm': { rating: 20, r: 0.0015 },
    'JST-RCY': { rating: 5, r: 0.010 }, Soldered: { rating: 999, r: 0.0001 } };

  function defaults() {
    return {
      name: 'DIY tilt-rotor bicopter (Pico 2 + NRF24)',
      frame: {
        armLength: 0.25,          // m, centreline to nacelle tilt axis (each side)
        hubHalfWidth: 0.045,      // m, half width of the centre body (arms start here)
        armMaterial: 'cf_tube',
        armSection: { type: 'round_tube', od: 0.012, wall: 0.001, width: 0.012, height: 0.012 },
        propAboveTilt: 0.045,     // m, prop plane above the tilt axis
        motorAxisOffsetX: 0.0,    // m, motor axis ahead of the tilt axis (thrust moment on the servo)
        nacelleCgAboveTilt: 0.022,// m, centre of mass of tilting parts above the tilt axis
        tiltLimitDeg: 35,         // mechanical stop of the nacelle
        gearHeight: 0.11,         // m, landing gear contact below the tilt axis
        gearSpanX: 0.16,          // m, fore-aft distance between skid contacts
        gearSpanY: 0.20,          // m, left-right distance between skids
        gearStroke: 0.008,        // m, effective compliance of the gear for drop impacts
        bodyDragArea: 0.012,      // m^2 Cd*A
        maneuverLoadFactor: 2.5,
        dropHeight: 0.5,          // m, hard-landing case
        targetSafetyFactor: 2.0,
      },
      // tilts: 'L' / 'R' means the part rotates with that nacelle. auto: computed from the frame.
      components: [
        { id: 'hub', name: 'Centre plate / hub', mass: 0.085, pos: [0, 0, -0.035], size: [0.13, 0.09, 0.03] },
        { id: 'armL', name: 'Left arm', auto: 'armL', mass: 0, pos: [0, 0.145, 0], size: [0.012, 0.21, 0.012] },
        { id: 'armR', name: 'Right arm', auto: 'armR', mass: 0, pos: [0, -0.145, 0], size: [0.012, 0.21, 0.012] },
        { id: 'nacL', name: 'Left nacelle bracket + bearing', mass: 0.016, pos: [0, 0.25, 0.005], size: [0.04, 0.03, 0.04], tilts: 'L' },
        { id: 'nacR', name: 'Right nacelle bracket + bearing', mass: 0.016, pos: [0, -0.25, 0.005], size: [0.04, 0.03, 0.04], tilts: 'R' },
        { id: 'motL', name: 'Left motor A2212 1000KV', mass: 0.052, pos: [0, 0.25, 0.025], size: [0.028, 0.028, 0.03], tilts: 'L' },
        { id: 'motR', name: 'Right motor A2212 1000KV', mass: 0.052, pos: [0, -0.25, 0.025], size: [0.028, 0.028, 0.03], tilts: 'R' },
        { id: 'propL', name: 'Left prop 1045', mass: 0.010, pos: [0, 0.25, 0.045], size: [0.254, 0.254, 0.005], tilts: 'L' },
        { id: 'propR', name: 'Right prop 1045', mass: 0.010, pos: [0, -0.25, 0.045], size: [0.254, 0.254, 0.005], tilts: 'R' },
        { id: 'servoL', name: 'Left servo MG90S', mass: 0.0134, pos: [0, 0.215, -0.012], size: [0.023, 0.012, 0.029] },
        { id: 'servoR', name: 'Right servo MG90S', mass: 0.0134, pos: [0, -0.215, -0.012], size: [0.023, 0.012, 0.029] },
        { id: 'escL', name: 'Left ESC SimonK 30A', mass: 0.025, pos: [0, 0.12, -0.012], size: [0.05, 0.025, 0.008] },
        { id: 'escR', name: 'Right ESC SimonK 30A', mass: 0.025, pos: [0, -0.12, -0.012], size: [0.05, 0.025, 0.008] },
        { id: 'batt', name: '3S LiPo 2200 mAh', mass: 0.185, pos: [0, 0, -0.07], size: [0.105, 0.034, 0.024] },
        { id: 'pdb', name: 'PDB', mass: 0.015, pos: [0, 0, -0.048], size: [0.05, 0.05, 0.004] },
        { id: 'pico', name: 'Raspberry Pi Pico 2', mass: 0.004, pos: [0.02, 0, -0.016], size: [0.051, 0.021, 0.004] },
        { id: 'imu', name: 'MPU6500 breakout', mass: 0.002, pos: [0, 0, -0.014], size: [0.02, 0.015, 0.003] },
        { id: 'radio', name: 'nRF24L01+ PA/LNA + antenna', mass: 0.012, pos: [-0.045, 0, -0.012], size: [0.04, 0.016, 0.01] },
        { id: 'ubec', name: 'Servo UBEC 5 V 3 A', mass: 0.012, pos: [0.04, 0, -0.045], size: [0.03, 0.018, 0.008] },
        { id: 'ldo', name: '3.3 V LDO (radio)', mass: 0.002, pos: [-0.03, 0, -0.016], size: [0.01, 0.01, 0.003] },
        { id: 'wires', name: 'Wiring + connectors', mass: 0.040, pos: [0, 0, -0.035], size: [0.1, 0.1, 0.02] },
        { id: 'gear', name: 'Landing gear', mass: 0.030, pos: [0, 0, -0.095], size: [0.16, 0.2, 0.01] },
      ],
      motor: { name: 'A2212/13T 1000KV', kv: 1000, rm: 0.090, i0: 0.5, maxCurrent: 13, maxCurrentTime: 60,
               rotorInertia: 5e-6, cth: 25, rth: 3.5, tmax: 100, spinL: -1, spinR: +1 }, // spin: +1 = CCW seen from above
      prop: { name: '1045 (10 x 4.5 in)', diameter: 0.254, pitch: 0.1143, ct: 0.090, cp: 0.041, mass: 0.010 },
      esc: { name: 'SimonK 30A', contCurrent: 30, burstCurrent: 40, burstTime: 10, rOn: 0.010, cth: 12, rth: 6, tmax: 110,
             // Calibrated end points. Uncalibrated SimonK defaults are about 1060 / 1860 us.
             calLow: 1000, calHigh: 2000, startDuty: 0.035, maxRateHz: 490, dampedLight: true },
      battery: { name: '3S LiPo 2200 mAh 25C', cells: 3, capacity: 2.2, cRating: 25, rCell: 0.012, cth: 200, rth: 6, tmax: 60, soc0: 1.0 },
      servo: { name: 'MG90S', stallTorque48: 0.176, stallTorque60: 0.216, speed48: 0.10, speed60: 0.08, // N*m, s/60deg
               degPerUs: 0.09, linkRatio: 1.0, iIdle: 0.010, iStall48: 0.70, iStall60: 0.90, gearStripTorque: 0.45,
               dirL: -1, dirR: +1, // +1: increasing PWM tilts the nacelle forward (thrust toward +x)
               trimDegL: 0, trimDegR: 0 },
      power: {
        battLead: { awg: 12, length: 0.10, connector: 'XT60' },
        escLead: { awg: 16, length: 0.12, connector: 'Bullet 3.5mm' },
        motorLead: { awg: 18, length: 0.08, connector: 'Bullet 3.5mm' },
        // Rail sources. fc5v feeds Pico VSYS (+ radio LDO input).
        fc5v: { source: 'escbec', v: 5.0, imax: 2.0, type: 'linear', dropout: 1.2, rOut: 0.05 },
        servo: { source: 'ubec', v: 5.0, imax: 3.0, type: 'buck', eff: 0.88, rOut: 0.03 },  // 'ubec' | 'fc5v'
        radio: { source: 'ldo', v: 3.3, imax: 0.8, dropout: 1.1, capUF: 47 },              // 'ldo' | 'pico3v3'
        pico3v3Limit: 0.30,
        commonGround: true,
        mpuBreakout: 'ldo',   // 'ldo' = has regulator (VCC 3.3-5 V); 'bare' = 3.3 V only
        mpuSupply: 'pico3v3', // 'pico3v3' | 'fc5v'
        vbatTop: 10000, vbatBottom: 1000, // ohm
        extraLoads: [ /* {name, rail: 'fc5v'|'servo'|'pico3v3'|'batt', avg: A, peak: A} */ ],
      },
      loads: { picoA: 0.030, imuA: 0.004, radioRxA: 0.030, radioTxPeakA: 0.130, radioTxDuty: 0.08, ledA: 0.002 },
      // rfPwrBits = handset HANDSET_RF_PWR_BITS (link_config.h). linkId = HANDSET_LINK_ID.
      link: { rfPwrBits: 0x00, linkId: '0xBE5336DF', paGainDb: 20, txAntDbi: 2, rxAntDbi: 2, rxSensDbm: -104, lnaSatDbm: -20,
              pathLossExp: 2.7, fadeSigmaDb: 4, wifiChannel: 0, wifiLevelDb: 15, pilotPos: [-4, 0, 1.2] },
      imuMount: { roll: 0, pitch: 0, yaw: 0 }, // physical IMU orientation on the frame (deg)
      environment: { windX: 0, windY: 0, gust: 0, altitudeAmsl: 0, tempC: 25 },
    };
  }

  // Masses of the auto-sized components (arms) from the frame definition.
  function resolveComponents(hw) {
    const f = hw.frame, mat = MATERIALS[f.armMaterial] || MATERIALS.cf_tube, sec = section(f.armSection);
    const free = Math.max(0.01, f.armLength - f.hubHalfWidth);
    return hw.components.map((c) => {
      const o = Object.assign({}, c, { pos: c.pos.slice(), size: (c.size || [0.01, 0.01, 0.01]).slice() });
      if (c.auto === 'armL' || c.auto === 'armR') {
        const s = c.auto === 'armL' ? 1 : -1;
        o.mass = mat.density * sec.A * free;
        o.pos = [0, s * (f.hubHalfWidth + free / 2), 0];
        o.size = [f.armSection.od || f.armSection.width, free, f.armSection.od || f.armSection.height];
      }
      if (c.tilts) {
        const s = c.tilts === 'L' ? 1 : -1;
        o.pos[1] = s * f.armLength;
      }
      return o;
    });
  }

  // Mass properties for given nacelle tilts (rad). Tilting parts rotate about the tilt axis
  // (body y through (0, +-L, 0)); positive tilt moves thrust toward +x.
  function massProperties(hw, tiltL = 0, tiltR = 0) {
    const comps = resolveComponents(hw);
    let m = 0; const cg = [0, 0, 0];
    const placed = comps.map((c) => {
      let p = c.pos;
      if (c.tilts) {
        const d = c.tilts === 'L' ? tiltL : tiltR, ax = [0, p[1], 0], rel = U.sub(p, ax);
        // rotation about +y by -d? We want +d to move +z parts toward +x: x' = x cos + z sin, z' = -x sin + z cos
        p = [ax[0] + rel[0] * Math.cos(d) + rel[2] * Math.sin(d), p[1], -rel[0] * Math.sin(d) + rel[2] * Math.cos(d)];
      }
      m += c.mass; cg[0] += c.mass * p[0]; cg[1] += c.mass * p[1]; cg[2] += c.mass * p[2];
      return Object.assign({}, c, { p });
    });
    const CG = m > 0 ? U.scale(cg, 1 / m) : [0, 0, 0];
    const I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (const c of placed) {
      const r = U.sub(c.p, CG), [lx, ly, lz] = c.size, mm = c.mass;
      // box own inertia + parallel axis
      const ixx = mm * (ly * ly + lz * lz) / 12, iyy = mm * (lx * lx + lz * lz) / 12, izz = mm * (lx * lx + ly * ly) / 12;
      I[0] += ixx + mm * (r[1] * r[1] + r[2] * r[2]);
      I[4] += iyy + mm * (r[0] * r[0] + r[2] * r[2]);
      I[8] += izz + mm * (r[0] * r[0] + r[1] * r[1]);
      I[1] -= mm * r[0] * r[1]; I[2] -= mm * r[0] * r[2]; I[5] -= mm * r[1] * r[2];
    }
    I[3] = I[1]; I[6] = I[2]; I[7] = I[5];
    const nacMass = placed.filter((c) => c.tilts === 'L').reduce((a, c) => a + c.mass, 0);
    return { mass: m, cg: CG, I, Iinv: U.mat3Inv(I), placed, nacelleMass: nacMass };
  }

  // Mass carried at the end of each arm (nacelle + motor + prop + servo), used for arm loads.
  function tipMass(hw, side) {
    const comps = resolveComponents(hw);
    const L = hw.frame.armLength;
    return comps.filter((c) => (c.tilts === side) || (!c.tilts && !c.auto && Math.sign(c.pos[1]) === (side === 'L' ? 1 : -1) && Math.abs(c.pos[1]) > 0.6 * L))
      .reduce((a, c) => a + c.mass, 0);
  }

  // ---- propulsion physics ----
  const Prop = {
    kT: (prop, rho) => prop.ct * rho * Math.pow(prop.diameter, 4) / (4 * Math.PI * Math.PI),   // T = kT * w^2
    kQ: (prop, rho) => prop.cp * rho * Math.pow(prop.diameter, 5) / (8 * Math.PI ** 3),         // Q = kQ * w^2
    inertia: (prop) => prop.mass * prop.diameter * prop.diameter / 12 * 0.75,
  };

  // Air density from altitude and temperature (ISA-ish).
  const airDensity = (altM, tempC) => {
    const T = tempC + 273.15, p = 101325 * Math.pow(1 - 2.25577e-5 * altM, 5.25588);
    return p / (287.05 * T);
  };

  // Steady-state motor operating point for a given duty and bus voltage (bisection on rpm).
  function motorSteady(hw, duty, vbus, rho) {
    const m = hw.motor, kvr = m.kv * 2 * Math.PI / 60, kt = 1 / kvr;
    const kT = Prop.kT(hw.prop, rho), kQ = Prop.kQ(hw.prop, rho);
    const V = duty * vbus;
    let lo = 0, hi = V * kvr;
    for (let i = 0; i < 60; i++) {
      const w = (lo + hi) / 2, I = (V - w / kvr) / m.rm;
      if (kt * (I - m.i0) > kQ * w * w) lo = w; else hi = w;
    }
    const w = lo, I = Math.max(0, (V - w / kvr) / m.rm);
    return { w, rpm: w * 60 / (2 * Math.PI), thrust: kT * w * w, torque: kQ * w * w, current: I, battCurrent: duty * I,
             pElec: V * I, pMech: kQ * w * w * w, eff: V * I > 0 ? kQ * w ** 3 / (V * I) : 0 };
  }

  function batteryOcv(hw, soc) { return hw.battery.cells * U.interp(LIPO_OCV, U.clamp(soc, 0, 1)); }
  function batteryResistance(hw) {
    const p = hw.power;
    const wires = AWG[p.battLead.awg].r * p.battLead.length * 2 + (CONNECTORS[p.battLead.connector] || CONNECTORS.Soldered).r;
    return hw.battery.cells * hw.battery.rCell + wires;
  }

  S.Hardware = { MATERIALS, AWG, CONNECTORS, LIPO_OCV, section, defaults, resolveComponents, massProperties, tipMass,
                 Prop, airDensity, motorSteady, batteryOcv, batteryResistance };
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
