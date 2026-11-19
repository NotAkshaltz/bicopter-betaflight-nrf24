// Betaflight-style flight controller model for the BICOPTER_PICO2 target.
// It mirrors the parts of Betaflight 2026.6.2 that decide whether the aircraft flies:
// RX watchdog + failsafe stages, arming checks, ACTUAL rates, angle mode, PID with
// Betaflight's P/I/D/F scaling, TPA, the BI motor mixer, the servo mixer (with servo
// min/max/middle/rate/reverse), motor/servo output rates and battery monitoring.
// It is a model, not the C code: compare against a real blackbox log before trusting a tune.
(function (S) {
  'use strict';
  const U = S.U;

  // name: [default, min, max, group, description, enumValues?]
  const P = {
    // rates
    rates_type: ['ACTUAL', 0, 0, 'Rates', 'Rates formula (only ACTUAL is modelled)', ['ACTUAL']],
    roll_rc_rate: [7, 1, 255, 'Rates', 'Center sensitivity /10 (deg/s)'],
    pitch_rc_rate: [7, 1, 255, 'Rates', 'Center sensitivity /10 (deg/s)'],
    yaw_rc_rate: [7, 1, 255, 'Rates', 'Center sensitivity /10 (deg/s)'],
    roll_srate: [67, 0, 255, 'Rates', 'Max rate /10 (deg/s)'],
    pitch_srate: [67, 0, 255, 'Rates', 'Max rate /10 (deg/s)'],
    yaw_srate: [67, 0, 255, 'Rates', 'Max rate /10 (deg/s)'],
    roll_expo: [0, 0, 100, 'Rates', 'Expo'],
    pitch_expo: [0, 0, 100, 'Rates', 'Expo'],
    yaw_expo: [0, 0, 100, 'Rates', 'Expo'],
    thr_mid: [50, 0, 100, 'Rates', 'Throttle curve mid point (%)'],
    thr_expo: [0, 0, 100, 'Rates', 'Throttle curve expo'],
    throttle_limit_percent: [100, 25, 100, 'Rates', 'Throttle limit (%)'],
    // PID
    p_roll: [45, 0, 250, 'PID', 'Roll P'], i_roll: [80, 0, 250, 'PID', 'Roll I'], d_roll: [30, 0, 250, 'PID', 'Roll D'], f_roll: [120, 0, 1000, 'PID', 'Roll feedforward'],
    p_pitch: [47, 0, 250, 'PID', 'Pitch P'], i_pitch: [84, 0, 250, 'PID', 'Pitch I'], d_pitch: [34, 0, 250, 'PID', 'Pitch D'], f_pitch: [125, 0, 1000, 'PID', 'Pitch feedforward'],
    p_yaw: [45, 0, 250, 'PID', 'Yaw P'], i_yaw: [80, 0, 250, 'PID', 'Yaw I'], d_yaw: [0, 0, 250, 'PID', 'Yaw D'], f_yaw: [120, 0, 1000, 'PID', 'Yaw feedforward'],
    angle_p_gain: [50, 0, 200, 'PID', 'Angle mode strength'],
    angle_limit: [60, 10, 90, 'PID', 'Angle mode max angle (deg)'],
    iterm_limit: [400, 0, 500, 'PID', 'I-term limit'],
    pidsum_limit: [500, 100, 1000, 'PID', 'PID sum limit roll/pitch'],
    pidsum_limit_yaw: [400, 100, 1000, 'PID', 'PID sum limit yaw'],
    tpa_rate: [65, 0, 100, 'PID', 'TPA rate (%)'],
    tpa_breakpoint: [1350, 750, 2250, 'PID', 'TPA breakpoint (us)'],
    tpa_mode: ['D', 0, 0, 'PID', 'TPA applies to', ['PD', 'D']],
    airmode: ['ON', 0, 0, 'PID', 'Air mode (sim alias for the AIRMODE mode/feature)', ['OFF', 'ON']],
    // filters
    gyro_lpf1_static_hz: [250, 0, 1000, 'Filters', 'Gyro low-pass 1 (PT1)'],
    gyro_lpf2_static_hz: [500, 0, 1000, 'Filters', 'Gyro low-pass 2 (PT1)'],
    dterm_lpf1_static_hz: [75, 0, 1000, 'Filters', 'D-term low-pass 1 (PT1)'],
    dterm_lpf2_static_hz: [150, 0, 1000, 'Filters', 'D-term low-pass 2 (PT1)'],
    imu_dcm_kp: [2500, 0, 32000, 'Filters', 'Attitude estimator accelerometer gain'],
    // mixer / motors / servos
    mixer: ['BICOPTER', 0, 0, 'Mixer', 'Mixer (BICOPTER = built-in BI rules; CUSTOM = mmix/smix tables)', ['BICOPTER', 'CUSTOM']],
    motor_pwm_protocol: ['PWM', 0, 0, 'Mixer', 'Motor protocol', ['PWM', 'ONESHOT125', 'DSHOT150', 'DSHOT300', 'DSHOT600']],
    motor_pwm_rate: [200, 50, 32000, 'Mixer', 'Motor PWM update rate (Hz); the Pico driver caps it at 480'],
    motor_idle: [550, 0, 2000, 'Mixer', 'Idle throttle when armed (0.01 %)'],
    motor_output_limit: [100, 1, 100, 'Mixer', 'Motor output limit (%)'],
    min_command: [1000, 750, 2250, 'Mixer', 'Motor pulse when disarmed (us)'],
    servo_pwm_rate: [50, 50, 498, 'Mixer', 'Servo rate (ignored on the Pico target: fixed 50 Hz)'],
    servo_lowpass_hz: [0, 0, 400, 'Mixer', 'Servo output low-pass (0 = off)'],
    yaw_motors_reversed: ['OFF', 0, 0, 'Mixer', 'Yaw motor direction', ['OFF', 'ON']],
    // receiver
    min_check: [1050, 750, 2250, 'Receiver', 'Throttle below this counts as low (arming)'],
    max_check: [1900, 750, 2250, 'Receiver', 'Stick above this counts as high'],
    mid_rc: [1500, 1200, 1700, 'Receiver', 'Stick centre'],
    rx_min_usec: [885, 750, 2250, 'Receiver', 'Lowest valid channel value'],
    rx_max_usec: [2115, 750, 2250, 'Receiver', 'Highest valid channel value'],
    deadband: [0, 0, 32, 'Receiver', 'Roll/pitch deadband (us)'],
    yaw_deadband: [0, 0, 100, 'Receiver', 'Yaw deadband (us)'],
    rc_smoothing_hz: [60, 0, 250, 'Receiver', 'RC setpoint smoothing cut-off (sim: PT1)'],
    rc_smoothing_feedforward_hz: [20, 0, 250, 'Receiver', 'Feedforward smoothing cut-off (sim stand-in for auto)'],
    rx_frame_timeout_ms: [150, 50, 1000, 'Receiver', 'RXLOSS after this long without a valid frame (rxFrameCheck)'],
    // failsafe
    failsafe_delay: [5, 1, 200, 'Failsafe', 'Stage 1 length (0.1 s). BF default 15; project sets 5'],
    failsafe_off_delay: [10, 0, 200, 'Failsafe', 'AUTO-LAND duration (0.1 s)'],
    failsafe_throttle: [1000, 750, 2250, 'Failsafe', 'Throttle during AUTO-LAND (us)'],
    failsafe_procedure: ['DROP', 0, 0, 'Failsafe', 'Stage 2 action', ['AUTO-LAND', 'DROP']],
    failsafe_recovery_delay: [5, 1, 200, 'Failsafe', 'Valid signal needed before recovery (0.1 s)'],
    small_angle: [25, 0, 180, 'Failsafe', 'Arming blocked above this tilt (deg)'],
    // battery
    battery_meter: ['ADC', 0, 0, 'Battery', 'Voltage meter', ['NONE', 'ADC']],
    vbat_scale: [110, 0, 255, 'Battery', 'Voltage divider scale (110 = 10k/1k)'],
    vbat_max_cell_voltage: [430, 100, 500, 'Battery', 'Cell full (0.01 V), used for cell count'],
    vbat_warning_cell_voltage: [350, 0, 500, 'Battery', 'Warning (0.01 V/cell)'],
    vbat_min_cell_voltage: [330, 0, 500, 'Battery', 'Critical (0.01 V/cell)'],
    // board alignment
    align_board_roll: [0, -180, 360, 'Board', 'IMU alignment roll (deg)'],
    align_board_pitch: [0, -180, 360, 'Board', 'IMU alignment pitch (deg)'],
    align_board_yaw: [0, -180, 360, 'Board', 'IMU alignment yaw (deg)'],
    // RX SPI / compile-time (config.h) values exposed for experiments
    rx_spi_protocol: ['NRF24_BICOPTER', 0, 0, 'Board', 'RX SPI protocol', ['NRF24_BICOPTER', 'NONE']],
    rx_spi_bus: [2, 1, 2, 'Board', 'RX SPI bus (2 = SPI1)'],
    gyro_1_spibus: [1, 1, 2, 'Board', 'Gyro SPI bus (1 = SPI0)'],
    nrf24_spi_clk_hz: [1000000, 100000, 20000000, 'Board', 'config.h NRF24L01_MAX_SPI_CLK_HZ'],
    gyro_spi_clk_hz: [20000000, 100000, 50000000, 'Board', 'Gyro SPI clock request'],
    link_id: ['0xBE5336DF', 0, 0, 'Board', 'config.h BICOPTER_NRF24_LINK_ID'],
    pid_loop_hz: [1000, 250, 1000, 'Board', 'Simulated PID loop rate (Hz)'],
    debug_mode: ['RX_NRF24_BICOPTER', 0, 0, 'Board', 'Debug mode', ['NONE', 'RX_NRF24_BICOPTER', 'GYRO_SCALED']],
  };

  const MODES = { 0: 'ARM', 1: 'ANGLE', 2: 'HORIZON', 12: 'PASSTHRU', 27: 'FAILSAFE', 28: 'AIRMODE' };
  const SERVO_INPUTS = ['Stabilised ROLL', 'Stabilised PITCH', 'Stabilised YAW', 'Stabilised THROTTLE', 'RC ROLL', 'RC PITCH', 'RC YAW', 'RC THROTTLE'];
  // Betaflight built-in BI rules: {target, input, rate}
  const BI_SMIX = [{ target: 4, input: 2, rate: 100 }, { target: 4, input: 1, rate: -100 }, { target: 5, input: 2, rate: 100 }, { target: 5, input: 1, rate: 100 }];
  const BI_MMIX = [{ thr: 1, roll: 1, pitch: 0, yaw: 0 }, { thr: 1, roll: -1, pitch: 0, yaw: 0 }];

  const DEFAULT_RESOURCES = {
    'MOTOR 1': 'A10', 'MOTOR 2': 'A11', 'SERVO 1': 'A20', 'SERVO 2': 'A21',
    'SPI_SCK 1': 'A02', 'SPI_SDO 1': 'A03', 'SPI_SDI 1': 'A04', 'GYRO_CS 1': 'A05', 'GYRO_EXTI 1': 'A22',
    'SPI_SCK 2': 'A14', 'SPI_SDO 2': 'A15', 'SPI_SDI 2': 'A12', 'RX_SPI_CS 1': 'A13', 'RX_SPI_CE 1': 'A16', 'RX_SPI_EXTI 1': 'A17',
    'ADC_BATT 1': 'A26', 'LED 1': 'A25',
  };

  const DEFAULT_CUSTOM_CODE = `// Custom firmware hook: runs every PID loop after the mixer, before outputs.
// io.motor[0..1] and io.servo[0..1] are pulse widths in us; change them to test
// new logic before porting it to C. io.state persists between calls.
// Available: io.armed, io.dt, io.throttle (0..1), io.pid {roll,pitch,yaw},
// io.setpoint {roll,pitch,yaw} deg/s, io.gyro {roll,pitch,yaw} deg/s,
// io.attitude {roll,pitch,yaw} deg, io.rc[0..7] us, io.vbat V, io.tilt [L,R] deg.
//
// Example (disabled): compensate collective thrust for nacelle tilt.
// if (io.armed) {
//   const k = 1 / Math.cos((Math.abs(io.tilt[0]) + Math.abs(io.tilt[1])) / 2 * Math.PI / 180);
//   for (let i = 0; i < 2; i++) io.motor[i] = Math.min(2000, 1000 + (io.motor[i] - 1000) * k);
// }
`;

  function defaults() {
    const params = {};
    for (const k in P) params[k] = P[k][0];
    const servos = [];
    for (let i = 0; i < 8; i++) servos.push({ min: 1000, max: 2000, mid: 1500, rate: 100, fwd: -1, reversed: [] });
    return {
      params, servos,
      smix: BI_SMIX.map((r) => Object.assign({ speed: 0, min: 0, max: 100, box: 0 }, r)),
      mmix: BI_MMIX.map((r) => Object.assign({}, r)),
      aux: [{ mode: 0, ch: 0, lo: 1700, hi: 2100 }, { mode: 1, ch: 1, lo: 1300, hi: 2100 }],
      rxfail: [{ m: 'a' }, { m: 'a' }, { m: 'a' }, { m: 'a' }, { m: 'h' }, { m: 'h' }, { m: 'h' }, { m: 'h' }],
      resources: Object.assign({}, DEFAULT_RESOURCES),
      customCode: DEFAULT_CUSTOM_CODE,
      customEnabled: false,
    };
  }

  // ---------------------------------------------------------------- CLI
  function fmtParam(name, v) { return `set ${name} = ${v}`; }

  function setParam(fw, name, value) {
    const meta = P[name];
    if (!meta) return `###ERROR: Invalid name: ${name}###`;
    if (meta[5]) {
      const v = String(value).trim().toUpperCase();
      if (!meta[5].includes(v)) return `###ERROR: Invalid value. Allowed: ${meta[5].join(', ')}###`;
      fw.params[name] = v;
    } else if (typeof meta[0] === 'string') {
      fw.params[name] = String(value).trim();
    } else {
      const n = Number(value);
      if (!Number.isFinite(n)) return `###ERROR: Invalid value for ${name}###`;
      if (n < meta[1] || n > meta[2]) return `###ERROR: ${name} out of range ${meta[1]}..${meta[2]}###`;
      fw.params[name] = n;
    }
    return `${name} set to ${fw.params[name]}`;
  }

  function auxLine(i, a) { return `aux ${i} ${a.mode} ${a.ch} ${a.lo} ${a.hi} 0 0`; }
  function servoLine(i, s) { return `servo ${i} ${s.min} ${s.max} ${s.mid} ${s.rate} ${s.fwd}`; }
  function smixLine(i, r) { return `smix ${i} ${r.target} ${r.input} ${r.rate} ${r.speed || 0} ${r.min ?? 0} ${r.max ?? 100} ${r.box || 0}`; }
  function mmixLine(i, r) { return `mmix ${i} ${r.thr.toFixed(3)} ${r.roll.toFixed(3)} ${r.pitch.toFixed(3)} ${r.yaw.toFixed(3)}`; }
  function rxfailLine(i, r) { return `rxfail ${i} ${r.m}${r.m === 's' ? ' ' + r.v : ''}`; }

  function dump(fw, onlyDiff) {
    const d = defaults(), out = ['# Betaflight / BICOPTER_PICO2 (simulator) 2026.6.2', onlyDiff ? '# diff' : '# dump', '', 'batch start', ''];
    out.push('# resources');
    for (const k of Object.keys(fw.resources)) if (!onlyDiff || fw.resources[k] !== d.resources[k]) out.push(`resource ${k} ${fw.resources[k]}`);
    out.push('', `# mixer`, `mixer ${fw.params.mixer}`);
    if (fw.params.mixer === 'CUSTOM') fw.mmix.forEach((r, i) => out.push(mmixLine(i, r)));
    out.push('', '# servo mixer');
    if (fw.params.mixer === 'CUSTOM') fw.smix.forEach((r, i) => out.push(smixLine(i, r)));
    fw.servos.forEach((s, i) => s.reversed.forEach((src) => out.push(`smix reverse ${i} ${src} r`)));
    out.push('', '# servo');
    fw.servos.forEach((s, i) => { const ds = d.servos[i]; if (!onlyDiff || JSON.stringify(s) !== JSON.stringify(ds)) out.push(servoLine(i, s)); });
    out.push('', '# aux');
    if (!onlyDiff || JSON.stringify(fw.aux) !== JSON.stringify(d.aux)) fw.aux.forEach((a, i) => out.push(auxLine(i, a)));
    out.push('', '# rxfail');
    fw.rxfail.forEach((r, i) => { if (!onlyDiff || JSON.stringify(r) !== JSON.stringify(d.rxfail[i])) out.push(rxfailLine(i, r)); });
    out.push('', '# master');
    for (const k in P) if (!onlyDiff || fw.params[k] !== d.params[k]) out.push(fmtParam(k, fw.params[k]));
    out.push('', 'batch end', '', '# end the command batch with: save');
    return out.join('\n');
  }

  // Runs one or more CLI lines; returns the text output.
  function cli(fw, text) {
    const out = [];
    for (let raw of String(text).split(/\r?\n/)) {
      const line = raw.replace(/#.*$/, '').trim();
      if (!line) continue;
      out.push('# ' + line);
      const [cmd, ...rest] = line.split(/\s+/);
      const args = rest;
      const c = cmd.toLowerCase();
      try {
        if (c === 'set') {
          const m = line.match(/^set\s+([a-z0-9_]+)\s*=?\s*(.*)$/i);
          if (!m) { out.push('###ERROR: set <name> = <value>###'); continue; }
          out.push(setParam(fw, m[1].toLowerCase(), m[2]));
        } else if (c === 'get') {
          const q = (args[0] || '').toLowerCase();
          const hits = Object.keys(P).filter((k) => k.includes(q));
          if (!hits.length) out.push('###ERROR: Invalid name###');
          for (const k of hits) {
            const meta = P[k];
            out.push(`${k} = ${fw.params[k]}`);
            out.push(meta[5] ? `Allowed values: ${meta[5].join(', ')}` : (typeof meta[0] === 'number' ? `Allowed range: ${meta[1]} - ${meta[2]}` : 'String'));
            out.push(`Default value: ${meta[0]}   (${meta[4]})`);
          }
        } else if (c === 'diff') out.push(dump(fw, true));
        else if (c === 'dump') out.push(dump(fw, false));
        else if (c === 'defaults') {
          const keepCode = fw.customCode; const d = defaults(); Object.assign(fw, d); fw.customCode = keepCode;
          out.push('Defaults restored (custom code kept).');
        } else if (c === 'save') out.push('Saving... (simulator: configuration stored, reboot simulated on next reset)');
        else if (c === 'aux') {
          if (!args.length) fw.aux.forEach((a, i) => out.push(auxLine(i, a)));
          else {
            const [i, mode, ch, lo, hi] = args.map(Number);
            if ([i, mode, ch, lo, hi].some((x) => !Number.isFinite(x))) { out.push('###ERROR: aux <idx> <modeId> <auxCh> <low> <high>###'); continue; }
            if (hi <= 900 && lo <= 900) { fw.aux.splice(i, 1); out.push(`aux ${i} removed`); continue; }
            fw.aux[i] = { mode, ch, lo, hi };
            fw.aux = fw.aux.filter(Boolean);
            out.push(auxLine(i, fw.aux[Math.min(i, fw.aux.length - 1)]) + `   (${MODES[mode] || 'mode ' + mode} on AUX${ch + 1})`);
          }
        } else if (c === 'servo') {
          if (!args.length) fw.servos.forEach((s, i) => out.push(servoLine(i, s)));
          else {
            const [i, min, max, mid, rate, fwd] = args.map(Number);
            if (!(i >= 0 && i < 8) || [min, max, mid, rate].some((x) => !Number.isFinite(x))) { out.push('###ERROR: servo <idx> <min> <max> <mid> <rate> <fwd>###'); continue; }
            if (!(min < mid && mid < max) || rate < -100 || rate > 100) { out.push('###ERROR: need min < mid < max and -100 <= rate <= 100###'); continue; }
            Object.assign(fw.servos[i], { min, max, mid, rate, fwd: Number.isFinite(fwd) ? fwd : -1 });
            out.push(servoLine(i, fw.servos[i]));
          }
        } else if (c === 'smix') {
          if (!args.length) { fw.smix.forEach((r, i) => out.push(smixLine(i, r))); if (fw.params.mixer !== 'CUSTOM') out.push('# note: mixer BICOPTER uses the built-in BI rules; these custom rules are ignored'); }
          else if (args[0] === 'reset') { fw.smix = []; out.push('servo mixer rules cleared'); }
          else if (args[0] === 'load') { fw.smix = BI_SMIX.map((r) => Object.assign({ speed: 0, min: 0, max: 100, box: 0 }, r)); out.push('BI rules loaded into custom table'); }
          else if (args[0] === 'reverse') {
            if (args.length === 1) { fw.servos.forEach((s, i) => out.push(`servo ${i} reversed inputs: [${s.reversed.join(',')}]`)); continue; }
            const i = Number(args[1]), src = Number(args[2]), r = (args[3] || '').toLowerCase();
            const s = fw.servos[i];
            if (!s || !(src >= 0 && src < 8)) { out.push('###ERROR: smix reverse <servo> <input> r|n###'); continue; }
            s.reversed = s.reversed.filter((x) => x !== src);
            if (r === 'r') s.reversed.push(src);
            out.push(`servo ${i} input ${src} (${SERVO_INPUTS[src]}) ${r === 'r' ? 'reversed' : 'normal'}`);
          } else {
            const [i, target, input, rate, speed, min, max, box] = args.map(Number);
            if (![i, target, input, rate].every(Number.isFinite)) { out.push('###ERROR: smix <rule> <servo> <input> <rate> [speed min max box]###'); continue; }
            fw.smix[i] = { target, input, rate, speed: speed || 0, min: Number.isFinite(min) ? min : 0, max: Number.isFinite(max) ? max : 100, box: box || 0 };
            fw.smix = fw.smix.filter(Boolean);
            out.push(smixLine(i, fw.smix[Math.min(i, fw.smix.length - 1)]));
            if (fw.params.mixer !== 'CUSTOM') out.push('# note: set mixer = CUSTOM for these rules to take effect');
          }
        } else if (c === 'mmix') {
          if (!args.length) fw.mmix.forEach((r, i) => out.push(mmixLine(i, r)));
          else if (args[0] === 'reset') { fw.mmix = []; out.push('motor mix cleared'); }
          else if (args[0] === 'load') { fw.mmix = BI_MMIX.map((r) => Object.assign({}, r)); out.push('BI motor mix loaded'); }
          else {
            const [i, thr, roll, pitch, yaw] = args.map(Number);
            if (![i, thr, roll, pitch, yaw].every(Number.isFinite) || i > 1) { out.push('###ERROR: mmix <0|1> <thr> <roll> <pitch> <yaw>###'); continue; }
            fw.mmix[i] = { thr, roll, pitch, yaw };
            out.push(mmixLine(i, fw.mmix[i]));
          }
        } else if (c === 'rxfail') {
          if (!args.length) fw.rxfail.forEach((r, i) => out.push(rxfailLine(i, r)));
          else {
            const i = Number(args[0]), m = (args[1] || '').toLowerCase();
            if (!(i >= 0 && i < 8) || !['a', 'h', 's'].includes(m)) { out.push('###ERROR: rxfail <ch> <a|h|s> [value]###'); continue; }
            if (m === 'a' && i >= 4) { out.push('###ERROR: auto is only for channels 0-3###'); continue; }
            fw.rxfail[i] = m === 's' ? { m, v: Number(args[2]) || 1500 } : { m };
            out.push(rxfailLine(i, fw.rxfail[i]));
          }
        } else if (c === 'resource') {
          if (!args.length || args[0] === 'show') Object.keys(fw.resources).forEach((k) => out.push(`resource ${k} ${fw.resources[k]}`));
          else {
            const key = `${args[0].toUpperCase()} ${args[1]}`, pin = (args[2] || '').toUpperCase();
            if (pin === 'NONE') { delete fw.resources[key]; out.push(`resource ${key} freed`); }
            else if (/^A\d{1,2}$/.test(pin)) { fw.resources[key] = 'A' + String(Number(pin.slice(1))).padStart(2, '0'); out.push(`resource ${key} ${fw.resources[key]}`); }
            else out.push('###ERROR: resource <NAME> <index> <A00..A29|NONE>###');
          }
        } else if (c === 'mixer') {
          if (!args.length) out.push(`Mixer: ${fw.params.mixer}`);
          else out.push(setParam(fw, 'mixer', args[0]));
        } else if (c === 'status') {
          out.push('MCU RP2350A clock=150MHz (simulated), target BICOPTER_PICO2, Betaflight 2026.6.2 + NRF24_BICOPTER v2');
        } else if (c === 'version') {
          out.push('# Betaflight / RP2350A (simulated) 2026.6.2 e0b7bb01 + bicopter-nrf24 v2');
        } else if (c === 'help') {
          out.push('Commands: set, get, diff, dump, defaults, save, aux, servo, smix [reset|load|reverse], mmix [reset|load], rxfail, resource, mixer, status, version');
          out.push('Modes for aux: ' + Object.entries(MODES).map(([k, v]) => `${k}=${v}`).join(', '));
          out.push('Servo inputs for smix: ' + SERVO_INPUTS.map((v, i) => `${i}=${v}`).join(', '));
        } else if (['batch', 'profile', 'rateprofile', 'feature', 'serial', 'map', 'beacon', 'beeper', 'board_name', 'manufacturer_id', 'exit', 'led', 'color', 'mode_color', 'vtxtable', 'vtx', 'adjrange', 'dma', 'timer'].includes(c)) {
          out.push(`(${c} accepted, not modelled)`);
        } else out.push(`###ERROR: UNKNOWN COMMAND: ${cmd}###`);
      } catch (e) { out.push('###ERROR: ' + e.message + '###'); }
    }
    return out.join('\n');
  }

  // ---------------------------------------------------------------- flight controller runtime
  function rotEuler(rd, pd, yd) { // rotation matrix for alignment angles (deg), ZYX
    const r = rd * U.DEG, p = pd * U.DEG, y = yd * U.DEG;
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr];
  }

  class FlightController {
    constructor(fw) { this.fw = fw; this.reset(0); }

    reset(now) {
      const p = this.fw.params, dt = 1 / p.pid_loop_hz;
      this.dt = dt;
      this.bootTime = now;
      this.armed = false;
      this.armSwitchPrev = false;
      this.armSwitchSeenOff = false;
      this.disarmReason = 'boot';
      this.flags = new Set(['CALIBRATING']);
      this.fs = { phase: 'IDLE', start: 0, validSince: null, landingUntil: 0 };
      this.lastFrameTime = -1e9;
      this.rcData = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000];
      this.rcRaw = this.rcData.slice();
      this.rxSignal = false;
      this.q = [1, 0, 0, 0];
      this.att = [0, 0, 0];
      this.gyroLpf = [0, 1, 2].map(() => [new U.PT1(p.gyro_lpf1_static_hz, dt), new U.PT1(p.gyro_lpf2_static_hz, dt)]);
      this.dLpf = [0, 1, 2].map(() => [new U.PT1(p.dterm_lpf1_static_hz, dt), new U.PT1(p.dterm_lpf2_static_hz, dt)]);
      this.spLpf = [0, 1, 2].map(() => new U.PT1(p.rc_smoothing_hz, dt));
      this.ffLpf = [0, 1, 2].map(() => new U.PT1(p.rc_smoothing_feedforward_hz, dt));
      this.servoLpf = [0, 1].map(() => new U.PT1(p.servo_lowpass_hz, dt));
      this.iterm = [0, 0, 0];
      this.prevGyro = [0, 0, 0];
      this.prevSp = [0, 0, 0];
      this.pid = [{}, {}, {}].map(() => ({ P: 0, I: 0, D: 0, F: 0, sum: 0 }));
      this.motorCmd = [p.min_command, p.min_command];
      this.servoCmd = [1500, 1500];
      this.motorOut = this.motorCmd.slice();
      this.servoOut = this.servoCmd.slice();
      this.nextMotorUpdate = now; this.nextServoUpdate = now;
      this.vbatFilt = null; this.cells = 0;
      this.batWarn = '';
      this.events = [];
      this.customState = {};
      this.customError = null;
      this._compileCustom();
      this.throttle = 0;
      this.setpoint = [0, 0, 0];
      this.gyroDeg = [0, 0, 0];
      this.boxes = new Set();
      this.align = rotEuler(p.align_board_roll, p.align_board_pitch, p.align_board_yaw);
    }

    _compileCustom() {
      this.customFn = null;
      if (!this.fw.customEnabled) return;
      try { this.customFn = new Function('io', 'Math', this.fw.customCode); } catch (e) { this.customError = 'Compile: ' + e.message; }
    }

    log(t, msg, level = 'info') { this.events.push({ t, msg, level }); if (this.events.length > 400) this.events.shift(); }

    // Called by the receiver adapter for every accepted frame.
    onFrame(now, channels) { this.lastFrameTime = now; this.rcRaw = channels.slice(); }

    // Project addition (failsafeOnReceiverReset): immediate disarm on a handset reboot.
    onReceiverReset(now) {
      if (this.armed) { this.disarm(now, 'FAILSAFE (handset restart / new session)'); }
      this.flags.add('FAILSAFE'); this.flags.add('ARM_SWITCH'); this.armSwitchSeenOff = false;
      this.fs = { phase: 'RX_LOSS_MONITORING', start: now, validSince: null };
      this.log(now, 'failsafeOnReceiverReset(): handset restart detected → immediate disarm', 'warn');
    }

    disarm(now, reason) {
      if (this.armed) this.log(now, 'DISARMED: ' + reason, reason.startsWith('switch') ? 'info' : 'warn');
      this.armed = false; this.disarmReason = reason;
      this.iterm = [0, 0, 0];
    }

    auxActive(mode) {
      for (const a of this.fw.aux) if (a.mode === mode) { const v = this.rcData[4 + a.ch]; if (v >= a.lo && v < a.hi) return true; }
      return false;
    }

    rxfailValue(i) {
      const r = this.fw.rxfail[i], p = this.fw.params;
      if (r.m === 's') return r.v;
      if (r.m === 'h') return this.rcRaw[i];
      return i === 2 ? p.rx_min_usec : p.mid_rc;
    }

    // sensors: {gyro:[p,q,r] rad/s (sensor frame), acc:[..] m/s^2 (sensor frame), vbatAdc: V}
    step(now, sensors) {
      const p = this.fw.params, dt = this.dt;
      if (now - this.bootTime > 1.0) this.flags.delete('CALIBRATING');

      // ---- sensors through board alignment
      const g = U.mat3MulV(this.align, sensors.gyro), a = U.mat3MulV(this.align, sensors.acc);
      const gyroDegRaw = g.map((x) => x * U.RAD);
      this.gyroDeg = gyroDegRaw.map((x, i) => this.gyroLpf[i][1].apply(this.gyroLpf[i][0].apply(x)));

      // ---- attitude estimator (Mahony, accelerometer only)
      const kp = (now - this.bootTime < 2.0) ? 10 : p.imu_dcm_kp / 10000;
      let wc = g.slice();
      const an = U.norm(a);
      if (an > 0.5 * U.G && an < 1.5 * U.G) {
        const up = U.qRotInv(this.q, [0, 0, 1]);           // estimated "up" in body frame
        const err = U.cross(U.scale(a, 1 / an), up);       // rotate estimate toward measured
        wc = U.add(wc, U.scale(err, kp));
      }
      this.q = U.qIntegrate(this.q, wc, dt);
      this.att = U.qToEuler(this.q).map((x) => x * U.RAD);

      // ---- RX watchdog, rxfail and failsafe
      const timeout = p.rx_frame_timeout_ms / 1000;
      const signal = (now - this.lastFrameTime) <= timeout;
      if (signal !== this.rxSignal) { this.log(now, signal ? 'RX signal OK' : `RXLOSS (no valid frame for ${p.rx_frame_timeout_ms} ms)`, signal ? 'info' : 'warn'); }
      this.rxSignal = signal;
      if (signal) this.rcData = this.rcRaw.slice(); else this.rcData = this.rcData.map((_, i) => this.rxfailValue(i));
      if (!signal) this.flags.add('RXLOSS'); else this.flags.delete('RXLOSS');
      this._failsafe(now, signal);
      if (this.auxActive(27)) this.flags.add('BOXFAILSAFE'); else this.flags.delete('BOXFAILSAFE');

      // ---- arming
      const armSw = this.auxActive(0);
      if (!armSw) { this.armSwitchSeenOff = true; this.flags.delete('ARM_SWITCH'); }
      else if (!this.armSwitchSeenOff) this.flags.add('ARM_SWITCH');
      if (this.rcData[2] >= p.min_check) this.flags.add('THROTTLE'); else this.flags.delete('THROTTLE');
      const tilt = Math.acos(U.clamp(Math.cos(this.att[0] * U.DEG) * Math.cos(this.att[1] * U.DEG), -1, 1)) * U.RAD;
      if (tilt > p.small_angle) this.flags.add('ANGLE'); else this.flags.delete('ANGLE');
      if (p.rx_spi_protocol !== 'NRF24_BICOPTER') this.flags.add('RX_PROTOCOL'); else this.flags.delete('RX_PROTOCOL');
      if (!this.armed && armSw && !this.armSwitchPrev) {
        const block = [...this.flags];
        if (block.length === 0) { this.armed = true; this.iterm = [0, 0, 0]; this.log(now, 'ARMED'); }
        else { this.log(now, 'Arming blocked: ' + block.join(', ') + ' (switch ARM off, then on again)', 'warn'); this.armSwitchSeenOff = false; }
      }
      if (this.armed && !armSw) this.disarm(now, 'switch');
      if (this.armed && this.flags.has('BOXFAILSAFE')) this.disarm(now, 'BOXFAILSAFE');
      this.armSwitchPrev = armSw;

      // ---- RC commands
      const mid = p.mid_rc;
      const stick = (i, db) => { let d = this.rcData[i] - mid; d = Math.abs(d) <= db ? 0 : d - Math.sign(d) * db; return U.clamp(d / (500 - db), -1, 1); };
      let rcIn = [stick(0, p.deadband), stick(1, p.deadband), -stick(3, p.yaw_deadband)]; // Betaflight negates yaw
      let thrIn = U.clamp((Math.max(this.rcData[2], p.min_check) - p.min_check) / (2000 - p.min_check), 0, 1);
      // throttle curve
      const tm = p.thr_mid / 100, te = p.thr_expo / 100, dx = thrIn - tm, span = dx > 0 ? 1 - tm : tm;
      thrIn = tm + dx * (1 - te + te * (dx * dx) / (span * span || 1));
      thrIn *= p.throttle_limit_percent / 100;
      const angleMode = this.auxActive(1) || this.auxActive(2) || this.fs.phase === 'LANDING';
      if (this.fs.phase === 'LANDING') { rcIn = [0, 0, 0]; thrIn = U.clamp((p.failsafe_throttle - 1000) / 1000, 0, 1); }
      this.throttle = thrIn;

      // ---- setpoints (ACTUAL rates)
      const rateKeys = [['roll_rc_rate', 'roll_srate', 'roll_expo'], ['pitch_rc_rate', 'pitch_srate', 'pitch_expo'], ['yaw_rc_rate', 'yaw_srate', 'yaw_expo']];
      const sp = [0, 1, 2].map((ax) => {
        const [rk, sk, ek] = rateKeys[ax], x = rcIn[ax], ax_ = Math.abs(x), e = p[ek] / 100;
        const center = p[rk] * 10, maxR = Math.max(0, p[sk] * 10 - center);
        const expof = ax_ * (Math.pow(x, 5) * e + x * (1 - e));
        return x * center + maxR * expof;
      });
      if (angleMode) {
        for (const ax of [0, 1]) {
          const target = rcIn[ax] * p.angle_limit, errA = target - this.att[ax];
          sp[ax] = U.clamp(errA * p.angle_p_gain / 10, -p[rateKeys[ax][1]] * 10, p[rateKeys[ax][1]] * 10);
        }
      }
      for (let i = 0; i < 3; i++) sp[i] = this.spLpf[i].apply(sp[i]);
      this.setpoint = sp;

      // ---- PID (Betaflight scaling)
      const gains = [['p_roll', 'i_roll', 'd_roll', 'f_roll'], ['p_pitch', 'i_pitch', 'd_pitch', 'f_pitch'], ['p_yaw', 'i_yaw', 'd_yaw', 'f_yaw']];
      const thrUs = 1000 + thrIn * 1000;
      const tpa = thrUs > p.tpa_breakpoint ? 1 - (p.tpa_rate / 100) * Math.min(1, (thrUs - p.tpa_breakpoint) / (2000 - p.tpa_breakpoint)) : 1;
      const airmode = p.airmode === 'ON' || this.auxActive(28);
      const stabilise = this.armed;
      const resetI = !stabilise || (!airmode && thrIn < 0.01);
      for (let ax = 0; ax < 3; ax++) {
        const [pk, ik, dk, fk] = gains[ax];
        const err = sp[ax] - this.gyroDeg[ax];
        const kP = p[pk] * 0.032029 * (p.tpa_mode === 'PD' ? tpa : 1);
        const P_ = kP * err;
        if (resetI) this.iterm[ax] = 0;
        else this.iterm[ax] = U.clamp(this.iterm[ax] + p[ik] * 0.244381 * err * dt, -p.iterm_limit, p.iterm_limit);
        const dg = this.dLpf[ax][1].apply(this.dLpf[ax][0].apply(this.gyroDeg[ax]));
        const D_ = -(p[dk] * 0.000529 * tpa) * (dg - this.prevGyro[ax]) / dt;
        this.prevGyro[ax] = dg;
        const F_ = this.ffLpf[ax].apply(p[fk] * 0.013754 * 0.01 * (sp[ax] - this.prevSp[ax]) / dt);
        this.prevSp[ax] = sp[ax];
        const lim = ax === 2 ? p.pidsum_limit_yaw : p.pidsum_limit;
        const sum = stabilise ? U.clamp(P_ + this.iterm[ax] + D_ + F_, -lim, lim) : 0;
        this.pid[ax] = { P: stabilise ? P_ : 0, I: stabilise ? this.iterm[ax] : 0, D: stabilise ? D_ : 0, F: stabilise ? F_ : 0, sum };
      }

      // ---- motor mixer
      const mm = p.mixer === 'CUSTOM' ? this.fw.mmix : BI_MMIX;
      const yawSign = p.yaw_motors_reversed === 'ON' ? -1 : 1;
      const r = this.pid[0].sum / 1000, pi = this.pid[1].sum / 1000, y = yawSign * this.pid[2].sum / 1000;
      let mix = [0, 1].map((i) => mm[i] ? r * mm[i].roll + pi * mm[i].pitch + y * mm[i].yaw : 0);
      const mx = Math.max(...mix), mn = Math.min(...mix), range = mx - mn;
      let thr = thrIn;
      if (range > 1) { mix = mix.map((v) => v / range); if (airmode) thr = 0.5; }
      else if (airmode || thr > 0.5) thr = U.clamp(thr, -mn, 1 - mx);
      const low = 1000 + 1000 * p.motor_idle / 10000, high = 1000 + 1000 * p.motor_output_limit / 100;
      const motors = [0, 1].map((i) => {
        if (!this.armed || !mm[i]) return p.min_command;
        return U.clamp(low + (high - low) * (mix[i] + thr * mm[i].thr), low, high);
      });

      // ---- servo mixer
      const passthru = this.auxActive(12);
      const inputs = new Array(8).fill(0);
      if (passthru) { inputs[0] = this.rcData[0] - mid; inputs[1] = this.rcData[1] - mid; inputs[2] = this.rcData[3] - mid; }
      else { inputs[0] = this.pid[0].sum * 0.7; inputs[1] = this.pid[1].sum * 0.7; inputs[2] = this.pid[2].sum * 0.7; }
      inputs[3] = motors[0] - 1500; inputs[4] = this.rcData[0] - mid; inputs[5] = this.rcData[1] - mid; inputs[6] = this.rcData[3] - mid; inputs[7] = this.rcData[2] - mid;
      const rules = p.mixer === 'CUSTOM' ? this.fw.smix : BI_SMIX.map((q) => Object.assign({ min: 0, max: 100, box: 0 }, q));
      const sv = new Array(8).fill(0);
      for (const rule of rules) {
        if (rule.box && !this.auxActive(rule.box)) continue; // simplified box handling
        const sp_ = this.fw.servos[rule.target]; if (!sp_) continue;
        const w = sp_.max - sp_.min, lo = (rule.min ?? 0) * w / 100 - w / 2, hi = (rule.max ?? 100) * w / 100 - w / 2;
        const dir = sp_.reversed.includes(rule.input) ? -1 : 1;
        sv[rule.target] += dir * U.clamp(inputs[rule.input] * rule.rate / 100, lo, hi);
      }
      // Betaflight puts BI servos on outputs 4/5 internally; this target maps them to SERVO 1/2.
      const servoIdx = [4, 5];
      let servos = servoIdx.map((idx, k) => {
        const s = this.fw.servos[idx] || this.fw.servos[k];
        let v = s.rate * sv[idx] / 100 + s.mid;
        if (p.servo_lowpass_hz > 0) v = this.servoLpf[k].apply(v);
        return U.clamp(v, s.min, s.max);
      });

      // ---- custom hook
      if (this.customFn) {
        const io = { armed: this.armed, dt, throttle: thrIn, motor: motors.slice(), servo: servos.slice(),
          pid: { roll: this.pid[0].sum, pitch: this.pid[1].sum, yaw: this.pid[2].sum },
          setpoint: { roll: sp[0], pitch: sp[1], yaw: sp[2] }, gyro: { roll: this.gyroDeg[0], pitch: this.gyroDeg[1], yaw: this.gyroDeg[2] },
          attitude: { roll: this.att[0], pitch: this.att[1], yaw: this.att[2] }, rc: this.rcData.slice(), vbat: this.vbatFilt || 0,
          tilt: sensors.tiltDeg || [0, 0], state: this.customState, time: now };
        try {
          this.customFn(io, Math);
          for (let i = 0; i < 2; i++) {
            if (Number.isFinite(io.motor[i])) motors[i] = U.clamp(io.motor[i], 1000, 2000);
            if (Number.isFinite(io.servo[i])) servos[i] = U.clamp(io.servo[i], 500, 2500);
          }
          if (!this.armed) motors[0] = motors[1] = p.min_command; // safety: hook cannot spin disarmed motors
        } catch (e) { this.customError = 'Runtime: ' + e.message; this.customFn = null; this.log(now, 'Custom code disabled: ' + e.message, 'error'); }
      }
      this.motorCmd = motors; this.servoCmd = servos;

      // ---- output timers: motor PWM at motor_pwm_rate (capped at 480 by the Pico driver), servos fixed 50 Hz
      const mRate = Math.min(480, p.motor_pwm_rate);
      if (now >= this.nextMotorUpdate) { this.motorOut = motors.slice(); this.nextMotorUpdate += 1 / mRate; if (this.nextMotorUpdate < now) this.nextMotorUpdate = now + 1 / mRate; }
      if (now >= this.nextServoUpdate) { this.servoOut = servos.slice(); this.nextServoUpdate += 1 / 50; if (this.nextServoUpdate < now) this.nextServoUpdate = now + 0.02; }

      // ---- battery
      if (p.battery_meter === 'ADC') {
        const v = sensors.vbatAdc * p.vbat_scale / 10;
        this.vbatFilt = this.vbatFilt === null ? v : this.vbatFilt + (v - this.vbatFilt) * Math.min(1, dt * 2 * Math.PI * 1);
        if (!this.cells && now - this.bootTime > 0.5) this.cells = Math.max(1, Math.ceil(this.vbatFilt / (p.vbat_max_cell_voltage / 100)));
        const cellV = this.cells ? this.vbatFilt / this.cells : 0;
        const w = !this.cells ? '' : cellV < p.vbat_min_cell_voltage / 100 ? 'CRITICAL' : cellV < p.vbat_warning_cell_voltage / 100 ? 'LOW' : '';
        if (w !== this.batWarn && w) this.log(now, `Battery ${w}: ${cellV.toFixed(2)} V/cell (sensed)`, 'warn');
        this.batWarn = w;
      }
      return { motorUs: this.motorOut, servoUs: this.servoOut };
    }

    _failsafe(now, signal) {
      const p = this.fw.params, fs = this.fs;
      switch (fs.phase) {
        case 'IDLE':
          if (!signal && this.armed) { fs.phase = 'STAGE1'; fs.start = now; this.log(now, `Failsafe stage 1 (rxfail values for ${p.failsafe_delay / 10} s)`, 'warn'); }
          break;
        case 'STAGE1':
          if (signal) { fs.phase = 'IDLE'; this.log(now, 'Signal back inside stage 1: control resumes'); break; }
          if (!this.armed) { fs.phase = 'IDLE'; break; }
          if (now - fs.start >= p.failsafe_delay / 10) {
            this.flags.add('FAILSAFE');
            if (p.failsafe_procedure === 'DROP') { this.disarm(now, 'FAILSAFE stage 2 DROP'); fs.phase = 'RX_LOSS_MONITORING'; fs.validSince = null; }
            else { fs.phase = 'LANDING'; fs.landingUntil = now + p.failsafe_off_delay / 10; this.log(now, 'Failsafe stage 2: AUTO-LAND', 'warn'); }
          }
          break;
        case 'LANDING':
          if (now >= fs.landingUntil) { this.disarm(now, 'FAILSAFE landing time elapsed'); fs.phase = 'RX_LOSS_MONITORING'; fs.validSince = null; }
          else if (signal) { fs.validSince = fs.validSince ?? now; if (now - fs.validSince >= p.failsafe_recovery_delay / 10) { fs.phase = 'IDLE'; this.flags.delete('FAILSAFE'); this.log(now, 'Failsafe recovered during landing'); } }
          else fs.validSince = null;
          break;
        case 'RX_LOSS_MONITORING':
          if (signal) {
            fs.validSince = fs.validSince ?? now;
            if (now - fs.validSince >= p.failsafe_recovery_delay / 10) {
              fs.phase = 'IDLE'; this.flags.delete('FAILSAFE');
              this.log(now, 'Failsafe recovery complete; switch ARM off then on to re-arm');
              if (this.auxActive(0)) { this.armSwitchSeenOff = false; this.flags.add('ARM_SWITCH'); }
            }
          } else fs.validSince = null;
          break;
      }
    }
  }

  S.Firmware = { PARAMS: P, MODES, SERVO_INPUTS, BI_SMIX, BI_MMIX, DEFAULT_RESOURCES, DEFAULT_CUSTOM_CODE, defaults, cli, dump, setParam, FlightController, rotEuler };
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
