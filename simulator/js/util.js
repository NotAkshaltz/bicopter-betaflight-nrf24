// Shared math helpers. Every module attaches itself to the global `Sim` namespace so the
// app runs from a double-clicked index.html (no bundler, no server) and also under Node.
(function (S) {
  'use strict';

  const U = {};
  U.DEG = Math.PI / 180;
  U.RAD = 180 / Math.PI;
  U.G = 9.80665;
  U.RHO_SEA = 1.225;

  U.clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
  U.lerp = (a, b, t) => a + (b - a) * t;
  U.sign = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);
  U.round = (x, n = 2) => { const p = Math.pow(10, n); return Math.round(x * p) / p; };
  U.deepCopy = (o) => JSON.parse(JSON.stringify(o));

  // Piecewise-linear interpolation over sorted [x, y] pairs.
  U.interp = (table, x) => {
    if (x <= table[0][0]) return table[0][1];
    for (let i = 1; i < table.length; i++) {
      if (x <= table[i][0]) {
        const [x0, y0] = table[i - 1], [x1, y1] = table[i];
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
      }
    }
    return table[table.length - 1][1];
  };

  // ---- vectors (plain arrays [x, y, z]) ----
  U.v = (x = 0, y = 0, z = 0) => [x, y, z];
  U.add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  U.sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  U.scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  U.dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  U.cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  U.norm = (a) => Math.sqrt(U.dot(a, a));
  U.unit = (a) => { const n = U.norm(a); return n > 1e-12 ? U.scale(a, 1 / n) : [0, 0, 0]; };

  // ---- 3x3 matrices (row-major arrays of 9) ----
  U.mat3MulV = (m, v) => [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
  U.mat3Inv = (m) => {
    const [a, b, c, d, e, f, g, h, i] = m;
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const det = a * A + b * B + c * C;
    if (Math.abs(det) < 1e-15) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
    const s = 1 / det;
    return [A * s, -(b * i - c * h) * s, (b * f - c * e) * s,
            B * s, (a * i - c * g) * s, -(a * f - c * d) * s,
            C * s, -(a * h - b * g) * s, (a * e - b * d) * s];
  };

  // ---- quaternions [w, x, y, z], body -> world ----
  U.qMul = (a, b) => [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
  U.qNorm = (q) => { const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]; };
  U.qConj = (q) => [q[0], -q[1], -q[2], -q[3]];
  U.qRot = (q, v) => { const p = U.qMul(U.qMul(q, [0, v[0], v[1], v[2]]), U.qConj(q)); return [p[1], p[2], p[3]]; };
  U.qRotInv = (q, v) => U.qRot(U.qConj(q), v);
  U.qFromAxisAngle = (ax, ang) => { const s = Math.sin(ang / 2); return [Math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s]; };
  // Integrate body rates w (rad/s) over dt.
  U.qIntegrate = (q, w, dt) => {
    const ang = U.norm(w) * dt;
    if (ang < 1e-12) return q;
    return U.qNorm(U.qMul(q, U.qFromAxisAngle(U.unit(w), ang)));
  };
  // Euler angles (roll about x, pitch about y, yaw about z), ZYX order, right-handed FLU frame:
  // roll + = right side down, pitch + = nose down, yaw + = nose left (counter-clockwise from above).
  U.qToEuler = (q) => {
    const [w, x, y, z] = q;
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const sp = U.clamp(2 * (w * y - z * x), -1, 1);
    const pitch = Math.asin(sp);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return [roll, pitch, yaw];
  };
  U.qFromEuler = (r, p, y) => {
    const cr = Math.cos(r / 2), sr = Math.sin(r / 2), cp = Math.cos(p / 2), sp = Math.sin(p / 2), cy = Math.cos(y / 2), sy = Math.sin(y / 2);
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
  };

  // ---- filters ----
  class PT1 {
    constructor(cutoffHz, dt) { this.y = 0; this.set(cutoffHz, dt); }
    set(cutoffHz, dt) {
      if (!cutoffHz || cutoffHz <= 0) { this.k = 1; return; }
      const rc = 1 / (2 * Math.PI * cutoffHz);
      this.k = dt / (rc + dt);
    }
    apply(x) { this.y += this.k * (x - this.y); return this.y; }
    reset(v = 0) { this.y = v; }
  }
  U.PT1 = PT1;

  // Second-order Butterworth low-pass (Betaflight's BIQUAD Q = 1/sqrt(2)).
  class Biquad {
    constructor(cutoffHz, sampleHz) { this.x1 = this.x2 = this.y1 = this.y2 = 0; this.set(cutoffHz, sampleHz); }
    set(cutoffHz, sampleHz) {
      this.bypass = !cutoffHz || cutoffHz <= 0 || cutoffHz >= sampleHz / 2;
      if (this.bypass) return;
      const w0 = 2 * Math.PI * cutoffHz / sampleHz, cs = Math.cos(w0), sn = Math.sin(w0);
      const alpha = sn / (2 * Math.SQRT1_2), a0 = 1 + alpha;
      this.b0 = (1 - cs) / 2 / a0; this.b1 = (1 - cs) / a0; this.b2 = this.b0;
      this.a1 = -2 * cs / a0; this.a2 = (1 - alpha) / a0;
    }
    apply(x) {
      if (this.bypass) return x;
      const y = this.b0 * x + this.b1 * this.x1 + this.b2 * this.x2 - this.a1 * this.y1 - this.a2 * this.y2;
      this.x2 = this.x1; this.x1 = x; this.y2 = this.y1; this.y1 = y;
      return y;
    }
  }
  U.Biquad = Biquad;

  // Seeded RNG so a test run is reproducible.
  U.rng = (seed) => {
    let a = seed >>> 0;
    const next = () => {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    next.gauss = () => {
      let u = 0, v = 0;
      while (u === 0) u = next();
      while (v === 0) v = next();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    };
    return next;
  };

  U.fmt = (x, n = 1, unit = '') => (x === undefined || x === null || Number.isNaN(x)) ? '—' : `${Number(x).toFixed(n)}${unit}`;

  S.U = U;
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
