// Minimal canvas 3D renderer for the airframe (no WebGL, no libraries).
(function (S) {
  'use strict';
  const U = S.U;

  class View3D {
    constructor(canvas) {
      this.c = canvas; this.g = canvas.getContext('2d');
      this.yaw = -2.4; this.pitch = 0.35; this.dist = 2.2; this.follow = true; this.target = [0, 0, 0.5];
      this.showVectors = true; this.showCG = true;
      let drag = null;
      canvas.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, yaw: this.yaw, pitch: this.pitch }; canvas.setPointerCapture(e.pointerId); });
      canvas.addEventListener('pointermove', (e) => { if (!drag) return; this.yaw = drag.yaw - (e.clientX - drag.x) * 0.008; this.pitch = U.clamp(drag.pitch + (e.clientY - drag.y) * 0.006, -0.2, 1.45); });
      canvas.addEventListener('pointerup', () => { drag = null; });
      canvas.addEventListener('wheel', (e) => { e.preventDefault(); this.dist = U.clamp(this.dist * Math.exp(e.deltaY * 0.001), 0.5, 40); }, { passive: false });
    }
    resize() {
      const r = this.c.getBoundingClientRect(), d = window.devicePixelRatio || 1;
      if (this.c.width !== Math.round(r.width * d) || this.c.height !== Math.round(r.height * d)) { this.c.width = Math.round(r.width * d); this.c.height = Math.round(r.height * d); }
      this.W = this.c.width; this.H = this.c.height; this.dpr = d;
    }
    // world -> screen
    setupCamera() {
      const t = this.target, cp = Math.cos(this.pitch);
      this.eye = [t[0] + this.dist * cp * Math.cos(this.yaw), t[1] + this.dist * cp * Math.sin(this.yaw), t[2] + this.dist * Math.sin(this.pitch)];
      const f = U.unit(U.sub(t, this.eye)), r = U.unit(U.cross(f, [0, 0, 1])), u = U.cross(r, f);
      this.cam = { f, r, u };
      this.focal = 0.9 * Math.min(this.W, this.H) * 1.1;
    }
    proj(p) {
      const d = U.sub(p, this.eye), z = U.dot(d, this.cam.f);
      if (z < 0.05) return null;
      return [this.W / 2 + U.dot(d, this.cam.r) * this.focal / z, this.H / 2 - U.dot(d, this.cam.u) * this.focal / z, z];
    }
    line(a, b, color, w = 1) {
      const A = this.proj(a), B = this.proj(b); if (!A || !B) return;
      const g = this.g; g.strokeStyle = color; g.lineWidth = w * this.dpr; g.beginPath(); g.moveTo(A[0], A[1]); g.lineTo(B[0], B[1]); g.stroke();
    }
    poly(pts, fill, stroke) {
      const P = pts.map((p) => this.proj(p)); if (P.some((x) => !x)) return;
      const g = this.g; g.beginPath(); P.forEach((p, i) => (i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]))); g.closePath();
      if (fill) { g.fillStyle = fill; g.fill(); } if (stroke) { g.strokeStyle = stroke; g.lineWidth = this.dpr; g.stroke(); }
    }
    box(center, size, toWorld, fill, stroke) {
      const [lx, ly, lz] = size.map((s) => s / 2);
      const v = [];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) v.push(toWorld([center[0] + sx * lx, center[1] + sy * ly, center[2] + sz * lz]));
      const faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]];
      const fz = faces.map((f) => ({ f, z: f.reduce((a, i) => a + U.dot(U.sub(v[i], this.eye), this.cam.f), 0) / 4 }));
      fz.sort((a, b) => b.z - a.z);
      for (const { f } of fz) this.poly(f.map((i) => v[i]), fill, stroke);
    }

    draw(w, theme) {
      this.resize();
      const g = this.g, hw = w.hw, fr = hw.frame;
      const cgW = w.cgWorld();
      if (this.follow) this.target = [U.lerp(this.target[0], cgW[0], 0.15), U.lerp(this.target[1], cgW[1], 0.15), U.lerp(this.target[2], Math.max(0.2, cgW[2]), 0.15)];
      this.setupCamera();
      g.fillStyle = theme.bg; g.fillRect(0, 0, this.W, this.H);
      // sky gradient
      const grad = g.createLinearGradient(0, 0, 0, this.H); grad.addColorStop(0, theme.sky1); grad.addColorStop(1, theme.sky2);
      g.fillStyle = grad; g.fillRect(0, 0, this.W, this.H);
      // ground grid
      const cx = Math.round(this.target[0]), cy = Math.round(this.target[1]), N = 12;
      for (let i = -N; i <= N; i++) {
        const major = (cx + i) % 5 === 0;
        this.line([cx + i, cy - N, 0], [cx + i, cy + N, 0], major ? theme.gridMajor : theme.grid, major ? 1.2 : 0.7);
        const majorY = (cy + i) % 5 === 0;
        this.line([cx - N, cy + i, 0], [cx + N, cy + i, 0], majorY ? theme.gridMajor : theme.grid, majorY ? 1.2 : 0.7);
      }
      // pilot marker
      const pp = hw.link.pilotPos; this.line([pp[0], pp[1], 0], pp, theme.pilot, 2); const P = this.proj(pp); if (P) { g.fillStyle = theme.pilot; g.beginPath(); g.arc(P[0], P[1], 4 * this.dpr, 0, 7); g.fill(); g.font = `${11 * this.dpr}px system-ui`; g.fillText('pilot', P[0] + 6 * this.dpr, P[1]); }
      // trail
      if (w.trail.length > 1) { g.strokeStyle = theme.trail; g.lineWidth = 1.5 * this.dpr; g.beginPath(); let first = true; for (const p of w.trail) { const q = this.proj(p); if (!q) { first = true; continue; } if (first) { g.moveTo(q[0], q[1]); first = false; } else g.lineTo(q[0], q[1]); } g.stroke(); }
      // shadow
      const toW = (p) => U.add(w.pos, U.qRot(w.q, p));
      const sh = [[-0.08, -L(fr), 0], [0.08, -L(fr), 0], [0.08, L(fr), 0], [-0.08, L(fr), 0]].map((p) => { const q = toW(p); return [q[0], q[1], 0.001]; });
      this.poly(sh, theme.shadow);

      // ---- aircraft
      const comps = S.Hardware.resolveComponents(hw);
      const parts = [];
      for (const c of comps) {
        if (c.auto) continue;
        if (/prop/i.test(c.id)) continue;
        let tw = toW;
        if (c.tilts) {
          const d = c.tilts === 'L' ? w.tilt[0] : w.tilt[1], ay = c.pos[1];
          tw = (p) => { const rel = [p[0], 0, p[2]]; const x = rel[0] * Math.cos(d) + rel[2] * Math.sin(d), z = -rel[0] * Math.sin(d) + rel[2] * Math.cos(d); return toW([x, p[1], z]); };
          void ay;
        }
        const color = /batt/.test(c.id) ? theme.battery : /mot/.test(c.id) ? theme.motor : /esc|pdb|pico|imu|radio|ubec|ldo/.test(c.id) ? theme.elec : /servo/.test(c.id) ? theme.servo : theme.frame;
        const center = U.qRot(w.q, c.pos);
        parts.push({ z: U.dot(U.sub(U.add(w.pos, center), this.eye), this.cam.f), draw: () => this.box(c.pos, c.size, tw, color, theme.edge) });
      }
      // arms
      const armW = (fr.armSection.od || fr.armSection.width || 0.012);
      for (const s of [1, -1]) parts.push({ z: U.dot(U.sub(toW([0, s * L(fr) / 2, 0]), this.eye), this.cam.f), draw: () => this.box([0, s * (fr.hubHalfWidth + (L(fr) - fr.hubHalfWidth) / 2), 0], [armW, L(fr) - fr.hubHalfWidth, armW], toW, theme.arm, theme.edge) });
      parts.sort((a, b) => b.z - a.z);
      for (const p of parts) p.draw();
      // gear
      const gx = fr.gearSpanX / 2, gy = fr.gearSpanY / 2, gz = -fr.gearHeight;
      for (const sy of [-1, 1]) { this.line(toW([-gx, sy * gy, gz]), toW([gx, sy * gy, gz]), theme.frame, 2.5); this.line(toW([0, sy * gy * 0.5, -0.04]), toW([0, sy * gy, gz]), theme.frame, 2); }
      // props + thrust vectors
      for (let i = 0; i < 2; i++) {
        const d = w.tilt[i], side = i === 0 ? 1 : -1, R = hw.prop.diameter / 2;
        const hub = [fr.motorAxisOffsetX * Math.cos(d) + fr.propAboveTilt * Math.sin(d), side * L(fr), -fr.motorAxisOffsetX * Math.sin(d) + fr.propAboveTilt * Math.cos(d)];
        const ex = [Math.cos(d), 0, -Math.sin(d)], ey = [0, 1, 0], ax = [Math.sin(d), 0, Math.cos(d)];
        const pts = []; for (let k = 0; k < 24; k++) { const a = k / 24 * 2 * Math.PI; pts.push(toW(U.add(hub, U.add(U.scale(ex, R * Math.cos(a)), U.scale(ey, R * Math.sin(a)))))); }
        const rpmF = U.clamp(w.rotor[i] / 900, 0, 1);
        this.poly(pts, rpmF > 0.05 ? `rgba(${theme.propRgb},${0.12 + 0.25 * rpmF})` : `rgba(${theme.propRgb},0.06)`, `rgba(${theme.propRgb},0.7)`);
        // blades
        const ph = w.vibPhase[i] * (i ? 1 : -1);
        for (const b of [0, Math.PI]) this.line(toW(hub), toW(U.add(hub, U.add(U.scale(ex, R * Math.cos(ph + b)), U.scale(ey, R * Math.sin(ph + b))))), theme.blade, 2);
        if (this.showVectors && w.thrust[i] > 0.05) {
          const len = w.thrust[i] / 10 * 0.35;
          this.line(toW(hub), toW(U.add(hub, U.scale(ax, len))), theme.thrust, 3);
        }
      }
      if (this.showCG) {
        const c = this.proj(cgW); if (c) { g.fillStyle = theme.cg; g.beginPath(); g.arc(c[0], c[1], 5 * this.dpr, 0, 7); g.fill(); g.strokeStyle = theme.bg; g.lineWidth = this.dpr; g.stroke(); }
        // gravity vector
        this.line(cgW, U.add(cgW, [0, 0, -0.2]), theme.cg, 1.5);
      }
      // nose marker
      this.line(toW([0.05, 0, -0.035]), toW([0.16, 0, -0.035]), theme.nose, 3);
    }
  }
  function L(fr) { return fr.armLength; }

  S.View3D = View3D;
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
