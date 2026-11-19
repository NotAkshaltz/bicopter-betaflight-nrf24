// Small canvas time-series chart with a legend and a fixed time window.
(function (S) {
  'use strict';

  class Chart {
    // series: [{key: fn(rec) -> number, label, color}]
    constructor(canvas, opts) { this.c = canvas; this.g = canvas.getContext('2d'); this.o = Object.assign({ window: 10, min: null, max: null, unit: '' }, opts); }
    draw(records, tEnd, theme, tStart) {
      const c = this.c, g = this.g, d = window.devicePixelRatio || 1, r = c.getBoundingClientRect();
      if (c.width !== Math.round(r.width * d) || c.height !== Math.round(r.height * d)) { c.width = Math.round(r.width * d); c.height = Math.round(r.height * d); }
      const W = c.width, H = c.height, pad = { l: 40 * d, r: 8 * d, t: 18 * d, b: 16 * d };
      g.clearRect(0, 0, W, H);
      const t1 = tEnd, t0 = tStart !== undefined ? tStart : t1 - this.o.window;
      // records are time-ordered: binary search the window start
      let lo = 0, hi = records.length;
      while (lo < hi) { const m = (lo + hi) >> 1; if (records[m].t < t0) lo = m + 1; else hi = m; }
      const pts = records.slice(lo).filter((x) => x.t <= t1);
      let mn = Infinity, mx = -Infinity;
      for (const s of this.o.series) for (const p of pts) { const v = s.key(p); if (Number.isFinite(v)) { if (v < mn) mn = v; if (v > mx) mx = v; } }
      if (this.o.min !== null) mn = Math.min(mn, this.o.min);
      if (this.o.max !== null) mx = Math.max(mx, this.o.max);
      if (!Number.isFinite(mn)) { mn = 0; mx = 1; }
      if (mx - mn < 1e-6) { mx += 1; mn -= 1; }
      const span = mx - mn; mn -= span * 0.05; mx += span * 0.05;
      const X = (t) => pad.l + (t - t0) / (t1 - t0) * (W - pad.l - pad.r), Y = (v) => H - pad.b - (v - mn) / (mx - mn) * (H - pad.t - pad.b);
      // grid
      g.strokeStyle = theme.chartGrid; g.lineWidth = d; g.font = `${10 * d}px system-ui`; g.fillStyle = theme.muted;
      for (let k = 0; k <= 4; k++) {
        const v = mn + (mx - mn) * k / 4, y = Y(v);
        g.beginPath(); g.moveTo(pad.l, y); g.lineTo(W - pad.r, y); g.stroke();
        g.fillText(fmtTick(v), 2 * d, y + 3 * d);
      }
      if (mn < 0 && mx > 0) { g.strokeStyle = theme.chartZero; g.beginPath(); g.moveTo(pad.l, Y(0)); g.lineTo(W - pad.r, Y(0)); g.stroke(); }
      g.fillText(`${(t0).toFixed(1)} s`, pad.l, H - 3 * d);
      g.fillText(`${(t1).toFixed(1)} s`, W - pad.r - 34 * d, H - 3 * d);
      // series
      for (const s of this.o.series) {
        g.strokeStyle = s.color; g.lineWidth = (s.width || 1.4) * d; g.setLineDash(s.dash ? s.dash.map((x) => x * d) : []);
        g.beginPath(); let started = false;
        for (const p of pts) { const v = s.key(p); if (!Number.isFinite(v)) { started = false; continue; } const x = X(p.t), y = Y(v); if (!started) { g.moveTo(x, y); started = true; } else g.lineTo(x, y); }
        g.stroke();
      }
      g.setLineDash([]);
      // title + legend
      let x = pad.l; g.font = `600 ${10.5 * d}px system-ui`; g.fillStyle = theme.text; g.fillText(this.o.title + (this.o.unit ? ` (${this.o.unit})` : ''), x, 12 * d);
      x += g.measureText(this.o.title + (this.o.unit ? ` (${this.o.unit})` : '')).width + 10 * d;
      g.font = `${10 * d}px system-ui`;
      for (const s of this.o.series) {
        const last = pts.length ? s.key(pts[pts.length - 1]) : NaN;
        const label = `${s.label} ${Number.isFinite(last) ? fmtTick(last) : ''}`;
        g.fillStyle = s.color; g.fillRect(x, 6 * d, 10 * d, 3 * d); x += 13 * d;
        g.fillStyle = theme.muted; g.fillText(label, x, 12 * d); x += g.measureText(label).width + 10 * d;
      }
    }
  }
  function fmtTick(v) { const a = Math.abs(v); return a >= 1000 ? v.toFixed(0) : a >= 100 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : v.toFixed(2); }

  S.Chart = Chart;
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
