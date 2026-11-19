// Radio link model: STM32F103 handset (handset_app.c behaviour), a 2.4 GHz channel and the
// aircraft receiver adapter (rx/nrf24_bicopter.c serviceRadio() logic), all on top of the
// ported protocol core in protocol.js. Times inside are integer microseconds.
(function (S) {
  'use strict';
  const U = S.U, PR = S.Protocol;

  const RF_PWR_DBM = { 0x00: -18, 0x02: -12, 0x04: -6, 0x06: 0 };

  class Handset {
    constructor(cfg, rng) {
      this.cfg = cfg; this.rng = rng;
      this.inputs = { roll: 1500, pitch: 1500, throttle: 1000, yaw: 1500, aux: [1000, 1900, 1000, 1000] };
      this.on = true;
      this.faults = { inputFaultUntil: 0, frozenUntil: 0 };
      this.boot(0);
    }
    boot(nowUs) {
      this.session = 1 + Math.floor(this.rng() * 255);
      this.seq = Math.floor(this.rng() * 65536);
      this.t0 = nowUs; this.k = 0;
      this.nextFrame = nowUs + 20000; // radio setup time after power-on
      this.announceUntil = nowUs + 20000 + 1000000;
      this.announcing = true;
      this.lastSampleUs = nowUs;
      this.stats = { sent: 0, restartPkts: 0, faultPkts: 0 };
    }
    channels() {
      const i = this.inputs;
      return [i.roll, i.pitch, i.throttle, i.yaw, i.aux[0], i.aux[1], i.aux[2], i.aux[3]].map((v) => U.clamp(Math.round(v), 1000, 2000));
    }
    // Returns a packet if one is due at nowUs.
    tick(nowUs) {
      if (!this.on || nowUs < this.nextFrame) return null;
      const slot = this.k;
      this.k++; this.nextFrame += PR.FRAME_PERIOD_US;
      const frozen = nowUs < this.faults.frozenUntil;
      if (!frozen) { this.seq = (this.seq + 1) & 0xFFFF; this.lastSampleUs = nowUs; }
      let flags = 0;
      if (nowUs < this.faults.inputFaultUntil || nowUs - this.lastSampleUs > 50000) flags |= PR.FLAG_TX_INPUT_FAULT;
      let ch = this.channels();
      if (this.announcing) {
        if (nowUs >= this.announceUntil && !frozen && ch[4] <= PR.ARM_LOW_US && ch[2] <= PR.THROTTLE_LOW_US && !(flags & 1)) this.announcing = false;
        else { flags |= PR.FLAG_TX_RESTART; ch[4] = 1000; ch[2] = 1000; }
      }
      const bytes = PR.encode(this.cfg.linkId >>> 0, { session: this.session, sampleSeq: this.seq, flags, channelUs: ch });
      this.stats.sent++;
      if (flags & 2) this.stats.restartPkts++; if (flags & 1) this.stats.faultPkts++;
      return { bytes, chIdx: slot % PR.RF_CHANNEL_COUNT, tUs: nowUs, flags };
    }
  }

  // Packet error probability for one packet.
  class RadioChannel {
    constructor(hwLink, rng) { this.cfg = hwLink; this.rng = rng; this.dropouts = []; this.extraLoss = 0; this.last = { prx: 0, per: 0 }; }
    rxPower(distM, rfPwrBits) {
      const c = this.cfg;
      const d = Math.max(0.1, distM);
      // free space to 1 m, then log-distance with the configured exponent (ground reflections, clutter)
      const fspl = 40.2 + (d <= 1 ? 20 * Math.log10(d) : 10 * c.pathLossExp * Math.log10(d));
      const fade = c.fadeSigmaDb * this.rng.gauss();
      return (RF_PWR_DBM[rfPwrBits] ?? 0) + c.paGainDb + c.txAntDbi + c.rxAntDbi - fspl - fade;
    }
    deliver(pkt, nowUs, distM, rfPwrBits, rfChannels) {
      for (const d of this.dropouts) if (nowUs >= d.from && nowUs < d.to) { this.last = { prx: -200, per: 1 }; return false; }
      const c = this.cfg, prx = this.rxPower(distM, rfPwrBits);
      let sens = c.rxSensDbm;
      if (c.wifiChannel > 0) {
        const fWifi = 2412 + 5 * (c.wifiChannel - 1), fNrf = 2400 + rfChannels[pkt.chIdx];
        if (Math.abs(fWifi - fNrf) <= 11) sens += c.wifiLevelDb;
      }
      let per = 1 / (1 + Math.exp((prx - sens - 3) / 1.5));
      if (prx > c.lnaSatDbm) per = Math.max(per, U.clamp((prx - c.lnaSatDbm) / 10, 0.3, 0.95)); // PA into LNA too close
      per = 1 - (1 - per) * (1 - this.extraLoss);
      this.last = { prx, per };
      return this.rng() >= per;
    }
  }

  class Receiver {
    constructor(linkId) { this.linkId = linkId >>> 0; this.init(0); }
    init(nowUs) {
      this.hop = PR.hopInit(nowUs);
      this.fresh = PR.freshnessInit();
      this.haveSession = false; this.observedSession = 0; this.restartAnnounced = false;
      this.armInterlockReleased = false;
      this.stats = { accepted: 0, malformed: 0, duplicate: 0, stale: 0, txFault: 0, resync: 0, offChannel: 0, resets: 0 };
      this.window = []; // accepted timestamps for link quality
      this.lastDecode = '';
    }
    tick(nowUs) { PR.hopTick(this.hop, nowUs); }
    // fc: FlightController (onFrame / onReceiverReset)
    onAir(pkt, nowUs, fc) {
      if (this.hop.idx !== pkt.chIdx) { this.stats.offChannel++; return; }
      const dec = PR.decode(this.linkId, pkt.bytes, pkt.bytes ? pkt.bytes.length : 0);
      this.lastDecode = PR.DECODE[dec.result];
      if (dec.result !== 0) { this.stats.malformed++; return; }
      const c = dec.control;
      PR.hopOnPacket(this.hop, nowUs, true);
      const newSession = this.haveSession && this.observedSession !== c.session;
      const restart = (c.flags & PR.FLAG_TX_RESTART) !== 0;
      if (newSession || (restart && !this.restartAnnounced)) {
        if (this.stats.accepted) { fc.onReceiverReset(nowUs / 1e6); this.stats.resets++; }
        this.armInterlockReleased = false;
        this.fresh = PR.freshnessInit();
      }
      this.haveSession = true; this.observedSession = c.session; this.restartAnnounced = restart;
      if (restart) { this.stats.txFault++; return; }
      const fr = PR.freshnessCheck(this.fresh, c.session, c.sampleSeq, c.flags);
      if (fr === 1) { this.stats.duplicate++; return; }
      if (fr === 2) { this.stats.stale++; return; }
      if (fr === 4) { this.stats.txFault++; return; }
      if (fr === 3) { this.stats.resync++; return; }
      const ch = c.channelUs.slice();
      if (!this.armInterlockReleased) {
        if (ch[PR.ARM_CHANNEL] <= PR.ARM_LOW_US && ch[PR.THROTTLE_CHANNEL] <= PR.THROTTLE_LOW_US) this.armInterlockReleased = true;
        else { ch[PR.ARM_CHANNEL] = 1000; ch[PR.THROTTLE_CHANNEL] = 1000; }
      }
      this.stats.accepted++;
      this.window.push(nowUs);
      fc.onFrame(nowUs / 1e6, ch);
    }
    linkQuality(nowUs) {
      while (this.window.length && nowUs - this.window[0] > 1e6) this.window.shift();
      return Math.min(100, this.window.length / 2); // 200 frames/s = 100 %
    }
  }

  S.Link = { Handset, RadioChannel, Receiver, RF_PWR_DBM };
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
