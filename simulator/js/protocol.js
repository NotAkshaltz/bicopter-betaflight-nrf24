// Line-by-line JavaScript port of firmware/src-changed/src/main/rx/nrf24_bicopter_link.c
// (NRF24_BICOPTER protocol v2). Keep it in step with the C file: the self-test below checks
// the same known vector the C tests assert, so any drift shows up in the Test Suite.
(function (S) {
  'use strict';

  const P = {
    VERSION: 2, PKT_TYPE: 0xB1, PAYLOAD_LEN: 20, ADDR_LEN: 5, CHANNEL_COUNT: 8,
    CHANNEL_VALUE_MAX: 1000, US_MIN: 1000, US_MAX: 2000,
    RF_CHANNEL_COUNT: 4, RF_CHANNEL_FIRST: 4, RF_CHANNEL_SPACING: 19, RF_CHANNEL_OFFSET_SPAN: 20,
    FRAME_PERIOD_US: 5000, SEQ_ACCEPT_WINDOW: 32767, SYNC_MAX_MISSED_SLOTS: 8,
    FLAG_TX_INPUT_FAULT: 0x01, FLAG_TX_RESTART: 0x02, FLAGS_DEFINED_MASK: 0x03,
    ARM_CHANNEL: 4, THROTTLE_CHANNEL: 2, ARM_LOW_US: 1250, THROTTLE_LOW_US: 1050,
  };
  P.SEARCH_DWELL_US = (P.RF_CHANNEL_COUNT + 1) * P.FRAME_PERIOD_US;

  P.DECODE = ['OK', 'BAD_LENGTH', 'BAD_TYPE', 'BAD_VERSION', 'BAD_TAG', 'BAD_CRC', 'BAD_FLAGS', 'BAD_RANGE', 'BAD_SESSION'];
  P.FRESH = ['ACCEPT', 'DUPLICATE', 'STALE', 'RESYNC_PENDING', 'TX_FAULT'];

  const u32 = (x) => x >>> 0;
  const u16 = (x) => x & 0xFFFF;

  P.mix32 = (x) => {
    x = u32(x);
    x = u32(x ^ (x >>> 16));
    x = u32(Math.imul(x, 0x7feb352d));
    x = u32(x ^ (x >>> 15));
    x = u32(Math.imul(x, 0x846ca68b));
    x = u32(x ^ (x >>> 16));
    return x;
  };
  P.linkTag = (id) => u16(P.mix32(id) >>> 16);
  P.crcSeed = (id) => { let s = P.mix32(u32(id ^ 0xC0FFEE11)) & 0xFFFF; return s === 0 ? 0x1D0F : s; };
  P.crc16 = (crc, data, len) => {
    for (let i = 0; i < len; i++) {
      crc = u16(crc ^ (data[i] << 8));
      for (let b = 0; b < 8; b++) crc = (crc & 0x8000) ? u16((crc << 1) ^ 0x1021) : u16(crc << 1);
    }
    return crc;
  };
  const fixAddr = (b) => (b === 0x00 || b === 0xFF || b === 0x55 || b === 0xAA) ? (b ^ 0x3C) : b;
  P.deriveAddress = (id) => {
    const a = P.mix32(u32(id ^ 0xA5C3B1E7));
    const b = P.mix32(u32(a ^ id));
    return [fixAddr(a & 0xFF), fixAddr((a >>> 8) & 0xFF), fixAddr((a >>> 16) & 0xFF), fixAddr((a >>> 24) & 0xFF), fixAddr(b & 0xFF)];
  };
  P.deriveRfChannels = (id) => {
    const off = P.mix32(u32(id ^ 0x5EED5EED)) % P.RF_CHANNEL_OFFSET_SPAN;
    return [0, 1, 2, 3].map((i) => P.RF_CHANNEL_FIRST + off + i * P.RF_CHANNEL_SPACING);
  };

  // control = {session, sampleSeq, flags, channelUs[8]} -> Uint8Array(20) or null
  P.encode = (id, c) => {
    if (c.session === 0 || (c.flags & ~P.FLAGS_DEFINED_MASK & 0xFF)) return null;
    for (let ch = 0; ch < 8; ch++) if (c.channelUs[ch] < P.US_MIN || c.channelUs[ch] > P.US_MAX) return null;
    const out = new Uint8Array(P.PAYLOAD_LEN);
    const tag = P.linkTag(id);
    out[0] = P.PKT_TYPE; out[1] = P.VERSION;
    out[2] = tag & 0xFF; out[3] = tag >>> 8;
    out[4] = c.session & 0xFF;
    out[5] = c.sampleSeq & 0xFF; out[6] = (c.sampleSeq >>> 8) & 0xFF;
    let bit = 0;
    for (let ch = 0; ch < 8; ch++) {
      const v = (c.channelUs[ch] - P.US_MIN) | 0;
      for (let k = 0; k < 10; k++, bit++) if (v & (1 << k)) out[7 + (bit >> 3)] |= (1 << (bit & 7));
    }
    out[17] = c.flags & 0xFF;
    const crc = P.crc16(P.crcSeed(id), out, 18);
    out[18] = crc & 0xFF; out[19] = crc >>> 8;
    return out;
  };

  // -> {result: index into P.DECODE, control}
  P.decode = (id, buf, len) => {
    if (len !== P.PAYLOAD_LEN) return { result: 1 };
    if (buf[0] !== P.PKT_TYPE) return { result: 2 };
    if (buf[1] !== P.VERSION) return { result: 3 };
    if ((buf[2] | (buf[3] << 8)) !== P.linkTag(id)) return { result: 4 };
    if ((buf[18] | (buf[19] << 8)) !== P.crc16(P.crcSeed(id), buf, 18)) return { result: 5 };
    const flags = buf[17];
    if (flags & ~P.FLAGS_DEFINED_MASK & 0xFF) return { result: 6 };
    const session = buf[4];
    if (session === 0) return { result: 8 };
    const ch = [];
    let bit = 0;
    for (let c = 0; c < 8; c++) {
      let v = 0;
      for (let k = 0; k < 10; k++, bit++) if (buf[7 + (bit >> 3)] & (1 << (bit & 7))) v |= (1 << k);
      if (v > P.CHANNEL_VALUE_MAX) return { result: 7 };
      ch.push(P.US_MIN + v);
    }
    return { result: 0, control: { session, sampleSeq: buf[5] | (buf[6] << 8), flags, channelUs: ch } };
  };

  P.freshnessInit = () => ({ haveRef: false, session: 0, lastSeq: 0, haveCandidate: false, candSession: 0, candSeq: 0 });
  P.freshnessCheck = (f, session, seq, flags) => {
    if (flags & (P.FLAG_TX_INPUT_FAULT | P.FLAG_TX_RESTART)) return 4;
    if (f.haveRef && session === f.session) {
      const step = u16(seq - f.lastSeq);
      if (step === 0) return 1;
      if (step <= P.SEQ_ACCEPT_WINDOW) { f.lastSeq = seq; f.haveCandidate = false; return 0; }
      return 2;
    }
    const cstep = u16(seq - f.candSeq);
    if (f.haveCandidate && f.candSession === session && cstep > 0 && cstep <= P.SEQ_ACCEPT_WINDOW) {
      f.haveRef = true; f.session = session; f.lastSeq = seq; f.haveCandidate = false;
      return 0;
    }
    f.haveCandidate = true; f.candSession = session; f.candSeq = seq;
    return 3;
  };

  // Hop state machine. Times are integer microseconds (wrap handled with |0 like int32_t).
  P.hopInit = (now) => ({ state: 0, idx: 0, lastHopUs: now, nextHopUs: now + P.SEARCH_DWELL_US, missedSlots: 0, heardThisSlot: false });
  P.hopOnPacket = (h, t, reliable) => {
    h.heardThisSlot = true; h.missedSlots = 0;
    if (h.state === 1 && !reliable) return;
    let next = t + P.FRAME_PERIOD_US / 2;
    if ((t - h.lastHopUs) < 0) next += P.FRAME_PERIOD_US;
    h.nextHopUs = next; h.state = 1;
  };
  P.hopTick = (h, now) => {
    const late = now - h.nextHopUs;
    if (late < 0) return false;
    if (h.state === 0) {
      h.idx = (h.idx + 1) % P.RF_CHANNEL_COUNT;
      h.nextHopUs = now + P.SEARCH_DWELL_US; h.lastHopUs = now; h.heardThisSlot = false;
      return true;
    }
    const slots = 1 + Math.floor(late / P.FRAME_PERIOD_US);
    const missedNow = slots - (h.heardThisSlot ? 1 : 0);
    h.missedSlots = Math.min(0xFFFF, h.missedSlots + missedNow);
    h.heardThisSlot = false;
    h.idx = (h.idx + slots) % P.RF_CHANNEL_COUNT;
    h.nextHopUs += slots * P.FRAME_PERIOD_US; h.lastHopUs = now;
    if (h.missedSlots > P.SYNC_MAX_MISSED_SLOTS) { h.state = 0; h.missedSlots = 0; h.nextHopUs = now + P.SEARCH_DWELL_US; }
    return true;
  };

  P.toHex = (buf) => Array.from(buf).map((b) => b.toString(16).padStart(2, '0')).join('').toUpperCase();

  // Same vector as the C tests: session 0x5A, seq 0x1234, channels 1000 + (i*137 mod 1001).
  P.selfTest = (linkId = 0xBE5336DF) => {
    const results = [];
    const expect = (name, ok, detail) => results.push({ name, ok: !!ok, detail });
    const ch = []; for (let i = 0; i < 8; i++) ch.push(1000 + ((i * 137) % 1001));
    const pkt = P.encode(linkId, { session: 0x5A, sampleSeq: 0x1234, flags: 0, channelUs: ch });
    const hex = pkt ? P.toHex(pkt) : 'null';
    expect('Known packet vector', hex === 'B10234095A3412002422D16624B66AF3EF002A89', hex);
    expect('Link tag 0x0934', P.linkTag(linkId) === 0x0934, '0x' + P.linkTag(linkId).toString(16));
    expect('CRC seed 0xFEFA', P.crcSeed(linkId) === 0xFEFA, '0x' + P.crcSeed(linkId).toString(16));
    const addr = P.deriveAddress(linkId).map((b) => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
    expect('Address 3C 41 58 CB F7', addr === '3C 41 58 CB F7', addr);
    const rf = P.deriveRfChannels(linkId).join(' ');
    expect('RF channels 20 39 58 77', rf === '20 39 58 77', rf);
    const dec = P.decode(linkId, pkt, 20);
    expect('Decode round-trip', dec.result === 0 && dec.control.channelUs.every((v, i) => v === ch[i]), P.DECODE[dec.result]);
    const bad = pkt.slice(); bad[9] ^= 0x01;
    expect('Corrupted byte rejected (CRC)', P.decode(linkId, bad, 20).result === 5, P.DECODE[P.decode(linkId, bad, 20).result]);
    expect('Wrong link ID rejected', P.decode(0x12345678, pkt, 20).result !== 0, P.DECODE[P.decode(0x12345678, pkt, 20).result]);
    const f = P.freshnessInit();
    const r1 = P.freshnessCheck(f, 7, 100, 0), r2 = P.freshnessCheck(f, 7, 101, 0), r3 = P.freshnessCheck(f, 7, 101, 0), r4 = P.freshnessCheck(f, 7, 99, 0);
    expect('Freshness: confirm, duplicate, stale', r1 === 3 && r2 === 0 && r3 === 1 && r4 === 2, [r1, r2, r3, r4].map((r) => P.FRESH[r]).join(','));
    const w1 = P.freshnessCheck(f, 7, 65535, 0); f.lastSeq = 65535; const w2 = P.freshnessCheck(f, 7, 0, 0);
    expect('Sequence wrap 65535 -> 0 accepted', w2 === 0, P.FRESH[w1] + ',' + P.FRESH[w2]);
    expect('Restart/fault packets never accepted', P.freshnessCheck(f, 7, 1, P.FLAG_TX_RESTART) === 4 && P.freshnessCheck(f, 7, 2, P.FLAG_TX_INPUT_FAULT) === 4, '');
    return results;
  };

  S.Protocol = P;
})(typeof globalThis !== 'undefined' ? (globalThis.Sim = globalThis.Sim || {}) : (window.Sim = window.Sim || {}));
