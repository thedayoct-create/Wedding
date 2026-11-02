// Excavator figure: geometry and model shared by the Node build and the live page.
// World units: 1 m = 20 units. x forward (toward the trench), y to the machine's left, z up.
// Everything here is plain functions of (k, G, P) so the page can inline this file and
// rebuild the moving parts per frame with the same code the build used for the first frame.

export const M = 20;

// Ground block, trench and cutaway notch.
export const GX0 = -84, GX1 = 262, GY0 = -70, GY1 = 50, GD = 64;
export const TX0 = 114, TX1 = 236, TY = 13;
export const DX = 4;
export const XS = Array.from({ length: Math.round((TX1 - TX0) / DX) + 1 }, (_, i) => TX0 + i * DX);
export const S0 = -26;
export const FLOOR_MIN = -GD + 10;

// Machine (20 t class): tracks 4.4 m long on a 2.4 m gauge, 0.6 m shoes.
export const TRACK = { half: 44, r: 9, y: 24, w: 6 };
export const FOOT = [26, 38];
export const BOOM = { len: 114, knee: [62, 16], h: [7, 10.5, 6.5], w: 6 };
export const STICK = { len: 58, w: 7 };
export const BUCKET_W = 12;
export const PILE = { x: 98, max: 15 };

// Joint limits (degrees) and the bucket's curl range.
export const LIM = { boom: [-46, 54], stick: [-152, -24], curl: [24, -156] };

// Bucket profile in its own frame: a from the bucket pin toward the teeth, b across.
// The mouth (between T and L) faces -b; the back of the shell bulges to +b.
export const BK = {
  plate: [[4, -9], [-2, -6.5], [-5, 0.5], [-2.4, 7.4], [6, 13.6], [16, 16.4], [24.6, 13.4], [30.4, 7], [33, 0.6], [33, -2.2]],
  shell: [[-5, 0.5], [-2.4, 7.4], [6, 13.6], [16, 16.4], [24.6, 13.4], [30.4, 7], [33, 0.6]],
  T: [4, -9],
  L: [33, -2.2],
  Q: [-1, 9],
  tip: [38.6, -1.6],
};
// Stick outline in its frame (origin at the boom-tip pin, a toward the bucket pin).
export const SK = {
  outline: [[-12, -6], [-2, -8], [58, -4.2], [58, 4.2], [-4, 13], [-13, 14], [-18, 5]],
  heel: [-13, 10],
  base: [8, 16.6],
  link: [47, 6.2],
};
export const LINK = { side: 12, h: 13.4 };
export const CAPACITY = 1.0; // m³, heaped
export const TRENCH_W = (2 * TY) / M; // m

const rad = (d) => (d * Math.PI) / 180;
const dir = (a) => [Math.cos(a), Math.sin(a)];
const add = (p, q) => [p[0] + q[0], p[1] + q[1]];
const sub = (p, q) => [p[0] - q[0], p[1] - q[1]];
const mul = (p, s) => [p[0] * s, p[1] * s];
const len = (p) => Math.hypot(p[0], p[1]);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const local = (o, ang) => ([a, b]) => [o[0] + a * Math.cos(ang) - b * Math.sin(ang), o[1] + a * Math.sin(ang) + b * Math.cos(ang)];

export function soilAt(s, x) {
  if (x <= TX0 || x >= TX1) return 0;
  const f = (x - TX0) / DX;
  const i = Math.min(XS.length - 2, Math.floor(f));
  const t = f - i;
  return s[i] * (1 - t) + s[i + 1] * t;
}

export function pileGeom(volume) {
  // Cone at a 34° angle of repose: V = π r² h / 3, h = r tan 34°.
  const v = Math.max(0, volume);
  const r = Math.min(PILE.max, Math.cbrt((3 * v) / (Math.PI * Math.tan(rad(34)))) * M);
  const h = r * Math.tan(rad(34));
  return { r, h };
}

export function pileHeight(volume, x) {
  const { r, h } = pileGeom(volume);
  const d = Math.abs(x - PILE.x);
  return d >= r ? 0 : h * (1 - d / r);
}

// Forward kinematics: every pin of the front attachment from three joint angles (degrees).
export function pose(boom, stick, curl) {
  const tb = rad(boom), ts = rad(boom + stick), tk = rad(boom + stick + curl);
  const B = local(FOOT, tb);
  const E = B([BOOM.len, 0]);
  const Sk = local(E, ts);
  const pin = Sk([STICK.len, 0]);
  const Bk = local(pin, tk);
  const K = Sk(SK.link);
  const Q = Bk(BK.Q);
  // J closes the four-bar: |J − K| = side link, |J − Q| = H-link, on the far side from the stick.
  const d = len(sub(Q, K));
  const a = (LINK.side ** 2 - LINK.h ** 2 + d ** 2) / (2 * d);
  const hh = Math.sqrt(Math.max(0, LINK.side ** 2 - a ** 2));
  const u = mul(sub(Q, K), 1 / d);
  const mid = add(K, mul(u, a));
  const n = [-u[1], u[0]];
  const J1 = add(mid, mul(n, hh)), J2 = add(mid, mul(n, -hh));
  const up = [-Math.sin(ts), Math.cos(ts)];
  const J = (J1[0] - K[0]) * up[0] + (J1[1] - K[1]) * up[1] > (J2[0] - K[0]) * up[0] + (J2[1] - K[1]) * up[1] ? J1 : J2;
  return {
    tb, ts, tk, B, E, Sk, pin, Bk, K, Q, J,
    tip: Bk(BK.tip),
    lip: Bk(BK.L),
    heel: Sk(SK.heel),
    stickBase: Sk(SK.base),
    boomLug: B([50, -12]),
    stickLug: B([60, 21.5]),
    houseLug: [36, 25],
    mouth: [Math.sin(tk), -Math.cos(tk)],
  };
}

// Inverse kinematics for the teeth: aim the tip at T with a given curl, elbow (boom) up.
export function solve(T, curl, stickNow) {
  const R = len(BK.tip);
  const off = Math.atan2(BK.tip[1], BK.tip[0]);
  let stickAbs = stickNow;
  let out = null;
  for (let pass = 0; pass < 10; pass++) {
    const phi = stickAbs + rad(curl) + off;
    const Bp = sub(T, mul(dir(phi), R));
    const v = sub(Bp, FOOT);
    const d = clamp(len(v), Math.abs(BOOM.len - STICK.len) + 1, BOOM.len + STICK.len - 0.5);
    const base = Math.atan2(v[1], v[0]);
    const inner = Math.acos(clamp((BOOM.len ** 2 + d ** 2 - STICK.len ** 2) / (2 * BOOM.len * d), -1, 1));
    const boom = base + inner;
    const E = add(FOOT, mul(dir(boom), BOOM.len));
    const stick = Math.atan2(Bp[1] - E[1], Bp[0] - E[0]);
    const bDeg = clamp((boom * 180) / Math.PI, LIM.boom[0], LIM.boom[1]);
    let sDeg = ((stick - boom) * 180) / Math.PI;
    while (sDeg > 180) sDeg -= 360;
    while (sDeg < -180) sDeg += 360;
    sDeg = clamp(sDeg, LIM.stick[0], LIM.stick[1]);
    stickAbs = rad(bDeg + sDeg);
    out = { boom: bDeg, stick: sDeg };
  }
  return out;
}

export function makeScene(k, G, P) {
  const V = G.viewOf(P);
  const L = G.lightOf(P);
  const at = (p) => k.iso(p, P);
  const S = (paths, style) => k.solidSvg(paths, style);
  const Ln = (d, style) => k.lineSvg(d, style);
  const D = (points, style) => k.dotsSvg(points, style);
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  // A prism of a polygon drawn in the x–z plane, extruded along y from y0 to y1.
  const FY = (y) => G.frameOf([0, y, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]);
  const xz = (poly, y0, y1) => G.prismOf(FY(0), poly.map(([x, z]) => [z, x]), y0, y1, P);
  const pinY = (p, r, y0, y1) => G.disc(y0, y1, r, G.frameAlong([p[0], 0, p[1]], [0, 1, 0]), P);
  const P3 = (p, y) => [p[0], y, p[1]];
  const pathOf = (pts, closed) => k.pathOf(pts.map(at), closed);

  // One flat face as solid paths, toned by its facing.
  function face(points, normal, extra = {}) {
    const d = pathOf(points, true);
    const band = G.bandOf(dot(normal, L));
    const shades = ["", "", "", ""];
    let top = "";
    if (band === 4) top = d;
    else shades[band] = d;
    return { fill: d, outline: extra.outline ?? d, crease: "", top, shades };
  }

  // ---------------------------------------------------------------- strata
  // Layer lines run round the cut faces as one path coordinate u, so they meet at every corner.
  const RUN = [
    { from: [GX0, GY1], to: [TX0, GY1] },
    { from: [TX0, GY1], to: [TX0, TY] },
    { from: [TX0, TY], to: [GX1, TY] },
    { from: [GX1, TY], to: [GX1, GY0] },
  ];
  const LAYERS = [
    { base: -9, amp: 1.4, f: 0.07 },
    { base: -31, amp: 2.6, f: 0.045 },
    { base: -50, amp: 2.2, f: 0.06 },
  ];
  const layerZ = (layer, u) => layer.base + layer.amp * Math.sin(u * layer.f) + layer.amp * 0.45 * Math.sin(u * layer.f * 2.7 + 1.3);
  const runStart = [];
  let acc = 0;
  for (const r of RUN) { runStart.push(acc); acc += Math.hypot(r.to[0] - r.from[0], r.to[1] - r.from[1]); }
  function strata(which, top = () => 0, step = 2) {
    // which: indices into RUN; top(x, y) is the ground surface over that face.
    const out = [];
    for (const index of which) {
      const r = RUN[index];
      const length = Math.hypot(r.to[0] - r.from[0], r.to[1] - r.from[1]);
      const n = Math.max(2, Math.ceil(length / step));
      const pointAt = (layer, t) => {
        const x = r.from[0] + (r.to[0] - r.from[0]) * t;
        const y = r.from[1] + (r.to[1] - r.from[1]) * t;
        return [x, y, layerZ(layer, runStart[index] + length * t)];
      };
      // A layer line stops exactly where it meets the surface above it (found by bisection).
      const meet = (layer, t0, t1) => {
        for (let k2 = 0; k2 < 18; k2++) {
          const tm = (t0 + t1) / 2;
          const p = pointAt(layer, tm);
          const below = p[2] < top(p[0], p[1]);
          const p0 = pointAt(layer, t0);
          if ((p0[2] < top(p0[0], p0[1])) === below) t0 = tm; else t1 = tm;
        }
        const p = pointAt(layer, (t0 + t1) / 2);
        return [p[0], p[1], top(p[0], p[1])];
      };
      for (const layer of LAYERS) {
        let run = [];
        let prev = null;
        for (let i = 0; i <= n; i++) {
          const t = i / n;
          const p = pointAt(layer, t);
          const inside = p[2] < top(p[0], p[1]);
          if (prev !== null && inside !== prev.inside) {
            const m = meet(layer, prev.t, t);
            if (inside) run.push(m);
            else { run.push(m); if (run.length > 1) out.push(run); run = []; }
          }
          if (inside) run.push(p);
          prev = { t, inside };
        }
        if (run.length > 1) out.push(run);
      }
    }
    return out;
  }
  // Stones in the gravel layer: fixed positions along u, kept below the surface.
  const STONES = [];
  {
    let seed = 7;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
    for (let i = 0; i < 260; i++) STONES.push({ u: rnd() * acc, z: -34 - rnd() * 30, r: 0.35 + rnd() * 0.35 });
  }
  function stonesOn(which, top = () => 0) {
    const pts = [];
    for (const s of STONES) {
      const index = runStart.findIndex((start, i) => s.u >= start && (i === runStart.length - 1 || s.u < runStart[i + 1]));
      if (!which.includes(index)) continue;
      const r = RUN[index];
      const length = Math.hypot(r.to[0] - r.from[0], r.to[1] - r.from[1]);
      const t = (s.u - runStart[index]) / length;
      if (t < 0.02 || t > 0.98) continue;
      const x = r.from[0] + (r.to[0] - r.from[0]) * t;
      const y = r.from[1] + (r.to[1] - r.from[1]) * t;
      const lower = layerZ(LAYERS[1], s.u) - 2.5;
      const bottom = -GD + 2;
      if (s.z > lower || s.z < bottom || s.z > top(x, y) - 3) continue;
      pts.push(at([x, y, s.z]));
    }
    return pts;
  }

  // ---------------------------------------------------------------- static ground (back)
  function groundBack() {
    const ring = [[GX0, GY0], [TX1, GY0], [TX1, -TY], [TX0, -TY], [TX0, GY1], [GX0, GY1]];
    const out = [`<path class="iso-halo" d="${k.haloOf({ x: GX0, y: GY0, w: GX1 - GX0, d: GY1 - GY0, r: 2 }, -GD, GD, P)}"/>`];
    out.push(S(k.extrude(ring, -GD, GD, P, { convex: false }), { tone: "lo", crease: "faint" }));
    out.push(...strata([0, 1]).map((run) => Ln(k.pathOf(run.map(at)), { tone: "faint" })));
    out.push(D(stonesOn([0, 1]), { size: 0.5, tone: "lo" }));
    // Track prints behind the machine: two rails of grouser marks that run out from under the tracks.
    for (const side of [-1, 1]) {
      const y0 = side * TRACK.y - TRACK.w, y1 = side * TRACK.y + TRACK.w;
      out.push(Ln(k.lineOnTop([GX0, y0], [0, y0], 0, P), { tone: "faint" }));
      out.push(Ln(k.lineOnTop([GX0, y1], [0, y1], 0, P), { tone: "faint" }));
      const marks = [];
      for (let x = GX0 + 5; x < -TRACK.half; x += 6) marks.push(k.lineOnTop([x, y0], [x, y1], 0, P));
      out.push(Ln(marks.join(""), { tone: "faint" }));
    }
    // Grade stakes and a string line along the far edge of the trench.
    const stakes = [TX0 + 6, (TX0 + TX1) / 2, TX1 - 6];
    for (const x of stakes) {
      out.push(S(k.slabOf({ x: x - 1.2, y: -24.2, w: 2.4, d: 2.4, r: 0.3 }, 0, 13, P, 2), { tone: "mid", crease: "none" }));
      out.push(S(k.slabOf({ x: x - 2.2, y: -23.5, w: 4.4, d: 1, r: 0.2 }, 9, 1.4, P, 2), { tone: "mid", crease: "none" }));
    }
    out.push(k.wireSvg(at([stakes[0], -23, 10.4]), at([stakes[2], -23, 10.4]), { via: [at([stakes[1], -23, 10.4])], tone: "lo", size: 0.9 }));
    return out.join("");
  }

  // ---------------------------------------------------------------- trench (behind the machine's front)
  function trenchBack(s) {
    const out = [];
    const prof = (y) => XS.map((x, i) => [x, y, s[i]]);
    // Far wall (y = −TY), from the floor up to the ground.
    const far = [[TX0, -TY, 0], [TX1, -TY, 0], ...prof(-TY).reverse()];
    out.push(S(face(far, [0, 1, 0]), { tone: "lo", crease: "none" }));
    // Layer lines on the far wall, above the floor: they end where they meet the floor.
    for (const layer of LAYERS) {
      const zAt = (x) => layerZ(layer, runStart[2] + (x - TX0) + 3);
      const ok = (x) => zAt(x) < 0 && zAt(x) > soilAt(s, x);
      let run = [];
      let prevX = null;
      const flush = () => { if (run.length > 1) out.push(Ln(k.pathOf(run.map(at)), { tone: "faint" })); run = []; };
      for (let x = TX0; x <= TX1; x += 2) {
        const inside = ok(x);
        if (prevX !== null && inside !== ok(prevX)) {
          let a = prevX, b = x;
          for (let i = 0; i < 16; i++) { const m = (a + b) / 2; if (ok(m) === ok(a)) a = m; else b = m; }
          const m = (a + b) / 2;
          run.push([m, -TY, Math.max(soilAt(s, m), Math.min(0, zAt(m)))]);
          if (!inside) flush();
        }
        if (inside) run.push([x, -TY, zAt(x)]);
        prevX = x;
      }
      flush();
    }
    // Wall at the machine end (x = TX0).
    out.push(S(face([[TX0, -TY, 0], [TX0, TY, 0], [TX0, TY, s[0]], [TX0, -TY, s[0]]], [1, 0, 0]), { tone: "lo", crease: "none" }));
    // Floor: one surface of strips, shaded by slope, outlined only along its two long edges.
    const shades = ["", "", "", ""];
    let top = "";
    for (let i = 0; i < XS.length - 1; i++) {
      const n = [-(s[i + 1] - s[i]), 0, DX];
      const l = Math.hypot(n[0], n[2]);
      const nn = [n[0] / l, 0, n[2] / l];
      if (dot(nn, V) <= 0) continue;
      const quad = pathOf([[XS[i], -TY, s[i]], [XS[i + 1], -TY, s[i + 1]], [XS[i + 1], TY, s[i + 1]], [XS[i], TY, s[i]]], true);
      const band = G.bandOf(dot(nn, L));
      if (band === 4) top += quad;
      else shades[band] += quad;
    }
    const rim = [...prof(-TY), ...prof(TY).reverse()];
    const fill = pathOf(rim, true);
    out.push(S({ fill, outline: k.pathOf(prof(-TY).map(at)), crease: "", top, shades }, { tone: "lo", crease: "none" }));
    // Bucket scrapes: faint lines along the floor following its slope, ending on both walls.
    const scrapes = [-7, 0, 7].map((y) => k.pathOf(prof(y).map(at)));
    out.push(Ln(scrapes.join(""), { tone: "faint" }));
    return out.join("");
  }

  // Section through the near wall of the trench: in front of everything inside it.
  function trenchFront(s) {
    const out = [];
    const pts = [[TX0, TY, -GD], [TX1, TY, -GD], ...XS.map((x, i) => [x, TY, s[i]]).reverse()];
    const sec = face(pts, [0, 1, 0]);
    out.push(S(sec, { tone: "lo", crease: "none" }));
    out.push(...strata([2], (x, y) => (x <= TX1 ? soilAt(s, x) : 99)).map((run) => { const pts = run.filter((p) => p[0] <= TX1 + 0.01); return pts.length > 1 ? Ln(k.pathOf(pts.map(at)), { tone: "faint" }) : ""; }));
    out.push(D(stonesOn([2], (x) => (x < TX1 ? soilAt(s, x) : -99)), { size: 0.5, tone: "lo" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- static ground (end block, in front)
  function groundEnd() {
    const out = [];
    out.push(S(k.extrude([[TX1, GY0], [GX1, GY0], [GX1, TY], [TX1, TY]], -GD, GD, P), { tone: "lo", crease: "faint" }));
    out.push(...strata([2, 3], (x) => (x < TX1 ? 99 : 0)).map((run) => {
      const pts = run.filter((p) => p[0] >= TX1 - 0.01);
      return pts.length > 1 ? Ln(k.pathOf(pts.map(at)), { tone: "faint" }) : "";
    }));
    out.push(D(stonesOn([3]), { size: 0.5, tone: "lo" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- spoil pile
  function pile(volume) {
    const { r, h } = pileGeom(volume);
    if (r < 2) return "";
    const F = G.frameOf([PILE.x, 0, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0]);
    const prof = [[0, r], [h * 0.3, r * 0.74, 1], [h * 0.7, r * 0.34, 1], [h, 0.6]];
    const out = [`<path class="iso-halo" d="${k.haloOf({ x: PILE.x - r, y: -r, w: 2 * r, d: 2 * r, r }, 0, 0.1, P)}" style="filter:blur(3px)"/>`];
    out.push(S(G.solidOf(prof, F, P, { rims: "first" }), { tone: "mid", crease: "none" }));
    // Clods on the visible face.
    const clods = [];
    for (let i = 0; i < 26; i++) {
      const a = (i * 2.399) % (2 * Math.PI);
      const f = 0.2 + ((i * 0.37) % 0.7);
      const p = [PILE.x + Math.cos(a) * r * f, Math.sin(a) * r * f, h * (1 - f) * 0.92];
      if (Math.cos(a) * V[0] + Math.sin(a) * V[1] > 0.05) clods.push(at(p));
    }
    out.push(D(clods, { size: 0.55, tone: "lo" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- tracks and car body
  function track(side) {
    const out = [];
    const yc = side * TRACK.y;
    const y0 = yc - TRACK.w, y1 = yc + TRACK.w;
    const H = TRACK.half, R = TRACK.r;
    const loop = [];
    for (let i = 0; i <= 16; i++) { const a = -Math.PI / 2 + (i / 16) * Math.PI; loop.push([H - R + R * Math.cos(a), R + R * Math.sin(a)]); }
    for (let i = 0; i <= 16; i++) { const a = Math.PI / 2 + (i / 16) * Math.PI; loop.push([-H + R + R * Math.cos(a), R + R * Math.sin(a)]); }
    out.push(S(xz(loop, y0, y1), { tone: "lo", crease: "faint" }));
    // Shoe joints all round the visible part of the belt.
    const marks = [];
    const per = [];
    const total = 2 * (2 * H - 2 * R) + 2 * Math.PI * R;
    for (let u = 2; u < total; u += 5.6) {
      let p, n;
      const straight = 2 * H - 2 * R;
      if (u < straight) { p = [H - R - u, 2 * R]; n = [0, 1]; }
      else if (u < straight + Math.PI * R) { const a = Math.PI / 2 + (u - straight) / R; p = [-H + R + R * Math.cos(a), R + R * Math.sin(a)]; n = [Math.cos(a), Math.sin(a)]; }
      else if (u < 2 * straight + Math.PI * R) { p = [-H + R + (u - straight - Math.PI * R), 0]; n = [0, -1]; }
      else { const a = -Math.PI / 2 + (u - 2 * straight - Math.PI * R) / R; p = [H - R + R * Math.cos(a), R + R * Math.sin(a)]; n = [Math.cos(a), Math.sin(a)]; }
      if (n[0] * V[0] + n[1] * V[2] > 0.12) marks.push(k.segment([p[0], y0, p[1]], [p[0], y1, p[1]], P));
      per.push(p);
    }
    out.push(Ln(marks.join(""), { tone: "faint" }));
    // Track frame on the outer face with idler, sprocket and rollers.
    const fr = [[-H + R + 2, 6], [H - R - 2, 6], [H - R - 6, 14.5], [-H + R + 6, 14.5]];
    const face1 = side > 0 ? y1 : y0;
    if (side > 0) {
      out.push(S(xz(fr, y1 - 1, y1 + 1), { tone: "mid", crease: "faint" }));
      const roll = [-24, -12, 0, 12, 24].map((x) => [x, 4.2]);
      for (const p of roll) out.push(S(pinY(p, 2.8, y1, y1 + 1.6), { tone: "mid", crease: "none" }));
      out.push(S(pinY([H - R, R], 6.6, y1, y1 + 1.4), { tone: "mid" }));
      out.push(S(pinY([H - R, R], 2.6, y1 + 1.4, y1 + 2.2), { tone: "mid", crease: "none" }));
      out.push(S(pinY([-H + R, R], 7, y1, y1 + 1.8), { tone: "mid" }));
      out.push(D(G.dotsOf(G.frameAlong([-H + R, y1 + 1.8, R], [0, 1, 0]), 0, 4.4, 9, P, { all: true }), { size: 0.45, tone: "lo" }));
      out.push(S(pinY([-H + R, R], 3, y1 + 1.8, y1 + 3.2), { tone: "mid" }));
      out.push(S(pinY([2, 16.6], 2.4, y1 - 0.6, y1 + 1), { tone: "mid", crease: "none" }));
      out.push(D([at([-20, y1 + 1, 10]), at([20, y1 + 1, 10]), at([0, y1 + 1, 10])], { size: 0.45, tone: "lo" }));
    } else {
      out.push(S(xz(fr, face1 - 1, face1), { tone: "mid", crease: "none" }));
    }
    return out.join("");
  }

  function carBody() {
    const out = [];
    out.push(S(k.slabOf({ x: -24, y: -18, w: 48, d: 36, r: 6 }, 6, 10, P, 6, 0.5), { tone: "mid" }));
    out.push(S(k.cylinder(0, 0, 21, 16, 5, P, 64, 0.6), { tone: "lo" }));
    out.push(D(G.dotsOf(G.frameOf([0, 0, 21], [0, 0, 1], [1, 0, 0], [0, 1, 0]), 0, 18.5, 28, P, { fade: [0.04, 0.42] }), { size: 0.45, tone: "mid" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- upper structure (behind the boom)
  function house() {
    const out = [];
    const deck = { x: -48, y: -25, w: 80, d: 50, r: 3 };
    out.push(S(k.slabOf(deck, 21, 5, P, 4, 0.6), { tone: "mid" }));
    // Counterweight: a heavy rounded block at the tail, with lifting eyes.
    const cw = { x: -58, y: -25, w: 22, d: 50, r: 11 };
    out.push(S(k.slabOf(cw, 22, 22, P, 10, 1.2), { tone: "mid" }));
    out.push(Ln(k.sideSeam(cw, 30, P, 10), { tone: "faint" }));
    for (const y of [-16, 16]) {
      out.push(S(k.slabOf({ x: -48, y: y - 2, w: 4, d: 4, r: 1 }, 44, 2.4, P, 3), { tone: "mid", crease: "none" }));
      out.push(L_ring(-46, y, 1, 46.4));
    }
    // Engine hood with louvres and an exhaust stack.
    const hood = { x: -38, y: -25, w: 32, d: 33, r: 2 };
    out.push(S(k.slabOf(hood, 26, 17, P, 4, 0.8), { tone: "mid" }));
    const louv = [];
    for (let i = 0; i < 6; i++) louv.push(k.planOutline({ x: -34 + i * 4.6, y: -20, w: 2.6, d: 18, r: 1 }, 43, P, 3));
    out.push(Ln(louv.join(""), { tone: "lo" }));
    out.push(Ln(k.sideSeam(hood, 36, P, 4), { tone: "faint" }));
    out.push(S(k.cylinder(-20, -15, 2.4, 43, 1.4, P, 20), { tone: "mid" }));
    out.push(S(k.cylinder(-20, -15, 1.6, 44.4, 12, P, 20), { tone: "mid", crease: "none" }));
    out.push(Ln(k.sideArc(-20, -15, 1.6, 54, P, 12), { tone: "lo" }));
    // Tanks and tool box on the right front.
    const tank = { x: -4, y: -25, w: 22, d: 14, r: 2 };
    out.push(S(k.slabOf(tank, 26, 15, P, 4, 0.6), { tone: "mid" }));
    out.push(S(k.cylinder(4, -19, 2, 41, 1.4, P, 20), { tone: "mid" }));
    out.push(Ln(k.sideSeam(tank, 33, P, 4), { tone: "faint" }));
    // Handrail along the right edge of the hood.
    const rail = [[-34, -23.4], [-22, -23.4], [-10, -23.4]];
    for (const [x, y] of rail) out.push(S(k.cylinder(x, y, 0.7, 43, 5, P, 10), { tone: "mid", crease: "none" }));
    out.push(Ln(k.segment([-34, -23.4, 48], [-10, -23.4, 48], P), { tone: "mid" }));
    // Boom foot brackets, far plate (the near one is drawn over the boom).
    out.push(S(xz([[14, 26], [34, 26], [30, 44], [22, 45]], -10, -7), { tone: "mid" }));
    // Boom cylinder lugs at the front of the deck.
    for (const y of [-15.5, 10.5]) out.push(S(xz([[30, 21], [40, 21], [38, 28], [33, 29]], y, y + 5), { tone: "mid", crease: "none" }));
    return out.join("");
  }
  function L_ring(x, y, r, z) { return Ln(k.ring(x, y, r, z, P, 12), { tone: "lo" }); }

  function nearBracket() {
    return S(xz([[14, 26], [34, 26], [30, 44], [22, 45]], 7, 10), { tone: "mid" }) +
      S(pinY(FOOT, 4, 10, 11.6), { tone: "mid" }) + D([at([FOOT[0], 11.6, FOOT[1]])], { size: 0.5, tone: "hi" });
  }

  // ---------------------------------------------------------------- cab (in front of the boom)
  function cab() {
    const out = [];
    const c = { x: -4, y: 10, w: 30, d: 15, r: 2 };
    out.push(S(k.slabOf({ x: -6, y: 9, w: 34, d: 17, r: 1 }, 26, 2, P, 3), { tone: "mid" }));
    out.push(S(k.slabOf(c, 28, 32, P, 4, 0.5), { tone: "mid" }));
    out.push(S(k.slabOf({ x: -5, y: 9, w: 32, d: 17, r: 2.4 }, 60, 2, P, 4, 0.6), { tone: "mid" }));
    // Side windows on the near face (y = 25): door glass and rear quarter.
    const y = 25;
    const win = (pts) => Ln(k.pathOf(pts.map(([x, z]) => at([x, y, z])), true), { tone: "lo" });
    out.push(win([[8, 34], [24, 34], [24, 57], [11, 57], [8, 52]]));
    out.push(win([[-1, 40], [5, 40], [5, 57], [-1, 57]]));
    out.push(Ln(k.segment([6.5, y, 28], [6.5, y, 60], P), { tone: "faint" }));
    out.push(D([at([9.6, y, 44])], { size: 0.55, tone: "mid" }));
    // Front screen on the x = 26 face, with a wiper.
    const fx = 26;
    out.push(Ln(k.pathOf([[11.6, 33], [24, 33], [24, 58], [11.6, 58]].map(([yy, z]) => at([fx, yy, z])), true), { tone: "lo" }));
    out.push(Ln(k.segment([fx, 18, 34], [fx, 21, 47], P), { tone: "faint" }));
    // Mirror on a stalk and a work light on the roof.
    out.push(Ln(k.segment([25, 24.4, 54], [29, 27.4, 56], P), { tone: "mid" }));
    out.push(S(k.slabOf({ x: 28.4, y: 26.6, w: 1.6, d: 3.2, r: 0.4 }, 54, 5, P, 2), { tone: "mid", crease: "none" }));
    out.push(S(k.slabOf({ x: 18, y: 14, w: 5, d: 4, r: 1 }, 62, 2.4, P, 3), { tone: "mid" }));
    // Step and grab rail by the door.
    out.push(S(k.slabOf({ x: 4, y: 25, w: 8, d: 2.6, r: 0.6 }, 24, 1.2, P, 2), { tone: "mid", crease: "none" }));
    out.push(Ln(k.segment([24.8, 25.6, 36], [24.8, 25.6, 52], P), { tone: "mid" }));
    out.push(D([at([24.8, 25.6, 36]), at([24.8, 25.6, 52])], { size: 0.5, tone: "mid" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- hydraulic cylinder
  function cylinder(p0, p1, y, { barrel, rb, rr, eye = 3.4, half = 2.6 }) {
    const a3 = [p1[0] - p0[0], 0, p1[1] - p0[1]];
    const length = Math.hypot(a3[0], a3[2]);
    const F = G.frameAlong([p0[0], y, p0[1]], a3);
    const bar = Math.min(barrel, length - 4);
    const parts = [];
    parts.push(S(G.disc(2.4, bar, rb, F, P), { tone: "mid" }));
    parts.push(S(G.disc(bar - 2.6, bar, rb + 0.6, F, P), { tone: "mid" }));
    const rod = S(G.disc(bar, length - 2.6, rr, F, P), { tone: "hi", crease: "none" });
    const pins = [S(pinY(p0, eye, y - half, y + half), { tone: "mid" }), S(pinY(p1, eye * 0.85, y - half, y + half), { tone: "mid" })];
    const nearEnd = dot([a3[0] / length, 0, a3[2] / length], V) > 0;
    const body = nearEnd ? [parts.join(""), rod] : [rod, parts.join("")];
    return [pins[0], pins[1], ...body, D([at([p0[0], y + half, p0[1]]), at([p1[0], y + half, p1[1]])], { size: 0.5, tone: "hi" })].join("");
  }

  // ---------------------------------------------------------------- boom
  function boomParts(q) {
    const B = local(FOOT, q.tb);
    const [h0, h1, h2] = BOOM.h;
    const kn = BOOM.knee;
    const t1 = Math.atan2(kn[1], kn[0]);
    const t2 = Math.atan2(-kn[1], BOOM.len - kn[0]);
    const n1 = [-Math.sin(t1), Math.cos(t1)], n2 = [-Math.sin(t2), Math.cos(t2)];
    const nb = [(n1[0] + n2[0]) / 2, (n1[1] + n2[1]) / 2];
    const nl = Math.hypot(nb[0], nb[1]);
    const nbu = [nb[0] / nl, nb[1] / nl];
    const miter = h1 / (n1[0] * nbu[0] + n1[1] * nbu[1]);
    const seg1 = [add([0, 0], mul(n1, -h0)), add(kn, mul(nbu, -miter)), add(kn, mul(nbu, miter)), add([0, 0], mul(n1, h0))].map(B);
    const seg2 = [add(kn, mul(nbu, -miter)), add([BOOM.len, 0], mul(n2, -h2)), add([BOOM.len, 0], mul(n2, h2)), add(kn, mul(nbu, miter))].map(B);
    const w = BOOM.w;
    const out = [];
    out.push(S(xz(seg1, -w, w), { tone: "hi", crease: "lo" }));
    out.push(S(xz(seg2, -w, w), { tone: "hi", crease: "lo" }));
    // Weld seams on the near side plate, inset from the flanges, fading at both ends.
    for (const off of [-1, 1]) {
      const pts = [];
      for (let i = 0; i <= 20; i++) {
        const s = 8 + (i / 20) * (BOOM.len - 16);
        let p, n, h;
        if (s < kn[0]) { const t = s / kn[0]; p = [kn[0] * t, kn[1] * t]; n = n1; h = h0 + (h1 - h0) * t; }
        else { const t = (s - kn[0]) / (BOOM.len - kn[0]); p = [kn[0] + (BOOM.len - kn[0]) * t, kn[1] * (1 - t)]; n = n2; h = h1 + (h2 - h1) * t; }
        const q2 = B(add(p, mul(n, off * (h - 2.2))));
        pts.push(at([q2[0], w, q2[1]]));
      }
      out.push(k.fadeLineSvg(pts, { fade: [6, 6], tone: "lo" }));
    }
    // Bosses: foot, cylinder lug underneath, stick-cylinder lug on top, nose.
    out.push(S(pinY(FOOT, 5.4, w, w + 1), { tone: "hi", crease: "lo" }));
    out.push(S(xz([[44, -6], [56, -6.4], [50, -13.6]].map(B), -4.5, 4.5), { tone: "hi", crease: "lo" }));
    out.push(S(pinY(q.boomLug, 2.6, 4.5, 5.4), { tone: "hi", crease: "lo" }));
    out.push(S(xz([[52, 8.6], [68, 7.4], [60, 24.4]].map(B), -3.4, 3.4), { tone: "hi", crease: "lo" }));
    out.push(S(pinY(q.stickLug, 2.8, 3.4, 4.3), { tone: "hi", crease: "lo" }));
    // Work light on the near plate.
    const lamp = B([30, 3]);
    out.push(S(pinY(lamp, 2.6, w, w + 2.4), { tone: "mid" }));
    out.push(Ln(k.sideRing([lamp[0], w + 2.4, lamp[1]], 1.6, "y", P, 16), { tone: "lo" }));
    // Hydraulic lines on top of the boom, clamped twice.
    const hose = [12, 40, 70, 100].map((s) => {
      const t = s < kn[0] ? s / kn[0] : (s - kn[0]) / (BOOM.len - kn[0]);
      const p = s < kn[0] ? [kn[0] * t, kn[1] * t] : [kn[0] + (BOOM.len - kn[0]) * t, kn[1] * (1 - t)];
      const n = s < kn[0] ? n1 : n2;
      const h = s < kn[0] ? h0 + (h1 - h0) * t : h1 + (h2 - h1) * t;
      return B(add(p, mul(n, h + 1.4)));
    });
    for (const yy of [-3, 3]) {
      out.push(Ln(k.pathOf(hose.map((p) => at([p[0], yy, p[1]]))), { tone: "mid" }));
      out.push(D([hose[0], hose[hose.length - 1]].map((p) => at([p[0], yy, p[1]])), { size: 0.8, tone: "mid" }));
    }
    for (const c of [hose[1], hose[2]]) out.push(S(G.disc(-4.4, 4.4, 1.6, G.frameAlong([c[0], 0, c[1]], [0, 1, 0]), P), { tone: "mid", crease: "none" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- stick
  function stickParts(q) {
    const Sk = q.Sk;
    const w = STICK.w;
    const out = [];
    out.push(S(xz(SK.outline.map(Sk), -w, w), { tone: "hi", crease: "lo" }));
    // Near-plate seams and bosses.
    const seam = (a0, a1, b0, b1) => {
      const pts = [];
      for (let i = 0; i <= 12; i++) { const t = i / 12; const p = Sk([a0 + (a1 - a0) * t, b0 + (b1 - b0) * t]); pts.push(at([p[0], w, p[1]])); }
      return k.fadeLineSvg(pts, { fade: [5, 5], tone: "lo" });
    };
    out.push(seam(6, 52, 10.4, 2.4), seam(6, 52, -5.6, -2.6));
    out.push(S(pinY(q.E, 5, w, w + 1), { tone: "hi", crease: "lo" }));
    out.push(S(pinY(q.heel, 3.4, w, w + 0.9), { tone: "hi", crease: "lo" }));
    out.push(S(pinY(q.K, 3, w, w + 0.9), { tone: "hi", crease: "lo" }));
    out.push(S(xz([[1, 12], [15, 10.6], [8, 19]].map(Sk), -3, 3), { tone: "hi", crease: "lo" }));
    out.push(D([q.E, q.heel, q.K].map((p) => at([p[0], w + 1, p[1]])), { size: 0.55, tone: "hi" }));
    out.push(D([[20, 0], [36, 0.6]].map(Sk).map((p) => at([p[0], w, p[1]])), { size: 0.45, tone: "lo" }));
    return out.join("");
  }

  // ---------------------------------------------------------------- bucket
  function bucketParts(q, fill) {
    const Bk = q.Bk;
    const W = BUCKET_W;
    const out = { back: [], front: [] };
    const plate = BK.plate.map(Bk);
    out.back.push(S(xz(plate, -W, -W + 1), { tone: "hi", crease: "lo" }));
    // Shell: strips along the outer curve, each 1 unit thick, depth sorted.
    const strips = [];
    for (let i = 0; i < BK.shell.length - 1; i++) {
      const a = BK.shell[i], b = BK.shell[i + 1];
      const d = sub(b, a);
      const l = len(d);
      const n = [d[1] / l, -d[0] / l]; // outward (toward +b side of the curve)
      const inward = mul(n, -1.1);
      const poly = [a, b, add(b, inward), add(a, inward)].map(Bk);
      const mid = Bk([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2]);
      strips.push({ key: dot([mid[0], 0, mid[1]], V), svg: S(xz(poly, -W + 0.5, W - 0.5), { tone: "hi", crease: "none" }) });
    }
    strips.sort((p, q2) => p.key - q2.key).forEach((s) => out.back.push(s.svg));
    // Soil: fills from the back of the bowl toward the mouth, then heaps above it.
    if (fill > 0.01) {
      const T = BK.T, Lp = BK.L;
      const m = sub(Lp, T);
      const ml = len(m);
      const nIn = [-m[1] / ml, m[0] / ml]; // into the bowl (toward +b)
      const depth = 15.5;
      const struck = Math.min(1, fill / 0.8);
      const cut = depth * (1 - struck);
      const inside = [];
      const poly = BK.plate;
      for (let i = 0; i < poly.length; i++) {
        const p = poly[i], r = poly[(i + 1) % poly.length];
        const dp = (p[0] - T[0]) * nIn[0] + (p[1] - T[1]) * nIn[1] - cut;
        const dr = (r[0] - T[0]) * nIn[0] + (r[1] - T[1]) * nIn[1] - cut;
        if (dp >= 0) inside.push(p);
        if ((dp >= 0) !== (dr >= 0)) { const t = dp / (dp - dr); inside.push([p[0] + (r[0] - p[0]) * t, p[1] + (r[1] - p[1]) * t]); }
      }
      if (inside.length > 2) out.back.push(S(xz(inside.map(Bk), -W + 1, W - 1), { tone: "lo", crease: "none", className: "soil" }));
      if (fill > 0.8) {
        const h = ((fill - 0.8) / 0.2) * 6.5;
        const heap = [T, [T[0] + m[0] * 0.3 - nIn[0] * h * 0.9, T[1] + m[1] * 0.3 - nIn[1] * h * 0.9], [T[0] + m[0] * 0.62 - nIn[0] * h, T[1] + m[1] * 0.62 - nIn[1] * h], Lp];
        out.back.push(S(xz(heap.map(Bk), -W + 1.2, W - 1.2), { tone: "lo", crease: "none", className: "soil" }));
        const grit = [];
        for (let i = 1; i < 9; i++) {
          const t = i / 9;
          const c = Bk([T[0] + m[0] * t - nIn[0] * h * 0.5 * Math.sin(Math.PI * t), T[1] + m[1] * t - nIn[1] * h * 0.5 * Math.sin(Math.PI * t)]);
          grit.push(at([c[0], ((i * 5.3) % 18) - 9, c[1]]));
        }
        out.back.push(D(grit, { size: 0.5, tone: "mid" }));
      }
    }
    // Near plate with its inset weld line, wear strip and pin boss.
    const near = xz(plate, W - 1, W);
    const inset = BK.plate.map(([a, b]) => [a + (14 - a) * 0.1, b + (4 - b) * 0.1]).map(Bk);
    out.front.push(S(near, { tone: "hi", crease: "lo", inner: Ln(k.pathOf(inset.map((p) => at([p[0], W, p[1]])), true), { tone: "faint" }) }));
    out.front.push(S(xz([[26, -2.4], [32.6, -2.2], [32.4, 2], [27.6, 5.2]].map(Bk), W, W + 0.6), { tone: "mid", crease: "none" }));
    out.front.push(S(pinY(q.pin, 4.4, W, W + 1.2), { tone: "hi", crease: "lo" }));
    out.front.push(S(pinY(q.Q, 3.4, W, W + 1), { tone: "hi", crease: "lo" }));
    out.front.push(D([q.pin, q.Q].map((p) => at([p[0], W + 1.2, p[1]])), { size: 0.55, tone: "hi" }));
    // Cutting edge and five teeth: the one lit accent.
    const edge = [[29.6, -3.4], [33.6, -3], [33.6, 0.8], [29.8, 1.4]].map(Bk);
    out.front.push(S(xz(edge, -W, W), { tone: "hi", crease: "lo", lit: true }));
    const teeth = [-9.6, -4.8, 0, 4.8, 9.6].map((y) => ({ y, key: y }));
    for (const t of teeth) {
      const tooth = [[32.6, -3.2], [38.6, -2.1], [38.9, -1.2], [32.8, 0.8]].map(Bk);
      out.front.push(S(xz(tooth, t.y - 1.5, t.y + 1.5), { tone: "hi", crease: "lo", lit: true }));
    }
    return out;
  }

  // A flat bar between two pins, rounded at both ends, in the x–z plane.
  function bar(p, r2, r, y0, y1, tone = "hi") {
    const d = sub(r2, p);
    const l = len(d);
    const a = Math.atan2(d[1], d[0]);
    const pts = [];
    for (let i = 0; i <= 10; i++) { const t = a + Math.PI / 2 + (i / 10) * Math.PI; pts.push([p[0] + r * Math.cos(t), p[1] + r * Math.sin(t)]); }
    for (let i = 0; i <= 10; i++) { const t = a - Math.PI / 2 + (i / 10) * Math.PI; pts.push([r2[0] + r * Math.cos(t), r2[1] + r * Math.sin(t)]); }
    return S(xz(pts, y0, y1), { tone, crease: "lo" }) + D([at([p[0], y1, p[1]]), at([r2[0], y1, r2[1]])], { size: 0.5, tone: "hi" });
  }

  // The whole front attachment, back to front.
  function arm(q, fill) {
    const bucket = bucketParts(q, fill);
    const out = [];
    out.push(cylinder(q.houseLug, q.boomLug, -13, { barrel: 34, rb: 2.8, rr: 1.6, eye: 2.8, half: 2 }));
    out.push(bar(q.K, q.J, 2.2, -9.6, -8));
    out.push(...bucket.back);
    out.push(boomParts(q));
    out.push(nearBracket());
    out.push(cylinder(q.houseLug, q.boomLug, 13, { barrel: 34, rb: 2.8, rr: 1.6, eye: 2.8, half: 2 }));
    out.push(cylinder(q.stickLug, q.heel, 0, { barrel: 44, rb: 3, rr: 1.7, eye: 3, half: 2.4 }));
    out.push(stickParts(q));
    out.push(cylinder(q.stickBase, q.J, 0, { barrel: 32, rb: 2.6, rr: 1.5, eye: 2.6, half: 2 }));
    out.push(bar(q.Q, q.J, 2.4, -5, 5));
    out.push(...bucket.front);
    out.push(bar(q.K, q.J, 2.2, 8, 9.6));
    return out.join("");
  }

  // Region of the y = +12 plane above the soil, as a screen path: the attachment is clipped to it,
  // so anything the bucket pushes below the ground or the trench floor is hidden by the soil.
  function aboveSoil(s) {
    const y = BUCKET_W;
    const pts = [[GX0 - 400, 700], [GX1 + 400, 700], [GX1 + 400, 0], [TX1, 0], ...XS.map((x, i) => [x, s[i]]).reverse(), [TX0, 0], [GX0 - 400, 0]];
    return k.pathOf(pts.map(([x, z]) => at([x, y, z])), true);
  }

  return { groundBack, trenchBack, trenchFront, groundEnd, pile, track, carBody, house, cab, arm, aboveSoil };
}
