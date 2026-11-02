// Build: node figures/excavator/build.mjs [--light]
// Needs the anatomy skill kit at ~/.claude/skills/anatomy (installed by .claude/hooks/session-start.sh).
import { readFileSync, writeFileSync } from "node:fs";
import { homedir } from "node:os";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import * as SC from "./scene.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const KIT = process.env.ANATOMY_KIT || join(homedir(), ".claude", "skills", "anatomy", "kit");
const k = await import(pathToFileURL(join(KIT, "iso-kit.mjs")).href);
const G = await import(pathToFileURL(join(KIT, "lathe.mjs")).href);
const theme = process.argv.includes("--light") ? "light" : "dark";

const WIDTH = 640;
const HEIGHT = 420;
const P = k.fitProjection(
  [...k.boxCorners({ x: SC.GX0, y: SC.GY0, w: SC.GX1 - SC.GX0, d: SC.GY1 - SC.GY0, r: 0 }, -SC.GD, 0), [60, -10, 128], [150, -10, 128]],
  WIDTH,
  HEIGHT,
  { pad: 18, azimuth: 58, elevation: 30 },
);

const scene = SC.makeScene(k, G, P);
const s0 = SC.XS.map(() => SC.S0);
const start = (() => {
  let curl = 0, j = null;
  for (let i = 0; i < 6; i++) { j = SC.solve([150, 42], curl, -1.6); curl = Math.max(-156, Math.min(24, -80 - j.boom - j.stick)); }
  return { boom: j.boom, stick: j.stick, curl };
})();
const q0 = SC.pose(start.boom, start.stick, start.curl);

const svg = k.figureSvg({
  width: WIDTH,
  height: HEIGHT,
  label:
    "굴착기 한 대가 도랑 앞에 서 있다. 앞쪽 흙은 단면으로 잘려 표토, 점토, 자갈층이 보인다. 붐, 암, 버킷이 세 개의 유압 실린더로 움직여 버킷 날로 도랑 바닥을 긁어 흙을 담고, 들어 올려 궤도 앞 사토 더미에 쏟는다.",
  body: [
    `<defs><clipPath id="ex-soil"><path data-part="clip" d="${scene.aboveSoil(s0)}"/></clipPath></defs>`,
    scene.groundBack(),
    `<g data-part="trench-back">${scene.trenchBack(s0)}</g>`,
    `<g data-part="pile">${scene.pile(0)}</g>`,
    scene.track(-1),
    scene.carBody(),
    scene.track(1),
    scene.house(),
    `<g data-part="arm" clip-path="url(#ex-soil)">${scene.arm(q0, 0)}</g>`,
    scene.cab(),
    `<g data-part="spill"></g>`,
    `<g data-part="trench-front">${scene.trenchFront(s0)}</g>`,
    scene.groundEnd(),
  ],
});

const strip = (text) => text.replace(/^import .*$/gm, "").replace(/^export /gm, "");
const lathe = strip(readFileSync(join(KIT, "lathe.mjs"), "utf8"))
  .replace(/\brunsOf\b/g, "L_runsOf")
  .replace(/\bsignedArea\b/g, "L_signedArea")
  .replace(/\boriented\b/g, "L_oriented");
const kitSrc = strip(readFileSync(join(KIT, "iso-kit.mjs"), "utf8"));
const sceneSrc = strip(readFileSync(join(HERE, "scene.mjs"), "utf8"));
const kitNames = [...kitSrc.matchAll(/^(?:function|const) (\w+)/gm)].map((m) => m[1]);
const latheNames = [...lathe.matchAll(/^(?:function|const) (\w+)/gm)].map((m) => m[1]);

const live = `
const k = { ${kitNames.join(", ")} };
const G = { ${latheNames.join(", ")} };
const P = ${JSON.stringify(P)};
const scene = makeScene(k, G, P);
const stage = document.querySelector(".iso-stage");
const svg = stage.querySelector("svg");
const $ = (name) => svg.querySelector('[data-part="' + name + '"]');
const layer = { arm: $("arm"), pile: $("pile"), back: $("trench-back"), front: $("trench-front"), spill: $("spill"), clip: $("clip") };
const readout = document.querySelector("[data-readout]");
const still = matchMedia("(prefers-reduced-motion: reduce)");
const clampN = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const smooth = (t) => t * t * (3 - 2 * t);
const curlDeg = (c) => LIM.curl[0] + (LIM.curl[1] - LIM.curl[0]) * c;

const st = {
  s: XS.map(() => S0), ref: XS.map(() => S0), pile: 0, fill: 0, moved: 0,
  cur: { x: 150, z: 42, c: ${(24 - start.curl) / 180} },
  j: { boom: ${start.boom}, stick: ${start.stick}, c: ${(24 - start.curl) / 180} },
  aim: { x: 150, z: 42, c: ${(24 - start.curl) / 180} },
  bits: [], dirty: { trench: false, pile: false },
  reset: 0,
};

// Tour: one dig cycle, teeth positions in world units, z relative to the soil where marked.
const TOUR = [
  { t: 0, x: 150, z: 42, phi: -80 },
  { t: 2.6, x: 206, z: -10, rel: true, phi: -72 },
  { t: 5.8, x: 150, z: -10, rel: true, phi: -140 },
  { t: 7.2, x: 144, z: 4, rel: true, phi: -188 },
  { t: 9.4, x: 128, z: 56, phi: -190 },
  { t: 10.8, x: 106, z: 42, phi: -186 },
  { t: 12.4, x: 104, z: 38, phi: -60 },
  { t: 13.8, x: 112, z: 46, phi: -64 },
  { t: 15.6, x: 150, z: 42, phi: -80 },
];
const CYCLE = TOUR[TOUR.length - 1].t;
function tourAt(t) {
  const u = t % CYCLE;
  let i = 0;
  while (i < TOUR.length - 2 && TOUR[i + 1].t <= u) i++;
  const a = TOUR[i], b = TOUR[i + 1];
  const f = smooth(clampN((u - a.t) / (b.t - a.t), 0, 1));
  const za = a.rel ? soilAt(st.ref, a.x) + a.z : a.z;
  const zb = b.rel ? soilAt(st.ref, b.x) + b.z : b.z;
  return { x: a.x + (b.x - a.x) * f, z: za + (zb - za) * f, phi: a.phi + (b.phi - a.phi) * f };
}
// The tour steers the bucket's attitude; the operator's control is the curl relative to the arm.
const cOfPhi = (phi) => clampN((phi - (st.j.boom + st.j.stick) - LIM.curl[0]) / (LIM.curl[1] - LIM.curl[0]), 0, 1);

const surfaceAt = (x) => (x > TX0 && x < TX1 ? soilAt(st.s, x) : Math.max(0, pileHeight(st.pile, x)));

// The teeth may bite at most 0.7 m below the floor as it was when they entered it, and not at all once the bucket is full.
function floorAt(x) {
  const inTrench = x > TX0 + 2 && x < TX1 - 2;
  if (!inTrench) return surfaceAt(x) + 0.5;
  if (st.fill >= CAPACITY - 0.005) return soilAt(st.s, x) - 0.5;
  return Math.max(soilAt(st.ref, x) - 11, FLOOR_MIN + 2);
}
const limitX = (x) => clampN(x, 60, TX1 - 8);

// The teeth shave the floor under them with a soft edge 1.2 m wide (the bucket is 1.2 m wide across the trench);
// what they shave goes into the bucket until it holds its rated 1.0 m³.
function dig(q) {
  const tip = q.tip;
  if (tip[0] <= TX0 + 1 || tip[0] >= TX1 - 1 || st.j.c > 0.92) return;
  const per = (DX / M) * TRENCH_W / M;
  for (let i = 0; i < XS.length; i++) {
    const d = Math.abs(XS[i] - tip[0]);
    if (d > 9) continue;
    const w = d < 5 ? 1 : 0.5 + 0.5 * Math.cos((Math.PI * (d - 5)) / 4);
    const floor = Math.max(tip[1] + (1 - w) * 6, FLOOR_MIN);
    if (st.s[i] <= floor) continue;
    const room = Math.max(0, CAPACITY - st.fill);
    const dz = Math.min(st.s[i] - floor, room / per);
    if (dz <= 0) return;
    st.s[i] -= dz;
    st.fill += dz * per;
    st.moved += dz * per;
    st.dirty.trench = true;
  }
}

let rand = 11;
const rnd = () => ((rand = (rand * 16807) % 2147483647) / 2147483647);
function spill(q, dt, calm) {
  const mz = q.mouth[1];
  if (st.fill <= 0 || mz > -0.18 || q.lip[1] < surfaceAt(q.lip[0]) + 2 || q.pin[1] < surfaceAt(q.pin[0]) + 4) return;
  const rate = 2.4 * clampN((-mz - 0.18) / 0.5, 0, 1);
  let out = Math.min(st.fill, rate * dt);
  st.fill -= out;
  if (calm) { land(q.lip[0], out); return; }
  const lumps = Math.max(1, Math.round(out / 0.006));
  for (let i = 0; i < lumps; i++) {
    const t = rnd();
    const x = q.lip[0] + (q.Bk ? 0 : 0) - t * 6;
    st.bits.push({ x: x + rnd() * 3, y: (rnd() - 0.5) * 20, z: q.lip[1] - rnd() * 2, vx: (rnd() - 0.5) * 8, vz: -rnd() * 10, v: out / lumps });
  }
}
function land(x, v) {
  if (x > TX0 && x < TX1) {
    const i = clampN(Math.round((x - TX0) / DX), 0, XS.length - 1);
    const per = (DX / M) * TRENCH_W / M;
    st.s[i] = Math.min(0, st.s[i] + v / per);
    st.dirty.trench = true;
  } else if (x <= TX0) { st.pile += v; st.dirty.pile = true; }
}
function fall(dt) {
  const g = 9.81 * M;
  st.bits = st.bits.filter((b) => {
    b.vz -= g * dt; b.x += b.vx * dt; b.z += b.vz * dt;
    const ground = surfaceAt(b.x);
    if (b.z <= ground) { land(b.x, b.v); return false; }
    return true;
  });
}

let q = pose(st.j.boom, st.j.stick, curlDeg(st.j.c));
let touring = !still.matches, clock = 0, idle = 0, frame = 0, last = 0, visible = false, armKey = "";
function draw() {
  const key = [st.j.boom, st.j.stick, st.j.c, st.fill].map((v) => v.toFixed(3)).join();
  if (key !== armKey) { armKey = key; layer.arm.innerHTML = scene.arm(q, Math.min(1, st.fill / CAPACITY)); }
  if (st.dirty.trench) {
    st.dirty.trench = false;
    layer.back.innerHTML = scene.trenchBack(st.s);
    layer.front.innerHTML = scene.trenchFront(st.s);
    layer.clip.setAttribute("d", scene.aboveSoil(st.s));
  }
  if (st.dirty.pile) { st.dirty.pile = false; layer.pile.innerHTML = scene.pile(st.pile); }
  layer.spill.innerHTML = st.bits.length ? k.dotsSvg(st.bits.map((b) => k.iso([b.x, b.y, b.z], P)), { size: 0.7, tone: "mid" }) : "";
  const depth = -Math.min(...st.s) / M;
  const text = "버킷 " + st.fill.toFixed(2) + " m³ · 깊이 " + depth.toFixed(1) + " m · 사토 " + st.pile.toFixed(1) + " m³";
  if (readout.textContent !== text) readout.textContent = text;
}

function tick(now) {
  const dt = last ? Math.min((now - last) / 1000, 1 / 30) : 1 / 60;
  last = now;
  const calm = still.matches;
  if (st.reset > 0) {
    const f = 1 - Math.exp(-dt / 0.4);
    st.s = st.s.map((v) => v + (S0 - v) * f);
    st.pile += (0 - st.pile) * f;
    st.dirty.trench = st.dirty.pile = true;
    st.reset -= dt;
    if (st.reset <= 0) { st.s = XS.map(() => S0); st.ref = st.s.slice(); st.pile = 0; }
  }
  if (touring) {
    const before = clock % CYCLE;
    clock += dt;
    if (clock % CYCLE < before && (st.pile > 3.4 || Math.min(...st.s) < FLOOR_MIN + 6)) st.reset = 2.2;
    const w = tourAt(clock);
    st.aim.x = w.x; st.aim.z = w.z; st.aim.c = cOfPhi(w.phi);
  } else if (!calm) {
    idle += dt;
    if (idle > 4) { touring = true; clock = 0; }
  }
  // Ease the teeth in the plane of the arm, keep them out of the soil they may not cut, then solve the joints.
  const f = calm ? 1 : 1 - Math.exp(-dt / 0.3);
  const fc = calm ? 1 : 1 - Math.exp(-dt / 0.4);
  const ax = limitX(st.aim.x);
  st.cur.x += (ax - st.cur.x) * f;
  st.cur.z += (clampN(st.aim.z, -GD, 104) - st.cur.z) * f;
  st.cur.c += (st.aim.c - st.cur.c) * fc;
  st.cur.z = Math.max(st.cur.z, floorAt(st.cur.x));
  const sol = solve([st.cur.x, st.cur.z], curlDeg(st.cur.c), ((st.j.boom + st.j.stick) * Math.PI) / 180);
  const fj = calm ? 1 : 1 - Math.exp(-dt / 0.06);
  st.j.boom += (sol.boom - st.j.boom) * fj;
  st.j.stick += (sol.stick - st.j.stick) * fj;
  st.j.c = st.cur.c;
  q = pose(st.j.boom, st.j.stick, curlDeg(st.j.c));
  if (q.tip[1] > soilAt(st.s, q.tip[0]) + 1) st.ref = st.s.slice();
  dig(q);
  spill(q, dt, calm);
  fall(dt);
  draw();
  frame = visible ? requestAnimationFrame(tick) : 0;
}
const run = () => { if (!frame && visible) { last = 0; frame = requestAnimationFrame(tick); } };
new IntersectionObserver((entries) => { visible = entries[entries.length - 1].isIntersecting; if (visible) run(); }, { rootMargin: "120px 0px" }).observe(stage);

// Pointer: the teeth follow the pointer in the plane of the arm; a tap or click curls or opens the bucket.
const o = k.iso([0, 0, 0], P), ex = k.iso([1, 0, 0], P), ez = k.iso([0, 0, 1], P);
const ax = [ex[0] - o[0], ex[1] - o[1]], az = [ez[0] - o[0], ez[1] - o[1]];
const det = ax[0] * az[1] - ax[1] * az[0];
function toWorld(event) {
  const m = svg.getScreenCTM();
  if (!m) return null;
  const p = new DOMPoint(event.clientX, event.clientY).matrixTransform(m.inverse());
  const dx = p.x - o[0], dy = p.y - o[1];
  return { x: (dx * az[1] - dy * az[0]) / det, z: (ax[0] * dy - ax[1] * dx) / det };
}
const take = () => { touring = false; idle = 0; run(); };
let press = null;
stage.addEventListener("pointermove", (event) => {
  if (event.pointerType === "touch" && !press) return;
  const w = toWorld(event);
  if (!w) return;
  if (press && Math.hypot(event.clientX - press.x, event.clientY - press.y) > 6) press.moved = true;
  st.aim.x = w.x; st.aim.z = w.z;
  take();
});
stage.addEventListener("pointerdown", (event) => { press = { x: event.clientX, y: event.clientY, moved: false }; stage.setPointerCapture(event.pointerId); take(); });
const release = (event) => {
  if (press && !press.moved) st.aim.c = st.aim.c > 0.5 ? 0 : 1;
  press = null;
  try { stage.releasePointerCapture(event.pointerId); } catch {}
  take();
};
stage.addEventListener("pointerup", release);
stage.addEventListener("pointercancel", () => { press = null; });
stage.addEventListener("keydown", (event) => {
  const step = event.shiftKey ? 18 : 6;
  let used = true;
  if (event.key === "ArrowRight") st.aim.x += step;
  else if (event.key === "ArrowLeft") st.aim.x -= step;
  else if (event.key === "ArrowUp") st.aim.z += step;
  else if (event.key === "ArrowDown") st.aim.z -= step;
  else if (event.key === " " || event.key === "Enter") st.aim.c = st.aim.c > 0.5 ? 0 : 1;
  else if (event.key === "r" || event.key === "R") st.reset = 2.2;
  else used = false;
  if (!used) return;
  event.preventDefault();
  st.aim.x = clampN(st.aim.x, 60, TX1);
  take();
  stage.setAttribute("aria-valuetext", st.aim.c > 0.5 ? "버킷 오므림" : "버킷 펼침");
});
draw();
`;

const body = k.plateHtml({
  fig: "Fig 1",
  title: "굴착기 · 도랑 파기",
  hint: "포인터로 버킷 이동 · 클릭으로 오므림/펼침",
  readout: "버킷 0.00 m³ · 깊이 1.3 m · 사토 0.0 m³",
  keys: [
    { mark: "lit", label: "버킷 날과 투스: 흙을 파고드는 부분" },
    { mark: "raised", label: "붐·암·버킷, 실린더 4개(붐 2, 암 1, 버킷 1)" },
    { mark: "flat", label: "앞쪽 흙을 잘라 낸 단면: 표토·점토·자갈층" },
  ],
  caption:
    "20 t급 굴착기(붐 5.7 m, 암 2.9 m, 버킷 1.0 m³)가 폭 1.3 m 도랑을 판다. 버킷 날이 흙 속으로 들어간 채 암을 당기며 버킷을 오므리면, 날이 지나간 만큼 바닥이 깎이고 그 부피가 그대로 버킷에 담긴다(가득 차면 더 파고들지 못한다). 버킷을 펼치면 흙이 중력으로 떨어져 궤도 앞 사토 더미(안식각 34°)로 쌓인다. 선회는 생략했다. 방향키로 이동, 스페이스로 오므림/펼침, R로 초기화.",
  body: `<div class="iso-stage" tabindex="0" role="slider" aria-label="굴착기: 포인터나 방향키로 버킷 위치를 옮기고, 클릭이나 스페이스로 버킷을 오므리거나 펼친다" aria-valuemin="0" aria-valuemax="1" aria-valuenow="0" aria-valuetext="버킷 펼침" style="touch-action:pan-y;cursor:crosshair">${svg}</div>`,
});

const out = theme === "light" ? "excavator-light.html" : "excavator.html";
writeFileSync(join(HERE, out), k.pageHtml({ title: "굴착기 · 도랑 파기", theme, body, script: kitSrc + "\n" + lathe + "\n" + sceneSrc + "\n" + live, width: 720 }));
console.log("wrote", out);
