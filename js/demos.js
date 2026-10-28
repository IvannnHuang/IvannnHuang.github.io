/* =========================================================
   Live canvas demos — one per project.
   Each factory gets an env {ctx, w, h} (w/h in CSS px, kept current
   by the runner) and returns { draw(dt, t), click?(x, y), move?(x, y), reset?() }.
   ========================================================= */

const DEMOS = {};

const PAL = {
  bg: "#070a12", panel: "#0d1220", line: "rgba(255,255,255,0.08)", grid: "rgba(255,255,255,0.045)",
  text: "#e7eaf2", muted: "#8d96ab", dim: "#5d667b",
  silicon: "#a78bfa", fpga: "#f472b6", emb: "#4ade80", rob: "#fbbf24", perf: "#38bdf8",
  brand: "#78f0d2", red: "#f87171", blue: "#60a5fa"
};
const THREAD_COLORS = ["#38bdf8", "#f472b6", "#fbbf24", "#4ade80", "#a78bfa", "#f87171"];

const rand = (a = 1, b) => (b === undefined ? Math.random() * a : a + Math.random() * (b - a));
const randi = n => Math.floor(Math.random() * n);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const lerp = (a, b, t) => a + (b - a) * t;
const ease = t => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const gauss = () => { let u = 0, v = 0; while (!u) u = Math.random(); while (!v) v = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };

function font(ctx, size, weight = 500, mono = true) {
  ctx.font = `${weight} ${size}px ${mono ? '"JetBrains Mono", ui-monospace, monospace' : 'Inter, system-ui, sans-serif'}`;
}
function txt(ctx, s, x, y, { size = 11, color = PAL.muted, align = "left", base = "alphabetic", weight = 500, mono = true } = {}) {
  font(ctx, size, weight, mono); ctx.fillStyle = color; ctx.textAlign = align; ctx.textBaseline = base; ctx.fillText(s, x, y);
}
function rr(ctx, x, y, w, h, r) {
  r = Math.min(r, w / 2, h / 2);
  ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}
function clear(env, dots = true) {
  const { ctx, w, h } = env;
  ctx.fillStyle = PAL.bg; ctx.fillRect(0, 0, w, h);
  if (dots) {
    ctx.fillStyle = PAL.grid;
    for (let x = 12; x < w; x += 22) for (let y = 12; y < h; y += 22) ctx.fillRect(x, y, 1.2, 1.2);
  }
}
function alpha(hex, a) {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${n >> 16},${(n >> 8) & 255},${n & 255},${a})`;
}
function glowLine(ctx, color, width, blur) { ctx.strokeStyle = color; ctx.lineWidth = width; ctx.shadowColor = color; ctx.shadowBlur = blur; }
function noGlow(ctx) { ctx.shadowBlur = 0; }

/* ---------------------------------------------------------
   1. TCAM — parallel ternary search + priority encoder
   --------------------------------------------------------- */
DEMOS.tcam = env => {
  const R = 8, C = 8, T = 2.8;
  const sym = () => { const r = Math.random(); return r < 0.24 ? "X" : r < 0.62 ? "0" : "1"; };
  let words = Array.from({ length: R }, () => Array.from({ length: C }, sym));
  let key, cellOk, rowOk, winner, t = 0, wrote = -1;
  function newSearch() {
    if (Math.random() < 0.4) { wrote = randi(R); words[wrote] = Array.from({ length: C }, sym); } else wrote = -1;
    const pick = Math.random() < 0.85 ? randi(R) : -1;
    key = Array.from({ length: C }, (_, j) => pick >= 0 && words[pick][j] !== "X" ? words[pick][j] : (Math.random() < 0.5 ? "0" : "1"));
    cellOk = words.map(w => w.map((s, j) => s === "X" || s === key[j]));
    rowOk = cellOk.map(r => r.every(Boolean));
    winner = rowOk.lastIndexOf(true);
  }
  newSearch();
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt; if (t > T) { t = 0; newSearch(); }
      clear(env);
      const s = Math.min((w * 0.56) / C, (h * 0.76) / (R + 1.6));
      const x0 = Math.max(46, w * 0.12), y0 = h * 0.12 + s * 0.2;
      const gy = y0 + s * 1.45;
      const ph = t < 0.5 ? 0 : t < 1.1 ? 1 : t < 1.6 ? 2 : 3;
      const names = ["PRECHARGE", "BROADCAST KEY", "EVALUATE", "PRIORITY ENCODE"];
      txt(ctx, names[ph], w - 14, 22, { align: "right", color: PAL.silicon, size: 11 });
      if (wrote >= 0 && t < 0.5) txt(ctx, `write → row ${wrote}`, w - 14, 38, { align: "right", color: PAL.dim, size: 10 });

      // search key + search lines
      txt(ctx, "key", x0 - 10, y0 + s * 0.62, { align: "right", color: PAL.muted, size: 10 });
      const drop = ph === 0 ? 0 : ph === 1 ? ease((t - 0.5) / 0.6) : 1;
      for (let j = 0; j < C; j++) {
        const x = x0 + j * s;
        const col = key[j] === "1" ? PAL.silicon : PAL.blue;
        rr(ctx, x + 3, y0, s - 6, s - 6, 5); ctx.fillStyle = alpha(col, 0.18); ctx.fill();
        ctx.strokeStyle = alpha(col, 0.7); ctx.lineWidth = 1; ctx.stroke();
        txt(ctx, key[j], x + s / 2, y0 + (s - 6) / 2 + 1, { align: "center", base: "middle", color: PAL.text, size: Math.min(13, s * 0.4), weight: 600 });
        if (drop > 0) {
          ctx.strokeStyle = alpha(col, 0.35 * (ph < 3 ? 1 : 0.5)); ctx.lineWidth = 2;
          ctx.beginPath(); ctx.moveTo(x + s / 2, y0 + s - 6); ctx.lineTo(x + s / 2, lerp(y0 + s, gy + R * s, drop)); ctx.stroke();
        }
      }
      // cells + matchlines
      const mlx0 = x0 + C * s, encX = mlx0 + Math.max(28, w * 0.06), encW = Math.max(50, w * 0.1);
      for (let i = 0; i < R; i++) {
        const y = gy + i * s;
        txt(ctx, `r${i}`, x0 - 10, y + s / 2, { align: "right", base: "middle", color: PAL.dim, size: 10 });
        for (let j = 0; j < C; j++) {
          const x = x0 + j * s, bad = ph >= 2 && !cellOk[i][j];
          rr(ctx, x + 3, y + 3, s - 6, s - 6, 4);
          ctx.fillStyle = bad ? alpha(PAL.red, 0.28) : words[i][j] === "X" ? "rgba(255,255,255,0.03)" : "rgba(255,255,255,0.06)"; ctx.fill();
          if (wrote === i && t < 0.5) { ctx.strokeStyle = alpha(PAL.brand, 0.8); ctx.lineWidth = 1; ctx.stroke(); }
          txt(ctx, words[i][j], x + s / 2, y + s / 2 + 1, { align: "center", base: "middle", color: words[i][j] === "X" ? PAL.dim : bad ? PAL.red : PAL.text, size: Math.min(12, s * 0.38) });
        }
        // matchline state
        let col, a = 1;
        if (ph <= 1) col = PAL.rob; // precharged high
        else if (rowOk[i]) col = PAL.emb;
        else { col = PAL.red; a = ph === 2 ? 1 - ease((t - 1.1) / 0.5) * 0.75 : 0.25; }
        ctx.save(); glowLine(ctx, alpha(col, a), i === winner && ph === 3 ? 3 : 2, ph <= 1 ? 8 : rowOk[i] ? 10 : 0);
        ctx.beginPath(); ctx.moveTo(mlx0 + 2, y + s / 2); ctx.lineTo(encX, y + s / 2); ctx.stroke(); ctx.restore();
      }
      // priority encoder
      rr(ctx, encX, gy, encW, R * s, 8); ctx.fillStyle = PAL.panel; ctx.fill(); ctx.strokeStyle = alpha(PAL.silicon, 0.5); ctx.lineWidth = 1; ctx.stroke();
      txt(ctx, "PRIORITY", encX + encW / 2, gy + R * s / 2 - 7, { align: "center", color: PAL.silicon, size: 9 });
      txt(ctx, "ENCODER", encX + encW / 2, gy + R * s / 2 + 7, { align: "center", color: PAL.silicon, size: 9 });
      const outX = encX + encW + 10;
      if (ph === 3) {
        const p = ease(Math.min(1, (t - 1.6) / 0.4));
        if (winner >= 0) {
          const wy = gy + winner * s + s / 2;
          ctx.save(); glowLine(ctx, PAL.emb, 2, 12);
          ctx.beginPath(); ctx.moveTo(encX + encW, gy + R * s / 2); ctx.lineTo(lerp(encX + encW, outX + 10, p), gy + R * s / 2); ctx.stroke(); ctx.restore();
          ctx.strokeStyle = alpha(PAL.emb, 0.6); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(encX + 4, wy); ctx.lineTo(encX + encW - 4, gy + R * s / 2); ctx.stroke(); ctx.setLineDash([]);
          if (outX + 14 < w) {
            txt(ctx, `row ${winner}`, outX + 14, gy + R * s / 2 - 4, { color: PAL.emb, size: 12, weight: 600 });
            txt(ctx, winner.toString(2).padStart(3, "0") + "b", outX + 14, gy + R * s / 2 + 12, { color: PAL.muted, size: 10 });
          }
          const others = rowOk.filter(Boolean).length - 1;
          if (others > 0) txt(ctx, `${others} lower-priority hit${others > 1 ? "s" : ""} masked`, x0, gy + R * s + 18, { color: PAL.dim, size: 10 });
        } else txt(ctx, "no match", outX + 10, gy + R * s / 2 + 4, { color: PAL.red, size: 11 });
      }
    }
  };
};

/* ---------------------------------------------------------
   2. 5-stage RISC-V pipeline with forwarding / stalls / squash
   --------------------------------------------------------- */
DEMOS.pipeline = env => {
  const prog = [
    { op: "lw", a: "x1, 0(x2)", rd: 1, rs: [2], load: 1 },
    { op: "add", a: "x3, x1, x4", rd: 3, rs: [1, 4] },
    { op: "mul", a: "x5, x3, x3", rd: 5, rs: [3] },
    { op: "sw", a: "x5, 4(x2)", rd: 0, rs: [5, 2] },
    { op: "addi", a: "x2, x2, 8", rd: 2, rs: [2] },
    { op: "bne", a: "x2, x6, loop", rd: 0, rs: [2, 6], br: 1 },
    { op: "xor", a: "x7, x7, x1", rd: 7, rs: [7, 1] },
    { op: "or", a: "x8, x8, x3", rd: 8, rs: [8, 3] }
  ];
  const hues = [190, 260, 320, 40, 140, 10, 220, 290];
  let uid = 0, pc = 0, slots = [null, null, null, null, null], prev = new Map(), t = 0, cycles = 0, retired = 0, event = "", fwd = [];
  const TICK = 0.75;
  const fetch = () => { const i = pc % prog.length; pc++; return { uid: uid++, i, ...prog[i] }; };
  const bubble = (sq = false) => ({ uid: uid++, bubble: true, sq });
  const squash = it => (it && !it.bubble ? { ...it, uid: it.uid, sq: true } : bubble());
  function tick() {
    const [IF, ID, EX, MEM, WB] = slots;
    if (WB && !WB.bubble && !WB.sq) retired++;
    cycles++;
    const old = new Map(); slots.forEach((s, k) => s && old.set(s.uid, k));
    let next;
    if (EX && EX.br && !EX.bubble && !EX.sq) {
      pc = 0; next = [fetch(), squash(IF), squash(ID), EX, MEM]; event = "branch taken in EX → squash 2";
    } else if (EX && EX.load && !EX.bubble && !EX.sq && ID && !ID.bubble && !ID.sq && ID.rs.includes(EX.rd)) {
      next = [IF, ID, bubble(), EX, MEM]; event = "load-use hazard → stall + bubble";
    } else { next = [fetch(), IF, ID, EX, MEM]; event = ""; }
    prev = old; slots = next;
    fwd = [];
    const ex = slots[2];
    if (ex && !ex.bubble && !ex.sq) {
      [[3, "MEM"], [4, "WB"]].forEach(([k]) => {
        const src = slots[k];
        if (src && !src.bubble && !src.sq && src.rd && ex.rs.includes(src.rd) && !fwd.some(f => f.reg === src.rd)) fwd.push({ from: k, reg: src.rd });
      });
    }
  }
  for (let k = 0; k < 4; k++) tick();
  retired = 0; cycles = 0;
  // ring network packets
  const packets = Array.from({ length: 5 }, () => ({ p: rand(8), v: rand(0.8, 1.6) * (Math.random() < 0.5 ? 1 : -1), c: THREAD_COLORS[randi(4)] }));
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt; let p = t / TICK; if (p >= 1) { t = 0; p = 0; tick(); }
      clear(env);
      const pad = 14, gap = Math.max(6, w * 0.014);
      const bw = (w - pad * 2 - gap * 4) / 5, by = h * 0.15, bh = Math.min(h * 0.3, 96);
      const names = ["IF", "ID", "EX", "MEM", "WB"];
      const bx = k => pad + k * (bw + gap);
      names.forEach((n, k) => {
        rr(ctx, bx(k), by, bw, bh, 10); ctx.fillStyle = PAL.panel; ctx.fill();
        ctx.strokeStyle = k === 2 && event ? alpha(PAL.rob, 0.6) : PAL.line; ctx.lineWidth = 1; ctx.stroke();
        txt(ctx, n, bx(k) + bw / 2, by - 7, { align: "center", color: PAL.silicon, size: 11, weight: 600 });
        if (k < 4) { ctx.fillStyle = "rgba(167,139,250,0.35)"; ctx.fillRect(bx(k) + bw + gap / 2 - 1, by + 8, 2, bh - 16); }
      });
      const e = ease(Math.min(1, p * 2.4));
      slots.forEach((s, k) => {
        if (!s) return;
        const from = prev.has(s.uid) ? prev.get(s.uid) : k - 1;
        const x = lerp(bx(from), bx(k), e) + 5, y = by + 10, tw = bw - 10, th = bh - 20;
        ctx.globalAlpha = from < 0 ? e : 1;
        rr(ctx, x, y, tw, th, 7);
        if (s.bubble) {
          ctx.setLineDash([4, 4]); ctx.strokeStyle = "rgba(255,255,255,0.25)"; ctx.stroke(); ctx.setLineDash([]);
          txt(ctx, "bubble", x + tw / 2, y + th / 2 + 4, { align: "center", color: PAL.dim, size: 10 });
        } else {
          const col = `hsl(${hues[s.i]} 80% 65%)`;
          ctx.fillStyle = s.sq ? "rgba(255,255,255,0.03)" : `hsla(${hues[s.i]},80%,60%,0.16)`; ctx.fill();
          ctx.strokeStyle = s.sq ? "rgba(255,255,255,0.15)" : `hsla(${hues[s.i]},80%,65%,0.7)`; ctx.lineWidth = 1; ctx.stroke();
          const fs = Math.max(8.5, Math.min(12, tw / 7.5));
          txt(ctx, s.op, x + tw / 2, y + th / 2 - 3, { align: "center", color: s.sq ? PAL.dim : col, size: fs + 1, weight: 600 });
          txt(ctx, s.a, x + tw / 2, y + th / 2 + fs + 1, { align: "center", color: s.sq ? PAL.dim : PAL.muted, size: Math.max(7.5, fs - 2.5) });
          if (s.sq) { ctx.strokeStyle = alpha(PAL.red, 0.7); ctx.beginPath(); ctx.moveTo(x + 8, y + th / 2); ctx.lineTo(x + tw - 8, y + th / 2); ctx.stroke(); }
        }
        ctx.globalAlpha = 1;
      });
      // forwarding arcs
      if (e > 0.6) fwd.forEach(f => {
        const x1 = bx(f.from) + bw / 2, x2 = bx(2) + bw / 2, yb = by + bh + 4, depth = f.from === 3 ? 22 : 34;
        ctx.save(); glowLine(ctx, PAL.brand, 1.8, 8);
        ctx.beginPath(); ctx.moveTo(x1, yb); ctx.bezierCurveTo(x1, yb + depth, x2, yb + depth, x2, yb); ctx.stroke(); ctx.restore();
        ctx.fillStyle = PAL.brand; ctx.beginPath(); ctx.moveTo(x2, yb); ctx.lineTo(x2 - 4, yb + 7); ctx.lineTo(x2 + 4, yb + 7); ctx.fill();
        txt(ctx, `bypass x${f.reg}`, (x1 + x2) / 2, yb + depth - 2, { align: "center", color: PAL.brand, size: 9.5 });
      });
      const ly = by + bh + 58;
      if (event) txt(ctx, "⚠ " + event, pad, ly, { color: PAL.rob, size: 11 });
      txt(ctx, `cycle ${cycles}   retired ${retired}   CPI ${retired ? (cycles / retired).toFixed(2) : "—"}`, pad, ly + 20, { color: PAL.muted, size: 10.5 });
      // mini ring network: 4 cores + 4 banks
      const rcx = w - pad - Math.min(70, h * 0.2), rcy = h - Math.min(70, h * 0.2) - 6, rad = Math.min(52, h * 0.15);
      if (rcy - rad > ly - 10) {
        ctx.strokeStyle = alpha(PAL.silicon, 0.35); ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(rcx, rcy, rad, 0, Math.PI * 2); ctx.stroke();
        for (let k = 0; k < 8; k++) {
          const a = (k / 8) * Math.PI * 2 - Math.PI / 2, nx = rcx + Math.cos(a) * rad, ny = rcy + Math.sin(a) * rad;
          rr(ctx, nx - 11, ny - 8, 22, 16, 4); ctx.fillStyle = PAL.panel; ctx.fill(); ctx.strokeStyle = k % 2 ? alpha(PAL.perf, 0.6) : alpha(PAL.silicon, 0.7); ctx.stroke();
          txt(ctx, (k % 2 ? "B" : "C") + (k >> 1), nx, ny + 3.5, { align: "center", size: 8.5, color: PAL.text });
        }
        packets.forEach(pk => {
          pk.p = (pk.p + pk.v * dt + 8) % 8;
          const a = (pk.p / 8) * Math.PI * 2 - Math.PI / 2;
          ctx.save(); ctx.fillStyle = pk.c; ctx.shadowColor = pk.c; ctx.shadowBlur = 8;
          ctx.beginPath(); ctx.arc(rcx + Math.cos(a) * rad, rcy + Math.sin(a) * rad, 3, 0, 7); ctx.fill(); ctx.restore();
        });
        txt(ctx, "ring", rcx, rcy + 3, { align: "center", size: 9, color: PAL.dim });
        txt(ctx, "4 cores ↔ 4 cache banks", rcx - rad - 18, rcy + 3, { align: "right", size: 9.5, color: PAL.muted });
      }
    }
  };
};

/* ---------------------------------------------------------
   3. TinyFlow — simulated-annealing placement + maze routing
   --------------------------------------------------------- */
DEMOS.tinyflow = env => {
  const NR = 5, NS = 12, G = 3; // rows, sites per row, routing grid per site
  const steps = ["RTL sim", "Synthesis", "Gate-level sim", "Placement", "Routing", "DRC", "LVS", "GDS"];
  const cellNames = ["INV", "NAND2", "NOR2", "AOI21", "NAND2", "INV", "NOR2", "AOI21", "INV", "NAND2", "NOR2", "INV", "AOI21", "NAND2", "INV", "NOR2", "NAND2", "INV"];
  let cells, nets, slotOf, T, stage, stageT, hist, routeQ, routed, bfs, done, failed;
  const colors = ["#a78bfa", "#38bdf8", "#f472b6", "#4ade80", "#fbbf24", "#f87171", "#60a5fa", "#2dd4bf", "#e879f9", "#facc15", "#fb923c", "#34d399"];
  function init() {
    const N = cellNames.length;
    const slots = [...Array(NR * NS).keys()].sort(() => Math.random() - 0.5);
    cells = cellNames.map((n, i) => ({ n, slot: slots[i], dx: 0, dy: 0 }));
    cells.forEach(c => { const p = slotXY(c.slot); c.dx = p[0]; c.dy = p[1]; });
    slotOf = new Map(cells.map((c, i) => [c.slot, i]));
    // two-pin nets on distinct cells, so every pin belongs to exactly one net
    const ends = [...Array(N).keys()].sort(() => Math.random() - 0.5);
    nets = [];
    for (let k = 0; k + 1 < N; k += 2) nets.push({ a: ends[k], b: ends[k + 1], c: colors[(k / 2) % colors.length] });
    T = 5; stage = 0; stageT = 0; hist = []; routed = []; routeQ = null; bfs = null; done = 0; failed = 0;
  }
  const slotXY = s => [s % NS, Math.floor(s / NS)];
  const hpwl = () => nets.reduce((acc, n) => { const [ax, ay] = slotXY(cells[n.a].slot), [bx, by] = slotXY(cells[n.b].slot); return acc + Math.abs(ax - bx) + Math.abs(ay - by) * 1.6; }, 0);
  function anneal(moves) {
    let cost = hpwl();
    for (let m = 0; m < moves; m++) {
      const i = randi(cells.length), target = randi(NR * NS), j = slotOf.get(target);
      const si = cells[i].slot;
      if (j === i) continue;
      cells[i].slot = target; if (j !== undefined) cells[j].slot = si;
      const nc = hpwl(), d = nc - cost;
      if (d <= 0 || Math.random() < Math.exp(-d / T)) {
        cost = nc; slotOf.delete(si); slotOf.set(target, i); if (j !== undefined) slotOf.set(si, j);
      } else { cells[i].slot = si; if (j !== undefined) cells[j].slot = target; }
    }
    return cost;
  }
  // routing grid
  const GW = NS * G, GH = NR * G;
  const pinNode = ci => { const [x, y] = slotXY(cells[ci].slot); return [x * G + 1, y * G + 1]; };
  let occ;
  function startRouting() {
    occ = new Int16Array(GW * GH).fill(-1);
    nets.forEach((n, k) => { [n.a, n.b].forEach(ci => { const [x, y] = pinNode(ci); occ[y * GW + x] = -2 - k; }); });
    routeQ = nets.map((_, k) => k).sort((p, q) => { const d = k => { const [ax, ay] = pinNode(nets[k].a), [bx, by] = pinNode(nets[k].b); return Math.abs(ax - bx) + Math.abs(ay - by); }; return d(p) - d(q); });
    nextNet();
  }
  function nextNet() {
    if (!routeQ.length) { bfs = null; return; }
    const k = routeQ.shift(), [sx, sy] = pinNode(nets[k].a), [tx, ty] = pinNode(nets[k].b);
    const dist = new Int32Array(GW * GH).fill(-1), from = new Int32Array(GW * GH).fill(-1);
    dist[sy * GW + sx] = 0;
    bfs = { k, front: [sy * GW + sx], dist, from, target: ty * GW + tx, level: 0, visited: [sy * GW + sx] };
  }
  function bfsStep() {
    const b = bfs, nf = [];
    for (const id of b.front) {
      const x = id % GW, y = (id / GW) | 0;
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nx = x + dx, ny = y + dy; if (nx < 0 || ny < 0 || nx >= GW || ny >= GH) continue;
        const nid = ny * GW + nx; if (b.dist[nid] >= 0) continue;
        const o = occ[nid]; if (o !== -1 && o !== -2 - b.k) continue;
        b.dist[nid] = b.level + 1; b.from[nid] = id; nf.push(nid); b.visited.push(nid);
        if (nid === b.target) { finishNet(); return; }
      }
    }
    b.level++; b.front = nf;
    if (!nf.length) { failed++; nextNet(); } // rip-up would retry; demo just skips
  }
  function finishNet() {
    const b = bfs, path = []; let id = b.target;
    while (id !== -1) { path.push(id); occ[id] = b.k; id = b.from[id]; }
    routed.push({ k: b.k, path }); done++; nextNet();
  }
  init();
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      stageT += dt;
      if (stage < 3 && stageT > 0.35) { stage++; stageT = 0; }
      else if (stage === 3) {
        const c = anneal(14); T *= Math.pow(0.4, dt); hist.push(c); if (hist.length > 140) hist.shift();
        if (T < 0.04) { stage = 4; stageT = 0; startRouting(); }
      } else if (stage === 4) {
        if (bfs) { for (let i = 0; i < 2; i++) if (bfs) bfsStep(); }
        else if (stageT > 0.4) { stage = 5; stageT = 0; }
      } else if (stage >= 5 && stage < 7 && stageT > 0.7) { stage++; stageT = 0; }
      else if (stage === 7 && stageT > 2.2) init();

      clear(env, false);
      const panelW = Math.max(118, w * 0.26), pad = 14;
      const dieW = w - panelW - pad * 3, dieH = h - pad * 2 - 18;
      const sw = Math.min(dieW / NS, dieH / NR / 1.35), rowH = sw * 1.35;
      const dx = pad + (dieW - sw * NS) / 2, dy = pad + 16 + (dieH - rowH * NR) / 2;
      txt(ctx, "die", dx, dy - 6, { color: PAL.dim, size: 10 });
      ctx.strokeStyle = alpha(PAL.silicon, 0.5); ctx.lineWidth = 1; ctx.strokeRect(dx - 4, dy - 4, sw * NS + 8, rowH * NR + 8);
      for (let r = 0; r < NR; r++) { ctx.fillStyle = r % 2 ? "rgba(167,139,250,0.04)" : "rgba(167,139,250,0.08)"; ctx.fillRect(dx, dy + r * rowH, sw * NS, rowH); }
      // power rails
      ctx.fillStyle = "rgba(96,165,250,0.18)"; for (let r = 0; r <= NR; r++) ctx.fillRect(dx, dy + r * rowH - 1, sw * NS, 2);
      const gx = x => dx + (x + 0.5) * (sw / G), gy = y => dy + (y + 0.5) * (rowH / G);
      // cells
      cells.forEach(c => {
        const [tx, ty] = slotXY(c.slot); c.dx = lerp(c.dx, tx, Math.min(1, dt * 10)); c.dy = lerp(c.dy, ty, Math.min(1, dt * 10));
        const x = dx + c.dx * sw + 1.5, y = dy + c.dy * rowH + 3;
        rr(ctx, x, y, sw - 3, rowH - 6, 3); ctx.fillStyle = "rgba(167,139,250,0.22)"; ctx.fill(); ctx.strokeStyle = alpha(PAL.silicon, 0.55); ctx.stroke();
        if (sw > 26) txt(ctx, c.n, x + (sw - 3) / 2, y + rowH - 10, { align: "center", size: Math.min(8.5, sw / 4.8), color: PAL.muted });
      });
      // ratsnest during placement
      if (stage <= 3) nets.forEach(n => {
        const a = cells[n.a], b = cells[n.b];
        ctx.strokeStyle = alpha(n.c, 0.55); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(dx + (a.dx + 0.5) * sw, dy + (a.dy + 0.4) * rowH); ctx.lineTo(dx + (b.dx + 0.5) * sw, dy + (b.dy + 0.4) * rowH); ctx.stroke();
      });
      // routing wavefront + wires
      if (stage >= 4) {
        if (bfs) {
          ctx.fillStyle = alpha(nets[bfs.k].c, 0.25);
          bfs.visited.forEach(id => ctx.fillRect(gx(id % GW) - 1.5, gy((id / GW) | 0) - 1.5, 3, 3));
        }
        routed.forEach(r => {
          ctx.save(); glowLine(ctx, nets[r.k].c, 2, 6); ctx.lineJoin = "round"; ctx.beginPath();
          r.path.forEach((id, i) => { const x = gx(id % GW), y = gy((id / GW) | 0); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); ctx.restore();
          [r.path[0], r.path[r.path.length - 1]].forEach(id => { ctx.fillStyle = "#fff"; ctx.fillRect(gx(id % GW) - 2, gy((id / GW) | 0) - 2, 4, 4); });
        });
      }
      if (stage === 5 || stage === 6) {
        const sx = dx + (stageT / 0.7) * sw * NS;
        ctx.fillStyle = alpha(PAL.emb, 0.12); ctx.fillRect(dx, dy, sx - dx, rowH * NR);
        ctx.fillStyle = PAL.emb; ctx.fillRect(sx, dy - 4, 2, rowH * NR + 8);
      }
      // side panel: steps + HPWL sparkline
      const px = w - panelW - pad;
      steps.forEach((s, i) => {
        const y = pad + 12 + i * Math.min(20, (h * 0.62) / steps.length);
        const st = i < stage || stage === 7 ? "done" : i === stage ? "run" : "todo";
        txt(ctx, st === "done" ? "✓" : st === "run" ? "▸" : "·", px, y, { color: st === "todo" ? PAL.dim : st === "run" ? PAL.rob : PAL.emb, size: 11 });
        txt(ctx, `0${i + 1} ${s}`, px + 14, y, { color: st === "todo" ? PAL.dim : PAL.text, size: 10 });
      });
      const sy = h * 0.72;
      txt(ctx, stage <= 3 ? `HPWL ${hist.length ? hist[hist.length - 1].toFixed(0) : "—"}  T ${T.toFixed(2)}` : `nets routed ${done}/${nets.length}`, px, sy - 6, { color: PAL.silicon, size: 10 });
      if (hist.length > 1) {
        const mx = Math.max(...hist), mn = Math.min(...hist), hh = h - sy - pad - 4;
        ctx.strokeStyle = PAL.silicon; ctx.lineWidth = 1.5; ctx.beginPath();
        hist.forEach((v, i) => { const x = px + (i / 139) * (panelW - 4), y = sy + 4 + (1 - (v - mn) / (mx - mn + 1e-6)) * hh; i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
        ctx.stroke();
      }
    }
  };
};

/* ---------------------------------------------------------
   4. Banded Smith–Waterman wavefront
   --------------------------------------------------------- */
DEMOS.smithwaterman = env => {
  const N = 13, K = 3, MATCH = 2, MIS = -1, GAP = -2;
  let A, B, H, dir, best, diag, t, path, phase, pathT;
  const bases = "ACGT";
  function init() {
    A = Array.from({ length: N }, () => bases[randi(4)]).join("");
    let b = A.split("").map(c => (Math.random() < 0.2 ? bases[randi(4)] : c));
    const p = 3 + randi(N - 6); if (Math.random() < 0.5) b.splice(p, 1); else b.splice(p, 0, bases[randi(4)]);
    B = b.slice(0, N).join("").padEnd(N, "A");
    H = Array.from({ length: N + 1 }, () => new Array(N + 1).fill(0));
    dir = Array.from({ length: N + 1 }, () => new Array(N + 1).fill(0));
    best = [0, 0, 0];
    for (let i = 1; i <= N; i++) for (let j = 1; j <= N; j++) {
      if (Math.abs(i - j) > K) { H[i][j] = null; continue; }
      const up = H[i - 1][j], lf = H[i][j - 1], dg = H[i - 1][j - 1];
      const d = (dg ?? 0) + (A[i - 1] === B[j - 1] ? MATCH : MIS), u = up === null ? -99 : up + GAP, l = lf === null ? -99 : lf + GAP;
      const v = Math.max(0, d, u, l); H[i][j] = v; dir[i][j] = v === 0 ? 0 : v === d ? 1 : v === u ? 2 : 3;
      if (v > best[0]) best = [v, i, j];
    }
    path = []; let i = best[1], j = best[2];
    while (i > 0 && j > 0 && H[i][j] > 0) { path.push([i, j]); const d = dir[i][j]; if (d === 1) { i--; j--; } else if (d === 2) i--; else j--; }
    diag = 2; t = 0; phase = 0; pathT = 0;
  }
  init();
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt;
      if (phase === 0 && t > 0.13) { t = 0; diag++; if (diag > 2 * N) phase = 1; }
      if (phase === 1) { pathT += dt; if (pathT > path.length * 0.08 + 2.6) init(); }
      clear(env);
      const S = Math.min(h - 40, w * 0.6), cs = S / (N + 1), x0 = 16, y0 = (h - S) / 2 + 6;
      // headers
      for (let j = 1; j <= N; j++) txt(ctx, B[j - 1], x0 + j * cs + cs / 2, y0 + cs / 2 + 4, { align: "center", color: PAL.perf, size: Math.min(12, cs * 0.5), weight: 600 });
      for (let i = 1; i <= N; i++) txt(ctx, A[i - 1], x0 + cs / 2, y0 + i * cs + cs / 2 + 4, { align: "center", color: PAL.fpga, size: Math.min(12, cs * 0.5), weight: 600 });
      const onPath = new Set(path.slice(0, phase ? Math.floor(pathT / 0.08) + 1 : 0).map(([i, j]) => i * 100 + j));
      for (let i = 1; i <= N; i++) for (let j = 1; j <= N; j++) {
        const x = x0 + j * cs, y = y0 + i * cs;
        if (H[i][j] === null) { ctx.fillStyle = "rgba(255,255,255,0.02)"; ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2); continue; }
        const computed = i + j < diag || phase;
        const front = !phase && i + j === diag - 1;
        if (computed) {
          const v = H[i][j] / Math.max(1, best[0]);
          ctx.fillStyle = `rgba(244,114,182,${0.08 + v * 0.6})`; ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2);
          if (cs > 17) txt(ctx, String(H[i][j]), x + cs / 2, y + cs / 2 + 3.5, { align: "center", size: Math.min(10, cs * 0.42), color: v > 0.5 ? "#fff" : PAL.muted });
        } else { ctx.strokeStyle = "rgba(255,255,255,0.06)"; ctx.strokeRect(x + 1.5, y + 1.5, cs - 3, cs - 3); }
        if (front) { ctx.save(); glowLine(ctx, PAL.brand, 1.5, 10); ctx.strokeRect(x + 1.5, y + 1.5, cs - 3, cs - 3); ctx.restore(); }
        if (onPath.has(i * 100 + j)) { ctx.save(); glowLine(ctx, PAL.rob, 2, 10); ctx.strokeRect(x + 2, y + 2, cs - 4, cs - 4); ctx.restore(); }
      }
      // side panel
      const px = x0 + S + 22, pw = w - px - 12;
      if (pw > 90) {
        txt(ctx, "BANDED SW", px, y0 + 12, { color: PAL.fpga, size: 11, weight: 600 });
        txt(ctx, `band |i−j| ≤ ${K}`, px, y0 + 32, { size: 10 });
        txt(ctx, "PE lanes sweep one", px, y0 + 50, { size: 10 });
        txt(ctx, "anti-diagonal / cycle", px, y0 + 64, { size: 10 });
        const cyc = Math.min(diag - 2, 2 * N - 1);
        txt(ctx, `cycle ${cyc}`, px, y0 + 88, { color: PAL.text, size: 11 });
        txt(ctx, `best score ${phase ? best[0] : "…"}`, px, y0 + 106, { color: PAL.text, size: 11 });
        if (phase && pathT > path.length * 0.08) {
          // build alignment strings
          let a = "", b = "", m = "";
          const pp = [...path].reverse();
          pp.forEach(([i, j], k) => {
            const pr = pp[k - 1];
            if (pr && i - pr[0] === 1 && j - pr[1] === 0) { a += A[i - 1]; b += "-"; m += " "; }
            else if (pr && i - pr[0] === 0 && j - pr[1] === 1) { a += "-"; b += B[j - 1]; m += " "; }
            else { a += A[i - 1]; b += B[j - 1]; m += A[i - 1] === B[j - 1] ? "|" : "·"; }
          });
          const fs = Math.min(12, (pw / Math.max(8, a.length)) * 1.55);
          txt(ctx, "traceback", px, y0 + 136, { color: PAL.rob, size: 10 });
          txt(ctx, a, px, y0 + 156, { color: PAL.fpga, size: fs, weight: 600 });
          txt(ctx, m, px, y0 + 156 + fs + 2, { color: PAL.muted, size: fs });
          txt(ctx, b, px, y0 + 156 + 2 * (fs + 2), { color: PAL.perf, size: fs, weight: 600 });
        }
      }
    }
  };
};

/* ---------------------------------------------------------
   5. Lorenz attractor — two trajectories, butterfly effect
   --------------------------------------------------------- */
DEMOS.lorenz = env => {
  const sig = 10, rho = 28, beta = 8 / 3, MAX = 1300;
  let a, b, ta, tb, ang, life;
  const step = (p, dt) => { const [x, y, z] = p; p[0] += sig * (y - x) * dt; p[1] += (x * (rho - z) - y) * dt; p[2] += (x * y - beta * z) * dt; };
  function init(eps = 0.001) {
    a = [1, 1, 1]; b = [1 + eps, 1, 1]; ta = []; tb = []; ang = 0.6; life = 0;
    // pre-integrate so the attractor is already visible on first paint
    for (let i = 0; i < 700; i++) { for (let k = 0; k < 6; k++) { step(a, 0.0055); step(b, 0.0055); } ta.push([...a]); tb.push([...b]); }
  }
  init();
  function proj(p, w, h) {
    const sc = Math.min(w, h) / 58, cx = w * 0.56, cy = h * 0.56;
    const x = p[0] * Math.cos(ang) - p[1] * Math.sin(ang), d = p[0] * Math.sin(ang) + p[1] * Math.cos(ang);
    return [cx + x * sc, cy - (p[2] - 25) * sc + d * sc * 0.18];
  }
  function trail(ctx, tr, col, w, h) {
    ctx.lineWidth = 1.3; ctx.lineCap = "round";
    // batch the trail into a few alpha bands: one stroke per band keeps this cheap
    const BANDS = 16, n = tr.length, pts = tr.map(p => proj(p, w, h));
    for (let k = 0; k < BANDS; k++) {
      const i0 = Math.max(1, Math.floor((k * n) / BANDS)), i1 = Math.floor(((k + 1) * n) / BANDS);
      ctx.strokeStyle = alpha(col, ((k + 1) / BANDS) * 0.6);
      ctx.beginPath(); ctx.moveTo(pts[i0 - 1][0], pts[i0 - 1][1]);
      for (let i = i0; i <= Math.min(i1, n - 1); i++) ctx.lineTo(pts[i][0], pts[i][1]);
      ctx.stroke();
    }
    const hd = proj(tr[tr.length - 1] || [0, 0, 25], w, h);
    ctx.save(); ctx.fillStyle = "#fff"; ctx.shadowColor = col; ctx.shadowBlur = 14; ctx.beginPath(); ctx.arc(hd[0], hd[1], 3, 0, 7); ctx.fill(); ctx.restore();
  }
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      life += dt; if (life > 28) init();
      ang = 0.15 + Math.sin(life * 0.18) * 0.6; // sway around the classic butterfly view
      for (let k = 0; k < 6; k++) { step(a, 0.0055); step(b, 0.0055); }
      ta.push([...a]); tb.push([...b]); if (ta.length > MAX) { ta.shift(); tb.shift(); }
      clear(env, false);
      trail(ctx, ta, PAL.perf, w, h); trail(ctx, tb, PAL.fpga, w, h);
      const d = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
      txt(ctx, "σ = 10   ρ = 28   β = 8/3", 14, 22, { size: 10.5, color: PAL.muted });
      txt(ctx, "27-bit fixed-point Euler · AXI → HDMI", 14, 38, { size: 10, color: PAL.dim });
      txt(ctx, `‖Δ‖ = ${d < 0.01 ? d.toExponential(1) : d.toFixed(2)}`, 14, 58, { size: 11, color: d > 5 ? PAL.fpga : PAL.text, weight: 600 });
      txt(ctx, "● x₀", w - 84, 22, { size: 10, color: PAL.perf }); txt(ctx, "● x₀ + δ", w - 84, 38, { size: 10, color: PAL.fpga });
    },
    click() { init(Math.pow(10, -rand(2, 5))); }
  };
};

/* ---------------------------------------------------------
   6. Mandelbrot — progressive render by 16 parallel lanes
   --------------------------------------------------------- */
const MSTOPS = [[0, 14, 18, 48], [0.2, 92, 60, 200], [0.42, 244, 114, 182], [0.62, 251, 191, 36], [0.8, 255, 246, 220], [1, 14, 18, 48]];
DEMOS.mandelbrot = env => {
  const LX = 4, LY = 4;
  const targets = [[-0.743643887, 0.131825904], [-0.1011, 0.9563], [-1.25066, 0.02012], [-0.7453, 0.1127], [0.2549, 0.0005]];
  let off = document.createElement("canvas"), oc = off.getContext("2d"), tmp = document.createElement("canvas");
  let bw = 0, bh = 0, img, cx, cy, sc, lanes, hold, zooms, ti = 0, maxIt;
  function setup() {
    bw = Math.max(40, Math.ceil(env.w / 2)); bh = Math.max(30, Math.ceil(env.h / 2));
    off.width = tmp.width = bw; off.height = tmp.height = bh;
    oc.fillStyle = "#000"; oc.fillRect(0, 0, bw, bh); img = oc.getImageData(0, 0, bw, bh);
    cx = -0.65; cy = 0; sc = baseSc(); zooms = 0; startRender();
  }
  const baseSc = () => Math.max(3.1 / bw, 2.6 / bh); // whole set fits at any aspect ratio
  function startRender() {
    maxIt = Math.floor(90 + 45 * Math.log2(1 + zooms * 2));
    lanes = [];
    for (let ly = 0; ly < LY; ly++) for (let lx = 0; lx < LX; lx++) {
      const x0 = Math.floor((lx * bw) / LX), x1 = Math.floor(((lx + 1) * bw) / LX), y0 = Math.floor((ly * bh) / LY), y1 = Math.floor(((ly + 1) * bh) / LY);
      lanes.push({ x0, x1, y0, y1, x: x0, y: y0, done: false });
    }
    hold = 0;
  }
  function zoomTo(nx, ny, f) {
    // reproject the current image so the zoom feels continuous
    oc.putImageData(img, 0, 0);
    const tc = tmp.getContext("2d"); tc.clearRect(0, 0, bw, bh); tc.drawImage(off, 0, 0);
    const nsc = sc * f, sx = (nx - nsc * bw / 2 - (cx - sc * bw / 2)) / sc, sy = (ny - nsc * bh / 2 - (cy - sc * bh / 2)) / sc;
    oc.imageSmoothingEnabled = true; oc.drawImage(tmp, sx, sy, bw * f, bh * f, 0, 0, bw, bh);
    img = oc.getImageData(0, 0, bw, bh);
    cx = nx; cy = ny; sc = nsc; zooms++; startRender();
  }
  function color(it, zr, zi) {
    if (it >= maxIt) return [0, 0, 0];
    const nu = it + 1 - Math.log2(Math.log2(zr * zr + zi * zi) / 2 + 1e-9);
    const k = 1 - Math.exp(-nu / 9); // far-outside points fade to the background
    // cyclic gradient: navy → violet → pink → amber → white → navy
    let t = (nu / 34) % 1; t = t < 0 ? t + 1 : t;
    let i = 0; while (i < MSTOPS.length - 2 && MSTOPS[i + 1][0] < t) i++;
    const [t0, ...c0] = MSTOPS[i], [t1, ...c1] = MSTOPS[i + 1], f = (t - t0) / (t1 - t0);
    return [k * lerp(c0[0], c1[0], f), k * lerp(c0[1], c1[1], f), k * lerp(c0[2], c1[2], f)];
  }
  function iterate(x, y) {
    const cr = cx + (x - bw / 2) * sc, ci = cy + (y - bh / 2) * sc;
    // cardioid / period-2 bulb shortcut
    const q = (cr - 0.25) ** 2 + ci * ci;
    if (q * (q + (cr - 0.25)) < 0.25 * ci * ci || (cr + 1) ** 2 + ci * ci < 0.0625) return [0, 0, 0];
    let zr = 0, zi = 0, zr2 = 0, zi2 = 0, it = 0;
    while (it < maxIt && zr2 + zi2 <= 16) { zi = 2 * zr * zi + ci; zr = zr2 - zi2 + cr; zr2 = zr * zr; zi2 = zi * zi; it++; }
    return color(it, zr, zi);
  }
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      if (Math.ceil(w / 2) !== bw || Math.ceil(h / 2) !== bh) setup();
      const t0 = performance.now(); let allDone = true;
      while (performance.now() - t0 < 7) {
        let progressed = false;
        for (const L of lanes) {
          if (L.done) continue;
          for (let k = 0; k < 24 && !L.done; k++) {
            const c = iterate(L.x, L.y), o = (L.y * bw + L.x) * 4;
            img.data[o] = c[0]; img.data[o + 1] = c[1]; img.data[o + 2] = c[2]; img.data[o + 3] = 255;
            if (++L.x >= L.x1) { L.x = L.x0; if (++L.y >= L.y1) L.done = true; }
          }
          progressed = true;
        }
        if (!progressed) break;
      }
      lanes.forEach(L => { if (!L.done) allDone = false; });
      oc.putImageData(img, 0, 0);
      ctx.imageSmoothingEnabled = true; ctx.drawImage(off, 0, 0, w, h);
      // lane overlay
      const sx = w / bw, sy = h / bh;
      lanes.forEach((L, i) => {
        ctx.strokeStyle = "rgba(255,255,255,0.08)"; ctx.lineWidth = 1; ctx.strokeRect(L.x0 * sx, L.y0 * sy, (L.x1 - L.x0) * sx, (L.y1 - L.y0) * sy);
        if (!L.done) { ctx.fillStyle = alpha(PAL.fpga, 0.85); ctx.fillRect(L.x0 * sx, L.y * sy, (L.x - L.x0) * sx, 2); }
      });
      if (allDone) { hold += dt; if (hold > 0.9) { if (zooms >= 7) { ti = (ti + 1) % targets.length; cx = -0.65; cy = 0; sc = baseSc(); zooms = 0; startRender(); } else { const [tx, ty] = targets[ti]; zoomTo(lerp(cx, tx, 0.75), lerp(cy, ty, 0.75), 0.36); } } }
      rr(ctx, 10, h - 34, 196, 24, 6); ctx.fillStyle = "rgba(6,8,14,0.7)"; ctx.fill();
      txt(ctx, `zoom ${(baseSc() / sc).toFixed(1)}×  ·  16 lanes  ·  it ${maxIt}`, 18, h - 18, { size: 10, color: PAL.text });
    },
    click(x, y) {
      const nx = cx + (x / env.w * bw - bw / 2) * sc, ny = cy + (y / env.h * bh - bh / 2) * sc;
      zoomTo(nx, ny, 0.4);
    }
  };
};

/* ---------------------------------------------------------
   7. SmolVLA action expert — flow matching on an action chunk
   --------------------------------------------------------- */
DEMOS.vla = env => {
  const L = 50, layers = ["SA", "CA", "SA", "CA", "SA", "CA", "SA", "CA"];
  let target, noise, t, hold, barT = 0;
  function init() {
    const f1 = rand(0.8, 1.6), f2 = rand(1.4, 2.6), p1 = rand(6), p2 = rand(6), a1 = rand(0.55, 0.85), a2 = rand(0.4, 0.8);
    target = Array.from({ length: L }, (_, i) => { const s = i / (L - 1); return [Math.sin(f1 * 3.2 * s + p1) * a1, Math.sin(f2 * 3.2 * s + p2) * a2 * Math.cos(s * 1.4)]; });
    noise = Array.from({ length: L }, () => [gauss() * 0.55, gauss() * 0.55]);
    t = 0; hold = 0;
  }
  init();
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      if (t < 1) t = Math.min(1, t + dt / 3.2); else { hold += dt; if (hold > 1.6) init(); }
      barT = Math.min(1, barT + dt / 1.5);
      clear(env);
      // layer stack
      const sx = 14, sw = Math.max(66, w * 0.15), sy = 30, lh = (h - sy - 52) / layers.length;
      txt(ctx, "ACTION EXPERT", sx, 18, { color: PAL.fpga, size: 10, weight: 600 });
      const k = Math.min(9, Math.floor(t * 10)), pulse = (t * 10) % 1;
      layers.forEach((n, i) => {
        const y = sy + i * lh, active = t < 1 && Math.floor(pulse * layers.length) === i;
        rr(ctx, sx, y + 2, sw, lh - 5, 6);
        ctx.fillStyle = active ? alpha(n === "SA" ? PAL.perf : PAL.fpga, 0.35) : "rgba(255,255,255,0.04)"; ctx.fill();
        ctx.strokeStyle = alpha(n === "SA" ? PAL.perf : PAL.fpga, active ? 0.9 : 0.3); ctx.stroke();
        txt(ctx, `${n === "SA" ? "Self-Attn" : "Cross-Attn"} + MLP`, sx + sw / 2, y + lh / 2 + 2, { align: "center", size: Math.min(9.5, sw / 9.5), color: active ? PAL.text : PAL.muted });
      });
      txt(ctx, "RoPE · GQA · RMSNorm", sx, h - 34, { size: 9, color: PAL.dim });
      // trajectory plot
      const px = sx + sw + 24, pw = w - px - 14, ph = h - 92, py = 22;
      rr(ctx, px, py, pw, ph, 10); ctx.fillStyle = "rgba(255,255,255,0.02)"; ctx.fill(); ctx.strokeStyle = PAL.line; ctx.stroke();
      txt(ctx, `flow step ${k + 1}/10  ·  action chunk = ${L} steps`, px + 10, py + 16, { size: 10, color: PAL.text });
      const tt = ease(t), mx = px + pw / 2, my = py + ph / 2 + 8, s = Math.min(pw, ph) * 0.38;
      const pts = target.map((p, i) => [mx + (lerp(noise[i][0], p[0], tt)) * s * 1.25, my + (lerp(noise[i][1], p[1], tt)) * s]);
      ctx.save(); rr(ctx, px, py, pw, ph, 10); ctx.clip();
      ctx.lineWidth = 1.5;
      for (let i = 1; i < L; i++) { ctx.strokeStyle = `hsla(${200 + i * 2.4},80%,65%,${0.25 + tt * 0.6})`; ctx.beginPath(); ctx.moveTo(...pts[i - 1]); ctx.lineTo(...pts[i]); ctx.stroke(); }
      pts.forEach((p, i) => { ctx.fillStyle = `hsl(${200 + i * 2.4} 85% 68%)`; ctx.beginPath(); ctx.arc(p[0], p[1], 2.4, 0, 7); ctx.fill(); });
      ctx.restore();
      txt(ctx, t < 1 ? "noise → refined trajectory" : "✓ action chunk ready", px + pw - 10, py + ph - 10, { align: "right", size: 10, color: t < 1 ? PAL.dim : PAL.emb });
      // speedup bars
      const bars = [["baseline", 1], ["fused+pipelined", 2], ["int8", 5]], by = h - 54, bwid = (pw - 20) / 3;
      bars.forEach(([n, v], i) => {
        const x = px + i * (bwid + 10), len = (v / 5) * bwid * ease(barT);
        ctx.fillStyle = "rgba(255,255,255,0.05)"; ctx.fillRect(x, by + 14, bwid, 9);
        ctx.fillStyle = [PAL.dim, PAL.perf, PAL.fpga][i]; ctx.fillRect(x, by + 14, len, 9);
        txt(ctx, `${n} ${v}×`, x, by + 8, { size: 9.5, color: PAL.muted });
      });
      txt(ctx, "csynth latency speedup", px, by + 38, { size: 9, color: PAL.dim });
    }
  };
};

/* ---------------------------------------------------------
   8. Galton board — real ball/peg collisions + histogram
   --------------------------------------------------------- */
DEMOS.galton = env => {
  const ROWS = 11; let balls = [], counts = new Array(ROWS + 1).fill(0), total = 0, spawn = 0;
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      const d = Math.min((w * 0.72) / (ROWS + 1), (h * 0.62) / (ROWS * 0.86 + 1.5)), cx = w / 2, y0 = h * 0.1 + d * 0.6, vy0 = d * 0.86;
      const rb = d * 0.14, rp = d * 0.11, g = d * 16, lastY = y0 + (ROWS - 1) * vy0;
      spawn += dt * 30;
      while (spawn > 1) { spawn--; if (balls.length < 240) balls.push({ x: cx + rand(-0.4, 0.4), y: y0 - d * 1.1, vx: rand(-4, 4), vy: 0 }); }
      const sub = 3, h2 = Math.min(dt, 0.04) / sub;
      for (let s = 0; s < sub; s++) for (const b of balls) {
        b.vy += g * h2; b.vx *= 0.995; b.x += b.vx * h2; b.y += b.vy * h2;
        // board walls follow the triangle so balls can't skip the pegs
        const hw = (clamp((b.y - y0) / vy0, 0, ROWS - 1) / 2 + 0.55) * d;
        if (Math.abs(b.x - cx) > hw) { b.x = cx + Math.sign(b.x - cx) * hw; b.vx = -b.vx * 0.3; }
        const r = Math.round((b.y - y0) / vy0);
        if (r >= 0 && r < ROWS) {
          const k = clamp(Math.round((b.x - cx) / d + r / 2), 0, r), px = cx + (k - r / 2) * d, py = y0 + r * vy0;
          const dx = b.x - px, dy = b.y - py, dist = Math.hypot(dx, dy);
          if (dist < rb + rp && dist > 0) {
            const nx = dx / dist, ny = dy / dist, vn = b.vx * nx + b.vy * ny;
            if (vn < 0) { b.vx -= 1.35 * vn * nx; b.vy -= 1.35 * vn * ny; b.vx += rand(-1, 1) * d * 0.4; }
            b.x = px + nx * (rb + rp); b.y = py + ny * (rb + rp);
          }
        }
      }
      balls = balls.filter(b => {
        if (b.y > lastY + d * 0.7) { const bin = clamp(Math.floor((b.x - (cx - ((ROWS + 1) / 2) * d)) / d), 0, ROWS); counts[bin]++; total++; return false; }
        return true;
      });
      if (total > 1800) { counts.fill(0); total = 0; }
      clear(env, false);
      // pegs
      ctx.fillStyle = "#1a3a2a";
      for (let r = 0; r < ROWS; r++) for (let k = 0; k <= r; k++) {
        ctx.beginPath(); ctx.arc(cx + (k - r / 2) * d, y0 + r * vy0, rp, 0, 7); ctx.fillStyle = "#cfeedd"; ctx.fill();
      }
      // balls
      ctx.strokeStyle = PAL.emb; ctx.lineWidth = 1.2;
      for (const b of balls) { ctx.beginPath(); ctx.arc(b.x, b.y, rb, 0, 7); ctx.stroke(); }
      // histogram
      const base = h - 10, maxH = h - (lastY + d) - 18, mx = Math.max(10, ...counts), x0 = cx - ((ROWS + 1) / 2) * d;
      counts.forEach((c, i) => {
        const bh = (c / mx) * maxH;
        ctx.fillStyle = alpha(PAL.emb, 0.75); ctx.fillRect(x0 + i * d + 2, base - bh, d - 4, bh);
      });
      if (total > 40) {
        const mean = counts.reduce((a, c, i) => a + c * i, 0) / total, vr = counts.reduce((a, c, i) => a + c * (i - mean) ** 2, 0) / total, sd = Math.sqrt(vr) || 1;
        ctx.save(); glowLine(ctx, PAL.rob, 1.6, 6); ctx.beginPath();
        for (let x = 0; x <= ROWS + 1; x += 0.1) {
          const gval = Math.exp(-((x - 0.5 - mean) ** 2) / (2 * sd * sd)) / (sd * Math.sqrt(2 * Math.PI)) * total;
          const X = x0 + x * d, Y = base - (gval / mx) * maxH; x ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
        }
        ctx.stroke(); ctx.restore();
      }
      txt(ctx, `in flight ${balls.length}`, 14, 22, { size: 10.5, color: PAL.emb });
      txt(ctx, `fallen ${total}`, 14, 38, { size: 10.5, color: PAL.emb });
      txt(ctx, "— Gaussian fit", w - 14, 22, { align: "right", size: 10, color: PAL.rob });
    }
  };
};

/* ---------------------------------------------------------
   9. DDS birdsong — phase accumulator, spectrogram, audio
   --------------------------------------------------------- */
DEMOS.dds = env => {
  let t = 0, acc = 0, hist = [], audio = null, playing = 0;
  // chirp pattern: 4 down-sweeps 4.2 → 2.0 kHz, then a rest
  const freqAt = tt => {
    const c = tt % 2.6; if (c > 2.0) return 0;
    const k = c % 0.5; if (k > 0.36) return 0;
    return 4200 - (k / 0.36) * 2200;
  };
  function play() {
    try {
      audio = audio || new (window.AudioContext || window.webkitAudioContext)();
      const now = audio.currentTime + 0.02, osc = audio.createOscillator(), gn = audio.createGain();
      osc.type = "sine"; osc.connect(gn); gn.connect(audio.destination); gn.gain.value = 0;
      for (let i = 0; i < 4; i++) {
        const s = now + i * 0.5;
        osc.frequency.setValueAtTime(4200, s); osc.frequency.exponentialRampToValueAtTime(2000, s + 0.36);
        gn.gain.setValueAtTime(0, s); gn.gain.linearRampToValueAtTime(0.07, s + 0.03); gn.gain.setValueAtTime(0.07, s + 0.32); gn.gain.linearRampToValueAtTime(0, s + 0.36);
      }
      osc.start(now); osc.stop(now + 2); playing = 2;
    } catch (e) { /* audio unavailable */ }
  }
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt; playing = Math.max(0, playing - dt);
      const f = freqAt(t);
      acc = (acc + (f / 4200) * dt * 1.6) % 1;
      hist.push(f); if (hist.length > 260) hist.shift();
      clear(env);
      // phase wheel
      const wx = Math.max(70, w * 0.17), wy = h * 0.42, R = Math.min(h * 0.27, w * 0.13);
      ctx.strokeStyle = PAL.line; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(wx, wy, R, 0, 7); ctx.stroke();
      for (let k = 0; k < 32; k++) { const a = (k / 32) * Math.PI * 2; ctx.fillStyle = PAL.dim; ctx.fillRect(wx + Math.cos(a) * (R + 6) - 1, wy + Math.sin(a) * (R + 6) - 1, 2, 2); }
      const a = acc * Math.PI * 2 - Math.PI / 2;
      ctx.save(); glowLine(ctx, PAL.emb, 2.5, 12); ctx.beginPath(); ctx.moveTo(wx, wy); ctx.lineTo(wx + Math.cos(a) * R, wy + Math.sin(a) * R); ctx.stroke(); ctx.restore();
      ctx.strokeStyle = alpha(PAL.emb, 0.5); ctx.lineWidth = 6; ctx.beginPath(); ctx.arc(wx, wy, R - 8, -Math.PI / 2, a); ctx.stroke();
      txt(ctx, "phase accumulator", wx, wy + R + 24, { align: "center", size: 10, color: PAL.muted });
      const inc = Math.round((f * 2 ** 32) / 50000);
      txt(ctx, `Δφ = 0x${inc.toString(16).padStart(8, "0")}`, wx, wy + R + 40, { align: "center", size: 10, color: PAL.emb });
      txt(ctx, `f = ${f ? Math.round(f) + " Hz" : "—"}`, wx, wy - R - 14, { align: "center", size: 11, color: PAL.text, weight: 600 });
      // spectrogram
      const sx = wx + R + 36, sw = w - sx - 14, sy = 18, sh = h * 0.46;
      rr(ctx, sx, sy, sw, sh, 8); ctx.fillStyle = "rgba(255,255,255,0.02)"; ctx.fill(); ctx.strokeStyle = PAL.line; ctx.stroke();
      txt(ctx, "spectrogram", sx + 8, sy + 14, { size: 9.5, color: PAL.dim }); txt(ctx, "5 kHz", sx + sw - 6, sy + 14, { align: "right", size: 9, color: PAL.dim });
      ctx.save(); ctx.globalCompositeOperation = "lighter";
      hist.forEach((fv, i) => {
        if (!fv) return;
        const x = sx + (i / 260) * sw, y = sy + sh - (fv / 5000) * sh;
        ctx.fillStyle = alpha(PAL.emb, 0.18); ctx.fillRect(x, y - 5, sw / 260 + 1, 10);
        ctx.fillStyle = alpha(PAL.emb, 0.9); ctx.fillRect(x, y - 1, sw / 260 + 1, 2);
      });
      ctx.restore();
      // scope
      const oy = sy + sh + 20, oh = h - oy - 14;
      rr(ctx, sx, oy, sw, oh, 8); ctx.fillStyle = "rgba(255,255,255,0.02)"; ctx.fill(); ctx.strokeStyle = PAL.line; ctx.stroke();
      txt(ctx, "DAC out (SPI)", sx + 8, oy + 14, { size: 9.5, color: PAL.dim });
      const amp = f ? 1 : 0.02, cyc = (f || 1000) / 600;
      ctx.save(); glowLine(ctx, PAL.emb, 1.6, 8); ctx.beginPath();
      for (let x = 0; x <= sw; x += 2) { const y = oy + oh / 2 + Math.sin((x / sw) * cyc * Math.PI * 2 + t * 12) * (oh * 0.34) * amp; x ? ctx.lineTo(sx + x, y) : ctx.moveTo(sx + x, y); }
      ctx.stroke(); ctx.restore();
      txt(ctx, playing ? "♪ playing…" : "▶ click to play", sx + sw - 8, oy + 14, { align: "right", size: 9.5, color: playing ? PAL.emb : PAL.muted });
    },
    click() { play(); }
  };
};

/* ---------------------------------------------------------
   10. PID helicopter — 1 kHz loop, complementary filter
   --------------------------------------------------------- */
DEMOS.pid = env => {
  const Kp = 30, Ki = 10, Kd = 10, KG = 8, B = 0.5, UMAX = 18;
  let th = 0.2, om = 0, integ = 0, target = (100 * Math.PI) / 180, lastClick = -10, t = 0, nextT = 4, prop = 0, terms = [0, 0, 0], u = 0;
  const hist = [], seq = [100, 140, 60, 150, 120];
  let si = 0;
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt;
      if (t > nextT && t - lastClick > 6) { si = (si + 1) % seq.length; target = (seq[si] * Math.PI) / 180; nextT = t + 4.5; }
      const n = Math.max(1, Math.round(Math.min(dt, 0.05) * 1000));
      for (let i = 0; i < n; i++) {
        const e = target - th; integ = clamp(integ + e * 0.001, -0.5, 0.5); // anti-windup
        terms = [Kp * e, Ki * integ, -Kd * om];
        u = clamp(terms[0] + terms[1] + terms[2] + 0, 0, UMAX);
        const acc = u - KG * Math.sin(th) - B * om; om += acc * 0.001; th += om * 0.001;
      }
      prop += u * dt * 3;
      const deg = (th * 180) / Math.PI, raw = deg + gauss() * 5;
      hist.push([deg, (target * 180) / Math.PI, raw]); if (hist.length > 240) hist.shift();
      clear(env);
      const pvx = w * 0.27, pvy = h * 0.5, L = Math.min(w * 0.19, h * 0.36);
      // stand
      ctx.strokeStyle = PAL.dim; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(pvx, pvy); ctx.lineTo(pvx, h - 18); ctx.moveTo(pvx - 30, h - 18); ctx.lineTo(pvx + 30, h - 18); ctx.stroke();
      // target ghost
      const tx = pvx + Math.sin(target) * L, ty = pvy + Math.cos(target) * L;
      ctx.setLineDash([5, 5]); ctx.strokeStyle = alpha(PAL.rob, 0.7); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(pvx, pvy); ctx.lineTo(tx, ty); ctx.stroke(); ctx.setLineDash([]);
      txt(ctx, `target ${Math.round((target * 180) / Math.PI)}°`, tx + 8, ty - 8, { size: 10, color: PAL.rob });
      // arm
      const ax = pvx + Math.sin(th) * L, ay = pvy + Math.cos(th) * L;
      ctx.save(); glowLine(ctx, PAL.emb, 5, 10); ctx.lineCap = "round"; ctx.beginPath(); ctx.moveTo(pvx, pvy); ctx.lineTo(ax, ay); ctx.stroke(); ctx.restore();
      ctx.fillStyle = PAL.text; ctx.beginPath(); ctx.arc(pvx, pvy, 5, 0, 7); ctx.fill();
      // motor + spinning prop (perpendicular to arm)
      const nx = Math.cos(th), ny = -Math.sin(th), pl = 18 * Math.abs(Math.cos(prop));
      ctx.fillStyle = "#cbd5e1"; ctx.beginPath(); ctx.arc(ax, ay, 6, 0, 7); ctx.fill();
      ctx.strokeStyle = alpha(PAL.text, 0.8); ctx.lineWidth = 3; ctx.beginPath();
      ctx.moveTo(ax - Math.sin(th) * 8 - nx * pl, ay - Math.cos(th) * 8 - ny * pl); ctx.lineTo(ax - Math.sin(th) * 8 + nx * pl, ay - Math.cos(th) * 8 + ny * pl); ctx.stroke();
      // thrust arrow
      ctx.strokeStyle = alpha(PAL.perf, 0.8); ctx.lineWidth = 2; const tl = (u / UMAX) * 34;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax + nx * tl, ay + ny * tl); ctx.stroke();
      txt(ctx, `${deg.toFixed(0)}°`, pvx - 14, pvy - 12, { align: "right", size: 13, color: PAL.text, weight: 600 });
      txt(ctx, "0° = hanging down", 14, 20, { size: 9.5, color: PAL.dim });
      // P / I / D bars
      const bx = w * 0.56, bw = w - bx - 16;
      ["P", "I", "D"].forEach((n, i) => {
        const y = 22 + i * 20, v = clamp(terms[i] / UMAX, -1, 1);
        txt(ctx, n, bx, y + 8, { size: 10.5, color: PAL.text, weight: 600 });
        ctx.fillStyle = "rgba(255,255,255,0.05)"; ctx.fillRect(bx + 16, y, bw - 16, 10);
        const mid = bx + 16 + (bw - 16) / 2; ctx.fillStyle = [PAL.emb, PAL.perf, PAL.fpga][i];
        ctx.fillRect(Math.min(mid, mid + v * (bw - 16) / 2), y, Math.abs(v) * (bw - 16) / 2, 10);
      });
      // plot
      const py = 96, ph = h - py - 22;
      rr(ctx, bx, py, bw, ph, 8); ctx.fillStyle = "rgba(255,255,255,0.02)"; ctx.fill(); ctx.strokeStyle = PAL.line; ctx.stroke();
      const Y = v => py + ph - (v / 180) * ph;
      [[2, alpha(PAL.red, 0.35), 1], [1, PAL.rob, 1.2], [0, PAL.emb, 2]].forEach(([k, c, lw]) => {
        ctx.strokeStyle = c; ctx.lineWidth = lw; ctx.beginPath();
        hist.forEach((p, i) => { const x = bx + (i / 239) * bw, y = Y(clamp(p[k], 0, 180)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
      });
      ctx.strokeStyle = "rgba(255,255,255,0.12)"; ctx.setLineDash([2, 4]); ctx.beginPath(); ctx.moveTo(bx, Y(90)); ctx.lineTo(bx + bw, Y(90)); ctx.stroke(); ctx.setLineDash([]);
      txt(ctx, "90°", bx + 4, Y(90) - 4, { size: 9, color: PAL.dim });
      txt(ctx, "raw accel", bx + bw - 8, py + 14, { align: "right", size: 9, color: alpha(PAL.red, 0.8) });
      txt(ctx, "filtered", bx + bw - 68, py + 14, { align: "right", size: 9, color: PAL.emb });
      txt(ctx, "target", bx + bw - 120, py + 14, { align: "right", size: 9, color: PAL.rob });
    },
    click(x, y) {
      const { w, h } = env, pvx = w * 0.27, pvy = h * 0.5;
      const a = Math.atan2(x - pvx, y - pvy);
      target = clamp(Math.abs(a), (20 * Math.PI) / 180, (160 * Math.PI) / 180); lastClick = t;
    }
  };
};

/* ---------------------------------------------------------
   11. Bayes-filter grid localization (Fast Robots)
   --------------------------------------------------------- */
DEMOS.bayes = env => {
  const GW = 14, GH = 9;
  const obst = [[4, 0, 1, 3], [9, 5, 1, 4], [6, 5, 2, 1], [11, 2, 2, 1], [2, 6, 1, 1]];
  const blocked = (x, y) => x < 0 || y < 0 || x >= GW || y >= GH || obst.some(([ox, oy, ow, oh]) => x >= ox && x < ox + ow && y >= oy && y < oy + oh);
  const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1]];
  const rayLen = (x, y, [dx, dy]) => { let n = 0; while (!blocked(x + dx * (n + 1), y + dy * (n + 1))) n++; return n + 0.5; };
  const exp = []; for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) exp.push(blocked(x, y) ? null : dirs.map(d => rayLen(x, y, d)));
  // waypoint loop via BFS
  const wps = [[1, 1], [1, 7], [7, 7], [7, 3], [12, 4], [12, 1], [8, 1]];
  function bfs(a, b) {
    const prev = new Map(), q = [a]; prev.set(a + "", null);
    while (q.length) { const c = q.shift(); if (c[0] === b[0] && c[1] === b[1]) break; for (const [dx, dy] of dirs) { const n = [c[0] + dx, c[1] + dy]; if (!blocked(...n) && !prev.has(n + "")) { prev.set(n + "", c); q.push(n); } } }
    const path = []; let c = b; while (c) { path.push(c); c = prev.get(c + ""); } return path.reverse();
  }
  let route = []; wps.forEach((p, i) => { const seg = bfs(p, wps[(i + 1) % wps.length]); route.push(...seg.slice(1)); });
  let ri = 0, pos = [...wps[0]], prevPos = [...pos], bel, t = 0, steps = 0, obs = null;
  const reset = () => { bel = exp.map(e => (e ? 1 : 0)); norm(); steps = 0; };
  const norm = () => { const s = bel.reduce((a, b) => a + b, 0) || 1; for (let i = 0; i < bel.length; i++) bel[i] /= s; };
  reset();
  function stepFilter() {
    const nxt = route[ri]; ri = (ri + 1) % route.length;
    const u = [nxt[0] - pos[0], nxt[1] - pos[1]]; prevPos = pos; pos = [...nxt];
    // predict
    const nb = new Array(bel.length).fill(0);
    for (let i = 0; i < bel.length; i++) {
      if (bel[i] < 1e-7) continue;
      const x = i % GW, y = (i / GW) | 0; let tx = x + u[0], ty = y + u[1]; if (blocked(tx, ty)) { tx = x; ty = y; }
      nb[ty * GW + tx] += bel[i] * 0.8;
      for (const [dx, dy] of dirs) { const ax = tx + dx, ay = ty + dy; if (blocked(ax, ay)) nb[ty * GW + tx] += bel[i] * 0.05; else nb[ay * GW + ax] += bel[i] * 0.05; }
    }
    bel = nb;
    // update with noisy ToF rays
    obs = dirs.map(d => rayLen(pos[0], pos[1], d) + gauss() * 0.35);
    for (let i = 0; i < bel.length; i++) {
      if (!exp[i]) { bel[i] = 0; continue; }
      let l = 1; for (let k = 0; k < 4; k++) l *= Math.exp(-((obs[k] - exp[i][k]) ** 2) / (2 * 0.7 * 0.7));
      bel[i] *= l;
    }
    norm(); steps++;
    if (steps > 34) reset();
  }
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt; if (t > 0.55) { t = 0; stepFilter(); }
      clear(env, false);
      const cs = Math.min((w - 28) / GW, (h - 44) / GH), ox = (w - cs * GW) / 2, oy = 30;
      const mx = Math.max(...bel); let best = bel.indexOf(mx);
      for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) {
        const i = y * GW + x, X = ox + x * cs, Y = oy + y * cs;
        if (!exp[i]) { ctx.fillStyle = "#1b2233"; ctx.fillRect(X, Y, cs, cs); continue; }
        ctx.fillStyle = "rgba(255,255,255,0.025)"; ctx.fillRect(X + 1, Y + 1, cs - 2, cs - 2);
        // dim a flat (uncertain) belief; brighten as it concentrates
        const v = Math.sqrt(bel[i] / (mx || 1)) * Math.min(1, 0.12 + mx * 4);
        if (v > 0.02) { ctx.fillStyle = `rgba(251,191,36,${v * 0.9})`; ctx.fillRect(X + 1, Y + 1, cs - 2, cs - 2); }
      }
      ctx.strokeStyle = alpha(PAL.rob, 0.4); ctx.lineWidth = 2; ctx.strokeRect(ox, oy, cs * GW, cs * GH);
      // most likely cell
      ctx.save(); glowLine(ctx, PAL.rob, 2, 10); ctx.strokeRect(ox + (best % GW) * cs + 1, oy + ((best / GW) | 0) * cs + 1, cs - 2, cs - 2); ctx.restore();
      // robot (interpolated)
      const e = ease(Math.min(1, t / 0.4)), rx = ox + (lerp(prevPos[0], pos[0], e) + 0.5) * cs, ry = oy + (lerp(prevPos[1], pos[1], e) + 0.5) * cs;
      if (obs) dirs.forEach(([dx, dy], k) => {
        const len = obs[k] * cs; ctx.strokeStyle = alpha(PAL.perf, 0.55); ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx + dx * len, ry + dy * len); ctx.stroke(); ctx.setLineDash([]);
        ctx.fillStyle = PAL.perf; ctx.beginPath(); ctx.arc(rx + dx * len, ry + dy * len, 2.2, 0, 7); ctx.fill();
      });
      ctx.save(); ctx.fillStyle = "#fff"; ctx.shadowColor = "#fff"; ctx.shadowBlur = 10; ctx.beginPath(); ctx.arc(rx, ry, cs * 0.22, 0, 7); ctx.fill(); ctx.restore();
      txt(ctx, `step ${steps}   P(best) = ${mx.toFixed(2)}`, ox, 20, { size: 10.5, color: PAL.text });
      txt(ctx, "■ belief  ○ true pose  ┄ ToF", ox + cs * GW, 20, { align: "right", size: 9.5, color: PAL.muted });
    }
  };
};

/* ---------------------------------------------------------
   12. Particle filter → RRT → drive (Roomba)
   --------------------------------------------------------- */
DEMOS.pfrrt = env => {
  const WW = 10, WH = 6;
  const rects = [[2.2, 0, 0.35, 3.3], [5, 2.7, 0.35, 3.3], [7.5, 0, 0.35, 2.6]];
  const keep = { x: 6.5, y: 4.4, r: 0.55 };
  const wps = [[1, 1], [1, 5], [3.7, 5], [3.7, 1.3], [6.3, 1.3], [8.8, 1.2], [8.8, 4.8], [6.4, 2.8]];
  const free = (x, y, m = 0.18) => x > m && y > m && x < WW - m && y < WH - m && !rects.some(([rx, ry, rw, rh]) => x > rx - m && x < rx + rw + m && y > ry - m && y < ry + rh + m) && Math.abs(x - keep.x) + Math.abs(y - keep.y) > keep.r + m;
  const segFree = (a, b) => { const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.06); for (let i = 0; i <= n; i++) if (!free(lerp(a[0], b[0], i / n), lerp(a[1], b[1], i / n))) return false; return true; };
  let phase, pt, robot, parts, tree, goalIdx, path, pathI, visited, heading;
  function init() {
    const start = randi(wps.length); robot = [...wps[start]]; heading = rand(6);
    parts = []; wps.forEach(w => { for (let i = 0; i < 40; i++) parts.push({ x: w[0] + gauss() * 0.22, y: w[1] + gauss() * 0.22, a: rand(6.28), w: 1 }); });
    visited = [start]; goalIdx = (start + 1) % wps.length; phase = "localize"; pt = 0;
  }
  function startPlan() { tree = [{ p: [...robot], par: -1 }]; path = null; phase = "plan"; pt = 0; }
  function growTree(n) {
    const goal = wps[goalIdx];
    for (let k = 0; k < n; k++) {
      const s = Math.random() < 0.12 ? goal : [rand(WW), rand(WH)];
      let bi = 0, bd = 1e9; tree.forEach((nd, i) => { const d = (nd.p[0] - s[0]) ** 2 + (nd.p[1] - s[1]) ** 2; if (d < bd) { bd = d; bi = i; } });
      const q = tree[bi].p, d = Math.sqrt(bd); if (d < 1e-6) continue;
      const st = Math.min(0.5, d), np = [q[0] + ((s[0] - q[0]) / d) * st, q[1] + ((s[1] - q[1]) / d) * st];
      if (!segFree(q, np)) continue;
      tree.push({ p: np, par: bi });
      if (Math.hypot(np[0] - goal[0], np[1] - goal[1]) < 0.5 && segFree(np, goal)) {
        tree.push({ p: [...goal], par: tree.length - 1 });
        path = []; let i = tree.length - 1; while (i >= 0) { path.push(tree[i].p); i = tree[i].par; } path.reverse(); pathI = 0; phase = "drive"; pt = 0; return;
      }
    }
  }
  init();
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      pt += dt;
      if (phase === "localize") {
        heading += dt * 2.2;
        if (pt % 0.12 < dt) {
          const sig = lerp(1.6, 0.18, Math.min(1, pt / 3));
          parts.forEach(p => { p.w = Math.exp(-((p.x - robot[0]) ** 2 + (p.y - robot[1]) ** 2) / (2 * sig * sig)) + 1e-9; });
          const sum = parts.reduce((a, p) => a + p.w, 0), N = parts.length, np = []; let c = parts[0].w / sum, i = 0; const r0 = Math.random() / N;
          for (let m = 0; m < N; m++) { const u = r0 + m / N; while (u > c && i < N - 1) { i++; c += parts[i].w / sum; } const p = parts[i]; np.push({ x: p.x + gauss() * 0.05, y: p.y + gauss() * 0.05, a: p.a + gauss() * 0.1, w: p.w }); }
          parts = np;
        }
        if (pt > 3.4) startPlan();
      } else if (phase === "plan") { growTree(5); if (pt > 6) startPlan(); }
      else if (phase === "drive") {
        let move = dt * 2.4;
        while (move > 0 && pathI < path.length - 1) {
          const nx = path[pathI + 1], d = Math.hypot(nx[0] - robot[0], nx[1] - robot[1]);
          heading = Math.atan2(nx[1] - robot[1], nx[0] - robot[0]);
          if (d <= move) { robot = [...nx]; pathI++; move -= d; } else { robot[0] += ((nx[0] - robot[0]) / d) * move; robot[1] += ((nx[1] - robot[1]) / d) * move; move = 0; }
        }
        parts.forEach(p => { p.x = lerp(p.x, robot[0] + gauss() * 0.12, 0.2); p.y = lerp(p.y, robot[1] + gauss() * 0.12, 0.2); });
        if (pathI >= path.length - 1) {
          visited.push(goalIdx);
          if (visited.length > 4) { phase = "done"; pt = 0; } else { goalIdx = (goalIdx + 1) % wps.length; startPlan(); }
        }
      } else if (phase === "done" && pt > 1.5) init();

      clear(env, false);
      const s = Math.min((w - 24) / WW, (h - 40) / WH), ox = (w - s * WW) / 2, oy = 30;
      const X = x => ox + x * s, Y = y => oy + y * s;
      ctx.strokeStyle = alpha(PAL.rob, 0.45); ctx.lineWidth = 2; ctx.strokeRect(ox, oy, WW * s, WH * s);
      ctx.fillStyle = "#1b2233"; rects.forEach(([rx, ry, rw, rh]) => ctx.fillRect(X(rx), Y(ry), rw * s, rh * s));
      ctx.strokeStyle = alpha(PAL.red, 0.8); ctx.setLineDash([4, 3]); ctx.beginPath();
      ctx.moveTo(X(keep.x), Y(keep.y - keep.r)); ctx.lineTo(X(keep.x + keep.r), Y(keep.y)); ctx.lineTo(X(keep.x), Y(keep.y + keep.r)); ctx.lineTo(X(keep.x - keep.r), Y(keep.y)); ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
      wps.forEach((p, i) => {
        ctx.strokeStyle = visited.includes(i) ? PAL.emb : i === goalIdx && phase !== "localize" ? PAL.rob : PAL.dim; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 6, 0, 7); ctx.stroke();
      });
      if (tree && phase !== "localize") {
        ctx.strokeStyle = alpha(PAL.rob, 0.35); ctx.lineWidth = 1; ctx.beginPath();
        tree.forEach(nd => { if (nd.par >= 0) { const q = tree[nd.par].p; ctx.moveTo(X(q[0]), Y(q[1])); ctx.lineTo(X(nd.p[0]), Y(nd.p[1])); } }); ctx.stroke();
      }
      if (path && phase === "drive") { ctx.save(); glowLine(ctx, PAL.rob, 2.5, 10); ctx.beginPath(); path.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke(); ctx.restore(); }
      ctx.strokeStyle = alpha(PAL.perf, 0.7); ctx.lineWidth = 1;
      parts.forEach(p => { ctx.beginPath(); ctx.moveTo(X(p.x), Y(p.y)); ctx.lineTo(X(p.x) + Math.cos(p.a) * 5, Y(p.y) + Math.sin(p.a) * 5); ctx.stroke(); });
      const R = s * 0.24;
      ctx.save(); ctx.fillStyle = "#e5e7eb"; ctx.shadowColor = "#fff"; ctx.shadowBlur = 10; ctx.beginPath(); ctx.arc(X(robot[0]), Y(robot[1]), R, 0, 7); ctx.fill(); ctx.restore();
      ctx.strokeStyle = "#0b0f19"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(X(robot[0]), Y(robot[1])); ctx.lineTo(X(robot[0]) + Math.cos(heading) * R, Y(robot[1]) + Math.sin(heading) * R); ctx.stroke();
      const label = { localize: "1 · localize — particles converge", plan: "2 · plan — RRT grows", drive: "3 · drive — PF tracks pose", done: "✓ waypoints visited" }[phase];
      txt(ctx, label, ox, 20, { size: 10.5, color: PAL.text });
    }
  };
};

/* ---------------------------------------------------------
   13. Arm IK → PRM roadmap in C-space → execute
   --------------------------------------------------------- */
DEMOS.prm = env => {
  const N = 56;
  let built = false, lay, cfree, nodes, edges, q, task, seg, phase, pt, carry, path, pathI, lastW = 0, lastH = 0, inserted;
  function layout() {
    const { w, h } = env, half = w * 0.5;
    const base = [half * 0.5, h - 26], L = Math.min(half * 0.28, (h - 50) * 0.42);
    // a low wall between the base and the drop zone: reachable only by lifting over it
    return { half, base, L, obs: [base[0] + L * 0.65, h - 26 - L * 0.45, L * 0.2, L * 0.45] };
  }
  function fk(t1, t2) {
    const { base, L } = lay, e = [base[0] + L * Math.cos(t1), base[1] - L * Math.sin(t1)];
    return [e, [e[0] + L * Math.cos(t1 + t2), e[1] - L * Math.sin(t1 + t2)]];
  }
  function collide(t1, t2) {
    const { base, obs } = lay, [e, tip] = fk(t1, t2), m = 4;
    for (let i = 1; i <= 10; i++) {
      for (const [a, b] of [[base, e], [e, tip]]) {
        const x = lerp(a[0], b[0], i / 10), y = lerp(a[1], b[1], i / 10);
        if (y > base[1] + 1) return true;
        if (x > obs[0] - m && x < obs[0] + obs[2] + m && y > obs[1] - m && y < obs[1] + obs[3] + m) return true;
      }
    }
    return false;
  }
  const edgeFree = (a, b) => { for (let i = 0; i <= 14; i++) if (collide(lerp(a[0], b[0], i / 14), lerp(a[1], b[1], i / 14))) return false; return true; };
  const T1 = [0.02, Math.PI - 0.02], T2 = [-Math.PI * 0.95, Math.PI * 0.95];
  function build() {
    lay = layout(); cfree = [];
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) cfree.push(!collide(lerp(T1[0], T1[1], (i + 0.5) / N), lerp(T2[0], T2[1], (j + 0.5) / N)));
    nodes = []; let tries = 0;
    while (nodes.length < 90 && tries++ < 2000) { const c = [rand(T1[0], T1[1]), rand(T2[0], T2[1])]; if (!collide(...c)) nodes.push(c); }
    edges = nodes.map(() => []);
    nodes.forEach((a, i) => connect(i));
    task = [{ x: lay.base[0] - lay.L * 1.45, y: lay.base[1] - 6, pick: true }, { x: lay.base[0] + lay.L * 1.4, y: lay.base[1] - 6, pick: false }, { x: lay.base[0] - lay.L * 0.95, y: lay.base[1] - 6, pick: true }, { x: lay.base[0] + lay.L * 1.55, y: lay.base[1] - 6, pick: false }];
    q = [Math.PI / 2, -0.6]; seg = 0; carry = false; nextTask(); built = true; lastW = env.w; lastH = env.h;
  }
  const dist = (a, b) => Math.hypot((a[0] - b[0]) * 1.4, a[1] - b[1]);
  function connect(i) {
    const a = nodes[i];
    nodes.map((b, j) => [dist(a, b), j]).filter(([d, j]) => j !== i && d < 1.1).sort((x, y) => x[0] - y[0]).slice(0, 6)
      .forEach(([d, j]) => { if (!edges[i].includes(j) && edgeFree(a, nodes[j])) { edges[i].push(j); edges[j].push(i); } });
  }
  function ik(x, y) {
    const { base, L } = lay, dx = x - base[0], dy = base[1] - y, D = (dx * dx + dy * dy - 2 * L * L) / (2 * L * L);
    if (Math.abs(D) > 1) return null;
    for (const s of [1, -1]) {
      const t2 = s * Math.acos(D), t1 = Math.atan2(dy, dx) - Math.atan2(L * Math.sin(t2), L + L * Math.cos(t2));
      if (t1 >= T1[0] && t1 <= T1[1] && !collide(t1, t2)) return [t1, t2];
    }
    return null;
  }
  function dijkstra(s, g) {
    const n = nodes.length, d = new Array(n).fill(Infinity), pr = new Array(n).fill(-1), done = new Array(n).fill(false); d[s] = 0;
    for (;;) { let u = -1; for (let i = 0; i < n; i++) if (!done[i] && d[i] < Infinity && (u < 0 || d[i] < d[u])) u = i; if (u < 0 || u === g) break; done[u] = true; edges[u].forEach(v => { const nd = d[u] + dist(nodes[u], nodes[v]); if (nd < d[v]) { d[v] = nd; pr[v] = u; } }); }
    if (d[g] === Infinity) return null; const p = []; for (let v = g; v >= 0; v = pr[v]) p.push(v); return p.reverse();
  }
  function nextTask() {
    const tk = task[seg % task.length], goal = ik(tk.x, tk.y);
    phase = "ik"; pt = 0; path = null; inserted = null;
    if (!goal) { seg++; return; }
    nodes.push([...q]); edges.push([]); const si = nodes.length - 1; connect(si);
    nodes.push(goal); edges.push([]); const gi = nodes.length - 1; connect(gi);
    inserted = [si, gi];
    const p = dijkstra(si, gi); path = p ? p.map(i => nodes[i]) : [q, goal]; pathI = 0;
  }
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      if (!built || Math.abs(w - lastW) > 2 || Math.abs(h - lastH) > 2) build();
      pt += dt;
      if (phase === "ik" && pt > 0.7) { phase = "query"; pt = 0; }
      else if (phase === "query" && pt > 0.7) { phase = "exec"; pt = 0; }
      else if (phase === "exec") {
        let mv = dt * 1.5;
        while (mv > 0 && pathI < path.length - 1) { const nx = path[pathI + 1], d = dist(q, nx); if (d <= mv) { q = [...nx]; pathI++; mv -= d; } else { q = [q[0] + ((nx[0] - q[0]) / d) * mv, q[1] + ((nx[1] - q[1]) / d) * mv]; mv = 0; } }
        if (pathI >= path.length - 1) { carry = task[seg % task.length].pick; seg++; nextTask(); }
      }
      clear(env, false);
      const { half, base, L, obs } = lay;
      // workspace
      ctx.fillStyle = "rgba(255,255,255,0.03)"; ctx.fillRect(10, base[1], half - 20, 2);
      ctx.fillStyle = "#1b2233"; ctx.fillRect(obs[0], obs[1], obs[2], obs[3]);
      txt(ctx, "WORKSPACE", 14, 20, { size: 10, color: PAL.rob, weight: 600 });
      task.forEach((tk, i) => { ctx.strokeStyle = tk.pick ? alpha(PAL.perf, 0.6) : alpha(PAL.emb, 0.6); ctx.strokeRect(tk.x - 8, tk.y - 8, 16, 14); });
      const cur = task[seg % task.length];
      if (phase === "ik") { const r = 8 + (pt / 0.7) * 10; ctx.strokeStyle = alpha(PAL.rob, 1 - pt / 0.7); ctx.beginPath(); ctx.arc(cur.x, cur.y, r, 0, 7); ctx.stroke(); }
      const [e, tip] = fk(...q);
      ctx.save(); glowLine(ctx, PAL.rob, 5, 8); ctx.lineCap = "round"; ctx.beginPath(); ctx.moveTo(...base); ctx.lineTo(...e); ctx.lineTo(...tip); ctx.stroke(); ctx.restore();
      [base, e].forEach(p => { ctx.fillStyle = PAL.text; ctx.beginPath(); ctx.arc(p[0], p[1], 4, 0, 7); ctx.fill(); });
      if (carry) { ctx.fillStyle = PAL.perf; ctx.fillRect(tip[0] - 6, tip[1] - 2, 12, 10); }
      ctx.strokeStyle = PAL.text; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(tip[0] - 7, tip[1] - 4); ctx.lineTo(tip[0] - 7, tip[1] + 6); ctx.moveTo(tip[0] + 7, tip[1] - 4); ctx.lineTo(tip[0] + 7, tip[1] + 6); ctx.stroke();
      // C-space
      const cx0 = half + 8, cw = w - cx0 - 12, cy0 = 30, ch = h - cy0 - 16;
      const CX = t1 => cx0 + ((t1 - T1[0]) / (T1[1] - T1[0])) * cw, CY = t2 => cy0 + ((t2 - T2[0]) / (T2[1] - T2[0])) * ch;
      txt(ctx, "C-SPACE  θ₁ × θ₂", cx0, 20, { size: 10, color: PAL.rob, weight: 600 });
      for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) if (!cfree[i * N + j]) { ctx.fillStyle = "rgba(248,113,113,0.16)"; ctx.fillRect(cx0 + (i / N) * cw, cy0 + (j / N) * ch, cw / N + 0.5, ch / N + 0.5); }
      ctx.strokeStyle = PAL.line; ctx.strokeRect(cx0, cy0, cw, ch);
      ctx.strokeStyle = "rgba(255,255,255,0.09)"; ctx.lineWidth = 1; ctx.beginPath();
      edges.forEach((es, i) => es.forEach(j => { if (j > i) { ctx.moveTo(CX(nodes[i][0]), CY(nodes[i][1])); ctx.lineTo(CX(nodes[j][0]), CY(nodes[j][1])); } })); ctx.stroke();
      ctx.fillStyle = PAL.dim; nodes.forEach(n => ctx.fillRect(CX(n[0]) - 1.5, CY(n[1]) - 1.5, 3, 3));
      if (inserted) inserted.forEach((k, ii) => {
        if (!nodes[k]) return; ctx.strokeStyle = alpha(ii ? PAL.emb : PAL.perf, 0.8); ctx.beginPath();
        edges[k].forEach(j => { ctx.moveTo(CX(nodes[k][0]), CY(nodes[k][1])); ctx.lineTo(CX(nodes[j][0]), CY(nodes[j][1])); }); ctx.stroke();
        ctx.fillStyle = ii ? PAL.emb : PAL.perf; ctx.beginPath(); ctx.arc(CX(nodes[k][0]), CY(nodes[k][1]), 4, 0, 7); ctx.fill();
      });
      if (path && phase !== "ik") { ctx.save(); glowLine(ctx, PAL.rob, 2.2, 8); ctx.beginPath(); path.forEach((p, i) => (i ? ctx.lineTo(CX(p[0]), CY(p[1])) : ctx.moveTo(CX(p[0]), CY(p[1])))); ctx.stroke(); ctx.restore(); }
      ctx.save(); ctx.fillStyle = "#fff"; ctx.shadowColor = "#fff"; ctx.shadowBlur = 8; ctx.beginPath(); ctx.arc(CX(q[0]), CY(q[1]), 3.5, 0, 7); ctx.fill(); ctx.restore();
      const lbl = { ik: "IK → goal config", query: "insert start/goal · Dijkstra query", exec: "execute joint-space path" }[phase];
      txt(ctx, lbl, w - 14, h - 22, { align: "right", size: 10, color: PAL.text });
      // keep the roadmap from growing without bound
      if (nodes.length > 130) build();
    }
  };
};

/* ---------------------------------------------------------
   14. Closed-loop stepper arm — encoders catch missed steps
   --------------------------------------------------------- */
DEMOS.stepper = env => {
  const STEP = 0.9, names = ["A · base", "B · shoulder", "C · elbow"], slip = [0.02, 0.12, 0.06];
  const J = [0, 1, 2].map(i => ({ tgt: 0, act: [0, 60, -40][i], est: [0, 60, -40][i], integ: 0, acc: 0, pulses: [], ph: rand(6) }));
  let t = 0, grip = 0;
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt;
      J.forEach((j, i) => {
        j.tgt = [Math.sin(t * 0.5 + j.ph) * 70, 60 + Math.sin(t * 0.37 + j.ph) * 20, -40 + Math.sin(t * 0.43 + j.ph) * 30][i];
        const enc = Math.round(j.act * 4096 / 360) * 360 / 4096; // AS5600: 12-bit
        const e = j.tgt - enc; j.integ = clamp(j.integ + e * dt, -20, 20);
        const rate = clamp(6 * e + 1.5 * j.integ, -260, 260); // steps/s
        j.acc += rate * dt * 0.25; let pulsed = false;
        while (Math.abs(j.acc) >= 1) {
          const s = Math.sign(j.acc); j.acc -= s; j.est += s * STEP; pulsed = true;
          if (Math.random() > slip[i]) j.act += s * STEP; // missed step under load
        }
        if (Math.abs(j.est - j.act) > 30) j.est = j.act; // re-home the open-loop count
        j.pulses.push(pulsed ? 1 : 0); if (j.pulses.length > 90) j.pulses.shift();
        j.enc = enc;
      });
      grip = 0.5 + 0.5 * Math.sin(t * 1.3);
      clear(env);
      // side view arm (B shoulder, C elbow), A shown as base yaw dial
      const half = w * 0.5, bx = half * 0.45, by = h - 30, L1 = Math.min(h * 0.36, half * 0.38), L2 = L1 * 0.85;
      ctx.fillStyle = "#1b2233"; rr(ctx, bx - 26, by - 6, 52, 18, 4); ctx.fill();
      ctx.fillStyle = "rgba(255,255,255,0.05)"; ctx.fillRect(10, by + 12, half - 20, 2);
      const a1 = (J[1].act * Math.PI) / 180, a2 = a1 + (J[2].act * Math.PI) / 180;
      const e = [bx + Math.cos(a1) * L1, by - 6 - Math.sin(a1) * L1], tip = [e[0] + Math.cos(a2) * L2, e[1] - Math.sin(a2) * L2];
      ctx.save(); glowLine(ctx, PAL.rob, 7, 8); ctx.lineCap = "round"; ctx.beginPath(); ctx.moveTo(bx, by - 6); ctx.lineTo(...e); ctx.lineTo(...tip); ctx.stroke(); ctx.restore();
      [[bx, by - 6], e].forEach(p => { ctx.fillStyle = "#2b3450"; ctx.beginPath(); ctx.arc(p[0], p[1], 8, 0, 7); ctx.fill(); ctx.strokeStyle = PAL.emb; ctx.lineWidth = 1.5; ctx.stroke(); });
      const ga = 0.25 + grip * 0.45;
      ctx.strokeStyle = PAL.text; ctx.lineWidth = 2.5;
      [-1, 1].forEach(sd => { ctx.beginPath(); ctx.moveTo(...tip); ctx.lineTo(tip[0] + Math.cos(a2 + sd * ga) * 14, tip[1] - Math.sin(a2 + sd * ga) * 14); ctx.stroke(); });
      txt(ctx, "● encoder on every joint", 14, 20, { size: 9.5, color: PAL.emb });
      txt(ctx, `servo gripper ${Math.round(120 + grip * 60)}°`, 14, 36, { size: 9.5, color: PAL.dim });
      // dials
      const dx = half + 10, rowH = (h - 20) / 3, R = Math.min(rowH * 0.36, 30);
      J.forEach((j, i) => {
        const cy = 10 + rowH * i + rowH / 2, cx = dx + R + 4;
        ctx.strokeStyle = PAL.line; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.stroke();
        const needle = (deg, col, lw, len = R - 3) => { const a = ((deg - 90) * Math.PI) / 180; ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(a) * len, cy + Math.sin(a) * len); ctx.stroke(); };
        needle(j.est, alpha(PAL.red, 0.6), 1.5, R - 6); needle(j.tgt, PAL.rob, 2); needle(j.enc, PAL.emb, 2.5, R - 9);
        const tx = cx + R + 12;
        txt(ctx, names[i], tx, cy - R + 10, { size: 10, color: PAL.text, weight: 600 });
        txt(ctx, `err ${(j.tgt - j.enc).toFixed(1)}°`, tx, cy - R + 24, { size: 9.5, color: PAL.emb });
        txt(ctx, `open-loop drift ${(j.est - j.act).toFixed(1)}°`, tx, cy - R + 37, { size: 9, color: alpha(PAL.red, 0.85) });
        const pw = w - tx - 12, py = cy + R - 8;
        if (pw > 30) { ctx.strokeStyle = alpha(PAL.perf, 0.7); ctx.lineWidth = 1; ctx.beginPath(); j.pulses.forEach((p, k) => { const x = tx + (k / 90) * pw, y = py - p * 8; k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
      });
      txt(ctx, "— target  — encoder  — step count", w - 12, h - 6, { align: "right", size: 9, color: PAL.dim });
    }
  };
};

/* ---------------------------------------------------------
   15. Tiled GEMM + runtime bars
   --------------------------------------------------------- */
DEMOS.gemm = env => {
  const NT = 4; let ii = 0, jj = 0, kk = 0, t = 0, acc = Array.from({ length: NT * NT }, () => 0), barT = 0;
  const bars = [["naive ijk", 18.2], ["loop order ikj", 3.79], ["tiled (BS 48)", 2.23], ["+ reg-block + prefetch", 1.5], ["best combined", 0.7], ["OpenBLAS (ref)", 0.07]];
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt; barT = Math.min(1, barT + dt / 1.6);
      if (t > 0.42) { t = 0; acc[ii * NT + jj] = kk + 1; kk++; if (kk >= NT) { kk = 0; jj++; if (jj >= NT) { jj = 0; ii++; if (ii >= NT) { ii = 0; acc.fill(0); } } } }
      clear(env);
      const leftW = w * 0.56, S = Math.min((leftW - 40) / 2, (h - 56) / 2), ts = S / NT;
      const cx = 18 + S + 14, cy = 26 + S + 10, ax = 18, by = 26;
      const grid = (x, y, label, fill) => {
        txt(ctx, label, x, y - 6, { size: 10, color: PAL.muted });
        for (let r = 0; r < NT; r++) for (let c = 0; c < NT; c++) { const f = fill(r, c); ctx.fillStyle = f || "rgba(255,255,255,0.04)"; ctx.fillRect(x + c * ts + 1, y + r * ts + 1, ts - 2, ts - 2); }
      };
      grid(ax, cy, "A", (r, c) => (r === ii && c === kk ? alpha(PAL.perf, 0.75) : null));
      grid(cx, by, "B", (r, c) => (r === kk && c === jj ? alpha(PAL.fpga, 0.75) : null));
      grid(cx, cy, "C += A·B", (r, c) => { const v = acc[r * NT + c]; return v ? `rgba(120,240,210,${0.12 + (v / NT) * 0.6})` : null; });
      ctx.save(); glowLine(ctx, PAL.brand, 2, 12); ctx.strokeRect(cx + jj * ts + 1, cy + ii * ts + 1, ts - 2, ts - 2); ctx.restore();
      // flow lines from A and B tiles into C tile
      ctx.strokeStyle = alpha(PAL.perf, 0.5); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(ax + kk * ts + ts / 2, cy + ii * ts + ts / 2); ctx.lineTo(cx + jj * ts + ts / 2, cy + ii * ts + ts / 2); ctx.stroke();
      ctx.strokeStyle = alpha(PAL.fpga, 0.5); ctx.beginPath(); ctx.moveTo(cx + jj * ts + ts / 2, by + kk * ts + ts / 2); ctx.lineTo(cx + jj * ts + ts / 2, cy + ii * ts + ts / 2); ctx.stroke(); ctx.setLineDash([]);
      rr(ctx, ax, by, S - 4, S - 10, 8); ctx.fillStyle = "rgba(255,255,255,0.02)"; ctx.fill(); ctx.strokeStyle = PAL.line; ctx.stroke();
      txt(ctx, "cache-resident", ax + 10, by + 18, { size: 9.5, color: PAL.dim });
      [[PAL.perf, "A tile"], [PAL.fpga, "B tile"], [PAL.brand, "C tile"]].forEach(([c, n], k) => { ctx.fillStyle = alpha(c, 0.75); ctx.fillRect(ax + 10, by + 30 + k * 16, 10, 10); txt(ctx, n, ax + 26, by + 39 + k * 16, { size: 9.5 }); });
      txt(ctx, `tile (${ii},${jj}) k=${kk}`, ax + 10, by + S - 20, { size: 9.5, color: PAL.text });
      // bars (log scale)
      const bx = leftW + 10, bw = w - bx - 54, bh = Math.min(22, (h - 60) / bars.length - 8);
      txt(ctx, "runtime, 2000² DGEMM (log)", bx, 22, { size: 10, color: PAL.perf });
      const lg = v => Math.log10(v / 0.04) / Math.log10(18.2 / 0.04);
      bars.forEach(([n, v], i) => {
        const y = 38 + i * (bh + 16);
        txt(ctx, n, bx, y + 2, { size: 9.5, color: i === 5 ? PAL.dim : PAL.muted });
        ctx.fillStyle = "rgba(255,255,255,0.04)"; ctx.fillRect(bx, y + 6, bw, bh);
        ctx.fillStyle = i === 5 ? alpha(PAL.perf, 0.3) : i === 4 ? PAL.brand : alpha(PAL.perf, 0.8);
        ctx.fillRect(bx, y + 6, lg(v) * bw * ease(barT), bh);
        txt(ctx, `${v} s`, bx + lg(v) * bw * ease(barT) + 4, y + 6 + bh / 2 + 4, { size: 9.5, color: PAL.text });
      });
    }
  };
};

/* ---------------------------------------------------------
   16. Back-substitution: per-row parallel dot products
   --------------------------------------------------------- */
DEMOS.bksb = env => {
  const n = 12, P = 4, CUT = 3; let row = n - 1, t = 0, x = new Array(n).fill(null), hold = 0;
  const vers = [["naive", 0.38], ["opt1", 0.27], ["opt2", 1.62], ["opt3", 2.25], ["opt4", 2.51], ["opt8", 2.69]];
  function chunks(i) {
    const len = n - 1 - i; if (len <= 0) return [];
    const k = len < CUT ? 1 : Math.min(P, Math.ceil(len / CUT));
    const out = []; let s = i + 1;
    for (let p = 0; p < k; p++) { const c = Math.floor(len / k) + (p < len % k ? 1 : 0); out.push([s, s + c]); s += c; }
    return out;
  }
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      if (row < 0) { hold += dt; if (hold > 1.4) { row = n - 1; x.fill(null); hold = 0; } }
      else { t += dt; if (t > 0.8) { t = 0; x[row] = 1; row--; } }
      clear(env);
      const S = Math.min(h - 50, w * 0.48), cs = S / n, ox = 16, oy = 34;
      const ch = row >= 0 ? chunks(row) : [];
      txt(ctx, "U · x = b'   (solve bottom-up)", ox, 20, { size: 10, color: PAL.muted });
      for (let i = 0; i < n; i++) for (let j = i; j < n; j++) {
        let col = "rgba(255,255,255,0.06)";
        if (i === row) {
          if (j === i) col = alpha(PAL.rob, 0.6);
          ch.forEach(([a, b], p) => { if (j >= a && j < b) col = alpha(THREAD_COLORS[p], 0.3 + 0.5 * ease(Math.min(1, t / 0.45))); });
        } else if (i > row) col = "rgba(56,189,248,0.10)";
        ctx.fillStyle = col; ctx.fillRect(ox + j * cs + 1, oy + i * cs + 1, cs - 2, cs - 2);
      }
      // x vector
      const xx = ox + S + 12;
      for (let i = 0; i < n; i++) {
        ctx.fillStyle = x[i] ? alpha(PAL.brand, 0.7) : i === row ? alpha(PAL.rob, 0.25) : "rgba(255,255,255,0.05)";
        ctx.fillRect(xx, oy + i * cs + 1, cs - 2, cs - 2);
      }
      txt(ctx, "x", xx + cs / 2 - 3, oy - 6, { size: 10 });
      if (row >= 0) {
        const ry = oy + row * cs + cs / 2;
        txt(ctx, ch.length > 1 ? `${ch.length} threads · per-thread partial sums` : ch.length ? "serial cutoff (short row)" : "x = b / u", ox, oy + S + 14, { size: 10, color: ch.length > 1 ? PAL.text : PAL.rob });
        if (t > 0.45) ch.forEach(([a, b], p) => {
          const sx = ox + ((a + b) / 2) * cs, e = ease(Math.min(1, (t - 0.45) / 0.3));
          ctx.fillStyle = THREAD_COLORS[p]; ctx.beginPath(); ctx.arc(lerp(sx, xx + cs / 2, e), lerp(ry - cs * 0.9, ry, e), 3, 0, 7); ctx.fill();
        });
      }
      // speedup bars
      const bx = xx + cs + 28, bw = w - bx - 12;
      if (bw > 80) {
        txt(ctx, "speedup vs serial", bx, 20, { size: 10, color: PAL.perf });
        vers.forEach(([nm, v], i) => {
          const y = 34 + i * Math.min(30, (h - 60) / vers.length);
          txt(ctx, nm, bx, y + 9, { size: 9.5, color: PAL.muted });
          ctx.fillStyle = v < 1 ? alpha(PAL.red, 0.7) : i === 5 ? PAL.brand : alpha(PAL.perf, 0.8);
          ctx.fillRect(bx + 40, y, (v / 3.3) * (bw - 40), 11);
          txt(ctx, `${v}×`, bx + 44 + (v / 3.3) * (bw - 40), y + 9.5, { size: 9.5, color: PAL.text });
        });
        ctx.strokeStyle = "rgba(255,255,255,0.25)"; ctx.setLineDash([2, 3]); const ox1 = bx + 40 + (1 / 3.3) * (bw - 40);
        ctx.beginPath(); ctx.moveTo(ox1, 28); ctx.lineTo(ox1, 34 + vers.length * Math.min(30, (h - 60) / vers.length)); ctx.stroke(); ctx.setLineDash([]);
      }
    }
  };
};

/* ---------------------------------------------------------
   17. Sparse GEMM load balancing — cyclic vs op-count split
   --------------------------------------------------------- */
DEMOS.spgemm = env => {
  const P = 4, R = 28; let rows, plans, t, hold;
  function init() {
    rows = Array.from({ length: R }, (_, i) => (i % 4 === 0 && Math.random() < 0.85 ? rand(9, 18) : rand(0.4, 2.2)));
    const cyc = Array.from({ length: P }, () => []); rows.forEach((_, i) => cyc[i % P].push(i));
    const total = rows.reduce((a, b) => a + b, 0), bal = Array.from({ length: P }, () => []); let accw = 0, p = 0;
    rows.forEach((v, i) => { if (accw >= (total * (p + 1)) / P && p < P - 1) p++; bal[p].push(i); accw += v; });
    plans = [["cyclic rows (naive)", cyc], ["split by op count (balanced)", bal]].map(([n, lanes]) => ({ n, lanes, span: Math.max(...lanes.map(l => l.reduce((a, i) => a + rows[i], 0))) }));
    plans.total = total; t = 0; hold = 0;
  }
  init();
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      const maxSpan = Math.max(...plans.map(p => p.span));
      if (t < maxSpan) t += dt * maxSpan / 3.6; else { hold += dt; if (hold > 2) init(); }
      clear(env);
      const lx = 64, lw = w - lx - 16, ph = (h - 30) / 2;
      plans.forEach((pl, k) => {
        const y0 = 16 + k * ph, lh = Math.min(18, (ph - 34) / P);
        txt(ctx, pl.n, 14, y0 + 10, { size: 10.5, color: k ? PAL.brand : PAL.text, weight: 600 });
        pl.lanes.forEach((lane, p) => {
          const y = y0 + 20 + p * (lh + 4); let x = 0;
          txt(ctx, `T${p}`, 14, y + lh - 4, { size: 9.5, color: THREAD_COLORS[p] });
          ctx.fillStyle = "rgba(255,255,255,0.03)"; ctx.fillRect(lx, y, lw, lh);
          lane.forEach(i => {
            const v = rows[i], vis = clamp(t - x, 0, v);
            if (vis > 0) { ctx.fillStyle = `hsla(${(i * 37) % 360},70%,60%,0.8)`; ctx.fillRect(lx + (x / maxSpan) * lw, y + 1, Math.max(1, (vis / maxSpan) * lw - 1), lh - 2); }
            x += v;
          });
        });
        const done = t >= pl.span, sx = lx + (Math.min(t, pl.span) / maxSpan) * lw;
        ctx.strokeStyle = done ? alpha(PAL.emb, 0.9) : alpha(PAL.text, 0.5); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(sx, y0 + 16); ctx.lineTo(sx, y0 + 22 + P * (lh + 4)); ctx.stroke();
        if (done) txt(ctx, `makespan ${pl.span.toFixed(0)} · ${(plans.total / pl.span).toFixed(1)}× on ${P} threads`, w - 14, y0 + 10, { align: "right", size: 10, color: PAL.emb });
      });
    }
  };
};

/* ---------------------------------------------------------
   18. RAG pipeline — where the time goes
   --------------------------------------------------------- */
DEMOS.rag = env => {
  const stages = [["Query", 0, PAL.muted], ["Embedding", 140, PAL.fpga], ["Vector search", 33, PAL.rob], ["Generation", 5353, PAL.perf], ["Answer", 0, PAL.emb]];
  const SCALE = 0.00045; // display seconds per ms
  let st = 0, t = 0, tokens = 0, tl = [0, 0, 0];
  return {
    draw(dt) {
      const { ctx, w, h } = env;
      t += dt;
      const dur = st === 0 ? 0.5 : st === 4 ? 1.2 : Math.max(0.35, stages[st][1] * SCALE);
      if (st >= 1 && st <= 3) tl[st - 1] += dt;
      if (st === 3) tokens += dt * 9;
      if (t > dur) { t = 0; st++; if (st > 4) { st = 0; tokens = 0; } }
      clear(env);
      const pad = 14, gap = 10, bw = (w - pad * 2 - gap * 4) / 5, by = 34, bh = Math.min(66, h * 0.22);
      stages.forEach(([n, ms, c], k) => {
        const x = pad + k * (bw + gap), act = k === st;
        rr(ctx, x, by, bw, bh, 9); ctx.fillStyle = act ? alpha(c, 0.18) : PAL.panel; ctx.fill(); ctx.strokeStyle = act ? c : PAL.line; ctx.lineWidth = act ? 1.5 : 1; ctx.stroke();
        txt(ctx, n, x + bw / 2, by + bh / 2 - 2, { align: "center", size: Math.min(10.5, bw / 8), color: act ? PAL.text : PAL.muted, weight: 600 });
        if (ms) txt(ctx, `${ms.toLocaleString()} ms`, x + bw / 2, by + bh / 2 + 13, { align: "center", size: 9, color: c });
        if (k < 4) { ctx.fillStyle = PAL.dim; ctx.fillRect(x + bw + 2, by + bh / 2, gap - 4, 1.5); }
        if (act && k > 0 && k < 4) { const p = t / dur; ctx.fillStyle = c; ctx.fillRect(x + 6, by + bh - 7, (bw - 12) * p, 3); }
      });
      txt(ctx, "mean per query · four services over HTTP", pad, 22, { size: 10, color: PAL.muted });
      // generation token stream
      const gy = by + bh + 26, words = "the answer is grounded in the retrieved passages about".split(" ");
      txt(ctx, "decode ▸ " + words.slice(0, Math.floor(tokens) % (words.length + 1)).join(" ") + (st === 3 && (t * 3) % 1 < 0.5 ? " ▍" : ""), pad, gy, { size: 10.5, color: st >= 3 ? PAL.perf : PAL.dim });
      // timeline breakdown
      const ty = gy + 26, tw = w - pad * 2, tot = 140 + 33 + 5353;
      txt(ctx, "share of wall time", pad, ty, { size: 10, color: PAL.muted });
      let x = pad;
      [[140, PAL.fpga], [33, PAL.rob], [5353, PAL.perf]].forEach(([ms, c]) => { const ww = (ms / tot) * tw; ctx.fillStyle = c; ctx.fillRect(x, ty + 8, Math.max(2, ww), 14); x += ww; });
      txt(ctx, "generation 96.9%", w - pad, ty + 38, { align: "right", size: 10.5, color: PAL.perf, weight: 600 });
      txt(ctx, "embed 2.5% · search 0.6%", pad, ty + 38, { size: 9.5, color: PAL.muted });
      // mini roofline
      const ry = ty + 54, rh = h - ry - 12, rw = Math.min(200, w * 0.4), rx = w - pad - rw;
      if (rh > 50) {
        ctx.strokeStyle = PAL.line; ctx.strokeRect(rx, ry, rw, rh);
        ctx.save(); glowLine(ctx, PAL.perf, 2, 6); ctx.beginPath(); ctx.moveTo(rx + 6, ry + rh - 6); ctx.lineTo(rx + rw * 0.62, ry + 10); ctx.lineTo(rx + rw - 6, ry + 10); ctx.stroke(); ctx.restore();
        const px = rx + rw * 0.3, py = lerp(ry + rh - 6, ry + 10, (0.3 * rw - 6) / (rw * 0.62 - 6)) + 6;
        ctx.fillStyle = PAL.rob; ctx.beginPath(); ctx.arc(px, py, 4, 0, 7); ctx.fill();
        txt(ctx, "decode", px + 8, py + 4, { size: 9.5, color: PAL.rob });
        txt(ctx, "Roofline", rx + 6, ry + 14, { size: 9, color: PAL.dim });
        txt(ctx, "23.6 GB/s/socket", rx + rw - 6, ry + rh - 8, { align: "right", size: 9, color: PAL.dim });
        txt(ctx, "decode is memory-", pad, ry + 18, { size: 10, color: PAL.text });
        txt(ctx, "bandwidth bound:", pad, ry + 32, { size: 10, color: PAL.text });
        txt(ctx, "69–78% of the BW roof,", pad, ry + 50, { size: 9.5, color: PAL.muted });
        txt(ctx, "flat with more cores", pad, ry + 64, { size: 9.5, color: PAL.muted });
      }
    }
  };
};
