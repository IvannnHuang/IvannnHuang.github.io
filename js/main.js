/* =========================================================
   Page wiring: hero trace background, timing-diagram header, index table, filters,
   catalog, demo runners and the detail modal.
   ========================================================= */
(() => {
  const $ = (s, el = document) => el.querySelector(s);
  const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const catById = Object.fromEntries(CATEGORIES.map(c => [c.id, c]));
  const countOf = id => PROJECTS.filter(p => p.cat === id).length;
  // catalog reads bottom-up through the stack: transistors first, robots last
  const catalogOrder = [...CATEGORIES].reverse();
  const CODE = { silicon: "S", fpga: "F", embedded: "E", perf: "P", robotics: "R" };
  const partNo = {};
  catalogOrder.forEach(c => PROJECTS.filter(p => p.cat === c.id).forEach((p, i) => (partNo[p.id] = `${CODE[c.id]}-${String(i + 1).padStart(2, "0")}`)));
  const esc = s => String(s).replace(/[&<>"]/g, ch => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]));

  $("#year").textContent = new Date().getFullYear();
  $("#stat-projects").textContent = PROJECTS.length;

  /* ---------------- demo runners ---------------- */
  const runners = new Set();
  let modalOpen = false;

  class Runner {
    constructor(canvas, name, { inModal = false } = {}) {
      this.canvas = canvas; this.ctx = canvas.getContext("2d"); this.name = name; this.inModal = inModal;
      this.env = { ctx: this.ctx, w: 0, h: 0 }; this.visible = inModal; this.hover = false; this.inst = null; this.dpr = 1;
      this.ro = new ResizeObserver(() => this.resize()); this.ro.observe(canvas);
      canvas.addEventListener("click", e => { const r = canvas.getBoundingClientRect(); this.inst?.click?.(e.clientX - r.left, e.clientY - r.top); });
      canvas.addEventListener("pointerenter", () => (this.hover = true));
      canvas.addEventListener("pointerleave", () => (this.hover = false));
      runners.add(this);
    }
    resize() {
      const r = this.canvas.getBoundingClientRect(); if (r.width < 2 || r.height < 2) return;
      this.dpr = Math.min(2, window.devicePixelRatio || 1);
      this.canvas.width = Math.round(r.width * this.dpr); this.canvas.height = Math.round(r.height * this.dpr);
      this.env.w = r.width; this.env.h = r.height;
      if (!this.inst && DEMOS[this.name]) this.inst = DEMOS[this.name](this.env);
      this.frame(0.016); // repaint immediately after the canvas is cleared by resizing
    }
    get active() {
      if (!this.inst || this.dead) return false;
      if (this.inModal) return modalOpen;
      return this.visible && !modalOpen && (!reduced || this.hover);
    }
    frame(dt) {
      if (!this.inst || this.dead) return;
      this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      try { this.inst.draw(dt); } catch (err) { console.error(`demo "${this.name}" failed`, err); this.dead = true; }
    }
    destroy() { this.ro.disconnect(); runners.delete(this); }
  }

  const frameHooks = [];
  let last = performance.now(), clockMul = 1; // clockMul > 1 = overclock easter egg: demos step several times per frame
  function loop(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if (!document.hidden) {
      for (let s = 0; s < clockMul; s++) runners.forEach(r => r.active && r.frame(dt));
      frameHooks.forEach(f => f(dt));
    }
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);

  const visObs = new IntersectionObserver(entries => entries.forEach(e => {
    const r = e.target.__runner; if (!r) return;
    r.visible = e.isIntersecting;
    if (e.isIntersecting && reduced && !r.warmed) { r.warmed = true; for (let i = 0; i < 90; i++) r.frame(0.033); }
  }), { rootMargin: "100px" });

  const goTo = id => { setFilter(id); $("#projects").scrollIntoView({ behavior: reduced ? "auto" : "smooth" }); };

  /* ---------------- hero: index table ---------------- */
  const toc = $("#toc");
  catalogOrder.forEach((c, i) => {
    const li = document.createElement("li");
    li.tabIndex = 0; li.setAttribute("role", "link"); li.style.setProperty("--accent", c.color);
    li.innerHTML = `<span class="num">${String(i + 1).padStart(2, "0")}</span>
      <span class="name"><b>${c.name}</b><small>${c.blurb}</small></span>
      <span class="count">${countOf(c.id)} projects</span><span class="go" aria-hidden="true">→</span>`;
    li.addEventListener("click", () => goTo(c.id));
    li.addEventListener("keydown", e => { if (e.key === "Enter") goTo(c.id); });
    toc.appendChild(li);
  });

  /* ---------------- hero: timing diagram ---------------- */
  const wv = $("#wave"), wx = wv.getContext("2d");
  const P = 26, SEG = 8, SL = P * SEG, LABW = 104;
  const INK = "#ecebe6", RULE = "#262624", RULE2 = "#3b3a36", DIM = "#6d6b64", MARK = "#ff5a36", SHEET = "#141413", PAPER = "#0e0e0d";
  const catHex = { silicon: "#a693ff", fpga: "#f26d9b", embedded: "#4cc98a", perf: "#5aa7ff", robotics: "#e9b54a" };
  let WW = 0, WH = 0, wdpr = 1, woff = SL * 0.35, hoverX = -1, waveVisible = true;
  const segCat = k => catalogOrder[((k % catalogOrder.length) + catalogOrder.length) % catalogOrder.length];
  function wresize() {
    const r = wv.getBoundingClientRect(); WW = r.width; WH = r.height; wdpr = Math.min(2, devicePixelRatio || 1);
    wv.width = Math.round(WW * wdpr); wv.height = Math.round(WH * wdpr); drawWave();
  }
  function digital(y, hiOf) {
    // step waveform; hiOf(t) says whether the signal is high at time t (evaluated per half clock)
    const hi = y - 9, lo = y + 9, H = P / 2;
    let t = Math.floor(woff / H) * H, lvl = hiOf(t) ? hi : lo;
    wx.beginPath(); wx.moveTo(LABW, lvl);
    for (t += H; LABW + t - woff < WW + H; t += H) {
      const x = Math.round(LABW + t - woff) + 0.5, n = hiOf(t) ? hi : lo;
      if (n !== lvl) { wx.lineTo(x, lvl); wx.lineTo(x, n); lvl = n; }
    }
    wx.lineTo(WW, lvl); wx.stroke();
  }
  function bus(y, label, hoverK) {
    const hh = 10, k0 = Math.floor(woff / SL), k1 = Math.floor((woff + WW - LABW) / SL);
    for (let k = k0; k <= k1; k++) {
      const c = segCat(k), xs = LABW + k * SL - woff, xe = xs + SL, d = 5;
      wx.beginPath(); wx.moveTo(xs + d, y - hh); wx.lineTo(xe - d, y - hh); wx.lineTo(xe, y); wx.lineTo(xe - d, y + hh); wx.lineTo(xs + d, y + hh); wx.lineTo(xs, y); wx.closePath();
      wx.fillStyle = k === hoverK ? catHex[c.id] + "40" : catHex[c.id] + "1a"; wx.fill(); wx.stroke();
      const vx0 = Math.max(xs, LABW), vx1 = Math.min(xe, WW);
      if (vx1 - vx0 > 46) {
        wx.fillStyle = catHex[c.id]; wx.font = '600 11px "JetBrains Mono", monospace'; wx.textAlign = "center"; wx.textBaseline = "middle";
        wx.fillText(label(c), (vx0 + vx1) / 2, y + 0.5);
      }
    }
  }
  function drawWave() {
    if (!WW) return;
    wx.setTransform(wdpr, 0, 0, wdpr, 0, 0);
    wx.fillStyle = SHEET; wx.fillRect(0, 0, WW, WH);
    const rows = ["clk", "area[2:0]", "valid", "count[4:0]"], rh = (WH - 8) / rows.length, ys = rows.map((_, i) => 4 + rh * i + rh / 2);
    // rising-edge gridlines
    wx.fillStyle = RULE;
    for (let t = Math.ceil(woff / P) * P; LABW + t - woff < WW; t += P) wx.fillRect(Math.round(LABW + t - woff), 0, 1, WH);
    // label column
    wx.fillStyle = PAPER; wx.fillRect(0, 0, LABW, WH); wx.fillStyle = RULE2; wx.fillRect(LABW, 0, 1, WH);
    wx.font = '500 11px "JetBrains Mono", monospace'; wx.textAlign = "right"; wx.textBaseline = "middle";
    rows.forEach((r, i) => { wx.fillStyle = i === 1 ? MARK : DIM; wx.fillText(r, LABW - 12, ys[i]); });
    wx.save(); wx.beginPath(); wx.rect(LABW + 1, 0, WW - LABW, WH); wx.clip();
    wx.strokeStyle = INK; wx.lineWidth = 1.4; wx.lineJoin = "miter"; wx.shadowColor = "rgba(236,235,230,0.35)"; wx.shadowBlur = 4;
    const hoverK = hoverX > LABW ? Math.floor((woff + hoverX - LABW) / SL) : null;
    digital(ys[0], t => Math.floor(t / (P / 2)) % 2 === 0);
    bus(ys[1], c => c.short.toUpperCase(), hoverK);
    digital(ys[2], t => { const r = ((t % SL) + SL) % SL; return r >= P && r < SL - P; });
    bus(ys[3], c => String(countOf(c.id)), hoverK);
    if (hoverX > LABW) { wx.strokeStyle = MARK; wx.lineWidth = 1; wx.setLineDash([3, 3]); wx.beginPath(); wx.moveTo(Math.round(hoverX) + 0.5, 0); wx.lineTo(Math.round(hoverX) + 0.5, WH); wx.stroke(); wx.setLineDash([]); }
    wx.restore();
  }
  new ResizeObserver(wresize).observe(wv);
  new IntersectionObserver(([e]) => (waveVisible = e.isIntersecting)).observe(wv);
  frameHooks.push(dt => { if (!waveVisible) return; if (!reduced && hoverX < 0) woff += dt * 24 * clockMul; drawWave(); });
  wv.addEventListener("pointermove", e => { hoverX = e.clientX - wv.getBoundingClientRect().left; wv.style.cursor = hoverX > LABW ? "pointer" : "default"; });
  wv.addEventListener("pointerleave", () => (hoverX = -1));
  wv.addEventListener("click", e => {
    const x = e.clientX - wv.getBoundingClientRect().left; if (x <= LABW) return;
    goTo(segCat(Math.floor((woff + x - LABW) / SL)).id);
  });

  /* ---------------- hero background: routed traces with signal pulses ---------------- */
  // from the first version of the site, re-inked in the current palette; pulses brighten near the pointer
  const hc = $("#hero-bg"), hx = hc.getContext("2d"), traceCols = [MARK, ...Object.values(catHex)];
  let traces = [], HW = 0, HH = 0, hdpr = 1, heroVisible = true, mouse = [-999, -999];
  function buildTraces() {
    const r = hc.getBoundingClientRect(); HW = r.width; HH = r.height; hdpr = Math.min(1.5, devicePixelRatio || 1);
    hc.width = Math.round(HW * hdpr); hc.height = Math.round(HH * hdpr);
    const g = 28, n = Math.min(170, Math.round((HW * HH) / 15000));
    traces = Array.from({ length: n }, () => {
      let x = Math.round(Math.random() * HW / g) * g, y = Math.round(Math.random() * HH / g) * g, horiz = Math.random() < 0.5;
      const pts = [[x, y]];
      for (let s = 0; s < 3 + Math.floor(Math.random() * 5); s++) {
        const len = g * (2 + Math.floor(Math.random() * 6)) * (Math.random() < 0.5 ? -1 : 1);
        if (horiz) x += len; else y += len; pts.push([x, y]);
        if (Math.random() < 0.5) { const d = g * (Math.random() < 0.5 ? -1 : 1); x += d; y += d; pts.push([x, y]); } // 45° jog
        horiz = !horiz;
      }
      const cum = [0]; for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
      return { pts, cum, len: cum[cum.length - 1], s: Math.random() * cum[cum.length - 1], v: 60 + Math.random() * 120, c: traceCols[Math.floor(Math.random() * traceCols.length)] };
    });
    drawTraces(0);
  }
  function at(tr, s) {
    s = Math.max(0, Math.min(tr.len, s)); let i = 1; while (i < tr.cum.length - 1 && tr.cum[i] < s) i++;
    const a = tr.pts[i - 1], b = tr.pts[i], f = (s - tr.cum[i - 1]) / (tr.cum[i] - tr.cum[i - 1] || 1);
    return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
  }
  function drawTraces(dt) {
    hx.setTransform(hdpr, 0, 0, hdpr, 0, 0); hx.clearRect(0, 0, HW, HH);
    hx.lineWidth = 1; hx.strokeStyle = "rgba(236,235,230,0.07)"; hx.beginPath();
    traces.forEach(tr => tr.pts.forEach((p, i) => (i ? hx.lineTo(p[0], p[1]) : hx.moveTo(p[0], p[1])))); hx.stroke();
    hx.fillStyle = "rgba(236,235,230,0.18)";
    traces.forEach(tr => { const a = tr.pts[0], b = tr.pts[tr.pts.length - 1]; hx.fillRect(a[0] - 2, a[1] - 2, 4, 4); hx.fillRect(b[0] - 2, b[1] - 2, 4, 4); });
    traces.forEach(tr => {
      if (!reduced) tr.s += tr.v * dt * clockMul;
      if (tr.s > tr.len + 60) tr.s = -20;
      const head = at(tr, tr.s), near = Math.hypot(head[0] - mouse[0], head[1] - mouse[1]) < 160;
      hx.strokeStyle = tr.c; hx.globalAlpha = near ? 1 : 0.55; hx.lineWidth = near ? 2 : 1.5; hx.shadowColor = tr.c; hx.shadowBlur = near ? 14 : 8;
      hx.beginPath(); for (let k = 0; k <= 8; k++) { const p = at(tr, tr.s - 50 + k * 6.25); k ? hx.lineTo(p[0], p[1]) : hx.moveTo(p[0], p[1]); } hx.stroke();
      hx.shadowBlur = 0; hx.globalAlpha = 1;
    });
  }
  new ResizeObserver(buildTraces).observe(hc);
  new IntersectionObserver(([e]) => (heroVisible = e.isIntersecting)).observe(hc);
  frameHooks.push(dt => heroVisible && drawTraces(dt));
  $(".hero").addEventListener("pointermove", e => { const r = hc.getBoundingClientRect(); mouse = [e.clientX - r.left, e.clientY - r.top]; });
  $(".hero").addEventListener("pointerleave", () => (mouse = [-999, -999]));

  /* ---------------- filters ---------------- */
  const filters = $("#filters");
  const tabs = [{ id: "all", short: "All", color: "var(--ink)" }, ...catalogOrder].map(c => {
    const b = document.createElement("button");
    b.className = "tab"; b.setAttribute("role", "tab"); b.dataset.id = c.id; b.style.setProperty("--accent", c.color);
    b.innerHTML = `${c.id === "all" ? "" : "<i></i>"}${c.short}<span>${c.id === "all" ? PROJECTS.length : countOf(c.id)}</span>`;
    b.addEventListener("click", () => setFilter(c.id));
    filters.appendChild(b); return b;
  });
  function setFilter(id) {
    tabs.forEach(t => t.setAttribute("aria-selected", String(t.dataset.id === id)));
    document.querySelectorAll(".category").forEach(sec => (sec.hidden = id !== "all" && sec.dataset.cat !== id));
  }

  /* ---------------- catalog ---------------- */
  const catalog = $("#catalog");
  const meta = p => `<span class="pn-box">${partNo[p.id]}</span><span>${esc(p.course)}</span>${p.team ? `<span>· ${esc(p.team)}</span>` : ""}${p.status ? `<span class="tag-status">${esc(p.status)}</span>` : ""}`;
  const specs = p => `<div class="specs">${p.metrics.map(([v, l]) => `<div><strong>${esc(v)}</strong><small>${esc(l)}</small></div>`).join("")}</div>`;
  const tagList = p => p.tags.map(t => `<span>${esc(t)}</span>`).join("");

  catalogOrder.forEach((c, ci) => {
    const list = PROJECTS.filter(p => p.cat === c.id);
    const sec = document.createElement("section");
    sec.className = "category"; sec.dataset.cat = c.id; sec.style.setProperty("--accent", c.color); sec.id = `cat-${c.id}`;
    sec.innerHTML = `<div class="cat-head"><span class="num">${String(ci + 1).padStart(2, "0")}</span><div><h3>${c.name}</h3><p>${c.blurb}</p></div><span class="cat-num">${CODE[c.id]}-01 … ${CODE[c.id]}-${String(list.length).padStart(2, "0")}</span></div><div class="grid"></div>`;
    const grid = $(".grid", sec);
    list.forEach(p => {
      const card = document.createElement("article");
      card.className = "card"; card.id = `p-${p.id}`; card.style.setProperty("--accent", c.color);
      card.innerHTML = `
        <figure class="fig">
          <div class="demo"><canvas role="img" aria-label="Animated demo: ${esc(p.title)}"></canvas></div>
          <figcaption class="mono"><span><span class="pn">Fig. ${partNo[p.id]}</span> — ${esc(p.hint || "live demo")}</span><span class="live">live</span></figcaption>
        </figure>
        <div class="card-body">
          <div class="card-meta">${meta(p)}</div>
          <h4>${esc(p.title)}</h4>
          <p>${esc(p.purpose)}</p>
          <p class="how"><b>Approach.</b> ${esc(p.how)}</p>
          ${specs(p)}
          <div class="card-foot"><div class="tags">${tagList(p)}</div>
            <div class="card-actions">
              ${p.repo ? `<a class="details-btn code-btn" href="${p.repo}" target="_blank" rel="noopener">Code ↗</a>` : ""}
              <button class="details-btn" aria-haspopup="dialog">Details →</button>
            </div></div>
        </div>`;
      grid.appendChild(card);
      const cv = $("canvas", card), r = new Runner(cv, p.demo);
      cv.__runner = r; visObs.observe(cv);
      $("button.details-btn", card).addEventListener("click", () => openModal(p));
    });
    catalog.appendChild(sec);
  });
  setFilter("all");

  /* ---------------- modal ---------------- */
  const modal = $("#modal");
  let modalRunner = null;
  function openModal(p) {
    const c = catById[p.cat];
    modal.style.setProperty("--accent", c.color);
    const figs = (p.figs || []).map(([src, cap, dark]) => `<figure class="${dark ? "dark" : ""}"><img src="assets/img/${src}" alt="${esc(cap)}" loading="lazy"><figcaption>${esc(cap)}</figcaption></figure>`).join("");
    const links = [...(p.repo ? [["GitHub repository", p.repo]] : []), ...(p.links || [])]
      .map(([l, u]) => `<a href="${u}" target="_blank" rel="noopener">${esc(l)} ↗</a>`).join("");
    $("#modal-body").innerHTML = `
      <div class="card-meta">${meta(p)}</div>
      <h3 id="modal-title">${esc(p.title)}</h3>
      <p class="purpose">${esc(p.purpose)}</p>
      ${specs(p)}
      <h5>How it works</h5>
      <ol class="steps">${p.steps.map(s => `<li><span>${s}</span></li>`).join("")}</ol>
      ${figs ? `<h5>From the project</h5><div class="figs">${figs}</div>` : ""}
      ${links ? `<h5>Links</h5><div class="links">${links}</div>` : ""}
      <h5>Stack</h5><div class="tags">${tagList(p)}</div>
      ${p.note ? `<p class="note">${esc(p.note)}</p>` : ""}`;
    $("#modal-hint").innerHTML = `<span><span class="pn" style="color:${c.color}">Fig. ${partNo[p.id]}</span> — ${esc(p.hint || "")}</span><span>live</span>`;
    modalRunner?.destroy();
    const old = $("#modal-canvas"), fresh = old.cloneNode(false); old.replaceWith(fresh);
    modalOpen = true;
    modal.showModal();
    modalRunner = new Runner(fresh, p.demo, { inModal: true });
    $("#modal-body").scrollTop = 0;
  }
  modal.addEventListener("close", () => { modalOpen = false; modalRunner?.destroy(); modalRunner = null; });
  $("#modal-close").addEventListener("click", () => modal.close());
  modal.addEventListener("click", e => { if (e.target === modal) modal.close(); });

  /* ---------------- skills ---------------- */
  $("#skills").innerHTML = `<tbody>${SKILLS.map(([n, col, items]) => `<tr style="--accent:${col}"><th class="mono">${n}</th><td><ul>${items.map(i => `<li>${i}</li>`).join("")}</ul></td></tr>`).join("")}</tbody>`;

  /* ---------------- hero: name decodes from hex on load ---------------- */
  const h1 = $(".hero h1"), NAME = h1.textContent;
  if (!reduced) {
    h1.setAttribute("aria-label", NAME);
    const HEX = "0123456789ABCDEF";
    let t0 = -1;
    requestAnimationFrame(function tick(now) {
      if (t0 < 0) t0 = now;
      const t = now - t0; let out = "", done = true;
      for (let i = 0; i < NAME.length; i++) {
        if (NAME[i] === " " || t > 260 + i * 60) out += esc(NAME[i]);
        else { out += `<span class="scr">${HEX[(Math.random() * 16) | 0]}</span>`; done = false; }
      }
      h1.innerHTML = out;
      if (!done) requestAnimationFrame(tick);
    });
  }

  /* ---------------- hero: probe cursor with zone readout ---------------- */
  const sheet = $(".sheet"), probe = $("#probe"), probeTag = $(".probe-tag", probe), zones = [...document.querySelectorAll(".zones span")];
  if (matchMedia("(hover: hover) and (pointer: fine)").matches) {
    sheet.addEventListener("pointermove", e => {
      const r = sheet.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top, col = Math.min(5, Math.floor((x / r.width) * 6));
      probe.style.setProperty("--px", `${x}px`); probe.style.setProperty("--py", `${y}px`);
      probeTag.textContent = `${"ABCDEF"[col]}${Math.min(3, Math.floor((y / r.height) * 4)) + 1} · ${Math.round(x)}, ${Math.round(y)}`;
      probeTag.classList.toggle("flip", x > r.width - 160);
      probe.classList.toggle("on", !e.target.closest(".wave")); // the timing diagram has its own cursor
      zones.forEach((z, i) => z.classList.toggle("hot", i === col));
    });
    sheet.addEventListener("pointerleave", () => { probe.classList.remove("on"); zones.forEach(z => z.classList.remove("hot")); });
  }

  /* ---------------- cards: reveal on scroll, spec values count up ---------------- */
  if (!reduced) {
    const revealObs = new IntersectionObserver(entries => entries.forEach(e => {
      if (e.isIntersecting) { e.target.classList.add("in"); revealObs.unobserve(e.target); }
    }), { rootMargin: "0px 0px -6% 0px" });
    document.querySelectorAll(".card, .section-head, .cat-head").forEach(el => { el.classList.add("rv"); revealObs.observe(el); });

    const countObs = new IntersectionObserver(entries => entries.forEach(e => {
      if (!e.isIntersecting) return;
      countObs.unobserve(e.target);
      e.target.querySelectorAll("strong").forEach(el => {
        const full = el.textContent, m = full.match(/^(\D*?)(\d+(?:\.\d+)?)(.*)$/);
        if (!m || m[3][0] === ",") return; // leave non-numeric and thousands-separated values alone
        const [, pre, num, post] = m, v = parseFloat(num), dec = (num.split(".")[1] || "").length, t0 = performance.now();
        (function step(now) {
          const f = Math.min(1, (now - t0) / 900), k = 1 - Math.pow(1 - f, 3);
          el.textContent = f < 1 ? pre + (v * k).toFixed(dec) + post : full;
          if (f < 1) requestAnimationFrame(step);
        })(t0);
      });
    }), { threshold: 0.6 });
    document.querySelectorAll(".card .specs").forEach(s => countObs.observe(s));
  }

  /* ---------------- toast + overclock easter egg ---------------- */
  const toastEl = $("#toast");
  let toastT = 0;
  function toast(msg) {
    toastEl.textContent = msg; toastEl.classList.add("show");
    clearTimeout(toastT); toastT = setTimeout(() => toastEl.classList.remove("show"), 2400);
  }
  function setOverclock(on) {
    clockMul = on ? 2 : 1;
    document.body.classList.toggle("oc", on); $("#sb-oc").hidden = !on;
    toast(on ? "Overclocked: every demo now runs at 2× clock" : "Clock restored to 1×");
  }
  const KONAMI = ["arrowup", "arrowup", "arrowdown", "arrowdown", "arrowleft", "arrowright", "arrowleft", "arrowright", "b", "a"];
  let kpos = 0;

  /* ---------------- command palette ---------------- */
  const pal = $("#palette"), palQ = $("#pal-q"), palList = $("#pal-list");
  const EMAIL = "qh229@cornell.edu";
  const isMac = /Mac|iPhone|iPad/.test(navigator.platform);
  document.querySelectorAll(".kbd-mod").forEach(k => (k.textContent = isMac ? "⌘" : "Ctrl"));
  const jump = sel => $(sel).scrollIntoView({ behavior: reduced ? "auto" : "smooth" });
  const COMMANDS = [
    ...[["#top", "Top of the sheet", "§0"], ["#projects", "Projects", "§1"], ["#about", "About", "§2"], ["#contact", "Contact", "§3"]]
      .map(([sel, label, hint]) => ({ group: "Go to", label, hint, run: () => jump(sel) })),
    ...catalogOrder.map(c => ({ group: "Filter", label: c.name, hint: `${countOf(c.id)} projects`, color: c.color, keys: c.short, run: () => goTo(c.id) })),
    ...catalogOrder.flatMap(c => PROJECTS.filter(p => p.cat === c.id).map(p => ({
      group: "Open", label: p.title, hint: partNo[p.id], color: c.color, keys: `${c.short} ${p.course} ${p.tags.join(" ")}`, run: () => openModal(p)
    }))),
    { group: "Do", label: "Copy email address", hint: EMAIL, keys: "contact mail", run: () => {
      const w = navigator.clipboard?.writeText(EMAIL);
      w ? w.then(() => toast(`Copied ${EMAIL}`), () => toast(EMAIL)) : toast(EMAIL);
    } },
    { group: "Do", label: "Open GitHub", hint: "↗", run: () => window.open("https://github.com/IvannnHuang", "_blank", "noopener") },
    { group: "Do", label: "Open LinkedIn", hint: "↗", run: () => window.open("https://linkedin.com/in/ivan-huang-4a02a7246", "_blank", "noopener") },
    { group: "Do", label: "Toggle overclock", hint: "2× demo clock", keys: "turbo speed fast", run: () => setOverclock(clockMul === 1) }
  ];
  function score(c, q) {
    if (!q) return 1;
    const hay = `${c.label} ${c.hint} ${c.keys || ""}`.toLowerCase(), i = hay.indexOf(q);
    if (i >= 0) return 200 - Math.min(i, 100) + (i === 0 || hay[i - 1] === " " ? 100 : 0);
    // fall back to a tight in-order subsequence match on the label only ("smthwtr" → Smith-Waterman)
    const label = c.label.toLowerCase();
    let j = 0, gap = 0;
    for (const ch of label) {
      if (ch === q[j]) { gap = 0; if (++j === q.length) return 10; }
      else if (j && ++gap > 3) return 0;
    }
    return 0;
  }
  let palItems = [], palSel = 0;
  function palRender() {
    const q = palQ.value.trim().toLowerCase();
    palItems = COMMANDS.map(c => [c, score(c, q)]).filter(([, s]) => s > 0).sort((a, b) => b[1] - a[1]).map(([c]) => c);
    palSel = 0;
    let group = "";
    palList.innerHTML = palItems.length ? palItems.map((c, i) => {
      const head = !q && c.group !== group ? `<li class="pal-group" role="presentation">${(group = c.group)}</li>` : "";
      return `${head}<li role="option" id="pal-${i}" data-i="${i}" aria-selected="${i === 0}" style="--accent:${c.color || "var(--dim)"}">
        <i></i><span class="pal-label">${q ? `<em>${c.group}</em>` : ""}${esc(c.label)}</span><span class="pal-hint">${esc(c.hint)}</span></li>`;
    }).join("") : `<li class="pal-empty">No matches. Try “fpga”, “risc-v” or “email”.</li>`;
    palQ.setAttribute("aria-activedescendant", palItems.length ? "pal-0" : "");
  }
  function palMove(i) {
    if (!palItems.length) return;
    palSel = (i + palItems.length) % palItems.length;
    palList.querySelectorAll("[role=option]").forEach(li => li.setAttribute("aria-selected", String(+li.dataset.i === palSel)));
    $(`#pal-${palSel}`).scrollIntoView({ block: "nearest" });
    palQ.setAttribute("aria-activedescendant", `pal-${palSel}`);
  }
  function palRun(i) { const c = palItems[i]; if (!c) return; pal.close(); c.run(); }
  function palOpen() {
    if (modal.open) modal.close();
    palQ.value = ""; palRender(); pal.showModal(); palQ.focus();
  }
  palQ.addEventListener("input", palRender);
  palQ.addEventListener("keydown", e => {
    if (e.key === "ArrowDown" || e.key === "ArrowUp") { e.preventDefault(); palMove(palSel + (e.key === "ArrowDown" ? 1 : -1)); }
    else if (e.key === "Enter") { e.preventDefault(); palRun(palSel); }
  });
  palList.addEventListener("click", e => { const li = e.target.closest("[data-i]"); if (li) palRun(+li.dataset.i); });
  palList.addEventListener("pointermove", e => { const li = e.target.closest("[data-i]"); if (li && +li.dataset.i !== palSel) palMove(+li.dataset.i); });
  pal.addEventListener("click", e => { if (e.target === pal) pal.close(); });
  $("#cmd-btn").addEventListener("click", palOpen);
  document.addEventListener("keydown", e => {
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === "k") { e.preventDefault(); pal.open ? pal.close() : palOpen(); return; }
    const el = document.activeElement, typing = el && (/^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName) || el.isContentEditable);
    if (typing) return;
    if (e.key === "/" && !pal.open) { e.preventDefault(); palOpen(); return; }
    const k = e.key.toLowerCase();
    kpos = k === KONAMI[kpos] ? kpos + 1 : k === KONAMI[0] ? 1 : 0;
    if (kpos === KONAMI.length) { kpos = 0; setOverclock(clockMul === 1); }
  });

  /* ---------------- status bar, scroll progress, active section ---------------- */
  const progress = $("#nav-progress"), navLinks = [...document.querySelectorAll(".nav-links a")];
  const SECTIONS = [["top", "§0 Sheet"], ["projects", "§1 Projects"], ["about", "§2 About"], ["contact", "§3 Contact"]].map(([id, label]) => ({ id, label, el: $(`#${id}`) }));
  let curSec = null;
  function onScroll() {
    const max = document.documentElement.scrollHeight - innerHeight;
    progress.style.transform = `scaleX(${max > 0 ? Math.min(1, scrollY / max) : 0})`;
    let cur = SECTIONS[0];
    SECTIONS.forEach(s => { if (s.el.getBoundingClientRect().top < innerHeight * 0.35) cur = s; });
    if (cur !== curSec) {
      curSec = cur; $("#sb-sec").textContent = cur.label;
      navLinks.forEach(a => (a.getAttribute("href") === `#${cur.id}` ? a.setAttribute("aria-current", "location") : a.removeAttribute("aria-current")));
    }
    $("#sb-pc").textContent = `PC 0x${(Math.round(scrollY / 4) * 4).toString(16).toUpperCase().padStart(5, "0")}`;
  }
  addEventListener("scroll", onScroll, { passive: true });
  addEventListener("resize", onScroll);
  onScroll();
  const sbCyc = $("#sb-cyc"), sbLive = $("#sb-live"), sbFps = $("#sb-fps");
  let cycles = 0, fps = 60, sbT = 0;
  frameHooks.push(dt => {
    cycles += dt * 25e6 * clockMul; fps += (1 / Math.max(dt, 1e-3) - fps) * 0.05;
    if ((sbT += dt) < 0.2) return;
    sbT = 0;
    let live = 0; runners.forEach(r => r.active && !r.inModal && live++);
    sbCyc.textContent = Math.floor(cycles).toLocaleString("en-US");
    sbLive.textContent = `${live}/${PROJECTS.length}`;
    sbFps.textContent = Math.round(fps);
  });
})();
