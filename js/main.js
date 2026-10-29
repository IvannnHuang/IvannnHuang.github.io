/* =========================================================
   Page wiring: timing-diagram header, index table, filters,
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
  let last = performance.now();
  function loop(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if (!document.hidden) { runners.forEach(r => r.active && r.frame(dt)); frameHooks.forEach(f => f(dt)); }
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
    wx.strokeStyle = INK; wx.lineWidth = 1.4; wx.lineJoin = "miter";
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
  frameHooks.push(dt => { if (!waveVisible) return; if (!reduced && hoverX < 0) woff += dt * 24; drawWave(); });
  wv.addEventListener("pointermove", e => { hoverX = e.clientX - wv.getBoundingClientRect().left; wv.style.cursor = hoverX > LABW ? "pointer" : "default"; });
  wv.addEventListener("pointerleave", () => (hoverX = -1));
  wv.addEventListener("click", e => {
    const x = e.clientX - wv.getBoundingClientRect().left; if (x <= LABW) return;
    goTo(segCat(Math.floor((woff + x - LABW) / SL)).id);
  });

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
          <figcaption class="mono"><span><span class="pn">Fig. ${partNo[p.id]}</span> — ${esc(p.hint || "live demo")}</span><span>live</span></figcaption>
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
})();
