/* =========================================================
   Page wiring: hero background, stack index, filters, catalog,
   demo runners and the detail modal.
   ========================================================= */
(() => {
  const $ = (s, el = document) => el.querySelector(s);
  const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const catById = Object.fromEntries(CATEGORIES.map(c => [c.id, c]));
  const countOf = id => PROJECTS.filter(p => p.cat === id).length;
  // catalog reads bottom-up through the stack: transistors first, robots last
  const catalogOrder = [...CATEGORIES].reverse();

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

  let last = performance.now();
  function loop(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if (!document.hidden) runners.forEach(r => r.active && r.frame(dt));
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);

  const visObs = new IntersectionObserver(entries => entries.forEach(e => {
    const r = e.target.__runner; if (!r) return;
    r.visible = e.isIntersecting;
    if (e.isIntersecting && reduced && !r.warmed) { r.warmed = true; for (let i = 0; i < 90; i++) r.frame(0.033); }
  }), { rootMargin: "100px" });

  /* ---------------- hero: stack index ---------------- */
  const stack = $("#stack");
  CATEGORIES.forEach((c, i) => {
    const b = document.createElement("button");
    b.className = "layer"; b.setAttribute("role", "listitem"); b.style.setProperty("--accent", c.color);
    b.innerHTML = `<span class="layer-icon">${c.icon}</span><span><b>${c.name}</b><small>${c.blurb}</small></span><span class="layer-count">${countOf(c.id)}</span>`;
    b.addEventListener("click", () => { setFilter(c.id); $("#projects").scrollIntoView({ behavior: reduced ? "auto" : "smooth" }); });
    stack.appendChild(b);
  });

  /* ---------------- filters ---------------- */
  const filters = $("#filters");
  const chips = [{ id: "all", short: "All projects", color: "var(--brand)" }, ...catalogOrder].map(c => {
    const b = document.createElement("button");
    b.className = "chip"; b.setAttribute("role", "tab"); b.dataset.id = c.id; b.style.setProperty("--accent", c.color);
    b.innerHTML = `<i></i>${c.short}<span>${c.id === "all" ? PROJECTS.length : countOf(c.id)}</span>`;
    b.addEventListener("click", () => setFilter(c.id));
    filters.appendChild(b); return b;
  });
  function setFilter(id) {
    chips.forEach(ch => ch.setAttribute("aria-selected", String(ch.dataset.id === id)));
    document.querySelectorAll(".category").forEach(sec => (sec.hidden = id !== "all" && sec.dataset.cat !== id));
  }

  /* ---------------- catalog ---------------- */
  const catalog = $("#catalog");
  const revealObs = new IntersectionObserver(entries => entries.forEach(e => { if (e.isIntersecting) { e.target.classList.add("in"); revealObs.unobserve(e.target); } }), { threshold: 0.08 });
  const esc = s => String(s).replace(/[&<>"]/g, ch => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]));
  const meta = p => `<span class="course">${esc(p.course)}</span>${p.team ? `<span>· ${esc(p.team)}</span>` : ""}${p.status ? `<span class="tag-status">${esc(p.status)}</span>` : ""}`;
  const metrics = p => p.metrics.map(([v, l]) => `<div class="metric"><strong>${esc(v)}</strong><small>${esc(l)}</small></div>`).join("");

  catalogOrder.forEach((c, ci) => {
    const list = PROJECTS.filter(p => p.cat === c.id);
    const sec = document.createElement("section");
    sec.className = "category"; sec.dataset.cat = c.id; sec.style.setProperty("--accent", c.color); sec.id = `cat-${c.id}`;
    sec.innerHTML = `<div class="cat-head"><div><h3>${c.name}</h3><p>${c.blurb}</p></div><span class="cat-num">0${ci + 1} / 0${catalogOrder.length} · ${list.length} project${list.length > 1 ? "s" : ""}</span></div><div class="grid"></div>`;
    const grid = $(".grid", sec);
    list.forEach((p, k) => {
      const card = document.createElement("article");
      card.className = "card"; card.id = `p-${p.id}`; card.style.setProperty("--accent", c.color); card.style.transitionDelay = `${(k % 2) * 0.08}s`;
      card.innerHTML = `
        <div class="demo"><canvas role="img" aria-label="Animated demo: ${esc(p.title)}"></canvas>
          <span class="demo-hint mono">${esc(p.hint || "live demo")}</span></div>
        <div class="card-body">
          <div class="card-meta">${meta(p)}</div>
          <h4>${esc(p.title)}</h4>
          <p>${esc(p.purpose)}</p>
          <p><b>How:</b> ${esc(p.how)}</p>
          <div class="metrics">${metrics(p)}</div>
          <div class="card-foot"><div class="tags">${p.tags.map(t => `<span>${esc(t)}</span>`).join("")}</div>
            <button class="details-btn" aria-haspopup="dialog">Details <span aria-hidden="true">→</span></button></div>
        </div>`;
      grid.appendChild(card);
      const cv = $("canvas", card), r = new Runner(cv, p.demo);
      cv.__runner = r; visObs.observe(cv); revealObs.observe(card);
      $(".details-btn", card).addEventListener("click", () => openModal(p));
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
    const links = (p.links || []).map(([l, u]) => `<a href="${u}" target="_blank" rel="noopener">${esc(l)} ↗</a>`).join("");
    $("#modal-body").innerHTML = `
      <div class="card-meta">${meta(p)}</div>
      <h3 id="modal-title">${esc(p.title)}</h3>
      <p class="purpose">${esc(p.purpose)}</p>
      <div class="metrics">${metrics(p)}</div>
      <h5>How it works</h5>
      <ol class="steps">${p.steps.map(s => `<li>${s}</li>`).join("")}</ol>
      ${figs ? `<h5>From the project</h5><div class="figs">${figs}</div>` : ""}
      ${links ? `<h5>Watch</h5><div class="links">${links}</div>` : ""}
      <h5>Stack</h5><div class="tags">${p.tags.map(t => `<span>${esc(t)}</span>`).join("")}</div>
      ${p.note ? `<p class="note">${esc(p.note)}</p>` : ""}`;
    $("#modal-hint").textContent = p.hint || "";
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
  $("#skills").innerHTML = SKILLS.map(([n, col, items]) => `<div class="skill-group" style="--accent:${col}"><h5>${n}</h5><ul>${items.map(i => `<li>${i}</li>`).join("")}</ul></div>`).join("");

  /* ---------------- nav ---------------- */
  const nav = $("#nav");
  const onScroll = () => nav.classList.toggle("scrolled", scrollY > 20);
  addEventListener("scroll", onScroll, { passive: true }); onScroll();

  /* ---------------- hero background: routed traces with signal pulses ---------------- */
  const hc = $("#hero-bg"), hx = hc.getContext("2d");
  const cols = ["#78f0d2", "#38bdf8", "#a78bfa", "#f472b6", "#fbbf24", "#4ade80"];
  let traces = [], HW = 0, HH = 0, hdpr = 1, heroVisible = true, mouse = [-999, -999];
  function buildTraces() {
    const r = hc.getBoundingClientRect(); HW = r.width; HH = r.height; hdpr = Math.min(2, devicePixelRatio || 1);
    hc.width = HW * hdpr; hc.height = HH * hdpr;
    const g = 28, n = Math.round((HW * HH) / 16000);
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
      return { pts, cum, len: cum[cum.length - 1], s: Math.random() * cum[cum.length - 1], v: 60 + Math.random() * 120, c: cols[Math.floor(Math.random() * cols.length)] };
    });
  }
  function at(tr, s) {
    s = Math.max(0, Math.min(tr.len, s)); let i = 1; while (i < tr.cum.length - 1 && tr.cum[i] < s) i++;
    const a = tr.pts[i - 1], b = tr.pts[i], f = (s - tr.cum[i - 1]) / (tr.cum[i] - tr.cum[i - 1] || 1);
    return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
  }
  let hlast = performance.now();
  function heroFrame(now) {
    const dt = Math.min(0.05, (now - hlast) / 1000); hlast = now;
    if (heroVisible && !document.hidden) {
      hx.setTransform(hdpr, 0, 0, hdpr, 0, 0); hx.clearRect(0, 0, HW, HH);
      hx.lineWidth = 1; hx.strokeStyle = "rgba(120,240,210,0.07)"; hx.beginPath();
      traces.forEach(tr => tr.pts.forEach((p, i) => (i ? hx.lineTo(p[0], p[1]) : hx.moveTo(p[0], p[1])))); hx.stroke();
      hx.fillStyle = "rgba(120,240,210,0.18)";
      traces.forEach(tr => { const [a, b] = [tr.pts[0], tr.pts[tr.pts.length - 1]]; hx.fillRect(a[0] - 2, a[1] - 2, 4, 4); hx.fillRect(b[0] - 2, b[1] - 2, 4, 4); });
      traces.forEach(tr => {
        if (!reduced) tr.s += tr.v * dt; if (tr.s > tr.len + 60) tr.s = -20;
        const head = at(tr, tr.s), near = Math.hypot(head[0] - mouse[0], head[1] - mouse[1]) < 160;
        hx.strokeStyle = tr.c; hx.globalAlpha = near ? 1 : 0.55; hx.lineWidth = near ? 2 : 1.5; hx.shadowColor = tr.c; hx.shadowBlur = near ? 14 : 8;
        hx.beginPath(); for (let k = 0; k <= 8; k++) { const p = at(tr, tr.s - 50 + k * 6.25); k ? hx.lineTo(p[0], p[1]) : hx.moveTo(p[0], p[1]); } hx.stroke();
        hx.shadowBlur = 0; hx.globalAlpha = 1;
      });
    }
    requestAnimationFrame(heroFrame);
  }
  buildTraces(); requestAnimationFrame(heroFrame);
  new ResizeObserver(buildTraces).observe(hc);
  new IntersectionObserver(([e]) => (heroVisible = e.isIntersecting)).observe(hc);
  $(".hero").addEventListener("pointermove", e => { const r = hc.getBoundingClientRect(); mouse = [e.clientX - r.left, e.clientY - r.top]; });
})();
