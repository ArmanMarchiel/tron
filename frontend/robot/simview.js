// Simulation view: the <img> is fed by the MuJoCo MJPEG stream in live mode and by a single rendered
// frame of the twin's historical joint configuration in history mode. A tab row switches between the
// environment camera and the camera mounted on the robot's wrist. Below it, per-joint bars compare
// actual (blue) vs expected (green) joint angles.
let mode = "live", historyTs = null, view = "environment", views = [];

function applySrc() {
  const img = document.getElementById("simview");
  // the cache-buster matters on a reload: without it the <img> keeps the MJPEG connection it already
  // has, which is still streaming the previous cell
  img.src = mode === "live" ? `/api/sim/stream?view=${view}&_=${Date.now()}`
                            : `/api/sim/frame?view=${view}&ts=${historyTs}&_=${Date.now()}`;
}

let pending = { az: 0, el: 0, zoom: 1 }, flushTimer = null;
async function sendCamera(body) {
  try { await fetch("/api/sim/camera", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }); } catch (e) { /* ignore */ }
}
function queueCamera(dAz, dEl, zoom) {
  pending.az += dAz; pending.el += dEl; pending.zoom *= zoom;
  if (flushTimer) return;
  flushTimer = setTimeout(() => { const b = { d_azimuth: pending.az, d_elevation: pending.el, zoom: pending.zoom }; pending = { az: 0, el: 0, zoom: 1 }; flushTimer = null; sendCamera(b); }, 40);
}
let vpTimer = null, vpLast = "";
function reportViewport() {
  const wrap = document.getElementById("simwrap");
  if (!wrap) return;
  const r = wrap.getBoundingClientRect();
  const w = Math.round(r.width), h = Math.round(r.height);
  if (!w || !h) return;
  const key = `${w}x${h}`;
  if (key === vpLast) return;
  vpLast = key;
  clearTimeout(vpTimer);
  vpTimer = setTimeout(() => {
    fetch("/api/sim/viewport", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ width: w, height: h }) })
      .then(() => applySrc())          // reconnect so the stream carries the new size
      .catch(() => {});
  }, 200);
}

function initControls() {
  const wrap = document.getElementById("simwrap");
  const zin = document.getElementById("zoom-in"), zout = document.getElementById("zoom-out"), rst = document.getElementById("cam-reset");
  zin.onclick = () => queueCamera(0, 0, 0.8);
  zout.onclick = () => queueCamera(0, 0, 1.25);
  rst.onclick = () => sendCamera({ reset: true });
  let drag = null;
  wrap.addEventListener("mousedown", (e) => { if (e.target.tagName === "BUTTON" || view !== "environment") return; drag = { x: e.clientX, y: e.clientY }; wrap.classList.add("dragging"); e.preventDefault(); });
  window.addEventListener("mousemove", (e) => { if (!drag) return; const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag = { x: e.clientX, y: e.clientY }; queueCamera(-dx * 0.4, -dy * 0.3, 1); });
  window.addEventListener("mouseup", () => { drag = null; wrap.classList.remove("dragging"); });
  wrap.addEventListener("wheel", (e) => { if (view !== "environment") return; e.preventDefault(); queueCamera(0, 0, e.deltaY > 0 ? 1.1 : 0.9); }, { passive: false });
  wrap.addEventListener("touchstart", (e) => { if (e.touches.length === 1 && view === "environment") drag = { x: e.touches[0].clientX, y: e.touches[0].clientY }; }, { passive: true });
  wrap.addEventListener("touchmove", (e) => { if (!drag || e.touches.length !== 1) return; const t = e.touches[0]; queueCamera(-(t.clientX - drag.x) * 0.4, -(t.clientY - drag.y) * 0.3, 1); drag = { x: t.clientX, y: t.clientY }; }, { passive: true });
  wrap.addEventListener("touchend", () => { drag = null; });
}
const OVERLAY_LABELS = { robot_rom: "Robot ROM", cnc_interior: "CNC interior", collab_front: "Reduced speed", operator_side: "Separation (SSM)" };
const prettify = (n) => OVERLAY_LABELS[n] || n.replace(/_/g, " ").replace(/^./, (c) => c.toUpperCase());

export async function refreshOverlays() {
  const el = document.getElementById("overlay-toggles");
  if (!el) return;
  let state;
  try { state = await fetch("/api/sim/overlays").then(r => r.json()); } catch (e) { el.style.display = "none"; return; }
  const names = Object.keys(state.overlays || {});
  if (!names.length) { el.style.display = "none"; return; }
  const colors = state.colors || {};
  el.innerHTML = `<span class="ov-title">Zones</span>` + names.map(n =>
    `<label><input type="checkbox" data-overlay="${n}"${state.overlays[n] ? " checked" : ""}>` +
    `<i class="sw" style="background:${colors[n] || "#888"}"></i>${prettify(n)}</label>`).join("");
  el.querySelectorAll("input").forEach(cb => cb.onchange = () => {
    fetch("/api/sim/overlays", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: cb.dataset.overlay, visible: cb.checked }) }).catch(() => {});
  });
}

function updateControls() {
  const wrap = document.getElementById("simwrap"), env = view === "environment";
  wrap.classList.toggle("fixed", !env);
  document.getElementById("viz-controls").style.display = env ? "" : "none";
  document.getElementById("viz-hint").textContent = env ? "drag to orbit · scroll to zoom" : "camera mounted on the wrist";
}

export async function initSimView(tabsEl) {
  initControls();
  refreshOverlays();
  reportViewport();
  window.addEventListener("resize", reportViewport);
  try { views = await fetch("/api/sim/views").then(r => r.json()); } catch (e) { views = [{ id: "environment", label: "Environment" }]; }
  tabsEl.innerHTML = views.map(v => `<button data-view="${v.id}" class="${v.id === view ? "active" : ""}">${v.label}</button>`).join("");
  tabsEl.querySelectorAll("button").forEach(b => b.onclick = () => {
    view = b.dataset.view;
    tabsEl.querySelectorAll("button").forEach(x => x.classList.toggle("active", x === b));
    applySrc(); updateControls();
  });
  updateControls();
}

export function setSimMode(m, ts, force = false) {
  if (!force && m === mode && m === "live") return;
  mode = m; historyTs = ts ?? null;
  applySrc();
}

