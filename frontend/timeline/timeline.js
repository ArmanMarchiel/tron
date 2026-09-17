// Time Series tab: per-joint small multiples (expected vs actual angle, divergence shading), end-effector
// and encoder error, peak joint speed vs limit, risk band with markers — all on one shared time axis with
// one scrubber. Click / drag to reconstruct the twin at that instant; double-click returns live.
import { fmtTs, api } from "../app.js";
let canvas, ctx, onScrub, onLive, mode = "live", cursorTs = null, data = null, windowS = 120, robotId = "robot-001", joints = [], limits = {};
const fmtClock = (ts) => fmtTs(ts).slice(0, 8);
const C = { act: "#1f6feb", exp: "#1a7f37", rep: "#7c3aed", err: "#b45309", bad: "#c81e1e", grid: "#e6ebf1", text: "#6b7785", fault: "#7c3aed", step: "#cbd5e1" };

export function initTimeline(el, scrub, live) {
  onScrub = scrub; onLive = live;
  el.innerHTML = `<h2>Time series <span class="dim">click / drag to reconstruct the twin at that instant · double-click returns live</span></h2>
    <canvas id="tl" width="1000" height="900"></canvas>
    <div class="tl-bar">
      <span>window:</span>
      <button class="small" data-w="60">60 s</button><button class="small" data-w="120">2 min</button><button class="small" data-w="300">5 min</button><button class="small" data-w="900">15 min</button>
      <span id="tl-cursor" class="dim"></span>
      <span class="spacer"></span>
      <span><i class="sw" style="background:${C.act}"></i>actual</span><span><i class="sw" style="background:${C.exp}"></i>expected</span>
      <span><i class="sw" style="background:${C.rep}"></i>reported</span><span><i class="sw" style="background:${C.err}"></i>divergence</span>
      <span><i class="sw" style="background:${C.bad}"></i>anomaly / incident</span><span><i class="sw" style="background:${C.fault}"></i>fault injected</span>
    </div>`;
  canvas = el.querySelector("#tl"); ctx = canvas.getContext("2d");
  el.querySelectorAll("button[data-w]").forEach(b => b.onclick = () => { windowS = +b.dataset.w; refresh(); });
  let dragging = false;
  const pick = (ev) => { if (!data) return; const r = canvas.getBoundingClientRect(); const x = (ev.clientX - r.left) / r.width; const ts = data.since + Math.max(0, Math.min(1, (x - LFRAC()) / (1 - LFRAC() - RFRAC()))) * (data.until - data.since); cursorTs = ts; draw(); onScrub(ts); };
  canvas.onmousedown = (e) => { dragging = true; pick(e); };
  canvas.onmousemove = (e) => { if (dragging) pick(e); };
  window.addEventListener("mouseup", () => { dragging = false; });
  canvas.ondblclick = () => onLive();
}
const L = 64, R = 16;
const LFRAC = () => L / (canvas.width || 1000), RFRAC = () => R / (canvas.width || 1000);

export function setTimelineMode(m, ts) { mode = m; cursorTs = ts ?? null; draw(); }
export function timelineShown() { draw(); }
export async function timelineTick(app) {
  robotId = app.liveTwin?.identity?.robot_id || robotId;
  joints = app.liveTwin?.identity?.joints || joints;
  limits = app.liveTwin?.identity?.limits || limits;
  if (mode === "live") await refresh();
}

async function refresh() {
  try {
    const until = Date.now() / 1000;
    data = await api(`/robots/${encodeURIComponent(robotId)}/timeline/track?since=${until - windowS}&until=${until}&points=600`);
    draw();
  } catch (e) { /* server not ready */ }
}

function draw() {
  if (!ctx || !data) return;
  const n = joints.length || 7;
  const cw = Math.round((canvas.clientWidth || 500) * 2);
  if (cw > 0 && canvas.width !== cw) canvas.width = cw;
  const rowH = 96, gap = 10, top = 16;
  const rows = n + 3; // joints + ee/encoder + speed + risk
  const H = top + rows * (rowH + gap) + 40;
  if (canvas.height !== H) canvas.height = H;
  const W = canvas.width;
  ctx.clearRect(0, 0, W, H); ctx.fillStyle = "#fbfcfe"; ctx.fillRect(0, 0, W, H);
  const { since, until, points, markers } = data;
  const X = (ts) => L + (ts - since) / (until - since) * (W - L - R);
  ctx.font = "17px Inter, sans-serif";

  const panel = (i) => { const y0 = top + i * (rowH + gap); return { y0, y1: y0 + rowH }; };
  const yScale = (p, lo, hi) => (v) => p.y1 - (v - lo) / (hi - lo || 1) * (p.y1 - p.y0);
  const label = (p, text) => { ctx.fillStyle = C.text; ctx.textAlign = "left"; ctx.fillText(text, 6, p.y0 + 18); };
  const frame = (p) => { ctx.strokeStyle = C.grid; ctx.lineWidth = 1; ctx.strokeRect(L, p.y0, W - L - R, p.y1 - p.y0); };
  const line = (get, color, Y, width = 2) => { ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); let started = false; points.forEach(pt => { const v = get(pt); if (v == null) { started = false; return; } const x = X(pt.ts), y = Y(v); started ? ctx.lineTo(x, y) : ctx.moveTo(x, y); started = true; }); ctx.stroke(); };
  const notice = (limits.tracking_notice || 0.05), alert = (limits.tracking_alert || 0.15);

  // joints
  for (let j = 0; j < n; j++) {
    const p = panel(j); frame(p);
    const vals = points.flatMap(pt => [pt.q?.[j], pt.q_exp?.[j], pt.q_rep?.[j]]).filter(v => v != null);
    const lo = Math.min(...vals, 0) - 0.1, hi = Math.max(...vals, 0) + 0.1;
    const Y = yScale(p, lo, hi);
    // divergence shading
    for (let i = 1; i < points.length; i++) {
      const e = Math.abs(points[i].errs?.[j] || 0); if (e < notice) continue;
      ctx.fillStyle = e >= alert ? "rgba(200,30,30,.14)" : "rgba(180,83,9,.12)";
      ctx.fillRect(X(points[i - 1].ts), p.y0, Math.max(1, X(points[i].ts) - X(points[i - 1].ts)), p.y1 - p.y0);
    }
    line(pt => pt.q_rep?.[j], C.rep, Y, 1.5);
    line(pt => pt.q_exp?.[j], C.exp, Y, 2);
    line(pt => pt.q?.[j], C.act, Y, 2);
    label(p, `${joints[j] || "joint" + (j + 1)}  [${lo.toFixed(1)}, ${hi.toFixed(1)}] rad`);
  }
  // ee + encoder
  let p = panel(n); frame(p);
  { const mx = Math.max(0.1, ...points.map(pt => Math.max(pt.ee_m || 0, pt.enc || 0))) * 1.1; const Y = yScale(p, 0, mx);
    line(pt => pt.ee_m, C.err, Y); line(pt => pt.enc, C.rep, Y, 1.5); label(p, `end-effector deviation (m) · reported vs physical (rad) · max ${mx.toFixed(2)}`); }
  // speed
  p = panel(n + 1); frame(p);
  { const vlim = limits.max_joint_velocity || 1.0; const mx = Math.max(vlim * 1.2, ...points.map(pt => Math.max(pt.vel, pt.vel_exp))) * 1.05; const Y = yScale(p, 0, mx);
    ctx.strokeStyle = C.bad; ctx.setLineDash([5, 5]); ctx.beginPath(); ctx.moveTo(L, Y(vlim)); ctx.lineTo(W - R, Y(vlim)); ctx.stroke(); ctx.setLineDash([]);
    line(pt => pt.vel_exp, C.exp, Y); line(pt => pt.vel, C.act, Y); label(p, `peak joint speed (rad/s) · limit ${vlim.toFixed(1)}`); }
  // risk band
  p = panel(n + 2); frame(p);
  for (let i = 1; i < points.length; i++) {
    const pt = points[i]; if (pt.risk === "NONE") continue;
    ctx.fillStyle = pt.risk === "CRITICAL" ? "rgba(200,30,30,.35)" : pt.risk === "HIGH" ? "rgba(194,65,12,.3)" : "rgba(180,83,9,.25)";
    ctx.fillRect(X(points[i - 1].ts), p.y0, Math.max(1, X(pt.ts) - X(points[i - 1].ts)), p.y1 - p.y0);
  }
  label(p, "risk · anomalies · incidents · faults · task steps");
  for (const m of markers) {
    let c = null;
    if (/Anomaly|Safety|Collision/.test(m.type) && !m.cleared) c = C.bad;
    else if (m.type === "IncidentCreated") c = C.bad;
    else if (m.type === "IncidentClosed") c = C.exp;
    else if (m.type === "StateDivergenceObserved") c = C.err;
    else if (m.type === "FaultInjected") c = C.fault;
    else if (m.type === "TrajectoryGoalReceived" || m.type === "TaskStepStarted") c = C.step;
    if (!c) continue;
    const x = X(m.ts); ctx.strokeStyle = c; ctx.lineWidth = m.type.startsWith("Incident") || m.type === "FaultInjected" ? 2 : 1;
    ctx.beginPath(); ctx.moveTo(x, m.type === "TrajectoryGoalReceived" || m.type === "TaskStepStarted" ? p.y0 : top); ctx.lineTo(x, p.y1); ctx.stroke();
    if (m.type === "IncidentCreated" || m.type === "FaultInjected" || m.type === "TaskStepStarted") { ctx.fillStyle = c; ctx.fillText(m.label, x + 4, p.y0 + 36 + (m.type === "TaskStepStarted" ? 20 : 0)); }
  }
  // time axis
  ctx.fillStyle = C.text;
  for (let i = 0; i <= 4; i++) { const ts = since + (until - since) * i / 4; ctx.fillText(fmtClock(ts), Math.min(W - 80, Math.max(L, X(ts) - 36)), H - 10); }
  // cursor
  if (cursorTs != null) { const x = X(cursorTs); ctx.strokeStyle = "#1f2933"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, H - 30); ctx.stroke(); }
  const c = document.getElementById("tl-cursor");
  if (c) c.textContent = cursorTs != null ? `cursor ${fmtTs(cursorTs)}` : `${points.length} snapshots · ${markers.length} markers`;
}
