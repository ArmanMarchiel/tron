// TRON web UI entry: live WebSocket twin stream, tabbed sidebar, history mode via timeline scrubbing.
import { initSimView, setSimMode, refreshOverlays } from "./robot/simview.js";
import { renderRobotState } from "./robot/state_panel.js";
import { renderExpectedActual } from "./twin/expected_actual.js";
import { renderSoftware } from "./twin/software_panel.js";
import { initScenarios, renderFaultStatus, renderTask, loadScenarioDoc, reloadFaults } from "./risk/scenarios.js";
import { initTimeline, timelineTick, setTimelineMode, timelineShown } from "./timeline/timeline.js";

export const app = { mode: "live", liveTwin: null, viewTwin: null, fault: null, historyTs: null, feed: [], tab: "state", connected: false, session: null };

const $ = (id) => document.getElementById(id);
export const fmtTs = (ts) => { const d = new Date(ts * 1000); return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0"); };
export const api = async (path, opts) => { const r = await fetch("/api" + path, opts); if (!r.ok) throw new Error(path + " " + r.status); return r.json(); };

function renderStatusLine(twin) {
  $("status-line").innerHTML = `<span>${app.mode === "live" ? '<b style="color:var(--ok)">● Live</b>' : '<b style="color:var(--medium)">◷ History</b>'}</span>
    <span>robot <b>${twin.identity.robot_id}</b> · ${twin.identity.name}</span>
    <span>sources <b>${(twin.identity.adapters || []).join(", ")}</b></span>
    <span>active incidents <b>${twin.incidents.active.length}</b></span>
    <span>${app.connected ? "connected" : "reconnecting…"}</span>`;
}

function renderAll(twin) {
  if (!twin) return;
  renderStatusLine(twin);
  renderRobotState($("robot-state"), twin);
  renderExpectedActual($("expected-actual"), twin);
  renderSoftware($("software-panel"), twin);
  renderTask($("task-panel"), twin, app.session);
  const rb = $("hdr-risk"); rb.textContent = "RISK: " + twin.risk.level + (twin.risk.active_rules.length ? ` · ${twin.risk.active_rules.map(r => r.rule).join(", ")}` : ""); rb.className = "risk-badge risk-" + twin.risk.level;
}

export function setLive() {
  app.mode = "live"; app.historyTs = null;
  $("history-banner").classList.add("hidden"); document.body.classList.remove("history");
  setTimelineMode("live");
  setSimMode("live");
  if (app.liveTwin) { app.viewTwin = app.liveTwin; renderAll(app.viewTwin); }
}

export async function viewAt(ts) {
  try {
    const s = await api(`/robots/${encodeURIComponent(app.liveTwin?.identity.robot_id || "robot-001")}/timeline/at?ts=${ts}`);
    app.mode = "history"; app.historyTs = ts; app.viewTwin = s.twin;
    $("history-ts").textContent = fmtTs(ts);
    $("history-banner").classList.remove("hidden"); document.body.classList.add("history");
    setTimelineMode("history", ts);
    setSimMode("history", ts);
    renderAll(app.viewTwin);
  } catch (e) { console.warn(e); }
}

function pushFeed(ev) {
  const p = ev.payload || {};
  const label = p.rule || p.incident_id || p.node || p.scenario || p.goal_name || p.level || "";
  const bad = /Anomaly|Safety|Collision|IncidentCreated/.test(ev.event_type) && !p.cleared;
  app.feed.unshift({ ts: ev.timestamp, text: `${ev.event_type} ${label}${p.cleared ? " (cleared)" : ""}`, bad, src: ev.source });
  app.feed = app.feed.slice(0, 60);
  $("event-feed").innerHTML = `<h2>Live event feed <span class="dim">derived + adapter events</span></h2><div class="feed">` +
    app.feed.map(f => `<div><span class="ts">${fmtTs(f.ts)}</span><span class="${f.bad ? "bad" : ""}">${f.text}</span> <span class="dim">${f.src}</span></div>`).join("") + `</div>`;
}

function connect() {
  const ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/api/ws");
  ws.onopen = () => { app.connected = true; };
  ws.onclose = () => { app.connected = false; if (app.viewTwin) renderStatusLine(app.viewTwin); setTimeout(connect, 1500); };
  ws.onmessage = (m) => {
    const msg = JSON.parse(m.data);
    if (msg.type === "twin") {
      app.liveTwin = msg.data; app.fault = msg.fault || null;
      if (msg.session && (!app.session || msg.session.id !== app.session.id)) { app.session = msg.session; onSessionChanged(); }
      renderFaultStatus(app.fault);
      if (app.mode === "live") { app.viewTwin = app.liveTwin; renderAll(app.viewTwin); }
    } else if (msg.type === "event") { pushFeed(msg.data); }
  };
}

// sidebar tabs
document.querySelectorAll("#tabs button").forEach(b => b.onclick = () => {
  app.tab = b.dataset.tab;
  document.querySelectorAll("#tabs button").forEach(x => x.classList.toggle("active", x === b));
  document.querySelectorAll(".tab-page").forEach(p => p.classList.toggle("active", p.dataset.page === app.tab));
  if (app.tab === "timeseries") timelineShown();
});

async function initPickers() {
  const [robots, scenarios, machines] = await Promise.all([
    api("/registry/robots"), api("/scenarios"), api("/machines").catch(() => [])]);
  $("pick-robot").innerHTML = robots.map(r => `<option value="${r.id}">${r.name}</option>`).join("");
  $("pick-scenario").innerHTML = scenarios.map(s => `<option value="${s.id}">${s.name || s.id}</option>`).join("");
  $("pick-machine").innerHTML = machines.map(m => `<option value="${m.id}">${m.name || m.id}</option>`).join("");
  $("pick-machine").style.display = machines.length ? "" : "none";
  $("btn-apply").onclick = async () => {
    const btn = $("btn-apply");
    btn.disabled = true;
    $("session-info").textContent = "loading cell…";
    try {
      const r = await fetch("/api/session", { method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ robot: $("pick-robot").value, scenario: $("pick-scenario").value, machine: $("pick-machine").value || null }) });
      if (!r.ok) { $("session-info").textContent = "failed: " + (await r.text()).slice(0, 120); return; }
      // Apply the new session straight away. Waiting for the twin stream to mention it is racy:
      // the adapter takes a moment to start publishing, so the view could sit on the old cell.
      app.session = await r.json();
      onSessionChanged();
    } finally {
      btn.disabled = false;
    }
  };
}
function onSessionChanged() {
  const s = app.session; if (!s) return;
  $("pick-robot").value = s.robot; $("pick-scenario").value = s.scenario;
  if (s.machine) $("pick-machine").value = s.machine;
  $("session-info").textContent = s.adapter === "mujoco" ? "" : s.adapter;
  app.feed = []; setSimMode("live", null, true);
  refreshOverlays();                      // zone toggles belong to the newly composed model
  loadScenarioDoc(s.scenario); reloadFaults(); }
initPickers();
$("btn-live").onclick = setLive;
$("event-feed").innerHTML = `<h2>Live event feed</h2><div class="feed dim">waiting for events…</div>`;
initSimView($("viz-tabs"));
initScenarios($("scenarios"));
initTimeline($("timeline"), viewAt, setLive);
connect();
setInterval(() => { if (app.liveTwin) timelineTick(app); }, 1000);
