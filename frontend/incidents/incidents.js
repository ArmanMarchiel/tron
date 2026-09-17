// Incident list + full reconstruction view (timeline, expected/actual, first divergence,
// root-cause candidates labelled Observed / Inferred / Possible cause, evidence).
import { fmtTs, api } from "../app.js";
let listEl, detailEl, onViewAt, incidents = [], selected = null, showOperator = false, detailCache = null;

export function initIncidents(list, detail, viewAt) { listEl = list; detailEl = detail; onViewAt = viewAt; renderDetail(null); }

export async function refreshIncidents() {
  try { incidents = await api("/incidents?limit=50"); } catch (e) { return; }
  renderList();
  if (selected) { const cur = incidents.find(i => i.incident_id === selected); if (cur && (cur.status === "open" || !detailCache || detailCache.updated_at !== cur.updated_at)) loadDetail(selected); }
}

export function onLiveEvent(ev) { if (/Incident/.test(ev.event_type)) refreshIncidents(); }

export function selectIncidentAt(ts) {
  const hit = incidents.find(i => i.created_at - 6 <= ts && ts <= (i.closed_at || Date.now() / 1000) + 1);
  if (hit && hit.incident_id !== selected) loadDetail(hit.incident_id);
}

function renderList() {
  listEl.innerHTML = `<h2>Incidents <span class="dim">${incidents.length}</span></h2>` + (incidents.length ? incidents.map(i => `
    <div class="inc-item ${i.incident_id === selected ? "sel" : ""}" data-id="${i.incident_id}">
      <div class="t"><span class="sev sev-${i.severity}">${i.severity}</span> ${i.incident_id} · ${i.title}</div>
      <div class="m">${i.status.toUpperCase()} · opened ${fmtTs(i.created_at)}${i.closed_at ? " · closed " + fmtTs(i.closed_at) : ""} · ${i.rules.length} rule(s): ${i.rules.join(", ")}</div>
    </div>`).join("") : `<div class="dim">No incidents. Inject a fault scenario to create one.</div>`);
  listEl.querySelectorAll(".inc-item").forEach(el => el.onclick = () => loadDetail(el.dataset.id));
}

async function loadDetail(id) {
  selected = id;
  try { detailCache = await api(`/incidents/${id}?include_operator=${showOperator}`); } catch (e) { return; }
  renderList(); renderDetail(detailCache);
}

function renderDetail(inc) {
  if (!inc) { detailEl.innerHTML = `<h2>Incident reconstruction</h2><div class="dim">Select an incident to reconstruct what happened.</div>`; return; }
  const r = inc.reconstruction, s = r.summary;
  const cands = r.root_cause_candidates.map(c => `<div class="cand"><span class="lbl lbl-${c.label.split(" ")[0]}">${c.label}</span> · <b>${c.rank}. ${c.title}</b><div class="dim">${c.detail}</div>
      <div class="ev">evidence: ${c.evidence.map(id => `<a data-ev="${id}">${id}</a>`).join("") || "—"}</div></div>`).join("");
  const tl = r.timeline.map(t => `<div class="tl-entry"><span class="ts" data-ts="${t.ts}" title="view twin at this instant">${fmtTs(t.ts)}</span><span class="k-${t.kind}">${t.kind}</span><span class="k-${t.kind}">${esc(t.text)}</span></div>`).join("");
  const evidence = r.evidence.map(e => `<details><summary>${fmtTs(e.timestamp)} · ${e.event_type} · ${e.source} · <span class="dim">${e.event_id}</span></summary><pre>${esc(JSON.stringify(e.payload, null, 1))}</pre></details>`).join("");
  detailEl.innerHTML = `<h2>Incident reconstruction <span class="sev sev-${inc.severity}">${inc.severity}</span> <span>${inc.incident_id} · ${inc.title} · ${inc.status.toUpperCase()}</span>
      <span class="spacer"></span><label class="dim"><input type="checkbox" id="op-toggle" ${showOperator ? "checked" : ""}> show operator (fault injection) events</label></h2>
    <div class="summary">
      <div><b>Expected</b>${esc(s.expected || "—")}</div>
      <div><b>Actual</b>${esc(s.actual || "—")}</div>
      <div><b>First detected divergence</b>${s.first_detected_divergence ? `<a data-ts="${s.first_detected_divergence}" style="color:var(--accent);cursor:pointer">${fmtTs(s.first_detected_divergence)}</a> <span class="dim">(${s.first_detected_divergence_event?.event_type})</span>` : "—"}</div>
      <div><b>Rules triggered</b>${s.rules_triggered.join(", ") || "—"}</div>
      <div><b>What the robot believed</b>${esc(s.robot_believed || "—")}</div>
      <div><b>What physically happened</b>${esc(s.physically || "—")}</div>
      <div style="grid-column:1/-1"><b>What changed</b>${s.what_changed.length ? s.what_changed.map(esc).join("<br>") : "—"}</div>
    </div>
    <h4>Root-cause candidates <span class="dim">(never asserted — ranked by evidence)</span></h4>${cands}
    <h4>Causal timeline <span class="dim">${r.timeline.length} entries · window ${fmtTs(r.window.since)} → ${fmtTs(r.window.until)} · click a time to reconstruct the twin</span></h4>
    <div style="max-height:360px;overflow:auto">${tl}</div>
    <h4>Evidence <span class="dim">${r.evidence.length} events</span></h4>${evidence || '<div class="dim">—</div>'}`;
  detailEl.querySelector("#op-toggle").onchange = (e) => { showOperator = e.target.checked; loadDetail(inc.incident_id); };
  detailEl.querySelectorAll("[data-ts]").forEach(el => el.onclick = () => onViewAt(+el.dataset.ts));
  detailEl.querySelectorAll("[data-ev]").forEach(el => el.onclick = () => { const d = [...detailEl.querySelectorAll("details")].find(x => x.querySelector("summary").textContent.includes(el.dataset.ev)); if (d) { d.open = true; d.scrollIntoView({ block: "center" }); } });
}
const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));
