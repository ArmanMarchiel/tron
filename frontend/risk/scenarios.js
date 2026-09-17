// Scenario tab: task step progress, machine tags, fault injection (from the active scenario file),
// and an editor for step durations / timeouts that writes back to the YAML file.
import { api, fmtTs } from "../app.js";
let faultsEl, statusEl, faults = [], scenarioDoc = null, activeScenarioId = null;

export async function initScenarios(el) {
  faultsEl = el;
  await reloadFaults();
}

export async function reloadFaults() {
  try { faults = await api("/faults"); } catch (e) { faults = []; }
  faultsEl.innerHTML = `<h2>Fault injection <span class="dim">simulation only · bound to task steps · each auto-clears</span></h2><div class="scen">` +
    faults.map(s => `<button data-id="${s.id}" title="${s.description}"><b>${s.id} · ${s.name}</b><span>expects ${s.expected_detection}${s.step && s.step !== "any" ? " · step " + s.step : ""}</span></button>`).join("") +
    `<button data-id="clear"><b>Clear fault</b><span>return to normal</span></button></div><div id="fault-status"></div>`;
  statusEl = faultsEl.querySelector("#fault-status");
  faultsEl.querySelectorAll("button").forEach(b => b.onclick = async () => {
    const id = b.dataset.id;
    await fetch(id === "clear" ? "/api/faults/clear" : `/api/faults/${id}/trigger`, { method: "POST" });
  });
}

export function renderFaultStatus(fault) {
  if (!faultsEl) return;
  faultsEl.querySelectorAll("button").forEach(b => b.classList.toggle("active", !!fault && b.dataset.id === fault.id));
  if (statusEl) statusEl.textContent = fault ? `ACTIVE: ${fault.id} ${fault.name} — ${Math.max(0, fault.ends_at - Date.now() / 1000).toFixed(0)} s remaining · ${fault.description}` : "";
}

export function renderTask(el, twin, session) {
  const t = twin.task || {}, m = twin.machine;
  const steps = scenarioDoc?.task?.steps || [];
  const doneSet = new Set(t.completed || []);
  const now = Date.now() / 1000;
  const prog = t.status === "active" && t.started_ts && t.duration_s ? Math.min(100, (now - t.started_ts) / t.duration_s * 100) : 0;
  const li = steps.map((s, i) => {
    const cls = t.step === s.id && t.status === "active" ? "active" : (t.last_failure?.step === s.id && now - t.last_failure.ts < 8) ? "failed" : (doneSet.has(s.id) ? "done" : "");
    const meta = [s.action, s.target || s.command || s.gripper || s.precondition || "", `${s.duration_s}s${s.timeout_s ? " / to " + s.timeout_s + "s" : ""}`].filter(Boolean).join(" · ");
    return `<li class="${cls}"><span class="idx">${i + 1}</span><span class="name"><b>${s.id}</b><div class="meta">${meta}</div>${cls === "active" ? `<div class="progress"><i style="width:${prog}%"></i></div>` : ""}</span><span class="meta">${cls === "active" ? (now - t.started_ts).toFixed(1) + " s" : ""}</span></li>`;
  }).join("");
  const mtags = m ? `<div class="machine-tags"><span>machine <b>${m.id}</b></span><span>state <b class="tag-${m.state}">${m.state ?? "—"}</b></span><span>door <b>${m.door ?? "—"}</b></span><span>chuck <b>${m.chuck ?? "—"}</b></span><span>cycle <b>${m.cycle_count ?? "—"}</b>${m.state === "RUNNING" ? ` (${Math.round((m.cycle_progress || 0) * 100)}%)` : ""}</span>${m.alarm ? `<span class="bad">ALARM ${m.alarm}</span>` : ""}${m.last_command ? `<span class="dim">last cmd ${m.last_command.command} from ${m.last_command.client} ${m.last_command.accepted ? "ok" : "REJECTED"}</span>` : ""}</div>` : "";
  el.innerHTML = `<h2>Task <span class="dim">${session ? session.scenario + " · " + session.robot : ""}</span> <span class="spacer"></span><span class="dim">${t.status || "idle"}${t.last_failure ? " · last failure: " + t.last_failure.step + " (" + t.last_failure.reason + ")" : ""}</span></h2>${mtags}<ul class="steps">${li}</ul>`;
}

export async function loadScenarioDoc(id) {
  if (!id || id === activeScenarioId) return;
  activeScenarioId = id;
  try { scenarioDoc = await api(`/scenarios/${id}`); } catch (e) { scenarioDoc = null; }
  renderEditor();
}

function renderEditor() {
  const el = document.getElementById("scenario-editor");
  if (!scenarioDoc) { el.innerHTML = ""; return; }
  const rows = scenarioDoc.task.steps.map((s, i) => `<tr><td>${i + 1}</td><td><b>${s.id}</b> <span class="dim">${s.action}</span></td>
    <td><input type="number" step="0.5" min="0.2" data-i="${i}" data-k="duration_s" value="${s.duration_s}"> s</td>
    <td><input type="number" step="1" min="0" data-i="${i}" data-k="timeout_s" value="${s.timeout_s ?? ""}" placeholder="—"> s</td></tr>`).join("");
  const zones = (scenarioDoc.zones || []).map((z, i) => `<tr><td>${z.id}</td><td>${z.type}</td><td>${z.max_tcp_speed != null ? `<input type="number" step="0.05" data-z="${i}" data-k="max_tcp_speed" value="${z.max_tcp_speed}"> m/s` : ""}${z.min_separation != null ? `<input type="number" step="0.05" data-z="${i}" data-k="min_separation" value="${z.min_separation}"> m` : ""}</td><td>${z.condition || ""}</td></tr>`).join("");
  el.innerHTML = `<h2>Scenario editor <span class="dim">${scenarioDoc.id} · saved to backend/scenarios/defs/${scenarioDoc.id}.yaml</span></h2><div class="editor">
    <h4>Task steps (duration · timeout)</h4><table>${rows}</table>
    <h4>Zones</h4><table>${zones || "<tr><td class='dim'>none</td></tr>"}</table>
    <div class="actions"><button id="ed-save" class="small">Save</button><button id="ed-save-apply" class="small">Save &amp; reload cell</button><span id="ed-msg" class="dim"></span></div></div>`;
  const collect = () => {
    const doc = JSON.parse(JSON.stringify(scenarioDoc));
    el.querySelectorAll("input[data-i]").forEach(inp => { const v = inp.value === "" ? null : +inp.value; doc.task.steps[+inp.dataset.i][inp.dataset.k] = v; });
    el.querySelectorAll("input[data-z]").forEach(inp => { doc.zones[+inp.dataset.z][inp.dataset.k] = +inp.value; });
    return doc;
  };
  const save = async (apply) => {
    const msg = el.querySelector("#ed-msg"); msg.textContent = "saving…";
    const r = await fetch(`/api/scenarios/${scenarioDoc.id}`, { method: "PUT", headers: { "Content-Type": "application/json" }, body: JSON.stringify(collect()) });
    if (!r.ok) { msg.textContent = "rejected: " + (await r.text()).slice(0, 200); return; }
    scenarioDoc = await r.json(); msg.textContent = "saved";
    if (apply) { msg.textContent = "reloading cell…"; await fetch("/api/session", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ robot: document.getElementById("pick-robot").value, scenario: scenarioDoc.id }) }); msg.textContent = "cell reloaded"; }
  };
  el.querySelector("#ed-save").onclick = () => save(false);
  el.querySelector("#ed-save-apply").onclick = () => save(true);
}
