// ROS runtime + network view: nodes, topics (publishers/subscribers/rates), DDS participants.
let policy = null;
fetch("/api/risk").then(r => r.json()).then(d => { policy = d.security_policy; }).catch(() => {});
export function renderSoftware(el, t) {
  const sw = t.software, net = t.network;
  const expNodes = policy?.expected_nodes || [], expPubs = policy?.expected_publishers || {}, expParts = policy?.expected_participants || [];
  const nodes = Object.entries(sw.nodes).filter(([, s]) => s.state === "running").map(([n]) => `<span class="${expNodes.length && !expNodes.includes(n) ? "bad" : ""}">${n}</span>`).join("");
  const topics = Object.entries(sw.topics).map(([name, i]) => {
    const pubs = (i.publishers || []).map(p => `<span class="${expPubs[name] && !expPubs[name].includes(p) ? "bad" : ""}">${p}</span>`).join(", ");
    return `<tr><td>${name}</td><td>${pubs || "—"} <span class="dim">→ ${(i.subscribers || []).join(", ") || "—"}</span> <span class="dim">${i.rate_hz != null ? i.rate_hz.toFixed(1) + " Hz" : ""}</span></td></tr>`;
  }).join("");
  const parts = Object.entries(net.participants).map(([n, p]) => `<span class="${expParts.length && !expParts.includes(n) ? "bad" : ""}" title="${p.guid || ""}">${n}${p.address && p.address !== "127.0.0.1" ? " @" + p.address : ""}</span>`).join("");
  el.innerHTML = `<h2>ROS 2 runtime &amp; network <span class="dim">observed</span></h2>
    <h4>Nodes</h4><div class="chips">${nodes || '<span class="dim">none</span>'}</div>
    <h4>Topics (publishers → subscribers)</h4><table class="kv">${topics}</table>
    <h4>DDS participants</h4><div class="chips">${parts || '<span class="dim">none</span>'}</div>`;
}
