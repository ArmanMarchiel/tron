// The centrepiece: EXPECTED vs ACTUAL vs DIVERGENCE, per joint and for the end-effector.
const f = (v, d = 3) => (v == null ? "—" : (+v).toFixed(d));
const ee = (p) => p ? `(${f(p.x, 3)}, ${f(p.y, 3)}, ${f(p.z, 3)})` : "—";
export function renderExpectedActual(el, t) {
  const e = t.expected, p = t.physical, d = t.divergence, b = t.software_belief;
  const cls = (v, notice, alert) => v == null ? "" : Math.abs(v) >= alert ? "div-alert" : Math.abs(v) >= notice ? "div-notice" : "div-none";
  const rows = t.identity.joints.map((n, i) => {
    const eq = e.position ? e.position[i] : null, aq = p.position[i], err = d.joint_errors[i];
    const ev = e.velocity ? e.velocity[i] : 0, av = p.velocity[i];
    return `<tr><td>${n}</td><td>${f(eq)}</td><td>${f(aq)}</td><td class="${cls(err, 0.05, 0.15)}">${err >= 0 ? "+" : ""}${f(err)}</td>
      <td>${f(ev, 2)}</td><td class="${Math.abs(av) > 1.0 ? "div-alert" : ""}">${f(av, 2)}</td><td class="${cls(av - ev, 0.2, 0.4)}">${f(Math.abs(av - ev), 2)}</td>
      <td class="${cls(d.encoder_errors[i], 0.05, 0.1)}">${f(b.position[i])}</td></tr>`;
  }).join("");
  const tr = e.trajectory;
  el.innerHTML = `<h2>Expected vs actual <span class="dim">— divergence level: <span class="div-${d.level}">${d.level.toUpperCase()}</span></span></h2>
  <div class="grid3">
    <div><h3>JOINT SPACE <span class="dim">(rad, rad/s)</span></h3>
      <table class="j"><tr><th>joint</th><th>q expected</th><th>q actual</th><th>Δ q</th><th>q̇ expected</th><th>q̇ actual</th><th>Δ q̇</th><th>q reported</th></tr>${rows}</table></div>
    <div><h3>SUMMARY</h3><table class="kv big">
      <tr><td>Expected goal</td><td>${e.goal ? `'${e.goal.name}'` : "—"} <span class="dim">${tr ? `${f(tr.duration_s, 1)} s · ${f(e.progress * 100, 0)}% · from ${e.command_source || "—"}` : e.basis}</span></td></tr>
      <tr><td>Actual goal</td><td>${t.software.planner.goal ? `'${t.software.planner.goal.name}'` : "—"} <span class="dim">controller ${t.software.controller.state}</span></td></tr>
      <tr><td>Controller</td><td>${e.controller} <span class="dim">/ ${t.software.controller.name}</span></td></tr>
      <tr><td>EE expected</td><td>${ee(e.ee)}</td></tr>
      <tr><td>EE actual</td><td class="${cls(d.ee_m, 0.03, 0.08)}">${ee(p.ee)}</td></tr>
      <tr><td>EE divergence</td><td class="${cls(d.ee_m, 0.03, 0.08)}">${d.ee_m == null ? "—" : f(d.ee_m) + " m"}</td></tr>
      <tr><td>Max tracking error</td><td class="${cls(d.joint_max_abs, 0.05, 0.15)}">${f(d.joint_max_abs)} rad <span class="dim">(${t.identity.joints[d.joint_max_index]}, rms ${f(d.joint_rms)})</span></td></tr>
      <tr><td>Max velocity Δ</td><td class="${cls(d.velocity_max_abs, 0.2, 0.4)}">${f(d.velocity_max_abs, 2)} rad/s <span class="dim">(${t.identity.joints[d.velocity_max_index]})</span></td></tr>
      <tr><td>Reported vs physical</td><td class="${cls(d.encoder_max_abs, 0.05, 0.1)}">${f(d.encoder_max_abs)} rad <span class="dim">(${t.identity.joints[d.encoder_max_index]})</span></td></tr>
    </table></div>
  </div>`;
}
