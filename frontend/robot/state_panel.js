const f = (v, d = 2) => (v == null ? "—" : (+v).toFixed(d));
const ee = (p) => p ? `(${f(p.x)}, ${f(p.y)}, ${f(p.z)})` : "—";
export function renderRobotState(el, t) {
  const p = t.physical, b = t.software_belief, sw = t.software, env = t.environment;
  const nodes = Object.values(sw.nodes).filter(n => n.state === "running").length;
  const vmax = Math.max(...(p.velocity || [0]).map(Math.abs));
  const vidx = (p.velocity || []).findIndex(v => Math.abs(v) === vmax);
  const emax = Math.max(...(p.effort || [0]).map(Math.abs));
  const goal = sw.planner.goal;
  el.innerHTML = `<h2>Robot state <span class="dim">observed</span></h2>
  <table class="kv">
    <tr><td>Status</td><td>${t.status}${p.collision ? ' <span class="bad">CONTACT ' + (p.collision_with || "") + "</span>" : ""}</td></tr>
    <tr><td>End-effector (physical)</td><td>${ee(p.ee)} m · ${f(p.ee_speed)} m/s <span class="dim">${p.source || ""}</span></td></tr>
    <tr class="sub"><td>End-effector (reported)</td><td>${ee(b.ee)} m <span class="dim">${b.source || ""}</span></td></tr>
    <tr><td>Peak joint speed</td><td>${f(vmax)} rad/s ${vidx >= 0 ? "(" + t.identity.joints[vidx] + ")" : ""}</td></tr>
    <tr><td>Peak joint effort</td><td>${f(emax, 1)} N·m</td></tr>
    <tr><td>Gripper</td><td>${p.gripper == null ? "—" : (p.gripper > 127 ? "open" : "closed")}</td></tr>
    <tr><td>Controller</td><td>${sw.controller.name} · ${sw.controller.state}${sw.controller.goal_name ? ` · '${sw.controller.goal_name}' ${f(sw.controller.progress * 100, 0)}%` : ""}</td></tr>
    <tr><td>Planner</td><td>${sw.planner.status}${goal ? ` → '${goal.name}' (${f(goal.duration_s, 1)} s)` : ""}</td></tr>
    <tr><td>Task step</td><td>${t.task?.run_done
      ? `<span class="ok">RUN COMPLETE</span> · ${t.task.cycles_done} part${t.task.cycles_done === 1 ? "" : "s"} finished`
      : (t.task?.step ? `'${t.task.step}' (${t.task.action}) · ${t.task.status}` : "—")}</td></tr>
    <tr><td>Parts</td><td>${t.task?.cycles_target
      ? `${t.task.cycles_done ?? 0} / ${t.task.cycles_target} complete`
      : `${t.task?.cycles_done ?? 0} complete <span class="dim">(continuous)</span>`}</td></tr>
    ${t.machine ? `<tr><td>Machine</td><td>${t.machine.state ?? "—"} · door ${t.machine.door ?? "—"} · chuck ${t.machine.chuck ?? "—"}${t.machine.alarm ? ' · <span class="bad">ALARM</span>' : ""}</td></tr>` : ""}
    <tr><td>Safety</td><td>${p.protective_stop ? `<span class="bad">PROTECTIVE STOP</span>${p.protective_stop_source ? ` <span class="dim">(${p.protective_stop_source.replace("_", " ")})</span>` : ""}` : "normal"} · scanner ${t.sensors.area_scanner?.intrusion == null ? "—" : (t.sensors.area_scanner.intrusion ? '<span class="bad">intrusion</span>' : "clear")}${p.human_in_scanner_field ? ' <span class="dim">(person physically in field)</span>' : ""}${p.human_in_reach_envelope ? ' · <span class="bad">person in robot ROM</span>' : ""}</td></tr>
    <tr><td>Last command</td><td>${sw.last_command ? `'${sw.last_command.goal_name}' from <span class="${sw.last_command.accepted_as_expected ? "" : "bad"}">${sw.last_command.source_node}</span>` : "—"}</td></tr>
    <tr><td>ROS graph</td><td>${nodes} nodes · ${Object.keys(sw.topics).length} topics · ${Object.keys(t.network.participants).length} DDS participants</td></tr>
    <tr><td>Nearest obstacle</td><td>${f(env.nearest_obstacle_distance, 3)} m</td></tr>
    <tr><td>Nearest human</td><td>${f(env.nearest_human_distance)} m${p.in_forbidden_zone ? ` · <span class="bad">EE in ${p.in_forbidden_zone}</span>` : ""}${(p.in_zones || []).length ? ` <span class="dim">zones: ${p.in_zones.join(", ")}</span>` : ""}</td></tr>
    <tr><td>Payload</td><td>${p.payload_kg == null ? "—" : f(p.payload_kg, 1) + " kg"}${t.divergence.torque_residual_max != null ? ` · torque residual ${f(t.divergence.torque_residual_max, 1)} N·m (${t.identity.joints[t.divergence.torque_residual_index]})` : ""}</td></tr>
  </table>`;
}
