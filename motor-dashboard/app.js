// ---------- 1. Imports and setup ----------
import { initializeApp } from "https://www.gstatic.com/firebasejs/10.12.2/firebase-app.js";
import { getAuth, signInWithEmailAndPassword, signOut, onAuthStateChanged }
  from "https://www.gstatic.com/firebasejs/10.12.2/firebase-auth.js";
import { getDatabase, ref, onValue, update, query, limitToLast }
  from "https://www.gstatic.com/firebasejs/10.12.2/firebase-database.js";
import { firebaseConfig } from "./firebase-config.js";

const app = initializeApp(firebaseConfig);
const auth = getAuth(app);
const db = getDatabase(app);
const BASE = "devices/motor1";
const $ = id => document.getElementById(id);
const fmt = (v, d = 1) => (typeof v === "number" ? v.toFixed(d) : "--");

let telemetry = {}, commandSeq = 0, unsubs = [], historyData = [], rangeMin = 60;
let cfg = { currWarn: 0.5, currTrip: 1, tempWarn: 40, tempTrip: 50, vibWarn: 1, vibTrip: 3 };

// ---------- 2. Gauges (drawn once, updated live) ----------
const GAUGES = [
  { key: "current",   label: "Current",    unit: "A",    dec: 2, max: () => cfg.currTrip * 1.25, warn: () => cfg.currWarn, trip: () => cfg.currTrip },
  { key: "vibration", label: "Vibration",  unit: "m/s²", dec: 2, max: () => cfg.vibTrip * 1.25,  warn: () => cfg.vibWarn,  trip: () => cfg.vibTrip },
  { key: "motorTemp", label: "Motor temp", unit: "°C",   dec: 1, max: () => cfg.tempTrip * 1.25, warn: () => cfg.tempWarn, trip: () => cfg.tempTrip },
  { key: "rpm",       label: "Speed",      unit: "RPM",  dec: 0, max: () => 3000 }
];
$("gauges").innerHTML = GAUGES.map(g => `
  <div class="gauge off" id="g-${g.key}">
    <svg viewBox="0 0 120 120" aria-hidden="true">
      <circle class="track" cx="60" cy="60" r="44" pathLength="100" stroke-dasharray="75 100"/>
      <circle class="val"   cx="60" cy="60" r="44" pathLength="100" stroke-dasharray="0 100"/>
    </svg>
    <div class="gv"><b>--</b><i>${g.unit}</i></div>
    <span>${g.label}</span>
  </div>`).join("");

function renderGauges() {
  const online = isOnline();
  GAUGES.forEach(g => {
    const el = $("g-" + g.key), v = telemetry[g.key], ok = typeof v === "number";
    const pct = ok ? Math.min(Math.max(v / g.max(), 0), 1) : 0;
    el.querySelector(".val").setAttribute("stroke-dasharray", `${pct * 75} 100`);
    el.querySelector("b").textContent = fmt(v, g.dec);
    let cls = "";
    if (!ok || !online) cls = "off";
    else if (g.trip) cls = v >= g.trip() ? "bad" : v >= g.warn() ? "warn" : "ok";
    el.className = "gauge " + cls;
  });
}

// ---------- 3. Charts ----------
const CH = {};
function makeChart(id, sets) {
  const css = getComputedStyle(document.documentElement);
  const muted = css.getPropertyValue("--muted").trim(), line = css.getPropertyValue("--line").trim();
  return new Chart($("c-" + id), {
    type: "line",
    data: { labels: [], datasets: sets.map(s => ({
      label: s.label, data: [], borderColor: s.color, backgroundColor: s.color + "22",
      fill: !!s.fill, borderWidth: 2, pointRadius: 0, tension: 0.3, spanGaps: true })) },
    options: {
      responsive: true, maintainAspectRatio: false, animation: false,
      interaction: { mode: "index", intersect: false },
      plugins: { legend: { display: sets.length > 1, labels: { color: muted, boxWidth: 10 } } },
      scales: {
        x: { ticks: { color: muted, maxTicksLimit: 6, maxRotation: 0 }, grid: { display: false } },
        y: { ticks: { color: muted }, grid: { color: line }, beginAtZero: !!sets[0].zero }
      }
    }
  });
}
function initCharts() {
  if (CH.current) return;
  CH.current   = makeChart("current",   [{ label: "Current", color: "#4cc3ff", fill: true, zero: true }]);
  CH.vibration = makeChart("vibration", [{ label: "Vibration", color: "#f5a524", fill: true, zero: true }]);
  CH.temp      = makeChart("temp",      [{ label: "Motor", color: "#ff7a5c" }, { label: "Ambient", color: "#2ecc8f" }]);
  CH.rpm       = makeChart("rpm",       [{ label: "RPM", color: "#b48cff", fill: true, zero: true }]);
}
const KEYS = { current: ["current"], vibration: ["vibration"], temp: ["motorTemp", "ambientTemp"], rpm: ["rpm"] };

function updateCharts() {
  if (!CH.current) return;
  const last = historyData.length ? historyData[historyData.length - 1].t : 0;
  const rows = historyData.filter(r => r.t >= last - rangeMin * 60);
  const labels = rows.map(r => new Date(r.t * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }));
  for (const [id, keys] of Object.entries(KEYS)) {
    CH[id].data.labels = labels;
    keys.forEach((k, i) => (CH[id].data.datasets[i].data = rows.map(r => r[k] ?? null)));
    CH[id].update();
  }
  $("noHistory").hidden = rows.length > 0;
}
$("rangeTabs").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  rangeMin = Number(b.dataset.min);
  document.querySelectorAll("#rangeTabs button").forEach(x => x.classList.toggle("active", x === b));
  updateCharts();
};

// ---------- 4. Login / logout ----------
$("loginBtn").onclick = async () => {
  $("loginError").textContent = "";
  try { await signInWithEmailAndPassword(auth, $("email").value.trim(), $("password").value); }
  catch { $("loginError").textContent = "Sign-in failed. Check email and password."; }
};
$("password").addEventListener("keydown", e => { if (e.key === "Enter") $("loginBtn").click(); });
$("logoutBtn").onclick = () => signOut(auth);

onAuthStateChanged(auth, user => {
  $("loginView").hidden = !!user;
  $("dashView").hidden = !user;
  unsubs.forEach(u => u()); unsubs = [];
  if (user) { initCharts(); startListening(); }
});

// ---------- 5. Live listeners ----------
function startListening() {
  unsubs.push(onValue(ref(db, `${BASE}/telemetry`), s => { telemetry = s.val() || {}; renderTelemetry(); }));
  unsubs.push(onValue(ref(db, `${BASE}/command/seq`), s => { commandSeq = s.val() || 0; }));
  unsubs.push(onValue(ref(db, `${BASE}/config`), s => {
    const c = s.val() || {};
    cfg = { ...cfg, ...c };
    document.querySelectorAll("#configForm input").forEach(i => {
      if (document.activeElement !== i) i.value = c[i.dataset.k] ?? "";
    });
    renderGauges();
  }));
  unsubs.push(onValue(ref(db, `${BASE}/history`), s => {
    historyData = [];
    s.forEach(c => { const v = c.val(); if (v && v.t) historyData.push(v); });
    historyData.sort((a, b) => a.t - b.t);
    updateCharts();
  }));
  unsubs.push(onValue(query(ref(db, `${BASE}/faults`), limitToLast(10)), s => {
    const list = $("faultList"), items = [];
    s.forEach(c => items.push(c.val()));
    list.innerHTML = "";
    if (!items.length) { list.innerHTML = '<li class="empty">No faults recorded.</li>'; return; }
    items.reverse().forEach(f => {
      const li = document.createElement("li");
      const b = document.createElement("b"), sm = document.createElement("small");
      b.textContent = f.reason;
      sm.textContent = `${new Date(f.ts).toLocaleString()} · ${f.current} A · ${f.motorTemp} °C · vib ${f.vibration}`;
      li.append(b, sm); list.appendChild(li);
    });
  }));
}

// ---------- 6. Drawing the live values ----------
function isOnline() { return telemetry.heartbeat && Date.now() - telemetry.heartbeat < 15000; }

function renderTelemetry() {
  $("voltage").textContent = fmt(telemetry.voltage, 2);
  $("power").textContent = fmt(telemetry.power, 1);
  $("ambientTemp").textContent = fmt(telemetry.ambientTemp);
  $("tempRise").textContent = fmt(telemetry.tempRise);
  $("rssi").textContent = fmt(telemetry.rssi, 0);
  const state = telemetry.state || "UNKNOWN";
  $("stateText").textContent = state;
  $("faultText").textContent = telemetry.fault || "";
  $("hero").className = "hero " + state.toLowerCase();
  renderGauges();
  updateStatus();
}

function updateStatus() {
  const online = isOnline(), state = telemetry.state;
  $("conn").className = "pill " + (online ? "on" : "off");
  $("conn").lastElementChild.textContent = online ? "Online" : "Offline";
  if (telemetry.heartbeat) {
    const s = Math.max(0, Math.round((Date.now() - telemetry.heartbeat) / 1000));
    $("lastSeen").textContent = s < 60 ? `Updated ${s} s ago` : `Last seen ${new Date(telemetry.heartbeat).toLocaleTimeString()}`;
  }
  $("startBtn").disabled = !(online && state === "STOPPED");
  $("resetBtn").disabled = !(online && state === "TRIPPED");
  $("stopBtn").disabled = false;             // stopping is always allowed
}
setInterval(() => { updateStatus(); renderGauges(); }, 1000);

// ---------- 7. Commands ----------
const sendCommand = f => update(ref(db, `${BASE}/command`), { ...f, seq: commandSeq + 1 });
$("startBtn").onclick = () => { if (confirm("Is the motor area clear? Start the motor?")) sendCommand({ run: true }); };
$("stopBtn").onclick = () => sendCommand({ run: false });
$("resetBtn").onclick = () => sendCommand({ resetFault: true });

// ---------- 8. Thresholds ----------
$("configForm").onsubmit = async e => {
  e.preventDefault();
  const v = {};
  document.querySelectorAll("#configForm input").forEach(i => { if (i.value !== "") v[i.dataset.k] = Number(i.value); });
  try { await update(ref(db, `${BASE}/config`), v); $("cfgMsg").textContent = "Saved. The ESP32 applies it within 30 s."; }
  catch (err) { $("cfgMsg").textContent = "Save failed: " + err.message; }
};
