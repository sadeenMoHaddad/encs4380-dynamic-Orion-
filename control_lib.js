/*
 * control_lib.js - shared code for the Control section (ENCS4380 Homework 2).
 *
 * Every Control page loads this file. It holds:
 *   - simulate(): the closed-loop simulator (plant + sensor + PID), stepped in time with RK4,
 *   - metrics():  rise time, overshoot, settling time, steady-state error,
 *   - small page helpers: plots (Plotly), the block diagram, "Predict, then check" boxes,
 *     the Arduino code generator and the copy button.
 *
 * Plant models (the same as Homework 1):
 *   type 1, first order  (Module 1):  tau * y' + y = K * u
 *   type 2, second order (Module 3):  y'' + 2 z wn y' + wn^2 y = K wn^2 u
 * Sensor in the feedback path (P2): tau_s * m' + m = y   (tau_s = 0 means m = y)
 *
 * Controller, continuous (Ts = 0), updated every h = 1 ms:
 *   u = Kp e + Ki * integral(e) - Kd * d(m)/dt      with e = r - m
 *   The derivative acts on the measurement (no "derivative kick" on a setpoint step)
 *   and is low-pass filtered with a 10 ms time constant.
 * Controller, digital (Ts > 0), the formula from the homework, computed at t = k Ts and
 * held constant until the next sample (zero-order hold):
 *   u[k] = Kp e[k] + Ki Ts sum_{j=0..k} e[j] + Kd (e[k] - e[k-1]) / Ts
 * Both: u is limited to [umin, umax]. With anti-windup on, the integral stops growing
 * while u is saturated in the direction the error pushes (conditional integration).
 */
(function (root) {
  "use strict";

  const H = 0.001;          // integration step, s
  const D_FILTER = 0.01;    // derivative filter time constant, s

  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  const fmt = (v, d = 2, unit = "") =>
    v == null || !isFinite(v) ? "—" : v.toFixed(d) + unit;

  /* d/dt of the state [y, y', m]. For a first-order plant y' is unused. */
  function deriv(p, s, u) {
    const d = [0, 0, 0];
    if (+p.type === 2) {
      d[0] = s[1];
      d[1] = p.wn * p.wn * (p.K * u - s[0]) - 2 * p.z * p.wn * s[1];
    } else {
      d[0] = (p.K * u - s[0]) / p.tau;
    }
    d[2] = p.taus > 0 ? (s[0] - s[2]) / p.taus : 0;
    return d;
  }

  function rk4(p, s, u, h) {
    const add = (a, b, k) => a.map((v, i) => v + k * b[i]);
    const k1 = deriv(p, s, u);
    const k2 = deriv(p, add(s, k1, h / 2), u);
    const k3 = deriv(p, add(s, k2, h / 2), u);
    const k4 = deriv(p, add(s, k3, h), u);
    return s.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  /*
   * simulate(p) -> { t, y, m, u, uSteps }
   * p: type, K, tau, wn, z        plant
   *    taus                        sensor time constant (0 = ideal sensor)
   *    kp, ki, kd                  gains
   *    r                           setpoint step (applied at t = 0)
   *    umin, umax                  control limits (default -umax .. umax)
   *    aw                          anti-windup on (default true)
   *    Ts                          controller sampling time (0 = continuous)
   *    T                           simulated time, s
   * uSteps: for the digital controller, the held value of u for each sample (t, u).
   */
  function simulate(p) {
    const q = Object.assign({ taus: 0, aw: true, Ts: 0, umax: 1e9 }, p);
    if (q.umin == null) q.umin = -q.umax;
    const N = Math.round(q.T / H);
    const every = Math.max(1, Math.ceil(N / 3000));     // keep at most ~3000 plot points
    const ts = q.Ts > 0 ? Math.max(H, Math.round(q.Ts / H) * H) : 0;
    const perSample = ts ? Math.round(ts / H) : 1;

    let s = [0, 0, 0];
    let integ = 0;          // continuous: integral of e;   digital: sum of e[j]
    let mf = 0;             // filtered measurement (continuous derivative)
    let ePrev = null;       // digital: e[k-1]
    let u = 0;
    const out = { t: [], y: [], m: [], u: [], uSteps: { t: [], u: [] } };

    for (let i = 0; i <= N; i++) {
      const t = i * H;
      if (!(q.taus > 0)) s[2] = s[0];
      const m = s[2];
      const e = q.r - m;

      if (!ts) {
        const dTerm = -q.kd * (m - mf) / D_FILTER;
        const iNew = integ + e * H;
        const raw = q.kp * e + q.ki * iNew + dTerm;
        u = clamp(raw, q.umin, q.umax);
        const pushingOut = (raw > q.umax && e > 0) || (raw < q.umin && e < 0);
        if (!(q.aw && pushingOut)) integ = iNew;
        mf += H * (m - mf) / D_FILTER;
      } else if (i % perSample === 0) {
        if (ePrev === null) ePrev = e;                 // no derivative kick on the first sample
        const sumNew = integ + e;
        const raw = q.kp * e + q.ki * ts * sumNew + q.kd * (e - ePrev) / ts;
        u = clamp(raw, q.umin, q.umax);
        const pushingOut = (raw > q.umax && e > 0) || (raw < q.umin && e < 0);
        if (!(q.aw && pushingOut)) integ = sumNew;
        ePrev = e;
        out.uSteps.t.push(t);
        out.uSteps.u.push(u);
      }

      if (i % every === 0 || i === N) {
        out.t.push(t); out.y.push(s[0]); out.m.push(m); out.u.push(u);
      }
      s = rk4(q, s, u, H);
      if (!isFinite(s[0]) || Math.abs(s[0]) > 1e9) break;
    }
    return out;
  }

  /*
   * metrics(sim, r): measured on the TRUE output y.
   *   final value yf = mean of the last 2% of the run
   *   rise time  = time from 10% to 90% of yf
   *   overshoot  = (peak - yf) / yf, in %
   *   settling   = last time y is outside yf +/- 2%  (null if it never settles)
   *   ess        = (r - yf) / r, in %
   */
  function metrics(o, r) {
    const { t, y } = o;
    const n = y.length;
    const tail = Math.max(3, Math.floor(n * 0.02));
    let yf = 0;
    for (let i = n - tail; i < n; i++) yf += y[i];
    yf /= tail;
    let lo = Infinity, hi = -Infinity;
    for (let i = n - tail; i < n; i++) { lo = Math.min(lo, y[i]); hi = Math.max(hi, y[i]); }
    const settledTail = Math.abs(yf) > 1e-9 && (hi - lo) <= 0.04 * Math.abs(yf);

    let peak = -Infinity;
    for (const v of y) peak = Math.max(peak, v);
    const cross = level => { const i = y.findIndex(v => v >= level); return i < 0 ? null : t[i]; };
    const t10 = cross(0.1 * yf), t90 = cross(0.9 * yf);
    const rise = yf > 0 && t10 != null && t90 != null ? t90 - t10 : null;
    const os = yf > 0 ? Math.max(0, (peak - yf) / yf * 100) : null;

    let settle = null;
    if (settledTail) {
      let last = -1;
      for (let i = n - 1; i >= 0; i--) if (Math.abs(y[i] - yf) > 0.02 * Math.abs(yf)) { last = i; break; }
      settle = last < 0 ? 0 : t[Math.min(n - 1, last + 1)];
    }
    const ess = r ? (r - yf) / r * 100 : null;
    return { yf, rise, os, settle, ess, oscillating: !settledTail, peak };
  }

  /* Integral of |e| over the run: one number for "how good" a response is. */
  function iae(o, r) {
    let s = 0;
    for (let i = 1; i < o.t.length; i++) s += Math.abs(r - o.y[i]) * (o.t[i] - o.t[i - 1]);
    return s;
  }

  /* ---------------- page helpers (browser only) ---------------- */

  const $ = id => document.getElementById(id);

  const PLOT_CFG = { responsive: true, displaylogo: false };
  function plot(id, traces, layout) {
    const base = {
      margin: { l: 52, r: 12, t: 36, b: 44 },
      legend: { orientation: "h", y: -0.22 },
      height: 320,
      font: { family: "system-ui, sans-serif", size: 12 },
    };
    Plotly.react(id, traces, Object.assign(base, layout || {}), PLOT_CFG);
  }

  function metricsHTML(m, extra = "") {
    const settle = m.oscillating ? "not settled" : fmt(m.settle, 2, " s");
    return `<div class="metrics">
      <div>Rise time (10–90%)<b>${fmt(m.rise, 3, " s")}</b></div>
      <div>Overshoot<b>${fmt(m.os, 1, " %")}</b></div>
      <div>Settling time (2%)<b>${settle}</b></div>
      <div>Steady-state error<b>${m.oscillating ? "—" : fmt(m.ess, 2, " %")}</b></div>
    </div>${m.oscillating ? '<p class="bad">The loop oscillates: it never settles.</p>' : ""}${extra}`;
  }

  /* Block diagram of the loop. opts.sensor: show the sensor block, opts.digital: show the sampler. */
  function diagram(opts = {}) {
    const sensorText = opts.sensor ? "Sensor 1/(τs·s+1)" : "Sensor (ideal)";
    const pid = opts.digital ? "Digital PID (Ts)" : "PID";
    return `<svg class="diagram" viewBox="0 0 680 150" role="img" aria-label="Feedback loop: setpoint, PID, limit, plant, sensor">
<defs><marker id="ar" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L8 4L0 8z" fill="currentColor"/></marker></defs>
<g fill="none" stroke="currentColor" stroke-width="1.5">
<line x1="22" y1="42" x2="62" y2="42" marker-end="url(#ar)"/><circle cx="76" cy="42" r="13"/>
<line x1="89" y1="42" x2="128" y2="42" marker-end="url(#ar)"/><rect x="129" y="22" width="110" height="40" rx="5"/>
<line x1="239" y1="42" x2="276" y2="42" marker-end="url(#ar)"/><rect x="277" y="22" width="92" height="40" rx="5"/>
<line x1="369" y1="42" x2="406" y2="42" marker-end="url(#ar)"/><rect x="407" y="22" width="96" height="40" rx="5"/>
<line x1="503" y1="42" x2="660" y2="42" marker-end="url(#ar)"/>
<line x1="590" y1="42" x2="590" y2="116"/><line x1="590" y1="116" x2="512" y2="116" marker-end="url(#ar)"/>
<rect x="362" y="96" width="150" height="40" rx="5"/><line x1="362" y1="116" x2="76" y2="116"/>
<line x1="76" y1="116" x2="76" y2="56" marker-end="url(#ar)"/></g>
<g fill="currentColor" font-size="13" font-family="system-ui, sans-serif">
<text x="6" y="47">r</text><text x="66" y="36" font-size="11">+</text><text x="82" y="64" font-size="12">−</text>
<text x="102" y="35" font-size="12">e</text><text x="${opts.digital ? 140 : 170}" y="47">${pid}</text>
<text x="250" y="35" font-size="12">u</text><text x="290" y="47">Limit u</text>
<text x="432" y="47">Plant</text><text x="666" y="47">y</text>
<text x="${opts.sensor ? 372 : 392}" y="121">${sensorText}</text>
<text x="160" y="110" font-size="11">measured m</text></g></svg>`;
  }

  /*
   * predictBox(id, cfg): a "Predict, then check" box.
   * cfg.questions: [{ q: "If I double Kp, the overshoot will…", run: () => [before, after], name: "overshoot (%)", tol }]
   * The user picks increase / decrease / stay the same; the page runs both simulations and says.
   */
  function predictBox(id, cfg) {
    const el = $(id);
    const opts = cfg.questions.map((x, i) => `<option value="${i}">${x.q}</option>`).join("");
    el.innerHTML = `<div class="predict"><strong>Predict, then check</strong>
      <label>Question <select id="${id}-q">${opts}</select></label>
      <label>My prediction <select id="${id}-a"><option value="up">it will increase</option>
      <option value="down">it will decrease</option><option value="same">it will stay about the same</option></select></label>
      <button type="button" id="${id}-b">Check</button><p id="${id}-r" aria-live="polite"></p></div>`;
    $(id + "-b").onclick = () => {
      const x = cfg.questions[+$(id + "-q").value];
      const [a, b] = x.run();
      const tol = Math.max(x.tol || 0.5, 0.03 * Math.abs(a || 0));
      const act = !isFinite(a) || !isFinite(b) ? "same" : b - a > tol ? "up" : a - b > tol ? "down" : "same";
      const words = { up: "increased", down: "decreased", same: "stayed about the same" };
      const ok = $(id + "-a").value === act;
      $(id + "-r").innerHTML = `<span class="${ok ? "ok" : "bad"}">${ok ? "✅ Your prediction matched." : "❌ Not quite."}</span>
        The ${x.name} ${words[act]}: ${fmt(a, 2)} → ${fmt(b, 2)}. ${x.why || ""}`;
    };
  }

  /* Bind inputs: <input type=range data-k="kp"> with <b data-v="kp"></b> showing the value.
   * Returns refresh(): call it after changing `state` in code, to move the sliders. */
  function bindSliders(rootId, state, onChange) {
    const root = $(rootId);
    const inputs = root.querySelectorAll("[data-k]");
    const show = () => root.querySelectorAll("[data-v]").forEach(b => {
      const v = state[b.dataset.v];
      b.textContent = typeof v === "number" ? +v.toFixed(4) : v;
    });
    const refresh = () => {
      inputs.forEach(inp => {
        const k = inp.dataset.k;
        if (inp.type === "checkbox") inp.checked = !!state[k];
        else inp.value = state[k];
      });
      show();
    };
    inputs.forEach(inp => {
      const k = inp.dataset.k;
      const ev = inp.tagName === "SELECT" || inp.type === "checkbox" ? "change" : "input";
      inp.addEventListener(ev, () => {
        state[k] = inp.type === "checkbox" ? inp.checked : (inp.tagName === "SELECT" ? inp.value : +inp.value);
        show(); onChange();
      });
    });
    refresh();
    return refresh;
  }

  /* Arduino code for the digital PID of the homework, with the current numbers. */
  function arduinoCode(c) {
    const n = v => (+v).toPrecision(4).replace(/\.?0+$/, "").replace(/\.?0+e/, "e");
    const tsMs = Math.round(c.Ts * 1000);
    return `// Digital PID, generated by the ENCS4380 Control toolkit
// u[k] = Kp e[k] + Ki Ts sum(e) + Kd (e[k] - e[k-1]) / Ts
const float Kp = ${n(c.kp)};
const float Ki = ${n(c.ki)};
const float Kd = ${n(c.kd)};
const float Ts = ${n(c.Ts)};            // s  (${tsMs} ms)
const float U_MIN = ${n(c.umin)};
const float U_MAX = ${n(c.umax)};
const bool  ANTI_WINDUP = ${c.aw ? "true" : "false"};

float setpoint = ${n(c.r)};
float errSum = 0;                        // sum of e[j]
float errPrev = 0;
bool  first = true;
unsigned long lastMs = 0;

float readMeasurement() {
  ${c.readLine || "return analogRead(A0) * (5.0 / 1023.0);   // TODO: your sensor"}
}

void writeOutput(float u) {
  ${c.writeLine || "analogWrite(9, (int)constrain(u * 255.0 / U_MAX, 0, 255));   // TODO: your actuator"}
}

void setup() {
  ${c.setupLine || "pinMode(9, OUTPUT);"}
}

void loop() {
  unsigned long now = millis();
  if (now - lastMs < ${tsMs}) return;    // run once every Ts
  lastMs += ${tsMs};

  float e = setpoint - readMeasurement();
  if (first) { errPrev = e; first = false; }

  float sumNew = errSum + e;
  float uRaw = Kp * e + Ki * Ts * sumNew + Kd * (e - errPrev) / Ts;
  float u = constrain(uRaw, U_MIN, U_MAX);

  bool saturated = (uRaw > U_MAX && e > 0) || (uRaw < U_MIN && e < 0);
  if (!(ANTI_WINDUP && saturated)) errSum = sumNew;   // anti-windup: stop integrating
  errPrev = e;

  writeOutput(u);
}
`;
  }

  function copyButton(btnId, preId) {
    $(btnId).onclick = async () => {
      const text = $(preId).textContent;
      try { await navigator.clipboard.writeText(text); }
      catch (err) {
        const ta = document.createElement("textarea");
        ta.value = text; document.body.appendChild(ta); ta.select();
        document.execCommand("copy"); ta.remove();
      }
      $(btnId).textContent = "Copied ✓";
      setTimeout(() => { $(btnId).textContent = "Copy code"; }, 1500);
    };
  }

  /* Shared navigation bar. */
  function nav(active) {
    const items = [["index.html", "Homework 1"], ["control.html", "Control overview"],
      ["control_p1.html", "P1 PID playground"], ["control_p2.html", "P2 Slow sensor"],
      ["control_p3.html", "P3 Digital PID"], ["control_p4.html", "P4 Tuning challenge"],
      ["control_my_system.html", "My system"]];
    const el = $("nav");
    if (el) el.innerHTML = items.map(([h, t]) =>
      `<a href="${h}"${h === active ? ' aria-current="page"' : ""}>${t}</a>`).join("");
  }

  const CL = { H, clamp, fmt, simulate, metrics, iae, $, plot, metricsHTML, diagram,
    predictBox, bindSliders, arduinoCode, copyButton, nav };
  if (typeof module !== "undefined" && module.exports) module.exports = CL;
  else root.CL = CL;
})(typeof window !== "undefined" ? window : globalThis);
