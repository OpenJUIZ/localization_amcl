// localization_amcl のブラウザ画面
//
// juiz の HTTP ブローカーの /api/container_process/call で ContainerProcess を呼ぶ。
// - コンテナ amcl: get_state(5 Hz)、get_map と get_map_info(地図が変わったとき)、set_initial_pose、
//   global_localization、set_map(multipart で画像を渡す)、load_map
// - juiz 0.2 の Transformations: 推定した姿勢は amcl の ContainerPose(/api/container/pose、基準は map_frame)から読む。
//   読めなければ get_state の pose を使う
// - mobile_sim(subsystem としてつながっていれば): get_state(真の姿勢)、get_map(地図)、set_velocity(操作)
//   localization_amcl の HTTP 経由で ipc://mobile_sim/... を呼ぶので、mobile_sim 側の CORS の設定は要らない
"use strict";

const POLL_MS = 200;
const TRAIL_MAX = 3000;

// ---- juiz の呼び出し ----

class JuizClient {
  constructor() {
    this.ids = {}; // "amcl_get_state" などの型名 → 識別子
    this.containers = {};
  }

  // ContainerProcess の一覧(subsystem の分も)から識別子を集める。同じ型が複数あれば最初のもの
  async discover() {
    const res = await fetch("/api/container_process/list?recursive=true");
    if (!res.ok) throw new Error(`container_process/list: ${res.status}`);
    for (const e of await res.json()) {
      const [type, container] = e.type_name.split(":");
      if (this.ids[type]) continue;
      const ns = e.namespace ? `${e.namespace}/` : "";
      this.ids[type] = `${e.broker_type_name}://${e.broker_name}/container_process/${ns}${e.name}::${e.type_name}`;
      this.containers[type] = container;
    }
    for (const k of ["get_state", "get_map", "get_map_info", "set_initial_pose", "global_localization", "set_map", "load_map"]) {
      if (!this.ids[`amcl_${k}`]) throw new Error(`amcl_${k} が見つかりません`);
    }
  }

  has(type) { return Boolean(this.ids[type]); }

  // Container `name` の、Anchor `reference` から見た位置。Transformations が無い・Transform が無いときは null
  async containerPose(name, reference) {
    if (this.noPose || !reference) return null;
    const q = `identifier=${encodeURIComponent(name)}&reference=${encodeURIComponent(reference)}`;
    const res = await fetch(`/api/container/pose?${q}`);
    if (res.status === 404) { this.noPose = true; return null; }
    if (!res.ok) return null;
    const p = (await res.json()).pose;
    const q4 = p.orientation;
    const theta = Math.atan2(2 * (q4.w * q4.z + q4.x * q4.y), 1 - 2 * (q4.y * q4.y + q4.z * q4.z));
    return { x: p.position.x, y: p.position.y, theta };
  }

  async call(type, args = {}, asBlob = false) {
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[type]);
    const res = await fetch(url, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(args),
    });
    if (!res.ok) throw new Error(`${type}: ${res.status} ${await res.text()}`);
    return asBlob ? res.blob() : res.json();
  }

  // 画像の引数を持つ Process を multipart で呼ぶ(画像は image/png、ほかは application/json の部分)
  async callMultipart(type, parts) {
    const form = new FormData();
    for (const [k, v] of Object.entries(parts)) {
      form.append(k, v instanceof Blob ? v : new Blob([JSON.stringify(v)], { type: "application/json" }));
    }
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[type]);
    const res = await fetch(url, { method: "PUT", body: form });
    if (!res.ok) throw new Error(`${type}: ${res.status} ${await res.text()}`);
    return res.json();
  }
}

// ---- 座標変換 ----

class View {
  constructor(canvas) {
    this.canvas = canvas;
    this.cx = 0; this.cy = 0; this.size = 20; // 表示する正方形の中心と一辺 [m]
  }

  setRange(xmin, ymin, xmax, ymax) {
    this.cx = (xmin + xmax) / 2;
    this.cy = (ymin + ymax) / 2;
    this.size = Math.max(xmax - xmin, ymax - ymin, 1);
  }

  get pxPerM() { return this.canvas.width / this.size; }

  toCanvas(x, y) {
    const k = this.pxPerM;
    return [(x - this.cx) * k + this.canvas.width / 2, this.canvas.height / 2 - (y - this.cy) * k];
  }

  // 画面上の位置 (CSS の px) → 世界座標
  fromClient(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    const px = ((clientX - rect.left) / rect.width) * this.canvas.width;
    const py = ((clientY - rect.top) / rect.height) * this.canvas.height;
    const k = this.pxPerM;
    return [(px - this.canvas.width / 2) / k + this.cx, (this.canvas.height / 2 - py) / k + this.cy];
  }
}

// ---- 地図 ----

class MapLayer {
  constructor() {
    this.image = null;
    this.info = null;
    this.version = -1;
  }

  async load(client) {
    const blob = await client.call("amcl_get_map", {}, true);
    const info = await client.call("amcl_get_map_info");
    this.image = await loadImage(blob);
    this.info = info;
    this.version = info.version;
  }

  draw(ctx, view) {
    if (!this.image) return;
    const { resolution: r, origin: [ox, oy], width: w, height: h } = this.info;
    const [x0, y0] = view.toCanvas(ox, oy + h * r); // 画像の左上
    const k = view.pxPerM * r;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.image, x0, y0, w * k, h * k);
  }
}

async function loadImage(blob) {
  const url = URL.createObjectURL(blob);
  try {
    const img = new Image();
    img.src = url;
    await img.decode();
    return img;
  } finally {
    URL.revokeObjectURL(url);
  }
}

// ---- 描画 ----

function draw(view, map, st, sim, trails, opts, picking) {
  const ctx = view.canvas.getContext("2d");
  ctx.fillStyle = "#cdcdcd";
  ctx.fillRect(0, 0, view.canvas.width, view.canvas.height);
  map.draw(ctx, view);
  if (!st) return;

  const robot = sim ? sim.robots[0] : null;
  const radius = robot ? robot.radius : 0.2;
  if (opts.showOdom && st.updates > 0) {
    drawPath(ctx, view, trails.odom, "rgba(120, 120, 120, 0.7)", [4, 4]);
    drawRobot(ctx, view, st.odom, radius, { stroke: "#888", dash: [4, 3] });
  }
  if (robot && opts.showTruth) {
    drawPath(ctx, view, trails.truth, "rgba(26, 138, 58, 0.6)");
    drawRobot(ctx, view, robot.pose, radius, { stroke: "#1a8a3a" });
  }
  if (opts.showParticles) drawParticles(ctx, view, st.particles);
  if (opts.showScan) drawScan(ctx, view, st.pose, st.scan, st.scan_range_max, st.laser_pose);
  drawCovariance(ctx, view, st.pose, st.covariance);
  drawRobot(ctx, view, st.pose, radius, { fill: "rgba(224, 96, 0, 0.85)", stroke: "#803000" });
  if (picking) drawArrow(ctx, view, picking, "#e06000");
}

function drawParticles(ctx, view, particles) {
  ctx.strokeStyle = "rgba(45, 106, 214, 0.8)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  const len = 6;
  for (const [x, y, th] of particles) {
    const [px, py] = view.toCanvas(x, y);
    ctx.moveTo(px, py);
    ctx.lineTo(px + len * Math.cos(-th), py + len * Math.sin(-th));
  }
  ctx.stroke();
  ctx.fillStyle = "rgba(45, 106, 214, 0.9)";
  for (const [x, y] of particles) {
    const [px, py] = view.toCanvas(x, y);
    ctx.fillRect(px - 1, py - 1, 2, 2);
  }
}

// x・y の共分散の 2σ の楕円
function drawCovariance(ctx, view, p, cov) {
  if (!cov) return;
  const a = cov[0], b = cov[1], d = cov[4];
  const tr = a + d, det = a * d - b * b;
  const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  const l1 = tr / 2 + disc, l2 = Math.max(0, tr / 2 - disc);
  const angle = Math.abs(b) < 1e-12 ? (a >= d ? 0 : Math.PI / 2) : Math.atan2(l1 - a, b);
  const [cx, cy] = view.toCanvas(p.x, p.y);
  const k = view.pxPerM;
  ctx.strokeStyle = "#e06000";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.ellipse(cx, cy, Math.max(2 * Math.sqrt(l1) * k, 1), Math.max(2 * Math.sqrt(l2) * k, 1), -angle, 0, 2 * Math.PI);
  ctx.stroke();
}

function drawPath(ctx, view, pts, color, dash = []) {
  if (pts.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.setLineDash(dash);
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    const [cx, cy] = view.toCanvas(x, y);
    if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawRobot(ctx, view, p, radius, { fill = null, stroke, dash = [] }) {
  const [cx, cy] = view.toCanvas(p.x, p.y);
  const rad = Math.max(radius * view.pxPerM, 4);
  ctx.lineWidth = 2;
  ctx.setLineDash(dash);
  ctx.strokeStyle = stroke;
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.stroke();
  ctx.setLineDash([]);
  // 向き(キャンバスは y が下向きなので角度の符号を反転)
  ctx.strokeStyle = fill ? "#fff" : stroke;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + rad * Math.cos(-p.theta), cy + rad * Math.sin(-p.theta));
  ctx.stroke();
}

function drawArrow(ctx, view, { x, y, theta }, color) {
  const [cx, cy] = view.toCanvas(x, y);
  const len = 40;
  const ex = cx + len * Math.cos(-theta), ey = cy + len * Math.sin(-theta);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 12 * Math.cos(-theta - 0.4), ey - 12 * Math.sin(-theta - 0.4));
  ctx.lineTo(ex - 12 * Math.cos(-theta + 0.4), ey - 12 * Math.sin(-theta + 0.4));
  ctx.fill();
}

// localization_amcl が最後に使ったスキャン([角度, 距離] の組)を、推定した姿勢から描く(地図の壁と重なれば推定が合っている)
function drawScan(ctx, view, pose, scan, rangeMax, mount) {
  if (!scan || !mount) return;
  const c = Math.cos(pose.theta), sn = Math.sin(pose.theta);
  const sx = pose.x + c * mount.x - sn * mount.y;
  const sy = pose.y + sn * mount.x + c * mount.y;
  const st = pose.theta + mount.theta;
  ctx.fillStyle = "#d4006a";
  for (const [angle, range] of scan) {
    if (!(range < rangeMax)) continue;
    const a = st + angle;
    const [ex, ey] = view.toCanvas(sx + range * Math.cos(a), sy + range * Math.sin(a));
    ctx.fillRect(ex - 1.5, ey - 1.5, 3, 3);
  }
}

// ---- 速度の入力(キーボードとパッド) ----

class VelocityInput {
  constructor(pad) {
    this.keys = new Set();
    this.padValue = null; // [前後 -1..1, 旋回 -1..1]
    this.pad = pad;
    window.addEventListener("keydown", (e) => {
      if (e.target instanceof HTMLInputElement && e.target.type === "text") return;
      if (this.keyAxis(e.key)) { this.keys.add(e.key); e.preventDefault(); }
    });
    window.addEventListener("keyup", (e) => this.keys.delete(e.key));
    window.addEventListener("blur", () => this.keys.clear());
    const move = (e) => {
      const rect = pad.getBoundingClientRect();
      const u = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      const v = ((e.clientY - rect.top) / rect.height) * 2 - 1;
      this.padValue = [clamp(-v, -1, 1), clamp(-u, -1, 1)];
    };
    pad.addEventListener("pointerdown", (e) => { pad.setPointerCapture(e.pointerId); move(e); });
    pad.addEventListener("pointermove", (e) => { if (this.padValue) move(e); });
    const release = () => { this.padValue = null; };
    pad.addEventListener("pointerup", release);
    pad.addEventListener("pointercancel", release);
    this.drawPad();
  }

  keyAxis(key) {
    return {
      w: [1, 0], ArrowUp: [1, 0], s: [-1, 0], ArrowDown: [-1, 0],
      a: [0, 1], ArrowLeft: [0, 1], d: [0, -1], ArrowRight: [0, -1],
    }[key];
  }

  value() {
    if (this.padValue) return this.padValue;
    if (this.keys.size === 0) return null;
    let f = 0, t = 0;
    for (const k of this.keys) {
      const ax = this.keyAxis(k);
      if (ax) { f += ax[0]; t += ax[1]; }
    }
    return [clamp(f, -1, 1), clamp(t, -1, 1)];
  }

  drawPad() {
    const ctx = this.pad.getContext("2d");
    const { width: w, height: h } = this.pad;
    ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = "#bbb";
    ctx.beginPath();
    ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h);
    ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
    ctx.stroke();
    const v = this.value();
    if (v) {
      ctx.fillStyle = "#2d6ad6";
      ctx.beginPath();
      ctx.arc(w / 2 - (v[1] * w) / 2, h / 2 - (v[0] * h) / 2, 10, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
}

// ---- 初期姿勢の指定(押した位置から、ドラッグした方を向く) ----

class PosePicker {
  constructor(view, button, onPick) {
    this.view = view;
    this.button = button;
    this.armed = false;
    this.current = null; // ドラッグ中の {x, y, theta}
    const canvas = view.canvas;
    button.addEventListener("click", () => this.arm(!this.armed));
    canvas.addEventListener("pointerdown", (e) => {
      if (!this.armed) return;
      canvas.setPointerCapture(e.pointerId);
      const [x, y] = view.fromClient(e.clientX, e.clientY);
      this.current = { x, y, theta: 0 };
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!this.current) return;
      const [x, y] = view.fromClient(e.clientX, e.clientY);
      if (Math.hypot(x - this.current.x, y - this.current.y) > 1e-3) {
        this.current.theta = Math.atan2(y - this.current.y, x - this.current.x);
      }
    });
    canvas.addEventListener("pointerup", () => {
      if (!this.current) return;
      const p = this.current;
      this.current = null;
      this.arm(false);
      onPick(p);
    });
    canvas.addEventListener("pointercancel", () => { this.current = null; });
  }

  arm(on) {
    this.armed = on;
    this.button.classList.toggle("armed", on);
    this.view.canvas.classList.toggle("picking", on);
  }
}

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function pushTrail(trail, x, y) {
  const last = trail[trail.length - 1];
  if (!last || Math.hypot(x - last[0], y - last[1]) > 0.02) {
    trail.push([x, y]);
    if (trail.length > TRAIL_MAX) trail.shift();
  }
}

// ---- 表示 ----

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => v.toFixed(d);
const fmtPose = (p) => `x ${fmt(p.x)}, y ${fmt(p.y)}, θ ${fmt(p.theta)}`;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const poseErr = (a, b) => `${fmt(Math.hypot(a.x - b.x, a.y - b.y), 3)} m, ${fmt(Math.abs(wrap(a.theta - b.theta)), 3)} rad`;

function showState(st, sim, name, fromContainer) {
  const ready = st.has_map && !st.waiting_laser_pose;
  $("s_pose").textContent = !st.has_map ? "(地図待ち)" : st.waiting_laser_pose ? "(LiDAR の取付位置待ち)" : fmtPose(st.pose);
  const c = st.covariance;
  $("s_sigma").textContent = c ? `x ${fmt(Math.sqrt(c[0]), 3)} m, y ${fmt(Math.sqrt(c[4]), 3)} m, θ ${fmt(Math.sqrt(c[8]), 3)} rad` : "-";
  $("s_frame").textContent = !st.map_frame ? "(書かない)"
    : st.pose_error ? `${st.map_frame} → ${name}: ${st.pose_error}`
    : `${st.map_frame} → ${name}${fromContainer ? "(ContainerPose から)" : ""}`;
  $("s_frame").classList.toggle("bad", Boolean(st.pose_error));
  $("s_mount").textContent = st.laser_pose ? fmtPose(st.laser_pose) : "-";
  $("s_particles").textContent = `${st.particle_count}(描くのは ${st.particles.length})`;
  $("s_updates").textContent = `${st.updates} 回`;
  $("s_scans").textContent = `${st.scans}(捨てた ${st.dropped})`;
  $("s_map").textContent = st.map_error ? `読めません: ${st.map_error}`
    : st.has_map ? `${st.map_source || "?"}(${st.map_version} 回目)` : "まだ無い";
  $("s_map").classList.toggle("bad", Boolean(st.map_error));
  $("s_error").textContent = st.last_error || "なし";
  $("s_error").classList.toggle("bad", Boolean(st.last_error));
  const robot = sim ? sim.robots[0] : null;
  $("s_truth").textContent = robot ? fmtPose(robot.pose) : "(mobile_sim が無い)";
  $("s_err").textContent = robot && ready ? poseErr(st.pose, robot.pose) : "-";
  $("s_odom_err").textContent = robot && st.updates > 0 ? poseErr(st.odom, robot.pose) : "-";
}

function setStatus(id, text, cls) {
  const c = $(id);
  c.textContent = text;
  c.className = `status ${cls}`;
}

function showError(err) {
  console.error(err);
  setStatus("connection", `エラー: ${err.message}`, "error");
}

function say(text) { $("message").textContent = text; }

// 地図を出す Process(mobile_sim や mapper)の画像と情報を、amcl_set_map に渡す
async function useMapFrom(client, getMap, getInfo, label) {
  const blob = await client.call(getMap, {}, true);
  const info = await client.call(getInfo);
  await client.callMultipart("amcl_set_map", {
    image: new Blob([blob], { type: "image/png" }),
    resolution: info.resolution,
    origin: info.origin,
    negate: Boolean(info.negate),
    occupied_thresh: info.occupied_thresh ?? 0.65,
    free_thresh: info.free_thresh ?? 0.196,
  });
  say(`${label} の地図を使います。粒子は今の推定位置のまわりに撒き直しました`);
}

async function main() {
  const client = new JuizClient();
  await client.discover();
  const name = client.containers.amcl_get_state;
  setStatus("connection", `接続済み(コンテナ ${name})`, "ok");
  const hasSim = client.has("mobile_sim_get_state");
  if (hasSim) setStatus("sim_connection", "mobile_sim につながっています", "ok");
  else setStatus("sim_connection", "mobile_sim は見つかりません(真の姿勢は出しません)", "warn");
  const canDrive = client.has("mobile_sim_set_velocity");
  $("drive").hidden = !canDrive;
  $("map_sim").hidden = !(client.has("mobile_sim_get_map") && client.has("mobile_sim_get_map_info"));
  $("map_mapper").hidden = !(client.has("gmapping_get_map") && client.has("gmapping_get_map_info"));

  const view = new View($("map"));
  const map = new MapLayer();
  const input = canDrive ? new VelocityInput($("pad")) : null;
  const trails = { truth: [], odom: [] };
  let st = null, sim = null, sentZero = true, loadingMap = false;

  const picker = new PosePicker(view, $("set_pose"), async (p) => {
    try {
      await client.call("amcl_set_initial_pose", {
        pose: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: st ? st.map_frame : "map" }, pose: { position: { x: p.x, y: p.y }, theta: p.theta } },
        cov_xx: -1, cov_yy: -1, cov_aa: -1,
      });
      trails.odom = [];
      say(`初期姿勢を ${fmtPose(p)} にしました`);
    } catch (err) { say(`できませんでした: ${err.message}`); }
  });
  $("global").addEventListener("click", async () => {
    try {
      await client.call("amcl_global_localization");
      say("粒子を地図全体に撒きました。ロボットを動かすと絞られていきます");
    } catch (err) { say(`できませんでした: ${err.message}`); }
  });
  $("map_sim").addEventListener("click", () =>
    useMapFrom(client, "mobile_sim_get_map", "mobile_sim_get_map_info", "mobile_sim").catch((err) => say(`できませんでした: ${err.message}`)));
  $("map_mapper").addEventListener("click", () =>
    useMapFrom(client, "gmapping_get_map", "gmapping_get_map_info", "mapper").catch((err) => say(`できませんでした: ${err.message}`)));
  $("map_load").addEventListener("click", async () => {
    try {
      const path = await client.call("amcl_load_map", { path: $("map_path").value });
      say(`読みました: ${path}`);
    } catch (err) { say(`読めませんでした: ${err.message}`); }
  });

  let busy = false;
  setInterval(async () => {
    if (busy) return; // 前の周がまだ終わっていない
    busy = true;
    try {
      st = await client.call("amcl_get_state");
      // 推定した姿勢は ContainerPose(Anchor map_frame から見た amcl)から読む
      const cp = st.updates > 0 ? await client.containerPose(name, st.map_frame) : null;
      if (cp) st.pose = cp;
      if (hasSim) sim = await client.call("mobile_sim_get_state");
      if (st.updates > 0) pushTrail(trails.odom, st.odom.x, st.odom.y);
      if (sim) pushTrail(trails.truth, sim.robots[0].pose.x, sim.robots[0].pose.y);

      // 地図は変わったときだけ取り直す
      if (!loadingMap && st.has_map && st.map_version !== map.version) {
        loadingMap = true;
        map.load(client).catch(showError).finally(() => { loadingMap = false; });
      }
      if (map.info) {
        const { resolution: r, origin: [ox, oy], width: w, height: h } = map.info;
        view.setRange(ox, oy, ox + w * r, oy + h * r);
      }
      draw(view, map, st.has_map ? st : null, sim, trails, {
        showParticles: $("show_particles").checked,
        showScan: $("show_scan").checked,
        showTruth: $("show_truth").checked,
        showOdom: $("show_odom").checked,
      }, picker.current);
      showState(st, sim, name, Boolean(cp));

      if (input) {
        // 入力があるあいだは指令を送り続ける(mobile_sim は指令が途切れると止まる)
        input.drawPad();
        const v = input.value();
        if (v) {
          await client.call("mobile_sim_set_velocity", {
            velocity: { linear: { x: v[0] * Number($("max_v").value), y: 0.0 }, angular: v[1] * Number($("max_w").value) },
            index: 0,
          });
          sentZero = false;
        } else if (!sentZero) {
          await client.call("mobile_sim_set_velocity", { velocity: { linear: { x: 0, y: 0 }, angular: 0 }, index: 0 });
          sentZero = true;
        }
      }
      setStatus("connection", `接続済み(コンテナ ${name})`, "ok");
    } catch (err) {
      showError(err);
    } finally {
      busy = false;
    }
  }, POLL_MS);
}

main().catch(showError);
