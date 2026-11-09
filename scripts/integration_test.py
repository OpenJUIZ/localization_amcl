"""mobile_sim と localization_amcl を IPC でつないで走らせ、自己位置推定の結果を確かめる(M2 の結合テスト)

mobile_sim をトピック cmd_vel で部屋の中を一周させ、localization_amcl が受けたトピック odom・scan・lidar_mount から
推定した姿勢を、mobile_sim の真の姿勢と比べる。続けて、初期姿勢の与え直し、大域的な推定、地図の差し替えを試す。

使い方:
  scripts/dev_plugin.sh build
  python3 scripts/integration_test.py --mobile-sim <mobile_sim のリポジトリ> [--juiz <juiz のパス>]
    mobile_sim は app/plugins/<target>/ にプラグインを置いておく(mobile_sim の scripts/dev_plugin.sh)
  python3 scripts/integration_test.py --mobile-sim-juizapp mobile_sim-0.3.3.juizapp [--juizapp dist/localization_amcl-0.1.1.juizapp]
    パッケージで動かす(ポートは mobile_sim 8000、localization_amcl 8002)
  python3 scripts/integration_test.py --sim-url http://localhost:8000 --amcl-url http://localhost:8002
    もう動いている mobile_sim と localization_amcl を試す(docker compose など)

IPC の名前 mobile_sim と localization_amcl を使うので、同じ機械でそれらを動かしていないこと。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# 部屋を一周して出発点に戻る道のり(mobile_sim の既定の地図。x = 1.5 の仕切り壁の出入口は y = -1〜1)
ROUTE = [
    (0.5, 0.0), (0.5, 3.8), (-4.0, 3.8), (-4.0, -3.8), (0.5, -3.8), (0.5, 0.0),
    (3.0, 0.0), (3.0, -2.5), (4.2, -2.5), (4.2, 1.2), (3.0, 1.2), (3.0, 0.0),
    (0.5, 0.0), (-2.0, 0.0),
]
# 大域的な推定のあと、粒子をしぼるために走る道のり
SHORT_ROUTE = [(0.5, 0.0), (0.5, 3.8), (-4.0, 3.8), (-4.0, -3.8), (0.5, -3.8), (0.5, 0.0), (-2.0, 0.0)]

# LiDAR を台車の中心から前にずらす量 [m]。mobile_sim の Transform robot0 -> lidar0 を変え、
# localization_amcl が lidar_mount から受け取って使うことを確かめる
MOUNT_X = 0.1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--juiz", default="juiz")
    ap.add_argument("--mobile-sim", type=Path, help="mobile_sim のリポジトリ(app/app.conf を使う)")
    ap.add_argument("--mobile-sim-juizapp", type=Path, help="mobile_sim の .juizapp(juiz run で動かす)")
    ap.add_argument("--juizapp", type=Path, help="localization_amcl の .juizapp(juiz run で動かす。ポートは 8002)")
    ap.add_argument("--sim-port", type=int, default=18000)
    ap.add_argument("--port", type=int, default=18002)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--sim-url", help="もう動いている mobile_sim(--amcl-url と一緒に使う)")
    ap.add_argument("--amcl-url", help="もう動いている localization_amcl")
    ap.add_argument("--no-global", action="store_true", help="大域的な推定の確認を飛ばす")
    ap.add_argument("--ui", type=Path, metavar="SCREENSHOT",
                    help="続けてブラウザ画面のテスト (scripts/ui_test.js、Playwright が要る) を走らせ、画面を保存する")
    args = ap.parse_args()
    if args.sim_url or args.amcl_url:
        if not (args.sim_url and args.amcl_url):
            sys.exit("give both --sim-url and --amcl-url")
        sim, base = args.sim_url.rstrip("/"), args.amcl_url.rstrip("/")
        for url in (sim, base):
            wait_ready(url, None)
        run(sim, base, args)
        print("OK")
        return
    if not args.mobile_sim and not args.mobile_sim_juizapp:
        sys.exit("give --mobile-sim or --mobile-sim-juizapp (or --sim-url and --amcl-url)")

    procs = []
    tmps = []
    try:
        if args.mobile_sim_juizapp:
            sim = "http://localhost:8000"
            work = tempfile.mkdtemp()
            env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
            procs.append(start([args.juiz, "run", str(args.mobile_sim_juizapp.resolve())], work, env))
        else:
            sim = f"http://localhost:{args.sim_port}"
            app = args.mobile_sim.resolve() / "app"
            conf = (app / "app.conf").read_text().replace('"port": 8000', f'"port": {args.sim_port}')
            tmp = app / ".amcl_integration_test.conf"
            tmp.write_text(conf)
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d", "--ipc", "mobile_sim"], app))
        wait_ready(sim, procs[-1])

        if args.juizapp:
            base = "http://localhost:8002"
            work = tempfile.mkdtemp()
            env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
            procs.append(start([args.juiz, "run", str(args.juizapp.resolve())], work, env))
        else:
            base = f"http://localhost:{args.port}"
            conf = (ROOT / "app" / "app.conf").read_text().replace('"port": 8002', f'"port": {args.port}')
            tmp = ROOT / "app" / ".integration_test.conf"
            tmp.write_text(conf)
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d"], ROOT / "app"))
        wait_ready(base, procs[-1])

        run(sim, base, args)
        if args.ui:
            print("# ui_test.js")
            subprocess.run(["node", str(ROOT / "scripts" / "ui_test.js"), f"{base}/", str(args.ui.resolve())], check=True)
        print("OK")
    finally:
        for p in reversed(procs):
            p.terminate()
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for t in tmps:
            t.unlink(missing_ok=True)


def start(cmd, cwd, env=None):
    log = tempfile.TemporaryFile()
    p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
    p.log = log
    return p


def wait_ready(base, proc):
    for _ in range(150):
        if proc is not None and proc.poll() is not None:
            proc.log.seek(0)
            sys.exit(f"juiz exited ({base}):\n" + proc.log.read().decode(errors="replace")[-4000:])
        try:
            with urllib.request.urlopen(f"{base}/api/container/list", timeout=1) as r:
                if json.load(r):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    sys.exit(f"juiz did not become ready ({base})")


def ident(base, name, type_name, container):
    return f"{base}/container_process/{name}::{type_name}:{container}"


def call(base, name, type_name, container, args=None, raw=False):
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident(base, name, type_name, container), safe="")
    req = urllib.request.Request(
        url, data=json.dumps(args or {}).encode(), method="PATCH", headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        body = r.read()
        return body if raw else json.loads(body)


def call_multipart(base, name, type_name, container, parts):
    """画像の引数を持つ Process を multipart で呼ぶ(画像は image/png、ほかは application/json)"""
    boundary = uuid.uuid4().hex
    body = b""
    for key, (ctype, data) in parts.items():
        body += f"--{boundary}\r\nContent-Disposition: form-data; name=\"{key}\"\r\nContent-Type: {ctype}\r\n\r\n".encode()
        body += data + b"\r\n"
    body += f"--{boundary}--\r\n".encode()
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident(base, name, type_name, container), safe="")
    req = urllib.request.Request(url, data=body, method="PUT",
                                 headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def sim_call(sim, name, kind, args=None, raw=False):
    return call(sim, name, f"mobile_sim_{kind}", "sim0", args, raw)


def amcl_call(base, name, kind, args=None, raw=False):
    return call(base, name, f"amcl_{kind}", "amcl0", args, raw)


def api(base, path, query=None, body=None, method="GET"):
    url = f"{base}/api/{path}"
    if query:
        url += "?" + urllib.parse.urlencode(query)
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def push_topic(base, topic, value):
    url = f"{base}/api/topic/push?topic_name={topic}"
    req = urllib.request.Request(url, data=json.dumps({"input": value}).encode(), method="PATCH",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        r.read()


def yaw(q):
    return math.atan2(2.0 * (q["w"] * q["z"] + q["x"] * q["y"]), 1.0 - 2.0 * (q["y"] ** 2 + q["z"] ** 2))


def pose2(p):
    return (p["pose"]["position"]["x"], p["pose"]["position"]["y"], yaw(p["pose"]["orientation"]))


def check(cond, msg):
    if not cond:
        sys.exit("FAILED: " + msg)
    print("ok -", msg)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def fmt(p):
    return f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"


def truth_pose(sim):
    p = sim_call(sim, "get_pose0", "get_pose")["pose"]
    return p["position"]["x"], p["position"]["y"], p["theta"]


def odom_pose(sim):
    p = sim_call(sim, "get_odom_pose0", "get_odom_pose")["pose"]
    return p["position"]["x"], p["position"]["y"], p["theta"]


def estimate(st):
    return st["pose"]["x"], st["pose"]["y"], st["pose"]["theta"]


def error(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2]))


def drive(sim, route, speed):
    """真の姿勢を見ながら、cmd_vel で道のりの点を順にたどる"""
    t0 = time.time()
    for gx, gy in route:
        deadline = time.time() + 60
        while True:
            if time.time() > deadline:
                sys.exit(f"FAILED: could not reach {(gx, gy)} (now {truth_pose(sim)})")
            x, y, th = truth_pose(sim)
            dist = math.hypot(gx - x, gy - y)
            if dist < 0.1:
                break
            err = wrap(math.atan2(gy - y, gx - x) - th)
            omega = max(-1.2, min(1.2, 2.0 * err))
            vx = 0.0 if abs(err) > 0.5 else min(speed, dist) * math.cos(err)
            push_topic(sim, "cmd_vel", {"linear": {"x": vx, "y": 0.0}, "angular": omega})
            time.sleep(0.05)
    push_topic(sim, "cmd_vel", {"linear": {"x": 0.0, "y": 0.0}, "angular": 0.0})
    return time.time() - t0


def move_lidar(sim, base):
    """mobile_sim の LiDAR の取付位置を Transform で変え、localization_amcl が受け取るのを待つ"""
    tid = next(t for t in api(sim, "transform/list") if "lidar0" in t)
    api(sim, "transform/update", {"identifier": tid}, {"translation": {"x": MOUNT_X, "y": 0.0, "z": 0.2}}, method="PATCH")
    for _ in range(50):
        st = amcl_call(base, "get_state0", "get_state")
        if abs(st["laser_pose"]["x"] - MOUNT_X) < 1e-9:
            break
        time.sleep(0.2)
    check(abs(st["laser_pose"]["x"] - MOUNT_X) < 1e-9, f"amcl takes the LiDAR mount from topic lidar_mount ({st['laser_pose']})")
    check(not st["waiting_laser_pose"], "amcl is not waiting for the mount any more")


def check_tracking(sim, base, label, max_err=0.15, max_yaw=0.1):
    time.sleep(1.0)  # 最後のスキャンが届くのを待つ
    truth = truth_pose(sim)
    odom = odom_pose(sim)
    st = amcl_call(base, "get_state0", "get_state")
    est = estimate(st)
    e, ey = error(est, truth)
    eo, _ = error(odom, truth)
    print(f"   {label}: {st['scans']} scans, {st['updates']} updates, dropped {st['dropped']}, {st['particle_count']} particles")
    print(f"   truth {fmt(truth)}  odom {fmt(odom)} (err {eo:.3f} m)  amcl {fmt(est)} (err {e:.3f} m, {ey:.3f} rad)")
    check(st["last_error"] == "", f"amcl has no error ({st['last_error']!r})")
    check(e < max_err, f"{label}: the estimate is within {max_err} m of the truth ({e:.3f} m)")
    check(ey < max_yaw, f"{label}: the heading is within {max_yaw} rad ({ey:.3f})")
    return st, truth, odom, e, eo


def check_single_tree(sim, base, st, est):
    """System をまたぐ Transform amcl_odom0 -> odom0 で、mobile_sim の中に map -> odom0 -> robot0 -> lidar0 がつながる"""
    m2o = pose2(api(base, "container/pose", {"identifier": "amcl_odom0", "reference": "map"}))
    want = st["map_to_odom"]
    check(
        math.hypot(m2o[0] - want["x"], m2o[1] - want["y"]) < 0.05 and abs(wrap(m2o[2] - want["theta"])) < 0.05,
        f"ContainerPose of amcl_odom0 from map is map_to_odom ({fmt(m2o)})",
    )
    remote = [t for t in api(sim, "transform/list") if "amcl_odom0" in t]
    check(len(remote) == 1, f"mobile_sim has the transform amcl_odom0 -> odom0 from localization_amcl ({remote})")
    prof = api(sim, "transform/profile_full", {"identifier": remote[0]})
    check(prof["remote"]["state"] == "ok", f"mobile_sim fetches amcl_odom0 from localization_amcl ({prof['remote']})")
    time.sleep(0.5)
    robot = pose2(api(sim, "container/pose", {"identifier": "robot0", "reference": "map"}))
    check(
        math.hypot(robot[0] - est[0], robot[1] - est[1]) < 0.05 and abs(wrap(robot[2] - est[2])) < 0.05,
        f"in mobile_sim, robot0 seen from map is the AMCL pose ({fmt(robot)})",
    )


def check_readonly(base):
    def readonly(name, kind):
        i = ident(base, name, f"amcl_{kind}", "amcl0")
        return api(base, "container_process/profile_full", {"identifier": i}).get("readonly", False)

    ro = [k for k in ("get_pose", "get_map", "get_map_info", "get_state") if readonly(f"{k}0", k)]
    check(len(ro) == 4, f"the getters of amcl0 are readonly ({ro})")
    check(not readonly("add_scan0", "add_scan"), "amcl_add_scan is not readonly")


def run(sim, base, args):
    for _ in range(50):
        st = amcl_call(base, "get_state0", "get_state")
        if st["scans"] > 0:
            break
        time.sleep(0.2)
    check(st["scans"] > 0, f"amcl receives scans over the topic ({st['scans']} scans)")
    check(st["has_map"] and st["map_error"] == "", f"amcl loaded the map from the conf ({st['map_source']}, {st['map_error']!r})")
    info = amcl_call(base, "get_map_info0", "get_map_info")
    sim_info = sim_call(sim, "get_map_info0", "get_map_info")
    check(info["width"] == sim_info["width"] and info["height"] == sim_info["height"]
          and info["origin"][:2] == sim_info["origin"][:2], f"amcl has the same map as mobile_sim ({info['width']}x{info['height']})")
    png = amcl_call(base, "get_map0", "get_map", raw=True)
    check(png[:4] == b"\x89PNG", "get_map returns PNG")
    move_lidar(sim, base)

    # 1. 初期姿勢 (conf の initial_pose) から追従して一周
    elapsed = drive(sim, ROUTE, args.speed)
    print(f"   drove {elapsed:.0f} s")
    st, truth, odom, e, eo = check_tracking(sim, base, "tracking")
    check(st["updates"] >= 30, f"the filter updated {st['updates']} times")
    check(e < eo, "the estimate is better than odometry")
    check(st["pose_error"] == "", f"amcl writes its ContainerPose without errors ({st['pose_error']!r})")
    est = estimate(st)
    p = pose2(api(base, "container/pose", {"identifier": "amcl0", "reference": "map"}))
    check(error(p, est)[0] < 0.05, f"ContainerPose of amcl0 from map is the estimate ({fmt(p)})")
    check_single_tree(sim, base, st, est)
    check_readonly(base)
    pose_topic = amcl_call(base, "get_pose0", "get_pose")
    check(pose_topic["header"]["frame_id"] == "map", "amcl_get_pose is in the map frame")

    # 2. 間違った初期姿勢をトピック initialpose で与え、正しい初期姿勢を与え直して立ち直る
    t = truth_pose(sim)
    push_topic(base, "initialpose", {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
                                     "pose": {"position": {"x": t[0] + 0.3, "y": t[1] - 0.3}, "theta": t[2] + 0.2}})
    time.sleep(0.5)
    st = amcl_call(base, "get_state0", "get_state")
    check(error(estimate(st), (t[0] + 0.3, t[1] - 0.3, t[2] + 0.2))[0] < 0.1,
          f"topic initialpose resets the particles ({fmt(estimate(st))})")
    drive(sim, SHORT_ROUTE, args.speed)
    check_tracking(sim, base, "after initialpose")

    # 3. 地図を画像で差し替える (mobile_sim の get_map の PNG をそのまま multipart で渡す)
    sim_png = sim_call(sim, "get_map0", "get_map", raw=True)
    before = amcl_call(base, "get_map_info0", "get_map_info")["version"]
    amcl_info_parts = {
        "image": ("image/png", sim_png),
        "resolution": ("application/json", json.dumps(sim_info["resolution"]).encode()),
        "origin": ("application/json", json.dumps(sim_info["origin"]).encode()),
        "negate": ("application/json", json.dumps(bool(sim_info.get("negate", 0))).encode()),
        "occupied_thresh": ("application/json", json.dumps(sim_info.get("occupied_thresh", 0.65)).encode()),
        "free_thresh": ("application/json", json.dumps(sim_info.get("free_thresh", 0.196)).encode()),
    }
    call_multipart(base, "set_map0", "amcl_set_map", "amcl0", amcl_info_parts)
    info = amcl_call(base, "get_map_info0", "get_map_info")
    check(info["version"] == before + 1 and info["source"] == "image", f"amcl_set_map replaces the map ({info['source']})")
    check(info["width"] == sim_info["width"], "the map from the image has the same size")
    # 読めないファイルはエラーになるが、localization_amcl は落ちない
    try:
        amcl_call(base, "load_map0", "load_map", {"path": "/no/such/map.yaml"})
        bad, body = None, ""
    except urllib.error.HTTPError as ex:
        bad, body = ex.code, ex.read().decode("utf-8", "replace")
    check(bad is not None and "cannot open" in body, f"load_map of a missing file is an error (HTTP {bad})")
    path = amcl_call(base, "load_map0", "load_map", {"path": "assets/maps/default.yaml"})
    check(path.endswith("default.yaml"), f"amcl_load_map reads the YAML ({path})")
    drive(sim, [(0.5, 0.0), (-2.0, 0.0)], args.speed)
    check_tracking(sim, base, "after replacing the map")

    # 4. 大域的な推定: 粒子を地図全体に撒き、走りながらしぼる
    if not args.no_global:
        amcl_call(base, "global_localization0", "global_localization")
        st = amcl_call(base, "get_state0", "get_state")
        check(st["particle_count"] >= 1000, f"global localization spreads the particles ({st['particle_count']})")
        # 左の部屋は対称に近いので、仕切りの右の部屋も回って向きを決める
        drive(sim, ROUTE, args.speed)
        check_tracking(sim, base, "global localization", max_err=0.25, max_yaw=0.15)


if __name__ == "__main__":
    main()
