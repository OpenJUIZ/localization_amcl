"""juiz で app/smoke_test.conf を起動し、localization_amcl のプラグインを読み込めることを確かめる(M0)

使い方: scripts/dev_plugin.sh build && python3 scripts/smoke_test.py [--juiz <juiz のパス>] [--port 18002]
"""
import argparse
import json
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--juiz", default="juiz")
    ap.add_argument("--port", type=int, default=18002)
    args = ap.parse_args()

    base = f"http://localhost:{args.port}"
    conf = (ROOT / "app" / "smoke_test.conf").read_text().replace('"port": 8002', f'"port": {args.port}')
    tmp = ROOT / "app" / ".smoke_test.conf"
    tmp.write_text(conf)
    log = tempfile.TemporaryFile()
    proc = subprocess.Popen([args.juiz, "-f", tmp.name, "-d"], cwd=ROOT / "app", stdout=log, stderr=subprocess.STDOUT)
    try:
        wait_ready(base, proc, log)
        version = call(base, "version0", "localization_amcl_version")
        expected = (ROOT / "CMakeLists.txt").read_text().split("project(localization_amcl VERSION ")[1].split()[0]
        if version != expected:
            sys.exit(f"FAILED: localization_amcl_version returned {version!r}, expected {expected!r}")
        print(f"ok - localization_amcl_version = {version}")
        print("OK")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        tmp.unlink(missing_ok=True)


def wait_ready(base, proc, log):
    for _ in range(150):
        if proc.poll() is not None:
            log.seek(0)
            sys.exit("juiz exited:\n" + log.read().decode(errors="replace"))
        try:
            with urllib.request.urlopen(f"{base}/api/process/list", timeout=1) as r:
                if json.load(r):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    log.seek(0)
    sys.exit("juiz did not become ready:\n" + log.read().decode(errors="replace"))


def call(base, name, type_name, args=None):
    ident = f"{base}/process/{name}::{type_name}"
    url = f"{base}/api/process/call?identifier=" + urllib.parse.quote(ident, safe="")
    req = urllib.request.Request(url, data=json.dumps(args or {}).encode(), method="PATCH",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.load(r)


if __name__ == "__main__":
    main()
