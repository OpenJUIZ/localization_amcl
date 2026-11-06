// ブラウザ画面の確認(M3)。mobile_sim と localization_amcl を起動しておき、Playwright で開いて操作する
// 使い方: node scripts/ui_test.js [http://localhost:8002/] [スクリーンショットの保存先]
const { chromium } = require("playwright");

const url = process.argv[2] || "http://localhost:8002/";
const shot = process.argv[3] || "ui_test.png";

function check(cond, msg) {
  if (!cond) { console.error("FAILED: " + msg); process.exit(1); }
  console.log("ok -", msg);
}

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1100, height: 860 } });
  const errors = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url);
  await page.waitForSelector("#connection.ok", { timeout: 10000 });
  check(true, "connected to juiz");
  check(await page.isVisible("#sim_connection.ok"), "finds mobile_sim through the subsystem");
  check(await page.isVisible("#drive"), "shows the drive pad");
  check(await page.isVisible("#map_sim"), "offers the map of mobile_sim");

  const pose = async (id) => {
    const t = await page.textContent(id);
    const m = t.match(/x (-?[\d.]+), y (-?[\d.]+), θ (-?[\d.]+)/);
    return m ? { x: Number(m[1]), y: Number(m[2]), th: Number(m[3]) } : null;
  };
  await page.waitForFunction(() => /^x /.test(document.getElementById("s_pose").textContent));
  const p0 = await pose("#s_pose");
  check(p0 !== null, `amcl has an estimated pose (${JSON.stringify(p0)})`);

  // 走らせると推定位置が動き、真の位置の近くにある
  await page.keyboard.down("w");
  await page.waitForTimeout(2000);
  await page.keyboard.up("w");
  await page.waitForTimeout(1500);
  const p1 = await pose("#s_pose");
  const truth = await pose("#s_truth");
  check(Math.hypot(p1.x - p0.x, p1.y - p0.y) > 0.4, `driving moves the estimated pose (${JSON.stringify(p1)})`);
  check(Math.hypot(p1.x - truth.x, p1.y - truth.y) < 0.2, `estimate is near the truth (${JSON.stringify(truth)})`);
  const frame = await page.textContent("#s_frame");
  check(frame.includes("map → amcl0") && frame.includes("ContainerPose"), `estimate is read from the ContainerPose (${frame})`);
  const mount = await page.textContent("#s_mount");
  check(/^x -?[\d.]+/.test(mount), `shows the LiDAR mount from lidar_mount (${mount})`);
  check(/^x [\d.]+ m/.test(await page.textContent("#s_sigma")), "shows the standard deviation");

  // 地図を描いたキャンバスに、空き(白)と壁(黒)がある
  const painted = await page.evaluate(() => {
    const c = document.getElementById("map");
    const d = c.getContext("2d").getImageData(0, 0, c.width, c.height).data;
    let white = 0, black = 0;
    for (let i = 0; i < d.length; i += 4) {
      if (d[i] > 240 && d[i + 1] > 240 && d[i + 2] > 240) white++;
      if (d[i] < 30 && d[i + 1] < 30 && d[i + 2] < 30) black++;
    }
    return { white, black };
  });
  check(painted.white > 5000 && painted.black > 100, `canvas shows free space and walls (${JSON.stringify(painted)})`);
  await page.screenshot({ path: shot });

  // 初期姿勢の指定: ボタンを押してから、地図の上でドラッグする(真の位置から 1 m ずらし、+y 向き)
  const box = await page.locator("#map").boundingBox();
  const toPage = await page.evaluate(([x, y]) => {
    // 地図は 10 m 四方 (原点 -5, -5) をキャンバス 640 px に描いている
    const c = document.getElementById("map");
    const r = c.getBoundingClientRect();
    return [((x + 5) / 10) * r.width, ((5 - y) / 10) * r.height];
  }, [truth.x + 1.0, truth.y]);
  await page.click("#set_pose");
  check(await page.$eval("#set_pose", (b) => b.classList.contains("armed")), "the button arms the pose picker");
  const sx = box.x + toPage[0], sy = box.y + toPage[1];
  await page.mouse.move(sx, sy);
  await page.mouse.down();
  await page.mouse.move(sx, sy - 30, { steps: 5 });
  await page.mouse.up();
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("初期姿勢"));
  await page.waitForTimeout(500);
  const p2 = await pose("#s_pose");
  check(Math.abs(p2.x - (truth.x + 1.0)) < 0.15 && Math.abs(p2.y - truth.y) < 0.15 && Math.abs(p2.th - Math.PI / 2) < 0.2,
        `dragging on the map sets the initial pose (${JSON.stringify(p2)})`);

  // 大域的な推定
  await page.click("#global");
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("撒きました"));
  await page.waitForTimeout(500);
  const particles = await page.textContent("#s_particles");
  check(Number(particles.split("(")[0]) >= 1000, `global localization spreads the particles (${particles})`);

  // mobile_sim の地図を使う
  await page.click("#map_sim");
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("mobile_sim の地図"), { timeout: 10000 });
  await page.waitForTimeout(600);
  const mapText = await page.textContent("#s_map");
  check(mapText.startsWith("image"), `the map of mobile_sim is used (${mapText})`);

  // ファイルを読む(元に戻す)
  await page.click("#map_load");
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("読みました"));
  check((await page.textContent("#s_map")).length > 0, "the map file is read");
  check((await page.textContent("#s_error")) === "なし", "no errors");

  check(errors.length === 0, `no page errors ${JSON.stringify(errors)}`);
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
