import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const output = new URL('../live-diagnostics-preview/', import.meta.url);
await mkdir(output, { recursive: true });
const browser = await chromium.launch({
  executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe',
  headless: true,
});
const context = await browser.newContext({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1 });
let feedEnabled = true;
const sockets = new Set();
await context.routeWebSocket(/ws:\/\/127\.0\.0\.1:8080\/?$/, ws => {
  sockets.add(ws);
  const send = data => ws.send(JSON.stringify(data));
  const status = () => {
    send({ type: 'global_status', v2v_network_established: true });
    for (let id = 0; id < 2; id++) {
      send({ type: 'vehicle_status', vehicle_id: `qcar-${id}`, status: 'connected', ip: 'Test feed' });
      send({ type: 'v2v_status', vehicle_id: `qcar-${id}`, v2v_active: true, v2v_peers: 1,
        v2v_protocol: 'UDP-Manager', v2v_local_rate: 50, v2v_fleet_rate: 20, runtime_mode: 'simulator' });
    }
  };
  ws.onMessage(raw => {
    const msg = JSON.parse(String(raw));
    if (msg.type === 'ping') setTimeout(() => send({ type: 'pong', ping_id: msg.ping_id }), 8);
    if (msg.type === 'get_status') status();
  });
  status();
  let frame = 0;
  const telemetryTimer = setInterval(() => {
    if (!feedEnabled) return;
    frame++;
    for (let id = 0; id < 2; id++) send({ type: 'telemetry', vehicle_id: `qcar-${id}`,
      x: Math.cos(frame / 100) * 2 - id, y: Math.sin(frame / 100), th: frame / 100,
      v: 0.6, u: 0.05, delta: 0.02, battery: 92 - id * 3, state: 'IDLE', gps_valid: true,
      loop_execution_ms: id ? 3.8 : 2.4 });
  }, 100);
  const statusTimer = setInterval(() => { if (feedEnabled) status(); }, 1000);
  ws.onClose(() => {
    clearInterval(telemetryTimer);
    clearInterval(statusTimer);
    sockets.delete(ws);
  });
});
const page = await context.newPage();
const errors = [];
page.on('pageerror', error => errors.push(error.message));
page.on('requestfailed', request => console.log('Request failed:', request.url(), request.failure()?.errorText));
await page.goto('http://127.0.0.1:3000/', { waitUntil: 'networkidle' });
await page.waitForFunction(() => getComputedStyle(document.querySelector('main')).display === 'flex');
await page.locator('[data-metric="execution"]').filter({ hasText: '2.4' }).waitFor();
await page.locator('[data-metric="v2v"]').filter({ hasText: '1 peer' }).waitFor();
await page.locator('[data-metric="rtt"]').filter({ hasText: /\d/ }).waitFor();
await page.waitForTimeout(1500);
await page.evaluate(() => {
  const label = document.createElement('span');
  label.textContent = 'TEST FEED · NO HARDWARE';
  label.style.cssText = 'position:fixed;bottom:12px;right:14px;z-index:100;padding:5px 8px;border:1px solid #334155;border-radius:5px;background:#0f172a;color:#a5b4fc;font:10px Inter,sans-serif;pointer-events:none';
  document.body.append(label);
});
const bar = page.getByRole('region', { name: 'Live diagnostics' });
const reports = [];
async function checkLayout(label) {
  const report = await bar.evaluate(element => {
    const rect = element.getBoundingClientRect();
    const cells = [...element.querySelectorAll('dd')].map(el => {
      const box = el.getBoundingClientRect();
      return { text: el.textContent, left: box.left, right: box.right, width: box.width, scrollWidth: el.scrollWidth };
    });
    return { width: rect.width, height: rect.height, left: rect.left, right: rect.right,
      viewport: innerWidth, pageOverflows: document.documentElement.scrollWidth > innerWidth,
      cells, collapsed: element.querySelector('button').getAttribute('aria-expanded') === 'false' };
  });
  assert.equal(report.pageOverflows, false, `${label}: page overflows viewport`);
  assert.ok(report.right <= report.viewport + 1, `${label}: diagnostics outside viewport`);
  for (const cell of report.cells) {
    assert.ok(cell.scrollWidth <= cell.width + 1, `${label}: metric overflows: ${cell.text}`);
  }
  reports.push({ label, ...report });
}
await checkLayout('desktop, both panels open');
assert.ok((await bar.boundingBox()).height <= 44);
await page.screenshot({ path: fileURLToPath(new URL('desktop.png', output)) });
await bar.screenshot({ path: fileURLToPath(new URL('strip.png', output)) });

// Vehicle changes must scope the timing to the newly selected vehicle.
await page.getByText('QCar 1', { exact: true }).first().click();
await page.locator('[data-metric="execution"]').filter({ hasText: '3.8' }).waitFor();
await page.getByText('QCar 0', { exact: true }).first().click();
await page.locator('[data-metric="execution"]').filter({ hasText: '2.4' }).waitFor();

await page.setViewportSize({ width: 1024, height: 768 });
await checkLayout('1024px, both panels open');
await page.setViewportSize({ width: 820, height: 900 });
await checkLayout('820px, both panels open');
await page.setViewportSize({ width: 390, height: 844 });
await page.getByTitle('Close Panel', { exact: true }).click();
await page.waitForTimeout(350);
await checkLayout('390px phone');
await page.screenshot({ path: fileURLToPath(new URL('mobile.png', output)) });
await page.setViewportSize({ width: 360, height: 800 });
await checkLayout('360px phone');
await page.getByTitle('Collapse live diagnostics', { exact: true }).click();
await checkLayout('collapsed');
assert.ok((await bar.boundingBox()).height <= 28);
await page.getByTitle('Show live diagnostics', { exact: true }).click();

// Keep responding to pings but silence vehicle data: old values must expire.
feedEnabled = false;
await page.waitForTimeout(3800);
assert.match(await bar.innerText(), /WAITING/);
assert.match(await page.locator('[data-metric="execution"]').innerText(), /—/);
assert.match(await page.locator('[data-metric="telemetry"]').innerText(), /0\.0/);
assert.doesNotMatch(await page.locator('[data-metric="v2v"]').innerText(), /1 peer/);
await page.getByTitle('Disconnect Bridge', { exact: true }).click();
await page.waitForTimeout(600);
assert.match(await bar.innerText(), /OFFLINE/);
assert.match(await page.locator('[data-metric="links"]').innerText(), /0\s*\/\s*2/);
assert.match(await page.locator('[data-metric="rtt"]').innerText(), /—/);
assert.deepEqual(errors, []);

await writeFile(new URL('verification.json', output), JSON.stringify({
  checks: ['desktop/tablet/mobile layout', 'vehicle selection', 'collapse/expand',
    'silent telemetry expiry', 'disconnect invalidation', 'no browser errors'], reports,
}, null, 2));
console.log(JSON.stringify({ passed: true, screenshots: output.pathname, reports }, null, 2));
await browser.close();
