/**
 * Execute the production PAGE in an isolated, headless browser.
 * Every browser request is fulfilled/aborted here. No localhost service,
 * serial port, saved calibration, or existing user browser is accessed.
 * Run with Node; set NODE_PATH or PLAYWRIGHT_MODULE if Playwright is external.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

function findPlaywright() {
  for (const name of [process.env.PLAYWRIGHT_MODULE, 'playwright',
    'C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright'].filter(Boolean)) {
    try { return require(name); } catch (error) { if (error.code !== 'MODULE_NOT_FOUND') throw error; }
  }
  throw new Error('Playwright unavailable: set PLAYWRIGHT_MODULE or NODE_PATH.');
}
const { chromium } = findPlaywright();
let source = fs.readFileSync(path.join(__dirname, 'y42_pc_direct_web.py'), 'utf8').replace(/\r\n/g, '\n');
// Optional review patch is applied only to this in-memory string. The production
// Python file is never imported, changed, or executed by this test runner.
if (process.env.Y42_UI_PATCH) {
  const patchLines = fs.readFileSync(process.env.Y42_UI_PATCH, 'utf8').split(/\r?\n/);
  for (let i = 0; i < patchLines.length; i++) {
    if (!patchLines[i].startsWith('@@ ')) continue;
    const oldLines = [], newLines = [];
    while (++i < patchLines.length && !patchLines[i].startsWith('@@ ')) {
      const line = patchLines[i];
      if (line.startsWith('-') || line.startsWith(' ')) oldLines.push(line.slice(1));
      if (line.startsWith('+') || line.startsWith(' ')) newLines.push(line.slice(1));
    }
    i--;
    const before = oldLines.join('\n') + '\n', after = newLines.join('\n') + '\n';
    const offset = source.indexOf(before);
    assert(offset >= 0 && source.indexOf(before, offset + 1) < 0, 'Review patch context must match exactly once');
    source = source.slice(0, offset) + after + source.slice(offset + before.length);
  }
}
const match = source.match(/^PAGE = r'''([\s\S]*?)'''\s*$/m);
assert(match, 'Production PAGE not found');
const html = match[1];
const origin = 'https://y42-ui.invalid';
const tests = [];
function test(name, fn) { tests.push({ name, fn }); }
const savedCalibration = {
  valid: true, internal_model_verified: true, quality_error: '',
  session_local_verified: false, session_tension_held: false,
  slope: [-4.7, 4.7, -4.7, 4.7], intercept: [1, 2, 3, 4],
  r2: [.998, .998, .998, .998], workspace_width: 110, workspace_height: 148,
};
function state(patch = {}) {
  return { ok: true, auto: 'idle', message: '设备空闲', control_unlocked: true,
    baud: 115200, saved_model_available: true, saved_model_reason: '',
    calibration_run_mode: 'idle', calibration_stage: 'idle',
    calibration: structuredClone(savedCalibration), ...patch };
}

async function fixture(browser, initial = state()) {
  const context = await browser.newContext({ locale: 'zh-CN', serviceWorkers: 'block', viewport: { width: 1400, height: 1000 } });
  const page = await context.newPage();
  const requests = [], getRequests = [], dialogs = [], pageErrors = [];
  let backend = initial, responseMode = 'ok', postDelay = 0, nextPollGate = null, benchmark = null, acceptConfirmation = false;
  const fulfillJson = (route, value, status = 200) => route.fulfill({ status,
    contentType: 'application/json', body: JSON.stringify(value) });
  await context.route('**/*', async route => {
    const request = route.request(), url = new URL(request.url());
    if (request.method() === 'GET') getRequests.push(url.pathname);
    // There is deliberately no route.continue()/fallback(): no real request.
    if (url.origin !== origin) return route.abort('blockedbyclient');
    if (url.pathname === '/') return route.fulfill({ contentType: 'text/html', body: html });
    if (request.method() === 'POST') {
      const body = request.postDataJSON(); requests.push({ url: url.pathname, body });
      if (postDelay) await new Promise(resolve => setTimeout(resolve, postDelay));
      if (responseMode === 'network') return route.abort('failed');
      if (responseMode === 'reject') return fulfillJson(route, { ok: false, error: '四绳基准尚未核验，请先启用标定。' }, 409);
      if (responseMode === 'invalid') return route.fulfill({ status: 500, contentType: 'text/html', body: '<h1>Internal error</h1>' });
      return fulfillJson(route, { ok: true, message: '操作已受理' });
    }
    if (url.pathname === '/api/ports') return fulfillJson(route, { ok: true, ports: [] });
    if (url.pathname === '/api/benchmark') return benchmark
      ? fulfillJson(route, { ok: true, ...structuredClone(benchmark) })
      : fulfillJson(route, { ok:false, error:'离线测试未设置固定验收图' }, 404);
    if (url.pathname === '/api/auto') {
      if (responseMode === 'poll-network') return route.abort('failed');
      const snapshot = structuredClone(backend), gate = nextPollGate; nextPollGate = null;
      if (gate) { gate.started(); await gate.released; }
      return fulfillJson(route, snapshot);
    }
    if (url.pathname === '/api/status') return fulfillJson(route, { ok: true, motors: [] });
    return fulfillJson(route, { ok: false, error: '离线测试未定义接口：' + url.pathname }, 404);
  });
  page.on('dialog', async dialog => {
    dialogs.push(dialog.message());
    if (dialog.type() === 'confirm' && acceptConfirmation) await dialog.accept();
    else await dialog.dismiss();
  });
  page.on('pageerror', error => pageErrors.push(error.message));
  await page.goto(origin + '/', { waitUntil: 'load' });
  await page.waitForFunction(() => typeof latestAuto !== 'undefined' && latestAuto.ok === true);
  const fixture = {
    page, requests, getRequests, dialogs, pageErrors,
    setResponse(mode, delay = 0) { responseMode = mode; postDelay = delay; },
    setBenchmark(value) { benchmark = structuredClone(value); },
    setConfirm(accepted) { acceptConfirmation = !!accepted; },
    holdNextPoll() {
      let started, release;
      const startedPromise = new Promise(resolve => { started = resolve; });
      const released = new Promise(resolve => { release = resolve; });
      nextPollGate = { started, released };
      return { started: startedPromise, release };
    },
    async setState(value) { backend = value; await page.evaluate(async () => {
      while (autoPollInFlight) await new Promise(resolve => setTimeout(resolve, 10));
      await refreshAuto();
    }); },
    async draw() { await page.evaluate(() => { handSegments = [[{ x: .3, y: .4 }, { x: .5, y: .6 }]]; points = []; paint(); }); },
    async toast() { return page.locator('#toastRegion').innerText(); },
    motionRequests() { return requests.filter(r => ['calibrate', 'path', 'tracking-start', 'direction-test', 'corner-check'].includes(r.body.action)); },
    async close() { await context.close(); assert.deepEqual(pageErrors, [], 'PAGE threw uncaught errors'); },
  };
  return fixture;
}

test('locked operation explains why and sends no motion', async f => {
  await f.setState(state({ control_unlocked: false }));
  await f.page.locator('button[onclick="runCustom()"]').click({ force: true });
  assert.match(await f.toast(), /连接.*自动准备/);
  assert.equal(f.motionRequests().length, 0);
});

test('busy drawing action explains current operation and sends no motion', async f => {
  await f.setState(state({ auto: 'calibration', message: '正在检查四绳基准' }));
  await f.draw();
  await f.page.locator('button[onclick="runCustom()"]').click({ force: true });
  assert.match(await f.toast(), /正在.*四绳基准|请等待/);
  assert.equal(f.motionRequests().length, 0);
});

test('busy trajectory also blocks direction-test buttons', async f => {
  await f.setState(state({ auto: 'trajectory', message: '正在运行手绘轨迹' }));
  await f.page.evaluate(() => toggleAdvancedUI());
  await f.page.locator('#directionSection').evaluate(el => { el.open = true; });
  await f.page.locator('#directionTestButton1').click({ force: true });
  assert.match(await f.toast(), /正在|请等待/);
  assert.equal(f.motionRequests().length, 0);
});

test('empty hand drawing has a Chinese visible reason', async f => {
  await f.page.evaluate(() => runCustom());
  assert.match(f.dialogs.join('\n') + await f.toast(), /绘制.*一笔|至少.*两个点|绘制.*轨迹/);
  assert.equal(f.motionRequests().length, 0);
});

test('saved coefficients after restart remain available for explicit activation', async f => {
  await f.setState(state({ calibration: { ...savedCalibration, internal_model_verified: false } }));
  await f.page.locator('#floatingCalibration').click({ force: true });
  await f.page.waitForTimeout(120);
  const actions = f.motionRequests();
  assert.equal(actions.length, 1);
  assert.equal(actions[0].body.action, 'calibrate');
  assert.equal(actions[0].body.force_full, false);
  assert.equal(actions[0].body.activate_only, true, 'Activation must not permit a full-corner fallback');
});

test('missing saved coefficients reveals the explicit full-calibration entry without motion', async f => {
  await f.setState(state({ saved_model_available: false, saved_model_reason: '没有保存的标定', calibration: {} }));
  await f.page.locator('#floatingCalibration').click({ force: true });
  assert.match(await f.toast(), /没有|未.*保存|完整/);
  assert.equal(f.motionRequests().length, 0);
  assert.equal(await f.page.locator('body').evaluate(el => el.classList.contains('advanced-ui')), true);
});

test('full calibration is explicitly requested using force_full', async f => {
  await f.page.evaluate(() => startFullCalibration());
  const actions = f.motionRequests();
  assert.equal(actions.length, 1);
  assert.equal(actions[0].body.action, 'calibrate');
  assert.equal(actions[0].body.force_full, true);
  assert.notEqual(actions[0].body.activate_only, true);
});

test('repeated activation while request is pending sends exactly once', async f => {
  f.setResponse('ok', 300);
  await f.page.evaluate(() => { void activateSavedCalibration(); void activateSavedCalibration(); });
  await f.page.waitForTimeout(380);
  assert.equal(f.motionRequests().length, 1);
});

test('old ready status cannot finish a calibration request before HTTP acceptance', async f => {
  const ready = state({ calibration: { ...savedCalibration, session_local_verified: true, session_tension_held: true } });
  await f.setState(ready); f.setResponse('ok', 350);
  await f.page.evaluate(() => { void startFullCalibration(); });
  await f.setState(ready);
  assert.equal(await f.page.evaluate(() => calibrationActivationPending), true);
  assert.doesNotMatch(await f.toast(), /标定已启用，可以运行轨迹/);
  await f.page.waitForTimeout(380);
  assert.equal(f.motionRequests().length, 1);
});

test('pre-acceptance state request returning late cannot report new calibration success', async f => {
  const ready = state({ calibration: { ...savedCalibration, session_local_verified: true, session_tension_held: true } });
  await f.setState(ready);
  const gate = f.holdNextPoll();
  await f.page.evaluate(() => { void refreshAuto(); });
  await gate.started;
  try {
    await f.page.evaluate(() => { void startFullCalibration(); });
    await f.page.waitForFunction(() => calibrationRequestAccepted && !calibrationRequestInFlight);
  } finally { gate.release(); }
  await f.page.waitForTimeout(30);
  assert.equal(await f.page.evaluate(() => calibrationActivationPending), true);
  assert.doesNotMatch(await f.toast(), /标定已启用，可以运行轨迹/);
  assert.equal(f.motionRequests().length, 1);
});

test('API refusal shows the server reason and never retries motion', async f => {
  f.setResponse('reject'); await f.draw();
  await f.page.evaluate(() => runCustom());
  assert.match(await f.toast(), /四绳基准尚未核验/);
  assert.equal(f.motionRequests().length, 1);
});

test('HTTP non-JSON failure gives a Chinese service error', async f => {
  f.setResponse('invalid'); await f.draw();
  await f.page.evaluate(() => runCustom());
  assert.match(await f.toast(), /无法识别.*500|服务.*500/);
  assert.equal(f.motionRequests().length, 1);
});

test('request timeout reports an unknown result, never falsely says not executed', async f => {
  f.setResponse('ok', 120);
  await f.page.evaluate(async () => { try { await post({ action: 'calibrate', activate_only: true }, '/api/auto', 30); } catch (_) {} });
  const toast = await f.toast();
  assert.match(toast, /未确认|无法确认|尚未确认/);
  assert.doesNotMatch(toast, /未执行|没有执行/);
  assert.equal(f.motionRequests().length, 1);
});

test('unknown activation result gives visible recovery instructions without a retry', async f => {
  f.setResponse('network');
  await f.page.evaluate(() => activateSavedCalibration());
  assert.match(await f.toast(), /刷新网页.*核对/);
  assert.match(await f.toast(), /急停.*可用/);
  f.setResponse('ok'); await f.setState(state());
  assert.match(await f.page.locator('#floatingCalibration').innerText(), /结果未确认.*刷新/);
  await f.page.locator('#floatingCalibration').click({ force: true });
  assert.equal(f.motionRequests().length, 1);
});

test('background worker failure is visible and does not start a full calibration', async f => {
  await f.setState(state({ auto: 'error', message: '已保存标定核验未通过：绳长基准变化', calibration_requires_full: true }));
  assert.match(await f.toast(), /绳长基准变化/);
  assert.equal(f.motionRequests().length, 0);
});

test('lost state polling is visible rather than silently hanging', async f => {
  f.setResponse('poll-network');
  await f.page.evaluate(async () => { while (autoPollInFlight) await new Promise(r => setTimeout(r, 10)); await refreshAuto(); });
  assert.match(await f.toast(), /连接|状态|通讯|网络|服务/);
  assert.equal(f.motionRequests().length, 0);
});

test('reloaded page uses backend full-calibration mode for its label', async f => {
  await f.setState(state({ auto: 'calibration', calibration_run_mode: 'full_four_corner', message: '完整四角标定运行中' }));
  assert.match(await f.page.locator('#floatingCalibration').innerText(), /完整.*标定/);
});

test('tension-held flag alone cannot label an unverified session ready', async f => {
  await f.setState(state({ calibration: { ...savedCalibration, session_local_verified: false, session_tension_held: true } }));
  assert.doesNotMatch(await f.page.locator('#floatingCalibration').innerText(), /已启用|已就绪/);
});

test('null, missing, and empty precision values remain unknown rather than zero', async f => {
  for (const value of [null, undefined, '', '  ']) {
    await f.setState(state({ trajectory_diagnostic: { rms_error_mm: value, max_error_mm: value, endpoint_error_mm: value, coverage: value } }));
    const text = await f.page.locator('#precisionSummary').innerText();
    assert.match(text, /等待|尚无/);
    assert.doesNotMatch(text, /RMS 0\.00|最大 0\.00|终点 0\.00/);
  }
});

test('valid model precision is distinguished from independent absolute measurement', async f => {
  await f.setState(state({ trajectory_diagnostic: { rms_error_mm: .91, coverage: null, max_error_mm: '', endpoint_error_mm: null } }));
  const text = await f.page.locator('#precisionSummary').innerText();
  assert.match(text, /RMS 0\.91/);
  assert.match(text, /电机.*模型|几何模型反算/);
  assert.match(text, /不是独立测得的绝对定位精度/);
  assert.match(text, /最大 —.*终点 —/);
});

test('loading saved parameters never starts motor motion', async f => {
  await f.page.waitForTimeout(350);
  assert.equal(f.motionRequests().length, 0);
});

test('same-size canvas sync preserves bitmap and never resets either dimension', async f => {
  const checks = await f.page.evaluate(() => {
    const result = [];
    for (const [id, sync] of [['canvas', syncCanvasGeometry], ['trackingCanvas', syncTrackingCanvasGeometry]]) {
      sync({});
      const canvas = document.getElementById(id), context = canvas.getContext('2d'), writes = [];
      context.fillStyle = '#123456'; context.fillRect(1, 1, 4, 4);
      const before = [...context.getImageData(2, 2, 1, 1).data];
      const width = Object.getOwnPropertyDescriptor(HTMLCanvasElement.prototype, 'width');
      const height = Object.getOwnPropertyDescriptor(HTMLCanvasElement.prototype, 'height');
      for (const [key, descriptor] of [['width', width], ['height', height]]) {
        Object.defineProperty(canvas, key, { configurable: true, get() { return descriptor.get.call(this); },
          set(value) { writes.push([key, value]); return descriptor.set.call(this, value); } });
      }
      try { for (let i = 0; i < 20; i++) sync({ workspace_width: 110, workspace_height: 148 }); }
      finally { delete canvas.width; delete canvas.height; }
      result.push({ id, writes, before, after: [...context.getImageData(2, 2, 1, 1).data] });
    }
    return result;
  });
  for (const check of checks) {
    assert.deepEqual(check.writes, [], check.id + ' dimensions were redundantly reset');
    assert.deepEqual(check.after, check.before, check.id + ' bitmap was cleared');
  }
});

test('empty raw feedback never falls back to filtered or cached drawing', async f => {
  const actual = await f.page.evaluate(() => {
    const outputs = [];
    for (const raw of [{ trajectory_actual_feedback_path_segments: [] }, { trajectory_actual_feedback_path: [] },
      { trajectory_actual_feedback_path_segments: null, trajectory_actual_feedback_path: [] }]) {
      renderTrajectoryPaths({ ...raw, trajectory_actual_path_segments: [[[11,22],[33,44]]], trajectory_actual_path: [[11,22],[33,44]] });
      outputs.push({ actual: structuredClone(trajectoryActual), source: document.getElementById('canvas').dataset.feedbackSource });
    }
    return outputs;
  });
  for (const output of actual) {
    assert.deepEqual(output.actual, []);
    assert.equal(output.source, 'encoder-cycles');
  }
});

test('single feedback points remain single and null coordinates never become zero', async f => {
  const output = await f.page.evaluate(() => {
    const snapshots = [];
    for (const trace of [[[[11,22]]], [[[null,22]]], [[[11,null]]], [[["",22]]], [[["11",22]]], [[null]], []]) {
      renderTrajectoryPaths({ trajectory_actual_feedback_path_segments: trace });
      snapshots.push(structuredClone(trajectoryActual));
    }
    return snapshots;
  });
  assert.deepEqual(output[0], [[{ x: .1, y: 1 - 22/148 }]]);
  for (const actual of output.slice(1)) assert.deepEqual(actual, []);
});

test('invalid raw feedback splits subpaths and never bridges a missing sample', async f => {
  const output = await f.page.evaluate(() => {
    const traces = [
      [[11,22],[22,33],null,[44,55],[55,66]],
      [[[11,22],[22,33],[null,44],[44,55],[55,66]]],
      [[[11,22],[22,33],[33,'44'],[44,55],[55,66]]],
    ];
    return traces.map(trace=> {
      renderTrajectoryPaths({ trajectory_actual_feedback_path_segments:trace });
      return structuredClone(trajectoryActual);
    });
  });
  const expected = [[[11,22],[22,33]],[[44,55],[55,66]]].map(s=>s.map(([x,y])=>({x:x/110,y:1-y/148})));
  for (const actual of output) assert.deepEqual(actual, expected);
});

test('feedback geometry preserves every raw point and uses straight segments without glow', async f => {
  const output = await f.page.evaluate(() => {
    const raw = [[[11,22],[21,24],[19,25],[35,40]], [[70,90],[72,89]]];
    const before = JSON.stringify(raw), canvas = document.getElementById('canvas'), context = canvas.getContext('2d');
    const calls = [], original = {};
    for (const name of ['moveTo','lineTo','bezierCurveTo','quadraticCurveTo','stroke']) {
      original[name] = context[name];
      context[name] = function(...args) {
        if (this.strokeStyle === '#a021ba') calls.push({name, args, width:this.lineWidth, shadow:this.shadowBlur});
        return original[name].apply(this, args);
      };
    }
    try { renderTrajectoryPaths({ trajectory_actual_feedback_path_segments: raw }); }
    finally { for (const [name, fn] of Object.entries(original)) context[name] = fn; }
    return { actual:structuredClone(trajectoryActual), unchanged:before===JSON.stringify(raw), calls, width:canvas.width, height:canvas.height };
  });
  const raw = [[[11,22],[21,24],[19,25],[35,40]], [[70,90],[72,89]]];
  assert.equal(output.unchanged, true);
  assert.deepEqual(output.actual, raw.map(s => s.map(([x,y]) => ({x:x/110,y:1-y/148}))));
  assert.equal(output.calls.filter(c => /CurveTo/.test(c.name)).length, 0);
  assert.equal(output.calls.filter(c => c.name === 'moveTo').length, 2, 'Separate raw segments must not be joined');
  const lines = output.calls.filter(c => c.name === 'lineTo');
  assert.equal(lines.length, 4);
  assert.deepEqual(lines.map(c=>c.args), raw.flatMap(s=>s.slice(1)).map(([x,y])=>[x/110*output.width,(1-y/148)*output.height]));
  assert(output.calls.every(c=>c.width === 2 && c.shadow === 0));
});

test('archived real run replays in production canvas with explicit offline label and no motion', async f => {
  const runId = '20260904_200343_291_trajectory_worker_f0e8cc3a';
  const runDir = path.join(__dirname, 'manual_test_runs', runId);
  const final = JSON.parse(fs.readFileSync(path.join(runDir, 'final_state.json'), 'utf8'));
  const manifest = JSON.parse(fs.readFileSync(path.join(runDir, 'manifest.json'), 'utf8'));
  const input = manifest.arguments[1].operator_input_absolute_segments;
  await f.setState(state({ ...final, auto:'idle', control_unlocked:false, port:'', message:'离线回放归档，设备未参与、不能发出运动命令' }));
  const actual = await f.page.evaluate(({input,runId}) => {
    document.title = '离线回放｜' + runId;
    handSegments = input.map(s=>s.map(([x,y])=>({x:x/110,y:1-y/148}))); points=[]; paint();
    const shell = document.querySelector('#drawSection .draw-canvas-shell');
    const label = document.createElement('div');
    label.textContent = '离线回放 · 2026-09-04 20:03:43 · 61 个编码器模型投影点（非真实球心测量）';
    label.style.cssText='font:700 19px Microsoft YaHei,Arial;padding:16px;color:#17324d;text-align:center';
    shell.prepend(label);
    return structuredClone(trajectoryActual);
  }, {input,runId});
  assert.equal(actual.flat().length, 61);
  assert.deepEqual(actual, final.trajectory_actual_feedback_path_segments.map(s=>s.map(([x,y])=>({x:x/110,y:1-y/148}))));
  assert.equal(f.motionRequests().length, 0);
  const outputDir = path.join(__dirname, '..', 'qa', 'ui-offline', '20260905_canvas');
  fs.mkdirSync(outputDir, {recursive:true});
  const screenshot = path.join(outputDir, 'production-canvas-200343-offline-replay.png');
  // Presentation only in this isolated fixture: avoid the sticky live-status
  // mock covering the explicit offline label when the element is screenshot.
  await f.page.addStyleTag({content:'#floatingStatus,#performancePanel,#toastRegion{display:none!important}'});
  await f.page.locator('#drawSection .draw-canvas-shell').screenshot({ path:screenshot, animations:'disabled' });
  console.log('Archived data production-PAGE screenshot: ' + screenshot);
});

function benchmarkFixture() {
  const filename = path.join(__dirname, '..', 'qa', 'handwriting_benchmark',
    '20260905_050654_751460Z_83420ddaa4ab43aaa93530b843f3c4f7', 'benchmark.json');
  const benchmark = JSON.parse(fs.readFileSync(filename, 'utf8'));
  assert.equal(benchmark.geometry_sha256, '6c0b45bd429ebcccc49b5676bcc95b6cf4b48bdf9a8a908d9050644a3c7a1565');
  return benchmark;
}

test('locked benchmark preview uses GET only, preserves settings, and hides old feedback', async f => {
  const benchmark = benchmarkFixture(); f.setBenchmark(benchmark);
  const locked = state({control_unlocked:false, calibration:{...savedCalibration,path_speed_mm_s:37},
    trajectory_target_path_segments:[[[5,5],[6,6]]], trajectory_actual_feedback_path_segments:[[[5,6],[6,7]]]});
  await f.setState(locked);
  await f.page.evaluate(()=>{document.getElementById('repeat').checked=true;});
  assert.equal(await f.page.locator('#loadBenchmarkButton').isDisabled(), false);
  assert.equal(await f.page.locator('#loadBenchmarkButton').getAttribute('data-motion'), null);
  await f.page.evaluate(()=>loadBenchmark());
  await f.setState(locked); // A subsequent poll must not reintroduce the old run.
  await f.page.waitForTimeout(1100); // Include any deferred parameter-save window.
  const snapshot = await f.page.evaluate(()=>({hand:structuredClone(handSegments), target:trajectoryTarget, actual:trajectoryActual,
    speed:document.getElementById('pathSpeed').value, repeat:document.getElementById('repeat').checked, preview:benchmarkPreviewOnly}));
  assert.deepEqual(snapshot.hand, benchmark.payload.segments);
  assert.deepEqual(snapshot.target, []); assert.deepEqual(snapshot.actual, []);
  assert.equal(snapshot.speed, '37'); assert.equal(snapshot.repeat, true); assert.equal(snapshot.preview, true);
  assert.equal(f.getRequests.filter(p=>p==='/api/benchmark').length, 1);
  assert.deepEqual(f.requests, [], 'Loading must not POST parameters, prepare, motion, or any other action');
  assert.match(await f.page.locator('#benchmarkNotice').innerText(), /测试图.*尚未运行/);
  await f.page.evaluate(()=>{
    document.title='固定验收图预览 · 未运行 · 非实测';
    const label=document.createElement('div');
    label.textContent='固定验收图预览 · 4 笔 / 138 点 · 未运行，非实测轨迹';
    label.style.cssText='font:700 20px Microsoft YaHei,Arial;padding:16px;color:#17324d;text-align:center';
    document.querySelector('#drawSection .draw-canvas-shell').prepend(label);
  });
  await f.page.addStyleTag({content:'#floatingStatus,#performancePanel,#toastRegion{display:none!important}'});
  const directory=path.join(__dirname,'..','qa','ui-offline','20260905_benchmark');
  fs.mkdirSync(directory,{recursive:true});
  const screenshot=path.join(directory,'benchmark-production-preview.png');
  await f.page.locator('#drawSection .draw-canvas-shell').screenshot({path:screenshot,animations:'disabled'});
  console.log('Benchmark preview (not measured): '+screenshot);
});

test('cancelling benchmark replacement preserves the complete operator drawing', async f => {
  f.setBenchmark(benchmarkFixture()); f.setConfirm(false); await f.draw();
  const before=await f.page.evaluate(()=>{points=[{x:.17,y:.29}];return {hand:structuredClone(handSegments),points:structuredClone(points)};});
  await f.page.evaluate(()=>loadBenchmark());
  const after=await f.page.evaluate(()=>({hand:structuredClone(handSegments),points:structuredClone(points)}));
  assert.deepEqual(after,before); assert.match(f.dialogs.join('\n'),/替换.*手绘/);
  assert.equal(await f.page.evaluate(()=>loadedBenchmark),null);
  await f.page.waitForTimeout(1100);
  assert.deepEqual(f.requests,[]);
});

test('confirming benchmark replacement changes only local drawing and does not run', async f => {
  const benchmark=benchmarkFixture(); f.setBenchmark(benchmark); f.setConfirm(true); await f.draw();
  await f.page.evaluate(()=>loadBenchmark());
  assert.match(f.dialogs.join('\n'),/替换.*手绘/);
  assert.deepEqual(await f.page.evaluate(()=>structuredClone(handSegments)),benchmark.payload.segments);
  assert.deepEqual(await f.page.evaluate(()=>points),[]);
  await f.page.waitForTimeout(1100);
  assert.deepEqual(f.requests,[]);
});

test('benchmark execution retains safety gate and submits provenance only after explicit run', async f => {
  const benchmark=benchmarkFixture(); f.setBenchmark(benchmark);
  const settings={...savedCalibration,path_speed_mm_s:37};
  await f.setState(state({control_unlocked:false,calibration:settings}));
  await f.page.evaluate(()=>loadBenchmark());
  await f.page.evaluate(()=>runCustom());
  assert.deepEqual(f.requests,[]); assert.match(await f.toast(),/连接.*自动准备/);
  assert.equal(await f.page.evaluate(()=>benchmarkPreviewOnly),true);
  await f.setState(state({calibration:settings}));
  await f.page.evaluate(()=>{document.getElementById('repeat').checked=true;return runCustom();});
  const paths=f.requests.filter(r=>r.url==='/api/path');
  assert.equal(paths.length,1); assert.equal(paths[0].body.action,'path'); assert.equal(paths[0].body.confirmed,true);
  assert.deepEqual(paths[0].body.segments,benchmark.payload.segments);
  assert.equal(paths[0].body.speed,37); assert.equal(paths[0].body.repeat,true);
  assert.equal(paths[0].body.controller_mode,'four_position');
  assert.deepEqual(paths[0].body.benchmark,{id:benchmark.benchmark_id,geometry_sha256:benchmark.geometry_sha256,scale:benchmark.scale});
  assert.equal(await f.page.evaluate(()=>benchmarkPreviewOnly),false);
});

async function runTests() {
  const executablePath = [process.env.Y42_TEST_BROWSER, chromium.executablePath(),
    'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
    'C:/Program Files/Google/Chrome/Application/chrome.exe'].find(file => file && fs.existsSync(file));
  const browser = await chromium.launch({ ...(executablePath ? { executablePath } : {}), headless: true,
    args: ['--disable-background-networking', '--disable-component-update', '--disable-sync', '--no-first-run'] });
  let failed = 0;
  try {
    for (const { name, fn } of tests) {
      let f;
      try { f = await fixture(browser); await fn(f); await f.close(); f = null; console.log('PASS ' + name); }
      catch (error) { failed++; console.error('FAIL ' + name + '\n  ' + error.message); }
      finally { if (f) await f.close().catch(() => {}); }
    }
    if (process.env.Y42_UI_SCREENSHOTS) {
      const outputDir = path.resolve(process.env.Y42_UI_SCREENSHOTS);
      fs.mkdirSync(outputDir, { recursive: true });
      const f = await fixture(browser);
      try {
        await f.page.screenshot({ path: path.join(outputDir, 'saved-calibration-ready-to-enable.png'), fullPage: true });
        await f.setState(state({ auto: 'trajectory', message: '正在运行手绘轨迹（离线界面验收）' }));
        await f.page.locator('button[onclick="runCustom()"]').click({ force: true });
        await f.page.evaluate(() => window.scrollTo(0, 0)); await f.page.waitForTimeout(250);
        await f.page.screenshot({ path: path.join(outputDir, 'busy-operation-visible-reason.png') });
        f.setResponse('ok'); await f.setState(state());
        f.setResponse('network'); await f.page.evaluate(() => activateSavedCalibration());
        await f.page.evaluate(() => window.scrollTo(0, 0)); await f.page.waitForTimeout(250);
        await f.page.screenshot({ path: path.join(outputDir, 'unknown-result-recovery-instructions.png') });
        console.log('Isolated UI screenshots: ' + outputDir);
      } finally { await f.close(); }
    }
  } finally { await browser.close(); }
  console.log(`${tests.length - failed}/${tests.length} isolated UI interaction tests passed; no real API requests.`);
  process.exitCode = failed ? 1 : 0;
}
module.exports = { test, state, runTests, clearTests: () => { tests.length = 0; } };
if (require.main === module) runTests().catch(error => { console.error(error); process.exitCode = 1; });
