/** Independent production-PAGE regression; all requests remain intercepted. */
'use strict';
const assert = require('node:assert/strict');
const path = require('node:path');
const fs = require('node:fs');
const { test, state, runTests, clearTests } = require('./test_y42_ui_offline.cjs');
clearTests();
const failure = {
  id: '20260905_103233_820_trajectory_worker_d007530c:failure',
  run_id: '20260905_103233_820_trajectory_worker_d007530c',
  worker: 'trajectory_worker', occurred_at: '2026-09-05T10:32:41+08:00',
  message: '移动到轨迹起点超时；M4差36.77°，本次未绘制成功。', acknowledged: false,
};

test('missed transient error still appears when automatic prepare has returned idle', async f => {
  // Browser never observes auto=error. Durable failure must be sufficient.
  await f.setState(state({ auto: 'idle', message: '自动准备完成', last_failure: failure }));
  assert.match(await f.toast(), /移动到轨迹起点超时/);
  assert.equal(f.motionRequests().length, 0);
  const banner = f.page.locator('#lastFailure');
  assert.match(await banner.innerText(), /移动到轨迹起点超时/);
  if (process.env.Y42_FAILURE_SCREENSHOT) {
    const filename = path.resolve(process.env.Y42_FAILURE_SCREENSHOT);
    fs.mkdirSync(path.dirname(filename), { recursive: true });
    await banner.scrollIntoViewIfNeeded();
    await f.page.waitForTimeout(250);
    await f.page.screenshot({ path: filename, fullPage: false, animations: 'disabled' });
    console.log('SCREENSHOT ' + filename);
  }
});

test('prepare and repeated idle polling cannot erase or repeatedly spam the same failure', async f => {
  await f.setState(state({ auto: 'error', message: failure.message, last_failure: failure }));
  await f.setState(state({ auto: 'idle', message: '自动准备完成', last_failure: failure }));
  await f.setState(state({ auto: 'idle', message: '设备空闲', last_failure: failure }));
  const messages = await f.page.locator('#toastRegion .toast-message').allTextContents();
  assert.equal(messages.filter(message => /M4差36.77/.test(message)).length, 1);
  assert.equal(f.motionRequests().length, 0);
});

test('different failed run with the same cause is still reported', async f => {
  await f.setState(state({ last_failure: failure }));
  assert.match(await f.toast(), /移动到轨迹起点超时/);
  await f.page.locator('#toastRegion .toast-close').click();
  await f.page.waitForTimeout(220);
  const next = { ...failure, id: failure.id + '-next', run_id: failure.run_id + '-next' };
  await f.setState(state({ last_failure: next }));
  assert.match(await f.toast(), /移动到轨迹起点超时/);
});

test('acknowledged failure is retained as data but does not raise a new popup', async f => {
  await f.setState(state({ last_failure: { ...failure, acknowledged: true } }));
  assert.doesNotMatch(await f.toast(), /移动到轨迹起点超时/);
  assert.equal(f.motionRequests().length, 0);
});

runTests().catch(error => { console.error(error); process.exitCode = 1; });
