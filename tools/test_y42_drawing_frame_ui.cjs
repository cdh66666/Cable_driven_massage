const assert=require('node:assert/strict');
const {test,state,runTests,clearTests}=require('./test_y42_ui_offline.cjs');
clearTests();
test('hand drawing sends original board coordinates with explicit frame',async f=>{
  await f.draw();
  await f.page.evaluate(()=>runCustom());
  const r=f.requests.find(r=>r.url==='/api/path');
  assert(r,'hand drawing request absent');
  assert.equal(r.body.coordinate_frame,'workspace_normalized');
  assert.deepEqual(r.body.segments,[[{x:.3,y:.4},{x:.5,y:.6}]]);
});
test('frame rejects outside pointer and preserves inside coordinates',async f=>{
  const result=await f.page.evaluate(()=>{
    const r=$('canvas').getBoundingClientRect();
    const p=(x,y)=>canvasPoint({clientX:r.left+x*r.width,clientY:r.top+y*r.height});
    return [p(.05,.5),p(.2,.3),p(.95,.5)];
  });
  assert.equal(result[0],null);assert.equal(result[2],null);
  assert(Math.abs(result[1].x-.2)<1e-8);assert(Math.abs(result[1].y-.3)<1e-8);
});
test('edited speed survives stale polling and reaches path request',async f=>{
  await f.page.locator('#pathSpeed').fill('100');
  await f.page.locator('#pathSpeed').blur();
  await f.setState(state({calibration:{path_speed_mm_s:50}}));
  assert.equal(await f.page.locator('#pathSpeed').inputValue(),'100');
  await f.draw();await f.page.evaluate(()=>runCustom());
  assert.equal(f.requests.find(r=>r.url==='/api/path').body.speed,100);
});
runTests().catch(e=>{console.error(e);process.exitCode=1;});
