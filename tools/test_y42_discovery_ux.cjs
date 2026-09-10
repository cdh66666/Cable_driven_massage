// 针对用户反馈的两个场景做端到端验证：
// A. 已连接（control_unlocked）状态下刷新页面：不扫总线也要立刻显示四条设备条（蓝色条）。
// B. 点“重新扫描”：第一遍 921600 就把 ID 1..4 全部亮出（不再先扫 ID 0–1 的九档波特率）；
//    扫描进行中再点一次不会重启扫描。
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/admin/.workbuddy/binaries/node/workspace/node_modules/playwright-core');
const tools=__dirname;
const html=fs.readFileSync(path.join(tools,'y42_pc_direct_web.py'),'utf8').split("PAGE = r'''")[1].split("'''")[0]+'<script>'+fs.readFileSync(path.join(tools,'y42_four_pages.js'),'utf8')+'</script>';
(async()=>{const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
try{
// ---------- 场景 A：已连接状态，刷新后无需扫描即显示四台 ----------
  const page=await browser.newPage(),calls=[],errors=[];page.on('pageerror',e=>errors.push(e.message));
  const telemetry={1:{id:1,online:true,position:1},2:{id:2,online:true,position:2},3:{id:3,online:true,position:3},4:{id:4,online:true,position:4}};
  await page.route('**/*',r=>{const url=new URL(r.request().url());
    if(url.pathname==='/')return r.fulfill({contentType:'text/html',body:html});
    const b=r.request().postDataJSON();if(b)calls.push(b);
    if(url.pathname==='/api/auto')return r.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,auto:'idle',control_unlocked:true,port:'COM25',calibration:{},message:'已连接',telemetry})});
    let data={ok:true,auto:'idle',calibration:{},control_unlocked:true,motors:Object.values(telemetry),ports:[{device:'COM25',label:'USB-SERIAL CH340 (COM25)',likely_adapter:true}],recommended:'COM25'};
    if(b?.action==='scan')data={ok:true,found:[]};
    return r.fulfill({contentType:'application/json',body:JSON.stringify(data)})});
  await page.goto('http://seed.invalid/');await page.waitForFunction(()=>document.querySelectorAll('#liveIds button').length>=4,null,{timeout:8000});
  const bars=await page.locator('#liveIds button').allTextContents();
  assert.deepEqual(bars.map(t=>t.match(/ID (\d)/)[1]),['1','2','3','4'],'应按 ID 升序显示四条');
  const scanCallsBefore=calls.filter(c=>c.action==='scan').length;
  assert.equal(scanCallsBefore,0,'已连接状态不应为了显示设备而发扫描请求');
  assert((await page.locator('#idState').textContent()).includes('已连接'),'idState 应说明这是实时遥测设备');
  console.log('PASS A: 已连接刷新页面立即显示 4 条设备（无扫描请求）');

// ---------- 场景 B：重新扫描一遍点亮全部，扫描中重复点击不重启 ----------
  await page.evaluate(()=>localStorage.removeItem('y42.portChosen'));
  // 让扫描只在 921600 有回包（模拟已统一 921600 的四台电机）
  await page.unroute('**/*');
  await page.route('**/*',r=>{const url=new URL(r.request().url());
    if(url.pathname==='/')return r.fulfill({contentType:'text/html',body:html});
    const b=r.request().postDataJSON();if(b)calls.push(b);
    if(url.pathname==='/api/auto')return r.fulfill({contentType:'application/json',body:JSON.stringify({ok:true,auto:'idle',control_unlocked:false,port:'COM25',calibration:{},message:'',telemetry:{}})});
    let data={ok:true,auto:'idle',calibration:{},control_unlocked:false,motors:[],ports:[{device:'COM25',label:'USB-SERIAL CH340 (COM25)',likely_adapter:true}],recommended:'COM25'};
    if(b?.action==='scan'){data={ok:true,found:b.baud===921600?[1,2,3,4]:[]};if(b.baud!==921600)return setTimeout(()=>r.fulfill({contentType:'application/json',body:JSON.stringify(data)}),400)}
    return r.fulfill({contentType:'application/json',body:JSON.stringify(data)})});
  await page.reload();await page.waitForFunction(()=>document.querySelectorAll('#liveIds button').length>=4,null,{timeout:8000});
  const reloadScans=calls.filter(c=>c.action==='scan');
  assert(reloadScans[0]&&reloadScans[0].baud===921600&&reloadScans[0].start===1&&reloadScans[0].end===10,'第一遍就应是 921600 · ID 1–10 全范围');
  assert.equal(reloadScans.filter(c=>c.baud===921600).length,1,'921600 一遍即可点亮全部，不应重复');
  // 扫描进行中（低速档还在扫）再点一次：不应重启
  await page.click('#scanButton');
  const clickState=await page.locator('#idState').textContent();
  const n921AtClick=calls.filter(c=>c.action==='scan'&&c.baud===921600).length;
  await page.waitForTimeout(900);
  const n921After=calls.filter(c=>c.action==='scan'&&c.baud===921600).length;
  assert.equal(await page.locator('#liveIds button').count(),4,'扫描中四条设备保持显示');
  assert(clickState.includes('正在扫描中'),'重复点击应提示正在扫描，实际：'+clickState);
  assert.equal(n921After,n921AtClick,'扫描中重复点击不应重启扫描（重启会再次发出 921600 扫描）');
  console.log('PASS B: 重新扫描第一遍 921600 全亮；扫描中重复点击不重启（921600 扫描次数不变）');
  assert.deepEqual(errors,[],'无页面错误');
  console.log('PASS: no page errors');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
