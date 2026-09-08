/**
 * Offline browser test for y42_onboarding.html.
 *
 * The browser never opens the real controller. A local fetch double returns
 * deterministic GET/POST responses, records every POST body, and rejects a
 * flow that skips a required confirmation or API stage.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

function findPlaywright(){
  for(const name of [process.env.PLAYWRIGHT_MODULE,'playwright','C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright'].filter(Boolean)){
    try{return require(name)}catch(error){if(error.code!=='MODULE_NOT_FOUND')throw error}
  }
  throw new Error('Playwright unavailable');
}
const {chromium}=findPlaywright();
const html=fs.readFileSync(path.join(__dirname,'y42_onboarding.html'),'utf8');

async function main(){
  const executablePath=process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe';
  const browser=await chromium.launch({headless:true,executablePath});
  const page=await browser.newPage({locale:'zh-CN',viewport:{width:1920,height:1080}});
  const postBodies=[];
  await page.setContent(html,{waitUntil:'load'});
  const clickAll=async selector=>{const n=await page.locator(selector).count();for(let i=0;i<n;i++)await page.locator(selector).nth(i).click()};
  const forceBlockedClick=async selector=>page.locator(selector).first().evaluate(element=>{element.disabled=false;element.click()});
  const waitFor=expression=>page.waitForFunction(expression,{timeout:5000});
  await page.exposeFunction('recordPost',body=>postBodies.push(body));

  assert.match(await page.locator('#modeBadge').innerText(),/离线演示/);
  assert.match(await page.locator('#identityWarning').innerText(),/重复 ID/);
  assert.equal(await page.locator('#connectNext').isDisabled(),false,'演示重复 ID 也必须能进入解决步骤');
  assert.equal(await page.locator('#prepareRealButton').isVisible(),false,'演示模式隐藏真实准备确认区');
  assert.equal(await page.locator('#onlyOneConnected').isVisible(),false,'演示模式隐藏逐台实机确认区');
  assert.equal(await page.locator('#runDemoButton').isDisabled(),true);

  // Exercise the novice route in order, starting from the first screen.
  await page.locator('#connectNext').click();
  await clickAll('.identity-demo');
  assert.equal(await page.locator('#identityNext').isDisabled(),false);
  await page.locator('#identityNext').click();
  await page.locator('#baudDemoButton').click();
  await page.locator('#baudNext').click();
  await clickAll('.direction-demo');
  await page.locator('#directionNext').click();
  await page.locator('#calibrationDemoButton').click();
  assert.match(await page.locator('#calibrationPill').innerText(),/通过/);
  await page.locator('#calibrationNext').click();
  await page.locator('#loadPreviewButton').click();
  await page.locator('#runDemoButton').click();
  await page.waitForTimeout(1600);
  assert.match(await page.locator('#eventLog').innerText(),/页面动画完成/);
  assert.match(await page.locator('#drawActionNote').innerText(),/不驱动电机/);
  assert.equal(postBodies.length,0,'演示流程不得发送 POST');

  await page.locator('#scenarioSelect').selectOption('mixedBaud');
  assert.match(await page.locator('#baudPill').innerText(),/混合/);
  assert.equal(await page.locator('#baudDemoButton').isDisabled(),false);
  await page.locator('#scenarioSelect').selectOption('directionUnset');
  assert.equal(await page.locator('#directionNext').isDisabled(),true);
  await page.locator('#scenarioSelect').selectOption('calibrationInvalid');
  assert.match(await page.locator('#calibrationPill').innerText(),/需要确认/);
  await page.locator('#scenarioSelect').selectOption('normal');
  assert.equal(await page.locator('#runDemoButton').isDisabled(),false);

  const mock = {
    prepared:false, controlUnlocked:false, readIndex:0, currentScanId:null, pendingVerify:null,
    hostBaud:921600, sourceBaud:115200,
    sourceIds:[11,12,13,14], targetIds:[1,2,3,4], directionReady:{},
    savedAttempts:0, calibration:{valid:false,session_local_verified:false,session_tension_held:false},
    auto:'idle', savedModel:true
  };
  await page.evaluate(({mockState})=>{
    window.__mockState=mockState;
    const original=window.fetch;
    window.fetch=async(input,options={})=>{
      const url=typeof input==='string'?input:input.url;
      const method=String(options.method||'GET').toUpperCase();
      const body=options.body?JSON.parse(options.body):{};
      const json=(data,status=200)=>new Response(JSON.stringify(data),{status,headers:{'Content-Type':'application/json'}});
      if(method==='GET' && url.endsWith('/api/ports'))return json({ok:true,ports:[{device:'COM-MOCK',label:'离线假端口'}]});
      if(method==='GET' && url.endsWith('/api/status'))return json({ok:true,motors:[1,2,3,4].map(id=>({id,online:true,rpm:0,phase_current:0}))});
      if(method==='GET' && url.endsWith('/api/auto'))return json({
        ok:true,auto:mockState.auto,control_unlocked:mockState.controlUnlocked,baud:mockState.hostBaud,
        message:'mock '+mockState.auto,saved_model_available:mockState.savedModel,
        direction_ready:mockState.directionReady,calibration:mockState.calibration
      });
      if(method==='POST' && url.endsWith('/api/command')){
        await window.recordPost(body);
        if(body.action==='prepare'){
          mockState.prepared=true;mockState.controlUnlocked=true;
          return json({ok:true,action:'prepare',message:'mock prepared'});
        }
        if(body.action==='scan'){
          if(mockState.pendingVerify!==null){
            const id=mockState.targetIds[mockState.pendingVerify];
            mockState.pendingVerify=null;
            return json({ok:true,action:'scan',found:[id]});
          }
          if(body.start===body.end && body.start===mockState.currentScanId){
            return json({ok:true,action:'scan',found:[mockState.currentScanId]});
          }
          mockState.currentScanId=mockState.sourceIds[mockState.readIndex];
          return json({ok:true,action:'scan',found:body.baud===mockState.sourceBaud?[mockState.currentScanId]:[]});
        }
        if(body.action==='baud-sweep'){
          mockState.readIndex+=1;
          return json({ok:true,action:'baud-sweep',recommended_baud:null,results:[{baud:mockState.sourceBaud,valid:[mockState.currentScanId]}]});
        }
        if(body.action==='set-id'){
          const index=mockState.sourceIds.indexOf(Number(body.old));
          if(index===-1)throw new Error('set-id old ID must be the ID just read');
          if(body.new!==mockState.targetIds[index])throw new Error('set-id target mismatch');
          mockState.sourceIds[index]=Number(body.new);
          mockState.pendingVerify=index;mockState.controlUnlocked=false;
          return json({ok:true,action:'set-id',accepted:true});
        }
        if(body.action==='baud-set')return json({ok:true,action:'baud-set',verified:true,target_baud:body.target_baud});
        if(body.action==='direction-test'){
          mockState.directionReady[String(body.id)]={completed:true,id:body.id};
          mockState.auto='idle';
          return json({ok:true,action:'direction-test',id:body.id});
        }
        if(body.action==='direction-save')return json({ok:true,action:'direction-save',id:body.id});
        if(body.action==='stop'){mockState.auto='idle';return json({ok:true,action:'stop'});}
        throw new Error('unexpected command '+body.action);
      }
      if(method==='POST' && url.endsWith('/api/auto')){
        await window.recordPost(body);
        if(body.action!=='calibrate')throw new Error('unexpected auto action');
        if(body.force_full===false){
          mockState.savedAttempts+=1;
          return json({ok:false,message:'mock 保存模型检查失败；不自动 full'},400);
        }
        if(body.force_full!==true)throw new Error('full calibration requires force_full=true');
        mockState.calibration={valid:true,session_local_verified:true,session_tension_held:true};
        mockState.auto='idle';
        return json({ok:true,action:'full-four-corner-calibration'});
      }
      if(method==='POST' && url.endsWith('/api/path')){
        await window.recordPost(body);
        if(body.action!=='path'||body.confirmed!==true)throw new Error('path safety body mismatch');
        if(!Array.isArray(body.segments)||body.segments[0].length<2)throw new Error('path points missing');
        mockState.auto='idle';return json({ok:true,message:'mock path accepted'});
      }
      return original(input,options);
    };
  },{mockState:mock});
  await page.evaluate(()=>window.y42Onboarding.setMode('real'));
  await page.waitForTimeout(100);
  assert.match(await page.locator('#modeBadge').innerText(),/真实服务/);
  assert.match(await page.locator('#modeNote').innerText(),/POST/);
  assert.equal(await page.locator('#prepareRealButton').isVisible(),true);
  assert.equal(postBodies.length,0,'切换真实模式只允许 GET');
  assert.equal(await page.locator('#prepareRealButton').isDisabled(),true,'四台准备应在身份/速率处理后才开放');
  await page.locator('#connectNext').click();

  // Identity writes require one-motor confirmation, source read, ACK, power
  // cycle acknowledgement, and target readback. The first write attempt is
  // deliberately made before those gates and must be blocked.
  await forceBlockedClick('#identityRows .identity-write');
  assert.equal(postBodies.length,0,'写 ID 前置条件不足不得 POST');
  await page.locator('#onlyOneConnected').check();
  await page.locator('#identityRows .identity-read').first().click();
  await waitFor(()=>window.y42Onboarding.state.identityRecords[0].read);
  assert.deepEqual(postBodies.map(body=>body.action),['scan','baud-sweep','scan'],'主机速率错误时应先 baud-sweep 再按发现速率回读');
  assert.equal(postBodies[2].baud,115200);
  await page.locator('#identityRows .identity-write').first().click();
  await waitFor(()=>window.y42Onboarding.state.identityRecords[0].writeAccepted);
  assert.equal(postBodies.at(-1).action,'set-id');
  assert.equal(postBodies.at(-1).old,11);assert.equal(postBodies.at(-1).new,1);
  await forceBlockedClick('#identityRows .identity-verify');
  assert.equal(postBodies.at(-1).action,'set-id','未确认断电重启不得回读');
  await page.locator('#reboot-confirm-0').check();
  await page.locator('#identityRows .identity-verify').first().click();
  await waitFor(()=>window.y42Onboarding.state.identityRecords[0].verified);
  assert.equal(postBodies.at(-1).action,'scan');
  assert.equal(await page.locator('#identityRows .identity-verify').first().isDisabled(),false);

  for(let index=1;index<4;index++){
    await page.locator('#identityRows .identity-read').nth(index).click();
    await page.waitForFunction(index => window.y42Onboarding.state.identityRecords[index].read,index);
    await page.locator('#identityRows .identity-write').nth(index).click();
    await page.waitForFunction(index => window.y42Onboarding.state.identityRecords[index].writeAccepted,index);
    await page.locator('#reboot-confirm-'+index).check();
    await page.locator('#identityRows .identity-verify').nth(index).click();
    await page.waitForFunction(index => window.y42Onboarding.state.identityRecords[index].verified,index);
  }
  assert.equal(await page.locator('#identityNext').isDisabled(),false);
  await page.locator('#identityNext').click();

  // baud-set is closed until all identity records and its own confirmation.
  await forceBlockedClick('#baudDemoButton');
  assert.equal(postBodies.at(-1).action,'scan','波特率确认未勾选不得 POST');
  await page.locator('#baudConfirm').check();
  await page.locator('#baudDemoButton').click();
  await waitFor(()=>window.y42Onboarding.state.baudDone);
  assert.equal(postBodies.at(-1).action,'baud-set');
  assert.equal(postBodies.at(-1).target_baud,115200);
  await page.locator('#baudNext').click();

  // set-id locks the controller, so explicitly prepare after identity/baud
  // and before motion. Preparation itself also requires its checkbox.
  await forceBlockedClick('#prepareRealButton');
  assert.equal(postBodies.length,18,'未勾选准备确认不得 POST');
  await page.locator('#prepareSafetyConfirm').check();
  await page.locator('#prepareRealButton').click();
  await page.waitForTimeout(40);
  assert.equal(postBodies.at(-1).action,'prepare');
  await page.locator('#directionConfirm').check();
  await page.locator('#directionRows .direction-test').first().click();
  await waitFor(()=>window.y42Onboarding.state.directionTested[0]);
  assert.equal(postBodies.at(-1).action,'direction-test');
  assert.match(await page.locator('#directionRows').innerText(),/45°测试完成/);
  await page.locator('#directionRows .direction-save[data-reverse="false"]').first().click();
  await waitFor(()=>window.y42Onboarding.state.directions[0]);
  assert.equal(postBodies.at(-1).action,'direction-save');
  for(let index=1;index<4;index++){
    await page.locator('#directionRows .direction-test').nth(index).click();
    await page.waitForFunction(index => window.y42Onboarding.state.directionTested[index],index);
    await page.locator('#directionRows .direction-save[data-reverse="false"]').nth(index).click();
    await page.waitForFunction(index => window.y42Onboarding.state.directions[index],index);
  }
  assert.equal(await page.locator('#directionNext').isDisabled(),false);
  await page.locator('#directionNext').click();

  // A saved-model failure must stop at the saved path; it must not turn into a
  // hidden full-calibration POST. Full calibration is a separate confirmation.
  await page.locator('#savedCalibrationConfirm').check();
  await page.locator('#calibrationSavedButton').click();
  await page.waitForFunction(()=>window.y42Onboarding.state.logs.some(message=>message.includes('保存模型核验未完成')), {timeout:5000});
  assert.equal(await page.evaluate(()=>window.__mockState.savedAttempts),1);
  assert.equal(postBodies.filter(body=>body.action==='calibrate').length,1);
  assert.equal(postBodies.at(-1).force_full,false);
  await page.locator('#fullCalibrationConfirm').check();
  await page.locator('#calibrationFullButton').click();
  await waitFor(()=>window.y42Onboarding.state.calibrationDone);
  assert.equal(postBodies.at(-1).force_full,true);
  assert.match(await page.locator('#calibrationPill').innerText(),/通过/);
  await page.locator('#calibrationNext').click();

  // Preview is always safe; actual path needs the final confirmation.
  await page.locator('#loadPreviewButton').click();
  await forceBlockedClick('#runDemoButton');
  assert.equal(postBodies.filter(body=>body.action==='path').length,0);
  await page.locator('#drawConfirm').check();
  await page.locator('#runDemoButton').click();
  await page.waitForFunction(()=>window.y42Onboarding.state.logs.some(message=>message.includes('真实 /api/path 已受理')), {timeout:5000});
  assert.equal(postBodies.at(-1).action,'path');
  await page.locator('#realStopButton').click();
  await page.waitForTimeout(40);
  assert.equal(postBodies.at(-1).action,'stop');

  await browser.close();
  console.log('PASS y42 onboarding: novice navigation, hidden demo confirmations, prepare, scan->baud-sweep->set-id->power-cycle->scan, baud-set, direction-test/save, saved calibration no-fallback, explicit full calibration, path, and stop');
}
main().catch(error=>{console.error(error.stack||error);process.exitCode=1});
