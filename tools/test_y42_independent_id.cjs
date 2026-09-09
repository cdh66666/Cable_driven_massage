const fs=require('fs'),assert=require('assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});try{
const page=await browser.newPage();const calls=[];
await page.route('**/*',r=>{const body=r.request().postDataJSON();if(!body&&r.request().url().includes('/onboarding'))return r.fulfill({contentType:'text/html',body:fs.readFileSync(__dirname+'/y42_real_lesson.html','utf8')});if(body)calls.push(body);return r.fulfill({contentType:'application/json',body:JSON.stringify(body?{ok:true,found:body.start===1?[1]:[],message:'ACK'}:{ok:true,auto:'error',control_unlocked:false})})});
await page.goto('http://test.invalid/onboarding#id');await page.click('#writeId');await page.waitForFunction(()=>document.getElementById('idResult').textContent==='ACK');
assert.deepEqual(calls.map(x=>x.action),['scan','scan','set-id']);assert.equal(calls[2].baud,115200);assert.equal(calls[2].old,1);assert.equal(calls[2].new,4);console.log('PASS: locked/error, uncalibrated single-motor ID update needs no arm/prepare/all-disable');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
