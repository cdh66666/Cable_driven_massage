const fs=require('node:fs'),path=require('node:path');
const {chromium}=require('C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..'),out=path.join(root,'qa/tutorial_landscape/real_lesson_20260908');fs.mkdirSync(out,{recursive:true});
(async()=>{
const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe'});
const context=await browser.newContext({viewport:{width:960,height:1080},locale:'zh-CN',recordVideo:{dir:out,size:{width:960,height:1080}}});
const page=await context.newPage(),timing={started_unix_ms:Date.now(),events:[]};
const log=(name,extra={})=>{timing.events.push({name,time:Date.now(),...extra});fs.writeFileSync(path.join(out,'screen_timing.json'),JSON.stringify(timing,null,2));console.log(name,flush=undefined)};
async function click(id){log('click',{id});await page.locator(id).scrollIntoViewIfNeeded();await page.locator(id).click();await page.waitForFunction(()=>typeof busy!=='undefined'&&!busy,{},{timeout:150000});const error=await page.evaluate(()=>window.lastLessonError);if(error)throw Error(id+': '+error);await page.waitForTimeout(600)}
async function chapter(name,n){log(name);await page.locator(`[data-step="${n}"]`).click();await page.waitForTimeout(700)}
try{
await page.goto('http://127.0.0.1:8787/onboarding');await page.waitForFunction(()=>window.lessonReady);await page.screenshot({path:path.join(out,'screen_start.png')});
if(process.argv.includes('--preview'))return;
log('connect');await click('#connect');
await chapter('identity',1);await click('#scanId');await click('#writeId');await click('#restartId');await click('#writeId');await click('#restartId');await click('#scanId');
await chapter('direction',2);for(const id of ['1','2','3','4']){log('motor_direction',{id});await page.locator('#motor').selectOption(id);await click('#testDir');await click('#flipDir');await click('#testDir');await click('#flipDir');}
await chapter('calibrate',3);await click('#calibrate');
await chapter('patterns',4);await click('#allPatterns');
log('words');await click('#loadWords');await page.screenshot({path:path.join(out,'xingmao_preview.png')});await page.waitForTimeout(1800);await click('#drawWords');
log('finish');await page.waitForTimeout(5500);
}catch(e){log('ERROR',{error:String(e)});throw e}
finally{fs.writeFileSync(path.join(out,'browser_api_audit.json'),JSON.stringify(await page.evaluate(()=>window.lessonAudit||[]),null,2));await page.screenshot({path:path.join(out,'screen_end.png')});const v=page.video();await context.close();await v.saveAs(path.join(out,process.argv.includes('--preview')?'preview.webm':'real_screen.webm'));await browser.close();log('closed')}
})().catch(e=>{console.error(e);process.exitCode=1});
