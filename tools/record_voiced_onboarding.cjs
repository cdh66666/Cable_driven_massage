const fs=require('node:fs'),path=require('node:path');
const {chromium}=require('C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const out=path.resolve(__dirname,'../qa/tutorial_landscape');
function duration(file){const b=fs.readFileSync(file);let rate=0;for(let p=12;p+8<=b.length;){const type=b.toString('ascii',p,p+4),n=b.readUInt32LE(p+4);if(type==='fmt ')rate=b.readUInt32LE(p+16);if(type==='data')return n/rate;p+=8+n+(n%2)}throw Error('WAV data missing')}
(async()=>{
 const durations=Array.from({length:7},(_,i)=>duration(path.join(out,`narration_${String(i+1).padStart(2,'0')}.wav`)));
 fs.writeFileSync(path.join(out,'narration_durations.json'),JSON.stringify(durations));
 const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe'});
 const context=await browser.newContext({viewport:{width:1920,height:1080},locale:'zh-CN',recordVideo:{dir:out,size:{width:1920,height:1080}}});
 const page=await context.newPage();let posts=0;
 await page.route('**/api/**',r=>{if(r.request().method()!=='GET'){posts++;return r.abort()}return r.continue()});
 await page.goto('http://127.0.0.1:8787/onboarding');
 const stage=async(i,fn)=>{const t=Date.now();await fn();await page.waitForTimeout(Math.max(0,durations[i]*1000-(Date.now()-t)));console.log('Chapter',i+1,'recorded',durations[i]);};
 await stage(0,async()=>{});
 await stage(1,async()=>{await page.locator('#connectNext').click();for(const b of await page.locator('.identity-demo').all()){await b.click();await page.waitForTimeout(durations[1]*180)}});
 await stage(2,async()=>{await page.locator('#identityNext').click();await page.waitForTimeout(1500);await page.locator('#baudDemoButton').click()});
 await stage(3,async()=>{await page.locator('#baudNext').click();for(const b of await page.locator('.direction-demo').all()){await b.click();await page.waitForTimeout(durations[3]*180)}});
 await stage(4,async()=>{await page.locator('#directionNext').click();await page.waitForTimeout(1800);await page.locator('#calibrationDemoButton').click()});
 await stage(5,async()=>{await page.locator('#loadPreviewButton').scrollIntoViewIfNeeded();await page.locator('#loadPreviewButton').click();await page.waitForTimeout(1500);await page.locator('#runDemoButton').click()});
 const v=page.video();await context.close();await v.saveAs(path.join(out,'voice_matched_onboarding.webm'));await browser.close();
 fs.writeFileSync(path.join(out,'voice_matched_onboarding_audit.json'),JSON.stringify({posts,mode:'offline demonstration',durations}));
 if(posts)throw Error('Unexpected POST');
})().catch(e=>{console.error(e);process.exit(1)});
