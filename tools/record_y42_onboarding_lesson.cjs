/* Background browser recording. Every POST is blocked: factory scenes are demonstration. */
const path=require('node:path');
const fs=require('node:fs');
const {chromium}=require('C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{
 const out=path.resolve(__dirname,'../qa/tutorial_landscape');fs.mkdirSync(out,{recursive:true});
 const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe'});
 const context=await browser.newContext({viewport:{width:1920,height:1080},locale:'zh-CN',recordVideo:{dir:out,size:{width:1920,height:1080}}});
 const page=await context.newPage();const blocked=[];
 await page.route('**/api/**',route=>{if(route.request().method()!=='GET'){blocked.push(route.request().url());return route.abort();}return route.continue();});
 await page.goto('http://127.0.0.1:8787/onboarding');
 await page.screenshot({path:path.join(out,'onboarding_preview.png')});
 const show=async id=>{await page.locator(id).scrollIntoViewIfNeeded();await page.waitForTimeout(3500)};
 await show('#step-connect');await page.locator('#connectNext').click();
 await show('#step-identity');
 for(const button of await page.locator('.identity-demo').all()){await button.click();await page.waitForTimeout(1500)}
 await page.locator('#identityNext').click();await show('#baudDemoButton');await page.locator('#baudDemoButton').click();await page.waitForTimeout(2500);
 await show('.direction-demo >> nth=0');for(const button of await page.locator('.direction-demo').all()){await button.click();await page.waitForTimeout(1200)}
 await show('#calibrationDemoButton');await page.locator('#calibrationDemoButton').click();await page.waitForTimeout(3000);
 await show('#loadPreviewButton');await page.locator('#loadPreviewButton').click();await page.locator('#runDemoButton').click();await page.waitForTimeout(5000);
 const video=page.video();await context.close();await video.saveAs(path.join(out,'新手网页演示_无配音草稿.webm'));await browser.close();
 fs.writeFileSync(path.join(out,'onboarding_recording_audit.json'),JSON.stringify({mode:'Demo only; no physical factory reset or motion',blockedPosts:blocked},null,2));
 console.log('Recorded 1920x1080 demo lesson; blocked POST count:',blocked.length);
})().catch(e=>{console.error(e);process.exit(1)});
