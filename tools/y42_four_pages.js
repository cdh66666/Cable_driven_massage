// Presentation only: keep existing controls and drawing handlers alive.
(()=>{
// Operational commands always use 921600. Discovery/migration requests carry
// their temporary probe baud explicitly and must never change this UI setting.
const setOperationalBaud=syncBaudSelects;
syncBaudSelects=()=>setOperationalBaud(921600);
syncBaudSelects();
// Older running controllers still route GET status through mutable scan state.
// Explicit operational rate also makes this UI fix effective without restart.
// 端口选择先于任何请求生效：所有请求都带上用户选定的串口（未选则让后台自动挑）。
const chosenPort=()=>{const el=document.getElementById('quickPort');return el&&el.value?el.value:''};
const portArg=()=>{const value=chosenPort();return value?{port:value}:{}};
const originalGetJson=getJson;
getJson=(url,timeoutMs=5000)=>url.startsWith('/api/status')?post({action:'status',baud:921600,...portArg()},'/api/command',timeoutMs,true):originalGetJson(url,timeoutMs);
const style=document.createElement('style');style.textContent=`
body{padding-bottom:100px!important}main{display:block!important;max-width:1100px;margin:auto}
.workflow-page{display:none!important}.workflow-page.active{display:block!important}
.workflow-page>.card{margin:18px 0}#workflowNav{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);display:flex;gap:6px;padding:8px;background:#fff;box-shadow:0 8px 40px #2343;border:1px solid #dbe6ee;border-radius:20px;z-index:1000;width:min(94%,620px)}
#workflowNav button{flex:1;min-width:0;border:0;border-radius:13px;padding:13px 6px;background:transparent;color:#62758b}#workflowNav button.active{background:#147e57;color:white}
#performancePanel,.hero,.setup-stack,#floatingPrepare,#floatingCalibration{display:none!important}
body .workflow-page #idSection,body .workflow-page #prepareSection,body .workflow-page #calibrationSection,body .workflow-page #directionSection,body .workflow-page #drawSection,body .workflow-page #quickBaud{display:block!important}
.workflow-page:not(.active) *{pointer-events:none}
#prepareSection>.quick-connect{display:none!important}#prepareSection>.muted,#calibrationSection>.muted,#drawSection>.muted{display:none!important}
.workflow-note{color:#587086;line-height:1.7}.workflow-primary{background:#147e57;color:white;border:0;padding:16px 28px;border-radius:14px;font-size:20px}
#idState.scanning{border-color:#9cc4f0;background:#eef5ff;color:#1c4e89;font-weight:650}
#liveIds button{border-radius:12px}
`;document.head.append(style);
const main=document.querySelector('main');const pages=[];
for(const [i,title] of ['修改 ID','连接','校准','绘制轨迹'].entries()){const page=document.createElement('div');page.className='workflow-page';page.id='workflowPage'+i;main.append(page);pages.push(page)}
const move=(id,index)=>{const node=document.getElementById(id);if(node)pages[index].append(node)};
move('idSection',0);move('prepareSection',1);move('directionSection',2);move('calibrationSection',2);move('drawSection',3);
const extras=document.createElement('details');extras.innerHTML='<summary>高级设置与日志</summary>';pages[3].append(extras);
for(const node of [...main.children])if(!pages.includes(node)){node.hidden=true;node.style.display='none'}
move('logDetails',3);
const idSection=$('idSection');idSection.querySelector('.muted').textContent='只接要修改的单台电机。不需要四台在线、连接准备或校准。填写当前 ID 和新 ID 后保存；断电重启后扫描确认。';
const idAdvanced=document.createElement('details');idAdvanced.innerHTML='<summary>电机当前波特率（出厂一般为115200）</summary>';idAdvanced.append($('quickBaud'));idSection.insertBefore(idAdvanced,idSection.children[3]);
idAdvanced.hidden=true;idAdvanced.style.display='none';
$('scanStart').value=0;$('scanEnd').value=10;$('scanStart').closest('.row').style.display='none';
$('scanButton').textContent='重新扫描';
const live=document.createElement('div');live.innerHTML='<p id="livePorts" class="workflow-note">正在查找串口…</p><div id="liveIds"></div><p class="workflow-note">自动扫描所有支持的波特率、ID 1–10（0 为广播地址，不查询）。仅改 ID 页自动扫描；运动中暂停。单台设备自动填入当前 ID，目标 ID 由你填写。</p>';idSection.insertBefore(live,$('scanButton'));
let detected=new Map(),discoveryBusy=false,idWriting=false,portRows=[],scanGeneration=0,selectedDevice=null;
// 端口选择器元素引用：一旦节点被 move 进“尚未挂载”的容器就会脱离文档，
// 此时 document.getElementById 会立刻返回 null（这正是之前 portChoice 不显示的根因）。
let portSelect=null,portHintEl=null;
// 端口选择：用户手动选过就尊重他的选择，否则默认选中看起来像 USB 转换器的那个。
let portChosen=localStorage.getItem('y42.portChosen')==='1',portRecommended='';
const scanTargets=()=>{const selected=port(),list=selected?portRows.filter(p=>p.device===selected):[...portRows].sort((a,b)=>(b.likely_adapter?1:0)-(a.likely_adapter?1:0));return list.length?list:[...portRows]};
// 自动扫描默认开启。scanPaused 只在本页会话内有效，绝不持久化：
// 否则上一次“一键连接”留下的暂停标记会在下次打开页面时继续生效，
// 表现就是“必须自己点一下重新扫描才开始扫描”。
try{localStorage.removeItem('y42.scanPaused')}catch(e){}
let scanPaused=false;
function pauseDiscovery(){scanPaused=true;scanGeneration++;$('scanButton').textContent='重新扫描（退出连接）'}
$('oldId').readOnly=true;$('oldId').title='自动选择第一个发现的设备，也可点击上方设备切换';
idSection.querySelector('button[onclick="setId()"]').textContent='保存 ID 并统一到921600';
const discoveryActive=()=>!scanPaused&&!latestAuto.control_unlocked&&pages.slice(0,2).some(p=>p.classList.contains('active'))&&!connecting&&!idWriting&&['idle','error'].includes(latestAuto.auto);
function showDetected(){const now=Date.now(),rows=[...detected.values()].filter(r=>portRows.some(p=>p.device===r.port)).sort((a,b)=>a.port===b.port?a.id-b.id:a.port<b.port?-1:1),key=r=>`${r.port}/${r.baud}/${r.id}`;if(!rows.some(r=>key(r)===selectedDevice))selectedDevice=rows.length?key(rows[0]):null;$('liveIds').replaceChildren();if(!rows.length)$('liveIds').textContent=portRows.length?'尚未收到电机回包，正在逐档扫描。串口在线不等于电机在线。':'未连接串口';for(const row of rows){const b=document.createElement('button'),selected=key(row)===selectedDevice;b.textContent=`${selected?'✓ 当前选中 · ':''}ID ${row.id} · ${row.port} · ${row.baud} · ${Math.round((now-row.seen)/1000)}秒前`;b.setAttribute('aria-pressed',String(selected));b.onclick=()=>{selectedDevice=key(row);showDetected()};$('liveIds').append(b);if(selected){$('oldId').value=row.id;if(!portChosen)syncPortSelects(row.port);syncBaudSelects(pages[1].classList.contains('active')?921600:row.baud)}}if(!connecting&&!scanPaused)for(const id of [1,2,3,4]){const el=$('connectionId'+id),matches=rows.filter(r=>r.id===id);if(el)el.textContent=`ID ${id} · `+(matches.length?matches.map(r=>`${r.baud} · ${Math.round((now-r.seen)/1000)}秒前回包`).join(' / '):'尚无回包')}}
function renderPortSelect(){const select=portSelect||$('quickPort');if(!select)return;const previous=select.value;select.innerHTML='';const auto=document.createElement('option');auto.value='';auto.textContent=portRecommended?`自动选择（推荐 ${portRecommended}）`:'自动选择可用串口';select.appendChild(auto);for(const p of portRows){const option=document.createElement('option');option.value=p.device;option.textContent=p.label+(p.likely_adapter?'':'（非 USB 串口）');select.appendChild(option)}if(portRows.some(p=>p.device===previous))select.value=previous}
function updatePortHint(){const hint=portHintEl||$('portHint');if(!hint)return;if(!portRows.length){hint.textContent='未发现任何串口：请检查 USB-RS485 转换器驱动与 USB 连接。';return}const online='串口在线：'+portRows.map(p=>p.device).join('、')+'。';const selected=port(),row=portRows.find(p=>p.device===selected);if(selected&&row)hint.textContent=`${online}当前使用 ${row.label}${row.likely_adapter?'':'（注意：它看起来不是 USB 串口转换器）'}，只在这个串口上扫描和连接。`;else if(selected)hint.textContent=`${online}所选 ${selected} 已不在线，请重新选择。`;else hint.textContent=`${online}未指定端口：将在所有串口上逐个扫描。`}
async function pollPortsOnce(){try{const data=await getJson('/api/ports');portRows=data.ports||[];portRecommended=data.recommended||'';renderPortSelect();
// 恢复上次手动选择的串口；该串口不在线时回退推荐串口，避免每次启动都退回“全部串口”慢速扫描（COM1 这类主板串口每档要多等 1 秒以上）。
if(!port()&&portRows.length){const stored=localStorage.getItem('y42.port')||'';const want=(portChosen&&stored&&portRows.some(p=>p.device===stored))?stored:(portRecommended&&portRows.some(p=>p.device===portRecommended)?portRecommended:'');if(want)syncPortSelects(want)}
$('livePorts').textContent=(portRows.length?'串口在线：'+portRows.map(p=>p.device).join('、'):'未发现串口，请接入 USB-RS485')+(port()?`　当前端口：${port()}`:'　当前端口：自动（全部串口）')+(scanPaused?'　自动扫描：已暂停（连接中）':'　自动扫描：开启');for(const [key,r] of detected)if(!portRows.some(p=>p.device===r.port))detected.delete(key);updatePortHint();showDetected()}catch(e){$('livePorts').textContent='串口状态读取失败：'+e.message}}
async function pollPorts(){await pollPortsOnce();setTimeout(pollPorts,2000)}
// 已连接状态下直接用实时遥测恢复设备条：一键连接成功后自动扫描是停止的，
// 此时刷新页面 detected 为空，改 ID 页会一条设备条都没有，用户会误以为“没扫到”。
// 只增不删：遥测不在线的旧条目保留（它的“秒前”会自然变老），避免串口瞬时抖动把列表清空。
function seedFromTelemetry(j){if(scanPaused||discoveryBusy||idWriting||!latestAuto.control_unlocked)return;const motors=(j&&Array.isArray(j.motors)&&j.motors.length)?j.motors:(j&&j.telemetry&&typeof j.telemetry==='object'?Object.values(j.telemetry):[]);if(!motors.length)return;const portName=port()||portRecommended||(j&&typeof j.port==='string'?j.port:'')||'';if(!portName)return;const online=motors.filter(m=>m&&m.online===true&&Number.isInteger(+m.id)&&+m.id>=1);if(!online.length)return;let changed=false;for(const m of online){const entryKey=`${portName}/921600/${+m.id}`,cur=detected.get(entryKey);if(cur)cur.seen=Date.now();else{detected.set(entryKey,{port:portName,baud:921600,id:+m.id,seen:Date.now()});changed=true}}if(changed){showDetected();$('idState').textContent=`已连接：显示实时遥测在线的 ${detected.size} 台（固定921600）。点「重新扫描」可重新扫描总线`}}
const previousRenderAuto=renderAuto;renderAuto=function(j){try{seedFromTelemetry(j)}catch(e){}previousRenderAuto(j)};
async function discover(){if(discoveryBusy||!discoveryActive())return;discoveryBusy=true;scanBusy=true;const generation=scanGeneration;const idState=$('idState');idState.classList.add('scanning');try{
const rates=[921600,115200,...SUPPORTED_BAUDS.filter(b=>b!==921600&&b!==115200)];
// 波特率优先、ID 1–10 一次扫全：第一遍（921600，正常通信速率）约 2 秒就能把
// ID 1..4 全部亮出来，而不是像旧顺序那样先把 ID 0–1 的九档波特率扫完
// （实测 20 秒以上），用户会误以为只扫到 ID 1。
for(const [rateIndex,rate] of rates.entries())for(const p of scanTargets()){
if(!discoveryActive()||generation!==scanGeneration)return;
idState.textContent=`正在扫描 ${p.device} · ${rate}（第 ${rateIndex+1}/${rates.length} 档波特率）· 已发现 ${detected.size} 台`;
const result=await post({action:'scan',port:p.device,baud:rate,start:1,end:10},'/api/command',20000,true);
for(const [key,r] of detected)if(r.port===p.device&&r.baud===rate)detected.delete(key);
for(const id of result.found||[])detected.set(`${p.device}/${rate}/${id}`,{port:p.device,baud:rate,id,seen:Date.now()});showDetected();
idState.textContent=`本轮已发现 ${detected.size} 台：${[...detected.values()].sort((a,b)=>a.id-b.id).map(r=>`ID ${r.id}@${r.baud}`).join('、')||'暂无'} · 继续检查其他波特率`;
}$('idState').textContent=detected.size?`扫描完成，发现 ${detected.size} 个地址；2秒后继续扫描`:`扫描完成但没有收到任何电机回包。若长时间如此，点「连接」页的「串口自检（只读）」确认这个串口是否真的收到总线信号`;
}catch(e){$('idState').textContent='扫描未完成：'+e.message}finally{try{if(port())await post({action:'status',baud:921600},'/api/command',5000,true)}catch(e){}discoveryBusy=false;scanBusy=false;$('idState').classList.remove('scanning')}}
async function discoveryLoop(){await discover();setTimeout(discoveryLoop,1000)}
$('scanButton').onclick=async()=>{if(connecting||idWriting||!['idle','error'].includes(latestAuto.auto)){showToast('请先停止当前操作，再扫描','info',4000);return}
// 扫描本来就在进行时不重启：重启会丢掉当前进度、从头再扫一遍，
// 表现就是“点一次只多显示一台”。已发现的设备始终保留显示。
if(discoveryBusy){$('idState').textContent=`正在扫描中（已发现 ${detected.size} 台），扫完自动继续，无需重复点击`;showToast(`正在扫描中，已发现 ${detected.size} 台`,'info',3000);return}
try{await post({armed:false},'/api/arm');controlsLocked=true;sessionSafetyConfirmed=false;latestAuto.control_unlocked=false;updateControlLock();scanPaused=false;scanGeneration++;$('scanButton').textContent='重新扫描';void discover()}catch(e){$('idState').textContent=e.message}};
setId=async()=>{if(idWriting)return;idWriting=true;scanGeneration++;try{
while(discoveryBusy)await new Promise(r=>setTimeout(r,100));const next=+$('newId').value,device=detected.get(selectedDevice);if(!device)throw Error('请等待发现设备，或点击列表选择设备');if(!Number.isInteger(next)||next<1||next>255)throw Error('目标 ID 必须为 1–255 整数');if(!['idle','error'].includes(latestAuto.auto))throw Error('请先停止运动，再修改 ID');
const old=device.id,args={port:device.port,baud:device.baud};
if(next!==old){if([...detected.values()].some(r=>r.port===device.port&&r.id===next))throw Error('目标 ID 已占用，请更换目标 ID');const target=await post({action:'scan',...args,start:next,end:next},'/api/command',10000,true);if(target.found?.length)throw Error('目标 ID 已占用，请更换目标 ID')}
if(device.baud!==921600){$('idState').textContent=`正在将 ID ${old} 统一为921600…`;const rate=await post({action:'baud-set-single',...args,id:old,target_baud:921600,confirmed:true},'/api/command',30000);if(!rate.verified)throw Error('波特率写入未确认，未继续修改 ID');detected.delete(selectedDevice);device.baud=921600;selectedDevice=`${device.port}/921600/${old}`;detected.set(selectedDevice,device);args.baud=921600;showDetected()}
if(next===old){const check=await post({action:'scan',...args,start:old,end:old},'/api/command',10000,true);if(!check.found?.includes(old))throw Error('921600 回读未确认，请重新扫描');$('idState').textContent=`已完成：ID ${old} 保持不变 · 921600 回读成功`}
else{const target=await post({action:'scan',...args,start:next,end:next},'/api/command',10000,true);if(target.found?.length)throw Error('921600 下目标 ID 已占用；波特率已更新，编号未改');const response=await post({action:'set-id',...args,old,new:next,confirmed:true});$('idState').textContent='波特率已统一921600；'+response.message;detected.delete(selectedDevice);selectedDevice=null;showDetected()}
}catch(e){$('idState').textContent=e.message;showToast(e.message,'error',7000)}finally{idWriting=false}};
// 「连接」页的端口选择器：显示全部串口、推荐 USB 转换器、支持刷新与只读自检。
const portStyle=document.createElement('style');portStyle.textContent=`
#portChoice{display:flex;flex-wrap:wrap;align-items:center;gap:10px;margin:4px 0 0;padding:14px;border:1px solid #dbe6ee;border-radius:14px;background:#f7fafc}
#portChoice>label{font-weight:700;color:#18344c;min-width:126px}
#portChoice>select{flex:1 1 250px;min-width:210px;height:52px;font-size:16px}
#portChoice>button{height:52px;min-width:116px}
#portHint{flex:1 1 100%;margin:4px 0 0;color:#3c5a75}
`;document.head.append(portStyle);
const connect=$('prepareSection');
const portRow=document.createElement('div');portRow.id='portChoice';
const portLabel=document.createElement('label');portLabel.textContent='USB-RS485 串口';
const portRefresh=document.createElement('button');portRefresh.type='button';portRefresh.className='ghost';portRefresh.textContent='刷新串口';
const portDiag=document.createElement('button');portDiag.type='button';portDiag.className='ghost';portDiag.textContent='串口自检（只读）';
const portHint=document.createElement('p');portHint.id='portHint';portHint.className='workflow-note';portHint.textContent='正在读取串口…';
// 顺序很关键：先把空的 portRow 插入已挂载的页面，再把 quickPort 移进去；
// 否则 quickPort 会随未挂载的 portRow 一起脱离文档，后续 getElementById 全部拿不到。
// 位置：放在顶部那张“串口在线 / 扫描状态”卡片（改 ID / 连接 两页共用）的最上面。
live.insertBefore(portRow,live.firstChild);
portSelect=$('quickPort');portHintEl=portHint;
portRow.append(portLabel,portSelect,portRefresh,portDiag,portHint);
if(portSelect)portSelect.addEventListener('change',()=>{portChosen=true;localStorage.setItem('y42.portChosen','1');localStorage.setItem('y42.port',port());updatePortHint();post({action:'set-port',port:port()},'/api/command',8000,true).catch(()=>{});discoveryActive()&&showDetected()});
portRefresh.onclick=async()=>{portRefresh.disabled=true;portRefresh.textContent='刷新中…';try{await pollPortsOnce()}finally{portRefresh.disabled=false;portRefresh.textContent='刷新串口'}};
portDiag.onclick=async()=>{if(portDiag.disabled)return;portDiag.disabled=true;const target=port()||'（自动选择的串口）';portHintEl.textContent=`正在自检 ${target}：只发送状态查询，不发运动指令…`;try{const result=await post({action:'port-diagnose',...portArg()},'/api/command',120000);const hits=Object.keys(result.found||{}).length;portHintEl.textContent=result.message;showToast(hits?'串口自检通过':'串口自检：没有读到总线信号',hits?'success':'error',10000)}catch(e){portHintEl.textContent='自检失败：'+e.message;showToast(e.message,'error',8000)}finally{portDiag.disabled=false}};
void pollPorts();setTimeout(discoveryLoop,500);
connect.querySelector('h2').textContent='统一波特率并连接';const action=document.createElement('div');action.innerHTML='<p class="workflow-note">接上已编号为 1–4 的四台电机。自动查找各台波特率，统一到 921600，再连接。</p><button id="workflowConnect" class="workflow-primary">一键连接</button><p id="workflowConnectState" role="status"></p>';connect.insertBefore(action,connect.children[2]);
const connectionIds=document.createElement('div');connectionIds.className='row';for(const id of [1,2,3,4]){const item=document.createElement('div');item.id='connectionId'+id;item.className='status';item.textContent=`ID ${id} · 待检查`;connectionIds.append(item)}action.append(connectionIds);const markId=(id,text)=>{$('connectionId'+id).textContent=`ID ${id} · ${text}`};
const retryDiscovery=document.createElement('button');retryDiscovery.id='retryDiscovery';retryDiscovery.textContent='缺少设备？重新扫描';retryDiscovery.hidden=true;retryDiscovery.onclick=async()=>{await $('scanButton').onclick();retryDiscovery.hidden=true};action.append(retryDiscovery);
const fixedBaud=document.createElement('p');fixedBaud.className='workflow-note';fixedBaud.textContent='电脑端正式通信：921600（固定）。探测旧速率 → 逐台统一921600 → 回读确认 → 连接。';action.prepend(fixedBaud);$('baudState').hidden=true;
const cal=$('calibrationSection');cal.querySelector('h2').textContent='四角校准';const calActions=document.createElement('div');calActions.innerHTML='<p class="workflow-note">首次使用先在上方检查收绳方向，再校准。</p><button class="workflow-primary" onclick="startFullCalibration()">开始 / 重新校准</button>';const detail=document.createElement('details');detail.innerHTML='<summary>高级参数与微调</summary>';for(const child of [...cal.children].slice(3))detail.append(child);cal.append(calActions,detail);
const nav=document.createElement('nav');nav.id='workflowNav';nav.setAttribute('aria-label','调试步骤');document.body.append(nav);
main.insertBefore(live,pages[0]);live.className='card';live.querySelector('p.workflow-note:last-child').textContent='自动发现：全部支持波特率 · ID 1–10。改 ID 和连接两页持续扫描；不同波特率的设备同时显示。运动时暂停。';
function show(index){pages.forEach((p,i)=>p.classList.toggle('active',i===index));live.hidden=index>1;[...nav.children].forEach((b,i)=>{b.classList.toggle('active',i===index);b.setAttribute('aria-current',i===index?'page':'false')});if(index===3){syncCanvasGeometry(latestAuto.calibration||{});paint()}window.scrollTo(0,0)}
['1 改 ID','2 连接','3 校准','4 绘制'].forEach((title,i)=>{const b=document.createElement('button');b.textContent=title;b.onclick=()=>show(i);nav.append(b)});show(0);
let connecting=false;
$('workflowConnect').onclick=async()=>{if(connecting)return;connecting=true;const button=$('workflowConnect'),status=$('workflowConnectState');button.disabled=true;try{
pauseDiscovery();
retryDiscovery.hidden=true;$('idState').textContent='正在连接：自动扫描已停止';
while(discoveryBusy)await new Promise(r=>setTimeout(r,100));
if(!['idle','error'].includes(latestAuto.auto))throw Error('请先急停当前运动，再连接');
if(!port()&&portRows.length>1)throw Error('检测到多个串口，请先在上方选择 USB-RS485 串口，再点一键连接');
status.textContent=`正在扫描各台电机的波特率（${port()||'自动选择串口'}）…`;const scan=await post({action:'baud-sweep',...portArg()},'/api/command',60000);
const rates={};for(const row of scan.results||[])for(const id of row.valid||[]){(rates[id]??=[]).push(row.baud)}
for(const id of [1,2,3,4])markId(id,rates[id]?.length?'已发现 · '+rates[id].join(' / '):'未找到');
for(const id of [1,2,3,4])if(!rates[id]?.length)throw Error('未找到 ID '+id+'，请检查供电、接线及编号');else if(rates[id].length!==1)throw Error('ID '+id+' 在多个波特率有回包，请先排查重复编号');
for(const id of [1,2,3,4]){const baud=rates[id][0];if(baud===921600)continue;markId(id,'正在统一波特率…');status.textContent='正在将 M'+id+' 改为 921600 并回读…';const r=await post({action:'baud-set-single',id,baud,target_baud:921600,confirmed:true,...portArg()},'/api/command',30000);markId(id,r.verified?'921600 已确认':'写入后未确认');if(!r.verified)throw Error('M'+id+' 写入后未确认成功')}
syncBaudSelects(921600);status.textContent='正在确认四台在 921600 回包…';const verify=await post({action:'baud-sweep',bauds:[921600],...portArg()},'/api/command',15000);const verifiedIds=(verify.results||[]).find(r=>r.baud===921600)?.valid||[];for(const id of [1,2,3,4])markId(id,verifiedIds.includes(id)?'已连接 · 921600':'921600 未回包');if(verify.recommended_baud!==921600)throw Error('四台未全部在 921600 回包，请检查后重试');
status.textContent='四台921600已确认；正在检查/切换 X 固件、锁定物理按键…';
const prepared=await post({action:'prepare',baud:921600,confirmed:true,...portArg()},'/api/command',60000);
if(!prepared.ready)throw Error(prepared.message||'X 固件或物理按键锁定未全部通过');
controlsLocked=false;sessionSafetyConfirmed=true;updateControlLock();setConnectionState(prepared.message);for(const id of [1,2,3,4])markId(id,'已连接 · 921600 · X · 按键已锁定');status.textContent='四台已连接 · 921600 · X 固件 · 按键已锁定；自动扫描已停止';$('idState').textContent='已连接：自动扫描已停止，正常通信固定921600';
}catch(e){status.textContent=e.message+'；自动扫描保持关闭，可点击重新扫描';retryDiscovery.hidden=false;showToast(e.message,'error',7000)}finally{connecting=false;button.disabled=false}};
})();
