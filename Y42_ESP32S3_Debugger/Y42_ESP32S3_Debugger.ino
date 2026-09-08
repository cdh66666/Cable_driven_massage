#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <vector>
#include <algorithm>
#include <float.h>

// -------------------- Hardware / Wi-Fi configuration --------------------
static constexpr int MOTOR_RX_PIN = 17;  // Current wiring: GPIO17 <- RS485 module TX
static constexpr int MOTOR_TX_PIN = 16;  // Current wiring: GPIO16 -> RS485 module RX
static constexpr uint32_t MOTOR_BAUD = 115200;
static constexpr int INFLATE_PUMP_PIN = 47;
static constexpr int DEFLATE_PUMP_PIN = 48;
static constexpr int DEFLATE_VALVE_PIN = 45;
static constexpr uint8_t ACTUATOR_ON = HIGH; // Change to LOW if using active-low relay modules.
static constexpr uint8_t ACTUATOR_OFF = LOW;
static const char *AP_SSID = "Y42-Debug";

HardwareSerial MotorSerial(1);
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

enum FirmwareType : uint8_t { FW_EMM = 0, FW_X = 1 };
FirmwareType firmwareType = FW_X; // This connected Y42 reports X firmware behavior.
uint8_t motorAddress = 1;
uint32_t pulsesPerRevolution = 3200; // Emm default: 1.8 deg motor, subdivision 16
String serialLog;

static const uint8_t MOTOR_IDS[4] = {1, 2, 3, 4};
static const uint8_t PULL_DIRECTIONS[4] = {0, 1, 0, 1}; // M1/M3 CW, M2/M4 CCW toward own corner
enum AutoState : uint8_t { AUTO_IDLE, AUTO_CAL_PULL, AUTO_CAL_WAIT, AUTO_DRAW_CIRCLE, AUTO_DRAW_TRIANGLE, AUTO_DRAW_CUSTOM, AUTO_ERROR };
struct MotorTelemetry { bool ok=false; float position=0, rpm=0; uint16_t current=0; uint8_t temperature=0, flags=0; };
struct PathPoint { float x, y; PathPoint(float px=0,float py=0):x(px),y(py){} };
AutoState autoState = AUTO_IDLE;
MotorTelemetry autoTelemetry[4];
float calibrationSamples[4][4] = {{0}}; // [corner][motor], signed encoder degrees
float calibrationSlope[4] = {0}, calibrationIntercept[4] = {0}, calibrationR2[4] = {0};
float calibrationGroupSlope[4][3] = {{0}}, calibrationSpread[4] = {0};
static constexpr float MAX_SLOPE_SPREAD = 3.7f; // deg/mm, user-observed repeatability limit
String calibrationQualityError;
bool calibrationValid = false;
String autoMessage = "等待自动校准";
uint8_t calibrationCorner = 0;
uint16_t calibrationStrongCurrent = 1300, calibrationTensionCurrent = 225;
static constexpr uint16_t MOTOR_HARD_CURRENT_LIMIT = 3000;
static constexpr uint16_t MOTOR_OPERATING_CURRENT_LIMIT = 2000;
static constexpr float TRAJECTORY_MAX_MOTOR_RPM = 3000.0f;
static constexpr float TRAJECTORY_RPM_FEEDFORWARD = 1.25f;
float effectiveSideMm = 130.0f; // User definition: 150mm - ball radius 20mm
uint32_t autoPhaseStarted = 0, autoNextSample = 0, autoSettleStarted = 0;
float autoLastPositions[4] = {0};
bool autoHaveLastPositions = false;
float autoCornerStartPositions[4] = {0};
bool autoHaveCornerStart = false, autoMovementSeen = false;
uint32_t drawPoint = 0, drawPointCount = 0;
bool drawWaitingForPoint = false;
uint32_t drawPointStarted = 0, drawLastSafety = 0;
uint8_t drawSafetyMotor = 0, drawSafetyField = 0;
bool drawPreparing=false;
uint32_t trajectoryStarted=0, trajectoryDurationMs=8000, drawStartSettledAt=0;
uint32_t customDurationMs=0;
uint16_t trajectoryIntervalMs=7;
static constexpr float TRAJECTORY_MAX_STEP_MM = 3.5f;
uint32_t drawLastUiUpdate=0;
float lastTrajectoryX=0,lastTrajectoryY=0;
float drawTargetDegrees[4]={0};
static constexpr float TRAJECTORY_PRETENSION_MM=1.0f;
std::vector<PathPoint> customPath;
std::vector<float> customCumulative;
float customPathLength=0;
float calibrationStableSum[4]={0};
uint8_t calibrationStableCount=0;
String pneumaticMode = "off";
bool inflatePumpOn=false, deflatePumpOn=false, deflateValveOn=false;
bool autoTensionHold = false;
bool repeatCustomContinuously=false;
uint32_t lastUserInstructionAt=0;
bool motorIdleTimerArmed=false;
static constexpr uint32_t MOTOR_IDLE_DISABLE_MS=10UL*60UL*1000UL;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Y42 电机调试台</title><style>
:root{--bg:#0b1020;--panel:#151c31;--line:#293451;--text:#edf2ff;--muted:#91a0be;--blue:#4f8cff;--cyan:#35d4c7;--red:#ff5868;--orange:#ffb547;--green:#49d17d}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#17284a 0,#0b1020 42%);color:var(--text);font-family:system-ui,-apple-system,"Microsoft YaHei",sans-serif;min-height:100vh}
.wrap{max-width:1440px;margin:auto;padding:10px}.head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:8px}.title{font-size:21px;font-weight:800}.sub{color:var(--muted);font-size:12px;margin-top:2px}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#65708a;margin-right:6px}.dot.ok{background:var(--green);box-shadow:0 0 10px var(--green)}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:8px}.card,.wide,.full{grid-column:span 6;background:rgba(21,28,49,.96);border:1px solid var(--line);border-radius:12px;padding:10px;box-shadow:0 7px 24px #0003}.card h2{font-size:14px;margin:0 0 7px;color:#cbd6ee}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}.metric{background:#0d1427;border:1px solid #26314e;border-radius:8px;padding:7px}.metric b{display:block;font-size:17px;margin-top:2px}.metric span{font-size:10px;color:var(--muted)}
.row{display:flex;flex-wrap:wrap;gap:6px;align-items:end;margin:6px 0}.field{flex:1;min-width:105px}label{display:block;color:var(--muted);font-size:10px;margin:0 0 3px}input,select{width:100%;border:1px solid #33415f;background:#0c1325;color:var(--text);padding:7px;border-radius:7px;outline:none}input:focus,select:focus{border-color:var(--blue)}button{border:0;border-radius:8px;padding:8px 10px;background:var(--blue);color:white;font-weight:700;cursor:pointer;white-space:nowrap}button:active{transform:translateY(1px)}button.secondary{background:#34415f}button.good{background:#168f55}button.warn{background:#c17a18}button.stop{background:var(--red);font-size:14px;width:100%;padding:9px}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:6px}.tip{font-size:10px;line-height:1.35;color:var(--muted);margin-top:6px}.badge{display:inline-block;padding:2px 6px;border-radius:99px;font-size:10px;background:#253252;color:#bfd0f3;margin:1px}
pre{height:125px;overflow:auto;white-space:pre-wrap;background:#090e1c;border:1px solid #26314e;border-radius:8px;padding:7px;color:#a9f3da;font:11px/1.35 ui-monospace,Consolas,monospace;margin:0}.toast{position:fixed;right:12px;bottom:12px;max-width:360px;background:#222d48;border:1px solid #3b4d74;border-radius:9px;padding:9px 12px;display:none;z-index:5}.danger{color:#ff8390}
#pathCanvas{display:block;width:min(100%,300px);aspect-ratio:1;margin:0 auto 6px;background:#090e1c;border:1px solid #33415f;border-radius:9px;touch-action:none;cursor:crosshair}
@media(max-width:1000px){.card,.wide,.full{grid-column:1/-1}.metrics{grid-template-columns:repeat(3,1fr)}}@media(max-width:480px){.wrap{padding:7px}.metrics{grid-template-columns:1fr 1fr}.row.stack{flex-direction:column;align-items:stretch}.row.stack .field{width:100%}.title{font-size:18px}}
</style></head><body><div class="wrap">
<div class="head"><div><div class="title">Y42 闭环步进电机调试台</div><div class="sub">ESP32-S3 · RS485 · GPIO16/17</div></div><div><span id="dot" class="dot"></span><span id="online">等待通讯</span></div></div>
<div class="grid">
<section class="card full"><h2>实时状态</h2><div class="metrics">
<div class="metric"><span>总线电压</span><b id="voltage">--</b></div><div class="metric"><span>总线电流</span><b id="busCurrent">--</b></div><div class="metric"><span>相电流</span><b id="phaseCurrent">--</b></div><div class="metric"><span>实时转速</span><b id="speed">--</b></div><div class="metric"><span>实时位置</span><b id="position">--</b></div><div class="metric"><span>驱动温度</span><b id="temperature">--</b></div>
</div><div id="flags" style="margin-top:10px"></div></section>

<section class="card"><h2>安全控制</h2><div class="buttons"><button class="good" onclick="action('enableAll','on=1')">四电机使能 / 锁轴</button><button class="secondary" onclick="action('enableAll','on=0')">四电机失能 / 松轴</button><button class="warn" onclick="action('release')">解除保护</button><button class="stop" style="font-size:14px;padding:8px" onclick="action('stop')">■ 四电机立即停止</button></div></section>

<section class="card full"><h2>四电机绳长自动校准与轨迹测试</h2><div class="row stack"><div class="field"><label>框架边长 mm</label><input id="calSide" type="number" min="50" max="1000" value="150"></div><div class="field"><label>小球直径 mm</label><input id="calBall" type="number" min="1" max="500" value="40"></div><div class="field"><label>目标电机电流 mA</label><input id="calStrong" type="number" min="50" max="2000" value="1300"></div><div class="field"><label>其余张紧电流 mA</label><input id="calWeak" type="number" min="20" max="500" value="225"></div></div><label style="display:flex;align-items:center;gap:7px;color:#ffb547;font-size:12px;margin:7px 0"><input id="calReady" type="checkbox" style="width:auto">机构范围已清空，人员已离开，并已准备物理急停</label><div class="buttons"><button id="calButton" class="warn" onclick="startCalibration()">开始四角校准</button><button class="stop" style="font-size:14px;padding:8px" onclick="autoCommand('abort')">四电机急停</button></div><div class="buttons" style="margin-top:6px"><button class="good" onclick="autoCommand('circle')">校准后画圆</button><button onclick="autoCommand('triangle')">校准后画三角形</button></div><div id="autoStatus" class="tip" style="margin-top:8px;padding:8px;background:#0d1427;border-radius:8px">正在读取校准状态…</div><div id="coeffs" class="tip"></div><div class="tip danger">目标电流可调至2000mA，其余225mA持续张紧；实测超过3000mA、过温、保护或通讯中断会立即停止。</div></section>

<section class="card full"><h2>手绘小球行驶轨迹（全有效范围）</h2><canvas id="pathCanvas" width="600" height="600"></canvas><div class="row"><div class="field"><label>轨迹速度 mm/s（固件按校准系数自动限制）</label><input id="pathSpeed" type="number" min="30" max="5000" value="300"></div></div><div class="buttons"><button class="secondary" onclick="clearDrawnPath()">清空重画</button><button id="runPathButton" class="good" onclick="runDrawnPath()">按原始轨迹运行</button><button class="warn" onclick="repeatDrawnPath()">循环上次轨迹（直到停止）</button><button class="secondary" onclick="stopPathLoop()">停止循环（本轮结束后）</button></div><div id="pathUploadState" class="tip">等待绘制</div><div class="tip">可输入最高 5000 mm/s；固件会根据 3000 RPM 电机上限、当前校准系数和 100 Hz 控制周期自动采用可实现的最高值。</div></section>

<section class="card full"><h2>小球气动控制</h2><div class="buttons"><button class="good" onclick="pump('inflate')">开始充气（GPIO47）</button><button class="warn" onclick="pump('deflate')">开始放气（GPIO48 + 阀门GPIO45）</button></div><div class="buttons" style="margin-top:9px"><button class="secondary" onclick="toggleOutput('deflate')">单独切换放气泵 GPIO48</button><button class="secondary" onclick="toggleOutput('valve')">单独切换阀门 GPIO45</button></div><div style="margin-top:9px"><button class="stop" onclick="pump('off')">停止全部气泵和阀门</button></div><div id="pumpState" class="tip">当前：全部关闭</div><div class="tip">正常放气时泵和阀门同步；两个“单独切换”按钮用于排查噪音，每按一次只翻转对应输出。当前按高电平有效控制。</div></section>

</div></div><div id="toast" class="toast"></div>
<script>
const $=id=>document.getElementById(id);let busy=false;
function show(msg,bad=false){const t=$('toast');t.textContent=msg;t.style.display='block';t.style.borderColor=bad?'#a83b4a':'#3b4d74';setTimeout(()=>t.style.display='none',2600)}
async function req(path,opt){try{const r=await fetch(path,opt);const j=await r.json();if(!r.ok||j.ok===false)throw Error(j.error||'请求失败');return j}catch(e){show(e.message,true);throw e}}
function qs(o){return new URLSearchParams(o).toString()}
async function action(name,args=''){const j=await req('/api/action?name='+name+(args?'&'+args:''),{method:'POST'});show(j.message)}
async function runSpeed(){await action('speed',qs({dir:$('sdir').value,rpm:$('srpm').value,acc:$('sacc').value}))}
async function runTorque(){await action('torque',qs({dir:$('tdir').value,current:$('tcurrent').value,slope:$('tslope').value,maxrpm:$('tmaxrpm').value}))}
async function runPosition(){await action('position',qs({dir:$('pdir').value,deg:$('pdeg').value,rpm:$('prpm').value,acc:$('pacc').value,mode:$('pmode').value}))}
async function home(){if(+$('homeMode').value>=2&&!confirm('危险动作：已确认回零参数、机械限位和人员安全？'))return;await action('home','mode='+$('homeMode').value)}
async function sendRaw(){const j=await req('/api/raw',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'hex='+encodeURIComponent($('raw').value)});show(j.message);refreshLog()}
let autoRunning=false;
async function autoCommand(action,extra={}){const j=await req('/api/auto',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:qs({action,...extra})});show(j.message);await refreshAuto()}
async function startCalibration(){if(!$('calReady').checked){$('autoStatus').textContent='请先勾选安全确认，再开始校准';show('请先勾选安全确认',true);return}const b=$('calButton');b.disabled=true;$('autoStatus').textContent='正在发送校准命令…';try{await autoCommand('calibrate',{side:$('calSide').value,ball:$('calBall').value,strong:$('calStrong').value,weak:$('calWeak').value})}catch(e){$('autoStatus').textContent='校准启动失败：'+e.message}finally{b.disabled=false}}
async function refreshAuto(){try{const j=await req('/api/auto');autoRunning=j.active;$('autoStatus').textContent=j.message;$('coeffs').innerHTML=j.valid?j.motors.map((m,i)=>`<span class="badge">M${i+1}: 平均 ${m.slope.toFixed(4)}°/mm · 三组 [${m.groups.map(v=>v.toFixed(3)).join(', ')}] · 偏差 ${m.spread.toFixed(3)} · R² ${m.r2.toFixed(3)}</span>`).join(''):'尚无有效绳长系数'}catch(e){}}
async function pump(mode){const j=await req('/api/pump',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:qs({mode})});$('pumpState').textContent='当前：'+j.state;show(j.message)}
async function toggleOutput(target){const j=await req('/api/output',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:qs({target})});$('pumpState').textContent=`当前：充气泵${j.inflate?'开':'关'} / 放气泵${j.deflate?'开':'关'} / 阀门${j.valve?'开':'关'}`;show(j.message)}
const pathCanvas=$('pathCanvas'),pathCtx=pathCanvas.getContext('2d');let drawnPath=[],drawing=false;
function redrawPath(){const w=pathCanvas.width,h=pathCanvas.height;pathCtx.clearRect(0,0,w,h);pathCtx.strokeStyle='#26314e';pathCtx.lineWidth=1;for(let i=1;i<4;i++){pathCtx.beginPath();pathCtx.moveTo(i*w/4,0);pathCtx.lineTo(i*w/4,h);pathCtx.stroke();pathCtx.beginPath();pathCtx.moveTo(0,i*h/4);pathCtx.lineTo(w,i*h/4);pathCtx.stroke()}if(drawnPath.length){pathCtx.strokeStyle='#35d4c7';pathCtx.lineWidth=6;pathCtx.lineJoin='round';pathCtx.lineCap='round';pathCtx.beginPath();drawnPath.forEach((p,i)=>(i?pathCtx.lineTo(p.x*w,p.y*h):pathCtx.moveTo(p.x*w,p.y*h)));pathCtx.stroke()}}
function canvasPoint(e){const r=pathCanvas.getBoundingClientRect();return{x:Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),y:Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))}}
pathCanvas.addEventListener('pointerdown',e=>{drawing=true;drawnPath=[canvasPoint(e)];pathCanvas.setPointerCapture(e.pointerId);redrawPath()});
function appendPointerSample(e){const p=canvasPoint(e),q=drawnPath.at(-1);if(!q||Math.hypot(p.x-q.x,p.y-q.y)>.0002)drawnPath.push(p)}
pathCanvas.addEventListener('pointermove',e=>{if(!drawing)return;const samples=e.getCoalescedEvents?e.getCoalescedEvents():[e];samples.forEach(appendPointerSample);redrawPath()});
pathCanvas.addEventListener('pointerup',e=>{if(drawing)appendPointerSample(e);drawing=false;redrawPath()});pathCanvas.addEventListener('pointercancel',()=>drawing=false);
function clearDrawnPath(){drawnPath=[];redrawPath()}
async function runDrawnPath(){if(drawnPath.length<2){show('请先在画布中绘制轨迹',true);return}if(drawnPath.length>2000){show('原始轨迹超过2000点，请缩短一次绘制时间',true);return}const data=drawnPath.map(p=>p.x.toFixed(4)+','+p.y.toFixed(4)).join(';'),btn=$('runPathButton');btn.disabled=true;$('pathUploadState').textContent=`正在原样上传 ${drawnPath.length} 个手绘点…`;try{const j=await req('/api/path',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:qs({points:data,speed:$('pathSpeed').value})});$('pathUploadState').textContent=`已原样接收 ${drawnPath.length} 点 · ${j.message}`;show(j.message);await refreshAuto()}catch(e){$('pathUploadState').textContent='失败：'+e.message}finally{btn.disabled=false}}
async function repeatDrawnPath(){try{await autoCommand('loop_path');$('pathUploadState').textContent='正在循环运行上次轨迹；按急停结束循环'}catch(e){$('pathUploadState').textContent='循环启动失败：'+e.message}}
async function stopPathLoop(){try{await autoCommand('stop_loop');$('pathUploadState').textContent='已取消后续循环，当前轮结束后停止'}catch(e){$('pathUploadState').textContent='停止循环失败：'+e.message}}
redrawPath();
function val(id,v,unit){$(id).textContent=v==null?'--':v+unit}
async function refresh(){if(busy||autoRunning)return;busy=true;try{const j=await req('/api/status');$('dot').className='dot '+(j.online?'ok':'');$('online').textContent=j.online?'电机在线':'电机无响应';val('voltage',j.voltage,' V');val('busCurrent',j.busCurrent,' mA');val('phaseCurrent',j.phaseCurrent,' mA');val('speed',j.speed,' RPM');val('position',j.position,'°');val('temperature',j.temperature,' ℃');const f=j.flags||{};$('flags').innerHTML=['enabled','reached','stall','protected','leftLimit','rightLimit'].map(k=>`<span class="badge">${{enabled:'使能',reached:'到位',stall:'堵转',protected:'保护',leftLimit:'左限位',rightLimit:'右限位'}[k]}：${f[k]?'是':'否'}</span>`).join('')}catch(e){}finally{busy=false}}
refresh();refreshAuto();setInterval(refresh,1500);setInterval(refreshAuto,1500);
</script></body></html>)HTML";

// -------------------- Protocol helpers --------------------
void appendLog(const String &line) {
  serialLog += line + "\n";
  if (serialLog.length() > 7500) serialLog.remove(0, serialLog.length() - 6000);
}

String hexBytes(const std::vector<uint8_t> &data) {
  String out;
  char b[4];
  for (size_t i = 0; i < data.size(); ++i) {
    snprintf(b, sizeof(b), "%02X", data[i]);
    if (i) out += ' ';
    out += b;
  }
  return out;
}

std::vector<uint8_t> transact(const std::vector<uint8_t> &tx, uint32_t firstByteTimeoutMs = 35) {
  while (MotorSerial.available()) MotorSerial.read();
  MotorSerial.write(tx.data(), tx.size());
  MotorSerial.flush();
  appendLog("TX  " + hexBytes(tx));

  std::vector<uint8_t> rx;
  uint32_t start = millis(), lastByte = start;
  bool received = false;
  while (true) {
    while (MotorSerial.available()) {
      rx.push_back((uint8_t)MotorSerial.read());
      received = true;
      lastByte = millis();
    }
    uint32_t now=millis();
    if(received && now-lastByte>=2) break;
    if(!received && now-start>=firstByteTimeoutMs) break;
    delay(1);
  }
  appendLog(rx.empty() ? "RX  (timeout)" : "RX  " + hexBytes(rx));
  return rx;
}

void pushU16(std::vector<uint8_t> &v, uint32_t n) {
  v.push_back((n >> 8) & 0xFF); v.push_back(n & 0xFF);
}
void pushU32(std::vector<uint8_t> &v, uint64_t n) {
  v.push_back((n >> 24) & 0xFF); v.push_back((n >> 16) & 0xFF);
  v.push_back((n >> 8) & 0xFF); v.push_back(n & 0xFF);
}
uint16_t readU16(const std::vector<uint8_t> &v, size_t p) {
  return ((uint16_t)v[p] << 8) | v[p + 1];
}
uint32_t readU32(const std::vector<uint8_t> &v, size_t p) {
  return ((uint32_t)v[p] << 24) | ((uint32_t)v[p + 1] << 16) | ((uint32_t)v[p + 2] << 8) | v[p + 3];
}

std::vector<uint8_t> simpleCommand(uint8_t code) {
  return transact({motorAddress, code, 0x6B});
}
bool validReply(const std::vector<uint8_t> &r, uint8_t code, size_t minLen) {
  return r.size() >= minLen && r[0] == motorAddress && r[1] == code && r.back() == 0x6B;
}
bool ensureMotorEnabled() {
  auto status = simpleCommand(0x3A);
  if (validReply(status, 0x3A, 4) && (status[2] & 0x01)) return true;
  auto enabled = transact({motorAddress, 0xF3, 0xAB, 0x01, 0x00, 0x6B});
  return enabled.size() >= 4 && enabled[0] == motorAddress && enabled[1] == 0xF3 && enabled[2] == 0x02;
}
String replyMessage(const std::vector<uint8_t> &r) {
  if (r.empty()) return "已发送，但电机未返回；请检查 A/B、共地和固件通讯设置";
  if (r.size() >= 4 && r[r.size() - 2] == 0x02) return "命令执行成功";
  if (r.size() >= 4 && r[r.size() - 2] == 0xE2) return "电机拒绝：参数/状态不满足";
  if (r.size() >= 4 && r[r.size() - 2] == 0xEE) return "电机报告命令格式错误";
  return "收到响应：" + hexBytes(r);
}

bool isAutoActive() {
  return autoState == AUTO_CAL_PULL || autoState == AUTO_CAL_WAIT || autoState == AUTO_DRAW_CIRCLE || autoState == AUTO_DRAW_TRIANGLE || autoState == AUTO_DRAW_CUSTOM;
}
bool validAddressReply(const std::vector<uint8_t> &r, uint8_t addr, uint8_t code, size_t minLen) {
  return r.size() >= minLen && r[0] == addr && r[1] == code && r.back() == 0x6B;
}
bool commandAccepted(const std::vector<uint8_t> &r, uint8_t addr, uint8_t code) {
  return r.size() >= 4 && r[0] == addr && r[1] == code && r[2] == 0x02 && r.back() == 0x6B;
}
bool enableAddress(uint8_t addr) {
  auto status = transact({addr, 0x3A, 0x6B});
  if (validAddressReply(status, addr, 0x3A, 4) && (status[2] & 0x01)) return true;
  return commandAccepted(transact({addr, 0xF3, 0xAB, 0x01, 0x00, 0x6B}), addr, 0xF3);
}
void stopAllMotors() {
  for (uint8_t addr : MOTOR_IDS) transact({addr, 0xFE, 0x98, 0x00, 0x6B});
}
void disableAllMotors() {
  for(uint8_t addr:MOTOR_IDS) transact({addr,0xF3,0xAB,0x00,0x00,0x6B});
}
void noteUserInstruction(bool armsMotor=false) {
  lastUserInstructionAt=millis();
  if(armsMotor) motorIdleTimerArmed=true;
}
bool sendTorqueAddress(uint8_t index, uint16_t current, bool towardOwnCorner) {
  uint8_t addr = MOTOR_IDS[index];
  uint8_t direction = towardOwnCorner ? PULL_DIRECTIONS[index] : (uint8_t)(1-PULL_DIRECTIONS[index]);
  std::vector<uint8_t> cmd = {addr, 0xC5, direction};
  pushU16(cmd, 2000); pushU16(cmd, current); cmd.push_back(0x00); pushU16(cmd, 10000); cmd.push_back(0x6B); // 1000.0 RPM limit
  return commandAccepted(transact(cmd), addr, 0xC5);
}
bool readTelemetry(uint8_t index, MotorTelemetry &t) {
  uint8_t addr = MOTOR_IDS[index];
  auto pos = transact({addr, 0x36, 0x6B});
  auto rpm = transact({addr, 0x35, 0x6B});
  auto cur = transact({addr, 0x27, 0x6B});
  auto tmp = transact({addr, 0x39, 0x6B});
  auto sta = transact({addr, 0x3A, 0x6B});
  if (!validAddressReply(pos,addr,0x36,8) || !validAddressReply(rpm,addr,0x35,6) || !validAddressReply(cur,addr,0x27,5) || !validAddressReply(tmp,addr,0x39,5) || !validAddressReply(sta,addr,0x3A,4)) { t.ok=false; return false; }
  t.position = readU32(pos,3) / 10.0f; if (pos[2] == 1) t.position = -t.position;
  t.rpm = readU16(rpm,3) / 10.0f; if (rpm[2] == 1) t.rpm = -t.rpm;
  t.current = readU16(cur,2);
  t.temperature = tmp[3]; // Real Y42 units observed: 00 1B means +27 C.
  t.flags = sta[2]; t.ok = true; return true;
}
void failAuto(const String &reason) {
  stopAllMotors();
  autoTensionHold = false;
  repeatCustomContinuously = false;
  autoState = AUTO_ERROR;
  autoMessage = "已紧急停止：" + reason;
  appendLog("AUTO FAULT: " + reason);
}
bool checkAllMotorSafety(uint16_t currentLimit) {
  for (uint8_t i=0;i<4;i++) {
    if (!readTelemetry(i, autoTelemetry[i])) { failAuto("电机 " + String(i+1) + " 通讯中断"); return false; }
    if (autoTelemetry[i].temperature >= 55) { failAuto("电机 " + String(i+1) + " 温度达到 " + String(autoTelemetry[i].temperature) + "C"); return false; }
    if (autoTelemetry[i].current > currentLimit) { failAuto("电机 " + String(i+1) + " 电流突增到 " + String(autoTelemetry[i].current) + "mA"); return false; }
    if (autoTelemetry[i].flags & 0x08) { failAuto("电机 " + String(i+1) + " 触发堵转保护"); return false; }
  }
  return true;
}
bool startCalibrationCorner() {
  for (uint8_t i=0;i<4;i++) if (!enableAddress(MOTOR_IDS[i])) { failAuto("电机 " + String(i+1) + " 无法使能"); return false; }
  for (uint8_t i=0;i<4;i++) if (i != calibrationCorner && !sendTorqueAddress(i, calibrationTensionCurrent, true)) { failAuto("电机 " + String(i+1) + " 持续张紧命令失败"); return false; }
  if (!sendTorqueAddress(calibrationCorner, calibrationStrongCurrent, true)) { failAuto("目标电机正向力矩命令失败"); return false; }
  autoState = AUTO_CAL_PULL; autoPhaseStarted = millis(); autoNextSample = 0; autoSettleStarted = 0; autoHaveLastPositions = false; autoHaveCornerStart=false; autoMovementSeen=false; calibrationStableCount=0;
  for(uint8_t i=0;i<4;i++) calibrationStableSum[i]=0;
  autoMessage = "校准角点 " + String(calibrationCorner+1) + "/4：M" + String(calibrationCorner+1) + "=" + String(calibrationStrongCurrent) + "mA，其余=" + String(calibrationTensionCurrent) + "mA";
  return true;
}
bool computeCalibration() {
  const float ax[4] = {0, effectiveSideMm, effectiveSideMm, 0};
  const float ay[4] = {effectiveSideMm, effectiveSideMm, 0, 0};
  bool quality=true;
  calibrationQualityError = "";
  for (uint8_t m=0;m<4;m++) {
    calibrationIntercept[m]=calibrationSamples[m][m]; // Own corner is defined as zero rope length.
    float slopeNumerator=0,slopeDenominator=0,firstSlope=0; uint8_t count=0; bool signsAgree=true;
    for(uint8_t c=0;c<4;c++) if(c!=m) {
      float length=hypotf(ax[c]-ax[m],ay[c]-ay[m]);
      float localSlope=(calibrationSamples[c][m]-calibrationIntercept[m])/length;
      if(count==0) firstSlope=localSlope; else if(localSlope*firstSlope<=0) signsAgree=false;
      calibrationGroupSlope[m][count]=localSlope;
      slopeNumerator+=length*(calibrationSamples[c][m]-calibrationIntercept[m]); slopeDenominator+=length*length; count++;
    }
    calibrationSlope[m]=slopeDenominator>0?slopeNumerator/slopeDenominator:0;
    calibrationSpread[m]=0;
    for(uint8_t g=0;g<3;g++) calibrationSpread[m]=max(calibrationSpread[m],fabsf(calibrationGroupSlope[m][g]-calibrationSlope[m]));
    float mean=0; for(uint8_t c=0;c<4;c++) mean+=calibrationSamples[c][m]; mean/=4;
    float ssTot=0,ssRes=0;
    for (uint8_t c=0;c<4;c++) {
      float x=hypotf(ax[c]-ax[m], ay[c]-ay[m]), y=calibrationSamples[c][m], pred=calibrationIntercept[m]+calibrationSlope[m]*x;
      ssTot+=(y-mean)*(y-mean); ssRes+=(y-pred)*(y-pred);
    }
    calibrationR2[m]=ssTot>0.001f?1.0f-ssRes/ssTot:0;
    appendLog("CAL M"+String(m+1)+" groups="+String(calibrationGroupSlope[m][0],3)+","+String(calibrationGroupSlope[m][1],3)+","+String(calibrationGroupSlope[m][2],3)+" maxDeviation="+String(calibrationSpread[m],3));
    if(fabsf(calibrationSlope[m])<0.05f||!signsAgree||calibrationR2[m]<0.85f||calibrationSpread[m]>MAX_SLOPE_SPREAD) {
      quality=false;
      if(calibrationQualityError.isEmpty()) calibrationQualityError="电机 "+String(m+1)+" 三组系数偏差 "+String(calibrationSpread[m],3)+"°/mm（允许 ≤ "+String(MAX_SLOPE_SPREAD,1)+"）";
    }
  }
  if(!quality) return false;
  preferences.putBytes("calSlope", calibrationSlope, sizeof(calibrationSlope));
  preferences.putBytes("calInt", calibrationIntercept, sizeof(calibrationIntercept));
  preferences.putBytes("calR2", calibrationR2, sizeof(calibrationR2));
  preferences.putBytes("calGroups", calibrationGroupSlope, sizeof(calibrationGroupSlope));
  preferences.putBytes("calSpread", calibrationSpread, sizeof(calibrationSpread));
  preferences.putFloat("calSide", effectiveSideMm);
  preferences.putBool("calValid", true);
  calibrationValid=true; return true;
}
bool sendLimitedPosition(uint8_t index, float targetDegrees) {
  uint8_t addr=MOTOR_IDS[index], dir=targetDegrees<0?1:0;
  uint64_t angle=lroundf(fabsf(targetDegrees)*10.0f);
  std::vector<uint8_t> cmd={addr,0xCD,dir};
  pushU16(cmd,1000); pushU16(cmd,1000); pushU16(cmd,4000); // 400 RPM for the move to trajectory start.
  pushU32(cmd,angle); cmd.push_back(0x01); cmd.push_back(0x01); // absolute, synchronized
  pushU16(cmd,2000); cmd.push_back(0x6B); // User-selected position-control current ceiling.
  return commandAccepted(transact(cmd),addr,0xCD);
}
void sendFastFrame(const std::vector<uint8_t> &cmd) {
  while(MotorSerial.available()) MotorSerial.read();
  MotorSerial.write(cmd.data(),cmd.size()); MotorSerial.flush();
  uint32_t started=millis();
  while(millis()-started<1) while(MotorSerial.available()) MotorSerial.read();
}
void sendFastFrame(const uint8_t *cmd,size_t len) {
  while(MotorSerial.available()) MotorSerial.read();
  MotorSerial.write(cmd,len); MotorSerial.flush();
  uint32_t started=millis();
  while(millis()-started<1) while(MotorSerial.available()) MotorSerial.read();
}
void sendLimitedPositionFast(uint8_t index,float targetDegrees) {
  uint8_t dir=targetDegrees<0?1:0;
  std::vector<uint8_t> cmd={MOTOR_IDS[index],0xCD,dir};
  pushU16(cmd,65535); pushU16(cmd,65535); pushU16(cmd,10000); // immediate following, 1000.0 RPM ceiling
  pushU32(cmd,lroundf(fabsf(targetDegrees)*10.0f)); cmd.push_back(0x01); cmd.push_back(0x01);
  pushU16(cmd,500); cmd.push_back(0x6B);
  sendFastFrame(cmd);
}
void moveBallToFast(float x,float y) {
  const float ax[4]={0,effectiveSideMm,effectiveSideMm,0};
  const float ay[4]={effectiveSideMm,effectiveSideMm,0,0};
  for(uint8_t i=0;i<4;i++) {
    float length=hypotf(x-ax[i],y-ay[i]);
    sendLimitedPositionFast(i,calibrationIntercept[i]+calibrationSlope[i]*length);
  }
  sendFastFrame({0x00,0xFF,0x66,0x6B});
}
void sampleTrajectory(float u,float &x,float &y);
bool beginTrajectory(AutoState state,uint32_t durationMs,const String &label);
float motorDegreesAt(uint8_t i,float x,float y) {
  const float ax[4]={0,effectiveSideMm,effectiveSideMm,0};
  const float ay[4]={effectiveSideMm,effectiveSideMm,0,0};
  float length=max(0.0f,hypotf(x-ax[i],y-ay[i])-TRAJECTORY_PRETENSION_MM);
  return calibrationIntercept[i]+calibrationSlope[i]*length;
}
void setDrawTargets(float x,float y) {
  for(uint8_t i=0;i<4;i++) drawTargetDegrees[i]=motorDegreesAt(i,x,y);
}
bool drawTargetsReached(float toleranceDegrees=8.0f) {
  for(uint8_t i=0;i<4;i++) {
    bool encoderClose=fabsf(autoTelemetry[i].position-drawTargetDegrees[i])<=toleranceDegrees;
    if(!encoderClose||fabsf(autoTelemetry[i].rpm)>15.0f) return false;
  }
  return true;
}
// The reference Cable_driven_massage firmware uses CB straight-through,
// current-limited absolute position commands packed into one AA multi-motor
// frame. CB continuously chases updated targets without rebuilding a
// trapezoidal acceleration profile at every interpolation sample.
void sendTrajectoryPosition(float x,float y,float nextX,float nextY,float lookaheadSeconds) {
  uint8_t packet[64]; size_t p=0;
  packet[p++]=0x00; packet[p++]=0xAA; packet[p++]=0x00; packet[p++]=0x00;
  auto put16=[&](uint32_t n){packet[p++]=(n>>8)&0xFF;packet[p++]=n&0xFF;};
  auto put32=[&](uint64_t n){packet[p++]=(n>>24)&0xFF;packet[p++]=(n>>16)&0xFF;packet[p++]=(n>>8)&0xFF;packet[p++]=n&0xFF;};
  for(uint8_t i=0;i<4;i++) {
    float currentTarget=motorDegreesAt(i,x,y), target=motorDegreesAt(i,nextX,nextY);
    float rpm=lookaheadSeconds>0.0001f?fabsf(target-currentTarget)/(lookaheadSeconds*6.0f):1.0f;
    rpm=max(1.0f,min(TRAJECTORY_MAX_MOTOR_RPM,rpm*TRAJECTORY_RPM_FEEDFORWARD));
    packet[p++]=MOTOR_IDS[i]; packet[p++]=0xCB; packet[p++]=target<0?1:0;
    put16(lroundf(rpm*10.0f)); put32(lroundf(fabsf(target)*10.0f));
    packet[p++]=0x01; packet[p++]=0x00; // absolute position in AA multi-motor frame
    put16(2000); packet[p++]=0x6B;
  }
  uint16_t frameLength=(p-4)+3; packet[2]=frameLength>>8; packet[3]=frameLength&0xFF;
  packet[p++]=0x6B; sendFastFrame(packet,p);
}
void sampleTrajectory(float u,float &x,float &y) {
  u=max(0.0f,min(1.0f,u)); float c=effectiveSideMm/2.0f;
  if(autoState==AUTO_DRAW_CIRCLE) {
    float a=2.0f*PI*u,r=effectiveSideMm*0.40f; x=c+r*cosf(a); y=c+r*sinf(a); return;
  }
  if(autoState==AUTO_DRAW_TRIANGLE) {
    const float vx[3]={c,effectiveSideMm*0.90f,effectiveSideMm*0.10f};
    const float vy[3]={effectiveSideMm*0.10f,effectiveSideMm*0.85f,effectiveSideMm*0.85f};
    float q=min(2.999999f,u*3.0f); uint8_t seg=(uint8_t)q,n=(seg+1)%3; float t=q-seg;
    x=vx[seg]+(vx[n]-vx[seg])*t; y=vy[seg]+(vy[n]-vy[seg])*t; return;
  }
  if(customPath.empty()) { x=c; y=c; return; }
  float target=u*customCumulative.back();
  size_t hi=std::lower_bound(customCumulative.begin()+1,customCumulative.end(),target)-customCumulative.begin();
  if(hi>=customPath.size()) { x=customPath.back().x; y=customPath.back().y; return; }
  size_t lo=hi-1; float span=customCumulative[hi]-customCumulative[lo]; float t=span>0.0001f?(target-customCumulative[lo])/span:0;
  x=customPath[lo].x+(customPath[hi].x-customPath[lo].x)*t;
  y=customPath[lo].y+(customPath[hi].y-customPath[lo].y)*t;
}
bool moveBallTo(float x, float y) {
  const float ax[4]={0,effectiveSideMm,effectiveSideMm,0};
  const float ay[4]={effectiveSideMm,effectiveSideMm,0,0};
  for(uint8_t i=0;i<4;i++) {
    float length=max(0.0f,hypotf(x-ax[i],y-ay[i])-TRAJECTORY_PRETENSION_MM);
    if(!enableAddress(MOTOR_IDS[i]) || !sendLimitedPosition(i,calibrationIntercept[i]+calibrationSlope[i]*length)) return false;
  }
  auto sync=transact({0x00,0xFF,0x66,0x6B});
  return !sync.empty();
}
bool checkTrajectoryMotorSafetyStep() {
  uint8_t i=drawSafetyMotor;
  uint8_t addr=MOTOR_IDS[i];
  uint8_t code=drawSafetyField==0?0x27:(drawSafetyField==1?0x39:0x3A);
  size_t expected=drawSafetyField==2?4:5;
  auto reply=transact({addr,code,0x6B},10);
  if(!validAddressReply(reply,addr,code,expected)) { failAuto("轨迹中电机 "+String(i+1)+" 通讯中断"); return false; }
  if(drawSafetyField==0) {
    autoTelemetry[i].current=readU16(reply,2);
    if(autoTelemetry[i].current>MOTOR_HARD_CURRENT_LIMIT) { failAuto("轨迹中电机 "+String(i+1)+" 电流超过3000mA"); return false; }
  } else if(drawSafetyField==1) {
    autoTelemetry[i].temperature=reply[3];
    if(autoTelemetry[i].temperature>=55) { failAuto("轨迹中电机 "+String(i+1)+" 温度达到55C"); return false; }
  } else {
    autoTelemetry[i].flags=reply[2];
    if(autoTelemetry[i].flags&0x08) { failAuto("轨迹中电机 "+String(i+1)+" 触发保护"); return false; }
  }
  drawSafetyField=(drawSafetyField+1)%3;
  if(drawSafetyField==0) drawSafetyMotor=(drawSafetyMotor+1)%4;
  return true;
}
void drawCoordinates(uint16_t point, float &x, float &y) {
  float c=effectiveSideMm/2.0f;
  if(autoState==AUTO_DRAW_CIRCLE) {
    float a=2.0f*PI*point/(drawPointCount-1), r=effectiveSideMm*0.40f; x=c+r*cosf(a); y=c+r*sinf(a); return;
  }
  // Large triangle using most of the calibrated workspace.
  const float vx[3]={c,effectiveSideMm*0.90f,effectiveSideMm*0.10f};
  const float vy[3]={effectiveSideMm*0.10f,effectiveSideMm*0.85f,effectiveSideMm*0.85f};
  uint16_t perSide=(drawPointCount-1)/3, p=point%(perSide*3), seg=p/perSide; float t=(p%perSide)/(float)perSide; uint8_t n=(seg+1)%3;
  x=vx[seg]+(vx[n]-vx[seg])*t; y=vy[seg]+(vy[n]-vy[seg])*t;
}
void autoLoop() {
  uint32_t now=millis();
  if(autoState==AUTO_IDLE&&autoTensionHold&&(int32_t)(now-autoNextSample)>=0) {
    autoNextSample=now+500;
    if(!checkAllMotorSafety(MOTOR_HARD_CURRENT_LIMIT)) return;
  }
  if(autoState==AUTO_CAL_WAIT) { if(now-autoPhaseStarted>=700) startCalibrationCorner(); return; }
  if(autoState==AUTO_CAL_PULL && (int32_t)(now-autoNextSample)>=0) {
    autoNextSample=now+200;
    // The pulling motor may reach the commanded 1300 mA at the corner. The
    // three tension motors are expected near 225 mA and get a lower hard trip.
    if(!checkAllMotorSafety(MOTOR_HARD_CURRENT_LIMIT)) return;
    if(!autoHaveCornerStart) { for(uint8_t i=0;i<4;i++) autoCornerStartPositions[i]=autoTelemetry[i].position; autoHaveCornerStart=true; }
    else for(uint8_t i=0;i<4;i++) if(fabsf(autoTelemetry[i].position-autoCornerStartPositions[i])>5.0f) autoMovementSeen=true;
    bool allControlled=true, positionsStable=autoHaveLastPositions;
    for(uint8_t i=0;i<4;i++) {
      if(fabsf(autoTelemetry[i].rpm)>80.0f) allControlled=false;
      if(autoHaveLastPositions&&fabsf(autoTelemetry[i].position-autoLastPositions[i])>3.0f) positionsStable=false;
      autoLastPositions[i]=autoTelemetry[i].position;
    }
    autoHaveLastPositions=true;
    uint16_t loadThreshold=calibrationStrongCurrent*50/100; if(loadThreshold<300) loadThreshold=300;
    bool loaded=autoTelemetry[calibrationCorner].current>=loadThreshold;
    bool targetSlow=fabsf(autoTelemetry[calibrationCorner].rpm)<35.0f;
    float minTravel=max(180.0f,effectiveSideMm*1.4f);
    uint8_t movedMotors=0;
    if(calibrationCorner>0) for(uint8_t i=0;i<4;i++) if(fabsf(autoTelemetry[i].position-calibrationSamples[calibrationCorner-1][i])>=minTravel) movedMotors++;
    bool movementRequirement=calibrationCorner==0?(autoMovementSeen||(now-autoPhaseStarted>800)):
      (movedMotors>=1&&fabsf(autoTelemetry[calibrationCorner].position-calibrationSamples[calibrationCorner-1][calibrationCorner])>=minTravel);
    bool cornerStable=targetSlow&&allControlled&&loaded&&movementRequirement&&positionsStable;
    if(cornerStable) {
      if(!autoSettleStarted) {
        autoSettleStarted=now; calibrationStableCount=0;
        for(uint8_t i=0;i<4;i++) calibrationStableSum[i]=0;
      }
      if(calibrationStableCount<20) {
        for(uint8_t i=0;i<4;i++) calibrationStableSum[i]+=autoTelemetry[i].position;
        calibrationStableCount++;
      }
    } else {
      autoSettleStarted=0; calibrationStableCount=0;
      for(uint8_t i=0;i<4;i++) calibrationStableSum[i]=0;
    }
    autoMessage="校准角点 "+String(calibrationCorner+1)+"/4：目标电流 "+String(autoTelemetry[calibrationCorner].current)+"mA，转速 "+String(autoTelemetry[calibrationCorner].rpm,1)+"RPM"+(autoSettleStarted?"，正在确认稳定":"，持续拉向角点");
    if(autoSettleStarted&&now-autoSettleStarted>=600&&calibrationStableCount>=4) {
      for(uint8_t i=0;i<4;i++) calibrationSamples[calibrationCorner][i]=calibrationStableSum[i]/calibrationStableCount;
      appendLog("CAL corner "+String(calibrationCorner+1)+" captured: "+String(autoTelemetry[0].position,1)+", "+String(autoTelemetry[1].position,1)+", "+String(autoTelemetry[2].position,1)+", "+String(autoTelemetry[3].position,1));
      calibrationCorner++;
      if(calibrationCorner>=4) {
        if(!computeCalibration()) { failAuto(calibrationQualityError.isEmpty()?"绳长线性拟合质量不足，请检查绳路和方向":calibrationQualityError); return; }
        for(uint8_t i=0;i<4;i++) if(!sendTorqueAddress(i,calibrationTensionCurrent,true)) { failAuto("校准后持续张紧命令失败"); return; }
        autoTensionHold=true; autoState=AUTO_IDLE; autoNextSample=now+500;
        autoMessage="四电机校准完成，系数已保存；四绳保持225mA张紧";
      } else if(!startCalibrationCorner()) return;
      return;
    }
    if(now-autoPhaseStarted>90000) { failAuto("角点 "+String(calibrationCorner+1)+" 90秒内未稳定到位；请检查绳路或增大目标电流"); return; }
  }
  if((autoState==AUTO_DRAW_CIRCLE||autoState==AUTO_DRAW_TRIANGLE||autoState==AUTO_DRAW_CUSTOM) && (int32_t)(now-autoNextSample)>=0) {
    if(drawPreparing) {
      autoNextSample=now+200;
      if(!checkAllMotorSafety(MOTOR_HARD_CURRENT_LIMIT)) return;
      bool reached=drawTargetsReached();
      if(reached) {
        if(!drawStartSettledAt) { drawStartSettledAt=now; autoMessage="已到轨迹起点，等待四绳张力稳定 0.3 秒"; }
        else if(now-drawStartSettledAt>=300) { drawPreparing=false; drawPoint=0; trajectoryStarted=now; drawLastSafety=now; autoNextSample=now; autoMessage="原始轨迹已完整接收，开始恒速位置控制"; }
      } else {
        drawStartSettledAt=0;
        if(now-drawPointStarted>30000) { failAuto("移动到轨迹起点超时：至少一台电机未到编码器目标位置"); return; }
      }
      return;
    }
    autoNextSample=now+trajectoryIntervalMs;
    bool safetySampled=false;
    if(now-drawLastSafety>=200) {
      if(drawWaitingForPoint) { if(!checkAllMotorSafety(MOTOR_HARD_CURRENT_LIMIT)) return; }
      else { if(!checkTrajectoryMotorSafetyStep()) return; }
      drawLastSafety=now; safetySampled=true;
    }
    if(drawWaitingForPoint) {
      if(!safetySampled) return;
      bool reached=drawTargetsReached();
      if(reached) {
        drawWaitingForPoint=false;
        if(repeatCustomContinuously&&autoState==AUTO_DRAW_CUSTOM) {
          if(!beginTrajectory(AUTO_DRAW_CUSTOM,customDurationMs,"循环轨迹")) return;
          autoMessage="本轮完成，正在返回起点继续下一轮；按急停结束循环";
        } else {
          autoState=AUTO_IDLE; autoMessage="轨迹绘制完成，四台电机保持当前位置";
        }
      }
      else if(now-drawPointStarted>15000) { failAuto("轨迹点到位超时"); return; }
      return;
    }
    uint32_t trajectoryElapsed=now-trajectoryStarted;
    if(trajectoryElapsed>=trajectoryDurationMs) {
      if(repeatCustomContinuously&&autoState==AUTO_DRAW_CUSTOM&&customPath.size()>=2&&
         hypotf(customPath.front().x-customPath.back().x,customPath.front().y-customPath.back().y)<=1.0f) {
        // A closed path can roll directly into the next cycle without an
        // endpoint stop, return move, or settling delay.
        float x,y; sampleTrajectory(0.0f,x,y);
        lastTrajectoryX=x; lastTrajectoryY=y; trajectoryStarted=now; drawPoint=0; autoNextSample=now;
        autoMessage="闭合轨迹无停顿进入下一轮；按急停结束循环";
        return;
      }
      float x,y,prevX,prevY;
      float previousPhase=max(0.0f,1.0f-trajectoryIntervalMs/(float)trajectoryDurationMs);
      sampleTrajectory(previousPhase,prevX,prevY); sampleTrajectory(1.0f,x,y);
      setDrawTargets(x,y);
      // Repeat the terminal target at the preceding segment's real speed; do
      // not overwrite it with a 1 RPM command just before arrival.
      sendTrajectoryPosition(prevX,prevY,x,y,trajectoryIntervalMs/1000.0f);
      drawWaitingForPoint=true; drawPointStarted=now; autoNextSample=now+200; autoMessage="实时轨迹段完成，正在校正终点"; return;
    }
    float phase=trajectoryElapsed/(float)trajectoryDurationMs;
    uint32_t nextElapsed=min(trajectoryDurationMs,trajectoryElapsed+trajectoryIntervalMs);
    float nextPhase=nextElapsed/(float)trajectoryDurationMs,x,y,nextX,nextY;
    drawPoint=(uint32_t)lroundf(phase*(drawPointCount-1));
    sampleTrajectory(phase,x,y); sampleTrajectory(nextPhase,nextX,nextY);
    sendTrajectoryPosition(x,y,nextX,nextY,max(0.001f,(nextElapsed-trajectoryElapsed)/1000.0f));
    lastTrajectoryX=nextX; lastTrajectoryY=nextY;
    const char *kind=autoState==AUTO_DRAW_CIRCLE?"圆形":(autoState==AUTO_DRAW_TRIANGLE?"三角形":"手绘");
    if(now-drawLastUiUpdate>=200) { autoMessage=String(kind)+"轨迹点 "+String(drawPoint)+"/"+String(drawPointCount)+" · ("+String(x,1)+","+String(y,1)+")mm"; drawLastUiUpdate=now; }
  }
}

void sendJson(const String &json, int code = 200) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json; charset=utf-8", json);
}

void setPneumaticMode(const String &mode) {
  // Always break-before-make: inflation and deflation can never be active together.
  digitalWrite(INFLATE_PUMP_PIN, ACTUATOR_OFF);
  digitalWrite(DEFLATE_PUMP_PIN, ACTUATOR_OFF);
  digitalWrite(DEFLATE_VALVE_PIN, ACTUATOR_OFF);
  inflatePumpOn=false; deflatePumpOn=false; deflateValveOn=false;
  pneumaticMode = "off";
  if (mode == "inflate") {
    digitalWrite(INFLATE_PUMP_PIN, ACTUATOR_ON);
    inflatePumpOn=true;
    pneumaticMode = "inflate";
  } else if (mode == "deflate") {
    digitalWrite(DEFLATE_VALVE_PIN, ACTUATOR_ON);
    digitalWrite(DEFLATE_PUMP_PIN, ACTUATOR_ON);
    deflatePumpOn=true; deflateValveOn=true;
    pneumaticMode = "deflate";
  }
}

void handlePump() {
  noteUserInstruction(false);
  String mode = server.arg("mode");
  if (mode != "inflate" && mode != "deflate" && mode != "off") {
    sendJson("{\"ok\":false,\"error\":\"气泵模式无效\"}", 400);
    return;
  }
  setPneumaticMode(mode);
  String state = pneumaticMode == "inflate" ? "充气" : (pneumaticMode == "deflate" ? "放气（气泵和阀门同时开启）" : "全部关闭");
  sendJson("{\"ok\":true,\"state\":\"" + state + "\",\"message\":\"气路已切换到" + state + "\"}");
}

void handleOutput() {
  noteUserInstruction(false);
  String target=server.arg("target");
  if(target=="deflate") {
    deflatePumpOn=!deflatePumpOn;
    digitalWrite(DEFLATE_PUMP_PIN,deflatePumpOn?ACTUATOR_ON:ACTUATOR_OFF);
  } else if(target=="valve") {
    deflateValveOn=!deflateValveOn;
    digitalWrite(DEFLATE_VALVE_PIN,deflateValveOn?ACTUATOR_ON:ACTUATOR_OFF);
  } else {
    sendJson("{\"ok\":false,\"error\":\"独立输出目标无效\"}",400); return;
  }
  pneumaticMode="manual";
  String state="{\"ok\":true,\"inflate\":"+String(inflatePumpOn?"true":"false")+",\"deflate\":"+String(deflatePumpOn?"true":"false")+",\"valve\":"+String(deflateValveOn?"true":"false")+",\"message\":\"已单独切换 "+String(target=="deflate"?"放气泵 GPIO48":"阀门 GPIO45")+"\"}";
  sendJson(state);
}
void redirectToPortal() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "Open Y42 motor debugger");
}
void sendActionResult(const std::vector<uint8_t> &r) {
  String msg = replyMessage(r); msg.replace("\\", "\\\\"); msg.replace("\"", "\\\"");
  sendJson("{\"ok\":true,\"message\":\"" + msg + "\",\"rx\":\"" + hexBytes(r) + "\"}");
}
long argLong(const char *name, long fallback) { return server.hasArg(name) ? server.arg(name).toInt() : fallback; }
double argDouble(const char *name, double fallback) { return server.hasArg(name) ? server.arg(name).toDouble() : fallback; }

// -------------------- HTTP handlers --------------------
void handleConfig() {
  long addr = argLong("addr", motorAddress), ppr = argLong("ppr", pulsesPerRevolution);
  if (addr < 1 || addr > 255 || ppr < 1) { sendJson("{\"ok\":false,\"error\":\"地址需为 1-255，脉冲数需大于 0\"}", 400); return; }
  motorAddress = (uint8_t)addr; pulsesPerRevolution = (uint32_t)ppr;
  firmwareType = server.arg("fw") == "x" ? FW_X : FW_EMM;
  sendJson("{\"ok\":true,\"message\":\"连接参数已应用\"}");
}

bool startMdns() {
  MDNS.end();
  if (!MDNS.begin("y42")) return false;
  MDNS.addService("http", "tcp", 80);
  return true;
}

void handleWifi() {
  String ssid = server.arg("ssid");
  String password = server.arg("pass");
  ssid.trim();
  if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 63) {
    sendJson("{\"ok\":false,\"error\":\"Wi-Fi 名称或密码长度无效\"}", 400); return;
  }
  preferences.putString("ssid", ssid);
  preferences.putString("pass", password);
  WiFi.disconnect(false, false);
  WiFi.begin(ssid.c_str(), password.c_str());
  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 12000) delay(100);
  if (WiFi.status() != WL_CONNECTED) {
    sendJson("{\"ok\":false,\"error\":\"连接失败；已保存，可检查密码后重试。应急热点仍可使用\"}", 400); return;
  }
  startMdns();
  String ip = WiFi.localIP().toString();
  appendLog("LAN: http://" + ip + " / http://y42.local");
  sendJson("{\"ok\":true,\"message\":\"连接成功，局域网地址 " + ip + "；以后打开 http://y42.local\"}");
}

void handleAutoGet() {
  String msg=autoMessage; msg.replace("\\","\\\\"); msg.replace("\"","\\\"");
  String j="{\"ok\":true,\"active\":"+String(isAutoActive()?"true":"false")+",\"valid\":"+String(calibrationValid?"true":"false")+",\"message\":\""+msg+"\",\"corner\":"+String(calibrationCorner)+",\"motors\":[";
  for(uint8_t i=0;i<4;i++) { if(i)j+=","; j+="{\"slope\":"+String(calibrationSlope[i],6)+",\"intercept\":"+String(calibrationIntercept[i],3)+",\"r2\":"+String(calibrationR2[i],4)+",\"groups\":["+String(calibrationGroupSlope[i][0],6)+","+String(calibrationGroupSlope[i][1],6)+","+String(calibrationGroupSlope[i][2],6)+"],\"spread\":"+String(calibrationSpread[i],6)+",\"position\":"+String(autoTelemetry[i].position,1)+",\"rpm\":"+String(autoTelemetry[i].rpm,1)+",\"current\":"+String(autoTelemetry[i].current)+",\"temperature\":"+String(autoTelemetry[i].temperature)+"}"; }
  j+="]}"; sendJson(j);
}
bool beginTrajectory(AutoState state,uint32_t durationMs,const String &label) {
  for(uint8_t i=0;i<4;i++) if(!enableAddress(MOTOR_IDS[i])) { failAuto("轨迹启动：电机 "+String(i+1)+" 无法使能"); return false; }
  autoTensionHold=false; autoState=state; trajectoryDurationMs=durationMs;
  float x,y; sampleTrajectory(0,x,y);
  setDrawTargets(x,y);
  if(!moveBallTo(x,y)) { failAuto("无法移动到轨迹起点"); return false; }
  drawPoint=0; drawPointCount=durationMs/trajectoryIntervalMs+1; drawPreparing=true; drawWaitingForPoint=false;
  lastTrajectoryX=x; lastTrajectoryY=y;
  drawPointStarted=millis(); autoNextSample=0; drawLastSafety=0; drawSafetyMotor=0; drawSafetyField=0; drawStartSettledAt=0; drawLastUiUpdate=0;
  autoMessage="正在移动到"+label+"起点"; return true;
}
float calibratedMaxTrajectorySpeed() {
  float maxSlope=0.0f;
  for(uint8_t i=0;i<4;i++) maxSlope=max(maxSlope,fabsf(calibrationSlope[i]));
  if(maxSlope<0.001f) return 300.0f;
  return min(5000.0f,TRAJECTORY_MAX_MOTOR_RPM*6.0f/(maxSlope*TRAJECTORY_RPM_FEEDFORWARD));
}
void handleAutoPost() {
  noteUserInstruction(true);
  String action=server.arg("action");
  if(action=="abort") { stopAllMotors(); autoTensionHold=false; repeatCustomContinuously=false; autoState=AUTO_IDLE; autoMessage="用户执行四电机急停，轨迹循环已退出"; sendJson("{\"ok\":true,\"message\":\"四台电机已停止，轨迹循环已退出\"}"); return; }
  if(action=="stop_loop") {
    bool wasRepeating=repeatCustomContinuously; repeatCustomContinuously=false;
    autoMessage=wasRepeating?"已停止后续循环，当前轮结束后保持终点":"当前没有正在运行的循环轨迹";
    sendJson("{\"ok\":true,\"message\":\""+autoMessage+"\"}"); return;
  }
  if(isAutoActive()) { sendJson("{\"ok\":false,\"error\":\"自动流程正在运行，请先急停\"}",409); return; }
  if(action=="calibrate") {
    repeatCustomContinuously=false;
    float side=server.arg("side").toFloat(), ball=server.arg("ball").toFloat();
    long strong=argLong("strong",1300), weak=argLong("weak",225);
    if(side<50||side>1000||ball<1||ball>=side||strong<50||strong>MOTOR_OPERATING_CURRENT_LIMIT||weak<20||weak>=strong) { sendJson("{\"ok\":false,\"error\":\"校准参数范围无效\"}",400); return; }
    effectiveSideMm=side-ball/2.0f; calibrationStrongCurrent=strong; calibrationTensionCurrent=weak;
    // Keep the last known-good calibration in NVS until a complete new fit passes.
    calibrationValid=false; autoTensionHold=false;
    for(uint8_t i=0;i<4;i++) { auto v=transact({MOTOR_IDS[i],0x1F,0x6B}); if(!validAddressReply(v,MOTOR_IDS[i],0x1F,7)){ failAuto("启动检查：电机 "+String(i+1)+" 不在线"); handleAutoGet(); return; } }
    if(!checkAllMotorSafety(MOTOR_HARD_CURRENT_LIMIT)) { handleAutoGet(); return; }
    calibrationCorner=0; autoState=AUTO_IDLE; autoMessage="开始校准";
    if(!startCalibrationCorner()) { handleAutoGet(); return; }
    sendJson("{\"ok\":true,\"message\":\"四角自动校准已启动\"}"); return;
  }
  if(action=="circle"||action=="triangle") {
    repeatCustomContinuously=false;
    if(!calibrationValid) { sendJson("{\"ok\":false,\"error\":\"请先完成四角校准\"}",409); return; }
    if(!beginTrajectory(action=="circle"?AUTO_DRAW_CIRCLE:AUTO_DRAW_TRIANGLE,action=="circle"?1200:900,action=="circle"?"圆形":"三角形")) { handleAutoGet(); return; }
    sendJson("{\"ok\":true,\"message\":\"轨迹测试已启动\"}"); return;
  }
  if(action=="loop_path") {
    if(!calibrationValid) { sendJson("{\"ok\":false,\"error\":\"请先完成四角校准\"}",409); return; }
    if(customPath.size()<2||customCumulative.size()!=customPath.size()||customDurationMs==0) { sendJson("{\"ok\":false,\"error\":\"还没有可重复的手绘轨迹，请先绘制并运行一次\"}",409); return; }
    repeatCustomContinuously=true;
    if(!beginTrajectory(AUTO_DRAW_CUSTOM,customDurationMs,"循环轨迹")) { repeatCustomContinuously=false; handleAutoGet(); return; }
    sendJson("{\"ok\":true,\"message\":\"已开始循环运行上次轨迹；按急停结束\"}"); return;
  }
  sendJson("{\"ok\":false,\"error\":\"未知自动操作\"}",404);
}

void handlePath() {
  noteUserInstruction(true);
  repeatCustomContinuously=false;
  if(isAutoActive()) { sendJson("{\"ok\":false,\"error\":\"自动流程正在运行，请先急停\"}",409); return; }
  if(!calibrationValid) { sendJson("{\"ok\":false,\"error\":\"请先完成四角校准\"}",409); return; }
  String data=server.arg("points");
  if(data.length()<7||data.length()>30000) { sendJson("{\"ok\":false,\"error\":\"手绘轨迹数据长度无效（最多约2000点）\"}",400); return; }
  std::vector<PathPoint> points; points.reserve(2000); int start=0;
  while(start<(int)data.length()&&points.size()<2000) {
    int end=data.indexOf(';',start); if(end<0) end=data.length(); int comma=data.indexOf(',',start);
    if(comma<=start||comma>=end) { sendJson("{\"ok\":false,\"error\":\"手绘轨迹格式错误\"}",400); return; }
    float nx=data.substring(start,comma).toFloat(),ny=data.substring(comma+1,end).toFloat();
    if(nx<0||nx>1||ny<0||ny>1) { sendJson("{\"ok\":false,\"error\":\"手绘坐标超出画布\"}",400); return; }
    // The whole canvas maps one-to-one to the full calibrated effective range.
    points.push_back(PathPoint(effectiveSideMm*nx,effectiveSideMm*(1.0f-ny))); start=end+1;
  }
  if(start<(int)data.length()) { sendJson("{\"ok\":false,\"error\":\"原始轨迹超过2000点，已拒绝运行且未截断\"}",400); return; }
  if(points.size()<2) { sendJson("{\"ok\":false,\"error\":\"至少需要两个轨迹点\"}",400); return; }
  size_t rawPointCount=points.size();
  // Remove sub-millimeter touch noise, then round corners once with Chaikin's
  // method. This keeps the drawn shape while avoiding rapid motor reversals.
  std::vector<PathPoint> filtered; filtered.reserve(points.size()); filtered.push_back(points.front());
  for(size_t i=1;i+1<points.size();i++) {
    if(hypotf(points[i].x-filtered.back().x,points[i].y-filtered.back().y)>=1.0f) filtered.push_back(points[i]);
  }
  if(hypotf(points.back().x-filtered.back().x,points.back().y-filtered.back().y)>0.05f) filtered.push_back(points.back());
  if(filtered.size()>=3) {
    std::vector<PathPoint> smooth; smooth.reserve(filtered.size()*2); smooth.push_back(filtered.front());
    for(size_t i=0;i+1<filtered.size();i++) {
      const PathPoint &a=filtered[i],&b=filtered[i+1];
      smooth.push_back(PathPoint(a.x*0.75f+b.x*0.25f,a.y*0.75f+b.y*0.25f));
      smooth.push_back(PathPoint(a.x*0.25f+b.x*0.75f,a.y*0.25f+b.y*0.75f));
    }
    smooth.push_back(filtered.back()); points.swap(smooth);
  } else points.swap(filtered);
  customPathLength=0;
  for(size_t i=1;i<points.size();i++) customPathLength+=hypotf(points[i].x-points[i-1].x,points[i].y-points[i-1].y);
  if(customPathLength<3) { sendJson("{\"ok\":false,\"error\":\"手绘轨迹太短\"}",400); return; }
  float requestedSpeed=server.arg("speed").toFloat();
  if(requestedSpeed<30.0f) requestedSpeed=30.0f;
  if(requestedSpeed>5000.0f) requestedSpeed=5000.0f;
  float motorLimitedSpeed=calibratedMaxTrajectorySpeed();
  float controllerLimitedSpeed=TRAJECTORY_MAX_STEP_MM*1000.0f/trajectoryIntervalMs;
  float updateLimitedSpeed=customPathLength*1000.0f/(trajectoryIntervalMs*4.0f);
  requestedSpeed=min(requestedSpeed,min(motorLimitedSpeed,min(controllerLimitedSpeed,updateLimitedSpeed)));
  customPath.clear(); customCumulative.clear(); customPath.reserve(points.size()*2); customCumulative.reserve(points.size()*2);
  customPath.push_back(points.front()); customCumulative.push_back(0.0f); float elapsedSeconds=0;
  for(size_t i=1;i<points.size();i++) {
    float segment=hypotf(points[i].x-points[i-1].x,points[i].y-points[i-1].y);
    elapsedSeconds+=segment/requestedSpeed; customPath.push_back(points[i]); customCumulative.push_back(elapsedSeconds);
  }
  uint32_t duration=(uint32_t)lroundf(elapsedSeconds*1000.0f);
  if(duration<trajectoryIntervalMs*4) { sendJson("{\"ok\":false,\"error\":\"轨迹过短或速度过高，无法严格还原；请降低速度\"}",400); return; }
  if(duration>600000) { sendJson("{\"ok\":false,\"error\":\"轨迹运行时间超过10分钟，请缩短轨迹\"}",400); return; }
  customDurationMs=duration;
  if(!beginTrajectory(AUTO_DRAW_CUSTOM,duration,"手绘轨迹")) { handleAutoGet(); return; }
  sendJson("{\"ok\":true,\"message\":\"已接收 "+String(rawPointCount)+" 个原始点并生成 "+String(customPath.size())+" 个连续平滑点；实际采用 "+String(requestedSpeed,1)+" mm/s（控制连续性上限 "+String(controllerLimitedSpeed,1)+"，电机上限 "+String(motorLimitedSpeed,1)+" mm/s），预计 "+String(duration/1000.0f,2)+" 秒\"}");
}

void handleAction() {
  noteUserInstruction(true);
  String name = server.arg("name");
  std::vector<uint8_t> cmd;
  if(isAutoActive() && name!="stop") { sendJson("{\"ok\":false,\"error\":\"自动流程运行中，仅允许急停\"}",409); return; }
  if(name=="stop") { stopAllMotors(); autoTensionHold=false; repeatCustomContinuously=false; autoState=AUTO_IDLE; autoMessage="四电机已停止，轨迹循环已退出"; sendJson("{\"ok\":true,\"message\":\"四台电机已停止，轨迹循环已退出\",\"rx\":\"\"}"); return; }
  if(name=="enableAll") {
    bool turnOn=argLong("on",1)!=0,ok=true;
    if(!turnOn) stopAllMotors();
    for(uint8_t addr:MOTOR_IDS) {
      if(turnOn) ok=enableAddress(addr)&&ok;
      else ok=commandAccepted(transact({addr,0xF3,0xAB,0x00,0x00,0x6B}),addr,0xF3)&&ok;
    }
    if(!turnOn) { autoTensionHold=false; repeatCustomContinuously=false; motorIdleTimerArmed=false; }
    if(!ok) { sendJson("{\"ok\":false,\"error\":\"至少一台电机未确认使能状态，请检查RS485通讯\"}",409); return; }
    sendJson("{\"ok\":true,\"message\":\"四台电机已"+String(turnOn?"同时使能并锁轴":"同时失能并松轴")+"\",\"rx\":\"\"}"); return;
  }
  if(autoTensionHold) { stopAllMotors(); autoTensionHold=false; }
  if ((name == "speed" || name == "torque" || name == "position" || name == "home") && !ensureMotorEnabled()) {
    sendJson("{\"ok\":false,\"error\":\"电机自动使能失败，请检查供电、保护状态和通讯\"}", 409); return;
  }
  if (name == "enable") cmd = {motorAddress, 0xF3, 0xAB, (uint8_t)(argLong("on", 1) ? 1 : 0), 0x00, 0x6B};
  else if (name == "clear") cmd = {motorAddress, 0x0A, 0x6D, 0x6B};
  else if (name == "release") cmd = {motorAddress, 0x0E, 0x52, 0x6B};
  else if (name == "abortHome") cmd = {motorAddress, 0x9C, 0x48, 0x6B};
  else if (name == "setHome") cmd = {motorAddress, 0x93, 0x88, 0x01, 0x6B};
  else if (name == "home") {
    long mode = argLong("mode", 0); if (mode < 0 || mode > 5) { sendJson("{\"ok\":false,\"error\":\"回零模式无效\"}", 400); return; }
    cmd = {motorAddress, 0x9A, (uint8_t)mode, 0x00, 0x6B};
  } else if (name == "torque") {
    long dir = argLong("dir", 0), current = argLong("current", 100), slope = argLong("slope", 200);
    double maxRpm = argDouble("maxrpm", 10);
    if (firmwareType != FW_X) { sendJson("{\"ok\":false,\"error\":\"电流/力矩模式仅支持 X 固件\"}", 400); return; }
    if (dir < 0 || dir > 1 || current < 0 || current > MOTOR_OPERATING_CURRENT_LIMIT || slope < 0 || slope > 65535 || maxRpm < 0.1 || maxRpm > 3000) {
      sendJson("{\"ok\":false,\"error\":\"电流模式参数超出范围\"}", 400); return;
    }
    cmd = {motorAddress, 0xC5, (uint8_t)dir};
    pushU16(cmd, slope); pushU16(cmd, current); cmd.push_back(0x00); pushU16(cmd, lround(maxRpm * 10.0)); cmd.push_back(0x6B);
  } else if (name == "speed") {
    long dir = argLong("dir", 0), acc = argLong("acc", 0); double rpm = argDouble("rpm", 0);
    if (dir < 0 || dir > 1 || rpm < 0 || rpm > 3000) { sendJson("{\"ok\":false,\"error\":\"方向或速度超出范围\"}", 400); return; }
    cmd = {motorAddress, 0xF6, (uint8_t)dir};
    if (firmwareType == FW_X) {
      if (acc < 0 || acc > 65535) { sendJson("{\"ok\":false,\"error\":\"X 固件加速度需为 0-65535\"}", 400); return; }
      pushU16(cmd, acc); pushU16(cmd, lround(rpm * 10.0)); cmd.push_back(0x00); cmd.push_back(0x6B);
    } else {
      if (acc < 0 || acc > 255) { sendJson("{\"ok\":false,\"error\":\"Emm 固件加速度需为 0-255\"}", 400); return; }
      pushU16(cmd, lround(rpm)); cmd.push_back((uint8_t)acc); cmd.push_back(0x00); cmd.push_back(0x6B);
    }
  } else if (name == "position") {
    long dir = argLong("dir", 0), acc = argLong("acc", 0), mode = argLong("mode", 2);
    double rpm = argDouble("rpm", 60), deg = argDouble("deg", 360);
    if (dir < 0 || dir > 1 || mode < 0 || mode > 2 || rpm <= 0 || rpm > 3000 || deg < 0) { sendJson("{\"ok\":false,\"error\":\"位置参数超出范围\"}", 400); return; }
    if (firmwareType == FW_X) {
      if (acc < 0 || acc > 65535 || deg > 429496729.5) { sendJson("{\"ok\":false,\"error\":\"X 固件加速度或角度超出范围\"}", 400); return; }
      cmd = {motorAddress, 0xFD, (uint8_t)dir}; pushU16(cmd, acc); pushU16(cmd, acc); pushU16(cmd, lround(rpm * 10.0)); pushU32(cmd, llround(deg * 10.0)); cmd.push_back((uint8_t)mode); cmd.push_back(0x00); cmd.push_back(0x6B);
    } else {
      if (acc < 0 || acc > 255) { sendJson("{\"ok\":false,\"error\":\"Emm 固件加速度需为 0-255\"}", 400); return; }
      uint64_t pulses = llround(deg * pulsesPerRevolution / 360.0);
      if (pulses > 0xFFFFFFFFULL) { sendJson("{\"ok\":false,\"error\":\"换算后的脉冲数超出范围\"}", 400); return; }
      cmd = {motorAddress, 0xFD, (uint8_t)dir}; pushU16(cmd, lround(rpm)); cmd.push_back((uint8_t)acc); pushU32(cmd, pulses); cmd.push_back((uint8_t)mode); cmd.push_back(0x00); cmd.push_back(0x6B);
    }
  } else { sendJson("{\"ok\":false,\"error\":\"未知操作\"}", 404); return; }
  sendActionResult(transact(cmd));
}

void handleRaw() {
  noteUserInstruction(true);
  String s = server.arg("hex"); s.replace(",", " "); s.trim();
  std::vector<uint8_t> data; int start = 0;
  while (start < (int)s.length()) {
    while (start < (int)s.length() && isspace((unsigned char)s[start])) start++;
    if (start >= (int)s.length()) break;
    int end = start; while (end < (int)s.length() && !isspace((unsigned char)s[end])) end++;
    String token = s.substring(start, end); char *tail = nullptr; long n = strtol(token.c_str(), &tail, 16);
    if (!tail || *tail || token.length() > 2 || n < 0 || n > 255) { sendJson("{\"ok\":false,\"error\":\"十六进制格式错误\"}", 400); return; }
    data.push_back((uint8_t)n); start = end;
  }
  if (data.size() < 3 || data.size() > 128) { sendJson("{\"ok\":false,\"error\":\"命令长度需为 3-128 字节\"}", 400); return; }
  sendActionResult(transact(data, 80));
}

void handleStatus() {
  auto rv = simpleCommand(0x24); delay(2); auto rb = simpleCommand(0x26); delay(2);
  auto rp = simpleCommand(0x27); delay(2); auto rs = simpleCommand(0x35); delay(2);
  auto rpos = simpleCommand(0x36); delay(2); auto rt = simpleCommand(0x39); delay(2); auto rf = simpleCommand(0x3A);
  bool online = !rv.empty() || !rb.empty() || !rp.empty() || !rs.empty() || !rpos.empty() || !rt.empty() || !rf.empty();
  String voltage="null", bus="null", phase="null", speed="null", position="null", temp="null"; uint8_t flags=0;
  if (validReply(rv,0x24,5)) voltage=String(readU16(rv,2)/1000.0,2);
  if (validReply(rb,0x26,5)) bus=String(readU16(rb,2));
  if (validReply(rp,0x27,5)) phase=String(readU16(rp,2));
  if (validReply(rs,0x35,6)) { double v=readU16(rs,3); if(firmwareType==FW_X)v/=10.0; if(rs[2]==1)v=-v; speed=String(v, firmwareType==FW_X?1:0); }
  if (validReply(rpos,0x36,8)) { double v=readU32(rpos,3); v=firmwareType==FW_X?v/10.0:v*360.0/65536.0; if(rpos[2]==1)v=-v; position=String(v,1); }
  if (validReply(rt,0x39,5)) temp=String(rt[3]);
  if (validReply(rf,0x3A,4)) flags=rf[2];
  String j="{\"ok\":true,\"online\":"+String(online?"true":"false")+",\"voltage\":"+voltage+",\"busCurrent\":"+bus+",\"phaseCurrent\":"+phase+",\"speed\":"+speed+",\"position\":"+position+",\"temperature\":"+temp;
  j += ",\"flags\":{\"enabled\":"+String((flags&0x01)?"true":"false")+",\"reached\":"+String((flags&0x02)?"true":"false")+",\"stall\":"+String((flags&0x04)?"true":"false")+",\"protected\":"+String((flags&0x08)?"true":"false")+",\"leftLimit\":"+String((flags&0x10)?"true":"false")+",\"rightLimit\":"+String((flags&0x20)?"true":"false")+"}}";
  sendJson(j);
}

void setup() {
  lastUserInstructionAt=millis();
  motorIdleTimerArmed=true;
  pinMode(INFLATE_PUMP_PIN, OUTPUT);
  pinMode(DEFLATE_PUMP_PIN, OUTPUT);
  pinMode(DEFLATE_VALVE_PIN, OUTPUT);
  setPneumaticMode("off");
  Serial.begin(115200);
  MotorSerial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  preferences.begin("y42wifi", false);
  calibrationValid=preferences.getBool("calValid",false);
  if(calibrationValid && preferences.getBytesLength("calSlope")==sizeof(calibrationSlope) && preferences.getBytesLength("calInt")==sizeof(calibrationIntercept)) {
    preferences.getBytes("calSlope",calibrationSlope,sizeof(calibrationSlope)); preferences.getBytes("calInt",calibrationIntercept,sizeof(calibrationIntercept));
    preferences.getBytes("calR2",calibrationR2,sizeof(calibrationR2)); effectiveSideMm=preferences.getFloat("calSide",130.0f); autoMessage="已载入保存的四电机绳长系数";
    if(preferences.getBytesLength("calGroups")==sizeof(calibrationGroupSlope)) preferences.getBytes("calGroups",calibrationGroupSlope,sizeof(calibrationGroupSlope));
    if(preferences.getBytesLength("calSpread")==sizeof(calibrationSpread)) preferences.getBytes("calSpread",calibrationSpread,sizeof(calibrationSpread));
  } else calibrationValid=false;
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  bool apStarted = WiFi.softAP(AP_SSID, nullptr, 1, false, 4); // Open 2.4 GHz AP, channel 1.
  dnsServer.start(53, "*", WiFi.softAPIP());
  appendLog(String("Open AP ") + (apStarted ? "ready: " : "failed: ") + AP_SSID + " http://" + WiFi.softAPIP().toString());
  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  // Captive-portal probes used by Android, iOS/macOS and Windows.
  server.on("/generate_204", HTTP_ANY, redirectToPortal);
  server.on("/gen_204", HTTP_ANY, redirectToPortal);
  server.on("/hotspot-detect.html", HTTP_ANY, redirectToPortal);
  server.on("/library/test/success.html", HTTP_ANY, redirectToPortal);
  server.on("/connecttest.txt", HTTP_ANY, redirectToPortal);
  server.on("/ncsi.txt", HTTP_ANY, redirectToPortal);
  server.on("/redirect", HTTP_ANY, redirectToPortal);
  server.on("/canonical.html", HTTP_ANY, redirectToPortal);
  server.on("/success.txt", HTTP_ANY, redirectToPortal);
  server.on("/fwlink", HTTP_ANY, redirectToPortal);
  server.on("/mobile/status.php", HTTP_ANY, redirectToPortal);
  server.on("/check_network_status.txt", HTTP_ANY, redirectToPortal);
  server.on("/kindle-wifi/wifistub.html", HTTP_ANY, redirectToPortal);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/wifi", HTTP_POST, handleWifi);
  server.on("/api/action", HTTP_POST, handleAction);
  server.on("/api/auto", HTTP_GET, handleAutoGet);
  server.on("/api/auto", HTTP_POST, handleAutoPost);
  server.on("/api/pump", HTTP_POST, handlePump);
  server.on("/api/output", HTTP_POST, handleOutput);
  server.on("/api/path", HTTP_POST, handlePath);
  server.on("/api/raw", HTTP_POST, handleRaw);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/log", HTTP_GET, [](){ String s=serialLog; s.replace("\\","\\\\"); s.replace("\"","\\\""); s.replace("\r",""); s.replace("\n","\\n"); sendJson("{\"ok\":true,\"log\":\""+s+"\"}"); });
  server.onNotFound(redirectToPortal);
  server.begin();
  Serial.printf("Y42 debugger ready: AP started=%d, SSID=%s, http://%s\n", apStarted ? 1 : 0, AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
  if(isAutoActive()) lastUserInstructionAt=millis();
  autoLoop();
  server.handleClient();
  if(!isAutoActive()) dnsServer.processNextRequest();
  uint32_t now=millis();
  if(motorIdleTimerArmed&&!isAutoActive()&&(uint32_t)(now-lastUserInstructionAt)>=MOTOR_IDLE_DISABLE_MS) {
    stopAllMotors(); disableAllMotors(); autoTensionHold=false; motorIdleTimerArmed=false;
    autoMessage="连续10分钟无新指令，四台电机已自动失能";
    appendLog("IDLE: all motors disabled after 10 minutes without a command");
  }
  delay(isAutoActive()?0:2);
}
