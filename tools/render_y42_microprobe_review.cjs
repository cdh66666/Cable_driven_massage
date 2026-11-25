// Offline visualization only. No controller import or hardware access.
const fs = require('fs');
const path = require('path');
const sharp = require('C:/Users/1/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');
async function main() {
  const run = path.resolve(process.argv[2]);
  const out = path.resolve(process.argv[3]);
  const events = fs.readFileSync(path.join(run, 'events.jsonl'), 'utf8').trim().split(/\r?\n/).map(JSON.parse);
  const result = events.findLast(e => e.event === 'run_end')?.result;
  const commands = events.filter(e => e.event === 'position_probe_serial_write' &&
    e.phase === 'microprobe_command_stream' && e.bytes_written === 61 && !e.error).map(e => {
      const bytes=Buffer.from(e.attempted_tx_hex,'hex');
      if (bytes.length!==61 || bytes[1]!==0xaa) throw Error('Invalid recorded motion frame');
      const targets=[0,1,2,3].map(i=>{
        const offset=4+i*14;
        const sign=bytes[offset+2] ^ [0,0,1,1][i];
        return bytes.readUInt32BE(offset+5)/10*(sign ? -1 : 1);
      });
      return {scheduled_send_time:e.started_monotonic_ns/1e9,targets_deg:targets};
    });
  if (!result || !commands.length) throw Error('No actual command stream; refusing a fabricated motion chart');
  const first = commands[0].scheduled_send_time;
  const duration = commands.at(-1).scheduled_send_time - first;
  const seen = new Set();
  const records = events.filter(e => {
    if (e.event !== 'position_microprobe_raw_record' || e.sampled_at < first || e.sampled_at > first + duration + .03) return false;
    const key = `${e.id}:${e.sampled_at}`;
    if (seen.has(key)) return false;
    seen.add(key); return true;
  });
  const escape = s => String(s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;');
  let parts = [`<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1010" viewBox="0 0 1200 1010"><style>text{font-family:'Microsoft YaHei','Segoe UI',sans-serif;fill:#dbe7f4}.small{font-size:16px}.label{font-size:19px;font-weight:600}</style><rect width="1200" height="1010" fill="#101b2d"/>`,
    `<text x="48" y="48" font-size="28" font-weight="700">实际电机微幅测试 · 目标与编码器反馈</text>`,
    `<text x="48" y="80" class="small">${escape(path.basename(run))}</text>`,
    `<text x="48" y="112" class="small">蓝线：发送目标　橙线：原始编码器反馈　单位：相对起点角度（°）</text>`,
    `<text x="48" y="142" class="small">不代表球心 XY 精度；异常帧触发中止，未完成全部四轴验收。</text>`];
  const x = t => 120 + Math.max(0, Math.min(duration, t)) / duration * 1020;
  for (let axis=1; axis<=4; axis++) {
    const top=176+(axis-1)*168, h=114;
    const rows=records.filter(e=>e.id===axis);
    const origin=result.q0[String(axis)];
    const y = q => top+h-(q+.2)*h;
    parts.push(`<text x="48" y="${top+24}" class="label">M${axis}</text>`);
    for (const q of [0,.5]) parts.push(`<line x1="120" y1="${y(q)}" x2="1140" y2="${y(q)}" stroke="#344559"/><text x="80" y="${y(q)+5}" class="small">${q}</text>`);
    const poly = (points,color) => `<polyline fill="none" stroke="${color}" stroke-width="2.6" points="${points.map(p=>p.map(v=>v.toFixed(2)).join(',')).join(' ')}"/>`;
    parts.push(poly(commands.map(c=>[x(c.scheduled_send_time-first),y(c.targets_deg[axis-1]-origin)]),'#55aaff'));
    parts.push(poly(rows.map(r=>[x(r.sampled_at-first),y(r.position-origin)]),'#ffbb62'));
    const maxRpm = Math.max(...rows.map(r=>Math.abs(r.rpm)));
    parts.push(`<text x="120" y="${top+143}" class="small">原始样本 ${rows.length}　峰值转速 ${maxRpm.toFixed(1)} RPM</text>`);
  }
  parts.push(`<text x="120" y="860" class="small">0 s</text><text x="1070" y="860" class="small">${duration.toFixed(2)} s</text>`);
  const c=result.cadence;
  parts.push(`<text x="48" y="908" class="label">${c.completed_commands} 条指令 · ${c.measured_command_hz.toFixed(2)} Hz · 漏 tick ${c.skipped_ticks} · 最大间隔 ${(c.max_interval_seconds*1000).toFixed(2)} ms</text>`,
    `<text x="48" y="944" class="small">退出原因：${escape(result.reason)}</text>`,
    `<text x="48" y="978" class="small">失能确认：${result.final_disabled_confirmed}　原配置恢复：${result.configuration_restored}　整轮通过：${result.success}</text></svg>`);
  fs.mkdirSync(out, {recursive:true});
  const svg=parts.join('\n');
  fs.writeFileSync(path.join(out,'encoder-traces.svg'),svg,{flag:'wx'});
  await sharp(Buffer.from(svg)).png().toFile(path.join(out,'encoder-traces.png'));
  console.log(JSON.stringify({image:path.join(out,'encoder-traces.png'),commands:commands.length,originalRecords:records.length}));
}
main().catch(e=>{ console.error(e); process.exitCode=1; });
