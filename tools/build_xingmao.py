"""Single-line handwritten 星猫. No motion is submitted by this exporter."""
import json
from pathlib import Path

star=[[(18,8),(18,28)],[(18,8),(42,8),(42,28)],[(18,18),(42,18)],[(18,28),(42,28)],[(24,32),(19,39)],[(21,36),(42,36)],[(20,44),(40,44)],[(30,31),(30,52)],[(15,52),(45,52)]]
cat=[[(16,6),(8,14)],[(7,10),(17,19),(17,42),(13,50),(9,47)],[(16,23),(6,35)],[(23,13),(50,13)],[(30,7),(30,19)],[(43,7),(43,19)],[(24,25),(24,48)],[(24,25),(49,25),(49,48)],[(24,36),(49,36)],[(36,25),(36,48)],[(24,48),(49,48)]]
segments=[]
for offset,strokes in [(0,star),(60,cat)]:
    segments += [[{'x':round(.05+.9*x/60,6),'y':round(.02+.96*(y+offset)/120,6)} for x,y in stroke] for stroke in strokes]
payload={'segments':segments,'speed':50,'confirmed':True,'open_loop':True,'repeat':False}
root=Path(__file__).resolve().parents[1]
(root/'tools/xingmao_path.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
# Physical scaled aspect ratio; transfer lines intentionally excluded.
paths=[]
for stroke in segments:
    points=' '.join(f'{p["x"]*440:.2f},{p["y"]*592:.2f}' for p in stroke)
    paths.append(f'<polyline points="{points}" fill="none" stroke="#166bc5" stroke-width="3"/>')
(root/'qa/slack_audit_20260908/xingmao_target.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg" width="440" height="592" viewBox="0 0 440 592"><rect width="440" height="592" fill="white"/>'+''.join(paths)+'</svg>',encoding='utf-8')
print('Exported 20 strokes; single-line target only, not an optical result')
