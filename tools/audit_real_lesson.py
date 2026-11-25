import json
from pathlib import Path
from collections import Counter
root=Path(__file__).resolve().parents[1];out=root/'qa/tutorial_landscape/real_lesson_20260908'
rows=json.loads((out/'browser_api_audit.json').read_text(encoding='utf-8'))
posts=[r for r in rows if r.get('body')]
ids=[r for r in posts if r['body'].get('action')=='set-id']
tests=[r for r in posts if r['body'].get('action')=='direction-test']
saves=[r for r in posts if r['body'].get('action')=='direction-save']
patterns=next(r for r in posts if r['body'].get('action')=='all-test-patterns')
word=next(r for r in posts if r['url']=='/api/path')
runs={}
for row in rows:
    run=row.get('result',{}).get('manual_test_last_run')
    if run:runs[run['run_id']]=run
summary={'id_roundtrip':[{'old':r['body']['old'],'new':r['body']['new'],'accepted':r['result'].get('accepted')} for r in ids],
 'direction_tests_per_motor':dict(Counter(r['body']['id'] for r in tests)),
 'direction_toggles_per_motor':dict(Counter(r['body']['id'] for r in saves if r['body'].get('reverse'))),
 'shapes':patterns['result']['shapes'],'word_strokes':len(word['body']['segments']),
 'word_accepted':word['result'].get('ok'),'word_speed':word['result'].get('cruise_speed_mm_s'),
 'errors':[r for r in posts if r.get('result',{}).get('ok') is False], 'runs':list(runs)}
assert summary['id_roundtrip']==[{'old':4,'new':5,'accepted':True},{'old':5,'new':4,'accepted':True}]
assert all(summary['direction_tests_per_motor'].get(i)==2 for i in [1,2,3,4])
assert all(summary['direction_toggles_per_motor'].get(i)==2 for i in [1,2,3,4])
assert len(summary['shapes'])==6 and 'square' not in summary['shapes']
assert summary['word_strokes']==20 and summary['word_accepted'] and not summary['errors']
(out/'acceptance_audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False),flush=True)
