"""All 0.1-second source frames corresponding to the right-side evidence clip."""
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
CAM=ROOT/'qa/camera_runs/20260908_143619_130638'
OUT=ROOT/'qa/slack_audit_20260908/full_review'
OUT.mkdir(exist_ok=True)
for page in range(37):
    sheet=np.full((1060,1200,3),255,dtype=np.uint8)
    for j in range(12):
        i=page*12+j
        if i>432:continue
        t=i/10
        source=25.9834218025+t
        # Extraction is at 10 fps; nearest source sample, no interpolated frame.
        frame=CAM/'frames'/f'frame_{round(source*10)+1:05}.jpg'
        if not frame.exists(): frame=CAM/f'frame_{round(source*10)+1:05}.jpg'
        im=cv2.resize(cv2.imread(str(frame)),(400,225))
        x=j%3*400;y=j//3*265
        sheet[y:y+225,x:x+400]=im
        cv2.putText(sheet,f'Clip {t:.1f}s / camera {source:.1f}s',(x+8,y+248),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,0,0),1,cv2.LINE_AA)
    cv2.imwrite(str(OUT/f'page_{page+1:02}.jpg'),sheet,[cv2.IMWRITE_JPEG_QUALITY,92])
print(OUT)
