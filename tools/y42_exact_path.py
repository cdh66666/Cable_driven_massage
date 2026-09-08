"""Pure, rest-to-rest polyline timing: geometry is never modified."""
import math


def exact_vertex_profile(path, cruise, acceleration, tick=0.01):
    if not all(math.isfinite(v) and v > 0 for v in (cruise, acceleration, tick)):
        raise ValueError('positive motion limits required')
    if len(path) < 2 or any(not all(math.isfinite(v) for v in point) for point in path):
        raise ValueError('at least two finite path points required')
    ticks=0; times=[0.0]; peak=0.0
    for first,second in zip(path,path[1:]):
        distance=math.dist(first,second)
        seconds=max(2*math.sqrt(distance/acceleration),2*distance/cruise)
        count=max(1,math.ceil(seconds/tick-1e-12))
        ticks+=count;times.append(ticks*tick)
        peak=max(peak,2*distance/(count*tick))
    return times,[0.0 for _ in path],peak
