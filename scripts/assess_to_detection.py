#!/usr/bin/env python3
"""Toe-off / stance assessment for a gait_fsm_replay run (companion of
eval_hs_replay.py, standard library only).

    scripts/assess_to_detection.py events.csv *_log.csv

Reference push-off = the deepest raw gyro_z in the 400 ms before a swing bout
(gyro_z > 1.5 rad/s for 50 ms); TO events are matched to it. Also prints the
stance anatomy from the gyro alone (landing zero-cross -> push-off trough is a
lower bound of the time on the ground, no detector involved).
"""
import os
sys_path_hack = os.path.dirname(os.path.abspath(__file__))
import sys, bisect, csv, statistics as st
sys.path.insert(0, sys_path_hack)
import eval_hs_replay as E
from collections import defaultdict
logs={}
for p in sys.argv[2:]:
    d=E.load_log(p)
    for f in E.FEET: d[f]['mc']=[(m[0],m[1],0,m[2]) for m in d[f]['motor']]
    logs[os.path.basename(p)]=d
events=defaultdict(list)
for row in csv.DictReader(open(sys.argv[1])):
    events[(row['log'],row['foot'],row['mode'])].append((float(row['t_detect_s']),float(row['t_event_s']),row['label']))
def summ(v):
    if not v: return 'n=0'
    v=sorted(v); q=lambda p:v[min(len(v)-1,int(p*len(v)))]
    return 'n=%3d median %+6.0f  p10 %+6.0f  p90 %+6.0f  min %+6.0f max %+6.0f'%(len(v),st.median(v),q(.1),q(.9),v[0],v[-1])
def ma5(imu):
    g=[s[4] for s in imu]; out=[]
    for i in range(len(g)):
        w=g[max(0,i-4):i+1]; out.append(sum(w)/len(w))
    return out
for foot in ('Left','Right'):
    R=defaultdict(list); C=defaultdict(int); missed=[]; false_to=defaultdict(list)
    for name,log in logs.items():
        imu=log[foot]['imu']; ts=[s[0] for s in imu]; gf=ma5(imu); n=len(imu)
        mc=log[foot]['mc']; mt=[m[0] for m in mc]
        # reference steps: swing bout (gz>1.5 for 5 samples); push-off trough = min gz in 400 ms before bout start; landing zc after
        steps=[]; i=1
        while i<n:
            run=0
            while i<n:
                run=run+1 if imu[i][4]>1.5 else 0
                if run>=5: break
                i+=1
            if i>=n: break
            b0=i-4
            k0=bisect.bisect_left(ts,imu[b0][0]-0.40)
            ktr=min(range(k0,b0+1),key=lambda k:imu[k][4])
            kf=min(range(k0,min(n,b0+5)),key=lambda k:gf[k])
            zc=None
            while i<n and imu[i][0]-imu[b0][0]<1.5:
                if imu[i][4]<0: zc=i; break
                i+=1
            steps.append(dict(tr=imu[ktr][0],depth=imu[ktr][4],fdepth=gf[kf],bout=imu[b0][0],zc=None if zc is None else imu[zc][0]))
            i=(zc or i)+1
        # stance anatomy between consecutive steps: zc(prev landing) -> trough(next push-off)
        for a,b in zip(steps,steps[1:]):
            if a['zc'] is None or not (0.3<b['tr']-a['zc']<1.8): continue
            k0=bisect.bisect_left(ts,a['zc']); k1=bisect.bisect_left(ts,b['tr'])
            flat=[k for k in range(k0,k1) if abs(imu[k][4])<0.3]
            if len(flat)<5: continue
            R['on-ground lower bound zc->trough'].append((b['tr']-a['zc'])*1e3)
            R['  slap: zc->flat'].append((imu[flat[0]][0]-a['zc'])*1e3)
            R['  foot-flat |gz|<0.3'].append((imu[flat[-1]][0]-imu[flat[0]][0])*1e3)
            R['  heel-rise+push-off: flat->trough'].append((b['tr']-imu[flat[-1]][0])*1e3)
            R['swing: trough->zc'].append((b['zc']-b['tr'])*1e3 if b['zc'] else float('nan')) if b['zc'] else None
            if b['zc']: R['cycle zc->zc'].append((b['zc']-a['zc'])*1e3); R['stance % lower bound'].append((b['tr']-a['zc'])/(b['zc']-a['zc'])*100)
        for mode in ('old','contact'):
            ev=events.get((name,foot,mode),[]); to=[e for e in ev if e[2]=='TO']; to_t=[e[0] for e in to]; claimed=set()
            for s in steps:
                if s['depth']>-1.0: continue          # no push-off rotation at all (shuffle)
                C[mode+' ref push-offs']+=1
                k=bisect.bisect_left(to_t,s['tr']-0.05)
                if k<len(to_t) and to_t[k]<=s['tr']+0.30:
                    claimed.add(k)
                    R[mode+' TO detect - raw trough'].append((to[k][0]-s['tr'])*1e3)
                    R[mode+' TO stamp  - raw trough'].append((to[k][1]-s['tr'])*1e3)
                    R[mode+' TO detect - swing>1.5 onset'].append((to[k][0]-s['bout'])*1e3)
                else:
                    C[mode+' missed']+=1
                    if mode=='contact': missed.append((name[9:15],round(s['tr'],2),round(s['depth'],1),round(s['fdepth'],1)))
            hs_t=[e[0] for e in ev if e[2]=='HS']
            for k,e in enumerate(to):
                if k in claimed: continue
                C[mode+' TO not at a push-off']+=1
                j=bisect.bisect_left(hs_t,e[0])-1
                since=None if j<0 else (e[0]-hs_t[j])*1e3
                slip=any(m[3]!=0 for m in mc[bisect.bisect_left(mt,e[0]-1.0):bisect.bisect_right(mt,e[0]+0.5)])
                i0=bisect.bisect_left(ts,e[0]); pk=max([s[4] for s in imu[i0:i0+40]] or [0])
                cat='during/after slip' if slip else ('slap trough (HS<500ms ago)' if since is not None and since<500 else ('weak swing after (peak %.1f)'%pk if pk>0.8 else 'no swing after (turn/shuffle/stand)'))
                false_to[mode+' '+cat.split(' (peak')[0]].append((name[9:15],round(e[0],2)))
    print('=====',foot)
    for k in ['on-ground lower bound zc->trough','  slap: zc->flat','  foot-flat |gz|<0.3','  heel-rise+push-off: flat->trough','swing: trough->zc','cycle zc->zc','stance % lower bound']:
        print('  %-36s %s'%(k,summ([x for x in R[k] if x==x])))
    for mode in ('old','contact'):
        print('  --',mode,': ref push-offs %d, missed %d, TO events not at a push-off %d'%(C[mode+' ref push-offs'],C[mode+' missed'],C[mode+' TO not at a push-off']))
        for k in [' TO detect - raw trough',' TO stamp  - raw trough',' TO detect - swing>1.5 onset']:
            print('  %-36s %s'%(mode+k,summ(R[mode+k])))
        for k,v in false_to.items():
            if k.startswith(mode): print('       ',k,len(v),v[:4])
    d=sorted(m[3] for m in missed); print('  contact-mode missed push-offs: filtered trough depths',d[:40])
