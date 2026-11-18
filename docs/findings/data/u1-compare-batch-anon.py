# Paired before/after comparison of two --batch-anon CSVs (docs/findings/audit-followups.md).
# Usage: python3 u1-compare-batch-anon.py DIR   (DIR holds before-batch-anon10.csv and after-batch-anon10.csv)
import csv, sys, statistics as st, re, math
S=sys.argv[1]
def load(p):
    rows={}
    for r in csv.DictReader(open(p)):
        if r['mode']=='none': continue
        key=(r['opponents'],r['setup'],r['layout'],r['partners'],r['sensor'],r['tracking'],r['seed'])
        rows[key]=r
    return rows
B=load(S+'/before-batch-anon10.csv'); A=load(S+'/after-batch-anon10.csv')
assert B.keys()==A.keys(), (len(B),len(A))
def slowed(r): return float(r['slowed_pct'] if r['focus'].startswith('#') else r['all_slowed_pct'])
def f(r,k):
    v=r[k]; return float(v) if v!='' else float('nan')
metrics=[('slowed %',slowed),('total contact s',lambda r:f(r,'contact_s')),('peak ref contact s',lambda r:f(r,'peak_ref_contact_s')),('ref pins',lambda r:f(r,'pins')),('hold switches/min',lambda r:f(r,'hold_switches_per_min'))]
noisy=[k for k in B if B[k]['sensor']!='Perfect']
perfect=[k for k in B if B[k]['sensor']=='Perfect']
seeds=sorted({k[6] for k in noisy},key=int)
print("noisy runs paired:",len(noisy),"seeds",len(seeds),"perfect runs:",len(perfect))
print("\n== Pooled over all camera rows: per-seed mean over the",len(noisy)//len(seeds),"setups, then mean +- SE over seeds ==")
for name,fn in metrics:
    bs=[];as_=[];ds=[]
    for s in seeds:
        ks=[k for k in noisy if k[6]==s]
        b=st.mean(fn(B[k]) for k in ks); a=st.mean(fn(A[k]) for k in ks)
        bs.append(b);as_.append(a);ds.append(a-b)
    se=st.stdev(ds)/math.sqrt(len(ds))
    print(f"{name:20s} before {st.mean(bs):8.3f} (seed sd {st.stdev(bs):.3f})  after {st.mean(as_):8.3f}  diff {st.mean(ds):+8.3f} +- {se:.3f} (SE)  min/max per-seed diff {min(ds):+.3f}/{max(ds):+.3f}")
# per group
groups={}
for k in noisy: groups.setdefault(k[:6],[]).append(k)
print("\n== Per setup group (camera rows,",len(seeds),"seeds): slowed diff mean +- SE; flagged if |diff| > 2 SE ==")
worse=better=0
lines=[]
for g,ks in sorted(groups.items()):
    d=[slowed(A[k])-slowed(B[k]) for k in ks]
    m=st.mean(d); se=st.stdev(d)/math.sqrt(len(d)) if len(d)>1 else float('nan')
    bsd=st.stdev([slowed(B[k]) for k in ks])
    c=[f(A[k],'contact_s')-f(B[k],'contact_s') for k in ks]
    pk=[f(A[k],'peak_ref_contact_s')-f(B[k],'peak_ref_contact_s') for k in ks]
    pins=(sum(f(B[k],'pins') for k in ks),sum(f(A[k],'pins') for k in ks))
    hs=(st.mean(f(B[k],'hold_switches_per_min') for k in ks),st.mean(f(A[k],'hold_switches_per_min') for k in ks))
    flag=''
    if m < -2*se: flag='WORSE'; worse+=1
    elif m > 2*se: flag='better'; better+=1
    lines.append(f"{' | '.join(g[:5])+' | '+g[5]:95s} slowed {m:+6.1f} +- {se:4.1f} (before sd {bsd:4.1f}) contact {st.mean(c):+6.1f}s peakRef {st.mean(pk):+5.2f}s pins {int(pins[0])}->{int(pins[1])} holdSw/min {hs[0]:5.1f}->{hs[1]:5.1f} {flag}")
print("\n".join(lines))
print(f"groups worse beyond 2 SE: {worse}, better beyond 2 SE: {better}, of {len(groups)}")
print("\n== Perfect tracking (deterministic, single runs) ==")
for k in sorted(perfect):
    b,a=B[k],A[k]
    print(f"{' | '.join(k[:4]):70s} slowed {slowed(b):+6.1f} -> {slowed(a):+6.1f}  contact {f(b,'contact_s'):6.1f} -> {f(a,'contact_s'):6.1f}  peakRef {f(b,'peak_ref_contact_s'):4.2f} -> {f(a,'peak_ref_contact_s'):4.2f}  pins {b['pins']}->{a['pins']}  holdSw/min {f(b,'hold_switches_per_min'):5.1f} -> {f(a,'hold_switches_per_min'):5.1f}")
