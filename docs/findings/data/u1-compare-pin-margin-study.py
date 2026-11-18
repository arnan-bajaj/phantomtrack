# Paired before/after comparison of seeded PinMarginStudy runs (docs/findings/audit-followups.md).
# Usage: python3 u1-compare-pin-margin-study.py DIR   (DIR holds {before,after}-pms-seed{0..9}.txt, the printed tables)
import sys,re,statistics as st,math
S=sys.argv[1]
pat=re.compile(r'^(\d\.\d m/s\s+\d+ ms \w+\s+\d \S+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+(\d+)\s+(\d+) \|\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+) \|\s+(\d+)\s+(\d+)\s+([+-]?\d+)%\s+(\S+)\s+([\d.]+)\s+([\d.]+)$')
def load(pfx):
    out={}
    for s in range(10):
        for line in open(f"{S}/{pfx}-pms-seed{s}.txt"):
            m=pat.match(line.rstrip())
            if m:
                g=m.groups()
                out[(g[0],s)]=dict(peak=float(g[1]),refpk=float(g[2]),backoffs=int(g[11]),refpins=int(g[12]),slowed=float(g[13]),touch=float(g[15]),holdsw=float(g[16]),grid=g[14])
    return out
B=load('before');A=load('after')
assert B.keys()==A.keys(), (len(B),len(A))
cfgs=sorted({k[0] for k in B})
print("configs",len(cfgs),"seeds 10")
pooled={m:[] for m in ['slowed','touch','peak','refpk','refpins','holdsw','backoffs']}
for s in range(10):
    for m in pooled:
        pooled[m].append((st.mean(B[(c,s)][m] for c in cfgs), st.mean(A[(c,s)][m] for c in cfgs)))
print("== Pooled over the 24 one-ghost setups: per-seed mean, then over seeds ==")
for m,v in pooled.items():
    d=[a-b for b,a in v]
    print(f"{m:9s} before {st.mean(b for b,a in v):8.3f} (seed sd {st.stdev([b for b,a in v]):.3f}) after {st.mean(a for b,a in v):8.3f} diff {st.mean(d):+.3f} +- {st.stdev(d)/math.sqrt(10):.3f} SE")
print("max peak any-touch before/after:",max(x['peak'] for x in B.values()),max(x['peak'] for x in A.values()))
print("max peak referee before/after:",max(x['refpk'] for x in B.values()),max(x['refpk'] for x in A.values()))
print("total ref pins before/after:",sum(x['refpins'] for x in B.values()),sum(x['refpins'] for x in A.values()))
print("gridlocked before/after:",sum(x['grid']=='YES' for x in B.values()),sum(x['grid']=='YES' for x in A.values()))
print("\n== Per setup (10 seeds): slowed diff mean +- SE, flagged if beyond 2 SE ==")
w=b_=0
for c in cfgs:
    d=[A[(c,s)]['slowed']-B[(c,s)]['slowed'] for s in range(10)]
    m=st.mean(d); se=st.stdev(d)/math.sqrt(10)
    flag='WORSE' if m< -2*se else ('better' if m>2*se else '')
    w+=flag=='WORSE'; b_+=flag=='better'
    t=st.mean(A[(c,s)]['touch']-B[(c,s)]['touch'] for s in range(10))
    pk=(max(B[(c,s)]['peak'] for s in range(10)),max(A[(c,s)]['peak'] for s in range(10)))
    rp=(max(B[(c,s)]['refpk'] for s in range(10)),max(A[(c,s)]['refpk'] for s in range(10)))
    hs=(st.mean(B[(c,s)]['holdsw'] for s in range(10)),st.mean(A[(c,s)]['holdsw'] for s in range(10)))
    print(f"{c:30s} slowed {st.mean(B[(c,s)]['slowed'] for s in range(10)):+5.1f}->{st.mean(A[(c,s)]['slowed'] for s in range(10)):+5.1f} diff {m:+5.1f}+-{se:3.1f} touch {t:+6.1f}s peak {pk[0]:.2f}->{pk[1]:.2f} refPk {rp[0]:.2f}->{rp[1]:.2f} holdSw/m {hs[0]:5.1f}->{hs[1]:5.1f} {flag}")
print("setups worse:",w,"better:",b_)
