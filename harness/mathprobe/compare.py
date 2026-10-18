import math, os, struct, sys
def load(p):
    b = open(p, "rb").read(); return struct.unpack(f"<{len(b)//8}d", b)
def key(x):  # monotone integer map of doubles, for ulp distance
    i = struct.unpack("<q", struct.pack("<d", x))[0]
    return i if i >= 0 else -(i & 0x7FFFFFFFFFFFFFFF)
fns = ["sin","cos","tan","asin","acos","atan","sinh","cosh","ln","log10","cbrt","atan2","powf","powi"]
REF = "py-glibc-amd64"
cols = [c for c in ["py-glibc-arm64", "rust-host", "rust-wasm32"] if os.path.isdir(f"out/{c}")]
print(f"{'fn':6} " + " ".join(f"{c:>26}" for c in cols) + "   (differing / 200000, max ulp; vs glibc x86_64)")
for f in fns:
    ref = REF if f != "powi" else "rust-host"
    if not os.path.isdir(f"out/{REF}"): ref = "rust-host"
    try: r = load(f"out/{ref}/{f}.bin")
    except FileNotFoundError: continue
    cells = []
    for c in cols:
        if c == ref: cells.append(f"{'(reference)':>26}"); continue
        try: v = load(f"out/{c}/{f}.bin")
        except FileNotFoundError: cells.append(f"{'-':>26}"); continue
        n = 0; mu = 0
        for a, b in zip(r, v):
            if a == b or (math.isnan(a) and math.isnan(b)) : 
                if a == b and a == 0 and math.copysign(1,a) != math.copysign(1,b): n += 1
                continue
            n += 1
            if math.isfinite(a) and math.isfinite(b): mu = max(mu, abs(key(a) - key(b)))
            else: mu = float("inf")
        cells.append(f"{n:>8} ({100*n/len(r):6.3f}%) ulp<={mu:<5}")
    print(f"{f:6} " + " ".join(cells))
