# check_update.py
# Segunda implementacion (enteros de Python, independiente de emulador.cpp) de UPDATE:
#   dZ'  = sat27( rsh(dz*6554, 8) )                   (una vez por fila y por imagen)
#   W[j][k] <- sat24( W[j][k] - ((dZ'*a[k] + R) >> (46-fw)) )     k = 0..n_in-1, en orden
#   b[j]    <- sat24( b[j]    - ((dZ'      + Rb) >> (30-fw)) )    (despues de la ultima columna)
#   R  : cercano -> 2^(sh-1) ; piso -> 0 ; estocastico -> bits bajos del LFSR tras UN paso
#        por peso (en orden de columna) y UN paso mas para el sesgo.
#   LFSR de Galois de 32 bits, mascara 0x80200003: s = (s>>1) ^ (0x80200003 if s&1 else 0)
# Compara W, b y estado del LFSR finales con los vectores del emulador.
#
# Uso (desde sim/):
#   python3 ../scripts/check_update.py PREF         vectores del ENTRENAMIENTO (1 imagen; PREF_modo.txt)
#   python3 ../scripts/check_update.py PREF sint    vectores de --test-update (PREF_u3_*, PREF_u2_*; T imagenes)
# Sale con codigo != 0 si falta un archivo o hay algun error. Chequea los VECTORES; no es el RTL.
import sys, os
if len(sys.argv) not in (2, 3) or (len(sys.argv) == 3 and sys.argv[2] != "sint"):
    sys.exit("Uso: python3 check_update.py PREFIJO [sint]")
P, SINT = sys.argv[1], len(sys.argv) == 3

def rd(ruta, bits):
    if not os.path.isfile(ruta):
        sys.exit(f"ERROR: falta {ruta}. Genera los vectores con el emulador v5.")
    out = []
    for l in open(ruta):
        v = int(l, 16)
        out.append(v - (1 << bits) if (bits < 32 and v >= 1 << (bits - 1)) else v)
    return out
def rd_u(ruta, bits):
    return [v & ((1 << bits) - 1) for v in rd(ruta, bits)]

ruta_modo = f"{P}_modo.txt"
if not os.path.isfile(ruta_modo): sys.exit(f"ERROR: falta {ruta_modo}")
modo, *fws = open(ruta_modo).read().split()
fws = [int(x) for x in fws]
if modo not in ("cercano", "estocastico", "piso"): sys.exit(f"ERROR: modo desconocido en {ruta_modo}")

rsh = lambda v, s: (v + (1 << (s - 1))) >> s
sat = lambda v, b: max(-(1 << (b - 1)), min((1 << (b - 1)) - 1, v))
def paso(s): return (s >> 1) ^ (0x80200003 if s & 1 else 0)

def actualizar(W, b, lf, dz, a, nout, nin, fw):
    sh, shb = 46 - fw, 30 - fw
    for j in range(nout):
        dzp = sat(rsh(dz[j] * 6554, 8), 27)
        for k in range(nin):
            p = dzp * a[k]
            if modo == "cercano": R = 1 << (sh - 1)
            elif modo == "piso": R = 0
            else:
                lf[j] = paso(lf[j]); R = lf[j] & ((1 << sh) - 1)
            W[j * nin + k] = sat(W[j * nin + k] - ((p + R) >> sh), 24)
        if modo == "cercano": Rb = 1 << (shb - 1)
        elif modo == "piso": Rb = 0
        else:
            lf[j] = paso(lf[j]); Rb = lf[j] & ((1 << shb) - 1)
        b[j] = sat(b[j] - ((dzp + Rb) >> shb), 24)

if SINT:
    # (nombre, n_out, n_in, capa)
    capas = [(f"{P}_u3", 10, 64, 3), (f"{P}_u2", 64, 128, 2)]
    def archivos(pref, capa):
        return dict(Wpre=f"{pref}_W_pre.hex", bpre=f"{pref}_b_pre.hex", lfpre=f"{pref}_lf_pre.hex",
                    dz=f"{pref}_dz.hex", a=f"{pref}_a.hex", Wpost=f"{pref}_W_post.hex",
                    bpost=f"{pref}_b_post.hex", lfpost=f"{pref}_lf_post.hex")
else:
    capas = [(f"{P}", 10, 64, 3), (f"{P}", 64, 128, 2), (f"{P}", 128, 784, 1)]
    def archivos(pref, capa):
        n = str(capa); ent = {3: "a2", 2: "a1", 1: "x"}[capa]
        return dict(Wpre=f"{pref}_W{n}_pre.hex", bpre=f"{pref}_b{n}_pre.hex", lfpre=f"{pref}_lf{n}_pre.hex",
                    dz=f"{pref}_dz{n}.hex", a=f"{pref}_{ent}.hex", Wpost=f"{pref}_W{n}_post.hex",
                    bpost=f"{pref}_b{n}_post.hex", lfpost=f"{pref}_lf{n}_post.hex")

total = 0
for pref, nout, nin, capa in capas:
    f = archivos(pref, capa)
    W = rd(f["Wpre"], 24); b = rd(f["bpre"], 24); lf = rd_u(f["lfpre"], 32)
    dz = rd(f["dz"], 24); a = rd_u(f["a"], 16)
    T = len(dz) // nout
    if len(W) != nout * nin or len(b) != nout or len(lf) != nout or len(dz) != T * nout or len(a) != T * nin:
        sys.exit(f"ERROR: tamanos inesperados en capa {capa}")
    for t in range(T):
        actualizar(W, b, lf, dz[t * nout:(t + 1) * nout], a[t * nin:(t + 1) * nin], nout, nin, fws[capa - 1])
    eW = sum(x != y for x, y in zip(W, rd(f["Wpost"], 24)))
    eb = sum(x != y for x, y in zip(b, rd(f["bpost"], 24)))
    el = sum(x != y for x, y in zip(lf, rd_u(f["lfpost"], 32)))
    print(f"capa {capa} ({nout}x{nin}, T={T}, {modo}): errores W={eW} b={eb} LFSR={el}")
    total += eW + eb + el
print("PASS" if total == 0 else "FAIL")
sys.exit(0 if total == 0 else 1)