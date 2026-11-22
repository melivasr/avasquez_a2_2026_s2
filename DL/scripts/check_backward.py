# check_backward.py
# Verifica, con un calculo ENTERO independiente en Python (segunda implementacion),
# que los vectores dorados de MATVEC_T + DERIV del emulador son correctos:
#   acc[k] = sum_j W[j][k]*dz[j]            (enteros, sin saturar)
#   dA[k]  = sat24(rsh(acc[k], 20))
#   s[k]   = rsh(a[k]*(65536-a[k]), 16)
#   dz_prev[k] = sat24(rsh(dA[k]*s[k], 16))
# rsh(v,s) = (v + 2^(s-1)) >> s  (desplazamiento aritmetico)
#
# Dos modos (desde sim/):
#   python3 ../scripts/check_backward.py PREF          vectores del entrenamiento (--vectores=PREF)
#   python3 ../scripts/check_backward.py PREF sint     vectores de --test-backward (PREF_c3_*, PREF_c2_*)
# Sale con codigo != 0 si falta un archivo o hay algun error. Es un chequeo de los
# VECTORES (emulador vs. esta segunda implementacion); NO es el RTL.
import sys, os
if len(sys.argv) not in (2, 3) or (len(sys.argv) == 3 and sys.argv[2] != "sint"):
    sys.exit("Uso: python3 check_backward.py PREFIJO [sint]")
P, SINT = sys.argv[1], len(sys.argv) == 3

def rd(ruta, bits):
    if not os.path.isfile(ruta):
        sys.exit(f"ERROR: falta {ruta}. Genera los vectores con el emulador v4 "
                 f"({'--test-backward --caso=N ' if SINT else ''}--vectores={P}).")
    out = []
    for l in open(ruta):
        v = int(l, 16)
        out.append(v - (1 << bits) if v >= 1 << (bits - 1) else v)
    return out

rsh = lambda v, s: (v + (1 << (s - 1))) >> s
sat = lambda v, b: max(-(1 << (b - 1)), min((1 << (b - 1)) - 1, v))

# (nombre, n_out, n_in, archivo W, dz, a, dA, dz_prev)
if SINT:
    capas = [(f"c{c}", no, ni, *[f"{P}_c{c}_{x}.hex" for x in ("W", "dz", "a", "dA", "dzp")])
             for c, no, ni in ((3, 10, 64), (2, 64, 128))]
else:
    capas = [("capa 3->2", 10, 64, f"{P}_W3_pre.hex", f"{P}_dz3.hex", f"{P}_a2.hex", f"{P}_dA2.hex", f"{P}_dz2.hex"),
             ("capa 2->1", 64, 128, f"{P}_W2_pre.hex", f"{P}_dz2.hex", f"{P}_a1.hex", f"{P}_dA1.hex", f"{P}_dz1.hex")]

total = 0
for nombre, nout, nin, fW, fdz, fa, fda, fdzp in capas:
    W = rd(fW, 24); dz = rd(fdz, 24); da = rd(fda, 24); dzp = rd(fdzp, 24)
    a = [v & 0xFFFF for v in rd(fa, 16)]            # Q0.16 SIN signo
    if not (len(W) == nout * nin and len(dz) == nout and len(a) == nin and len(da) == nin and len(dzp) == nin):
        sys.exit(f"ERROR: tamanos inesperados en {nombre}")
    bad = nsat = 0
    for k in range(nin):
        d = rsh(sum(W[j * nin + k] * dz[j] for j in range(nout)), 20)
        nsat += abs(d) >= 1 << 23 and d != -(1 << 23)
        d = sat(d, 24)
        s = rsh(a[k] * (65536 - a[k]), 16)
        bad += (d != da[k]) + (sat(rsh(d * s, 16), 24) != dzp[k])
    print(f"{nombre}: errores={bad}  dA que saturan={nsat}/{nin}  max|dA|={max(map(abs, da))}")
    total += bad
print("PASS" if total == 0 else "FAIL")
sys.exit(0 if total == 0 else 1)