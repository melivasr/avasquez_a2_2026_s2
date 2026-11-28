# aplicar_sint.py - ediciones de sintetizabilidad (paso 3). Ejecutar desde DL/.
# Aborta SIN escribir nada si algun ancla no aparece exactamente las veces esperadas.
import os, re, shutil, sys

def leer(f):
    if not os.path.isfile(f):
        sys.exit(f"ERROR: no existe {f} (ejecuta desde DL/)")
    return open(f).read()

# ---------- lane_mac.sv: bancos pequenos a MLAB ----------
f1 = "rtl/acelerador/lane_mac.sv"; s1 = leer(f1)
if "ramstyle" in s1: sys.exit("lane_mac.sv ya tiene ramstyle")
s1, n = re.subn(r'^([ \t]*)(logic\s[^\n;]*\b(?:bmem|dzmem|lfmem)\s*\[)',
                r'\1(* ramstyle = "MLAB" *) \2', s1, flags=re.M)
if n != 3: sys.exit(f"ABORTO lane_mac.sv: {n} coincidencias (esperadas 3)")

# ---------- motor_dl.sv: sin multiplicadores en carga/lectura ni en f_bstep ----------
f2 = "rtl/acelerador/motor_dl.sv"; s2 = leer(f2)
if "f_rowoff" in s2: sys.exit("motor_dl.sv ya editado")
NUEVA = """case (l)
            2'd1: f_bstep = (P1 - 1) * 784;
            2'd2: f_bstep = (P2 - 1) * 128;
            default: f_bstep = (P3 - 1) * 64;
        endcase
    endfunction

    // Offset de fila en el banco de W de una lane: q * entradas(lay), con q = fila / L.
    // Se escribe con desplazamientos para NO inferir un multiplicador (784 = 512 + 256 + 16).
    function automatic [AW-1:0] f_rowoff(input [1:0] l, input [AW-1:0] q);
        case (l)
            2'd1:    f_rowoff = (q << 9) + (q << 8) + (q << 4);
            2'd2:    f_rowoff = q << 7;
            default: f_rowoff = q << 6;
        endcase
    endfunction"""
pat = r"f_bstep = \(f_np\(l\) - 8'd1\) \* f_ni\(l\);\s*endfunction"
if len(re.findall(pat, s2)) != 1: sys.exit("ABORTO motor_dl.sv: ancla de f_bstep")
s2 = re.sub(pat, lambda m: NUEVA, s2)
for a, b in (("(ld_row / L) * f_ni(ld_lay)", "f_rowoff(ld_lay, ld_row / L)"),
             ("(hr_row / L) * f_ni(hr_lay)", "f_rowoff(hr_lay, hr_row / L)")):
    if s2.count(a) != 1: sys.exit(f"ABORTO motor_dl.sv: ancla {a}")
    s2 = s2.replace(a, b)

# ---------- mut_dl.py: M7 nuevo y M17..M20 ----------
f3 = "scripts/mut_dl.py"; s3 = leer(f3)
if "M17" in s3: sys.exit("mut_dl.py ya editado")
M7 = ''' "M7  salto de raddr del backward (capa 2) sin el -1":("2'd2: f_bstep = (P2 - 1) * 128;","2'd2: f_bstep = P2 * 128;"),\n'''
s3, n = re.subn(r'^ "M7 .*\n', lambda m: M7, s3, flags=re.M)
if n != 1: sys.exit("ABORTO mut_dl.py: no encuentro la linea de M7")
NUEVOS = ''' "M17 f_rowoff capa 1: desplazamiento 4 -> 5":("2'd1:    f_rowoff = (q << 9) + (q << 8) + (q << 4);","2'd1:    f_rowoff = (q << 9) + (q << 8) + (q << 5);"),
 "M18 f_rowoff capa 2: q<<7 -> q<<6":("2'd2:    f_rowoff = q << 7;","2'd2:    f_rowoff = q << 6;"),
 "M19 f_rowoff capa 3: q<<6 -> q<<5":("default: f_rowoff = q << 6;","default: f_rowoff = q << 5;"),
 "M20 salto de raddr del backward (capa 3) sin el -1":("default: f_bstep = (P3 - 1) * 64;","default: f_bstep = P3 * 64;"),
'''
ancla = "}\nsel = sys.argv[1:]"
if s3.count(ancla) != 1: sys.exit("ABORTO mut_dl.py: ancla del diccionario")
s3 = s3.replace(ancla, NUEVOS + ancla)

# ---------- todo bien: respaldo y escritura ----------
os.makedirs("bak_sint", exist_ok=True)
for f, s in ((f1, s1), (f2, s2), (f3, s3)):
    shutil.copy(f, "bak_sint/" + os.path.basename(f))
    open(f, "w").write(s if s.endswith("\n") else s + "\n")
print("OK: lane_mac.sv, motor_dl.sv y mut_dl.py editados (originales en bak_sint/)")
