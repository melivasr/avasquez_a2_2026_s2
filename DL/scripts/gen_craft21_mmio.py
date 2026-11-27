#!/usr/bin/env python3
"""Genera imágenes de Craft21 y valores esperados de decodificación para las pruebas de MMIO."""

import argparse
from pathlib import Path

NOP = 0x00580000
FREEZE = 0x00600000


def reg(index):
    if not 0 <= index < 32:
        raise ValueError(f"invalid register x{index}")
    return index


def field(value, width, signed=False):
    minimum = -(1 << (width - 1)) if signed else 0
    maximum = (1 << (width - int(signed))) - 1
    if not minimum <= value <= maximum:
        raise ValueError(f"{value} does not fit in {width} bits")
    return value & ((1 << width) - 1)


def addi(rd, rs, immediate, variant=0):
    return ((field(immediate, 16, variant == 2) << 16) |
            (variant << 14) | (reg(rs) << 9) | (reg(rd) << 4) | 1)


def memory(opcode, load, register, base, offset):
    offset = field(offset, 13, signed=True)
    high = (offset >> 5) << 20
    low = offset & 31
    operands = ((low << 14) | (reg(register) << 4)) if load else (
        (reg(register) << 14) | (low << 4))
    return high | (int(load) << 19) | operands | (reg(base) << 9) | opcode


def lw(rd, base, offset):
    return memory(2, True, rd, base, offset)


def sw(rs, base, offset):
    return memory(2, False, rs, base, offset)


def bne(rs1, rs2, words):
    offset = field(words, 13, signed=True)
    return ((offset >> 5) << 24) | (1 << 19) | (reg(rs2) << 14) | (
        reg(rs1) << 9) | ((offset & 31) << 4) | 6


def jal(rd, words):
    offset = field(words, 18, signed=True)
    return ((offset >> 10) << 20) | ((offset & 1023) << 9) | (reg(rd) << 4) | 4


def write_hex(path, values, digits):
    with path.open("w") as out:
        out.writelines(f"{value:0{digits}x}\n" for value in values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    smoke = [addi(1, 0, 1, 1), lw(2, 1, 0), sw(2, 0, 0x100), FREEZE]
    integrated = [
        *smoke[:-1],
        lw(3, 1, 0), lw(4, 1, 0x34), addi(5, 4, 1),
        sw(3, 0, 0x104), sw(5, 0, 0x108),
        addi(6, 0, 7), sw(6, 1, 8),
        memory(3, False, 6, 1, 8), sw(6, 1, 9), lw(8, 1, 9),
        memory(3, True, 9, 1, 8),
        sw(6, 1, 0x80),  # Fuera de MMIO, la RAM conserva sus alias de dirección.
        addi(0, 0, 9), NOP, NOP, addi(7, 0, 1),
        jal(10, 2), FREEZE,  # Se descarta el FREEZE del camino que no se tomó.
        sw(10, 0, 0x10C),
        sw(6, 1, 0x7C), lw(11, 1, 0x7C),  # Palabra reservada en la ventana MMIO.
        FREEZE,
    ]
    waited = [
        addi(1, 0, 1, 1), addi(6, 0, 33), lw(2, 1, 0),
        bne(6, 0, 3), sw(6, 0, 0x110), FREEZE,
        sw(2, 1, 8), sw(2, 0, 0x100), FREEZE,
    ]
    # Palabra, campos rd, rs1 y rs2, tamaño, escritura en registro,
    # escritura en memoria, fuente de WB e inmediato extendido.
    unit = [
        (addi(1, 0, 1, 1), 1, 0, 5, 1, 1, 0, 0, 0x10000),
        (addi(7, 5, -1, 2), 7, 5, 30, 1, 1, 0, 0, -1),
        (lw(2, 1, -4), 2, 1, 28, 1, 1, 0, 1, -4),
        (sw(6, 3, -32), 0, 3, 6, 1, 0, 1, 0, -32),
        (memory(3, True, 9, 1, 8), 9, 1, 8, 0, 1, 0, 1, 8),
        (memory(3, False, 6, 1, 8), 8, 1, 6, 0, 0, 1, 0, 8),
        (bne(6, 0, -3), 29, 6, 0, 1, 0, 0, 0, -12),
        (jal(10, 2), 10, 2, 0, 1, 1, 0, 2, 8),
    ]
    for name, program in (("id", smoke), ("integrado", integrated), ("espera", waited)):
        directory = args.output / name / "programs"
        directory.mkdir(parents=True, exist_ok=True)
        write_hex(directory / "program.hex",
                  program + [NOP] * (16384 - len(program)), 8)
        data = bytearray(65536)
        data[8:13] = bytes.fromhex("11 22 33 44 55")
        write_hex(directory / "data.hex", data, 2)

    with (args.output / "decode.txt").open("w") as out:
        for word, *fields, immediate in unit:
            print(f"{word:08x}", *fields, f"{immediate & 0xffffffff:08x}", file=out)


if __name__ == "__main__":
    main()
