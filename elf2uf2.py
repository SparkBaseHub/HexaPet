import sys
import struct

UF2_MAGIC_START0 = 0x0A324655 # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157 # Random magic
UF2_MAGIC_END    = 0x0AB16F30 # "\0\xf1\xb7\xa1"
FAMILY_ID_RP2040 = 0xe48bff56 # KORREKT: 0xe48bff56 (Raspberry Pi RP2040)

def convert_bin_to_uf2(bin_path, uf2_path):
    with open(bin_path, "rb") as f:
        data = f.read()

    num_blocks = (len(data) + 255) // 256
    target_addr = 0x10000000 # Flash Base Address

    with open(uf2_path, "wb") as out:
        for block_no in range(num_blocks):
            ptr = block_no * 256
            chunk = data[ptr:ptr+256]
            if len(chunk) < 256:
                chunk = chunk + b'\x00' * (256 - len(chunk))

            header = struct.pack(
                "<IIIIIIII",
                UF2_MAGIC_START0,
                UF2_MAGIC_START1,
                0x00002000, # flags: familyID present
                target_addr + ptr,
                256,
                block_no,
                num_blocks,
                FAMILY_ID_RP2040
            )
            padding = b'\x00' * (512 - 32 - 256 - 4)
            footer = struct.pack("<I", UF2_MAGIC_END)
            out.write(header + chunk + padding + footer)

    print(f"[UF2 GENERATOR] Erstellt: {uf2_path} ({len(data)} Bytes)")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python elf2uf2.py <input.bin> <output.uf2>")
        sys.exit(1)
    convert_bin_to_uf2(sys.argv[1], sys.argv[2])