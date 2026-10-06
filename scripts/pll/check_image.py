"""Check the complete Flash image while the CPU continues running."""
import pathlib
def check_image(tcl,elf):
    expected=pathlib.Path(elf).with_suffix('.bin').read_bytes()
    if not expected: raise RuntimeError('Empty firmware BIN')
    for offset in range(0,len(expected),1024):
        chunk=expected[offset:offset+1024]
        actual=bytes(int(x,0) for x in tcl(f'read_memory {0x08000000+offset} 8 {len(chunk)}').split())
        if actual!=chunk:
            raise RuntimeError('Flash and ELF/BIN do not match; verify the matching build before using RAM symbols')
