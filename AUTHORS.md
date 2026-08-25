# Authors and Credits

## Raiden2_MiSTer core

**Author**: Umberto Parisi ([rmonic79](https://github.com/rmonic79))

The original RTL source files for the Raiden II / Raiden DX specific logic
(under `rtl/Raiden2/`, excluding the third-party cores listed below) and the
project wrapper `Raiden2.sv` are copyright Umberto Parisi and distributed
under GNU GPL v3 or later.

This includes the reimplementation of the **Seibu COP** coprocessor
(`Raiden2_cop3.sv`), written from the microcode tables found in the game ROMs
and validated command by command against MAME.

## Third-party components

This core builds on top of excellent open-source projects. All third-party
sources retain their original copyright and license. The core as a whole
is distributed under **GNU GPL v3 or later** to stay compatible with the
most restrictive upstream (JTFRAME / JTCORES).

| Component | Author | Project | License |
|-----------|--------|---------|---------|
| **ucore** — cycle-exact NEC V30 main CPU (microcode-level, validated against real silicon) | Martin Donlon ([wickerwaka](https://github.com/wickerwaka)) | [wickerwaka/nec_test](https://github.com/wickerwaka/nec_test) | GPL-3 |
| **T80** — Zilog Z80 (sound CPU) core | Daniel Wallner | [T80](https://opencores.org/projects/t80) | BSD-like |
| **JT51** — Yamaha YM2151 (OPM) FM synthesizer | Jose Tejada ([@topapate](https://twitter.com/topapate)) | [jotego/jt51](https://github.com/jotego/jt51) | GPL-3 |
| **JT6295** — OKI MSM6295 ADPCM decoder (two instances) | Jose Tejada | [jotego/jt6295](https://github.com/jotego/jt6295) | GPL-3 |
| **JTFRAME** — framework, clock enables, filters, mixer, shift registers | Jose Tejada | [jotego/jtframe](https://github.com/jotego/jtframe) | GPL-3 |
| **Savestate infrastructure** — ssbus, memory_stream, ram adaptors | Martin Donlon ([wickerwaka](https://github.com/wickerwaka)) | [wickerwaka/Arcade-TaitoF2_MiSTer](https://github.com/wickerwaka/Arcade-TaitoF2_MiSTer) | GPL-3 |
| **SDRAM controller** | Sorgelig / MiSTer-devel | [MiSTer-devel](https://github.com/MiSTer-devel) | GPL-3 |
| **MAME** — reference for Seibu hardware, memory maps, COP behaviour, sprite decryption, timing | MAMEDev team | [mamedev/mame](https://github.com/mamedev/mame) | GPL-2+ |
| **sys/ framework** — MiSTer HPS/IO, OSD, video scaler, audio | Sorgelig / MiSTer-devel | [MiSTer-devel/Main_MiSTer](https://github.com/MiSTer-devel/Main_MiSTer) | GPL-3 |

The analog geometry modules `crt_adjust.sv` and `crt_vsize.sv` come from
[MiSTer-CRT-Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust), also by
Umberto Parisi, with help from **Andrea Bogazzi**
([@asturur](https://github.com/asturur)) on the H-Size implementation.

## Reference

- **Raiden II / Raiden DX arcade hardware** — Seibu Kaihatsu, 1993-1994. This
  FPGA core is a reimplementation from hardware documentation, MAME source
  code, the microcode tables inside the game ROMs, and observation of real
  hardware behavior. ROMs are **not** included and must be provided by the
  user.
- **MAME project** — invaluable reference for memory maps, timing, the Seibu
  video/sprite hardware, the COP coprocessor and the sprite decryption.
  [mamedev/mame](https://github.com/mamedev/mame)
