# bridge

Bridges between buses.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. A bridge is the completer of one bus and the requester of another, joined by the bus-neutral stalling contract `RegTarget`: the downstream bus is adopted into a `RegTarget`, and the `RegTarget` is bound to the upstream bus.

Each bus is an instance of the `Bus` type class in `hwcore`, with two methods: `bindT` makes completer pins from a `RegTarget`, and `adopt` makes a `RegTarget` from completer pins. The bridge is then one generic function, `bridge`, and adding a bus means adding one instance. APB4 and AXI4-Lite are instances in `amba`, and TL-UL is an instance in `tilelink`.

This package gives two of those bridges requester pins that leave the chip, so they can be synthesized:

| Module | Upstream | Downstream |
| :-- | :-- | :-- |
| `mkApb4ToAxi4Lite` | APB4 completer pins | AXI4-Lite requester pins |
| `mkApb4ToTlul` | APB4 completer pins | TL-UL requester pins |

The AXI4-Lite requester raises AWVALID and WVALID together, holds each VALID with its address and data until the handshake, and treats SLVERR, DECERR and the exclusive-access EXOKAY as errors. The TL-UL requester keeps channel A in registers, holds `d_ready` high so it can take a response in the same cycle as its request, sends a read as Get with a full `a_mask`, sends a fully strobed write as PutFullData and any other write as PutPartialData, aligns the address to the word because APB4 may address a byte inside one, and treats `d_denied`, a corrupt AccessAckData or a wrong response opcode as errors. Every error reaches the APB4 host as PSLVERR. The TL-UL rules were checked against the protocol monitor in rocket-chip as well as the specification.

The testbench drives an APB4 host through four bridges. Downstream are the library's AXI4-Lite and TL-UL binders in front of a register target that stalls for three cycles, a picky AXI4-Lite target that gives AWREADY only after WVALID, gives ARREADY every other cycle and answers late, and a TL-UL target that answers combinationally in the same cycle. It checks writes, reads, byte strobes, errors and the write count, and a monitor on every downstream requester port checks that VALID and the payload stay stable until the handshake and that AWVALID never rises without WVALID.

Bridges with AXI4-Lite, TL-UL or Wishbone upstream, a Wishbone requester, protection attributes, concurrent transfers and bursts are not implemented.

## Specification sources

The specifications this library is implemented against, with their links, digests and the clause-by-clause comparison, are kept on the [`spec` branch](https://github.com/Tape-Out/bridge/tree/spec).

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
