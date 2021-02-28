[![QUARTIQ Matrix Chat](https://img.shields.io/matrix/quartiq:matrix.org)](https://matrix.to/#/#quartiq:matrix.org)
[![HITL (private)](https://github.com/quartiq/hitl/workflows/Stabilizer/badge.svg)](https://github.com/quartiq/hitl/actions?query=workflow%3AStabilizer)

# Stabilizer Firmware

![Flow diagram](stabilizer_pid.svg)

![Hardware](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)

## Features

* dual channel
* SPI ADC
* SPI DAC
* 500 kHz rate, timed sampling
* 2 µs latency, unmatched between channels
* f32 IIR math
* generic biquad (second order) IIR filter
* anti-windup
* derivative kick avoidance

## Limitations/TODOs

* Fixed AFE gains
* The IP and MAC address are [hardcoded](src/hardware/configuration.rs)
* Expose configurable limits
* 100Base-T only
* Digital IO, GPIO header, AFE header, EEM header are not handled

## Hardware

See https://github.com/sinara-hw/Stabilizer

## Minimal bootstrapping documentation

* Clone or download this
* Get [rustup](https://rustup.rs/)
* Get [cargo-binutils](https://github.com/rust-embedded/cargo-binutils/)
* `rustup target add thumbv7em-none-eabihf`
* `cargo build --release`
* Do not try the debug (default) mode. It is guaranteed to panic.

### Using Cargo-embed

* Install `cargo-embed`: `cargo install cargo-embed`
* Program the device: `cargo embed --bin dual-iir --release`

### Using GDB/OpenOCD

* Get a recent openocd, a JTAG adapter ("st-link" or some clone) and
  everything connected and permissions setup. Most
  [Nucleo](https://www.digikey.de/short/p41h4v) boards have a
  detachable ST-Link v2 and are cheap.[^swd]
* Get a multiarch `gdb` (or a cross arm gdb and edit `.cargo/config` accordingly)
* `openocd -f stabilizer.cfg` and leave it running
* `cargo run --release`

[^swd]: Build a cable: connect a standard 8 conductor ribbon with the wires numbered
  `1-8` to the pins on the St-Link v2 single row 2.54mm connector as `647513(82)`
  (`(i)` marks an unused wire)
  and to the [1.27mm dual row](https://www.digikey.de/short/p41h0n) on Stabilizer as `657483x2x1`
  (`x` marks an unused pin, enumeration is standard for dual row, as in the
  schematic).
  It's just folding the ribbon between wires `5` and `6`. The signals on the ribbon
  are then `NRST,TDI,TDO,TCK,TMS,3V3,GND,GND`.

### Using USB-DFU

* Install the DFU USB tool (`dfu-util`)
* Connect to the Micro USB connector below the RJ45
* Short JC2/BOOT
* `cargo objcopy --release --bin dual-iir -- -O binary dual-iir.bin` or `arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/dual-iir dual-iir.bin`
* `dfu-util -a 0 -s 0x08000000:leave -D dual-iir.bin`

### Using ST-Link virtual mass storage

* `cargo objcopy --release --bin dual-iir -- -O binary dual-iir.bin` or `arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/dual-iir dual-iir.bin`
* Connect the ST-Link debugger
* copy `dual-iir.bin` to the `NODE_H743ZI` USB disk

## Protocol

Stabilizer can be configured via MQTT under the topic `stabilizer/settings/<setting>`. Refer to
[`miniconf`](https://github.com/quartiq/miniconf) for more information about topics.
