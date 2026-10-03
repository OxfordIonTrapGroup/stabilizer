[![Continuous Integration](https://github.com/OxfordIonTrapGroup/stabilizer/actions/workflows/ci.yml/badge.svg)](https://github.com/OxfordIonTrapGroup/stabilizer/actions/workflows/ci.yml)

# Stabilizer Firmware (Oxford Ion Trap Group)

Firmware for the [Sinara Stabilizer](https://github.com/sinara-hw/Stabilizer) board (two fast
ADC/DAC channels, Ethernet/MQTT, USB) and its mezzanines, as used in the Oxford Ion Trap Group:
the upstream applications plus the group's own ones for fibre noise cancellation, coil current
stabilisation with mains feedforward, and the 674 nm laser lock.

## Relationship to upstream

This is a fork of [`quartiq/stabilizer`](https://github.com/quartiq/stabilizer). Everything
generic is documented upstream and not repeated here:

* the [Stabilizer documentation](https://quartiq.de/stabilizer) covers setup (power, network,
  MQTT broker, flashing over SWD or USB DFU, the USB serial terminal), the MQTT settings,
  telemetry and streaming interfaces, and the upstream applications (`dual-iir`, `lockin`, `dds`,
  `fls`, `mpll`);
* the [hardware](https://github.com/sinara-hw/Stabilizer/wiki) and
  [Pounder](https://github.com/sinara-hw/Pounder/wiki) wikis describe the boards.

Branches: `next` is the group's development branch. It is upstream `main` as of the v0.11 merge
(RTIC 2, miniconf 0.20, the current settings tree layout) followed by the group's additions, and
upstream is merged in from time to time. `master` is the earlier generation of this fork (before
that merge, v0.9-era firmware with miniconf 0.9), which some lab devices still run; the GUIs can
talk to both.

What this fork adds on top of upstream:

* the applications `fnc`, `current_sense` and `l674` (below), and support for the group's
  current sense mezzanine (its offset DAC on SPI1, the MCU's auxiliary DAC outputs) and for the
  EEM connector's GPIO lines when no Urukul is attached;
* a fix in the MQTT settings client (through a fork of `minimq`): the device answers up to ten
  requests at a time instead of one, and accepts requests of up to 1 kB, which several clients
  (GUIs, scripts) on one device need;
* additions to the `py/stabilizer` Python package (`stream_parser` with per-source decoders,
  Pounder DDS conversions, sample periods per application), which the GUIs use.

Changes are listed in [`CHANGELOG.md`](CHANGELOG.md) under *Unreleased*, continuing upstream's.

## Applications

All applications share the upstream structure: settings under `dt/sinara/<app>/<id>/settings/`
(the ID is the MAC address unless configured otherwise), telemetry at `.../telemetry`, and the
UDP data stream. The settings of the group's applications are documented in the crate docs at the
top of their `src/bin/<app>.rs` (the `Settings` struct is the settings reference; build the docs
with `cargo doc` as described upstream, or read the source). `cargo run --bin <app> --target
<host-tuple>` prints the default settings and their JSON schema without hardware.

| Application | Hardware | Description |
| :--- | :--- | :--- |
| `dual-iir`, `lockin`, `dds`, `fls`, `mpll` | as upstream | Upstream applications, see the [documentation](https://quartiq.de/stabilizer) |
| [`fnc`](src/bin/fnc.rs) | Pounder | Fibre noise cancellation: the filtered beat-note error signal of each channel is integrated into the phase of a Pounder DDS output (`ch/N/pounder/...` for the DDS frequencies, amplitudes and attenuations). The stream carries ADC samples and the DDS phase offset words |
| [`current_sense`](src/bin/current_sense.rs) | current sense mezzanine | `dual-iir` with a mains-synchronised harmonic feedforward on channel 0: the rising edges of a mains reference on DI0 are timestamped, a PLL tracks the mains phase, and the sum of the first five harmonics (`feedforward_harmonics/N` = amplitude and phase) is added to the channel 0 input. `frontend_offset` and `feedback_offset` set the board's offset DAC and the feedback offset current. Called `ff_fb` until October 2026 |
| [`l674`](src/bin/l674.rs) | EEM GPIO | Lock of the 674 nm SolsTiS onto its cavity: channel 0 drives the fast PZT from the error signal, channel 1 drives the slow PZT from the channel 0 output, with two biquads each and a gain ramp when the lock is switched on. ADC1 is the cavity transmission: lock detection with threshold and holdoff on LVDS6, the filtered transmission readable at `lock_detect/adc1_filtered`, and an auxiliary TTL output on LVDS7 for the hold of the AOM lock |

For `fnc` and `current_sense`, a Pounder or current sense board must be attached (`current_sense`
also runs without, with the offset DACs having no effect); `l674` needs the EEM connector in its
GPIO population (no Urukul).

## Building and flashing

As upstream, with the current stable Rust toolchain:

```sh
rustup target add thumbv7em-none-eabihf
cargo build --release --bin current_sense          # or fnc, l674, dual-iir, ...
cargo run --release --bin current_sense            # flash and run with probe-rs, RTT log on stdout
```

Without a debug probe, make a raw image and flash it over USB DFU (see the upstream setup page for
putting the board into DFU mode):

```sh
cargo objcopy --release --bin current_sense -- -O binary current_sense.bin
dfu-util -a 0 -s 0x08000000:leave -R -D current_sense.bin
```

The MQTT broker (and, if needed, a static IP or a different MQTT ID) is set over the USB serial
port and stored in flash, as the firmware default is the host name `mqtt`; the lab broker is
`10.255.6.4`:

```sh
python -m serial /dev/tty.usbmodem...   # then in the menu:
set /net/broker "10.255.6.4"
store
platform reboot
```

The settings a device had under an earlier firmware stay retained on the broker under their old
topics, and the device logs an error for each when it connects; clear them with
`mosquitto_pub -r -n -t <topic>`.

## GUIs and tools

The graphical interfaces live in
[`OxfordIonTrapGroup/stabilizer-ui`](https://github.com/OxfordIonTrapGroup/stabilizer-ui): one
window per application (filters, device settings, a live scope of the stream, recording to HDF5,
transfer function measurements with the swept-sine source, long-term spectral density estimates),
a launcher which finds the devices on the broker, and for `l674` the lock monitor with automatic
relocking. The UI installs `py/stabilizer` from this repository, pinned to a commit, so a change to
the settings layout here goes together with a UI change there. Its `tools/` (on its
`testing-tools` branch) include a simulated device for each application for working on the MQTT
side without hardware.

Scripting from Python goes through the [`miniconf`](https://github.com/quartiq/miniconf) client,
e.g. `python -m miniconf -b 10.255.6.4 -d dt/sinara/current_sense/+ '/ch/0/gain="G2"'`.

## Hardware

[![Stabilizer](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)](https://github.com/sinara-hw/Stabilizer)

[![Pounder](https://user-images.githubusercontent.com/1338946/125936814-3664aa2d-a530-4c85-9393-999a7173424e.png)](https://github.com/sinara-hw/Pounder/wiki)
