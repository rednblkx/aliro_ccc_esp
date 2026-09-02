# aliro_ccc_esp

ESP-IDF component implementing an **Aliro** UWB ranging responder on Qorvo **DW3000** / **DW3720** (untested) transceivers. Aliro is the lock access standard developed by the CSA (Connectivity Standards Alliance); this component implements the responder side of the Aliro UWB ranging protocol (CCC-style ranging, Ranging Setup M1–M4).

The component is self-contained and meant to be reused across projects: it exposes `aliro_uwb::UwbDw3000Channel`, an implementation of `ddk::aliro::UwbRangingChannel` — the UWB seam of the [`DigitalDoorKey`](https://github.com/rednblkx/DigitalDoorKey) digital-key component. A small amount of plumbing in the host app (frame routing between the BLE flow and this channel, arming at the URSK handoff, a range callback) is all that's needed to wire it in; see [Integration](#integration).

## How it fits together

```
host app (e.g. an Aliro-over-BLE flow such as DigitalDoorKey's BleFlow)
  │  owns UwbDw3000Channel, sets sender + range callbacks,
  │  calls aliro_uwb::init() at boot, arms it at the URSK handoff
  ▼
ddk_dw3000_channel.cpp  ── implements ddk::aliro::UwbRangingChannel
  │  ESP adapters (clock, logger, SPI, GPIO), M1–M4 handling,
  │  IRQ task
  ▼
upstream/  (vendored aliro_ccc_core, platform-agnostic C++20)
  │  RangingSession, DW3000Controller, setup codec,
  │  CCC key derivation + SP0 security, distance estimation
  ▼
Qorvo decadriver (FetchContent: br101/dw3000-decadriver-source)
  │
  ▼
deca_port.c  ── driver porting layer (mutex, delays)
```

Frame flow: BLE frames arrive BleSK-decrypted at `handle_frame()`; outgoing frames go through the `send` callback set via `set_sender()`. `poll()` is ticked from BLE receive slices and delivers each new measurement as `(distance_cm, age_ms)` through the callback set with `set_range_callback()`. Range-gated decision logic (distance/freshness thresholds, unlock) lives in the host app, not here.

## Layout

| Path | Contents |
|---|---|
| `ddk_dw3000_channel.cpp`, `include/ddk_dw3000_channel.h` | ESP glue: `aliro_uwb::UwbDw3000Channel`, hardware adapters, `aliro_uwb::init()` |
| `deca_port.c` | Decawave driver porting layer (critical sections, `vTaskDelay`/`ets_delay_us`) |
| `upstream/` | Vendored `aliro_ccc_core` engine (ranging, crypto, protocol codecs, session, DW3000 controller behind HAL interfaces) + its own host CMake project |
| `Kconfig.projbuild` | Pin assignments, SPI speed, chip selection |

## Configuration (Kconfig)

All `CONFIG_DW3000_*` options live in the `DW3000 driver` menu. Defaults are provided for ESP32-S3 and ESP32-C6; on other targets every pin defaults to `-1`, which fails a `static_assert` at build time until real pins are configured — the component does not guess.

- `DW3000_GPIO_IRQ` / `DW3000_GPIO_RESET` / `DW3000_GPIO_WAKEUP` — control pins
- `DW3000_SPI_MOSI` / `DW3000_SPI_MISO` / `DW3000_SPI_CLK` / `DW3000_SPI_CS` — SPI wiring (bus `SPI2_HOST`)
- `DW3000_SPI_MAX_MHZ` — fast-mode SPI clock (default 36 MHz; register access starts at 2 MHz and switches up)
- `DW3000_CHIP_DW3000` / `DW3000_CHIP_DW3720` — chip selection (DW3720 covers QM33xx)

## Integration

The host app must provide the plumbing around the channel:

1. Instantiate `aliro_uwb::UwbDw3000Channel` and register it with the BLE flow. With DigitalDoorKey's `ddk::BleFlow`: `flow.set_uwb_ranging_channel(&channel)`.
2. Set the out-of-band sender: `channel.set_sender(...)` — frames returned here must be BleSK-sealed and sent over the BLE link.
3. Call `aliro_uwb::init()` once at boot to set up SPI/GPIO and start the IRQ task. Hardware init is deferred-safe: `arm()` re-inits if needed.
4. Arm at the URSK handoff: `channel.arm(session_id, ursk)` (session id, 32-byte URSK).
5. Tick `channel.poll()` from BLE receive slices; consume `(distance_cm, age_ms)` in the range callback set via `channel.set_range_callback()`.
6. Call `channel.stop()` on flow teardown.

The component never touches BLE, Matter, or NVS itself — all connectivity is inverted through callbacks and the `UwbRangingChannel` interface.

## Building

Add as an ESP-IDF component (copy into `components/`, add as a submodule, or point `EXTRA_COMPONENT_DIRS` at it) and build the host project as usual, e.g.:

```sh
idf.py build
```

The first configure run needs network access: the decadriver is fetched via CMake FetchContent.

## Ranging flow

1. App arms the channel with the session id and 32-byte URSK at the BLE URSK handoff.
2. Phone sends a Ranging Notification (`INIT_RANGING`) → component replies with Setup M1 (capabilities: channels 5+9, sync codes 9–12, all CHAP configs, hopping disabled-preferred).
3. Phone sends Setup M2 → component parses it, builds and sends M3.
4. Phone sends Setup M4 → component starts the UWB ranging session; each measurement is reported via the range callback.

Hopping is not implemented: if the negotiated session uses hopping, a warning is logged and ranging will likely fail. Suspend requests are answered and suspend the session.

## Dependencies

`esp_timer`, `driver`, `freertos`, `esp_rom`, `mbedtls`, `DigitalDoorKey`. C++20.

## License & Legal

This project is licensed under MIT — see [LICENSE](LICENSE), excluding the linked DW3000 driver, for which it supplies it's own licenses, see repo [here](https://github.com/br101/dw3000-decadriver-source)

Aliro is a trademark of Connectivity Standards Alliance (CSA)
