# mcp_jakka

**A robust, dependency-free MCP2515 CAN driver for Arduino / ESP32.**

A clean-room MCP2515 driver written because the ageing Seeed/Longan `mcp_can`
libraries — the ones everyone copies — have real defects that bite hard on a
busy or noisy CAN bus. `mcp_jakka` is small, self-contained (Arduino + SPI
only), and designed to behave on a real vehicle bus, not just a happy bench.

The name is deliberately weird so it never gets confused with the dozen other
`mcp_can` / `mcp_canbus` / `mcp2515` libraries floating around.

```cpp
#include <SPI.h>
#include <mcp_jakka.h>

McpJakka can(SPI, /*CS=*/10);

void setup() {
  SPI.begin(12, 13, 11, 10);        // YOU own SPI: sck, miso, mosi, cs
  can.begin(500000, 16);            // 500 kbps, 16 MHz crystal
}

void loop() {
  McpCanFrame rx;
  while (can.receive(rx)) {         // non-blocking, DLC always <= 8
    // ... use rx.id / rx.len / rx.data / rx.ext / rx.rtr
  }
}
```

---

## Why another MCP2515 library?

The popular Seeed-derived `mcp_can` is fine on an UNO blinking an LED. On a real
bus it has problems this library fixes:

| Issue in the old library | What `mcp_jakka` does |
|---|---|
| `readMsgBuf()` masks the DLC to 0–15 and copies that many bytes into your buffer → **buffer overrun on a malformed/oversized DLC** | **Every received DLC is hard-clamped to 8.** A bad frame can never corrupt memory. |
| `begin()` calls a bare `SPI.begin()` with **no pins**, clobbering custom SPI wiring (e.g. ESP32-S3) | The driver **never touches `SPI.begin()`** — you pass a `SPIClass&` and set the pins yourself. |
| `sendMsg()` **busy-waits until the frame is on the wire**, serialising every transmit and stalling on errors | **Fire-and-forget**: loads any free TX buffer and returns immediately. The three hardware TX buffers act as a small queue. |
| Reports init success even when the **bitrate is invalid** | `begin()` returns `false` on an unsupported bitrate or a chip that doesn't answer (presence check after reset). |
| No bus-off handling | `busOff()` + `reinit()` for one-call recovery; plus `errorFlags()` / `txErrorCount()` / `rxErrorCount()`. |
| `setMsg()` contains `rtr = rtr;` (assigns the parameter to itself) | Rewritten; RTR is handled correctly on both TX and RX. |

No global `SPI` assumptions, no hidden `delay()`s in the hot path, no external
dependencies.

---

## Features

- Standard (11-bit) and extended (29-bit) identifiers, data and remote frames.
- Non-blocking `receive()` / `send()` — perfect for gateways and tight loops.
- Caller-owned `SPIClass` instance + configurable SPI clock (use `HSPI`/`FSPI`,
  share a bus, or drop the clock for marginal wiring).
- Loopback mode for a self-test with **no bus or transceiver attached**.
- Bus-off detection and recovery; error-counter accessors.
- Receives everything by default (no filter setup needed) — ideal for sniffers
  and man-in-the-middle bridges.

---

## Supported hardware

- Any MCP2515 + transceiver (MCP2551/TJA1050/SN65HVD230/etc.).
- Tested on the **Autosport Labs ESP32-CAN-X2** (ESP32-S3 + on-board MCP2515,
  16 MHz crystal, CS=10, MOSI=11, SCK=12, MISO=13).
- **Crystal:** 16 MHz bit-timing tables are built in (the common case). 8/20 MHz
  aren't included yet — `begin()` returns `false` for them. PRs welcome.
- **Bitrates (16 MHz):** 100k, 125k, 250k, 500k, 1000k.

> Classic CAN 2.0A/B only — this is an MCP2515; it is **not** CAN-FD.

---

## Installation

**Arduino IDE / Library Manager:** *Sketch ▸ Include Library ▸ Add .ZIP Library…*
and select this folder zipped, or clone into your `libraries/` directory:

```
git clone https://github.com/jakka351/mcp_jakka.git
```

**PlatformIO:** add to `platformio.ini`:

```ini
lib_deps = https://github.com/jakka351/mcp_jakka.git
```

---

## Wiring

| MCP2515 | MCU (example: ESP32-CAN-X2) |
|---|---|
| SCK  | GPIO 12 |
| SI (MOSI) | GPIO 11 |
| SO (MISO) | GPIO 13 |
| CS   | GPIO 10 |
| INT  | GPIO 3 *(optional — this driver polls)* |
| VCC/GND | 3V3 / GND |

The MCP2515's CANH/CANL go through a transceiver to the bus. Terminate the bus
with 120 Ω at each physical end.

---

## API

### Construct

```cpp
McpJakka(SPIClass &spi, uint8_t csPin, uint32_t spiHz = 10000000);
```
`spiHz` defaults to the MCP2515 maximum (10 MHz). Drop to 8/4 MHz if your wiring
is long or marginal.

### Initialise

```cpp
bool begin(uint32_t bitrate, uint8_t crystalMHz = 16, bool loopback = false);
bool reinit();   // re-run begin() with the last parameters (recovery)
```
Call `SPI.begin(sck, miso, mosi, cs)` **before** `begin()`. Returns `false` if
the chip doesn't respond or the bitrate/crystal is unsupported.

### Receive (non-blocking)

```cpp
bool available();              // is a frame waiting?
bool receive(McpCanFrame &f);  // true + fills f if a frame was read (DLC clamped to 8)
```

### Transmit (fire-and-forget)

```cpp
bool send(const McpCanFrame &f);   // false if all 3 TX buffers are busy (retry later)
```

### Diagnostics

```cpp
uint8_t errorFlags();     // EFLG register
uint8_t txErrorCount();   // TEC
uint8_t rxErrorCount();   // REC
bool    busOff();         // true if the controller is bus-off
uint8_t opMode();         // CANSTAT op-mode bits (0x00 normal, 0x40 loopback, 0x80 config)
```

### Frame type

```cpp
struct McpCanFrame {
  uint32_t id   = 0;        // 11-bit or 29-bit identifier
  uint8_t  len  = 0;        // 0..8
  uint8_t  data[8] = {0};
  bool     ext  = false;    // extended 29-bit id
  bool     rtr  = false;    // remote-transmission-request frame
};
```

---

## Example

See [`examples/SelfTest`](examples/SelfTest/SelfTest.ino) — a loopback test that
sends a frame every second and prints it as it loops back, with no bus required.

---

## Design notes

- **You own SPI.** The driver only ever does `beginTransaction`/`transfer`/
  `endTransaction` on the `SPIClass` you give it. It never calls `SPI.begin()`,
  so it can't fight your pin mapping or another device on the bus.
- **Fire-and-forget TX.** `send()` writes to the first free TX buffer via the
  `LOAD TX BUFFER` SPI command and issues `RTS` — it does not wait for the frame
  to be acknowledged or transmitted. Three buffers give you a little headroom;
  if all are busy you get `false` and decide whether to retry or drop.
- **RX can't overrun you.** Frames are read with the `READ RX BUFFER` command
  (which auto-clears the interrupt flag on CS-rising), and the DLC is clamped to
  8 before any copy.
- **Bounded waits.** Mode transitions poll `CANSTAT` with a finite timeout —
  there are no unbounded busy-loops anywhere.

---

## Limitations

- Classic CAN only (no CAN-FD).
- 16 MHz crystal bit-timing tables only (for now).
- Polled, not interrupt-driven (the INT pin is broken out if you want to gate
  your polling on it). For most loops, polling `receive()` is plenty fast.

---

## License

MIT — Copyright (c) 2026 **Tester Present Specialist Automotive Solutions**.
See [LICENSE](LICENSE). The driver is original work; the bit-timing register
constants are factual values from the Microchip MCP2515 datasheet.

## Credits

Created by **Tester Present Specialist Automotive Solutions** (jakka351). The
register-level driver was designed and reviewed during the FG Falcon TCM
man-in-the-middle project.
