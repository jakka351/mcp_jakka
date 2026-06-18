// =============================================================================
//  mcp_jakka.h  -  minimal, robust MCP2515 driver for the TCM bus  (class McpJakka)
//
//  Named "mcp_jakka" on purpose so it never gets confused with the pile of
//  mcp_can / mcp_canbus / mcp2515 libraries floating around the Arduino world.
//
//  Written for this project to replace the Longan/Seeed mcp_can library, whose
//  RX path can overflow the caller's buffer on a bad DLC, whose begin() calls a
//  bare SPI.begin() that fights our custom ESP32-S3 pin map, and whose send()
//  blocks until the frame is on the wire. This driver:
//
//    * never calls SPI.begin() itself - the caller owns SPI + its pins;
//    * uses an explicit SPIClass instance and SPISettings;
//    * ALWAYS clamps a received DLC to 8 (no buffer overrun, ever);
//    * sends fire-and-forget into any of the 3 TX buffers (non-blocking);
//    * exposes error counters + bus-off state and can re-init on bus-off.
//
//  Self-contained: depends only on Arduino + SPI (no project headers), so it
//  can be dropped into any sketch or published as a standalone library.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>

// A single classic-CAN (CAN 2.0A/B) frame. The library owns this type so it has
// no external dependencies.
struct McpCanFrame {
  uint32_t id   = 0;        // 11-bit (standard) or 29-bit (extended) identifier
  uint8_t  len  = 0;        // data length, 0..8 (always clamped to 8 on receive)
  uint8_t  data[8] = {0};
  bool     ext  = false;    // true = extended 29-bit id
  bool     rtr  = false;    // true = remote-transmission-request frame
};

class McpJakka {
public:
  McpJakka(SPIClass &spi, uint8_t csPin, uint32_t spiHz = 10000000UL);

  // Initialise to the given bitrate (bps) for the given crystal (MHz). The
  // caller must have called spi.begin(sck,miso,mosi,cs) beforehand. Set
  // loopback=true for a self-test that needs no bus or transceiver.
  // Returns false if the chip does not respond or the bitrate is unsupported.
  bool begin(uint32_t bitrate, uint8_t crystalMHz = 16, bool loopback = false);
  bool reinit();                 // re-run begin() with the last parameters (recovery)

  // RX - non-blocking. Returns true and fills f (DLC clamped to <=8) when a
  // frame was read.
  bool available();
  bool receive(McpCanFrame &f);

  // TX - fire-and-forget into the first free TX buffer. Returns false (sends
  // nothing) if all three TX buffers are busy; the caller may retry later.
  bool send(const McpCanFrame &f);

  // Diagnostics
  uint8_t errorFlags();          // EFLG register
  uint8_t txErrorCount();        // TEC
  uint8_t rxErrorCount();        // REC
  bool    busOff();              // EFLG.TXBO
  uint8_t opMode();              // CANSTAT op-mode bits (0x00 normal, 0x80 config, ...)

private:
  SPIClass   &_spi;
  uint8_t     _cs;
  uint32_t    _hz;
  SPISettings _set;
  uint32_t    _bitrate  = 500000;
  uint8_t     _xtal     = 16;
  bool        _loopback = false;

  inline void select()   { digitalWrite(_cs, LOW); }
  inline void deselect() { digitalWrite(_cs, HIGH); }

  void    reset();
  uint8_t readReg(uint8_t addr);
  void    readRegs(uint8_t addr, uint8_t *buf, uint8_t n);
  void    writeReg(uint8_t addr, uint8_t val);
  void    modifyReg(uint8_t addr, uint8_t mask, uint8_t val);
  uint8_t readStatus();
  bool    setMode(uint8_t mode);
  bool    configRate(uint32_t bitrate, uint8_t xtal);
};
