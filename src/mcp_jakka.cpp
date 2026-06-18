// =============================================================================
//  mcp_jakka.cpp  -  implementation of the McpJakka MCP2515 driver
// =============================================================================
#include "mcp_jakka.h"

// ---- SPI instruction set ----------------------------------------------------
#define CMD_RESET        0xC0
#define CMD_READ         0x03
#define CMD_WRITE        0x02
#define CMD_BITMOD       0x05
#define CMD_READ_STATUS  0xA0
#define CMD_LOAD_TX0     0x40   // load starting at TXB0SIDH (0x42=TXB1, 0x44=TXB2)
#define CMD_RTS_TX0      0x81   // request-to-send TXB0 (0x82=TXB1, 0x84=TXB2)
#define CMD_READ_RX0     0x90   // read starting at RXB0SIDH (0x94=RXB1); auto-clears RXnIF

// ---- registers --------------------------------------------------------------
#define REG_CANSTAT      0x0E
#define REG_CANCTRL      0x0F
#define REG_TEC          0x1C
#define REG_REC          0x1D
#define REG_CNF3         0x28
#define REG_CNF2         0x29
#define REG_CNF1         0x2A
#define REG_CANINTE      0x2B
#define REG_CANINTF      0x2C
#define REG_EFLG         0x2D
#define REG_TXB0CTRL     0x30
#define REG_RXB0CTRL     0x60
#define REG_RXB1CTRL     0x70

// ---- bit masks --------------------------------------------------------------
#define MODE_NORMAL      0x00
#define MODE_LOOPBACK    0x40
#define MODE_CONFIG      0x80
#define MODE_MASK        0xE0
#define RXM_RECEIVE_ANY  0x60   // RXBnCTRL.RXM = 11 -> accept everything (MITM sniffer)
#define RXB_BUKT         0x04   // RXB0CTRL rollover to RXB1
#define TXREQ            0x08   // TXBnCTRL.TXREQ
#define INTE_RX          0x03   // RX0IE | RX1IE
#define STAT_RX0IF       0x01
#define STAT_RX1IF       0x02
#define STAT_TX0REQ      0x04
#define STAT_TX1REQ      0x10
#define STAT_TX2REQ      0x40
#define EFLG_TXBO        0x20
#define RXB_IDE          0x08   // RXBnSIDL.IDE (extended frame)
#define DLC_RTR          0x40   // RXBnDLC.RTR

McpJakka::McpJakka(SPIClass &spi, uint8_t csPin, uint32_t spiHz)
  : _spi(spi), _cs(csPin), _hz(spiHz), _set(spiHz, MSBFIRST, SPI_MODE0) {}

// ---- low level SPI ----------------------------------------------------------
void McpJakka::reset() {
  _spi.beginTransaction(_set);
  select(); _spi.transfer(CMD_RESET); deselect();
  _spi.endTransaction();
  delay(10);                              // allow the device to enter config mode
}

uint8_t McpJakka::readReg(uint8_t addr) {
  _spi.beginTransaction(_set);
  select();
  _spi.transfer(CMD_READ); _spi.transfer(addr);
  uint8_t v = _spi.transfer(0x00);
  deselect();
  _spi.endTransaction();
  return v;
}

void McpJakka::readRegs(uint8_t addr, uint8_t *buf, uint8_t n) {
  _spi.beginTransaction(_set);
  select();
  _spi.transfer(CMD_READ); _spi.transfer(addr);
  for (uint8_t i = 0; i < n; i++) buf[i] = _spi.transfer(0x00);
  deselect();
  _spi.endTransaction();
}

void McpJakka::writeReg(uint8_t addr, uint8_t val) {
  _spi.beginTransaction(_set);
  select();
  _spi.transfer(CMD_WRITE); _spi.transfer(addr); _spi.transfer(val);
  deselect();
  _spi.endTransaction();
}

void McpJakka::modifyReg(uint8_t addr, uint8_t mask, uint8_t val) {
  _spi.beginTransaction(_set);
  select();
  _spi.transfer(CMD_BITMOD); _spi.transfer(addr); _spi.transfer(mask); _spi.transfer(val);
  deselect();
  _spi.endTransaction();
}

uint8_t McpJakka::readStatus() {
  _spi.beginTransaction(_set);
  select();
  _spi.transfer(CMD_READ_STATUS);
  uint8_t v = _spi.transfer(0x00);
  deselect();
  _spi.endTransaction();
  return v;
}

// ---- mode + bit timing ------------------------------------------------------
bool McpJakka::setMode(uint8_t mode) {
  modifyReg(REG_CANCTRL, MODE_MASK, mode);
  for (int i = 0; i < 20; i++) {                 // bounded wait (~20 ms), never infinite
    if ((readReg(REG_CANSTAT) & MODE_MASK) == mode) return true;
    delay(1);
  }
  return false;
}

bool McpJakka::configRate(uint32_t bitrate, uint8_t xtal) {
  // Field-proven CNF1/CNF2/CNF3 values for a 16 MHz crystal.
  uint8_t cnf1, cnf2, cnf3;
  if (xtal != 16) return false;                  // only 16 MHz is fitted on this board
  switch (bitrate) {
    case 1000000: cnf1 = 0x00; cnf2 = 0xD0; cnf3 = 0x82; break;
    case 500000:  cnf1 = 0x00; cnf2 = 0xF0; cnf3 = 0x86; break;
    case 250000:  cnf1 = 0x41; cnf2 = 0xF1; cnf3 = 0x85; break;
    case 125000:  cnf1 = 0x03; cnf2 = 0xF0; cnf3 = 0x86; break;
    case 100000:  cnf1 = 0x03; cnf2 = 0xFA; cnf3 = 0x87; break;
    default:      return false;
  }
  writeReg(REG_CNF1, cnf1);
  writeReg(REG_CNF2, cnf2);
  writeReg(REG_CNF3, cnf3);
  return true;
}

bool McpJakka::begin(uint32_t bitrate, uint8_t crystalMHz, bool loopback) {
  _bitrate  = bitrate;
  _xtal     = crystalMHz;
  _loopback = loopback;

  pinMode(_cs, OUTPUT);
  deselect();

  reset();
  // Presence check: after RESET the device must be in CONFIG mode.
  if ((readReg(REG_CANSTAT) & MODE_MASK) != MODE_CONFIG) return false;
  if (!setMode(MODE_CONFIG)) return false;
  if (!configRate(bitrate, crystalMHz)) return false;

  // Receive everything (RXM = "receive any"), RXB0 rolls over into RXB1.
  writeReg(REG_RXB0CTRL, RXM_RECEIVE_ANY | RXB_BUKT);
  writeReg(REG_RXB1CTRL, RXM_RECEIVE_ANY);

  // Enable RX interrupt flags (we poll them; the INT pin is also available).
  writeReg(REG_CANINTE, INTE_RX);
  writeReg(REG_CANINTF, 0x00);

  return setMode(_loopback ? MODE_LOOPBACK : MODE_NORMAL);
}

bool McpJakka::reinit() { return begin(_bitrate, _xtal, _loopback); }

// ---- RX ---------------------------------------------------------------------
bool McpJakka::available() {
  return (readStatus() & (STAT_RX0IF | STAT_RX1IF)) != 0;
}

bool McpJakka::receive(McpCanFrame &f) {
  uint8_t st = readStatus();
  uint8_t cmd;
  if      (st & STAT_RX0IF) cmd = CMD_READ_RX0;        // 0x90
  else if (st & STAT_RX1IF) cmd = CMD_READ_RX0 | 0x04; // 0x94
  else return false;

  uint8_t hdr[5], data[8];
  _spi.beginTransaction(_set);
  select();
  _spi.transfer(cmd);
  for (uint8_t i = 0; i < 5; i++) hdr[i]  = _spi.transfer(0x00);  // SIDH,SIDL,EID8,EID0,DLC
  for (uint8_t i = 0; i < 8; i++) data[i] = _spi.transfer(0x00);  // always read 8 (in-bounds)
  deselect();
  _spi.endTransaction();
  // Raising CS after a READ RX BUFFER command auto-clears the RXnIF flag.

  uint8_t sidh = hdr[0], sidl = hdr[1], eid8 = hdr[2], eid0 = hdr[3], dlcreg = hdr[4];

  if (sidl & RXB_IDE) {                          // extended 29-bit id
    uint32_t id = ((uint32_t)sidh << 3) | (sidl >> 5);
    id = (id << 2) | (sidl & 0x03);
    id = (id << 8) | eid8;
    id = (id << 8) | eid0;
    f.id  = id & 0x1FFFFFFF;
    f.ext = true;
    f.rtr = (dlcreg & DLC_RTR) != 0;             // RXBnDLC.RTR (extended remote)
  } else {                                       // standard 11-bit id
    f.id  = ((uint32_t)sidh << 3) | (sidl >> 5);
    f.ext = false;
    f.rtr = (sidl & 0x10) != 0;                  // RXBnSIDL.SRR (standard remote)
  }

  uint8_t dlc = dlcreg & 0x0F;
  if (dlc > 8) dlc = 8;                           // <<< hard clamp - never overrun
  f.len = dlc;
  memcpy(f.data, data, dlc);
  return true;
}

// ---- TX (fire-and-forget) ---------------------------------------------------
bool McpJakka::send(const McpCanFrame &f) {
  // LOAD command = 0x40/0x42/0x44 (SIDH of TXB0/1/2); RTS = 0x81/0x82/0x84.
  // Note the RTS opcode is 0x80 | (1<<buf) - it does NOT share the LOAD offset.
  uint8_t st = readStatus();
  uint8_t loadCmd, rtsCmd;
  if      (!(st & STAT_TX0REQ)) { loadCmd = 0x40; rtsCmd = 0x81; }
  else if (!(st & STAT_TX1REQ)) { loadCmd = 0x42; rtsCmd = 0x82; }
  else if (!(st & STAT_TX2REQ)) { loadCmd = 0x44; rtsCmd = 0x84; }
  else return false;                              // all 3 buffers busy - caller retries

  uint8_t len = f.len > 8 ? 8 : f.len;
  uint8_t sidh, sidl, eid8, eid0;
  if (f.ext) {
    uint32_t id = f.id & 0x1FFFFFFF;
    sidh = (uint8_t)(id >> 21);
    sidl = (uint8_t)(((id >> 18) & 0x07) << 5) | 0x08 | (uint8_t)((id >> 16) & 0x03);
    eid8 = (uint8_t)(id >> 8);
    eid0 = (uint8_t)(id);
  } else {
    uint16_t id = f.id & 0x7FF;
    sidh = (uint8_t)(id >> 3);
    sidl = (uint8_t)((id & 0x07) << 5);
    eid8 = 0;
    eid0 = 0;
  }
  uint8_t dlc = (len & 0x0F) | (f.rtr ? 0x40 : 0x00);  // TXBnDLC.RTR for remote frames

  _spi.beginTransaction(_set);
  select();
  _spi.transfer(loadCmd);                         // load starting at TXBnSIDH
  _spi.transfer(sidh); _spi.transfer(sidl);
  _spi.transfer(eid8); _spi.transfer(eid0);
  _spi.transfer(dlc);
  for (uint8_t i = 0; i < len; i++) _spi.transfer(f.data[i]);
  deselect();
  _spi.endTransaction();

  _spi.beginTransaction(_set);
  select();
  _spi.transfer(rtsCmd);                          // request-to-send this buffer
  deselect();
  _spi.endTransaction();
  return true;
}

// ---- diagnostics ------------------------------------------------------------
uint8_t McpJakka::errorFlags()   { return readReg(REG_EFLG); }
uint8_t McpJakka::txErrorCount() { return readReg(REG_TEC); }
uint8_t McpJakka::rxErrorCount() { return readReg(REG_REC); }
bool    McpJakka::busOff()       { return (readReg(REG_EFLG) & EFLG_TXBO) != 0; }
uint8_t McpJakka::opMode()       { return readReg(REG_CANSTAT) & MODE_MASK; }
