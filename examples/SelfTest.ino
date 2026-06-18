// ============================================================================
//  mcp_jakka - SelfTest
//
//  Brings the MCP2515 up in LOOPBACK mode, so it needs NO CAN bus and NO
//  transceiver traffic: every frame you send is looped straight back to RX.
//  Great for confirming your wiring / CS pin / crystal before going on a bus.
//
//  Pins below are the Autosport Labs ESP32-CAN-X2 defaults - change to suit.
// ============================================================================
#include <SPI.h>
#include <mcp_jakka.h>

#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define PIN_CS    10

McpJakka can(SPI, PIN_CS);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // The caller owns SPI - set your board's pins here, then begin().
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // 500 kbps, 16 MHz crystal, loopback = true (no bus/transceiver required).
  if (!can.begin(500000, 16, /*loopback=*/true)) {
    Serial.println(F("MCP2515 not responding - check CS pin, wiring and crystal."));
    while (true) delay(1000);
  }
  Serial.println(F("mcp_jakka loopback self-test @ 500 kbps"));
}

uint32_t last = 0;
uint8_t  counter = 0;

void loop() {
  if (millis() - last >= 1000) {
    last = millis();
    McpCanFrame tx;
    tx.id  = 0x123;
    tx.len = 4;
    tx.data[0] = counter++;
    tx.data[1] = 0xDE; tx.data[2] = 0xAD; tx.data[3] = 0xBE;
    Serial.print(F("TX 0x"));  Serial.print(tx.id, HEX);
    Serial.println(can.send(tx) ? F(" ... queued") : F(" ... all TX buffers busy"));
  }

  McpCanFrame rx;
  while (can.receive(rx)) {
    Serial.print(F("RX 0x")); Serial.print(rx.id, HEX);
    Serial.print(F(" ["));    Serial.print(rx.len); Serial.print(F("]"));
    for (uint8_t i = 0; i < rx.len; i++) { Serial.print(' '); Serial.print(rx.data[i], HEX); }
    if (rx.rtr) Serial.print(F("  (RTR)"));
    Serial.println();
  }
}
