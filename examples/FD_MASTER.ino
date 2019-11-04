#include <FlexCAN_T4.h>
#include <TeensyCAN.h>

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> FD;

TeensyCAN node36 = TeensyCAN(36);

void setup(void) {
  Serial.begin(115200); delay(400);
  pinMode(6, OUTPUT); digitalWrite(6, LOW);
  FD.begin();
  Node.setID(100);
  Node.setBus(_CAN3);

  Node.onReceive(cb);
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = 1000000;
  config.baudrateFD = 2000000;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 87.5;
  FD.setRegions(64);
  FD.setBaudRate(config);
  FD.onReceive(canSniff);
  FD.enableMBInterrupts();
  FD.mailboxStatus();
}

void cb(const uint8_t* buffer, uint16_t length, AsyncTC info) {
  Serial.print("Node: ");
  Serial.print(info.node);
  Serial.print("\tPacketID: ");
  Serial.print(info.packetid);
  Serial.print("\tBroadcast: ");
  Serial.print(info.broadcast);
  Serial.print("\tData: ");
  for ( uint8_t i = 0; i < length; i++ ) {
    ::Serial.print(buffer[i]);
    ::Serial.print(" ");
  }::Serial.println();
}


void canSniff(const CANFD_message_t &msg) {
  Serial.print("ISR - MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print("  EDL: "); Serial.print(msg.edl );
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}





void loop() {
  FD.events();
  Node.events();
}

void canSniff20(const CAN_message_t &msg) { // global callback
  Serial.print("T4: ");
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print(" OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print(" BUS "); Serial.print(msg.bus);
  Serial.print(" LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(msg.flags.remote);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" IDHIT: "); Serial.print(msg.idhit);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
