#include <TeensyCAN.h>
#include "Arduino.h"
#include <atomic>
#include <util/atomic.h>
#include "Stream.h"
#include "circular_buffer.h"
#include <FlexCAN_T4.h>

Circular_Buffer<uint8_t, (uint32_t)pow(2, ceil(log(MAX_NODE_RECEIVING) / log(2))), MAX_PAYLOAD_SIZE> TeensyCAN::storage;
Circular_Buffer<uint8_t, 8, MAX_PAYLOAD_SIZE> TeensyCAN::completed_frames;
uint8_t TeensyCAN::nodeID = 127;
uint32_t TeensyCAN::nodeNet = 0x1FFC0000;
FlexCAN_T4_Base* TeensyCAN::_flexcanPtr = nullptr;
_tC_ptr TeensyCAN::_handler = nullptr;
TeensyCAN Node = TeensyCAN(0);
volatile int TeensyCAN::payload_ack_check = 0x15;

TeensyCAN::TeensyCAN(uint8_t node) {
  Serial.featuredNode = node;
  Serial.serial_access |= 0UL;
  Serial.port = 0;
  Serial1.featuredNode = node;
  Serial1.serial_access |= 1UL;
  Serial1.port = 1;
  Serial2.featuredNode = node;
  Serial2.serial_access |= 2UL;
  Serial2.port = 2;
  Serial3.featuredNode = node;
  Serial3.serial_access |= 3UL;
  Serial3.port = 3;
  Serial4.featuredNode = node;
  Serial4.serial_access |= 4UL;
  Serial4.port = 4;
  Serial5.featuredNode = node;
  Serial5.serial_access |= 5UL;
  Serial5.port = 5;
  Serial6.featuredNode = node;
  Serial6.serial_access |= 6UL;
  Serial6.port = 6;

  Wire.featuredNode = node;
  Wire.wire_access = 0x80 | 0UL;
  Wire.port = 0;
  Wire1.featuredNode = node;
  Wire1.wire_access = 0x80 | 1UL;
  Wire1.port = 1;
  Wire2.featuredNode = node;
  Wire2.wire_access = 0x80 | 2UL;
  Wire2.port = 2;
  Wire3.featuredNode = node;
  Wire3.wire_access = 0x80 | 3UL;
  Wire3.port = 3;

  SPI.featuredNode = node;
  SPI.spi_access = 0x80 | 0UL;
  SPI.port = 0;
  SPI1.featuredNode = node;
  SPI1.spi_access = 0x80 | 1UL;
  SPI1.port = 1;
  SPI2.featuredNode = node;
  SPI2.spi_access = 0x80 | 2UL;
  SPI2.port = 2;
}



void TeensyCAN::events() {
  if ( TeensyCAN::completed_frames.size() ) {
    uint8_t buffer[MAX_PAYLOAD_SIZE] = { 0 };
    TeensyCAN::completed_frames.pop_front(buffer, MAX_PAYLOAD_SIZE);
    AsyncTC info;
    if ( buffer[1] < 128 ) {
      info.node = buffer[1];
      info.packetid = buffer[6];
      info.broadcast = !buffer[0];
      if ( TeensyCAN::_handler ) TeensyCAN::_handler(buffer+7,((uint16_t)(buffer[2] << 8) | buffer[3]),info);
    }
    else { /* commands/responses */
      info.node = buffer[1] - 127;
      info.packetid = ((uint16_t)(buffer[6] << 8) | buffer[7]);
      info.broadcast = !buffer[0];
      if ( info.packetid == 0 ) { /* Serial write(buf,size), write(b), println, print */
        uint16_t result = 0;
        if ( buffer[8] == 0x80 ) result = ::Serial.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        else if ( buffer[8] == 0x81 ) result = ::Serial1.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        else if ( buffer[8] == 0x82 ) result = ::Serial2.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        else if ( buffer[8] == 0x83 ) result = ::Serial3.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        else if ( buffer[8] == 0x84 ) result = ::Serial4.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        else if ( buffer[8] == 0x85 ) result = ::Serial5.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        else if ( buffer[8] == 0x86 ) result = ::Serial6.write(buffer+9,((uint16_t)(buffer[2] << 8) | buffer[3]));
        uint8_t respond[] = { (uint8_t)(result >> 8), (uint8_t)result };
        if ( buffer[0] ) Node.Serial.sendControl(respond,sizeof(respond),buffer[8],1,info.node);
      }
      else if ( info.packetid == 2 ) { /* Serial available() */
        uint16_t result = 0;
        if ( buffer[8] == 0x80 ) result = ::Serial.available();
        else if ( buffer[8] == 0x81 ) result = ::Serial1.available();
        else if ( buffer[8] == 0x82 ) result = ::Serial2.available();
        else if ( buffer[8] == 0x83 ) result = ::Serial3.available();
        else if ( buffer[8] == 0x84 ) result = ::Serial4.available();
        else if ( buffer[8] == 0x85 ) result = ::Serial5.available();
        else if ( buffer[8] == 0x86 ) result = ::Serial6.available();
        uint8_t respond[] = { (uint8_t)(result >> 8), (uint8_t)result };
        if ( buffer[0] ) Node.Serial.sendControl(respond,sizeof(respond),buffer[8],3,info.node);
      }
      else if ( info.packetid == 4 ) { /* Serial peek() */
        uint16_t result = 0;
        if ( buffer[8] == 0x80 ) result = ::Serial.peek();
        else if ( buffer[8] == 0x81 ) result = ::Serial1.peek();
        else if ( buffer[8] == 0x82 ) result = ::Serial2.peek();
        else if ( buffer[8] == 0x83 ) result = ::Serial3.peek();
        else if ( buffer[8] == 0x84 ) result = ::Serial4.peek();
        else if ( buffer[8] == 0x85 ) result = ::Serial5.peek();
        else if ( buffer[8] == 0x86 ) result = ::Serial6.peek();
        uint8_t respond[] = { (uint8_t)(result >> 8), (uint8_t)result };
        if ( buffer[0] ) Node.Serial.sendControl(respond,sizeof(respond),buffer[8],5,info.node);
      }
      else if ( info.packetid == 6 ) { /* Serial read(buf,size) */
        uint16_t result = 0;
        char array[((uint16_t)(buffer[9] << 8) | buffer[10]) + 2] = { 0 };
        if ( buffer[8] == 0x80 ) result = ::Serial.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        else if ( buffer[8] == 0x81 ) result = ::Serial1.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        else if ( buffer[8] == 0x82 ) result = ::Serial2.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        else if ( buffer[8] == 0x83 ) result = ::Serial3.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        else if ( buffer[8] == 0x84 ) result = ::Serial4.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        else if ( buffer[8] == 0x85 ) result = ::Serial5.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        else if ( buffer[8] == 0x86 ) result = ::Serial6.readBytes(array+2,((uint16_t)(buffer[9] << 8) | buffer[10]));
        array[0] = (uint8_t)(result >> 8);
        array[1] = (uint8_t)(result);
        if ( buffer[0] ) Node.Serial.sendControl((const uint8_t*)array,sizeof(array),buffer[8],7,info.node);
      }
      else if ( info.packetid == 8 ) { /* Serial read() */
        uint16_t result = 0;
        if ( buffer[8] == 0x80 ) result = ::Serial.read();
        else if ( buffer[8] == 0x81 ) result = ::Serial1.read();
        else if ( buffer[8] == 0x82 ) result = ::Serial2.read();
        else if ( buffer[8] == 0x83 ) result = ::Serial3.read();
        else if ( buffer[8] == 0x84 ) result = ::Serial4.read();
        else if ( buffer[8] == 0x85 ) result = ::Serial5.read();
        else if ( buffer[8] == 0x86 ) result = ::Serial6.read();
        uint8_t respond[] = { (uint8_t)(result >> 8), (uint8_t)result };
        if ( buffer[0] ) Node.Serial.sendControl(respond,sizeof(respond),buffer[8],9,info.node);
      }


    }
  }
}

void TeensyCAN::NodeFeatures::sendControl(const uint8_t *array, uint16_t len, uint8_t port, uint16_t packetid, uint8_t toNode, uint8_t delay_send, uint32_t timeout) {
  delay_send = constrain(delay_send, 1U, 50U);
  len += 7;
  uint8_t buffer[len];
  memmove(&buffer[7], &array[0], len - 7);
  buffer[0] = (len - 7) >> 8;
  buffer[1] = (len - 7);
  uint16_t checksum = 0xBEEF;
  for ( uint32_t i = 7; i < len; i++ ) checksum ^= buffer[i];
  buffer[2] = checksum >> 8;
  buffer[3] = checksum;
  buffer[4] = packetid >> 8;
  buffer[5] = packetid;
  buffer[6] = port;

  uint8_t mbx_size = _flexcanPtr->getFirstTxBoxSize();
  uint16_t chunks = (int)ceil((float)len / (mbx_size - 2));
  uint8_t frames[chunks][mbx_size];

  for ( uint16_t i = 0; i < chunks; i++ ) {
    if ( i < chunks - 1 ) {
      memmove(&frames[i][2], &buffer[i * (mbx_size - 2)], (mbx_size - 2));
      frames[i][0] = i >> 8;
      frames[i][1] = i;
      continue;
    }
    for ( uint8_t k = (len - (i * (mbx_size - 2))) + 2; k < mbx_size; k++ ) frames[i][k] = 0xAA;
    memmove(&frames[i][2], &buffer[i * (mbx_size - 2)], (len - (i * (mbx_size - 2))));
    frames[i][0] = i >> 8;
    frames[i][1] = i;
  }

  CANFD_message_t frameFD;
  CAN_message_t frameCAN;

  frameFD.flags.extended = frameFD.seq = frameFD.edl = frameFD.brs = frameCAN.flags.extended = frameCAN.seq = 1;
  frameFD.len = mbx_size;
  frameCAN.len = 8;

  for ( uint16_t j = 0; j < chunks; j++ ) {
    delay(delay_send);
    if ( !j && chunks != 1 ) frameFD.id = frameCAN.id = nodeNet | toNode << 7 | nodeID | 5UL << 14;
    else if ( j == ( chunks - 1 ) ) frameFD.id = frameCAN.id = nodeNet | toNode << 7 | nodeID | 7UL << 14;
    else if ( j ) frameFD.id = frameCAN.id = nodeNet | toNode << 7 | nodeID | 6UL << 14;
    if ( _flexcanPtr->isFD() ) {
      memmove(&frameFD.buf[0], &frames[j][0], mbx_size);
      _flexcanPtr->write(frameFD);
    }
    else {
      memmove(&frameCAN.buf[0], &frames[j][0], mbx_size);
      _flexcanPtr->write(frameCAN);
    }
  }
}

int TeensyCAN::NodeFeatures::available() {
  if ( serial_access ) {
    uint8_t array[2] = { 0 };
    sendControl(array, 2, serial_access, 2, featuredNode);
    if ( !featuredNode ) return 0; /* no responses for global calls */
    uint8_t response[MAX_PAYLOAD_SIZE] = { 0 };
    response[1] = (uint8_t)((featuredNode) + 127);
    response[6] = 0;
    response[7] = 3;
    uint32_t timeout = millis();
    while (!TeensyCAN::completed_frames.find(response, MAX_PAYLOAD_SIZE, 1, 6, 7)) if ( millis() - timeout > 500 ) return 0xFF;
    return ((uint16_t)(response[9] << 8) | response[10]);
  }
  return 0;
}

int TeensyCAN::NodeFeatures::peek() {
  if ( serial_access ) {
    uint8_t array[2] = { 0 };
    sendControl(array, 2, serial_access, 4, featuredNode);
    if ( !featuredNode ) return 0; /* no responses for global calls */
    uint8_t response[MAX_PAYLOAD_SIZE] = { 0 };
    response[1] = (uint8_t)((featuredNode) + 127);
    response[6] = 0;
    response[7] = 5;
    uint32_t timeout = millis();
    while (!TeensyCAN::completed_frames.find(response, MAX_PAYLOAD_SIZE, 1, 6, 7)) if ( millis() - timeout > 500 ) return 0xFF;
    return ((uint16_t)(response[9] << 8) | response[10]);
  }
  return 0;
}

int TeensyCAN::NodeFeatures::read() {
  if ( serial_access ) {
    uint8_t array[2] = { 0 };
    sendControl(array, 2, serial_access, 8, featuredNode);
    if ( !featuredNode ) return 0; /* no responses for global calls */
    uint8_t response[MAX_PAYLOAD_SIZE] = { 0 };
    response[1] = (uint8_t)((featuredNode) + 127);
    response[6] = 0;
    response[7] = 9;
    uint32_t timeout = millis();
    while (!TeensyCAN::completed_frames.find(response, MAX_PAYLOAD_SIZE, 1, 6, 7)) if ( millis() - timeout > 500 ) return 0xFF;
    return ((uint16_t)(response[9] << 8) | response[10]);
  }
  return 0;
}

size_t TeensyCAN::NodeFeatures::readBytes(uint8_t* buf, size_t size) {
  if ( serial_access ) {
    uint8_t array[2] = { (uint8_t)(size >> 8), (uint8_t)size };
    sendControl(array, 2, serial_access, 6, featuredNode);
    if ( !featuredNode ) return 0; /* no responses for global calls */
    uint8_t response[MAX_PAYLOAD_SIZE] = { 0 };
    response[1] = (uint8_t)((featuredNode) + 127);
    response[6] = 0;
    response[7] = 7;
    uint32_t timeout = millis();
    while (!TeensyCAN::completed_frames.find(response, MAX_PAYLOAD_SIZE, 1, 6, 7)) if ( millis() - timeout > 1500 ) return 0xFF;
    memmove(&buf[0], &response[11], ((uint16_t)(response[9] << 8) | response[10]));
    return ((uint16_t)(response[9] << 8) | response[10]);
  }
  return 0;
}

size_t TeensyCAN::NodeFeatures::write(const uint8_t *buf, size_t size) {
  if ( serial_access ) {
    sendControl(buf, size, serial_access, 0, featuredNode);
    if ( !featuredNode ) return 0; /* no responses for global calls */
    uint8_t response[MAX_PAYLOAD_SIZE] = { 0 };
    response[1] = (uint8_t)((featuredNode) + 127);
    response[6] = 0;
    response[7] = 1;
    uint32_t timeout = millis();
    while (!TeensyCAN::completed_frames.find(response, MAX_PAYLOAD_SIZE, 1, 6, 7)) if ( millis() - timeout > 500 ) return 0xFF;
    return ((uint16_t)(response[9] << 8) | response[10]);
  }
  return 0;
}

size_t TeensyCAN::NodeFeatures::println(const char *p) {
  char _text[strlen(p) + 1];
  memmove(&_text[0],&p[0],strlen(p));
  _text[sizeof(_text) - 1] = '\n';
  return write((const uint8_t*)_text, sizeof(_text));
}

uint8_t TeensyCAN::sendMsg(const uint8_t *array, uint16_t len, uint8_t packetid, uint8_t delay_send, uint32_t timeout) {
  delay_send = constrain(delay_send, 1U, 50U);
  payload_ack_check = 0;
  len += 5;
  uint8_t buffer[len];
  memmove(&buffer[5], &array[0], len - 5);
  buffer[0] = (len - 5) >> 8;
  buffer[1] = (len - 5);
  uint16_t checksum = 0xBEEF;
  for ( uint32_t i = 5; i < len; i++ ) checksum ^= buffer[i];
  buffer[2] = checksum >> 8;
  buffer[3] = checksum;
  buffer[4] = packetid;

  uint8_t mbx_size = _flexcanPtr->getFirstTxBoxSize();
  uint16_t chunks = (int)ceil((float)len / (mbx_size - 2));
  uint8_t frames[chunks][mbx_size];

  for ( uint16_t i = 0; i < chunks; i++ ) {
    if ( i < chunks - 1 ) {
      memmove(&frames[i][2], &buffer[i * (mbx_size - 2)], (mbx_size - 2));
      frames[i][0] = i >> 8;
      frames[i][1] = i;
      continue;
    }
    for ( uint8_t k = (len - (i * (mbx_size - 2))) + 2; k < mbx_size; k++ ) frames[i][k] = 0xAA;
    memmove(&frames[i][2], &buffer[i * (mbx_size - 2)], (len - (i * (mbx_size - 2))));
    frames[i][0] = i >> 8;
    frames[i][1] = i;
  }

  CANFD_message_t frameFD;
  CAN_message_t frameCAN;

  frameFD.flags.extended = frameFD.seq = frameFD.edl = frameFD.brs = frameCAN.flags.extended = frameCAN.seq = 1;
  frameFD.len = mbx_size;
  frameCAN.len = 8;

  for ( uint16_t j = 0; j < chunks; j++ ) {
    delay(delay_send);
    if ( !j && chunks != 1 ) frameFD.id = frameCAN.id = nodeNet | Serial.featuredNode << 7 | nodeID | 1UL << 14;
    else if ( j == ( chunks - 1 ) ) frameFD.id = frameCAN.id = nodeNet | Serial.featuredNode << 7 | nodeID | 3UL << 14;
    else if ( j ) frameFD.id = frameCAN.id = nodeNet | Serial.featuredNode << 7 | nodeID | 2UL << 14;
    if ( _flexcanPtr->isFD() ) {
      memmove(&frameFD.buf[0], &frames[j][0], mbx_size);
      _flexcanPtr->write(frameFD);
    }
    else {
      memmove(&frameCAN.buf[0], &frames[j][0], mbx_size);
      _flexcanPtr->write(frameCAN);
    }
  }
  if ( Serial.featuredNode ) { /* only wait for node specific ACKs, no responses for global payloads */
    uint32_t _timeout = millis();
    while ( payload_ack_check != 0x06 ) {
      if ( millis() - _timeout > timeout ) break; 
      if ( payload_ack_check == 0x15 ) break; /* crc failed */
      if ( payload_ack_check == 0x24 ) break; /* can't support bigger payloads on reception */
    }
    return (( TeensyCAN::payload_ack_check ) ? TeensyCAN::payload_ack_check : 0xFF); /* timeout == 0xFF */
  }
  return 0x1; /* Global payload, No Response */
}

void frame_processing(uint32_t id, const uint8_t *array, uint8_t len) { /* handle CAN2.0 and CANFD data */
  if ( (id & 0x1FFC0000) != TeensyCAN::nodeNet ) return; /* other frame IDs not for local nodes */
  if (((id & 0x3F80) >> 7) != TeensyCAN::nodeID && ((id & 0x3F80) >> 7)) return; /* other nodes blocked, accept only global or specific traffic */

  if ( ((id & 0x3C000) >> 14) < 4 ) { /* callback payload assembly code, local and global */
    uint8_t buffer[MAX_PAYLOAD_SIZE] = { 0, (uint8_t)(id & 0x7F) };
    if ( TeensyCAN::storage.find(buffer, MAX_PAYLOAD_SIZE, 1, 1, 1) ) {
      uint16_t sequence = ((uint16_t)(array[0] << 8) | array[1]);
      memmove(&buffer[0] + 2 + ((sequence) * (len - 2)), &array[0] + 2, (len - 2));
      if ( ((id & 0x3C000) >> 14) < 3 ) TeensyCAN::storage.replace(buffer, MAX_PAYLOAD_SIZE, 1, 1, 1);
      else if ( ((id & 0x3C000) >> 14) == 3 ) {
        TeensyCAN::storage.findRemove(buffer, MAX_PAYLOAD_SIZE, 1, 1, 1);
        uint16_t checksum = 0xBEEF;
        for ( uint16_t i = 7; i < ((uint16_t)(buffer[2] << 8) | buffer[3]) + 7; i++ ) checksum ^= buffer[i];
        CANFD_message_t frameFD;
        CAN_message_t frameCAN;
        frameFD.flags.extended = frameFD.seq = frameFD.edl = frameFD.brs = frameCAN.flags.extended = frameCAN.seq = 1;
        frameFD.len = frameCAN.len = 8;
        frameFD.id = frameCAN.id = (TeensyCAN::nodeNet | (uint32_t)buffer[1] << 7 | TeensyCAN::nodeID | (4UL << 14));
        frameFD.buf[2] = frameCAN.buf[2] = 0x15;
        if ( checksum == ((uint16_t)(buffer[4] << 8) | buffer[5]) ) {
          TeensyCAN::completed_frames.push_back(buffer, MAX_PAYLOAD_SIZE);
          frameFD.buf[2] = frameCAN.buf[2] = 0x06;
        }
        if ( buffer[0] ) { /* if not a broadcast, reply */
          if ( Node._flexcanPtr->getFirstTxBoxSize() < len ) frameFD.buf[2] = frameCAN.buf[2] = 0x24;
          if ( Node._flexcanPtr->isFD() ) Node._flexcanPtr->write(frameFD);
          else Node._flexcanPtr->write(frameCAN);
        }
      }
    }
    else if ( ((id & 0x3C000) >> 14) == 1 ) {
      memmove(&buffer[0], &array[0], len);
      buffer[0] = (id & 0x3F80) >> 7; /* this node, or broadcast? */
      buffer[1] = id & 0x7F; /* remote node */
      TeensyCAN::storage.push_back(buffer, MAX_PAYLOAD_SIZE);
    }
    else if ( ((id & 0x3C000) >> 14) == 3 ) { /* for new and complete payload in a single frame */
      uint16_t checksum = 0xBEEF;
      for ( uint16_t i = 7; i < ((uint16_t)(array[2] << 8) | array[3]) + 7; i++ ) checksum ^= array[i];
      CANFD_message_t frameFD;
      CAN_message_t frameCAN;
      frameFD.flags.extended = frameFD.seq = frameFD.edl = frameFD.brs = frameCAN.flags.extended = frameCAN.seq = 1;
      frameFD.len = frameCAN.len = 8;
      frameFD.id = frameCAN.id = (TeensyCAN::nodeNet | (uint32_t)buffer[1] << 7 | TeensyCAN::nodeID | (4UL << 14));
      frameFD.buf[2] = frameCAN.buf[2] = 0x15;
      if ( checksum == ((uint16_t)(array[4] << 8) | array[5]) ) {
        memmove(&buffer[0], &array[0], MAX_PAYLOAD_SIZE);
        buffer[0] = (id & 0x3F80) >> 7; /* this node, or broadcast? */
        buffer[1] = id & 0x7F; /* remote node */
        TeensyCAN::completed_frames.push_back(buffer, MAX_PAYLOAD_SIZE);
        frameFD.buf[2] = frameCAN.buf[2] = 0x06;
      }
      if ( buffer[0] ) { /* if not a broadcast, reply */
        if ( Node._flexcanPtr->getFirstTxBoxSize() < len ) frameFD.buf[2] = frameCAN.buf[2] = 0x24;
        if ( Node._flexcanPtr->isFD() ) Node._flexcanPtr->write(frameFD);
        else Node._flexcanPtr->write(frameCAN);
      }
    }
    return;
  }

  else if ( (((id & 0x3C000) >> 14) == 4) ) {
    TeensyCAN::payload_ack_check = array[2];
    return;
  }



  else if ( (((id & 0x3C000) >> 14) < 8) ) {
    uint8_t buffer[MAX_PAYLOAD_SIZE] = { 0, (uint8_t)((id & 0x7F) + 127) };
    if ( TeensyCAN::storage.find(buffer, MAX_PAYLOAD_SIZE, 1, 1, 1) ) {
      uint16_t sequence = ((uint16_t)(array[0] << 8) | array[1]);
      memmove(&buffer[0] + 2 + ((sequence) * (len - 2)), &array[0] + 2, (len - 2));
      if ( ((id & 0x3C000) >> 14) < 7 ) TeensyCAN::storage.replace(buffer, MAX_PAYLOAD_SIZE, 1, 1, 1);
      else if ( ((id & 0x3C000) >> 14) == 7 ) {
        TeensyCAN::storage.findRemove(buffer, MAX_PAYLOAD_SIZE, 1, 1, 1);
        uint16_t checksum = 0xBEEF;
        for ( uint16_t i = 9; i < ((uint16_t)(buffer[2] << 8) | buffer[3]) + 9; i++ ) checksum ^= buffer[i];
        if ( checksum == ((uint16_t)(buffer[4] << 8) | buffer[5]) ) TeensyCAN::completed_frames.push_back(buffer, MAX_PAYLOAD_SIZE);
      }
    }
    else if ( ((id & 0x3C000) >> 14) == 5 ) {
      memmove(&buffer[0], &array[0], len);
      buffer[0] = (id & 0x3F80) >> 7; /* this node, or broadcast? */
      buffer[1] = (id & 0x7F) + 127; /* remote node, offset scan */
      TeensyCAN::storage.push_back(buffer, MAX_PAYLOAD_SIZE);
    }
    else if ( ((id & 0x3C000) >> 14) == 7 ) { /* complete single frame payload (fd mode) */
      uint16_t checksum = 0xBEEF;
      for ( uint16_t i = 9; i < ((uint16_t)(array[2] << 8) | array[3]) + 9; i++ ) checksum ^= array[i];
      if ( checksum == ((uint16_t)(array[4] << 8) | array[5]) ) {
        memmove(&buffer[0], &array[0], MAX_PAYLOAD_SIZE);
        buffer[0] = (id & 0x3F80) >> 7; /* this node, or broadcast? */
        buffer[1] = (id & 0x7F) + 127; /* remote node, offset scan */
        TeensyCAN::completed_frames.push_back(buffer, MAX_PAYLOAD_SIZE);
      }
    }
    return;
  }




  if ( ((id & 0x3F80) >> 7) != TeensyCAN::nodeID ) return; /* pick up frames only meant for this node */

}

void ext_outputFD3(const CANFD_message_t &msg) { /* FD frame, send to processing which handles CAN2.0 and CANFD */
  frame_processing(msg.id, msg.buf, msg.len);
}

void ext_output3(const CAN_message_t &msg) { /* CAN2.0 frame, send to processing which handles CAN2.0 and CANFD */
  frame_processing(msg.id, msg.buf, msg.len);
}
