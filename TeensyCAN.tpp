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
TeensyCAN Node;
volatile int TeensyCAN::payload_ack_check = 0x15;



TeensyCAN::TeensyCAN(uint8_t node) {
  Serial.featuredNode = node;
}



void TeensyCAN::events() {
  if ( TeensyCAN::completed_frames.size() ) {
    uint8_t buffer[MAX_PAYLOAD_SIZE] = { 0 };
    TeensyCAN::completed_frames.pop_front(buffer, MAX_PAYLOAD_SIZE);
    AsyncTC info;
    info.node = buffer[1];
    info.packetid = buffer[6];
    info.broadcast = !buffer[0];
    if ( TeensyCAN::_handler ) TeensyCAN::_handler(buffer+7,((uint16_t)(buffer[2] << 8) | buffer[3]),info);
  }
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
    delay(delay_send);
  }
  if ( Serial.featuredNode ) { /* only wait for node specific ACKs, no responses for global payloads */
    uint32_t _timeout = millis();
    while ( payload_ack_check != 0x06 ) {
      if ( millis() - _timeout > timeout ) break; 
      if ( payload_ack_check == 0x15 ) break;
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
        if ( Node._flexcanPtr->isFD() ) Node._flexcanPtr->write(frameFD);
        else Node._flexcanPtr->write(frameCAN);
      }
    }
  }

  if ( (((id & 0x3C000) >> 14) == 4) ) {
    TeensyCAN::payload_ack_check = array[2];
  }

  if ( ((id & 0x3F80) >> 7) != TeensyCAN::nodeID ) return; /* pick up frames only meant for this node */

}

void ext_outputFD3(const CANFD_message_t &msg) { /* FD frame, send to processing which handles CAN2.0 and CANFD */
  frame_processing(msg.id, msg.buf, msg.len);
}

void ext_output3(const CAN_message_t &msg) { /* CAN2.0 frame, send to processing which handles CAN2.0 and CANFD */
  frame_processing(msg.id, msg.buf, msg.len);
}