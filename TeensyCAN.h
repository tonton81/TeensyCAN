#if !defined(_TEENSYCAN_H_)
#define _TEENSYCAN_H_
#include "Arduino.h"
#include "circular_buffer.h"

#define MAX_PAYLOAD_SIZE 200
#define MAX_NODE_RECEIVING 8
#define NODE_UPTIME_LIMIT 3000
#define NODE_KEEPALIVE 500

struct AsyncTC {
  uint8_t node = 0;
  uint8_t packetid = 0;
  bool broadcast = 0;
};

typedef void (*_tC_ptr)(const uint8_t* buffer, uint16_t length, AsyncTC info);
typedef void (*_tC_detectptr)(AsyncTC info);

class TeensyCAN {
  public:
    TeensyCAN(uint8_t node);
    static void events();
    void setBus(FlexCAN_T4_Base *flexcanptr) { _flexcanPtr = flexcanptr; }
    void setID(uint8_t id) { nodeID = constrain(id, 1, 127); }
    void setNet(uint32_t net = 0x1FFC0000) { nodeNet = constrain(net, 0x40000UL, 0x1FFC0000UL); }
    uint8_t sendMsg(const uint8_t *array, uint16_t len, uint8_t packetid = 0, uint8_t delay_send = 0, uint32_t timeout = 500);
    static Circular_Buffer<uint8_t, (uint32_t)pow(2, ceil(log(MAX_NODE_RECEIVING) / log(2))), MAX_PAYLOAD_SIZE> storage;
    static Circular_Buffer<uint8_t, 8, MAX_PAYLOAD_SIZE> completed_frames;
    static uint32_t nodeNet;
    static uint8_t nodeID;
    static void onReceive(_tC_ptr handler) { TeensyCAN::_handler = handler; }
    static _tC_ptr _handler;
    TeensyCAN() { Serial.featuredNode = 0;}
    static FlexCAN_T4_Base* _flexcanPtr;

    class NodeFeatures : public Stream {
      public:
        NodeFeatures(){;}
        virtual size_t print(const char *p) {return 0;}
        virtual size_t println(const char *p){return 0;}
        virtual size_t write(uint8_t val) {return 0;}
        virtual size_t write(const char *buf, size_t size) {return 0;}
        virtual size_t write(const uint8_t *buf, size_t size){return 0;}
        virtual int available(){return 0;}
        virtual int peek(){return 0;}
        virtual int read(){return 0;}
        virtual size_t read(uint8_t* buf, size_t size){return 0;}
        virtual void flush() {return;}

      private:
        uint8_t featuredNode;
        uint8_t serial_access;
        uint8_t spi_access;
        uint8_t wire_access;
        uint8_t port; // serial/wire/spi port
        friend TeensyCAN;
    };
    NodeFeatures Serial;




  static volatile int payload_ack_check;

  private:
};

extern TeensyCAN Node;

#include "TeensyCAN.tpp"

#endif
