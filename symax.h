#ifndef _SYMAX_H_
#define _SYMAX_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "nrf24l01.h"

#define SYMAX_MAX_PACKET_SIZE 16   // X11,X12,X5C-1 10-byte, X5C 16-byte

#define SYMAX_MAX_RF_CHANNELS    17

enum {
    SYMAX_INIT1 = 0,
    SYMAX_BIND2,
    SYMAX_BIND3,
    SYMAX_DATA
};

class SymaX
{
public:
  SymaX(nRF24L01 * radio);
  void initialize();
  uint16_t callback();
  
private:
  uint8_t checksum(uint8_t *data);
  void readControls(uint8_t* throttle, uint8_t* rudder, uint8_t* elevator, uint8_t* aileron, uint8_t* flags);
  void buildPacketX5C(uint8_t bind);
  void buildPacket(uint8_t bind);
  void sendPacket(uint8_t bind);
  void initializeRxTxAddr();
  void setChannels(uint8_t address);
  
  void init();
  void init1();
  void init2();
  
private:
  nRF24L01 * nrf24l01;

  uint8_t packet[SYMAX_MAX_PACKET_SIZE];
  uint8_t packet_size;
  uint16_t counter;
  uint32_t packet_counter;
  uint8_t throttle;
  uint8_t rudder;
  uint8_t elevator;
  uint8_t aileron;
  uint8_t flags;
  uint8_t rx_tx_addr[5];

  // frequency channel management
  uint8_t current_chan;
  uint8_t chans[SYMAX_MAX_RF_CHANNELS];
  uint8_t num_rf_channels;

  uint8_t phase;
};

#endif // _SYMAX_H_
