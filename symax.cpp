#include "symax.h"
#include "common.h"

#define BIND_COUNT 345   // 1.5 seconds
#define FIRST_PACKET_DELAY  12000

#define PACKET_PERIOD        4000     // Timeout for callback in uSec

#define FLAG_FLIP      0x01
#define FLAG_VIDEO     0x02
#define FLAG_PICTURE   0x04
#define FLAG_HEADLESS  0x08

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10
};

#define PAYLOADSIZE 10       // receive data pipes set to this size, but unused

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)

// Bit vector from bit position
#define BV(bit) (1 << bit)

SymaX::SymaX(nRF24L01 * radio):
  nrf24l01(radio)
{
}

void SymaX::initialize()
{
    //CLOCK_StopTimer();
    //tx_power = RF_POWER;
    packet_counter = 0;
    flags = 0;

    init();
    phase = SYMAX_INIT1;

    //PROTOCOL_SetBindState(BIND_COUNT * PACKET_PERIOD / 1000);
    //CLOCK_StartTimer(INITIAL_WAIT, symax_callback);
    delayMicroseconds(500);
}

uint8_t SymaX::checksum(uint8_t *data)
{
    uint8_t sum = data[0];

    for (int i=1; i < packet_size-1; i++)
        if (current_protocol==PROTO_SYMAXOLD)
            sum += data[i];
        else
            sum ^= data[i];
    
    return sum + ((current_protocol==PROTO_SYMAXOLD) ? 0 : 0x55);
}

void SymaX::readControls(uint8_t* throttle, uint8_t* rudder, uint8_t* elevator, uint8_t* aileron, uint8_t* flags)
{
  if (ppm[CHANNEL1] < PPM_MID) {
    *aileron = map(ppm[CHANNEL1], PPM_MIN, PPM_MID, 0x00, 0x7F);
  } else {
    *aileron = map(ppm[CHANNEL1], PPM_MID, PPM_MAX, 0x80, 0x7FF);
  }

  if (ppm[CHANNEL2] < PPM_MID) {
    *elevator = map(ppm[CHANNEL2], PPM_MIN, PPM_MID, 0x00, 0x7F);
  } else {
    *elevator = map(ppm[CHANNEL2], PPM_MID, PPM_MAX, 0x80, 0xFF);
  }
  
  *throttle = map(ppm[CHANNEL3], PPM_MIN, PPM_MAX, 0x00, 0xFF);

  if (ppm[CHANNEL4] < PPM_MID) {
    *rudder = map(ppm[CHANNEL4], PPM_MIN, PPM_MID, 0x00, 0x7F);
  } else {
    *rudder = map(ppm[CHANNEL4], PPM_MID, PPM_MAX, 0x80, 0xFF);
  }

  //if (Channels[CHANNEL6] <= 0)
  //    *flags &= ~FLAG_FLIP;
  //else
  //    *flags |= FLAG_FLIP;

  //if (Channels[CHANNEL7] <= 0)
  //    *flags &= ~FLAG_PICTURE;
  //else
  //    *flags |= FLAG_PICTURE;

  //if (Channels[CHANNEL8] <= 0)
  //    *flags &= ~FLAG_VIDEO;
  //else
  //    *flags |= FLAG_VIDEO;

  //if (Channels[CHANNEL9] <= 0)
  //    *flags &= ~FLAG_HEADLESS;
  //else
  //    *flags |= FLAG_HEADLESS;
}

/**
 * Build packet for old SymaX protocol.
 */
void SymaX::buildPacketX5C(uint8_t bind)
{
  if (bind) {
    memset(packet, 0, packet_size);
    packet[7] = 0xae;
    packet[8] = 0xa9;
    packet[14] = 0xc0;
    packet[15] = 0x17;
  } else {
    readControls(&throttle, &rudder, &elevator, &aileron, &flags);

    packet[0] = throttle;
    packet[1] = rudder;
    packet[2] = elevator ^ 0x80;  // reversed from default
    packet[3] = aileron;
    packet[4] = X5C_CHAN2TRIM(rudder ^ 0x80);     // drive trims for extra control range
    packet[5] = X5C_CHAN2TRIM(elevator);
    packet[6] = X5C_CHAN2TRIM(aileron ^ 0x80);
    packet[7] = 0xae;
    packet[8] = 0xa9;
    packet[9] = 0x00;
    packet[10] = 0x00;
    packet[11] = 0x00;
    packet[12] = 0x00;
    packet[13] = 0x00;
    packet[14] = (flags & FLAG_VIDEO   ? 0x10 : 0x00) 
                 | (flags & FLAG_PICTURE ? 0x08 : 0x00)
                 | (flags & FLAG_FLIP    ? 0x01 : 0x00)
                 | 0x04;  // always high rates (bit 3 is rate control)
    packet[15] = checksum(packet);
  }
}

/**
 * Build packet for the new SymaX protocol.
 */
void SymaX::buildPacket(uint8_t bind) {
  if (bind) {
    packet[0] = rx_tx_addr[4];
    packet[1] = rx_tx_addr[3];
    packet[2] = rx_tx_addr[2];
    packet[3] = rx_tx_addr[1];
    packet[4] = rx_tx_addr[0];
    packet[5] = 0xaa;
    packet[6] = 0xaa;
    packet[7] = 0xaa;
    packet[8] = 0x00;
  } else {
    readControls(&throttle, &rudder, &elevator, &aileron, &flags);

    packet[0] = throttle;
    packet[1] = elevator;
    packet[2] = rudder;
    packet[3] = aileron;
    packet[4] = (flags & FLAG_VIDEO   ? 0x80 : 0x00) 
                | (flags & FLAG_PICTURE ? 0x40 : 0x00);
    // use trims to extend controls
    packet[5] = (elevator >> 2) | 0xc0;  // always high rates (bit 7 is rate control)
    packet[6] = (rudder >> 2)   | (flags & FLAG_FLIP  ? 0x40 : 0x00);
    packet[7] = (aileron >> 2)  | (flags & FLAG_HEADLESS ? 0x80 : 0x00);
    packet[8] = 0x00;
  }
  packet[9] = checksum(packet);
}

void SymaX::sendPacket(uint8_t bind)
{
    if (current_protocol==PROTO_SYMAXOLD)
        buildPacketX5C(bind);
    else
        buildPacket(bind);

#if defined(DEBUG)
    Serial.print(throttle);
    Serial.print("   ");
    Serial.print(rudder);
    Serial.print("   ");
    Serial.print(elevator);
    Serial.print("   ");
    Serial.println(aileron);
#endif
    
    // clear packet status bits and TX FIFO
    nrf24l01->writeReg(NRF24L01_07_STATUS, 0x70);
    nrf24l01->writeReg(NRF24L01_00_CONFIG, 0x2e);
    nrf24l01->writeReg(NRF24L01_05_RF_CH, chans[current_chan]);
    nrf24l01->flushTx();

    nrf24l01->writePayload(packet, packet_size);

    if (packet_counter++ % 2) {   // use each channel twice
        current_chan = (current_chan + 1) % num_rf_channels;
    }

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
    //if (tx_power != Model.tx_power) {
    //    //Keep transmit power updated
    //    tx_power = Model.tx_power;
    //    nrf24l01->setPower(tx_power);
    //}
}

// Generate address to use from TX id and manufacturer id (STM32 unique id)
void SymaX::initializeRxTxAddr()
{
    for(uint8_t i=0; i<4; i++)
        rx_tx_addr[i] = transmitterID[i];
    rx_tx_addr[4] = 0xa2; // this is constant in ID
}

// channels determined by last byte of tx address
void SymaX::setChannels(uint8_t address) {
  static const uint8_t start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
  static const uint8_t start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
  static const uint8_t start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};

  uint8_t laddress = address & 0x1f;
  uint8_t i;
  uint32_t *pchans = (uint32_t *)chans;   // avoid compiler warning

  num_rf_channels = 4;

  if (laddress < 0x10) {
    if (laddress == 6) laddress = 7;
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_1[i] + laddress;
    }
  } else if (laddress < 0x18) {
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_2[i] + (laddress & 0x07);
    }
    if (laddress == 0x16) {
      chans[0] += 1;
      chans[1] += 1;
    }
  } else if (laddress < 0x1e) {
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_3[i] + (laddress & 0x07);
    }
  } else if (laddress == 0x1e) {
      *pchans = 0x38184121;
  } else {
      *pchans = 0x39194121;
  }
}

void SymaX::init()
{
    const u8 bind_rx_tx_addr[] = {0xab,0xac,0xad,0xae,0xaf};
    const u8 rx_tx_addr_x5c[] = {0x6d,0x6a,0x73,0x73,0x73};   // X5C uses same address for bind and data

    nrf24l01->initialize();

    nrf24l01->setTxRxMode(TX_EN);

    nrf24l01->readReg(NRF24L01_07_STATUS);
    nrf24l01->writeReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO)); 
    nrf24l01->writeReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    nrf24l01->writeReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
    nrf24l01->writeReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    nrf24l01->writeReg(NRF24L01_04_SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
    nrf24l01->writeReg(NRF24L01_05_RF_CH, 0x08);

    if (current_protocol==PROTO_SYMAXOLD) {
      nrf24l01->setBitrate(NRF24L01_BR_1M);
      packet_size = 16;
    } else {
      nrf24l01->setBitrate(NRF24L01_BR_250K);
      packet_size = 10;
    }

    nrf24l01->setPower(RF_POWER);
    nrf24l01->writeReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    nrf24l01->writeReg(NRF24L01_08_OBSERVE_TX, 0x00);
    nrf24l01->writeReg(NRF24L01_09_CD, 0x00);
    nrf24l01->writeReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    nrf24l01->writeReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    nrf24l01->writeReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    nrf24l01->writeReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    nrf24l01->writeReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);   // bytes of data payload for pipe 1
    nrf24l01->writeReg(NRF24L01_12_RX_PW_P1, PAYLOADSIZE);
    nrf24l01->writeReg(NRF24L01_13_RX_PW_P2, PAYLOADSIZE);
    nrf24l01->writeReg(NRF24L01_14_RX_PW_P3, PAYLOADSIZE);
    nrf24l01->writeReg(NRF24L01_15_RX_PW_P4, PAYLOADSIZE);
    nrf24l01->writeReg(NRF24L01_16_RX_PW_P5, PAYLOADSIZE);
    nrf24l01->writeReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here

    nrf24l01->writeRegisterMulti(NRF24L01_10_TX_ADDR,
                                (current_protocol==PROTO_SYMAXOLD) ? rx_tx_addr_x5c : bind_rx_tx_addr,
                                5);

    nrf24l01->readReg(NRF24L01_07_STATUS);

    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    nrf24l01->activate(0x53); // magic for BK2421 bank switch
    //dbgprintf("Trying to switch banks\n");
    if (nrf24l01->readReg(NRF24L01_07_STATUS) & 0x80) {
        //dbgprintf("BK2421 detected\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        nrf24l01->writeRegisterMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4);
        nrf24l01->writeRegisterMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4);
        nrf24l01->writeRegisterMulti(0x03, (u8 *) "\x99\x00\x39\x21", 4);
        nrf24l01->writeRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
        nrf24l01->writeRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
        nrf24l01->writeRegisterMulti(0x06, (u8 *) "\x00\x00\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x07, (u8 *) "\x00\x00\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x08, (u8 *) "\x00\x00\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x09, (u8 *) "\x00\x00\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x0A, (u8 *) "\x00\x00\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x0B, (u8 *) "\x00\x00\x00\x00", 4);
        nrf24l01->writeRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
        nrf24l01->writeRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
        nrf24l01->writeRegisterMulti(0x0E, (u8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
        nrf24l01->writeRegisterMulti(0x04, (u8 *) "\xFF\x96\x82\x1B", 4);
        nrf24l01->writeRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4);
    } else {
        //dbgprintf("nRF24L01 detected\n");
    }
    nrf24l01->activate(0x53); // switch bank back

    nrf24l01->flushTx();
    nrf24l01->readReg(NRF24L01_07_STATUS);
    nrf24l01->writeReg(NRF24L01_07_STATUS, 0x0e);
    nrf24l01->readReg(NRF24L01_00_CONFIG); 
    nrf24l01->writeReg(NRF24L01_00_CONFIG, 0x0c); 
    nrf24l01->writeReg(NRF24L01_00_CONFIG, 0x0e);  // power on
}

void SymaX::init1()
{
    // duplicate stock tx sending strange packet (effect unknown)
    uint8_t first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00};
    uint8_t chans_bind[] = {0x4b, 0x30, 0x40, 0x20};
    uint8_t chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                           0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};

    nrf24l01->flushTx();
    nrf24l01->writeReg(NRF24L01_05_RF_CH, 0x08);
    nrf24l01->writePayload(first_packet, 15);

    if (current_protocol==PROTO_SYMAXOLD) {
      num_rf_channels = sizeof(chans_bind_x5c);
      memcpy(chans, chans_bind_x5c, num_rf_channels);
    } else {
      initializeRxTxAddr();   // make info available for bind packets
      num_rf_channels = sizeof(chans_bind);
      memcpy(chans, chans_bind, num_rf_channels);
    }
    current_chan = 0;
    packet_counter = 0;
}

void SymaX::init2()
{
    uint8_t chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                           0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};

    if (current_protocol==PROTO_SYMAXOLD) {
      num_rf_channels = sizeof(chans_data_x5c);
      memcpy(chans, chans_data_x5c, num_rf_channels);
    } else {
      setChannels(rx_tx_addr[0]);
      nrf24l01->writeRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
    }
    current_chan = 0;
    packet_counter = 0;
}

uint16_t SymaX::callback()
{
  switch (phase) {
    case SYMAX_INIT1:
        init1();
        phase = SYMAX_BIND2;
        return FIRST_PACKET_DELAY;
        break;

    case SYMAX_BIND2:
        counter = BIND_COUNT;
        phase = SYMAX_BIND3;
        sendPacket(1);
        break;

    case SYMAX_BIND3:
        if (counter == 0) {
            init2();
            phase = SYMAX_DATA;
            //PROTOCOL_SetBindState(0);
            //MUSIC_Play(MUSIC_DONE_BINDING);
        } else {
            sendPacket(1);
            counter -= 1;
        }
        break;

    case SYMAX_DATA:
        sendPacket(0);
        break;
  }
  return PACKET_PERIOD;
}
