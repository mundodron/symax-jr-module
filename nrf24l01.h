/**
 * Based on
 * https://github.com/DeviationTX/deviation/blob/master/src/common.h
 */
#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Register map
enum {
    NRF24L01_00_CONFIG      = 0x00,
    NRF24L01_01_EN_AA       = 0x01,
    NRF24L01_02_EN_RXADDR   = 0x02,
    NRF24L01_03_SETUP_AW    = 0x03,
    NRF24L01_04_SETUP_RETR  = 0x04,
    NRF24L01_05_RF_CH       = 0x05,
    NRF24L01_06_RF_SETUP    = 0x06,
    NRF24L01_07_STATUS      = 0x07,
    NRF24L01_08_OBSERVE_TX  = 0x08,
    NRF24L01_09_CD          = 0x09,
    NRF24L01_0A_RX_ADDR_P0  = 0x0A,
    NRF24L01_0B_RX_ADDR_P1  = 0x0B,
    NRF24L01_0C_RX_ADDR_P2  = 0x0C,
    NRF24L01_0D_RX_ADDR_P3  = 0x0D,
    NRF24L01_0E_RX_ADDR_P4  = 0x0E,
    NRF24L01_0F_RX_ADDR_P5  = 0x0F,
    NRF24L01_10_TX_ADDR     = 0x10,
    NRF24L01_11_RX_PW_P0    = 0x11,
    NRF24L01_12_RX_PW_P1    = 0x12,
    NRF24L01_13_RX_PW_P2    = 0x13,
    NRF24L01_14_RX_PW_P3    = 0x14,
    NRF24L01_15_RX_PW_P4    = 0x15,
    NRF24L01_16_RX_PW_P5    = 0x16,
    NRF24L01_17_FIFO_STATUS = 0x17,
    NRF24L01_1C_DYNPD       = 0x1C,
    NRF24L01_1D_FEATURE     = 0x1D,
    //Instructions
    NRF24L01_61_RX_PAYLOAD  = 0x61,
    NRF24L01_A0_TX_PAYLOAD  = 0xA0,
    NRF24L01_E1_FLUSH_TX    = 0xE1,
    NRF24L01_E2_FLUSH_RX    = 0xE2,
    NRF24L01_E3_REUSE_TX_PL = 0xE3,
    NRF24L01_50_ACTIVATE    = 0x50,
    NRF24L01_60_R_RX_PL_WID = 0x60,
    NRF24L01_B0_TX_PYLD_NOACK = 0xB0,
    NRF24L01_FF_NOP         = 0xFF,
    NRF24L01_A8_W_ACK_PAYLOAD0 = 0xA8,
    NRF24L01_A8_W_ACK_PAYLOAD1 = 0xA9,
    NRF24L01_A8_W_ACK_PAYLOAD2 = 0xAA,
    NRF24L01_A8_W_ACK_PAYLOAD3 = 0xAB,
    NRF24L01_A8_W_ACK_PAYLOAD4 = 0xAC,
    NRF24L01_A8_W_ACK_PAYLOAD5 = 0xAD,
};

// Bit mnemonics
enum {
    NRF24L01_00_MASK_RX_DR  = 6,
    NRF24L01_00_MASK_TX_DS  = 5,
    NRF24L01_00_MASK_MAX_RT = 4,
    NRF24L01_00_EN_CRC      = 3,
    NRF24L01_00_CRCO        = 2,
    NRF24L01_00_PWR_UP      = 1,
    NRF24L01_00_PRIM_RX     = 0,

    NRF24L01_07_RX_DR       = 6,
    NRF24L01_07_TX_DS       = 5,
    NRF24L01_07_MAX_RT      = 4,

    NRF2401_1D_EN_DYN_ACK   = 0,
    NRF2401_1D_EN_ACK_PAY   = 1,
    NRF2401_1D_EN_DPL       = 2,
};

// Bitrates
enum {
    NRF24L01_BR_1M = 0,
    NRF24L01_BR_2M,
    NRF24L01_BR_250K,
    NRF24L01_BR_RSVD
};

enum TXRX_State {
  TXRX_OFF,
  TX_EN,
  RX_EN,
};

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { 
  RF24_PA_MIN = 0,    // -18dBm /  7.0mA
  RF24_PA_LOW,        // -12dBm /  7.5mA
  RF24_PA_HIGH,       //  -6dBm /  9.0mA
  RF24_PA_MAX,        //   0dBm / 11.3mA
  RF24_PA_ERROR 
} rf24_pa_dbm_e ;

class nRF24L01
{
public:
  /**
   * Arduino Constructor
   *
   * Creates a new instance of this driver.  Before using, you create an instance
   * and send in the unique pins that this chip is connected to.
   *
   * @param _cepin The pin attached to Chip Enable on the RF module
   * @param _cspin The pin attached to Chip Select
   */
  nRF24L01(uint8_t _cepin, uint8_t _cspin);

  void begin();
  
  void CS_HI();
  void CS_LO();
  void initialize();
  uint8_t writeReg(uint8_t reg, uint8_t data);
  uint8_t writeRegisterMulti(uint8_t reg, const uint8_t data[], uint8_t length);
  uint8_t writePayload(uint8_t *data, uint8_t length);
  uint8_t readReg(uint8_t reg);
  uint8_t readRegisterMulti(uint8_t reg, uint8_t data[], uint8_t length);
  uint8_t readPayload(uint8_t *data, uint8_t length);
  uint8_t strobe(uint8_t state);
  uint8_t flushTx();
  uint8_t flushRx();
  uint8_t activate(uint8_t code);
  uint8_t setBitrate(uint8_t bitrate);
  uint8_t setPower(uint8_t power);
  void CE_lo();
  void CE_hi();
  void setTxRxMode(enum TXRX_State mode);
  int reset();
  
private:
  uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint8_t cs_pin; /**< SPI Chip select */
  uint8_t rf_setup;
};

#endif // _NRF24L01_H_
