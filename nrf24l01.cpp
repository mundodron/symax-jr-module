#include "nrf24l01.h"

#include <SPI.h>

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
//#define NOP           0xFF  // already defined as NRF24L01_FF_NOP in iface_nrf24l01.h

#define PROTOSPI_xfer(byte) SPI.transfer(byte);

nRF24L01::nRF24L01(uint8_t _cepin, uint8_t _cspin):
  ce_pin(_cepin), cs_pin(_cspin)
{
}

void nRF24L01::begin()
{
  SPI.begin();
}

void nRF24L01::CS_HI() {
    //PROTO_CS_HI(NRF24L01);
}

void nRF24L01::CS_LO() {
    //PROTO_CS_LO(NRF24L01);
}
void nRF24L01::initialize()
{
    rf_setup = 0x0F;
    //XN297_SetScrambledMode(XN297_SCRAMBLED);
}    

uint8_t nRF24L01::writeReg(uint8_t reg, uint8_t data)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(W_REGISTER | (REGISTER_MASK & reg));
    PROTOSPI_xfer(data);
    CS_HI();
    return res;
}

uint8_t nRF24L01::writeRegisterMulti(uint8_t reg, const uint8_t data[], uint8_t length)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++)
    {
        PROTOSPI_xfer(data[i]);
    }
    CS_HI();
    return res;
}

uint8_t nRF24L01::writePayload(uint8_t *data, uint8_t length)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++)
    {
        PROTOSPI_xfer(data[i]);
    }
    CS_HI();
    return res;
}

uint8_t nRF24L01::readReg(uint8_t reg)
{
    CS_LO();
    PROTOSPI_xfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t data = PROTOSPI_xfer(0xFF);
    CS_HI();
    return data;
}

uint8_t nRF24L01::readRegisterMulti(uint8_t reg, uint8_t data[], uint8_t length)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(R_REGISTER | (REGISTER_MASK & reg));
    for(uint8_t i = 0; i < length; i++)
    {
        data[i] = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

uint8_t nRF24L01::readPayload(uint8_t *data, uint8_t length)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(R_RX_PAYLOAD);
    for(uint8_t i = 0; i < length; i++)
    {
        data[i] = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

uint8_t nRF24L01::strobe(uint8_t state)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(state);
    CS_HI();
    return res;
}

uint8_t nRF24L01::flushTx()
{
    return strobe(FLUSH_TX);
}

uint8_t nRF24L01::flushRx()
{
    return strobe(FLUSH_RX);
}

uint8_t nRF24L01::activate(uint8_t code)
{
    CS_LO();
    uint8_t res = PROTOSPI_xfer(ACTIVATE);
    PROTOSPI_xfer(code);
    CS_HI();
    return res;
}

uint8_t nRF24L01::setBitrate(uint8_t bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return writeReg(NRF24L01_06_RF_SETUP, rf_setup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm. 
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
uint8_t nRF24L01::setPower(uint8_t power)
{
    uint8_t nrf_power = power;
    if(power > 3) {
        nrf_power = 3;
    }
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((nrf_power & 0x03) << 1);
    return writeReg(NRF24L01_06_RF_SETUP, rf_setup);
}
void nRF24L01::CE_lo()
{
#if HAS_MULTIMOD_SUPPORT
    PROTOCOL_SetSwitch(NRF24L01);
#endif
}
void nRF24L01::CE_hi()
{
#if HAS_MULTIMOD_SUPPORT
    uint8_t en = SPI_ProtoGetPinConfig(NRF24L01, ENABLED_PIN);
    uint8_t csn = SPI_ProtoGetPinConfig(NRF24L01, CSN_PIN);
    SPI_ConfigSwitch(en | 0x0f, en | (0x0f ^ csn));
#endif
}

void nRF24L01::setTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
        CE_lo();
        writeReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        writeReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        delayMicroseconds(130);
        CE_hi();
    } else if (mode == RX_EN) {
        CE_lo();
        writeReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        writeReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        writeReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        writeReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        delayMicroseconds(130);
        CE_hi();
    } else {
        writeReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
        CE_lo();
    }
}

int nRF24L01::reset()
{
    flushTx();
    flushRx();
    uint8_t status1 = strobe(NRF24L01_FF_NOP);
    uint8_t status2 = readReg(NRF24L01_07_STATUS);
    setTxRxMode(TXRX_OFF);
#ifdef EMULATOR
    return 1;
#endif
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

