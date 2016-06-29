#include <SPI.h>
#include <util/atomic.h>
#include <EEPROM.h>
#include "nrf24l01.h"
#include "symax.h"
#include "common.h"

uint8_t current_protocol;

uint8_t transmitterID[4];

static bool reset = true;

// EEPROM locations
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

// PPM stream settings
#define PPM_pin   3  // PPM in
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally

volatile uint16_t Servo_data[12];
uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};
static volatile bool ppm_ok = false;
                           
nRF24L01 radio(9, 10);
SymaX symax(&radio);

void setup() {
  #if defined(DEBUG)
  Serial.begin(115200);
  #endif
  
  // put your setup code here, to run once:
  radio.begin();  

  // PPM ISR setup
  attachInterrupt(digitalPinToInterrupt(PPM_pin), read_ppm, CHANGE);
  TCCR1A = 0;  //reset timer1
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

  set_txid(false);
}

void loop() {
  uint32_t timeout;
  
  // reset/rebind
  if (reset) {
    reset_or_rebind();
  }
  
  // process protocol
  switch(current_protocol) {
    case PROTO_SYMAX5C1:
    case PROTO_SYMAXOLD:
      timeout = symax.callback();
      break;
  }

  // updates ppm values out of ISR
  update_ppm();
    
  // wait before sending next packet
  while(micros() < timeout)
  {   };
}

//------------------------------------------------------------

// reset or rebind
void reset_or_rebind()
{
  reset = false;
  select_protocol();
  radio.reset();
  radio.initialize();
  init_protocol();
}

//------------------------------------------------------------

void select_protocol()
{
  current_protocol = PROTO_SYMAX5C1;

  // update eeprom 
  EEPROM.update(ee_PROTOCOL_ID, current_protocol);
}
//------------------------------------------------------------

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

//------------------------------------------------------------

void init_protocol()
{
  switch(current_protocol) {
    case PROTO_SYMAX5C1:
    case PROTO_SYMAXOLD:
      symax.initialize();
      break;
  }
}

//------------------------------------------------------------

// update ppm values out of ISR    
void update_ppm()
{
    for(uint8_t ch=0; ch < CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }    
}

//------------------------------------------------------------

void read_ppm()
{
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok=false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    }
    else{  //servo values between 510us and 2420us will end up here
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if(chan==3)
                ppm_ok = true; // 4 first channels Ok
        }
        chan++;
    }
}
