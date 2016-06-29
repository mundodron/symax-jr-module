#ifndef _COMMON_H_
#define _COMMON_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "nrf24l01.h"

#define RF_POWER RF24_PA_MIN 

// supported protocols
enum {
    //PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    //PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    //PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    //PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    //PROTO_H7,           // EAchine H7, MoonTop M99xx
    //PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    //PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    //PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    //PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    //PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    //PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
    //PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    PROTO_END
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

extern uint8_t current_protocol;

extern uint8_t transmitterID[];

extern uint16_t ppm[12];

#endif // _COMMON_H_
