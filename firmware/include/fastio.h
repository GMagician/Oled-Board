#pragma once

/**
 * Fast I/O Routines
 * Use direct port manipulation to save scads of processor time.
 */

/**
 * Include Ports and Functions
 */
#include <wiring_private.h>
#include "fastio/fastio_4313.h"

/**
 * Magic I/O routines
 */
#define _READ(IO)             bool(DIO##IO##_RPORT & _BV(DIO##IO##_PIN))

#define _WRITE(IO,V)          do { if (V) sbi(DIO##IO##_WPORT, DIO##IO##_PIN); \
                                   else   cbi(DIO##IO##_WPORT, DIO##IO##_PIN); \
                              } while (0)

#define _TOGGLE(IO)           sbi(DIO##IO##_RPORT, DIO##IO##_PIN)

#define _SET_INPUT(IO)        cbi(DIO##IO##_DDR, DIO##IO##_PIN)
#define _SET_OUTPUT(IO)       sbi(DIO##IO##_DDR, DIO##IO##_PIN)

#define SET_INPUT(IO)         do { _SET_INPUT(IO); _WRITE(IO, LOW); } while (0)
#define SET_INPUT_PULLUP(IO)  do { _SET_INPUT(IO); _WRITE(IO, HIGH); } while (0)
#define SET_OUTPUT(IO)        _SET_OUTPUT(IO)
#define SET_PWM               SET_OUTPUT

#define READ(IO)              _READ(IO)
#define WRITE(IO,V)           _WRITE(IO,V)
#define TOGGLE(IO)            _TOGGLE(IO)

#define OUT_WRITE(IO,V)       do { SET_OUTPUT(IO); WRITE(IO,V); } while (0)

/**
 * Timer and Interrupt Control
 */
// Compare Modes
enum CompareMode {
  COM_NORMAL,          //  0
  COM_TOGGLE,          //  1  Non-PWM: OCnx ... Both PWM (WGM 9,11,14,15): OCnA only ... else NORMAL
  COM_CLEAR_SET,       //  2  Non-PWM: OCnx ... Fast PWM: OCnx/Bottom ... PF-FC: OCnx Up/Down
  COM_SET_CLEAR        //  3  Non-PWM: OCnx ... Fast PWM: OCnx/Bottom ... PF-FC: OCnx Up/Down
};

#define _SET_COMnQ(T,V)     (*T.TCCRnA = (*T.TCCRnA & ~(0x3 << (6-2*T.q))) | (uint8_t(V) << (6-2*T.q)))

struct Timer {
  volatile uint8_t * TCCRnA;
  volatile uint16_t * OCRnQ;
  uint8_t q;                    // the timer output [0->1] (A->B)
};

#define GET_PWM_TIMER(P)    (P==PB3 ? Timer({&TCCR1A, &OCR1A, 0}) : \
                             P==PB4 ? Timer({&TCCR1A, &OCR1B, 1}) : \
                             Timer({nullptr, nullptr, 0}))

#define SET_LEDSPWM_DUTY(P,D) do{                                   \
                                Timer timer = GET_PWM_TIMER(P);     \
                                if (timer.TCCRnA == nullptr)        \
                                  WRITE(P, D>=128);                 \
                                else if (D == 0 || D == 255) {      \
                                  _SET_COMnQ(timer, COM_NORMAL);    \
                                  WRITE(P, D>=128);                 \
                                }                                   \
                                else {                              \
                                  _SET_COMnQ(timer, COM_CLEAR_SET); \
                                  *timer.OCRnQ = D;                 \
                                }                                   \
                              } while (0)
