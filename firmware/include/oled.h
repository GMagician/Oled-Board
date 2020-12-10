#pragma once

#include <Arduino.h>
extern "C" {
  #include <USI_TWI_Slave\USI_TWI_Slave.h>
}
#include "include/macros.h"
#include "include/fastio.h"
#include "include/pins.h"

#define NUM_OF_GPIO       4
#define GPIO_SETUP(N)     if ((gpioDirMask | config.buttonsRemapMask) & _BV(N)) \
                            SET_OUTPUT(GPIO##N##_PIN);                          \
                          else if (gpioPullUpMask & _BV(N))                     \
                            SET_INPUT_PULLUP(GPIO##N##_PIN);                    \
                          else                                                  \
                            SET_INPUT(GPIO##N##_PIN);
#define GPIO_WRITE(N)     if (config.buttonsRemapMask & _BV(N))                                                     \
                            WRITE(GPIO##N##_PIN, (debouncedButtonsStatus ^ config.buttonsRemapPolarity) & _BV(N));  \
                          else if (gpioDirMask & _BV(N))                                                            \
                            WRITE(GPIO##N##_PIN, gpioOutStatusMask & _BV(N));


// I2C registers
enum I2CRegister :uint8_t {
  EVENT = 0x01,
  RELEASEMASK,
  DEBOUNCE_TIME,
  BTNHOLD_TIME,

  KEYBEEP_DURATION = 0x10,
  KEYBEEP_MASK,
  BEEP_DURATION,
  BEEP_TONE,
  BEEP_FREQ_HIGH,
  BEEP_FREQ_LOW,

  REDLED_PWM = 0x20,
  GREENLED_PWM,

  GPIO_DIR = 0x30,
  GPIO_IO,
  GPIO_PULLUP,
  GPIO_EVENTMASK,

  I2C_EEPROM = 0xC0, // sizeof(Configuration) consecutive registers are used

  SWVERSION = 0xF0
};

I2CRegister operator++(I2CRegister &orig) {
  orig = static_cast<I2CRegister>(orig + 1); // static_cast required because enum + int -> int
  return orig;
}

// EEPROM mapping
struct Configuration {
  uint8_t i2cAddress;               // i2c address
  struct Option {
    uint8_t wheelAcceleration:1;    // Wheel acceleration option
    uint8_t :6;   // reserved
    uint8_t reverseEncoder:1;       // Reverse wheel encoder count
  } option;
  uint8_t buttonsDebounceTime;      // milliseconds
  uint8_t buttonsHoldTime;          // centiseconds
  uint8_t buttonsRemapMask;         // KeysMask
  uint8_t buttonsRemapPolarity;     // KeysMask
  uint8_t encoderAccRate;           // Increments
  uint8_t encoderDecRate;           // Decrements
};


/**
  * Event handling
  */
enum EventCategory { EC_NONE, EC_WHEEL, EC_BUTTON, EC_GPIO };

// Encoder wheel event
struct WheelEvent {
  uint8_t antiClockWise:1;
  uint8_t clockWise:1;
  uint8_t steps:3;    // 0 or cumulative steps When 'ExtendedFeature.wheelAcc' is enabled
  EventCategory category:3;
  };

// Buttons event
enum ButtonEventID { RELEASE, PRESS, HELD };
enum ButtonID { WHEEL, MAIN, LEFT, RIGHT };

ButtonID &operator++(ButtonID &v) {
  v = static_cast<ButtonID>(static_cast<uint8_t>(v) + 1);
  return v;
}

struct ButtonEvent {
    ButtonEventID event:2;
    ButtonID button:2;
    uint8_t :1;       // reserved
    EventCategory category:3;
  };

struct GPIOEvent {
  uint8_t gpios:4;
  uint8_t :1;         // reserved
  EventCategory category:3;
};

union Event {
  WheelEvent wheel;
  ButtonEvent buttons;
  GPIOEvent gpios;
  uint8_t rawData;
};
constexpr static Event NO_EVENT = { EC_NONE };
