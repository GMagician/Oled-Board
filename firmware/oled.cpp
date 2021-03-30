/**
 * Oled 1.3" control board
 *
 * Copyright(C) 2020-2021 GMagician
 *
 * Part of this code is extracted from framework source and modified to reduce footprint
 */
#include "include/oled.h"

static I2CRegister i2cRegister;

static const Configuration PROGMEM defaultConfig = {
  .i2cAddress = 0x3D,
  .option = { .wheelAcceleration = false, .reverseEncoder = false },
  .buttonsDebounceTime = 20,
  .buttonsHoldTime = 75,
  .buttonsRemapMask = 0b0000,
  .buttonsRemapPolarity = 0b0000,
  .encoderAccRate = 25,
  .encoderDecRate = 2
};
static Configuration config;

static uint8_t gpioDirMask,
               gpioPullUpMask,
               gpioEventMask,
               gpioOutStatusMask;

static uint16_t beepFrequency = 2000;

static uint8_t redLedDuty,
               greenLedDuty;

static uint8_t buttonsReleaseMask,
               keyBeepMask,
               keyBeepDuration = 10;      // centiseconds

static uint16_t toneUsedFrequency;

static bool wheelHasMoved;
static int16_t wheelAccComponent;

#define QUEUE_SIZE    8   // MUST BE x^2
static Event eventsQueue[QUEUE_SIZE];
static volatile uint8_t eventsCount,
                        eventsHeadIndex,
                        eventsTailIndex;


/******************************/
/* WHEEL ACCELERATION HANDLER */
/******************************/
void wheelAccelerationHandler() {
  if (wheelHasMoved) {
    wheelHasMoved = false;
    wheelAccComponent += config.encoderAccRate;
    if (wheelAccComponent > (7 << 8))
      wheelAccComponent = (7 << 8);
  }
  else {
    wheelAccComponent -= config.encoderDecRate;
    if (wheelAccComponent < 0)
      wheelAccComponent = 0;
  }
}


/****************************/
/* BASE TIME TIMER HANDLING */
/****************************/
#define ms_TIMER_PRESCALER  0b010                                 // clk/8 (PWM=4KHz)
#define uS_X_ms_OVERFLOW    (clockCyclesToMicroseconds(8 * 256))  // 8=prescaler
#define FRACT_INC           ((uS_X_ms_OVERFLOW % 1000) >> 3)
#define FRACT_MAX           (1000 >> 3)

static uint16_t timerMillisCount;
static uint8_t timerFractionalCount;

SIGNAL(TIMER1_OVF_vect) {
  // copy to local variables so it can be stored in registers and shrink code
  uint8_t f = timerFractionalCount;

  f += FRACT_INC;
  if (f >= FRACT_MAX) {
    f -= FRACT_MAX;
    ++timerMillisCount;

    if (config.option.wheelAcceleration)
      wheelAccelerationHandler();
  }

  timerFractionalCount = f;
}

uint16_t millis() {
  uint16_t m;

  // timerMillisCount read is not atomic then disable interrupts to prevent race conditions
  uint8_t savedSREG = SREG;
  noInterrupts();
  m = timerMillisCount;
  SREG = savedSREG;

  return m;
}


/*******************/
/* EEPROM HANDLING */
/*******************/
/**
 *  Load value from eeprom specified address
 */
uint8_t eeRead(uint8_t address) {
  // Wait eeprom idle
  while(EECR & _BV(EEPE))
    ;
  EEAR = address;
  sbi (EECR, EERE);

  return EEDR;
}

/**
 *  Save value into eeprom at specified address
 */
void eeWrite(uint8_t address, uint8_t value) {
  // Wait eeprom idle
  while(EECR & _BV(EEPE))
    ;
  // Set Programming mode
  EECR &= ~(_BV(EEPM1) | _BV(EEPM0));
  // Set up address and data
  EEAR = address;
  EEDR = value;

  uint8_t savedSREG = SREG;
  noInterrupts();
  sbi(EECR, EEMPE);   // Program enable
  sbi(EECR, EEPE);    // Start write
  SREG = savedSREG;
}

/**
 *  Load configuration from eeprom
 */
void loadConfiguration() {
  uint8_t * configPtr = (uint8_t *)&config;
  for (uint8_t eeAddr = 0; eeAddr < sizeof(config); ++eeAddr, ++configPtr)
    *configPtr = eeRead(eeAddr);

  if (config.i2cAddress & 0b10000000) {
    // Most important thing is i2c address.
    // If invalid restore eeprom defaults to let user to communicate
    uint8_t * configPtr = (uint8_t *)&config;
    const uint8_t * defConfigPtr = (const uint8_t *)&defaultConfig;
    for (uint8_t eeAddr = 0; eeAddr < sizeof(config); ++eeAddr, ++configPtr, ++defConfigPtr) {
      *configPtr = pgm_read_byte(defConfigPtr);
      eeWrite(eeAddr, *configPtr);
    }
  }
}


/**************/
/* BOARD CODE */
/**************/
/**
 *  Signal interrupt event
 */
FORCE_INLINE void setInterruptPin() {
  OUT_WRITE(INTERRUPT_PIN, LOW);
}

/**
 *  Clear interrupt event
 */
FORCE_INLINE void resetInterruptPin() {
  SET_INPUT_PULLUP(INTERRUPT_PIN);
}

/**
 *  Read GPIOs
 */
FORCE_INLINE uint8_t readGPIOs() {
  #if GPIO3_PIN == PD3 && GPIO2_PIN == PD2 && GPIO1_PIN == PD1 && GPIO0_PIN == PD0
    return PIND & 0x0F;
  #else
    return (READ(GPIO3_PIN) << 3) | (READ(GPIO2_PIN) << 2) | (READ(GPIO1_PIN) << 1) | READ(GPIO0_PIN);
  #endif
}

/**
 *  Set red led brightness
 */
void setRedLEDBrightness() {
  SET_LEDSPWM_DUTY(REDLED_PIN, redLedDuty);
}

/**
 *  Set green led brightness
 */
void setGreenLEDBrightness() {
  SET_LEDSPWM_DUTY(GREENLED_PIN, greenLedDuty);
}

/**
 *  Add an event into queue
 */
void pushEvent(Event event) {
  if (eventsCount == QUEUE_SIZE) return;

  eventsQueue[eventsHeadIndex] = event;
  eventsHeadIndex = (eventsHeadIndex + 1) & (QUEUE_SIZE - 1);
  uint8_t savedSREG = SREG;
  noInterrupts();
  ++eventsCount;
  setInterruptPin();
  SREG = savedSREG;
}

/**
 *  Extract an event from queue
 */
Event popEvent() {
  if (eventsCount == 0)
    return NO_EVENT;

  Event event = eventsQueue[eventsTailIndex];
  if (--eventsCount == 0)
    resetInterruptPin();
  eventsTailIndex = (eventsTailIndex + 1) & (QUEUE_SIZE - 1);

  return event;
}

/**
 *  Play a tone of the specified duration
 */
static uint32_t toneTimerToggleCount;

ISR(TIMER0_COMPA_vect) {
  --toneTimerToggleCount;
  if (toneTimerToggleCount == 0) {
    cbi(TIMSK, OCIE0A);                               // Disable interrupt
    TCCR0B = 0b00000000;                              // Stop the clock (only some CS0x bits may be set so no matter if I write 0)
    WRITE(BUZZER_PIN, LOW);                           // Mute buzzer
  }
}

void playTone(uint8_t centisecs) {
  if (centisecs == 0 || beepFrequency == 0)
    return;

  toneUsedFrequency = beepFrequency;

  if (!(TIMSK & _BV(OCIE0A))) {
    TCNT0 = 0;            // Start timer at 0
    sbi(TIFR, OCF0A);     // Clear the Timer interrupt flag
  }

  // Determine which prescaler to use
  // Set the Output Compare Register (rounding up)
  uint32_t ocr = F_CPU / 2 / beepFrequency;
  uint8_t prescalerBits = 0b001;  // clk/1
  if (ocr > 256) {
    ocr >>= 3;
    prescalerBits = 0b010;        // clk/8
    if (ocr > 256) {
      ocr >>= 3;
      prescalerBits = 0b011;      // clk/64
      if (ocr > 256) {
        ocr >>= 2;
        prescalerBits = 0b100;    // clk/256
        if (ocr > 256) {
          // Can't do any better
          ocr >>= 2;
          prescalerBits = 0b101;  // clk/1024
        }
      }
    }
  }
  ocr -= 1; // Note we are doing the subtraction of 1 here to save repeatedly calculating ocr from just the frequency in the if tree above
  OCR0A = ocr;

  // Determine how many times the value toggles
  toneTimerToggleCount = (2L * beepFrequency * centisecs) / 100;

  // Output Compare A Match Interrupt Enable
  sbi(TIMSK, OCIE0A);

  // Start timer
  TCCR0B = prescalerBits << CS00;   // No other bits but CS0x are modified in whole code
}

/**
 *  GPIO initialization
 */
void gpioSetup() {
  MREPEAT(NUM_OF_GPIO, GPIO_SETUP);
}


/****************/
/* I2C HANDLING */
/****************/
/**
 *  Handle I2C registers requests
 */
uint8_t i2cRegisterHandler(bool isWrite, uint8_t value = 0) {
  switch (i2cRegister) {
    case EVENT:
      return isWrite ? -1 : popEvent().rawData;

    case RELEASEMASK:
      if (isWrite)
        buttonsReleaseMask = value >> 1;  // Remove unused bit
      return buttonsReleaseMask << 1;     // Restore for unused bit

    case DEBOUNCE_TIME:
      if (isWrite)
        config.buttonsDebounceTime = value;
      return config.buttonsDebounceTime;

    case BTNHOLD_TIME:
      if (isWrite)
        config.buttonsHoldTime = value;
      return config.buttonsHoldTime;

    case KEYBEEP_DURATION:
      if (isWrite)
        keyBeepDuration = value;
      return keyBeepDuration;

    case KEYBEEP_MASK:
      if (isWrite)
        keyBeepMask = value;
      return keyBeepMask;

    case BEEP_DURATION:
      if (isWrite)
        playTone(value);
      return toneTimerToggleCount * 1000 / 2 / toneUsedFrequency;

    case BEEP_TONE:
      if (isWrite)
        beepFrequency = uint16_t(value) * 10;
      return beepFrequency / 10;

    case BEEP_FREQ_HIGH:
      if (isWrite)
        beepFrequency = (uint16_t(value) << 8) | (beepFrequency & 0xFF);
      return beepFrequency >> 8;

    case BEEP_FREQ_LOW:
      if (isWrite)
        beepFrequency = (beepFrequency & 0xFF00) | value;
      return beepFrequency;

    case REDLED_PWM:
      if (isWrite) {
        redLedDuty = value;
        setRedLEDBrightness();
      }
      return redLedDuty;

    case GREENLED_PWM:
      if (isWrite) {
        greenLedDuty = value;
        setGreenLEDBrightness();
      }
      return greenLedDuty;

    case GPIO_DIR:
      if (isWrite) {
        gpioDirMask = value;
        gpioSetup();
      }
      return gpioDirMask;

    case GPIO_IO:
      if (isWrite)
        gpioOutStatusMask = value;
      return readGPIOs();

    case GPIO_PULLUP:
      if (isWrite) {
        gpioPullUpMask = value;
        gpioSetup();
      }
      return gpioPullUpMask;

    case GPIO_EVENTMASK:
      if (isWrite)
        gpioEventMask = value & (_BV(NUM_OF_GPIO) - 1);
      return gpioEventMask;

    case SWVERSION:
      return 2;

    default:
      uint8_t eepromAddr = (i2cRegister - I2C_EEPROM);
      if (i2cRegister >= I2C_EEPROM && eepromAddr < sizeof(config)) {
        if (isWrite)
          eeWrite(eepromAddr, value);
        return eeRead(eepromAddr);
      }
      return 0xFF;
  }
}

/**
 *  i2c "received data" event handler
 */
void onI2CReceive(int availableData) {
  if (availableData) {
    i2cRegister = I2CRegister(USI_TWI_Receive_Byte());
    while (--availableData > 0) {
      i2cRegisterHandler(true, USI_TWI_Receive_Byte());
      ++i2cRegister;
    }
  }
}

/**
 *  I2C "to transmit data" event handler
 */
void onI2CRequest() {
  int8_t value = i2cRegisterHandler(false);
  USI_TWI_Transmit_Byte(value);
}


/**************************/
/* FRAMEWORK ENTRY POINTS */
/**************************/
/**
 *  Hardware initializations
 */
void setup() {
  noInterrupts();
  CLKPR = _BV(CLKPCE);      // System clock divisor to 1 (clock is 8MHz)
  CLKPR = 0b00000000;
  interrupts();

  // Timer 1 in Fast PWM mode (Clear Timer on 0xFF, disconnect OC1A)
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(WGM12) | (ms_TIMER_PRESCALER << CS10);
  TIMSK = _BV(TOIE1);

  // Main button
  SET_INPUT_PULLUP(MAINBTN_PIN);
  SET_PWM(REDLED_PIN);
  setRedLEDBrightness();
  SET_PWM(GREENLED_PIN);
  setGreenLEDBrightness();

  // Buttons
  SET_INPUT_PULLUP(LEFTBTN_PIN);
  SET_INPUT_PULLUP(RIGHTBTN_PIN);

  // Encoder
  SET_INPUT_PULLUP(ENCA_PIN);
  SET_INPUT_PULLUP(ENCB_PIN);
  SET_INPUT_PULLUP(ENCBTN_PIN);

  // Buzzer
  TCCR0B = 0b00000000;                              // Reset to default (it's initialized by framework init code)
  TCCR0A = (COM_TOGGLE << COM0A0) | _BV(WGM01);     // Timer in CTC mode (Clear Timer on OCR0A, toggle OC0A on Compare Match)
  OUT_WRITE(BUZZER_PIN, LOW);
  toneUsedFrequency = beepFrequency;                // Initialize if read before playing a tone

  loadConfiguration();     // Read EEPROM stored configuration

  // GPIO
  gpioSetup();

  // Interrupt pin
  resetInterruptPin();

  // I2C
  USI_TWI_Slave_Initialise(config.i2cAddress);
  USI_TWI_On_Slave_Receive = onI2CReceive;
  USI_TWI_On_Slave_Transmit = onI2CRequest;

  // Floating pins are not a good thing, pull'em up
  SET_INPUT_PULLUP(UNUSED1_PIN);
}

/**
 *  Main loop
 */
void loop() {
  uint16_t actualTime = millis();

  //  Buttons handling
  static uint8_t debouncedButtonsStatus,
                 buttonsHeldStatus;
  static uint16_t pressTime[RIGHT+1];     // +1 is required to have a good sized array

  uint8_t buttonsStatus = (!READ(RIGHTBTN_PIN) << RIGHT) |
                          (!READ(LEFTBTN_PIN) << LEFT) |
                          (!READ(MAINBTN_PIN) << MAIN) |
                          (!READ(ENCBTN_PIN) << WHEEL);
  for (ButtonID btnID = WHEEL; btnID <= RIGHT; ++btnID) {
    uint8_t btnMask = _BV(btnID);

    if (buttonsStatus & btnMask) {
      // Debounce
      if (!(debouncedButtonsStatus & btnMask) && (actualTime - pressTime[btnID]) >= config.buttonsDebounceTime) {
        debouncedButtonsStatus |= btnMask;

        Event event = BUTTON_EVENT(PRESS, btnID);
        pushEvent(event);

        if ((keyBeepMask >> 1) & btnMask) // keyBeepMask bit 0 is wheel movement so >> 1 to get only buttons
          playTone(keyBeepDuration);
      }
      // Held detection
      if (!(buttonsHeldStatus & btnMask) && (actualTime - pressTime[btnID]) >= (config.buttonsHoldTime * 10)) {
        buttonsHeldStatus |= btnMask;

        Event event = BUTTON_EVENT(HELD, btnID);
        pushEvent(event);
      }
    }
    else /*if (!(buttonsStatus & btnMask))*/ {
      // Button has been release, check if event is required
      if (debouncedButtonsStatus & buttonsReleaseMask & btnMask) {
        Event event = BUTTON_EVENT(RELEASE, btnID);
        pushEvent(event);
      }
      debouncedButtonsStatus &= ~btnMask;
      buttonsHeldStatus &= ~btnMask;
      pressTime[btnID] = actualTime;
    }
  }

  // Rotary encoder handling (with acceleration)
  static bool triggerArmed,
              prevEncA,
              prevEncB;

  bool encA = READ(ENCA_PIN);
  bool encB = READ(ENCB_PIN);
  bool encBChanged = encB ^ prevEncB;
  wheelHasMoved |= (encA ^ prevEncA || encBChanged);
  if (prevEncA)
    triggerArmed = prevEncA;
  else if (triggerArmed) {
    if (encBChanged) {
      bool cw = prevEncB ^ config.option.reverseEncoder;
      Event event = ENCODER_EVENT(cw);
      pushEvent(event);

      if (keyBeepMask & 0b00001)
        playTone(keyBeepDuration);

      triggerArmed = false;
    }
  }
  prevEncB = encB;
  prevEncA = encA;

  // GPIO handling
  static uint8_t prevGPIOStatus;

  uint8_t gpioStatus = readGPIOs();
  if ((gpioStatus ^ prevGPIOStatus) & gpioEventMask) {
    Event event = GPIO_EVENT(gpioStatus);
    pushEvent(event);
  }
  prevGPIOStatus = gpioStatus;

  MREPEAT(NUM_OF_GPIO, GPIO_WRITE); // Always refresh outputs (needed when buttons are remapped to GPIOs)
}
