---
layout: post
title: Arduino to Arduino SPI with Custom Clock Rate
categories: [electronics]
tags: [Arduino, SPI]
---


### Introduction

In the [previous post]({% post_url 2019-03-31-arduino-to-arduino-spi-communication %})
I built a demo of SPI between two Arduino UNOs. I used the Arduino's old SPI
library and the SPI internals were abstracted away. I was still curious about
how SPI can be setup between two heterogeneous systems.

I decided to continue with the current arduino setup and started to look into
various parameters that affect SPI.

<video playinline autoplay loop muted style="max-width: 100%; min-height: 410px;">
  <source type="video/webm" src="/assets/a2a-spi-hero-animation.webm">
  <source type="video/mp4" src="/assets/a2a-spi-hero-animation.mp4">
</video>

Refer my [previous post]({% post_url 2019-03-31-arduino-to-arduino-spi-communication %})
for breadboard layout and schematic.

### SPI Configuration Parameters

Systems that are talking through SPI should align on three configuration parameters:

1. Clock Rate
2. Bit Order i.e. LSB First or MSB First
3. Data Mode

In the new Arduino SPI API, all these parameters could be configured through
[SPISettings](https://www.arduino.cc/en/Reference/SPISettings) object.

In my previous experiement, I relied on the default values for Data Order, and
Data Mode, and I used `SPI_CLOCK_DIV8` on master Arduino, and left default clock
configuration on slave. I wanted to verify these default values to understand
the configuration of my working setup.

I looked into the SPI.h code from [ArduinoCore-avr repository](https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/SPI/src/SPI.h).
This is the same file that Arduino IDE loads while compiling my sketch. I also
found out that Arduino UNO's microcontroller (ATmega328P) stores SPI
configuration in `SPCR` (SPI Control Register) and `SPSR` (SPI Status Register),
and I can print the values of these registers and verify the configuration
through Serial monitor. One can refer [this article](http://maxembedded.com/2013/11/the-spi-of-the-avr/#Reg_Description)
to understand the definition of each bit in `SPCR` and `SPSR` if you are too lazy
to search the datasheet.

So, based on SPI.h and register values, I found out that

 - The default clock rate is 4 MHz,
 - Bit Order is set to `MSBFIRST`, and
 - Data Mode is set to `SPI_MODE0`, which CPOL=0, and CPHA=0.

Read more about Data Mode in this [wikipedia page](http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Clock_polarity_and_phase).

Also, in the previous setup, when I setup clock divider to `SPI_CLOCK_DIV8`, it got
the Clock Rate Down to 1 MHz (i.e 16 Mhz / 8, as 16 MHz is default clock
rate for Arduino UNO).


### Custom Clock Rate

I tried out various clock rates from 125 KHz (fosc / 128) to 8 MHz (fosc / 2) and
I found out that the setup doesn't work when the both the systems are
configured to use the same SPI clock rate i.e. same values in SPI2X, SPR1, SPR0 registers.

Later, I looked into ATmega328P datasheet ([page 167](https://www.sparkfun.com/datasheets/Components/SMD/ATMega328.pdf)) and found out that the recommended
slave clock rate needs to be longer than 4x of master clock rate.

> To ensure correct sampling of the clock signal, the minimum low and high periods should be:
>
> Low periods: Longer than 2 CPU clock cycles.
>
> High periods: Longer than 2 CPU clock cycles.

With this bit of information, I decided to have slave SPI rate to be 4x faster
than master, and at low and high clock speeds, this configuration worked fine.


Note that 4x is not ideal because the data sheet mentions that it should be
longer than 4x, so I would recommend 8x difference for production systems.


**Working Low Speed Setup:** [commit](https://github.com/keplerbelt/arduino-to-arduino-spi-demo/commit/e92ad7c9fd97c661009cb97302bc60a54ec6639b)

- SPI Master Uno: 125 KHz
- SPI Slave Uno: 500 KHz


**Working High Speed Setup:** [commit](https://github.com/keplerbelt/arduino-to-arduino-spi-demo/commit/534fcbbd63cbc2503b20a1f5866a7eccdce004a3)

- SPI Master Uno: 2 MHz
- SPI Slave Uno: 8 MHz


### Programming

I had a make a few changes in the message format and optimise the ISR callback
on slave because I was noticing that there are some ASCII 7 (BELL) characters
being sent.

#### Code on Master Arduino (with Push Buttons)

```c
#include <SPI.h>
#include "pins_arduino.h"

struct Button {
  int pin = 0;
  int currentState = HIGH;
  unsigned long stateSince = 0;
  int debounce = 100;
  int toggleValue = 0;
  char label = '-';
};

struct Button buttonA;
struct Button buttonB;

SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);

void setup (void) {
  // Serial for Debugging
  Serial.begin (9600);

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Setup button A
  buttonA.pin = 6;
  buttonA.label = 'A';
  pinMode(buttonA.pin, INPUT_PULLUP);

  // Setup button B
  buttonB.pin = 7;
  buttonB.label = 'B';
  pinMode(buttonB.pin, INPUT_PULLUP);
}

void loop (void) {
  bool buttonA_changed = buttonToggle(&buttonA);
  bool buttonB_changed = buttonToggle(&buttonB);

  if (buttonA_changed) {
    xferData(buttonA);
  }

  if (buttonB_changed) {
    xferData(buttonB);
  }
}

void xferData(struct Button button) {
  char c;

  SPI.beginTransaction(spiSettings);

  // Enable Slave Select
  // SS is pin 10
  digitalWrite(SS, LOW);

  // Create payload
  char data[4] = {button.label, ('0' + button.toggleValue), 'Z', '\n'};
  Serial.println(data);

  // Send data
  for (const char * p = data; c = *p; p++) {
    SPI.transfer(c);
  }

  // Disable Slave Select
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
}


bool buttonToggle(struct Button* button) {
  bool toggleState = false;
  int reading = digitalRead(button->pin);

  if (reading == button->currentState) {
  } else {
    unsigned long int currentTime = millis();

    if (((currentTime - button->stateSince) > button->debounce)
        && button->currentState == LOW) {
      toggleState = true;
      if (button->toggleValue == 0) {
        button->toggleValue = 1;
      } else {
        button->toggleValue = 0;
      }
    }
    button->currentState = reading;
    button->stateSince = currentTime;
  }
  return toggleState;
}
```

#### Code on Slave Arduino (with LEDs)

```c
#include "pins_arduino.h"

char buf [100];
volatile byte pos;
volatile boolean process_it;

int pinA = 6;
int pinB = 7;


void setup (void) {
  // Serial for debugging
  Serial.begin (9600);

  // Act as slave, MISO should be OUTPUT
  pinMode(MISO, OUTPUT);

  // Turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // Turn on interrupts
  SPCR |= _BV(SPIE);

  // CLK = 8 MHz
  //SPCR |= _BV(SPR1);
  SPSR |= _BV(SPI2X);

  pos = 0;
  process_it = false;

  // LED Output
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
}


// SPI ISR
ISR (SPI_STC_vect) {
  byte c = SPDR;

  if (c > 47 && c < 91) {
    // add to buffer if room
    if (pos < sizeof buf) {
      buf[pos++] = c;

      // Z (ASCII 90) means process the buffer
      if (c == 90) {
        process_it = true;
      }
    }
  } else {
    pos = 0;
  }
}

void loop (void) {
  // wait for flag set in ISR
  if (process_it) {
    changeLed(buf[0], buf[1]);
    process_it = false;
  }
}

// update LED output
void changeLed(char pinSel, char valSel) {
  int pin, value;

  if (pinSel == 'A') {
    pin = pinA;
  } else if (pinSel == 'B') {
    pin = pinB;
  } else {
    return;
  }

  if (valSel == '1') {
    value = HIGH;
  } else if (valSel == '0') {
    value = LOW;
  } else {
    return;
  }

  digitalWrite(pin, value);
}
```

### Next Steps

Now I am quite confident about dealing with different SPI configuration
parameters. I want to bring in a different arduino board in the mix. I would
like setup SPI between an Arduino UNO (16 MHz) and Arduino Due (84 Mhz).

Stay tuned and feel free to [tweet me](https://www.twitter.com/kepler_belt)
if you like/hate my content.
