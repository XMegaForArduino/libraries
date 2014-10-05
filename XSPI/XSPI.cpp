/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

// adapted for the ATXMega processors by Bob Frazier, S.F.T. Inc.

#include "pins_arduino.h"
#include "XSPI.h"

// NOTE:  this is for SPIC only, on the xmega, which should be mapped to pins 11-13 for Arduino compatibility
//        'SS' is typically assigned as '10'.  MOSI, MISO, and SCK must be assigned to the corresponding SPIC pins
//        Also, according to the specs, SS as an input can switch this to slave mode when low, so leave it as an output
//        (see 'D' manual section 18.3, and others)

SPIClass SPI;

void SPIClass::begin()
{
  spiRate = spiRate > 12 ? 6 : spiRate/2;

  SPIC_INTCTRL = 0; // disable SPI interrupts

  // Set SS to high so a connected chip will be "deselected" by default
  digitalWrite(SS, HIGH);
  // SS must be in output mode even it is not chip select
  pinMode(SS, OUTPUT);
  pinMode(MISO, INPUT | INPUT_SENSE_BOTH);

  // enable and initialize as SPI master with max clock and mode 0

  SPIC_CTRL = SPI_ENABLE_bm  // enable SPI
            | SPI_MASTER_bm  // master mode
            | SPI_CLK2X_bm   // clock 2x bit
            | SPI_MODE_0_gc  // mode zero
            | SPI_PRESCALER_DIV4_gc;  // low 2 bits are rate, see D manual section 18.6.1 table 18-3
  // NOTE:  mode bits 2 and 3 are both zero for mode 0

  // By doing this AFTER enabling SPI, we avoid accidentally
  // clocking in a single bit since the lines go directly
  // from "input" to SPI control.  
  // http://code.google.com/p/arduino/issues/detail?id=888
  // NOTE: not sure if it applies to xmega
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
}


void SPIClass::end()
{
  SPIC_CTRL &= ~SPI_ENABLE_bm; // disable SPI
}

void SPIClass::setBitOrder(uint8_t bitOrder)
{
  if(bitOrder == LSBFIRST)
  {
    SPIC_CTRL |= SPI_DORD_bm;
  }
  else
  {
    SPIC_CTRL &= ~(SPI_DORD_bm);
  }
}

void SPIClass::setDataMode(uint8_t mode)
{
  SPIC_CTRL = (SPIC_CTRL & ~SPI_MODE_gm) | ((mode & 3) << SPI_MODE_gp);
}

void SPIClass::setClockDivider(uint8_t rate)
{
  SPIC_CTRL = (SPIC_CTRL & ~SPI_PRESCALER_gm) | (rate & 3) << SPI_PRESCALER_gp; // pre-scaler
  SPIC_CTRL = (SPIC_CTRL & ~SPI_CLK2X_bm) | (rate & 4) ? SPI_CLK2X_bm : 0;      // clock 2x
}

byte SPIClass::transfer(byte _data)
{
register uint8_t tval = SPIC_STATUS; // read status register [to clear the bit]
volatile short ctr; // temporary

  SPIC_DATA = b; // NOW, accessing the SPIC_DATA clears the status bit (D manual, 18.6.3)
                 // and it transmits the FF on the SPI bus, while receiving a data byte

  ctr = 0;
  while (!(SPIC_STATUS & _BV(7)))// { } // wait for the bit flag that says it's complete
  {
    if(!(++ctr)) // this is a safety to prevent infinite loops.  remove if not needed.
      break;
  }

  return SPIC_DATA;
}

void SPIClass::attachInterrupt()
{
  SPI_INTCTRL = (SPI_INTCTRL & ~SPI_INTLVL_gm)
              | ((SPI_DEFAULT_INTLVL & 3) << SPI_INTLVL_gp);
}

void SPIClass::detachInterrupt()
{
  SPI_INTCTRL &= ~SPI_INTLVL_gm
}


