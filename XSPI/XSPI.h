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

#ifndef _XSPI_H_INCLUDED
#define _XSPI_H_INCLUDED

#include <stdio.h>
#include <Arduino.h>
#include <avr/pgmspace.h>

#ifndef __AVR_XMEGA__
#error do not include this file for non-XMEGA processors
#endif // __AVR_XMEGA__

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
//#define SPI_CLOCK_DIV64 0x07 - redundant

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define SPI_DEFAULT_INTLVL 3 /* int level 3 by default */

// NOTE: this class is defined for SPI master only

class SPIClass
{
public:
  static byte transfer(byte _data);

  // SPI Configuration methods

  static void attachInterrupt();
  static void detachInterrupt(); // Default

  static void begin(); // Default
  static void end();

  static void setBitOrder(uint8_t);
  static void setDataMode(uint8_t);
  static void setClockDivider(uint8_t);
};

extern SPIClass SPI;

#endif // _XSPI_H_INCLUDED

