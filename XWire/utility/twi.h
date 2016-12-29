//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                        _              _     _                            //
//                       | |_ __      __(_)   | |__                         //
//                       | __|\ \ /\ / /| |   | '_ \                        //
//                       | |_  \ V  V / | | _ | | | |                       //
//                        \__|  \_/\_/  |_|(_)|_| |_|                       //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*
  ** ORIGINAL COPYRIGHT BANNER **
  (this includes the copyright terms, which apply to this, a derived work)

  twi.h - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Modified for XMega processors by Bob Frazier for XMegaForArduino project
// http://github.com/XMegaForArduino

#ifndef twi_h
#define twi_h

#include <Arduino.h> // make sure, because I use custom board-specific definitions

#ifdef __cplusplus
extern "C"
{
#endif // cplusplus

  #include <inttypes.h>

  // note:  default frequency is 100khz
  #ifndef TWI_FREQ
  #define TWI_FREQ 100000L
  #endif

  #ifndef TWI_BUFFER_LENGTH
  #define TWI_BUFFER_LENGTH 32
  #endif

  // internal state machine states (note 'ready' is zero)
  #define TWI_READY 0
  #define TWI_MRX   1
  #define TWI_MTX   2
  #define TWI_SRX   3
  #define TWI_STX   4

  // various errors (for compatibility)
  #define TWI_ERROR_NONE       0xff
  #define TWI_ERROR_SLAVE_NACK 0
  #define TWI_ERROR_DATA_NACK  1
  #define TWI_ERROR_BUS_ERROR  2
  #define TWI_ERROR_ARB_LOST   3
  #define TWI_ERROR_COLLISION  4

  // utility functions  

  void twi_init(TWI_t *);          // call *AFTER* 'setAddress()' for slave
  void twi_shutdown(TWI_t *);      // added, shuts down TWI so you can re-start it without booting
  void twi_setAddress(TWI_t *, uint8_t); // call this *BEFORE* calling 'twi_init()' for slave
  uint8_t twi_readFrom(TWI_t *, uint8_t, uint8_t*, uint8_t, uint8_t, uint16_t);
  uint8_t twi_writeTo(TWI_t *, uint8_t, const uint8_t *, uint8_t, uint8_t, uint8_t);
  uint8_t twi_transmit(TWI_t *, const uint8_t*, uint8_t);
  void twi_attachSlaveRxEvent(TWI_t *, void (*)(TWI_t *, void *, const uint8_t *, int), void * );
  void twi_attachSlaveTxEvent(TWI_t *, void (*)(TWI_t *, void *), void * );
  void twi_stop(TWI_t *);

#ifdef __cplusplus
}
#endif // cplusplus

#endif

