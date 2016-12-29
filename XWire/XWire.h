//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                __  ____        __ _                _                     //
//                \ \/ /\ \      / /(_) _ __  ___    | |__                  //
//                 \  /  \ \ /\ / / | || '__|/ _ \   | '_ \                 //
//                 /  \   \ V  V /  | || |  |  __/ _ | | | |                //
//                /_/\_\   \_/\_/   |_||_|   \___|(_)|_| |_|                //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*
  ORIGINAL COPYRIGHT BANNER for 'TwoWire.h' from which it is derived
  (this includes the copyright terms, which apply to this, a derived work)

  TwoWire.h - TWI/I2C library for Arduino & Wiring
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

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

// Modified for XMega processors by Bob Frazier for XMegaForArduino project
// http://github.com/XMegaForArduino


#ifndef TwoWire_h
#define TwoWire_h

#include <Arduino.h> /* make sure I do this - it gets me important OTHER definitions from core and variants */

#include <inttypes.h>
#include "Stream.h"

#include "utility/twi.h" /* this allows use of the 'twi.h' low-level interface also, by including THIS file */

#define TWO_WIRE_BUFFER_LENGTH 32 /* matches TWI_BUFFER_LENGTH in 'twi.h' for now */


class TwoWire : public Stream
{
  protected:
    /*static*/ uint8_t rxBuffer[TWO_WIRE_BUFFER_LENGTH];
    /*static*/ uint8_t rxBufferIndex;
    /*static*/ uint8_t rxBufferLength;

    /*static*/ uint8_t txAddress;
    /*static*/ uint8_t txBuffer[TWO_WIRE_BUFFER_LENGTH];
    /*static*/ uint8_t txBufferIndex;
    /*static*/ uint8_t txBufferLength;

    /*static*/ uint8_t transmitting;
    /*static*/ void (*user_onRequest)(TwoWire *);
    /*static*/ void (*user_onReceive)(TwoWire *,int);

    // callbacks [must map back into the class]
    static void onRequestService(TWI_t *, void *);
    static void onReceiveService(TWI_t *, void *, const uint8_t *, int);

    TWI_t *pTWI; // pointer to TWI register for this connection

  private:  // linked list of objects

    static TwoWire * pHead;
    TwoWire *pNext;


  public:
    TwoWire(TWI_t *);
    ~TwoWire();

    static TwoWire * FromTWI(TWI_t *pTWI); // get ptr to object from TWI_t pointer

    void begin();
    void begin(uint8_t);
    void begin(int);
    void beginTransmission(uint8_t);
    void beginTransmission(int);

    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);

    void onReceive(void (*)(TwoWire *, int) );
    void onRequest( void (*)(TwoWire *) );
  
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }

    using Print::write;
};

extern TwoWire Wire;


#endif

