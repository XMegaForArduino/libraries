//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//          __  ____        __ _                                            //
//          \ \/ /\ \      / /(_) _ __  ___     ___  _ __   _ __            //
//           \  /  \ \ /\ / / | || '__|/ _ \   / __|| '_ \ | '_ \           //
//           /  \   \ V  V /  | || |  |  __/ _| (__ | |_) || |_) |          //
//          /_/\_\   \_/\_/   |_||_|   \___|(_)\___|| .__/ | .__/           //
//                                                  |_|    |_|              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*
  ORIGINAL COPYRIGHT BANNER for 'TwoWire.cpp' from which it is derived
  (this includes the copyright terms, which apply to this, a derived work)

  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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



extern "C"
{
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include "utility/twi.h"
#include "XWire.h"

// Initialize Class Variables //////////////////////////////////////////////////
// NOTE:  the xmega can have multiple TWI interfaces... instantiate!

TwoWire * TwoWire::pHead = NULL; // points to head of chain

//uint8_t TwoWire::rxBuffer[TWO_WIRE_BUFFER_LENGTH];
//uint8_t TwoWire::rxBufferIndex = 0;
//uint8_t TwoWire::rxBufferLength = 0;

//uint8_t TwoWire::txAddress = 0;
//uint8_t TwoWire::txBuffer[TWO_WIRE_BUFFER_LENGTH];
//uint8_t TwoWire::txBufferIndex = 0;
//uint8_t TwoWire::txBufferLength = 0;

//uint8_t TwoWire::transmitting = 0;
//void (*TwoWire::user_onRequest)(void);
//void (*TwoWire::user_onReceive)(int);

// Constructors ////////////////////////////////////////////////////////////////

TwoWire::TwoWire(TWI_t *pR)
{
  pTWI = pR;

  rxBufferIndex = 0;
  rxBufferLength = 0;

  txAddress = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  transmitting = 0;

  user_onRequest = NULL;
  user_onReceive = NULL;

  pNext = NULL;

  // TODO:  turn off interrupts for this?
  if(!pHead)
  {
    pHead = this;
  }
  else
  {
    TwoWire *pLast = pHead;
    while(pLast->pNext)
    {
      pLast = pLast->pNext;
    }

    pLast->pNext = this; // added to end of chain
  }
}

TwoWire::~TwoWire()
{
TwoWire *pThat = pHead;

  // TODO:  shut down

  // remove from chain

  // TODO:  turn off interrupts for this?
  while(pThat && pThat->pNext != this)
  {
    pThat = pThat->pNext;
  }

  if(pThat)
  {
    pThat->pNext = pNext; // unhook
  }  
}

TwoWire * TwoWire::FromTWI(TWI_t *pTWI)
{
TwoWire *pThat = pHead;

  // TODO:  turn off interrupts for this?
  while(pThat && pThat->pTWI != pTWI)
  {
    pThat = pThat->pNext;
  }

  return pThat;
}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWire::begin(void)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init(pTWI);
}

void TwoWire::begin(uint8_t address)
{
  twi_setAddress(pTWI, address);
  twi_attachSlaveTxEvent(pTWI, onRequestService, this);
  twi_attachSlaveRxEvent(pTWI, onReceiveService, this);

  begin();
}

void TwoWire::begin(int address)
{
  begin((uint8_t)address);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  // clamp to buffer length
  if(quantity > TWO_WIRE_BUFFER_LENGTH)
  {
    quantity = TWO_WIRE_BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(pTWI, address, rxBuffer, quantity, sendStop, 10);

  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire::beginTransmission(uint8_t address)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to 
//	perform a repeated start. 
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
  // transmit buffer (blocking)
  int8_t ret = twi_writeTo(pTWI, txAddress, txBuffer, txBufferLength, 1, sendStop);

  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
  return endTransmission(true);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(uint8_t data)
{
  if(transmitting)
  {
  // in master transmitter mode
    // don't bother if buffer is full
    if(txBufferLength >= TWO_WIRE_BUFFER_LENGTH)
    {
      setWriteError();
      return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer   
    txBufferLength = txBufferIndex;
  }
  else
  {
  // in slave send mode
    // reply to master
    twi_transmit(pTWI, &data, 1);
  }
  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  if(transmitting)
  {
  // in master transmitter mode
    for(size_t i = 0; i < quantity; ++i)
    {
      write(data[i]);
    }
  }
  else
  {
  // in slave send mode
    // reply to master
    twi_transmit(pTWI, data, quantity);
  }
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::available(void)
{
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::read(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength)
  {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::peek(void)
{
  int value = -1;
  
  if(rxBufferIndex < rxBufferLength)
  {
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void TwoWire::flush(void)
{
  // XXX: to be implemented.
}

// behind the scenes function that is called when data is received
void TwoWire::onReceiveService(TWI_t *pTWI, void *pCtx, const uint8_t *inBytes, int numBytes)
{
  TwoWire *pTW = (TwoWire *)pCtx;
  
  if(!pTW)
  {
    pTW = FromTWI(pTWI);
  }

  // don't bother if user hasn't registered a callback (or no object)
  if(!pTW || !pTW->user_onReceive)
  {
    return;
  }
  // don't bother if rx buffer is in use by a master requestFrom() op
  // i know this drops data, but it allows for slight stupidity
  // meaning, they may not have read all the master requestFrom() data yet
  if(pTW->rxBufferIndex < pTW->rxBufferLength)
  {
    return;
  }
  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel
  for(uint8_t i = 0; i < numBytes; ++i)
  {
    pTW->rxBuffer[i] = inBytes[i];    
  }
  // set rx iterator vars
  pTW->rxBufferIndex = 0;
  pTW->rxBufferLength = numBytes;

  // alert user program
  pTW->user_onReceive(pTW, numBytes);
}

// behind the scenes function that is called when data is requested
void TwoWire::onRequestService(TWI_t *pTWI, void *pCtx)
{
  TwoWire *pTW = (TwoWire *)pCtx;
  
  if(!pTW)
  {
    pTW = FromTWI(pTWI);
  }

  // don't bother if user hasn't registered a callback
  if(!pTW || !pTW->user_onRequest)
  {
    return;
  }

  // reset tx buffer iterator vars
  // !!! this will kill any pending pre-master sendTo() activity

  pTW->txBufferIndex = 0;
  pTW->txBufferLength = 0;

  // alert user program
  pTW->user_onRequest(pTW);
}

// sets function called on slave write
void TwoWire::onReceive( void (*function)(TwoWire *, int) )
{
  user_onReceive = function;
}

// sets function called on slave read
void TwoWire::onRequest( void (*function)(TwoWire *) )
{
  user_onRequest = function;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

//TwoWire Wire = TwoWire();
TwoWire Wire(&(DEFAULT_TWI));


