//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                         _              _                                 //
//                        | |_ __      __(_)    ___                         //
//                        | __|\ \ /\ / /| |   / __|                        //
//                        | |_  \ V  V / | | _| (__                         //
//                         \__|  \_/\_/  |_|(_)\___|                        //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*
  ** ORIGINAL COPYRIGHT BANNER **
  (this includes the copyright terms, which apply to this, a derived work)

  twi.c - TWI/I2C library for Wiring & Arduino
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

// --------------------------------------------------------------------------------
// Modified for XMega processors by 'Big Bad Bombastic Bob' Frazier
// for the XMegaForArduino project http://github.com/XMegaForArduino
//
// NOTE:  In many ways, this is a TRIVIAL implementation of the 2-wire protocol.
//        The most useful information that can be gleaned from this library is
//        the sequence of events, and correct bit assignments for the various
//        registers.  A similar work that ONLY implements the XMega-specific
//        code would not, at least by ME, be considered a "derived work", and as
//        such, COULD be included in 'closed source' applications (for example).
//        However, the original GPL license requirement exists for anything that
//        is an actual "derived work".
//        So, use discretion, if you use THIS as an example of "how to do TWI".
// --------------------------------------------------------------------------------


#include <math.h>
#include <stdlib.h>
#include <inttypes.h>


#include "twi.h" // the one in THIS directory, and NOT 'util/twi.h' (which is needed for the atmega version)


//#define MASTER_SMART_ACK /* uncomment this to use 'smart ack' and not the ISR to generate acks from master */
//#define MASTER_QUICK_COMMAND /* uncomment this to set the 'quick command' bit in init - this typically screws up though */
//#define USE_STATIC_PORT_STATE /* uncomment this to pre-allocate buffers for all available TWI; otherwise, use malloc() */
//#define DEBUG /* uncomment this for debug output - you must implement the debug output functions elsewhere */

// NOTE:  'twi.h' includes Arduino.h which includes the correct pins_arduino.h file

#ifndef DEFAULT_TWI
#error no default TWI
#endif // DEFAULT_TWI

#define TWI_ADDRESS_MASTER 0xff
#define TWI_ADDR_WRITE 0x00 /* not defined in io.h included file, so I define here */
#define TWI_ADDR_READ  0x01 /* I added this for bit arbitration */


#ifdef DEBUG
extern void error_print(const char *p1); // TEMPORARY
extern void error_print_(const char *p1); // TEMPORARY
extern void error_printL(unsigned long); // TEMPORARY
extern void error_printL_(unsigned long); // TEMPORARY
#endif // DEBUG


typedef struct __TWI_STATE__
{
  // COMMON
  uint8_t twi_address;             // slave address, or FFH aka TWI_ADDRESS_MASTER for MASTER

  volatile uint8_t twi_state;
  volatile uint8_t twi_slarw;      // for master, the slave address + R/~W bit that I'm working with
  volatile uint8_t twi_sendStop;   // should the transaction end with a stop?
  volatile uint8_t twi_error;

  union // use a union to consolidate size a bit.  can't be BOTH master AND slave, right?
  {
    struct
    {
      // master stuff
      uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
      volatile uint8_t twi_masterBufferIndex;
      volatile uint8_t twi_masterBufferLength;
    };

    struct
    {
      // slave stuff
      uint8_t twi_txBuffer[TWI_BUFFER_LENGTH]; // this matches 'twi_masterBuffer' offset
      volatile uint8_t twi_txBufferIndex;
      volatile uint8_t twi_txBufferLength;

      // extra members not present for 'master'
      uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
      volatile uint8_t twi_rxBufferIndex;

      void (*twi_onSlaveTransmit)(TWI_t *, void *);                // call on 'start' before transmitting
      void (*twi_onSlaveReceive)(TWI_t *, void *, const uint8_t *, int); // call on 'stop' or 're-start' after receive

      void *pTXCtx; // context pointer for twi_onSlaveTransmit
      void *pRXCtx; // context pointer for twi_onSlaveReceive
    };
  };

} TWI_STATE;

#if !defined(TWI_PORT0) && !defined(TWI_PORT1) && !defined(TWI_PORT2) && !defined(TWI_PORT3)
#define TWI_PORT0 DEFAULT_TWI
#define SDA0 SDA
#define SCL0 SCL
#endif // set up for using only default TWI if none of the 4 possible TWI ports are defined

// each TWI port has its own 'port state' variables

#ifdef USE_STATIC_PORT_STATE

#ifdef TWI_PORT0
TWI_STATE port0_state = { TWI_ADDRESS_MASTER, 0, 0, 0, 0};
#endif // TWI_PORT0

#ifdef TWI_PORT1
TWI_STATE port1_state = { TWI_ADDRESS_MASTER, 0, 0, 0, 0};
#endif // TWI_PORT1

#ifdef TWI_PORT2
TWI_STATE port2_state = { TWI_ADDRESS_MASTER, 0, 0, 0, 0};
#endif // TWI_PORT2

#ifdef TWI_PORT3
TWI_STATE port3_state = { TWI_ADDRESS_MASTER, 0, 0, 0, 0};
#endif // TWI_PORT3


static TWI_STATE *GetTWISTATE(TWI_t *pTWI)
{
#ifdef TWI_PORT0
  if(pTWI == &(TWI_PORT0))
  {
    return &port0_state;
  }
#endif // TWI_PORT0
#ifdef TWI_PORT1
  if(pTWI == &(TWI_PORT1))
  {
    return &port1_state;
  }
#endif // TWI_PORT1
#ifdef TWI_PORT2
  if(pTWI == &(TWI_PORT2))
  {
    return &port2_state;
  }
#endif // TWI_PORT2
#ifdef TWI_PORT3
  if(pTWI == &(TWI_PORT3))
  {
    return &port3_state;
  }
#endif // TWI_PORT0

  return NULL;
}

#else // !USE_STATIC_PORT_STATE

#ifdef TWI_PORT0
TWI_STATE *port0_state = NULL; // this will be allocated as-needed by 'malloc()'
#endif // TWI_PORT0

#ifdef TWI_PORT1
TWI_STATE *port1_state = NULL;
#endif // TWI_PORT1

#ifdef TWI_PORT2
TWI_STATE *port2_state = NULL;
#endif // TWI_PORT2

#ifdef TWI_PORT3
TWI_STATE *port3_state = NULL;
#endif // TWI_PORT3

static TWI_STATE *ConstructPortState(void)
{
TWI_STATE *pRval;

  pRval = (TWI_STATE *)malloc(sizeof(*pRval));

  if(pRval)
  {
    memset(pRval, 0, sizeof(*pRval));
    pRval->twi_address = TWI_ADDRESS_MASTER;
  }

  return pRval;
}

static TWI_STATE *GetTWISTATE(TWI_t *pTWI)
{
#ifdef TWI_PORT0
  if(pTWI == &(TWI_PORT0))
  {
    if(!port0_state)
    {
      port0_state = ConstructPortState();
    }

    return port0_state;
  }
#endif // TWI_PORT0
#ifdef TWI_PORT1
  if(pTWI == &(TWI_PORT1))
  {
    if(!port1_state)
    {
      port1_state = ConstructPortState();
    }

    return port1_state;
  }
#endif // TWI_PORT1
#ifdef TWI_PORT2
  if(pTWI == &(TWI_PORT2))
  {
    if(!port2_state)
    {
      port2_state = ConstructPortState();
    }

    return port2_state;
  }
#endif // TWI_PORT2
#ifdef TWI_PORT3
  if(pTWI == &(TWI_PORT3))
  {
    if(!port3_state)
    {
      port3_state = ConstructPortState();
    }

    return port3_state;
  }
#endif // TWI_PORT0

  return NULL;
}

#endif // USE_STATIC_PORT_STATE


static __inline char IsMaster(TWI_STATE *pS)
{
  return pS ? pS->twi_address == TWI_ADDRESS_MASTER : 0; // TWI_ADDRESS_MASTER marks this as 'master mode'
}


/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 *
 * for MASTER operation, do not call twi_setAddress() beforehand
 * for SLAVE operation, call twi_setAddress() with the slave address
 * NOTE:  this is the way the original library was designed.
 */
void twi_init(TWI_t *pTWI)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
char iSDA, iSCL;
uint8_t oldSREG, oldADDR;
void *wTemp, *wTemp2;


  if(!pS)
  {
    return;
  }

  // set up pins

  iSDA = SDA;
  iSCL = SCL;
#ifdef TWI_PORT0
  if(pTWI == &(TWI_PORT0))
  {
    iSDA = SDA0;
    iSCL = SCL0;
  }
#endif // TWI_PORT0
#ifdef TWI_PORT1
  if(pTWI == &(TWI_PORT1))
  {
    iSDA = SDA1;
    iSCL = SCL1;
  }
#endif // TWI_PORT1
#ifdef TWI_PORT2
  if(pTWI == &(TWI_PORT2))
  {
    iSDA = SDA2;
    iSCL = SCL2;
  }
#endif // TWI_PORT2
#ifdef TWI_PORT3
  if(pTWI == &(TWI_PORT3))
  {
    iSDA = SDA3;
    iSCL = SCL3;
  }
#endif // TWI_PORT0

  // TODO:  determine if using internal pullup resistors will in any way work
  //        in order to limit the need for external components
  pinMode(iSDA, INPUT);
  pinMode(iSCL, INPUT);


  // disable interrupts
  oldSREG = SREG;
  cli();

  oldADDR = pS->twi_address; // preserve in case twi_address was called before
  wTemp = pS->twi_onSlaveTransmit;
  wTemp2 = pS->twi_onSlaveReceive;
  memset(pS, 0, sizeof(*pS));
  pS->twi_address = oldADDR; // restore it
  pS->twi_onSlaveTransmit = wTemp;
  pS->twi_onSlaveReceive = wTemp2;


  pS->twi_state = TWI_READY;
  pS->twi_sendStop = true;    // default value

  // initialize TWI
  pTWI->CTRL = 0x2; // normal 2 wire, 50ns delay      0;  // normal 2-wire, no delays

  pTWI->SLAVE.CTRLA = 0; // disable ints, slave mode
  pTWI->SLAVE.CTRLB = 0; // no command, send 'ack' for smart ack
  pTWI->SLAVE.STATUS = TWI_SLAVE_DIF_bm | TWI_SLAVE_APIF_bm // clear interrupt/status bits
                     | TWI_SLAVE_RXACK_bm | TWI_SLAVE_COLL_bm | TWI_SLAVE_BUSERR_bm;

  pTWI->MASTER.CTRLA = 0; // disable ints, master mode
  pTWI->MASTER.CTRLB = 0; // disable quick command, disable 'smart ack'
  pTWI->MASTER.CTRLC = 0; // 'ack' behavior, no command
  pTWI->MASTER.STATUS = TWI_MASTER_RIF_bm | TWI_MASTER_WIF_bm // clear interrupt/status bits (shuts off SCL/SDA holds, etc.)
                      | TWI_MASTER_RXACK_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm;

  if((((long)F_CPU) / (2L * TWI_FREQ) - 5) > 255L)
  {
    pTWI->MASTER.BAUD = 0xff; // slowest rate (around 64khz?)
  }
  else
  {
    // baud rate for TWI MASTER, from AU manual 21.9.5, D manual 17.9.5, and others
    pTWI->MASTER.BAUD = (uint8_t)(((long)F_CPU) / (2L * TWI_FREQ) - 5);
  }

  // enable twi module, acks, and twi interrupt

  // if I am in SLAVE mode, initialize the slave address and set up slave interrupts
  // otherwise set up the master interrupts

  if(!IsMaster(pS)) // slave mode
  {
    // set twi slave address (skip over 'general call enable' bit)
    // AU manual 21.10.4, D manual 17.10.4
    pTWI->SLAVE.ADDR = pS->twi_address << 1;
    pTWI->SLAVE.ADDRMASK = 0; // disable address mask (only respond to ADDR)

    // enable interface as slave and interrupts - AU manual 21.10.1
    pTWI->SLAVE.CTRLA = TWI_SLAVE_INTLVL_HI_gc // high priority interrupt level
                      | TWI_SLAVE_DIEN_bm      // enable data interrupt
                      | TWI_SLAVE_APIEN_bm     // enable address/stop 'APIF' interrupt
                      | TWI_SLAVE_ENABLE_bm    // enable interface in 'slave' mode
#ifdef SLAVE_SMART_ACK
                      | TWI_SLAVE_SMEN_bm;     // enable 'SMART' acks (see CTRLB TWI_SLAVE_ACKACT_bm)
#endif // SLAVE_SMART_ACK
//                      | TWI_SLAVE_PMEN_bm      // temporarily enable promiscuous mode
                      | TWI_SLAVE_PIEN_bm;     // APIF interrupt for STOP condition
  }
  else
  {
    pTWI->MASTER.STATUS = 0; // force status to '0' initially (probably won't do anything)

    pTWI->MASTER.CTRLB = 0 // disable the bus timeout - normal for I2C
#ifdef MASTER_SMART_ACK
                         | TWI_MASTER_SMEN_bm     // enable 'SMART' acks (auto-master-ack after read DATA using 'ACKACT' bit)
#endif // MASTER_SMART_ACK
#ifdef MASTER_QUICK_COMMAND
                         | TWI_MASTER_QCEN_bm     // enable 'quick command' (auto-int right after slave ack)
#endif // MASTER_QUICK_COMMAND
                         ;

    pTWI->MASTER.CTRLA = TWI_MASTER_INTLVL_HI_gc // high priority interrupt
                       | TWI_MASTER_RIEN_bm      // enable read data interrupt
                       | TWI_MASTER_WIEN_bm      // enable write data interrupt
                       | TWI_MASTER_ENABLE_bm;    // enable TWI master

    // force bus state to 'idle' _NOW_ - this should make things happen
    pTWI->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc; // force bus state to 'idle'

    pS->twi_address = TWI_ADDRESS_MASTER;  // slave address - FFH for MASTER [always]
  }

  SREG = oldSREG; // restore int flag and exit
}

// Function:  twi_shutdown(void)
// Desc:      shuts down TWI interface, re-initializes to startup state
//
void twi_shutdown(TWI_t *pTWI)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t oldSREG;


  if(!pS)
  {
    return;
  }

  oldSREG = SREG;
  cli(); // ints off until after this part

  // shut down TWI, re-initialize it
  pTWI->SLAVE.CTRLA = 0; // disable ints, slave mode
  pTWI->MASTER.CTRLA = 0; // disable ints, master mode

  pTWI->SLAVE.CTRLB = 0; // no command, send 'ack' for smart ack
  pTWI->SLAVE.STATUS = TWI_SLAVE_DIF_bm | TWI_SLAVE_APIF_bm // clear interrupt/status bits
                     | TWI_SLAVE_RXACK_bm | TWI_SLAVE_COLL_bm | TWI_SLAVE_BUSERR_bm;

  pTWI->MASTER.CTRLB = 0; // disable quick command, disable 'smart ack'
  pTWI->MASTER.CTRLC = 0; // 'ack' behavior, no command
  pTWI->MASTER.STATUS = TWI_MASTER_RIF_bm | TWI_MASTER_WIF_bm // clear interrupt/status bits (shuts off SCL/SDA holds, etc.)
                      | TWI_MASTER_RXACK_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm;

  SREG = oldSREG; // restore int flag


  // re-initialize all global variables. The API requires that the address be set for 'MASTER'

  pS->twi_address = TWI_ADDRESS_MASTER;  // slave address - FFH for MASTER
  pS->twi_state = 0;
  pS->twi_slarw = 0;
  pS->twi_sendStop = 0;      // should the transaction end with a stop

  pS->twi_onSlaveTransmit = (void *)0;
  pS->twi_onSlaveReceive = (void *)0;
  pS->pTXCtx = NULL;
  pS->pRXCtx = NULL;

  pS->twi_masterBufferIndex = 0;
  pS->twi_masterBufferLength = 0;

  pS->twi_txBufferIndex = 0;
  pS->twi_txBufferLength = 0;

  pS->twi_rxBufferIndex = 0;

  pS->twi_error = 0;
}


/* 
 * Function twi_setAddress
 * Desc     sets slave address (call before twi_init() for slave mode)
 * Input    none
 * Output   none
 */
void twi_setAddress(TWI_t *pTWI, uint8_t address)
{
TWI_STATE *pS = GetTWISTATE(pTWI);

  if(!pS)
  {
    return;
  }

  pS->twi_address = address & 0x7f; // make sure I filter the high bit

  // in case I'm already running...
  // set twi slave address (skip over 'general call enable' bit)
  // AU manual 21.10.4, D manual 17.10.4

  pTWI->SLAVE.ADDR = address << 1;
  pTWI->SLAVE.ADDRMASK = 0; // disable address mask (only respond to ADDR)
}

/* 
 * Function twi_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 *          wait: number of milliseconds to wait (maximum), 0 == infinite
 * Output   number of bytes read
 *
 * NOTE: this function BLOCKS until complete.  TODO, a timeout and a 0 length returned on timeout
 *       also do NOT call this function with interrupts disabled or it WILL hang!
 */
uint8_t twi_readFrom(TWI_t *pTWI, uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop, uint16_t wait_timeout)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t i1;
uint8_t oldSREG;
uint16_t wMillis;


  if(!pS)
  {
    return 0;
  }

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length)
  {
    return 0;
  }

  // NOTE:  this will infinite loop if interrupts disabled
  // sei(); // TODO uncomment if needed

  wMillis = (uint16_t)millis();

  // wait until twi is ready, become master receiver
  while(TWI_READY != pS->twi_state) // TODO:  check 
  {
    if(wait_timeout && wait_timeout < ((uint16_t)millis() - wMillis))
    {
#ifdef DEBUG
      error_print("read READY timeout"); // temporary
#endif // DEBUG

      return 0; // timeout, no bytes read
    }

    wait_for_interrupt(); // wait for interrupt to occur [most efficient]
  }

  memset(pS->twi_masterBuffer, 0, sizeof(pS->twi_masterBuffer));

  // build sla+w, slave device address + w bit
  pS->twi_slarw = TWI_ADDR_READ; /* sets the 'read/~write' bit in the 8 bit address */
  pS->twi_slarw |= address << 1; /* the other 7 bits [address left-shifted by 1] */


  oldSREG = SREG;
  cli(); // ints off until after this part

  pS->twi_state = TWI_MRX;
  pS->twi_sendStop = 1; // always [just in case]

  // reset error state (0xFF.. no error occured)
  pS->twi_error = 0xFF;

  // initialize buffer iteration vars
  pS->twi_masterBufferIndex = 0;
  pS->twi_masterBufferLength = length;

  pTWI->MASTER.ADDR = pS->twi_slarw; // assigning address does a 'start'

  SREG = oldSREG; // ints re-enabled (if they were on before)
  // sei(); // make sure ints on [uncomment if needed]

  wMillis = (uint16_t)millis();

  // wait for read operation to complete
  while(TWI_MRX == pS->twi_state &&
        (pS->twi_masterBufferIndex < pS->twi_masterBufferLength ||
         pS->twi_sendStop))
  {
    if(wait_timeout && wait_timeout < ((uint16_t)millis() - wMillis))
    {
#ifdef DEBUG
      error_print("read timeout"); // temporary
#endif // DEBUG
      break; // stop waiting
    }

    wait_for_interrupt(); // wait for interrupt to occur [most efficient]
  }

  if(pS->twi_masterBufferIndex < length)
  {
    length = pS->twi_masterBufferIndex;
  }

  // copy twi buffer to data
  for(i1 = 0; i1 < length; i1++)
  {
    data[i1] = pS->twi_masterBuffer[i1];
  }

  return length;
}

/* 
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          sendStop: boolean indicating whether or not to send a stop at the end
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
uint8_t twi_writeTo(TWI_t *pTWI, uint8_t address, const uint8_t * data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t i;
uint8_t oldSREG;
uint16_t wMillis;


  if(!pS)
  {
    return 1;
  }

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length)
  {
    return 1;
  }

  // wait until twi is ready, become master transmitter

  wMillis = (uint16_t)millis();

  while(TWI_READY != pS->twi_state)
  {
    if(((uint16_t)millis() - wMillis) < 10) // 10 msecs to respond
    {
      wait_for_interrupt(); // wait for interrupt to occur [most efficient]
    }
    else
    {
      twi_stop(pTWI); // sets state to 'ready'

#ifdef DEBUG
      error_print("write READY timeout");
#endif // DEBUG
      return -2; // timeout
    }      
  }

  // build sla+w, slave device address + w bit
  pS->twi_slarw = TWI_ADDR_WRITE; /* sets the 'write' bit in the 8 bit address */
  pS->twi_slarw |= address << 1; /* the other 7 bits [address left-shifted by 1] */


  oldSREG = SREG;
  cli(); // ints off until after this part

  pS->twi_state = TWI_MTX;  // says "I am in TX mode"
  pS->twi_sendStop = sendStop;

  // reset error state (0xFF.. no error occured)
  pS->twi_error = 0xFF;

  // initialize buffer iteration vars
  pS->twi_masterBufferIndex = 0;
  pS->twi_masterBufferLength = length;
  
  // copy data to twi buffer
  for(i = 0; i < length; ++i)
  {
    pS->twi_masterBuffer[i] = data[i];
  }

  // NOTE:  errata on 'E' series says that if you write data THEN address, the
  //        data will be 'sent' immediately after writing the address, but it will be 00H
  //        but in THIS case I want to send the data byte correctly, so 'address first'
  //        Also this re-starts a transaction if one already being done

  pTWI->MASTER.CTRLC = 0; // no command, ~ACK bit clear (probably needed)
  pTWI->MASTER.ADDR = pS->twi_slarw; // this should start it

  SREG = oldSREG; // ints re-enabled (if they were on before)


  if(wait)
  {
    wMillis = (uint16_t)millis();

    while(TWI_MTX == pS->twi_state)
    {
      if(((uint16_t)millis() - wMillis) < 5) // 5 msecs to respond
      {
        wait_for_interrupt(); // wait for interrupt to occur [most efficient]
      }
      else
      {
        twi_stop(pTWI); // sets state to 'ready'

        if(pS->twi_error != 0xff)
        {
          goto error_return;
        }

#ifdef DEBUG
        error_print("write MTX timeout");
#endif // DEBUG

        return -1; // timeout
      }      
    }
  }
  
error_return:

  if (pS->twi_error == 0xFF)
  {
    return 0;  // success
  }
  else if (pS->twi_error == TWI_ERROR_SLAVE_NACK)
  {
    return 2;  // error: address send, nack received
  }
  else if (pS->twi_error == TWI_ERROR_DATA_NACK)
  {
    return 3;  // error: data send, nack received
  }
  else
  {
#ifdef DEBUG
    error_print_("twi_writeTo error=");
    error_printL((uint8_t)(pS->twi_error) & 0xff);
#endif // DEBUG

    return 4;  // other twi error
  }
}

/* 
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t twi_transmit(TWI_t *pTWI, const uint8_t* data, uint8_t length)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t i1;


  if(!pS)
  {
    return 1;
  }

  pS->twi_txBufferLength = 0; // just in case, clear first
  
  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length)
  {
    return 1;
  }
  
  // ensure we are currently a slave transmitter
  if(TWI_STX != pS->twi_state)
  {
    return 2;
  }
  
  // set length and copy data into tx buffer
  pS->twi_txBufferLength = length;
  for(i1 = 0; i1 < length; ++i1)
  {
    pS->twi_txBuffer[i1] = data[i1];
  }
  
  return 0;
}

/* 
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveRxEvent(TWI_t * pTWI, void (*function)(TWI_t *, void *, const uint8_t *, int), void *pCtx )
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t oldSREG;

  if(!pS)
  {
    return;
  }

  oldSREG = SREG;
  cli(); // turn of interrupts to prevent accidentally sending wrong pRXCtx

  pS->twi_onSlaveReceive = function;
  pS->pRXCtx = pCtx; // serialized to ensure consistency

  SREG = oldSREG; // restore int flag, basically
}

/* 
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveTxEvent(TWI_t *pTWI, void (*function)(TWI_t *, void *), void *pCtx )
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t oldSREG;

  if(!pS)
  {
    return;
  }

  oldSREG = SREG;
  cli(); // turn of interrupts to prevent accidentally sending wrong pTXCtx

  pS->twi_onSlaveTransmit = function;
  pS->pTXCtx = pCtx; // serialized to ensure consistency

  SREG = oldSREG; // restore int flag, basically
}


/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop(TWI_t *pTWI)
{
TWI_STATE *pS = GetTWISTATE(pTWI);

  if(!pS)
  {
    return;
  }

  if(!IsMaster(pS))
  {
    // should never happen (do nothing)
    return;
  }

  // note this only sends a stop with SUCCESS, and that only matters when reading;
  // to send a stop with NACK, use TWI_MASTER_CMD_STOP_gc | TWI_MASTER_ACKACT_bm


  pTWI->MASTER.CTRLC = 0;
  pTWI->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc; // 3 == STOP

  uint16_t wTemp = 0;
  uint8_t bTemp2 = 0;

  while(1)
  {
    uint8_t bTemp = pTWI->MASTER.STATUS & TWI_MASTER_BUSSTATE_gm;

    if(bTemp == TWI_MASTER_BUSSTATE_IDLE_gc)
    {
      break; // idle bus state - I am done waiting
    }

    if(bTemp != TWI_MASTER_BUSSTATE_OWNER_gc &&
       bTemp != TWI_MASTER_BUSSTATE_BUSY_gc) // neither BUSY nor pWNED - 17.9.4
    {
#ifdef DEBUG
      error_print_("twi_stop - unknown state: ");
      error_printL(bTemp);
#endif // DEBUG

      break;
    }

    if(++wTemp == 0) // 65536 loops, about 65msec?
    {
      bTemp2++;

      if(bTemp2 > 16) // done this 16 times?
      {
        // force bus state to 'idle' _NOW_
        pTWI->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc; // force bus state to 'idle'

#ifdef DEBUG
        error_print("BUS STATE RESET");
#endif // DEBUG
        break;
      }

      if((bTemp2 & 3) == 0) // every 4 times
      {
        pTWI->MASTER.CTRLC = 0;
        pTWI->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc; // re-issue the stop
      }
//      wTemp = 0; not needed, but left for reference
    }
  }

  // update twi state (always)
  pS->twi_state = TWI_READY;
}

/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus(TWI_t *pTWI)
{
TWI_STATE *pS = GetTWISTATE(pTWI);

  if(!pS)
  {
    return;
  }

  // release bus
  pTWI->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc; // force bus state to 'idle'

  // update twi state
  pS->twi_state = TWI_READY;
}


// slave
void do_slave_isr(TWI_t *pTWI)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t status;


  if(!pS)
  {
    return;
  }

  status = pTWI->SLAVE.STATUS;

  // type of interrupt:

  if(status & TWI_SLAVE_BUSERR_bm) // bus error
  {
    // TODO:  what do I do for a bus error?  manual says master must enabled to detect this with its state logic
   
    pS->twi_state = TWI_READY; // bus NACK and I'm "off"

    pS->twi_error = TWI_ERROR_BUS_ERROR;

    pTWI->SLAVE.STATUS |= TWI_SLAVE_BUSERR_bm; // clear the bit

#ifdef DEBUG
    error_print("BUS error");
#endif // DEBUG
  }
  else if(status & TWI_SLAVE_COLL_bm) // slave collision
  {
    // manual says this is cleared by writing a '1', or by a START transaction from the master

    pS->twi_state = TWI_READY; // bus NACK and I'm "off"

    pS->twi_error = TWI_ERROR_COLLISION;

    pTWI->SLAVE.STATUS |= TWI_SLAVE_COLL_bm; // clear the bit

#ifdef DEBUG
    error_print("COLLISION error");
#endif // DEBUG
  }
  else
  {
    if(status & TWI_SLAVE_APIF_bm) // address or stop
    {
      // write a '1' to this bit to clear it, or execute a command (like for DIF)
      // SCL remains 'stretched' until I clear this bit (for address only)
      // if PIEN bit is set in CTRLA, STOP will also set APIF
      // for that reason I can check for it here


      // regardless, if I'm currently in 'read' mode, this terminates
      // the read process and forces a read callback, either a 'STOP' or
      // a 're-START' transaction in this case.

      if(pS->twi_state == TWI_SRX) // slave read
      {
        if(pS->twi_onSlaveReceive)
        {
          pS->twi_onSlaveReceive(pTWI, pS->pRXCtx, pS->twi_rxBuffer, pS->twi_rxBufferIndex);
        }

        pS->twi_rxBufferIndex = 0;
      }

      pS->twi_state = TWI_READY; // pre-condition

      if(status & TWI_SLAVE_AP_bm) // it's an address (table 21-9 in AU manual)
      {
        // NOTE:  it's either a START or a re-START.

        if(status & TWI_SLAVE_DIR_bm) // read (from ME)
        {
          pS->twi_state = TWI_STX;
          pS->twi_txBufferIndex = pS->twi_txBufferLength = 0; // initialize (TODO: only if onSlaveTransmit?)

          // request for txBuffer to be filled and length to be set
          // note: user must call twi_transmit(bytes, length) to do this
          if(pS->twi_onSlaveTransmit)
          {
            pS->twi_onSlaveTransmit(pTWI, pS->pRXCtx);
          }

          // if they didn't change buffer & length, initialize it [old lib behavior did this]
          if(0 == pS->twi_txBufferLength)
          {
            pS->twi_txBufferLength = 1;
            pS->twi_txBuffer[0] = 0x00;
          }

          // this next part acknowledges the action
          pTWI->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; // ack it
        }
        else // master writing to ME
        {
          pS->twi_state = TWI_SRX;

          // this next part acknowledges the action by receiving the next byte

          pTWI->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; // ack it
        }
      }
      else // it's a STOP
      {
        // TODO:  anything else ??

//        pTWI->SLAVE.CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc; // ack 'complete' and wait for START sequence
      }
    }

    if(status & TWI_SLAVE_DIF_bm) // data interrupt
    {
      // DIF bit will clear if I write '1' to it, or if I execute a command
      // SCL remains 'stretched' until I clear this bit.

      // read or write?
      if(status & TWI_SLAVE_DIR_bm) // read (me transmit)
      {
        // send next byte
        if(pS->twi_txBufferIndex < pS->twi_txBufferLength)
        {
          pTWI->SLAVE.DATA = pS->twi_txBuffer[pS->twi_txBufferIndex++];
        }
        else
        {
          pTWI->SLAVE.CTRLB = TWI_SLAVE_ACKACT_bm | TWI_SLAVE_CMD_COMPTRANS_gc; // NACK 'complete' and wait for START sequence

#if 0 /* if I want to 'send more data' I _could_ do it THIS way */
          if(pS->twi_onSlaveTransmit)
          {
            pS->twi_txBufferIndex = pS->twi_txBufferLength = 0; // re-initialize
            pS->twi_onSlaveTransmit(pTWI, pS->pRXCtx);
          }

          // if they didn't change buffer & length, initialize it [old lib behavior did this]
          if(0 == pS->twi_txBufferLength)
          {
            pTWI->SLAVE.DATA = 0xff; // "no data" value
          }
          else
          {
            pTWI->SLAVE.DATA = pS->twi_txBuffer[pS->twi_txBufferIndex++];
          }
#endif // 0          
        }

        pTWI->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; // send an ACK (as in "I have data for ya, enjoy")
      }
      else // write
      {
        // get next byte
        if(pS->twi_rxBufferIndex < sizeof(pS->twi_rxBuffer))
        {
          pS->twi_rxBuffer[pS->twi_rxBufferIndex++] = pTWI->SLAVE.DATA;
          pTWI->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc; // send an ACK (as in 'thank you sir, may I have another?')
        }
        else
        {
//          pTWI->SLAVE.CTRLB = TWI_SLAVE_ACKACT_bm | TWI_SLAVE_CMD_RESPONSE_gc; // send a NACK (i.e. 'no more please, I'm full now')

          pTWI->SLAVE.CTRLB = TWI_SLAVE_ACKACT_bm | TWI_SLAVE_CMD_COMPTRANS_gc; // NACK 'complete' and wait for START sequence
        }
      }

      pTWI->SLAVE.STATUS = TWI_SLAVE_DIF_bm; // clear the bit
    }

    // NOTE:  I *could* end up with another start/stop interrupt while doing this...
  }

}

#ifdef TWI_VECTOR_S0
ISR(TWI_VECTOR_S0)
{
  do_slave_isr(&(TWI_PORT0));
}
#endif // TWI_VECTOR_S0
#ifdef TWI_VECTOR_S1
ISR(TWI_VECTOR_S1)
{
  do_slave_isr(&(TWI_PORT1));
}
#endif // TWI_VECTOR_S1
#ifdef TWI_VECTOR_S2
ISR(TWI_VECTOR_S2)
{
  do_slave_isr(&(TWI_PORT2));
}
#endif // TWI_VECTOR_S2
#ifdef TWI_VECTOR_S3
ISR(TWI_VECTOR_S3)
{
  do_slave_isr(&(TWI_PORT3));
}
#endif // TWI_VECTOR_S3


// master
void do_master_isr(TWI_t *pTWI)
{
TWI_STATE *pS = GetTWISTATE(pTWI);
uint8_t stat;


  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);

  if(!pS)
  {
    pS->twi_error = 0x7f;

    return;
  }

  cli(); // make sure

  // in section 17.5 of the 'D' manual, the process is described.  There are several
  // conditions I need to test for, via interrupts:
  // a) write interrupt+hold [M3]
  // b) read interrupt+hold [M4]
  // c) bus idle [M1?]

  // 4 possible conditions in diagram:
  // M1 - arbitration lost or bus error
  // M2 - address transmitted, NOT acknowledged by slave
  // M3 - address transmitted and acknowledged (write) [write interrupt + hold generated]
  // M4 - address transmitted and acknowledged (read) [read interrupt + hold generated]

  // sequence is like this:
  // a) software waits for idle
  // b) set address
  // c) start+address is sent
  // d) read/write interrupt+hold [ISR sends/receives next byte] - TWI_MASTER_CLKHOLD_bm
  // e) repeat d as needed [detect 'stop' for read]
  // f) STOP

  // STATUS register
  // when writing, the slave COULD 'NACK' the result.  See section 17.9.4 'STATUS' register
  // Also, if the STATUS register bits 6 or 7 are set, must clear by either reading or writing DATA,
  // by issuing a command via CMD, or writing the ADDR register [or a '1' to that bit in the STATUS reg]

  // Additionally, the 'arbitration lost' bit needs to be looked at [if SDA goes to zero while writing a '1' bit]
  // re-writing the ADDR clears the 'arbitration lost' bit [or writing a '1' to that bit in the STATUS reg]

  // BUSERR is similar (out of sequence start/stop or wrong bit count) and cleared by writing ADDR
  // or writing a '1' to that bit in the STATUS reg.

  // status bits (low 2 bits):  00 = unknown, 01 = idle, 02 = OWNER, 03 = BUSY  [write '01' to force IDLE state]

  stat = pTWI->MASTER.STATUS;

  if(stat & TWI_MASTER_ARBLOST_bm) // arbitration lost
  {
    pTWI->MASTER.STATUS = TWI_MASTER_ARBLOST_bm; // to clear the bit
    pS->twi_error = TWI_ERROR_ARB_LOST;
    twi_stop(pTWI);

    pS->twi_state = TWI_READY;
#ifdef DEBUG
    error_print("arb lost"); // temporary, do this
#endif // DEBUG

    pTWI->MASTER.STATUS = TWI_ERROR_ARB_LOST; // clear the bit
  }
  else if(stat & TWI_MASTER_BUSERR_bm) // bus error
  {
    pTWI->MASTER.STATUS = TWI_MASTER_BUSERR_bm; // to clear the bit
    pS->twi_error = TWI_ERROR_BUS_ERROR;
    twi_stop(pTWI);

    pS->twi_state = TWI_READY;
#ifdef DEBUG
    error_print("bus error"); // temporary, do this
#endif // DEBUG

    pTWI->MASTER.STATUS = TWI_ERROR_BUS_ERROR; // clear the bit
  }
  else if((pTWI->MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_OWNER_gc)
  {
    // TODO:  check for TWI_ERROR_SLAVE_NACK and TWI_ERROR_DATA_NACK conditions
    //        and assign twi_error appropriately

    if(stat & TWI_MASTER_RIF_bm) // read interrupt
    {
      pTWI->MASTER.STATUS = TWI_MASTER_CLKHOLD_bm | TWI_MASTER_RIF_bm; // clear hold bit NOW

      // add data to input buffer
      if(stat & TWI_MASTER_RXACK_bm) // slave sent RX 'NACK' indicating 'end of data'
      {
#ifdef MASTER_SMART_ACK
        register uint8_t bTemp;

        pTWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm; // no command, ~ACK bit set

        bTemp = pTWI->MASTER.DATA; // read it anyway...

        pS->twi_state = TWI_MRX; // not 'STOP', actually (TODO:  make it 'stop' ?)
#else // MASTER_SMART_ACK

        pTWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_RECVTRANS_gc; // sends a NACK first

//        pS->twi_state = TWI_MRX; // not 'STOP', actually (TODO:  make it 'stop' ?)
        if(pS->twi_sendStop)
        {
          twi_stop(pTWI);
        }

        pS->twi_state = TWI_READY; // indicate that the data has stopped flowing and move on
#endif // MASTER_SMART_ACK
      }
      else
      {
        if(pS->twi_masterBufferIndex < sizeof(pS->twi_masterBuffer))
        {
#ifdef MASTER_SMART_ACK
          if(pS->twi_masterBufferIndex < (pS->twi_masterBufferLength - 1))
          {
            pTWI->MASTER.CTRLC = 0; // no command, ~ACK bit clear (send more!)
          }
          else
          {
            pTWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm; // no command, ~ACK bit set (don't send more)
          }
#endif // MASTER_SMART_ACK

          pS->twi_masterBuffer[pS->twi_masterBufferIndex++] = pTWI->MASTER.DATA;

#ifndef MASTER_SMART_ACK
          if(pS->twi_masterBufferIndex < pS->twi_masterBufferLength)
          {
            pTWI->MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc; // ~ACK bit is ZERO (send more!)
          }
          else
          {
            pTWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_RECVTRANS_gc; // sends a NACK (STOP sending)
          }
#endif // MASTER_SMART_ACK
        }
        else
        {
#ifdef MASTER_SMART_ACK
          register uint8_t bTemp;

          pTWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm; // no command, ~ACK bit set

          bTemp = pTWI->MASTER.DATA; // read it anyway...

#else // MASTER_SMART_ACK

          pTWI->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_RECVTRANS_gc; // sends a NACK first (do NOT send more!)

#endif // MASTER_SMART_ACK
        }

        if(pS->twi_masterBufferIndex >= pS->twi_masterBufferLength)
        {
          // send a STOP
          if(pS->twi_sendStop)
          {
            twi_stop(pTWI);

            pS->twi_state = TWI_READY;
          }

          // TODO:  do something special?  probably not...
        }
      }
    }

    if(stat & TWI_MASTER_WIF_bm) // write interrupt
    {
      pTWI->MASTER.STATUS = TWI_MASTER_CLKHOLD_bm | TWI_MASTER_WIF_bm; // clear bits

      if(!(stat & TWI_MASTER_RXACK_bm) // received acknowledged (flag is clear)
         || pS->twi_masterBufferIndex >= pS->twi_masterBufferLength)
      {
        if(pS->twi_masterBufferIndex < pS->twi_masterBufferLength)
        {
          // copy data to output register and ack
          pTWI->MASTER.DATA = pS->twi_masterBuffer[pS->twi_masterBufferIndex++];
        }

        if(pS->twi_masterBufferIndex >= pS->twi_masterBufferLength)
        {
          if(pS->twi_sendStop)
          {
            twi_stop(pTWI);
          }

          pS->twi_state = TWI_READY;
        }
      }
      else // 'NACK' (stop sending)
      {
        pS->twi_error = TWI_ERROR_SLAVE_NACK;

        twi_stop(pTWI); // always

        pS->twi_state = TWI_READY;
      }
    }
  }

  stat = pTWI->MASTER.STATUS;

  if(stat & TWI_MASTER_CLKHOLD_bm) // is there a clock hold active?
  {
    pTWI->MASTER.STATUS = (stat & ~TWI_MASTER_BUSSTATE_gm) // clear bus state bits - writing 0 has no effect
                        | TWI_MASTER_WIF_bm | TWI_MASTER_RIF_bm; // clear all of these bits by writing '1'

#ifdef DEBUG
    error_print_("clock hold detected [should not happen] value=");
    error_printL(stat);
#endif // DEBUG
  }

  sei(); // allow ints again (in case it doesn't)
}

#ifdef TWI_VECTOR_M0
ISR(TWI_VECTOR_M0)
{
  do_master_isr(&(TWI_PORT0));
}
#endif // TWI_VECTOR_M0
#ifdef TWI_VECTOR_M1
ISR(TWI_VECTOR_M1)
{
  do_master_isr(&(TWI_PORT1));
}
#endif // TWI_VECTOR_M1
#ifdef TWI_VECTOR_M2
ISR(TWI_VECTOR_M2)
{
  do_master_isr(&(TWI_PORT2));
}
#endif // TWI_VECTOR_M2
#ifdef TWI_VECTOR_M3
ISR(TWI_VECTOR_M3)
{
  do_master_isr(&(TWI_PORT3));
}
#endif // TWI_VECTOR_M3


#if 0
void old_isr(void /*ISR(TWI_VECT)*/)
{
  switch(TW_STATUS)
  {
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR = twi_slarw;
      twi_reply(1);
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      // if there is data to send, send it, otherwise stop 
      if(twi_masterBufferIndex < twi_masterBufferLength)
      {
        // copy data to output register and ack
        TWDR = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply(1);
      }
      else
      {
        if (twi_sendStop)
        {
          twi_stop();
        }
        else
        {
          twi_inRepStart = true;  // we're gonna send the START
          // don't enable the interrupt. We'll generate the start, but we 
          // avoid handling the interrupt until we're in the next transaction,
          // at the point where we would normally issue the start.
          TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
          twi_state = TWI_READY;
        }
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength)
      {
        twi_reply(1);
      }
      else
      {
        twi_reply(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
      if (twi_sendStop)
        twi_stop();
      else
      {
        twi_inRepStart = true;  // we're gonna send the START
        // don't enable the interrupt. We'll generate the start, but we 
        // avoid handling the interrupt until we're in the next transaction,
        // at the point where we would normally issue the start.
        TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
        twi_state = TWI_READY;
      }    
      break;
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      twi_state = TWI_SRX;
      // indicate that rx buffer can be overwritten and ack
      twi_rxBufferIndex = 0;
      twi_reply(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH)
      {
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        twi_reply(1);
      }
      else
      {
        // otherwise nack
        twi_reply(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // put a null char after data if there's room
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH)
      {
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // sends ack and stops interface for clock stretching
      twi_stop();
      // callback to user defined callback
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // since we submit rx buffer to "wire" library, we can reset it
      twi_rxBufferIndex = 0;
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply(0);
      break;
    
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      // enter slave transmitter mode
      twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      twi_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twi_txBufferLength = 0;
      // request for txBuffer to be filled and length to be set
      // note: user must call twi_transmit(bytes, length) to do this
      twi_onSlaveTransmit();
      // if they didn't change buffer & length, initialize it
      if(0 == twi_txBufferLength)
      {
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // transmit first byte from buffer, fall
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // copy data to output register
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength)
      {
        twi_reply(1);
      }
      else
      {
        twi_reply(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply(1);
      // leave slave receiver state
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}
#endif // 0

