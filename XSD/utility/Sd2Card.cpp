/* Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino Sd2Card Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

// adapted for the ATXMega processors by Bob Frazier, S.F.T. Inc. and partially refactored to remove warnings, improve readability


#include <Arduino.h>
#include "Sd2Card.h"
//------------------------------------------------------------------------------
#ifndef __AVR_XMEGA__
#error this file is for XMEGA only
#endif // __AVR_XMEGA__

#ifndef DEFAULT_SPI // if not defined, use SPIC - see Arduino.h (should already be defined)
#warning defining DEFAULT_SPI as SPIC
#define DEFAULT_SPI SPIC
#endif // DEFAULT_SPI

#ifndef SOFTWARE_SPI

#define SPIREG_INTCTRL *((volatile uint8_t *)&((&(DEFAULT_SPI))->INTCTRL))
#define SPIREG_CTRL    *((volatile uint8_t *)&((&(DEFAULT_SPI))->CTRL))
#define SPIREG_DATA    *((volatile uint8_t *)&((&(DEFAULT_SPI))->DATA))
#define SPIREG_STATUS  *((volatile uint8_t *)&((&(DEFAULT_SPI))->STATUS))
#ifdef SPIC_CTRLB
#define SPIREG_CTRLB   *((volatile uint8_t *)&((&(DEFAULT_SPI))->CTRLB))
#endif // SPIC_CTRLB


// functions for hardware SPI

static void spiInit(uint8_t spiRate) /* using the SdFat library definitions this should work */
{
  spiRate = spiRate > 12 ? 6 : spiRate/2;

  SPIREG_INTCTRL = 0; // disable SPI interrupts

  // enable and initialize as SPI master with max clock and mode 0

  SPIREG_CTRL = SPI_ENABLE_bm  // enable SPI
              | SPI_MASTER_bm  // master mode
              | SPI_MODE_0_gc  // mode zero
              | ((spiRate & 1 || spiRate == 6) ? 0 : SPI_CLK2X_bm) // clock 2x bit
              | (spiRate >> 1); // bits 1:0 [matches values used for atmega] see D manual section 18.6.1 table 18-3
  // NOTE:  mode bits 2 and 3 are both zero for mode 0

#ifdef SPIREG_CTRLB
  SPIREG_CTRLB = SPI_SSD_bm; // disables SS pin low causing 'slave mode'
#endif // SPIREG_CTRLB
}

/** Send a byte to the card */
static void spiSend(uint8_t b)
{
register uint8_t tval = SPIREG_STATUS; // read status register [to clear the bit]
volatile short ctr; // temporary

  tval = tval; // so I don't get an 'unused variable' warning

  SPIREG_STATUS = SPI_IF_bm; // this is ALSO supposed to clear the bit

  SPIREG_DATA = b; // accessing the SPIREG_DATA clears the status bit (D manual, 18.6.3)
                 // and it transmits the data on the SPI bus, while receiving a data byte
  ctr = 0;
  while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete
  {
    if(!(++ctr)) // TODO:  remove if not needed - it busts out to prevent infinite loop
      break;
  }

  // received data byte ignored
}
/** Receive a byte from the card */
static  uint8_t spiRec(void)
{
  register uint8_t tval = SPIREG_STATUS; // read status register [to clear the bit]
  volatile short ctr; // temporary

  tval = tval; // so I don't get an 'unused variable' warning

  SPIREG_STATUS = SPI_IF_bm; // this is ALSO supposed to clear the bit
  SPIREG_STATUS = 0; // temporary, for testing

  SPIREG_DATA = 0XFF; // accessing the SPIREG_DATA clears the status bit (D manual, 18.6.3)
                    // and it transmits the FF data on the SPI bus, while receiving a data byte

  ctr = 0;
  while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete
  {
    if(!(++ctr)) // TODO:  remove if not needed - it busts out to prevent infinite loop
      break;
  }

  return SPIREG_DATA; // return the received data byte
}

#ifdef OPTIMIZE_HARDWARE_SPI

static uint8_t spiRec(uint8_t* buf, size_t n)
{
register uint8_t b;
volatile short ctr; // temporary

  if (n-- == 0)
    return 0;

  b = SPIREG_STATUS; // read status register [to clear the bit]
  SPIREG_STATUS = SPI_IF_bm; // this is ALSO supposed to clear the bit

  SPIREG_DATA = 0XFF; // accessing the SPIREG_DATA clears the status bit (D manual, 18.6.3)
                    // and it transmits the FF data on the SPI bus, while receiving a data byte

  if(buf) // can be NULL
  {
    for (size_t i = 0; i < n; i++)
    {
      ctr = 0;
      while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete
      {
        if(!(++ctr)) // TODO:  remove if not needed - it busts out to prevent infinite loop
          break;
      }

      b = SPIREG_DATA;
      SPIREG_DATA = 0XFF; // queue up another
      buf[i] = b;
    }
  }
  else
  {
    for (size_t i = 0; i < n; i++)
    {
      ctr = 0;
      while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete
      {
        if(!(++ctr)) // TODO:  remove if not needed - it busts out to prevent infinite loop
          break;
      }

      b = SPIREG_DATA; // will eat this for NULL buffer
      SPIREG_DATA = 0XFF; // queue up another
    }
  }

  ctr = 0;
  while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete [one last time]
  {
    if(!(++ctr))
      break;
  }

  if(buf) // can be NULL
  {
    buf[n] = SPIREG_DATA;
  }

  return 0;
}

static void spiSend(const uint8_t* buf , size_t n)
{
register uint8_t b;
volatile short ctr; // temporary
register size_t i;

  if (n == 0)
    return;

  b = SPIREG_STATUS;    // read status register [to clear the bit]
  SPIREG_STATUS = SPI_IF_bm; // this is ALSO supposed to clear the bit

  SPIREG_DATA = buf[0]; // accessing the SPIREG_DATA clears the status bit (D manual, 18.6.3)
                      // and it transmits the data on the SPI bus, while receiving a data byte

  if (n > 1)
  {
    b = buf[1];
    i = 2;

    while (1)
    {
      ctr = 0;
      while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete
      {
        if(!(++ctr)) // TODO:  remove if not needed - it busts out to prevent infinite loop
          break;
      }

      SPIREG_DATA = b;

      if (i == n)
        break;

      b = buf[i++];
    }
  }

  ctr = 0;
  while (!(SPIREG_STATUS & SPI_IF_bm))// { } // wait for the bit flag that says it's complete [one last time]
  {
    if(!(++ctr)) // TODO:  remove if not needed - it busts out to prevent infinite loop
      break;
  }
}

#endif // OPTIMIZE_HARDWARE_SPI

#else  // SOFTWARE_SPI

#ifdef OPTIMIZE_HARDWARE_SPI
#error this code only supports OPTIMIZE_HARDWARE_SPI when SOFTWARE_SPI is NOT defined
#endif // OPTIMIZE_HARDWARE_SPI

//------------------------------------------------------------------------------
/** nop to tune soft SPI timing */
#define nop asm volatile ("nop\n\t")
//------------------------------------------------------------------------------
/** Soft SPI receive */
uint8_t spiRec(void)
{
  uint8_t data = 0;
  // no interrupts during byte receive - about 8 us
  cli();
  // output pin high - like sending 0XFF
  fastDigitalWrite(SPI_MOSI_PIN, HIGH);

  for (uint8_t i = 0; i < 8; i++)
  {
    fastDigitalWrite(SPI_SCK_PIN, HIGH);

    // adjust so SCK is nice
    nop;
    nop;

    data <<= 1;

    if (fastDigitalRead(SPI_MISO_PIN))
      data |= 1;

    fastDigitalWrite(SPI_SCK_PIN, LOW);
  }
  // enable interrupts
  sei();
  return data;
}
//------------------------------------------------------------------------------
/** Soft SPI send */
void spiSend(uint8_t data)
{
  // no interrupts during byte send - about 8 us
  cli();
  for (uint8_t i = 0; i < 8; i++)
  {
    fastDigitalWrite(SPI_SCK_PIN, LOW);

    fastDigitalWrite(SPI_MOSI_PIN, data & 0X80);

    data <<= 1;

    fastDigitalWrite(SPI_SCK_PIN, HIGH);
  }
  // hold SCK high for a few ns
  nop;
  nop;
  nop;
  nop;

  fastDigitalWrite(SPI_SCK_PIN, LOW);
  // enable interrupts
  sei();
}
#endif  // SOFTWARE_SPI
//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg)
{
  // end read if in partialBlockRead mode
  readEnd();

  // select card
  chipSelectLow();

  // wait up to 300 ms if busy
  waitNotBusy(300);

  // send command
  spiSend(cmd | 0x40);

  // send argument
  for (int8_t s = 24; s >= 0; s -= 8)
  {
    spiSend(arg >> s);
  }

  // send CRC
  uint8_t crc = 0XFF;
  if (cmd == CMD0) crc = 0X95;  // correct crc for CMD0 with arg 0
  if (cmd == CMD8) crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
  spiSend(crc);

  // wait for response
  for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++)
  { }

  return status_;
}
//------------------------------------------------------------------------------
/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t Sd2Card::cardSize(void)
{
  csd_t csd;

  if (!readCSD(&csd))
    return 0;

  if (csd.v1.csd_ver == 0)
  {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10)
                      | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                          | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  }
  else if (csd.v2.csd_ver == 1)
  {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
                      | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  }
  else
  {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}
//------------------------------------------------------------------------------

void Sd2Card::chipSelectHigh(void)
{
  digitalWrite(chipSelectPin_, HIGH);
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void)
{
  digitalWrite(chipSelectPin_, LOW);
}
//------------------------------------------------------------------------------
/** Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 *
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock)
{
  if (!eraseSingleBlockEnable())
  {
    error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
    goto fail;
  }
  if (type_ != SD_CARD_TYPE_SDHC)
  {
    firstBlock <<= 9;
    lastBlock <<= 9;
  }
  if (cardCommand(CMD32, firstBlock)
    || cardCommand(CMD33, lastBlock)
    || cardCommand(CMD38, 0))
  {
      error(SD_CARD_ERROR_ERASE);
      goto fail;
  }
  if (!waitNotBusy(SD_ERASE_TIMEOUT))
  {
    error(SD_CARD_ERROR_ERASE_TIMEOUT);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Determine if card supports single block erase.
 *
 * \return The value one, true, is returned if single block erase is supported.
 * The value zero, false, is returned if single block erase is not supported.
 */
uint8_t Sd2Card::eraseSingleBlockEnable(void)
{
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}
//------------------------------------------------------------------------------
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  The reason for failure
 * can be determined by calling errorCode() and errorData().
 */
uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin)
{
  errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;
  // 16-bit init start time allows over a minute
  uint16_t t0 = (uint16_t)millis();
  uint32_t arg;

  // set pin modes
  pinMode(chipSelectPin_, OUTPUT);
  chipSelectHigh();

#ifndef SOFTWARE_SPI

  pinMode(MISO, INPUT | INPUT_SENSE_BOTH);

  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);

  DEBUG_OUT(F("Calling spiInit()\r\n"));

  spiInit(SPI_SD_INIT_RATE);

  DEBUG_OUT(F("Returned from spiInit()\r\n"));

#else // SOFTWARE_SPI

  pinMode(SPI_MISO_PIN, INPUT | INPUT_SENSE_BOTH);
  pinMode(SPI_MOSI_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);

#endif  // SOFTWARE_SPI

  // must supply min of 74 SPI clock cycles with CS high.
  for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);

  DEBUG_OUT(F("Calling chipSelectLow()\r\n"));
  chipSelectLow();

  // command to go idle in SPI mode
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE)
  {
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT)
    {
      error(SD_CARD_ERROR_CMD0);
      DEBUG_OUT(F("SD_INIT_TIMEOUT exceeded - SD_CARD_ERROR_CMD0\r\n"));

      goto fail;
    }
  }
  // check SD version
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND))
  {
    type(SD_CARD_TYPE_SD1);
  }
  else
  {
    // only need last byte of r7 response
    for (uint8_t i = 0; i < 4; i++)
      status_ = spiRec();

    if (status_ != 0XAA)
    {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type(SD_CARD_TYPE_SD2);
  }
  // initialize card and send host supports SDHC if SD2
  arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE)
  {
    // check for timeout
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT)
    {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }
  // if SD2 read OCR register to check for SDHC card
  if (type() == SD_CARD_TYPE_SD2)
  {
    if (cardCommand(CMD58, 0))
    {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }

    if ((spiRec() & 0XC0) == 0XC0)
      type(SD_CARD_TYPE_SDHC);

    // discard rest of ocr - contains allowed voltage range
    for (uint8_t i = 0; i < 3; i++)
      spiRec();
  }

  chipSelectHigh();

#ifndef SOFTWARE_SPI
  return setSckRate(sckRateID);
#else  // SOFTWARE_SPI
  return true;
#endif  // SOFTWARE_SPI

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Enable or disable partial block reads.
 *
 * Enabling partial block reads improves performance by allowing a block
 * to be read over the SPI bus as several sub-blocks.  Errors may occur
 * if the time between reads is too long since the SD card may timeout.
 * The SPI SS line will be held low until the entire block is read or
 * readEnd() is called.
 *
 * Use this for applications like the Adafruit Wave Shield.
 *
 * \param[in] value The value TRUE (non-zero) or FALSE (zero).)
 */
void Sd2Card::partialBlockRead(uint8_t value)
{
  readEnd();
  partialBlockRead_ = value;
}
//------------------------------------------------------------------------------
/**
 * Read a 512 byte block from an SD card device.
 *
 * \param[in] block Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.

 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst)
{
  return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from an SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readData(uint32_t block,
        uint16_t offset, uint16_t count, uint8_t* dst)
{
//  uint16_t n; not used; left for reference just in case

  if (count == 0)
    return true;

  if ((count + offset) > 512)
  {
    goto fail;
  }

  if (!inBlock_ || block != block_ || offset < offset_)
  {
    block_ = block;

    // use address if not SDHC card
    if (type()!= SD_CARD_TYPE_SDHC)
      block <<= 9;

    if (cardCommand(CMD17, block))
    {
      error(SD_CARD_ERROR_CMD17);
      goto fail;
    }
    if (!waitStartBlock())
    {
      goto fail;
    }
    offset_ = 0;
    inBlock_ = 1;
  }

#ifdef OPTIMIZE_HARDWARE_SPI
  if(offset > 0)
  {
    spiRec(NULL, offset);
  }
  if(count > 0)
  {
    spiRec(dst, count);
  }
  offset_ = offset + count;
#else  // OPTIMIZE_HARDWARE_SPI

  // skip data before offset
  for (;offset_ < offset; offset_++)
  {
    spiRec();
  }
  // transfer data
  for (uint16_t i = 0; i < count; i++)
  {
    dst[i] = spiRec();
  }
#endif  // OPTIMIZE_HARDWARE_SPI

  offset_ += count;
  if (!partialBlockRead_ || offset_ >= 512)
  {
    // read rest of data, checksum and set chip select high
    readEnd();
  }

  return true;

fail:
  chipSelectHigh();

  return false;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void Sd2Card::readEnd(void)
{
  if (inBlock_)
  {
      // skip data and crc
#ifdef OPTIMIZE_HARDWARE_SPI
    if(offset_ < 514)
    {
      spiRec(NULL, 514 - offset_);
    }
    offset_ = 514;
#else  // OPTIMIZE_HARDWARE_SPI
    while (offset_++ < 514) spiRec();
#endif  // OPTIMIZE_HARDWARE_SPI
    chipSelectHigh();
    inBlock_ = 0;
  }
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t Sd2Card::readRegister(uint8_t cmd, void* buf)
{
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  if (cardCommand(cmd, 0))
  {
    error(SD_CARD_ERROR_READ_REG);
    goto fail;
  }

  if (!waitStartBlock())
    goto fail;

  // transfer data
  for (uint16_t i = 0; i < 16; i++)
  {
    dst[i] = spiRec();
  }

  spiRec();  // get first crc byte
  spiRec();  // get second crc byte
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Set the SPI clock rate.
 *
 * \param[in] sckRateID A value in the range [0, 6].
 *
 * The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 * SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 * for \a scsRateID = 7.
 *
 * \return The value one, true, is returned for success and the value zero,
 * false, is returned for an invalid value of \a sckRateID.
 */
uint8_t Sd2Card::setSckRate(uint8_t sckRateID)
{
  if (sckRateID > 7)
  {
    error(SD_CARD_ERROR_SCK_RATE);
    return false;
  }

#ifndef SOFTWARE_SPI 
  if(sckRateID & 1 || sckRateID == 6)
  {
    SPIREG_CTRL &= ~SPI_CLK2X_bm;
  }
  else
  {
    SPIREG_CTRL |= SPI_CLK2X_bm; // 2x clock
  }

  // NOTE:  this only works if SPI_RESCALER_bm is 3
  SPIREG_CTRL = (SPIREG_CTRL & ~SPI_PRESCALER_gm) | ((sckRateID >> 1) & SPI_PRESCALER_gm);
            // bits 1:0 [matches values used for atmega] see D manual section 18.6.1 table 18-3

#endif // SOFTWARE_SPI 

  return true;
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(uint16_t timeoutMillis)
{
  uint16_t t0 = millis();
  do
  {
    if (spiRec() == 0XFF)
      return true;
  }
  while (((uint16_t)millis() - t0) < timeoutMillis);

  DEBUG_OUT(F("Warning, waitNotBusy timed out: "));
  DEBUG_OUT((unsigned long)spiRec());
  DEBUG_OUT(F(","));
  DEBUG_OUT((unsigned long)spiRec());
  DEBUG_OUT(F(","));
  DEBUG_OUT((unsigned long)spiRec());
  DEBUG_OUT(F(","));
  DEBUG_OUT((unsigned long)spiRec());
  DEBUG_OUT(F("\r\n"));

  return false;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t Sd2Card::waitStartBlock(void)
{
  uint16_t t0 = millis();
  while ((status_ = spiRec()) == 0XFF)
  {
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT)
    {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK)
  {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src)
{
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0)
  {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO

  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC)
    blockNumber <<= 9;

  if (cardCommand(CMD24, blockNumber))
  {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }

  if (!writeData(DATA_START_BLOCK, src))
    goto fail;

  // wait for flash programming to complete
  if (!waitNotBusy(SD_WRITE_TIMEOUT))
  {
    error(SD_CARD_ERROR_WRITE_TIMEOUT);
    goto fail;
  }
  // response is r2 so get and check two bytes for nonzero
  if (cardCommand(CMD13, 0) || spiRec())
  {
    error(SD_CARD_ERROR_WRITE_PROGRAMMING);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Write one data block in a multiple block write sequence */
uint8_t Sd2Card::writeData(const uint8_t* src)
{
  // wait for previous write to finish
  if (!waitNotBusy(SD_WRITE_TIMEOUT))
  {
    error(SD_CARD_ERROR_WRITE_MULTIPLE);
    chipSelectHigh();
    return false;
  }
  return writeData(WRITE_MULTIPLE_TOKEN, src);
}
//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
uint8_t Sd2Card::writeData(uint8_t token, const uint8_t* src)
{
#ifdef OPTIMIZE_HARDWARE_SPI

  spiSend(token);
  spiSend(src, 512);

#else  // OPTIMIZE_HARDWARE_SPI

  spiSend(token);

  for (uint16_t i = 0; i < 512; i++)
  {
    spiSend(src[i]);
  }

#endif  // OPTIMIZE_HARDWARE_SPI

  spiSend(0xff);  // dummy crc
  spiSend(0xff);  // dummy crc

  status_ = spiRec();

  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED)
  {
    error(SD_CARD_ERROR_WRITE);
    chipSelectHigh();
    return false;
  }

  return true;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and writeStop()
 * for optimized multiple block writes.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount)
{
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0)
  {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO
  // send pre-erase count
  if (cardAcmd(ACMD23, eraseCount))
  {
    error(SD_CARD_ERROR_ACMD23);
    goto fail;
  }
  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC)
    blockNumber <<= 9;
  if (cardCommand(CMD25, blockNumber))
  {
    error(SD_CARD_ERROR_CMD25);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.
 *
* \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStop(void)
{
  if (!waitNotBusy(SD_WRITE_TIMEOUT))
    goto fail;

  spiSend(STOP_TRAN_TOKEN);

  if (!waitNotBusy(SD_WRITE_TIMEOUT))
    goto fail;

  chipSelectHigh();
  return true;

 fail:
  error(SD_CARD_ERROR_STOP_TRAN);
  chipSelectHigh();
  return false;
}

