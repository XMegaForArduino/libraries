/* This file has been prepared for Doxygen automatic documentation generation. */
/*! \file *********************************************************************
 *
 * \brief Test signal generation, generates a Quadrature signal.
 *
 *    This file contains code to generate a Quadrature signal for testing
 *    of the system. The signal generated goes either direction given by a define
 *    the CW_DIR_SIG gives a signal that counts in clockwise direction. The
 *    CCW_DIR_SIG define gives a signal in counter clockwise direction.
 *
 *    To use the test signal the GENERATE_TEST_SIGNAL has to be uncommented in the
 *    qdec_example.c file.
 *    The generate_qdec_signal will then be included and a signal generated.
 *    The signal output pins then has to be connected with the input pins to the
 *    Quadrature decoder.
 *
 *    When the signal is connected and the output portC is connected to the led’s
 *    the frequency value will be displayed in hex, pin0 LSB.
 *
 *    To test that the index signal is correct, when enabled, one can set a
 *    breakpoint in the timer counter C0 error interrupt routine. If the
 *    interrupt routine is run more than 1 time (first time resets the count)
 *    the index setting is wrong. If it is correct the execution should not break
 *    when the index signal is connected, but if the signal is disconnected (remove cable)
 *    the code should break.
 *
 *    If a scope is used the frequency measurement can be controlled, the measured
 *    frequency should be equal to the frequency on the scope given by the time
 *    between the index signal.
 *
 * \par Application note:
 *      AVR1600: Using the XMEGA Quadrature Decoder
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1699 $
 * $Date: 2008-07-30 09:20:10 +0200 (on, 30 jul 2008) $
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "avr_compiler.h"
#include "qdec_signal_generator.h"

/*! \brief The port to set the signal out on. */
PORT_t * q_test_sig_Port;


/*! \brief Number of lines in the Quadrature encoder. */
uint8_t test_lineCount;


/*! \brief Initializes TCE0 to create Qadrature signal.
 *
 *  Calculates the values to create a Quadrature signal that has a
 *  frequency given from the freq parameter.
 *
 * \param qPort     The port to set the signal out on.
 * \param lineCount The number of lines in the Quadrature encoder.
 * \param freq      The frequency that is put out,
 *                  (time between index signal/one rotation).
 * \param dir       The direction of the signal to be generated,
 *                  clockwise or counterclockwise signal.
 */
void generate_qdec_signal(PORT_t * qPort, uint8_t lineCount, uint8_t freq, bool dir)
{
	uint16_t ticks, quarter, half_quarter;

	/* The following code calculates the upper boundary of the timer and the
	 * interrupt positions to get a correct Quadrature signal of the given frequency.
	 *
	 * The different compare interrupts sets the phase0 and phase90 signals.
	 * Compare A interrupt sets phase0 and clears phase90
	 * Compare B interrupt sets phase0 and phase90
	 * Compare C interrupt clears phase0 and sets phase90
	 * Compare D interrupt clears phase0 and phase90.
	 *
	 * Ccompare A interrupt also sets the index signal when one round has passed.
	 */

	/* Calculates upper boundary of timer to get desired frequency.*/
	ticks = F_CPU / (freq * lineCount);
	quarter = ticks/4;
	half_quarter = ticks/8;

	if(dir == 1){
		TCE0.CCA = half_quarter;
		TCE0.CCB = half_quarter+quarter;
		TCE0.CCC = half_quarter+(2*quarter);
		TCE0.CCD = half_quarter+(3*quarter);
	}else{
		TCE0.CCA = half_quarter+(3*quarter);
		TCE0.CCB = half_quarter+(2*quarter);
		TCE0.CCC = half_quarter+quarter;
		TCE0.CCD = half_quarter;
	}

	TCE0.PER = ticks;
	TCE0.CTRLA = TC_CLKSEL_DIV1_gc;

	/* Enable low level interrupt on CCA, CCB, CCC and CCD.*/
	TCE0.INTCTRLB = TC0_CCAINTLVL0_bm | TC0_CCBINTLVL0_bm |
	                TC0_CCCINTLVL0_bm | TC0_CCDINTLVL0_bm;
	TCC0.INTCTRLA = TC0_ERRINTLVL0_bm;

	qPort->DIRSET = 0xFF;
	q_test_sig_Port = qPort;

	test_lineCount = lineCount;
}


/*! \brief Creates a Quadrature signal, up or down counting depending
 *         on CW_DIR_SIG or CCW_DIR_SIG.
 */
ISR(TCE0_CCA_vect)
{
	static uint16_t i = 0;
	
	/* Set pin0 Phase0 signal.*/
	q_test_sig_Port->OUT = (q_test_sig_Port->OUT & ~0x03) | 0x01; 

	i++;

	/* Clear index.*/
	q_test_sig_Port->OUT = (q_test_sig_Port->OUT & ~0x04);

	/*  When (i = test_lineCount) one round has passed.
	 *  Includes a check for i "bigger than" for error handling.
	 */
	if(i>=test_lineCount){

		/* Set index. Lasts 4 states.*/
		q_test_sig_Port->OUT |= 0x04;
		i = 0;
	}
}


/*! \brief Creates a Quadrature signal, up or down counting depending
 *         on CW_DIR_SIG or CCW_DIR_SIG.
 */
ISR(TCE0_CCB_vect)
{
	/* Set pin0 and pin1 phase0 and phase90 signal.*/
	q_test_sig_Port->OUT = (q_test_sig_Port->OUT & ~0x03) | 0x01 | 0x02;
}


/*! \brief Creates a Quadrature signal, up or down counting depending
 *          on CW_DIR_SIG or CCW_DIR_SIG.
 */
ISR(TCE0_CCC_vect)
{
	/* Set pin1 phase90 signal. Clear pin0 phase0 signal.*/
	q_test_sig_Port->OUT = (q_test_sig_Port->OUT & ~0x03) | 0x02;
}


/*! \brief Creates a Quadrature signal, up or down counting depending
 *          on CW_DIR_SIG or CCW_DIR_SIG.
 */
ISR(TCE0_CCD_vect)
{
	/* Clear pin0 and pin1, phase0 and phase90 signal.*/
	q_test_sig_Port->OUT = (q_test_sig_Port->OUT & ~0x03);
}


/*! \brief Error interrupt routine for when index signal is used.
 *
 *         This interrupt happens if the count value is not at BOTTOM
 *         when the index signal comes. The interrupt also happens if
 *         the count value passes the BOTTOM value.
 */
ISR(TCC0_ERR_vect)
{
	static uint8_t j = 0;
	j++;

	/* Since index needs to initialize, one error will happen first round.
	 * If output is desired at every error, remove if statement or set (j>0).
	 */
	if(j>2){

		/* To test if index works, set breakpoint here.
		 * It should NOT break when index is correct.
		 */
		q_test_sig_Port->OUT = q_test_sig_Port->OUT ^ 0x40;
		j=0;
	}
}
