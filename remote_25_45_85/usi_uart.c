/*****************************************************************************
*
* Copyright (C) 2003 Atmel Corporation
*
* File              : USI_UART.c
* Compiler          : IAR EWAAVR 2.28a
* Created           : 18.07.2002 by JLL
* Modified          : 02-10-2003 by LTA
*
* Support mail      : avr@atmel.com
*
* Application Note  : AVR307 - Half duplex UART using the USI Interface
*
* Description       : Functions for USI_UART_receiver and USI_UART_transmitter.
*                     Uses Pin Change Interrupt to detect incoming signals.
*
*
****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "usi_uart.h"


//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER)
#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ))

#if ( (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
#define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
#define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
#define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
#define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0

//********** Static Variables **********//
static unsigned char          USI_UART_TxData;
#ifdef ENABLE_RX
static unsigned char          UART_RxBuf[UART_RX_BUFFER_SIZE];  // UART buffers. Size is definable in the header file.
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
#endif
static unsigned char          UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

static volatile unsigned char UART_ongoing_Transmission_From_Buffer;
static volatile unsigned char UART_ongoing_Transmission_Of_Package;
static volatile unsigned char UART_ongoing_Reception_Of_Package;
static volatile unsigned char UART_reception_Buffer_Overflow;

//********** USI_UART functions **********//

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x )
{
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}

// Flush the UART buffers.
void USI_UART_Flush_Buffers( void )
{
#ifdef ENABLE_RX
	UART_RxTail = 0;
	UART_RxHead = 0;
#endif
	UART_TxTail = 0;
	UART_TxHead = 0;
}

// Initialize USI for UART transmission.
void USI_UART_Initialise_Transmitter( void )
{
	cli();
	TCNT0  = 0x00;
#if TIMER_PRESCALER == 1
	TCCR0B = (0<<CS02)|(0<<CS01)|(1<<CS00);  			      // Reset the prescaler and start Timer0.
#elif TIMER_PRESCALER == 8
	TCCR0B = (0<<CS02)|(1<<CS01)|(0<<CS00);  			      // Reset the prescaler and start Timer0.
#else
#error TIMER_PRESCALER can only be 1 or 8
#endif
	TIFR   = (1<<TOV0);                                       // Clear Timer0 OVF interrupt flag.
	TIMSK |= (1<<TOIE0);                                      // Enable Timer0 OVF interrupt.

	USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
	         (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
	         (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 OVER as USI Clock source.
	         (0<<USITC);

	USIDR  = 0xFF;                                            // Make sure MSB is '1' before enabling USI_DO.
	USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
	         0x0F;                                            // Preload the USI counter to generate interrupt at first USI clock.
	DDRB  |= (1<<PB1);                                        // Configure USI_DO as output.

	UART_ongoing_Transmission_From_Buffer = TRUE;

	sei();
}

#ifdef ENABLE_RX

// Initialize USI for UART reception.
// Note that this function only enables pinchange interrupt on the USI Data Input pin.
// The USI is configured to read data within the pinchange interrupt.
void USI_UART_Initialise_Receiver( void )
{
	PORTB |=   (1<<PB1)|(1<<PB0);							// Enable pull up on USI DO and DI
	DDRB  &= ~((1<<PB1)|(1<<PB0));							// Set USI DI and DO
	USICR  =  0;                                            // Disable USI.
	GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
	GIMSK |=  (1<<PCIE);                                    // Enable pin change interrupt for PORTB
	PCMSK |=  (1<<PCINT0);									// Enable pin change interrupt for PB0
}

#endif

// Puts data in the transmission buffer, after reversing the bits in the byte.
// Initiates the transmission routines if not already started.
void USI_UART_Transmit_Byte( unsigned char data )
{
	unsigned char tmphead;

	tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;        // Calculate buffer index.
	while ( tmphead == UART_TxTail );                           // Wait for free space in buffer.
	UART_TxBuf[tmphead] = Bit_Reverse(data);                    // Reverse the order of the bits in the data byte and store data in buffer.
	UART_TxHead = tmphead;                                      // Store new index.

	if ( !UART_ongoing_Transmission_From_Buffer )    // Start transmission from buffer (if not already started).
	{
		while ( UART_ongoing_Reception_Of_Package ); // Wait for USI to finish reading incoming data.
		USI_UART_Initialise_Transmitter();
	}
}

#ifdef ENABLE_RX

// Returns a byte from the receive buffer. Waits if buffer is empty.
unsigned char USI_UART_Receive_Byte( void )
{
	unsigned char tmptail;

	while ( UART_RxHead == UART_RxTail );                 // Wait for incoming data
	tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK;  // Calculate buffer index
	UART_RxTail = tmptail;                                // Store new index
	return Bit_Reverse(UART_RxBuf[tmptail]);              // Reverse the order of the bits in the data byte before it returns data from the buffer.
}

// Check if there is data in the receive buffer.
unsigned char USI_UART_Data_In_Receive_Buffer( void )
{
	return ( UART_RxHead != UART_RxTail );                // Return 0 (FALSE) if the receive buffer is empty.
}

// ********** Interrupt Handlers ********** //

// The pin change interrupt is used to detect USI_UART reception.
// It is here the USI is configured to sample the UART signal.
ISR(SIG_PIN_CHANGE)
{
	if (!( PINB & (1<<PB0) ))                                     // If the USI DI pin is low, then it is likely that it
	{                                                             //  was this pin that generated the pin change interrupt.
		TCNT0  = INTERRUPT_STARTUP_DELAY + INITIAL_TIMER0_SEED;   // Plant TIMER0 seed to match baudrate (incl interrupt start up time.).
		#if TIMER_PRESCALER == 1
		TCCR0B  = (0<<CS02)|(0<<CS01)|(1<<CS00);  			      // Reset the prescaler and start Timer0.
		#elif TIMER_PRESCALER == 8
		TCCR0B  = (0<<CS02)|(1<<CS01)|(0<<CS00);  			      // Reset the prescaler and start Timer0.
		#else
		#error TIMER_PRESCALER can only be 1 or 8
		#endif
		TIFR   = (1<<TOV0);                                       // Clear Timer0 OVF interrupt flag.
		TIMSK |= (1<<TOIE0);                                      // Enable Timer0 OVF interrupt.

		USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
		         (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
		         (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 OVER as USI Clock source.
		         (0<<USITC);
		// Note that enabling the USI will also disable the pin change interrupt.
		USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
		USI_COUNTER_SEED_RECEIVE;                                 // Preload the USI counter to generate interrupt.

		GIMSK &=  ~(1<<PCIE);                                     // Disable pin change interrupt for PORTB

		UART_ongoing_Reception_Of_Package = TRUE;
	}
}

#endif

// The USI Counter Overflow interrupt is used for moving data between memory and the USI data register.
// The interrupt is used for both transmission and reception.
ISR(SIG_USI_OVERFLOW)
{
	#ifdef ENABLE_RX
	unsigned char tmphead;
	#endif
	unsigned char tmptail;

	// Check if we are running in Transmit mode.
	if( UART_ongoing_Transmission_From_Buffer )
	{
		// If ongoing transmission, then send second half of transmit data.
		if( UART_ongoing_Transmission_Of_Package )
		{
			UART_ongoing_Transmission_Of_Package = FALSE;    // Clear on-going package transmission flag.

			USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);                 // Load USI Counter seed and clear all USI flags.
			USIDR = (USI_UART_TxData << 3) | 0x07;                      // Reload the USIDR with the rest of the data and a stop-bit.
		}
		// Else start sending more data or leave transmit mode.
		else
		{
			// If there is data in the transmit buffer, then send first half of data.
			if ( UART_TxHead != UART_TxTail )
			{
				UART_ongoing_Transmission_Of_Package = TRUE; // Set on-going package transmission flag.

				tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;    // Calculate buffer index.
				UART_TxTail = tmptail;                                  // Store new index.
				USI_UART_TxData = UART_TxBuf[tmptail];                  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
				// The bit reversing is moved to the application section to save time within the interrupt.
				USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);            // Load USI Counter seed and clear all USI flags.
				USIDR  = (USI_UART_TxData >> 2) | 0x80;                 // Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
				                                                        //  of bit of bit reversed data).
			}
			// Else enter receive mode.
			else
			{
				UART_ongoing_Transmission_From_Buffer = FALSE;

				TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                // Stop Timer0.
				#ifdef ENABLE_RX
				PORTB |=   (1<<PB1)|(1<<PB0);							// Enable pull up on USI DO and DI
				DDRB  &= ~((1<<PB1)|(1<<PB0));							// Set USI DI and DO
				USICR  =  0;                                            // Disable USI
				GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag
				GIMSK |=  (1<<PCIE);                                    // Enable pin change interrupt for PORTB
				PCMSK |=  (1<<PCINT0);									// Enable pin change interrupt for PB0
				#else
				PORTB |=  (1<<PB1);										// Enable pull up on USI DO
				DDRB  &= ~(1<<PB1);										// Set USI DO
				USICR  =  0;                                            // Disable USI
				#endif
			}
		}
	}
#ifdef ENABLE_RX
	// Else running in receive mode.
	else
	{
		UART_ongoing_Reception_Of_Package = FALSE;

		tmphead     = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;        // Calculate buffer index.

		if ( tmphead == UART_RxTail )                                   // If buffer is full trash data and set buffer full flag.
		{
			UART_reception_Buffer_Overflow = TRUE;           // Store status to take actions elsewhere in the application code
		}
		else                                                            // If there is space in the buffer then store the data.
		{
			UART_RxHead = tmphead;                                      // Store new index.
			UART_RxBuf[tmphead] = USIDR;                                // Store received data in buffer. Note that the data must be bit reversed before used.
		}                                                               // The bit reversing is moved to the application section to save time within the interrupt.

		TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                // Stop Timer0.
		PORTB |=  (1<<PB1)|(1<<PB0);							// Enable pull up on USI DO and DI
		DDRB  &= ~((1<<PB1)|(1<<PB0));							// Set USI DI and DO
		USICR  =  0;                                            // Disable USI.
		GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
		GIMSK |=  (1<<PCIE);									// Enable pin change interrupt for PORTB
		PCMSK |=  (1<<PCINT0);									// Enable pin change interrupt for PB0
	}
#endif
}
// Timer0 Overflow interrupt is used to trigger the sampling of signals on the USI ports.
ISR(SIG_OVERFLOW0)
{
	TCNT0 += TIMER0_SEED;                   // Reload the timer,
	// current count is added for timing correction.
}
