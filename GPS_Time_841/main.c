/*
 * GPS_Time_841.c
 *
 * Created: 4/30/2016 8:42:04 PM
 * Author : Rick Shory
 */ 

#define F_CPU 8000000UL
#define CT_10MS 1250
#define TOGGLE_INTERVAL 100

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/sleep.h>

// function prototypes
void stayRoused(uint16_t dSec);
void endRouse(void);

enum machStates
{
	Asleep = 0,
	Idle,  // done with work but not yet allowed to go to sleep
	WakedFromSleep, // first test on wake from sleep
	TimerInitialized, // Timer1 has been initialized
	PoweredOnGPS, // the GPS has been powered on
	SerialRxFromGPS, // Serial data has been received from the GPS
	ValidNMEARxFromGPS, // Valid NMEA data has been received from the GPS
	ValidTimeRxFromGPS, // Valid timestamp has been received from the GPS
	PoweredOffGPS // the GPS has been powered off
};

enum stateFlagsBits
{
	isRoused, // triggered by initiating Reset, and re-triggered by any activity while awake
	reRoused, // re-triggered while awake, used to reset timeout
	isGPSPowerOn, // is the GPS powered on
	isSerialRxFromGPS, // has some serial data been received from the GPS
	isValid_NMEA_RxFromGPS, // has some valid NMEA data been received from the GPS
	isValidTimeRxFromGPS, // has a valid Timestamp been received from the GPS
	sfBit6, // unused
	sfBit7 // unused
};

volatile uint8_t machineState;
volatile uint8_t stateFlags = 0;
//volatile uint8_t iTmp;
volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker
volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep
volatile uint8_t Timer1, Timer2, Timer3;	// 100Hz decrement timer

//pins by package
//    PDIP QFN     used for programming
// PA0 13   5
// PA1 12   4 Tx0 (default location)
// PA2 11   3 Rx0 (default location)
// PA3 10   2        
// PA4  9   1 Rx1    yes
// PA5  8  20 Tx1    yes
// PA6  7  16        yes
// PA7  6  15 Tx0 (alternative location)
// PB0  2  11
// PB1  3  12
// PB2  5  14 Rx0 (alternative location)
// PB3  4  13        yes

#define LED PA3

int main(void)
{
	
	// set up to blink an LED
	DDRA |= (1<<LED);

	// set up a heartbeat counter, interrupt every 0.1 second
	// set up 16-bit TIMER1
	// F_CPU = 8MHz
	OCR1A = CT_10MS; // 0.01 sec; use prescaler 64
	// only channel A will cause CTC

	// note that WGM1[3:0] is split over TCCR1A and TCCR1B
	// WGM1[3:2] is TCCR1B[4:3] while WGM1[1:0] is TCCR1A[1:0]

	// TCCR1A – Timer/Counter1 Control Register A
	// 7 COM1A1
	// 6 COM1A0
	// 5 COM1B1
	// 4 COM1B0
	// (3:2 reserved)
	// 1 WGM11
	// 0 WGM10
	
	// WGM1[3:0]=(0100), Clear Timer on Compare Match (CTC) Mode using OCR1A for TOP
	// TCCR1B[4:3]=(0:1)
	// TCCR1A[1:0]=(0:0)
	// the counter is cleared to zero when the counter value (TCNT1)
	// matches OCR1A, OCF1A Interrupt Flag Set
	// TOV1 flag is set in the same timer clock cycle that the counter counts from MAX to 0x0000
	// ? since timer clears on match, returns to 0, this never happens
	
	// in Compare Output Mode, non-PWM:
	// do not output the compare match on any pins
	//   COM1A[1:0], TCCR1A[7:6]=(0:0), Normal port operation, OC1A disconnected
	//   COM0B[1:0], TCCR0A[5:4]=(0:0), Normal port operation, OC1B disconnected
		TCCR1A = 0b00000000;
	
	// TCCR1B – Timer/Counter1 Control Register B
	// 7 ICNC1: Input Capture Noise Canceler
	// 6 ICES1: Input Capture Edge Select
	// (5 reserved)
	// 4 WGM13
	// 3 WGM12
	// 2 CS12 clock select bit 2
	// 1 CS11 clock select bit 1
	// 0 CS10 clock select bit 0
	
	// use ICNC1, ICES1 unused, leave as default of 0
	// use WGM1[3:2]=(01) for CTC Mode
	// use CS0[2:0]=(011), prescale 64
	TCCR1B = 0b00001011;

	// TCCR1C – Timer/Counter1 Control Register C
	// 7 FOC1A: Force Output Compare for Channel A
	// 6 FOC1B: Force Output Compare for Channel B
	// (5:0 reserved)
	// all unused, leave default of zero
	//TCCR1C = 0;

	// TIMSK1 – Timer/Counter 1 Interrupt Mask Register
	// (7, 6, 4, 3 reserved)
	//  When these bits are written to one, and the I-flag in the Status Register is set (interrupts globally
	//  enabled), the corresponding interrupt is enabled. The
	//  corresponding Interrupt Vector is executed when the
	//  corresponding bit is set in the Timer/Counter Interrupt Flag Register – TIFR1.
	// 5 ICIE1: Timer/Counter, Input Capture Interrupt Enable (not used here)
	// 2 OCIE1B: Timer/Counter1 Output Compare Match B Interrupt Enable (not used here)
	// 1 OCIE1A: Timer/Counter1 Output Compare Match A Interrupt Enable
	// 0 TOIE1: Timer/Counter1, Overflow Interrupt Enable (not used here)

	cli(); // temporarily disable interrupts
	// set the counter to zero
	TCNT1 = 0;
	// enable Output Compare 1A interrupt
	TIMSK1 |= (1<<OCIE1A);
	// clear the interrupt flag, write a 1 to the bit location
	TIFR1 |= (1<<OCF1A);
	
	// power-down sleep enable
	// clear interrupt flag; probably don't need to do this because flag is cleared on level interrupts
	GIFR |= (1<<INTF0);
	// To enter a sleep mode, the SE bit in MCUCR must be set and
	// a SLEEP instruction must be executed. The SMn bits in
	// MCUCR select which sleep mode will be activated by
	// the SLEEP instruction.
			
	// MCUCR – MCU Control Register
	// (7, 6, 2 reserved)
	// 5 - SE: Sleep Enable
	// 4:3 – SM[1:0]: Sleep Mode Select Bits 1 and 0
	//       SM[1:0]=(10) for Power-down
	// 1:0 – ISC0[1:0]: Interrupt Sense Control 0 Bit 1 and Bit 0
	// ISC01 ISC00 Description
	//   0     0    The low level of INT0 generates an interrupt request
	//   0     1    Any logical change on INT0 generates an interrupt request
	//   1     0    The falling edge of INT0 generates an interrupt request
	//   1     1    The rising edge of INT0 generates an interrupt request
	//           try any-logic-change
	// don't set SE yet
	// set the global interrupt enable bit.
	sei();
	
	stayRoused(1000); // stay roused for 10 seconds

    while (1) 
    {
		if (!(stateFlags & (1<<isRoused))) {
			// go to sleep
			PORTA &= ~(1<<LED); // turn off pilot light blinkey
			// To enter a sleep mode, the SE bit in MCUCR must be set and 
			// a SLEEP instruction must be executed. The SMn bits in
			// MCUCR select which sleep mode will be activated by 
			// the SLEEP instruction.
			
			// MCUCR – MCU Control Register
			// (7, 6, 2 reserved)
			// 5 - SE: Sleep Enable
			// 4:3 – SM[1:0]: Sleep Mode Select Bits 1 and 0
			//       SM[1:0]=(10) for Power-down
			// 1:0 – ISC0[1:0]: Interrupt Sense Control 0 Bit 1 and Bit 0
			// ISC01 ISC00 Description
			//   0     0    The low level of INT0 generates an interrupt request
			//   0     1    Any logical change on INT0 generates an interrupt request
			//   1     0    The falling edge of INT0 generates an interrupt request
			//   1     1    The rising edge of INT0 generates an interrupt request
			//           try any-logic-change
			// set SE (sleep enable)
			MCUCR |= (1<<SE);
			// go intoPower-down mode SLEEP
			asm("sleep");
		}
    }
}

void stayRoused(uint16_t dSec)
{
	cli(); // temporarily disable interrupts to prevent Timer1 from
	// changing the count partway through
	if ((dSec) > rouseCountdown) { // never trim the rouse interval, only extend it
		rouseCountdown = dSec;
	}
	stateFlags |= (1<<isRoused);
	PORTA |= (1<<LED); // set pilot light on
	sei();
}

void endRouse(void) {
	cli(); // temporarily disable interrupts to prevent Timer1 from
	// changing the count partway through
	rouseCountdown = 0;
	stateFlags &= ~(1<<isRoused);
	PORTA &= ~(1<<LED); // force pilot light off
	sei();
	
}

ISR(TIMER1_COMPA_vect) {
	// occurs when TCNT1 matches OCR1A
	// set to occur at 100Hz
	char n;
	int16_t t;
	if (--ToggleCountdown <= 0) {
		PORTA ^= (1<<LED); // toggle bit 2, pilot light blinkey
		ToggleCountdown = TOGGLE_INTERVAL;
	}
	

	t = rouseCountdown;
	if (t) rouseCountdown = --t;
	
	//	if (--rouseCountdown <= 0)
	if (!rouseCountdown)
	{
		stateFlags &= ~(1<<isRoused);
	}

	n = Timer1;						// 100Hz decrement timer 
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;
	n = Timer3;
	if (n) Timer3 = --n;

}
