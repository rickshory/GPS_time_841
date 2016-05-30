/*
 * GPS_Time_841.c
 *
 * Created: 4/30/2016 8:42:04 PM
 * Author : Rick Shory
 */ 

#define F_CPU 8000000UL
#define CT_10MS 1250
#define TOGGLE_INTERVAL 100
#define GPS_TX_BAUD 4800
#define UC_RX_BAUD 9600
#define RX_BUF_LEN 128
#define TX_BUF_LEN 32

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/sleep.h>

// function prototypes
void stayRoused(uint16_t dSec);
void endRouse(void);
void sendSetTimeSignal(void);
int gpsOn(void);
int gpsOff(void);
int parseNMEA(void);

/*
example
$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68

225446       Time of fix 22:54:46 UTC
A            Navigation receiver warning A = OK, V = warning
4916.45,N    Latitude 49 deg. 16.45 min North
12311.12,W   Longitude 123 deg. 11.12 min West
000.5        Speed over ground, Knots
054.7        Course Made Good, True
191194       Date of fix  19 November 1994
020.3,E      Magnetic variation 20.3 deg East
*68          mandatory checksum
*/

// NMEA sentence, for example:
//$GPRMC,110919.000,A,4532.1047,N,12234.3348,W,1.98,169.54,090316,,,A*77
enum NMEA_fields {sentenceType, timeStamp, isValid, curLat, isNorthOrSouth, curLon, isEastOrWest,
speedKnots, trueCourse, dateStamp, magVar, varEastOrWest, checkSum};

static volatile union Prog_status // Program status bit flags
{
	unsigned char status;
	struct
	{
		unsigned char gps_Request_Active:1;
		unsigned char gps_Power_Enabled:1;
		unsigned char gps_Being_Pulsed:1;
		unsigned char listen_To_GPS:1;
		unsigned char cur_Rx_Bit:1;
		unsigned char cmd_Tx_ongoing:1;
		unsigned char new_NMEA_Field:1;
		unsigned char serial_Received:1;
	};
	} Prog_status = {0};

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

volatile uint8_t machineState, GpsOnAttempts = 0, GpsOffAttempts = 0;
volatile uint8_t stateFlags = 0;
//volatile uint8_t iTmp;
volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker
volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep
volatile uint8_t Timeout1, Timeout2, Timeout3;	// 100Hz decrement timeouts

static volatile char receiveByte;
static volatile char recBuf[RX_BUF_LEN];
static volatile char *recBufInPtr;
static volatile char cmdOut[TX_BUF_LEN] = "t2016-03-19 20:30:01 -08\n\r\n\r\0";
static volatile char *cmdOutPtr;
static volatile char *NMEA_Ptrs[13]; // array of pointers to field positions within the captured NMEA sentence

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
#define TX1 PA5
#define RX0 PA2
#define GPS_PWR PB0
#define PULSE_GPS PB1

int main(void)
{
	uint16_t GpsTxUbrr, UcRxUbrr;

	// set up to blink an LED
	DDRA |= (1<<LED);
	
	// set up Tx1 as an output
	DDRA |= (1<<TX1);
	
	// set up GPS control lines
	DDRB |= (1<<GPS_PWR); // set as output
	PORTB &= ~(1<<GPS_PWR); // assure GPS power enable starts low
	DDRB |= (1<<PULSE_GPS); // set as output
	PORTB &= ~(1<<PULSE_GPS); // assure pulse signal to GPS starts low

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
	
	// Set up USARTS
	// make sure enabled, no power reductions for USARTS
	// PRR – Power Reduction Register
	// 7 – PRTWI: Power Reduction Two-Wire Interface
	// 6 – PRUSART1: Power Reduction USART1
	// 5 – PRUSART0: Power Reduction USART0
	// 4 – PRSPI: Power Reduction SPI
	// 3 – PRTIM2: Power Reduction Timer/Counter2
	// 2 – PRTIM1: Power Reduction Timer/Counter1
	// 1 – PRTIM0: Power Reduction Timer/Counter0
	// 0 – PRADC: Power Reduction ADC
	
	// setting bits=0 assures power reduction is OFF
	PRR &= ~((1<<PRUSART1) | (1<<PRUSART0));
	
	// UCSRnC– USART Control and Status Register C
	// 7:6 use default (00)= Asynchronous USART
	// 5:4 use default (00)= Parity disabled
	// 3 use default, 0= 1 stop bit
	// 2:1 – UCSZn[1:0]: Character Size, 
	//       Together with the UCSZn2 bit, the UCSZn[1:0] bits set the number of data bits
	// 2:1 use (11)= 8-bit character size (use default UCSRnB[UCSZn2]= 0)
	// 0 use default 0, Clock Polarity, don't care, not used by Asynchronous USART
	
	// Set USART0 to receive the GPS Tx
	// set USART0 baud rate
	GpsTxUbrr = 103; // (F_CPU/(16 * GPS_TX_BAUD))-1 for GPS Tx at 4800 baud
	UBRR0H = (unsigned char)(GpsTxUbrr>>8);
	UBRR0L = (unsigned char)GpsTxUbrr;
	// set USART0 to receive
	// UCSR0A, use defaults
	// UCSR0B - use defaults except for these 
	// 7 – RXCIE0: RX Complete Interrupt Enable
	// 4 – RXEN0: Receiver Enable
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0);
	// set USART0 frame format: 8data, 1stop bit
	UCSR0C = (3<<UCSZ00);
	//UCSR0D, leave default 0, do not let Rx wake this uC, probably would be noise

	// Set USART1 to transmit to the main uC Rx
	// set USART1 baud rate
	UcRxUbrr = 51; // (F_CPU/(16 * UC_RX_BAUD))-1 for main uC Rx at 9600 baud
	UBRR1H = (unsigned char)(UcRxUbrr>>8);
	UBRR1L = (unsigned char)UcRxUbrr;
	// set USART1 to transmit
	// UCSR1A, use defaults
	UCSR1B = (1<<TXEN1); // use defaults except for this
	// set USART1 frame format: 8data, 1stop bit
	UCSR1C = (3<<UCSZ10);
	//UCSR1D, leave default 0, do not let Rx wake this uC
	
	//REMAP[0] – U0MAP: USART0 Pin Mapping
	// U0MAP RXD0 TXD0 Note
	//  0    PA2  PA1  Default
	//  1    PB2  PA7  Remapped
	// for now, leave as default 0

	cli(); // temporarily disable interrupts
	// set the counter to zero
	TCNT1 = 0;
	// enable Output Compare 1A interrupt
	TIMSK1 |= (1<<OCIE1A);
	// clear the interrupt flag, write a 1 to the bit location
	TIFR1 |= (1<<OCF1A);
	
	// clear interrupt flag; probably don't need to do this because flag is cleared on level interrupts
	GIFR |= (1<<INTF0);

	
	// set the global interrupt enable bit.
	sei();
	
	stayRoused(1500); // stay roused for 15 seconds
	
	if (!gpsOn()) { // successfully turned on GPS
		stayRoused(1000); // extend rouse interval another 10 seconds at least
	} else {
		; // let rouse time out
	}

    while (1) 
    {
		if (!(stateFlags & (1<<isRoused))) {
			// go to sleep
			PORTA &= ~(1<<LED); // turn off pilot light blinkey
			
			// before setting PRADC (PRR[0], below) assure ADC is disabled
			// may not be necessary, if ADC never enabled, but assures lowest power
			// ADCSRA – ADC Control and Status Register A
			// 7 - ADEN: ADC Enable. Writing this bit to zero turns off the ADC
			ADCSRA &= ~(1<<ADEN);
			
			// PRR – Power Reduction Register
			// 7 – PRTWI: Power Reduction Two-Wire Interface
			// 6 – PRUSART1: Power Reduction USART1
			// 5 – PRUSART0: Power Reduction USART0
			// 4 – PRSPI: Power Reduction SPI
			// 3 – PRTIM2: Power Reduction Timer/Counter2
			// 2 – PRTIM1: Power Reduction Timer/Counter1
			// 1 – PRTIM0: Power Reduction Timer/Counter0
			// 0 – PRADC: Power Reduction ADC
			
			// shut down any peripheral clocks by writing all 1s
			// may not be necessary in power-down sleep mode, but assures lowest power
			PRR = 0xff;
			
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
				
			// set Power-down sleep mode, wait to set SE, don't care about any other bits
			MCUCR = 0b00010000;
			// set SE (sleep enable)
			MCUCR |= (1<<SE);
			// go intoPower-down mode SLEEP
			asm("sleep");
		} // end of go-to-sleep

		// continue main program loop
	
		// following will be the usual exit point
		// calls a function to send the set-time signal back to the main uC
		// that function, if successful, will tie things up and end Rouse mode
		// which will allow this uC to shut down till woken again by Reset
		if (stateFlags & (1<<isValidTimeRxFromGPS)) {
			sendSetTimeSignal();
		}
    } // end of 'while(1)' main program loop
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

int gpsOn(void) {
	uint8_t tryCt;
	Prog_status.serial_Received = 0; // assure flag starts clear
	PORTB |= (1<<GPS_PWR); // turn GPS power on
	for (Timeout1 = 100; Timeout1; ); // Wait for 1sec (1000ms) for power to stabilize
	// try 3 times to turn GPS on
	for (tryCt = 0; tryCt < 3; tryCt++)	{
		PORTB |= (1<<PULSE_GPS); // pulse the GPS
		for (Timeout1 = 20; Timeout1; ); // apply a pulse duration of 200ms
		PORTB &= ~(1<<PULSE_GPS);
		for (Timeout1 = 100; Timeout1; ); // Wait 1sec
		if (Prog_status.serial_Received == 1) { // if we are receiving serial from the GPS
			return 0; // good to go
		}
	}
	// not receiving serial from GPS after 3 tries
	PORTB &= ~(1<<GPS_PWR); // turn GPS power circuit off
	return 1;
}

int gpsOff(void) {
	
	Prog_status.serial_Received = 0; // assure flag starts clear
	return 0; // stub
}
int parseNMEA(void) {
	char *endParsePtr, *parsePtr = recBuf;
	int fldCounter = sentenceType; // 0th NMEA field
	// cleanly get the current position of the NMEA buffer pointer
	cli();
	endParsePtr = recBufInPtr;
	sei();
	Prog_status.new_NMEA_Field = 1; // we are starting to work on the first field
	while (1) {
		if (parsePtr++ > endParsePtr) { // if we can't complete the parsing
			return 1; // exit, flag that it failed
		}
		if (*parsePtr == ',') { // the field delimiter
			if (Prog_status.new_NMEA_Field == 1) { // we just started working on a field and
				// encountered a comma indicating the next field, so the current field is empty
				NMEA_Ptrs[fldCounter] = NULL;
			}
			fldCounter++; // point to the next field
			Prog_status.new_NMEA_Field = 1; // working on a new field
			} else { // not a comma
			if (Prog_status.new_NMEA_Field == 1) { // we are starting a new field
				// we have a non-comma character, so the field contains something
				NMEA_Ptrs[fldCounter] = parsePtr; // plant the field pointer here
				Prog_status.new_NMEA_Field = 0; // the field is now no longer new
			}
		}
		if (fldCounter > sentenceType) { // if we've got sentence-type complete, test for "GPRMC"
			// optimize test to fail early if invalid
			// for testing break down conditions
			if (*(NMEA_Ptrs[sentenceType] + 4) != 'C') return 6;
			if (*(NMEA_Ptrs[sentenceType] + 3) != 'M') return 5;
			if (*(NMEA_Ptrs[sentenceType] + 2) != 'R') return 4;
			if (*(NMEA_Ptrs[sentenceType] + 1) != 'P') return 3;
			if (*(NMEA_Ptrs[sentenceType]) != 'G') return 2;
			// not a sentence type we can use
		}
		if (fldCounter > isValid) { // if we've got the validity char, test it
			if (*(NMEA_Ptrs[isValid]) != 'A') {
				return 9; // GPS says data not valid
			}
		}
		if (fldCounter > dateStamp) { // don't need any fields after this
			return 0;
		}
	}
}

void sendSetTimeSignal(void) {
	// this will be the usual tie-up point
	// transmit the set-time signal back to the main uC
	// then set flag(s) to signal this uC to shut down
	
	// for testing, send a dummy message
	/*
	// wait for the transmit buffer to be empty
	while (!(UCSR1A & (1<<TXC1))) { // bit is set when Tx shift register is empty
		;
	}
	// write a 1 to clear the Transmit Complete bit
	UCSR1A &= (1<<TXC1);
	*/
	cmdOutPtr = cmdOut;
	while (*cmdOutPtr != '\0') {
		while (!(UCSR1A & (1<<UDRE1))) { // Tx data register UDRn ignores any write unless UDREn=1
			;
		}
		UDR1 = *cmdOutPtr++; // put the character to be transmitted in the Tx buffer
	}
	// reset the following flag, to allow the next periodic diagnostics
	stateFlags &= ~(1<<isValidTimeRxFromGPS);
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
	
	// for testing, fake that we got a valid time signal
	// do this every 1.5 seconds
	if ((rouseCountdown % 150) == 0) {
		stateFlags |= (1<<isValidTimeRxFromGPS);
	}
		
	
	t = rouseCountdown;
	if (t) rouseCountdown = --t;
	
	//	if (--rouseCountdown <= 0)
	if (!rouseCountdown)
	{
		stateFlags &= ~(1<<isRoused);
	}


	
	n = Timeout1;						// 100Hz decrement timer 
	if (n) Timeout1 = --n;
	n = Timeout2;
	if (n) Timeout2 = --n;
	n = Timeout3;
	if (n) Timeout3 = --n;

}

ISR(USART0_RX_vect) {
	// occurs when USART0 Rx Complete
	// *recBufInPtr++ = UDR0; // basic task, put the character in the buffer
	char receiveByte = UDR0;
	if (receiveByte == '$') { // beginning of NMEA sentence
		recBufInPtr = recBuf; // point to start of buffer
	}
	*recBufInPtr++ = receiveByte; // put character in buffer
	Prog_status.serial_Received = 1; // flag that serial is being received
	// if overrun for some reason, wrap; probably bad data anyway and will be repeated
	if (recBufInPtr >= (recBuf + RX_BUF_LEN)) {
		recBufInPtr = recBuf; // wrap overflow to start of buffer
	}
}