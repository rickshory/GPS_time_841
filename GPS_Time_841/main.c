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
#define TX_BUF_LEN 64

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/sleep.h>

// function prototypes
void stayRoused(uint16_t dSec);
void endRouse(void);
void sendSetTimeSignal(void);
void sendDebugSignal(void);
int parseNMEA(void);
void restoreCmdDefault(void);

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
enum NMEA_fields {sentenceType=0, timeStamp, isValid, curLat, isNorthOrSouth, curLon, isEastOrWest,
speedKnots, trueCourse, dateStamp, magVar, varEastOrWest, checkSum};
//$GPRMC,000143.056,V, , , , , , ,191210,  ,  ,N*42
//0     ,1         ,2,3,4,5,6,7,8,9     ,10,11,12
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

static volatile union NMEA_status // NMEA status bit flags
// mostly salient for debugging
{
	unsigned char nmea_stat_char;
	struct
	{
		unsigned char got_lon_field:1;
		unsigned char got_time_field:1;
		unsigned char got_date_field:1;
		unsigned char is_nmea_string:1;
		unsigned char is_nmea_sentence:1;
		unsigned char is_gprmc_sentence:1;
		unsigned char valid_data:1;
		unsigned char valid_time_signal:1;
	};
	} NMEA_status = {0};

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
	isTimeForDebugDiagnostics, // for testing
	sfBit7 // unused
};

volatile uint8_t machineState, GpsOnAttempts = 0, GpsOffAttempts = 0;
volatile uint8_t stateFlags = 0;
//volatile uint8_t iTmp;
volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker
volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep
volatile uint8_t Timer1, Timer2, Timer3;	// 100Hz decrement timer

static volatile char receiveByte;
static volatile char recBuf[RX_BUF_LEN];
static volatile char *recBufInPtr;
static volatile char cmdOut[TX_BUF_LEN] = "t2016-03-19 20:30:01 -08\n\r\n\r\0";
static volatile char *cmdOutPtr;
static volatile char *NMEA_Ptrs[13]; // array of pointers to field positions within the captured NMEA sentence

static uint8_t i;
static char *d;
static volatile char timeOfFix[12];
static volatile char longitudeNum[9];
static volatile char longitudeEorW[2];
static volatile char dateOfFix[7];


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
#define TX0 PA1 // not currently used
#define RX0 PA2 // used to Rx from GPS
#define TX1 PA5 // used to Tx time signal
#define RX1 PA4 // not currently used
#define GPS_PWR PB0
#define PULSE_GPS PB1
#define TEST_FORCE_GPS_OFF PA0


int main(void)
{
	uint16_t GpsTxUbrr, UcRxUbrr;

	// set up to blink an LED
	DDRA |= (1<<LED);
	
	// set up Tx1 as an output
	DDRA |= (1<<TX1);

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
	//UCSR0D, leave default 0, do not let Rx wake this uC

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
	
	stayRoused(1000); // stay roused for 10 seconds

    while (1) 
    {
		if (!(stateFlags & (1<<isRoused))) {
			stayRoused(1000); // for testing, keep awake
/*			
			// go to sleep
			PORTA &= ~(1<<LED); // turn off pilot light blinkey
			
			// shut down UARTs
			UCSR0B = 0;
			UCSR1B = 0;
						
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
*/
		} // end of go-to-sleep

		// continue main program loop
		// debugging diagnostics, put flag characters into the output string
		if (Prog_status.serial_Received) {
			int n = parseNMEA();
			if (n==0) {
				stateFlags |= (1<<isValidTimeRxFromGPS);
				// in final version, turn off UART and GPS here
//			} else {
//				stateFlags &= ~(1<<isValidTimeRxFromGPS);
			}

		}
	
		// following will be the usual exit point
		// calls a function to send the set-time signal back to the main uC
		// that function, if successful, will tie things up and end Rouse mode
		// which will allow this uC to shut down till woken again by Reset

		if (stateFlags & (1<<isValidTimeRxFromGPS)) {
			sendSetTimeSignal();
		} else {
			// for testing, insert diagnostics
			if (stateFlags & (1<<isTimeForDebugDiagnostics)) {
				sendDebugSignal();
			}
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

int parseNMEA(void) {
	char *endParsePtr, *parsePtr = (char*)recBuf;
	int fldCounter = sentenceType; // 0th NMEA field
	NMEA_status.nmea_stat_char = 0; // clear all flags; most are only salient for debugging
	// cleanly get the current position of the NMEA buffer pointer
	cli();
	endParsePtr = (char*)recBufInPtr;
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
		if (!(NMEA_status.is_gprmc_sentence)) {
			if (fldCounter > sentenceType) { // if we've got sentence-type complete, test for "GPRMC"
				NMEA_status.is_nmea_sentence = 1;
				// optimize test to fail early if invalid
				// for testing break down conditions
				if (*(NMEA_Ptrs[sentenceType] + 4) != 'C') return 6;
				if (*(NMEA_Ptrs[sentenceType] + 3) != 'M') return 5;
				if (*(NMEA_Ptrs[sentenceType] + 2) != 'R') return 4;
				if (*(NMEA_Ptrs[sentenceType] + 1) != 'P') return 3;
				if (*(NMEA_Ptrs[sentenceType]) != 'G') return 2;
				// not a sentence type we can use
				NMEA_status.is_gprmc_sentence = 1;
			}
		}
		if (!(NMEA_status.valid_data)) {
			if (fldCounter > isValid) { // if we've got the validity char, test it
				if (*(NMEA_Ptrs[isValid]) == 'A') {
					NMEA_status.valid_data = 1;
				} else {
					return 9; // GPS says data not valid
				}
			}			
		}

		// copy needed parameters into their own strings; may not ultimately do it this way
		switch (fldCounter) {
			case timeStamp:
				d = (char*)timeOfFix;
//				NMEA_status.got_time_field = 1;
				break;
			case curLon:
				d = (char*)longitudeNum;
//				NMEA_status.got_lon_field = 1;
				break;
			case isEastOrWest:
				d = (char*)longitudeEorW;
				break;
			case dateStamp:
				d = (char*)dateOfFix;
//				NMEA_status.got_date_field = 1;
				break;
			default:
				d = NULL;
				break;
		}
		if (d != NULL) {
			switch (fldCounter) {
				case timeStamp:
					NMEA_status.got_time_field = 1;
					break;
				case dateStamp:
					NMEA_status.got_date_field = 1;
					break;
			}
			i = 0;
			while (*(NMEA_Ptrs[fldCounter]+i) != ',') {
				d[i] = (*(NMEA_Ptrs[fldCounter]+i));
				i++;
			}
			d[i] = '\0';
		}
		
		if (fldCounter > dateStamp) { // don't need any fields after this
			// when GPS starts up, it gives the date string as "181210" and 
			// time string as "235846.nnn", incrementing every second so 
			// time soon rolls over to "000000.nnn" and date to "191210"
//			if ((NMEA_status.valid_data) && (NMEA_status.got_date_field) && (NMEA_status.got_time_field)) {
			if ((NMEA_status.got_date_field) && (NMEA_status.got_time_field)) {
				NMEA_status.valid_time_signal = 1;
				return 0;
			} else {
				return 10;
			}
			
			
		}
	}
}


void sendSetTimeSignal(void) {
	// this will be the usual tie-up point
	// transmit the set-time signal back to the main uC
	// then set flag(s) to signal this uC to shut down

	// build the set-time command
	// target format: "t2016-03-19 20:30:01 -08\n\r\n\r\0"
	// NMEA format examples
	// 190316       Date of fix  19 March 2016
	// 203001       Time of fix 20:30:01 UTC
	// rearrange date digits
	if (dateOfFix[4] == '\0') {
		cmdOut[3] = 'x';
	} else {
		cmdOut[3] = dateOfFix[4];
	}
	// try this kludge
//	*(cmdOutPtr + 3) = *(NMEA_Ptrs[dateStamp] + 4);
	if (dateOfFix[5] == '\0') {
		cmdOut[4] = 'x';
		} else {
		cmdOut[4] = dateOfFix[5];
	}
	if (dateOfFix[2] == '\0') {
		cmdOut[6] = 'x';
		} else {
		cmdOut[6] = dateOfFix[2];
	}
	if (dateOfFix[3] == '\0') {
		cmdOut[7] = 'x';
		} else {
		cmdOut[7] = dateOfFix[3];
	}
	if (dateOfFix[0] == '\0') {
		cmdOut[9] = 'x';
		} else {
		cmdOut[9] = dateOfFix[0];
	}
	if (dateOfFix[1] == '\0') {
		cmdOut[10] = 'x';
		} else {
		cmdOut[10] = dateOfFix[1];
	}
	// rearrange time digits
/*
	if (timeOfFix[0] == '\0') {
		cmdOut[12] = 'x';
	} else {
		cmdOut[12] = timeOfFix[0];
	}
	if (timeOfFix[1] == '\0') {
		cmdOut[13] = 'x';
		} else {
		cmdOut[13] = timeOfFix[1];
	}
	if (timeOfFix[2] == '\0') {
		cmdOut[15] = 'x';
		} else {
		cmdOut[15] = timeOfFix[2];
	}
	if (timeOfFix[3] == '\0') {
		cmdOut[16] = 'x';
		} else {
		cmdOut[16] = timeOfFix[3];
	}
	if (timeOfFix[4] == '\0') {
		cmdOut[18] = 'x';
		} else {
		cmdOut[18] = timeOfFix[4];
	}
	if (timeOfFix[5] == '\0') {
		cmdOut[19] = 'x';
		} else {
		cmdOut[19] = timeOfFix[5];
	}

*/
	// try this kludge
	if (recBuf[7] == '\0') {
		cmdOut[12] = 'x';
		} else {
		cmdOut[12] = recBuf[7];
	}
	
	if (recBuf[8] == '\0') {
		cmdOut[13] = 'x';
		} else {
		cmdOut[13] = recBuf[8];
	}
	if (recBuf[9] == '\0') {
		cmdOut[15] = 'x';
		} else {
		cmdOut[15] = recBuf[9];
	}
	if (recBuf[10] == '\0') {
		cmdOut[16] = 'x';
		} else {
		cmdOut[16] = recBuf[10];
	}
	if (recBuf[11] == '\0') {
		cmdOut[18] = 'x';
		} else {
		cmdOut[18] = recBuf[11];
	}
	if (recBuf[12] == '\0') {
		cmdOut[19] = 'x';
		} else {
		cmdOut[19] = recBuf[12];
	}
	// for now, always use UTC
	cmdOut[21] = '+';
	cmdOut[22] = '0';
	cmdOut[22] = '0';

	cmdOutPtr = cmdOut;	
	while (*cmdOutPtr != '\0') {
		while (!(UCSR1A & (1<<UDRE1))) { // Tx data register UDRn ignores any write unless UDREn=1
			;
		}
		UDR1 = *cmdOutPtr++; // put the character to be transmitted in the Tx buffer
	}

	Prog_status.serial_Received = 0; // clear the flag that says serial has been received
}

void sendDebugSignal(void) {
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
	// debugging diagnostics, put flag characters into the output string
	if (Prog_status.serial_Received) {
		*cmdOutPtr = 'r';
		// diagnostics; put the result code of parsing attempt into the output string
		// shove some flag characters in
		if (NMEA_status.got_lon_field) 
			*(cmdOutPtr + 2) = 'L';
		if (NMEA_status.got_time_field)
			*(cmdOutPtr + 3) = 'T';
		if (NMEA_status.got_date_field)
			*(cmdOutPtr + 4) = 'D';
		if (NMEA_status.is_nmea_string)
			*(cmdOutPtr + 5) = 'S';
		if (NMEA_status.is_gprmc_sentence)
			*(cmdOutPtr + 6) = 'G';
		if (NMEA_status.valid_data)
			*(cmdOutPtr + 7) = 'V';
/*
		unsigned char got_lon_field:1;
		unsigned char got_time_field:1;
		unsigned char got_date_field:1;
		unsigned char is_nmea_string:1;
		unsigned char is_nmea_sentence:1;
		unsigned char is_gprmc_sentence:1;
		unsigned char valid_data:1;
		unsigned char valid_time_signal:1;
		
		*(cmdOutPtr + 1 + n) = ('A' + n);
*/		

	}
	while (*cmdOutPtr != '\0') {
		while (!(UCSR1A & (1<<UDRE1))) { // Tx data register UDRn ignores any write unless UDREn=1
			;
		}
		UDR1 = *cmdOutPtr++; // put the character to be transmitted in the Tx buffer
	}
	// reset the following flag, to allow the next periodic diagnostics
	stateFlags &= ~(1<<isTimeForDebugDiagnostics);
	// after testing diagnostics, put the string back as it was
	restoreCmdDefault();
	Prog_status.serial_Received = 0; // clear the flag that says serial has been received
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
		stateFlags |= (1<<isTimeForDebugDiagnostics);
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

void restoreCmdDefault(void) {
	// restore the output buffer to its default 't2016-03-19 20:30:01 -08'
	cmdOut[0] = 't';
	cmdOut[1] = '2';
	cmdOut[2] = '0';
	cmdOut[3] = '1';
	cmdOut[4] = '6';
	cmdOut[5] = '-';
	cmdOut[6] = '0';
	cmdOut[7] = '3';
	cmdOut[8] = '-';
	cmdOut[9] = '1';
	cmdOut[10] = '0';
	cmdOut[11] = ' ';
	cmdOut[12] = '2';
	cmdOut[13] = '0';
	cmdOut[14] = ':';
	cmdOut[15] = '3';
	cmdOut[16] = '0';
	cmdOut[17] = ':';
	cmdOut[18] = '0';
	cmdOut[19] = '1';
	cmdOut[20] = ' ';
	cmdOut[21] = '-';
	cmdOut[22] = '0';
	cmdOut[23] = '8';
	cmdOut[24] = '\n';
	cmdOut[25] = '\r';
	cmdOut[26] = '\n';
	cmdOut[27] = '\r';
	cmdOut[28] = '\0';
}