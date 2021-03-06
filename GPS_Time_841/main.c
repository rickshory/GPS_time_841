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
#define GPS_RX_BUF_LEN 128
#define MAIN_RX_BUF_LEN 32
#define MAIN_TX_BUF_LEN 64
#define GPS_RX_TIMEOUT 150 // 100 ticks = 1 second
#define ADC_SAMPLES_TO_AVERAGE_PWR_2 6 // e.g. 3 means 2^3=8, 5 means 2^5=32
// works for 0 to 6 but >=7 overflows 16 bit cumulative value
#define CELL_V_OK_FOR_GPS 570 // determined empirically
// this level, 1.2246V, should always have enough power to complete the cycle

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/sleep.h>

typedef struct {
	char * const buffer;
	int head;
	int tail;
	const int maxLen;
} circBuf_t;

#define CIRCBUF_DEF(x,y) char x##_space[y]; circBuf_t x = { x##_space,0,0,y}

// function prototypes
uint16_t readCellVoltage(void); // don't need any parameters, always fills global cellVoltageReading
void stayRoused(uint16_t dSec);
void endRouse(void);
void sendSetTimeSignal(void);
void sendDebugSignal(void);
int parseNMEA(void);
void restoreCmdDefault(void);
int circBufPut(circBuf_t *c, char d);
int circBufGet(circBuf_t *c, char *d);
void delay_ms(uint16_t x);


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

static volatile union Prog_status // Program status bit flags
{
	unsigned char status;
	struct
	{
		unsigned char gps_Request_Active:1;
		unsigned char gps_Power_Enabled:1;
		unsigned char gps_Being_Pulsed:1;
		unsigned char listen_To_GPS:1;
		unsigned char parse_timeout_started:1;
		unsigned char wait_for_main_Rx_started:1;
		unsigned char main_serial_Received:1;
		unsigned char gps_serial_Received:1;
	};
} Prog_status = {0};

static volatile union NMEA_status // NMEA status bit flags
{
	unsigned char nmea_stat_char;
	struct
	{
		unsigned char use_nmea:1;
		unsigned char valid_data:1;
		unsigned char captureNMEA:1;
		unsigned char bufferFull:1;
		unsigned char valid_timestamp:1;
		unsigned char valid_datestamp:1;
		unsigned char bit6:1;
		unsigned char bit7:1;
	};
} NMEA_status = {0};

static volatile union cmd_status // main Rx/Tx command status bit flags
{
	unsigned char cmd_stat_char;
	struct
	{
		unsigned char use_cmd:1;
		unsigned char valid_cmd_data:1;
		unsigned char capture_cmd:1;
		unsigned char buffer_full:1;
		unsigned char pwr_gps_on:1;
		unsigned char pulse_gps:1;
		unsigned char rx1_disabled:1;
		unsigned char bit7:1;
	};
	} cmd_status = {0};

enum machStates
{
	Asleep = 0, // default on chip reset
	CheckingCellVoltage, // verify there is enough power to do the job
	Initializing,  // setting up timer, usarts, etc.
	WaitingForMain, // waiting to Rx serial from main uC, prevents run-on if not in-system
	TurningOnGPS, // attempting to power up and wake the GPS module
	ParsingNMEA, // attempting to extract a valid time signal from GPS NMEA data
	TurningOffGPS, // attempting to systematically shut down the GPS module
	ShuttingDown // preparing for sleep; sub-states depend of status flags
};

static volatile union stateFlags // status bit flags
{
	unsigned char stat_char;
	struct
	{
		unsigned char isRoused:1; // triggered by initiating Reset, and re-triggered by any activity while awake
		unsigned char reRoused:1; // re-triggered while awake, used to reset timeout
		unsigned char isGPSPowerOn:1; // is the GPS powered on
		unsigned char bit3:1; //
		unsigned char isValid_NMEA_RxFromGPS:1; // has some valid NMEA data been received from the GPS
		unsigned char isValidTimeRxFromGPS:1; // has a valid Timestamp been received from the GPS
		unsigned char setTimeCommandSent:1; // final set-time command has been sent
		unsigned char isTimeForDebugDiagnostics:1; // for testing
	};
} stateFlags = {0};

// union allows writing in Lo and Hi bytes of ADC, and reading out whole word
typedef struct {
	union {
		struct {
			uint8_t adcLoByte, adcHiByte; // this is the correct endian-ness
		};
		struct  {
			uint16_t adcWholeWord;
		};
	};
} adcData;

volatile adcData cellVoltageReading;
volatile uint8_t machineState = Asleep, GpsOnAttempts = 0;
volatile uint8_t ToggleCountdown = TOGGLE_INTERVAL; // timer for diagnostic blinker
volatile uint16_t rouseCountdown = 0; // timer for keeping system roused from sleep
volatile uint8_t gpsTimeoutCountdown = 0; // track if serial Rx from GPS
volatile uint16_t Timer1;	// 100Hz decrement timer, available for general use
static volatile char cmdOut[MAIN_TX_BUF_LEN] = 
	"x2016-03-19 20:30:01 -08\n\r\n\r0000.000000 00000.000000 \n\r\n\r\0"; // default, for testing
// latitude specifier is character 'N' or 'S'
// latitude, raw from NMEA is whole degrees (2 digits, padded with leading zeros) concatenated with decimal minutes
// longitude specifier is character 'E' or 'W'
// longitude, raw from NMEA is whole degrees (3 digits, padded with leading zeros) concatenated with decimal minutes
static volatile char cmdClear[6] = "\n\r\n\r\0"; // empty command, to clear the main uC buffer
static volatile char *cmdOutPtr, *cmdClearPtr;
static volatile int nmea_capture_counter;
static volatile int cmd_capture_ctr = 0;
static volatile int fldCounter;
static volatile int posCounter;

CIRCBUF_DEF(gps_recBuf, GPS_RX_BUF_LEN);
CIRCBUF_DEF(main_recBuf, MAIN_RX_BUF_LEN);

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
#define RX1 PA4 // used to Rx from main uC, to verify connected
#define GPS_PWR PB0
#define PULSE_GPS PB1
#define TEST_FORCE_GPS_OFF PA0


int main(void)
{
	// even if battery voltage is too low, or unconnected
	// give 5ms pilot-light flash, to let user know something happened
	// set up LED, which will blink during normal operation
	DDRA |= (1<<LED); // set pin as output
	PORTA &= ~(1<<LED); // initially force low
	asm volatile ("nop"); // let settle
	PORTA |= (1<<LED); // turn on
	delay_ms(5); // wait
	PORTA &= ~(1<<LED); // turn off
	
	machineState = CheckingCellVoltage;
	if (readCellVoltage() < CELL_V_OK_FOR_GPS) { // determined empirically
		machineState = ShuttingDown;
		goto skipInitialization;
	}
	machineState = Initializing;
	uint16_t GpsTxUbrr, UcRxUbrr;

	
	// set up GPS control lines as outputs
	DDRB |= (1<<PULSE_GPS); // set pin as output
	PORTB &= ~(1<<PULSE_GPS); // initially force low
	DDRB |= (1<<GPS_PWR); // set pin as output
	PORTB &= ~(1<<GPS_PWR); // initially force low
	
	// set up Tx1 as an output
	DDRA |= (1<<TX1);

	// set up a heartbeat counter, interrupt every 0.1 second
	// set up 16-bit TIMER1
	// F_CPU = 8MHz
	OCR1A = CT_10MS; // 0.01 sec; use prescaler 64
	// only channel A will cause CTC

	// note that WGM1[3:0] is split over TCCR1A and TCCR1B
	// WGM1[3:2] is TCCR1B[4:3] while WGM1[1:0] is TCCR1A[1:0]

	// TCCR1A  Timer/Counter1 Control Register A
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
	
	// TCCR1B  Timer/Counter1 Control Register B
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

	// TCCR1C  Timer/Counter1 Control Register C
	// 7 FOC1A: Force Output Compare for Channel A
	// 6 FOC1B: Force Output Compare for Channel B
	// (5:0 reserved)
	// all unused, leave default of zero
	//TCCR1C = 0;

	// TIMSK1  Timer/Counter 1 Interrupt Mask Register
	// (7, 6, 4, 3 reserved)
	//  When these bits are written to one, and the I-flag in the Status Register is set (interrupts globally
	//  enabled), the corresponding interrupt is enabled. The
	//  corresponding Interrupt Vector is executed when the
	//  corresponding bit is set in the Timer/Counter Interrupt Flag Register  TIFR1.
	// 5 ICIE1: Timer/Counter, Input Capture Interrupt Enable (not used here)
	// 2 OCIE1B: Timer/Counter1 Output Compare Match B Interrupt Enable (not used here)
	// 1 OCIE1A: Timer/Counter1 Output Compare Match A Interrupt Enable
	// 0 TOIE1: Timer/Counter1, Overflow Interrupt Enable (not used here)
	
	// Set up USARTS
	// make sure enabled, no power reductions for USARTS
	// PRR  Power Reduction Register
	// 7  PRTWI: Power Reduction Two-Wire Interface
	// 6  PRUSART1: Power Reduction USART1
	// 5  PRUSART0: Power Reduction USART0
	// 4  PRSPI: Power Reduction SPI
	// 3  PRTIM2: Power Reduction Timer/Counter2
	// 2  PRTIM1: Power Reduction Timer/Counter1
	// 1  PRTIM0: Power Reduction Timer/Counter0
	// 0  PRADC: Power Reduction ADC
	
	// setting bits=0 assures power reduction is OFF
	PRR &= ~((1<<PRUSART1) | (1<<PRUSART0));
	
	// UCSRnC USART Control and Status Register C
	// 7:6 use default (00)= Asynchronous USART
	// 5:4 use default (00)= Parity disabled
	// 3 use default, 0= 1 stop bit
	// 2:1  UCSZn[1:0]: Character Size, 
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
	// 7  RXCIE0: RX Complete Interrupt Enable
	// 4  RXEN0: Receiver Enable
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0);
	// set USART0 frame format: 8data, 1stop bit
	UCSR0C = (3<<UCSZ00);
	//UCSR0D, leave default 0, do not let Rx wake this uC

	// Set USART1 to Tx/Rx with the main uC
	// set USART1 baud rate
	UcRxUbrr = 51; // (F_CPU/(16 * UC_RX_BAUD))-1 for main uC Rx at 9600 baud
	UBRR1H = (unsigned char)(UcRxUbrr>>8);
	UBRR1L = (unsigned char)UcRxUbrr;
	// set USART1 to transmit and receive
	// UCSR1A, use defaults
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1); // use defaults except for these
	// set USART1 frame format: 8data, 1stop bit
	UCSR1C = (3<<UCSZ10);
	//UCSR1D, leave default 0, do not let Rx wake this uC
	PUEA |= (1<<RX1);
	// pull up Rx1; otherwise if unconnected can get bleed-through from serial diagnostics
	
	//REMAP[0]  U0MAP: USART0 Pin Mapping
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
	
	// set initial conditions for capturing serial NMEA data
	NMEA_status.captureNMEA = 0; // do not even capture to buffer until first sentence found
	NMEA_status.use_nmea = 0; // do not try to parse until start-of-sentence found
	NMEA_status.valid_data = 0; // not valid data yet
		
	machineState = WaitingForMain; // waiting to Rx serial from main uC, prevents run-on if not in-system
	// in-system can be emulated by sending any serial at 9600 baud into this chip's Rx1
	
	skipInitialization:
    while (1) {
		nextIteration:
		
		if (machineState == WaitingForMain) { 
			// wait 30 seconds to Rx serial from main board; prevents run-on if not connected to anything
			if (!Prog_status.wait_for_main_Rx_started) {
				stayRoused(3000); // stay roused for 30 seconds
				Prog_status.wait_for_main_Rx_started = 1;
			}
			
			if (Prog_status.main_serial_Received) {
				machineState = TurningOnGPS;
				endRouse();
				goto nextIteration;
			}
			// if nothing receive in 30 seconds, shut down
			if (!(stateFlags.isRoused)) {
				machineState = ShuttingDown;
				goto nextIteration;
			}
		} // end of (machineState == WaitingForMain)
		
		if (machineState == TurningOnGPS) {
			if (Prog_status.gps_serial_Received) { // if GPS serial is being receive, a pulse successfully woke the GPS
				if (!(Prog_status.gps_Being_Pulsed)) { // not in the midst of being pulsed
					endRouse(); // end any rouse
					machineState = ParsingNMEA;
					goto nextIteration;
				}
			}			
			if (stateFlags.isRoused) { // this section is rouse-based
				goto nextIteration; // if roused, don't do any of the following
			}
			if (!(stateFlags.isGPSPowerOn)) { // enable power to GPS
				PORTB |= (1<<GPS_PWR); // raise the pin that enables power to the GPS module
				stayRoused(200); // rouse for 2 seconds for power to settle
				GpsOnAttempts = 0; // define and set to zero
				stateFlags.isGPSPowerOn = 1; // flag that power is now on
			} else { // by either bypassing or entering above block, GPS power is now on
				// try 3 times to wake GPS
				// if successful, go to (machineState = ParsingNMEA)
				// if not, shut down
				if (GpsOnAttempts > 3) {
					PORTB &= ~(1<<GPS_PWR); // turn off physical power to GPS module
					stateFlags.isGPSPowerOn = 0; // flag that power is off
					machineState = ShuttingDown;
					goto nextIteration;
				}
				if (!(Prog_status.gps_Being_Pulsed)) { // not being pulsed
					// initiate a pulse
					PORTB |= (1<<PULSE_GPS); // set high
					stayRoused(20); // pulse duration, about 0.2 sec
					Prog_status.gps_Being_Pulsed = 1;
				} else { // just finished a pulse
					PORTB &= ~(1<<PULSE_GPS); // set low
					GpsOnAttempts++; // tally another attempt made
					stayRoused(500); // wait for up to 5 sec before trying again
					Prog_status.gps_Being_Pulsed = 0;
				}
				goto nextIteration;
			} // end test of GPS power on
		} // end of (machineState == TurningOnGPS)

		if (machineState == ParsingNMEA) { 
			if (!(Prog_status.gps_serial_Received)) { // GPS has failed
				PORTB &= ~(1<<GPS_PWR); // turn off physical power to GPS module
				machineState = ShuttingDown; // begin shut down
				goto nextIteration;
			}

			// try for 3 minutes = 180 sec = 18000 ticks 10ms each
			// can do within uint16_t max 65535
			if (!Prog_status.parse_timeout_started) {
				stayRoused(18000); // stay roused for 3 minutes
				Prog_status.parse_timeout_started = 1;
			}

			// already tested gps_serial_Received 
			int n = parseNMEA();
			if (n==0) {
				stateFlags.isValidTimeRxFromGPS = 1;
			}

			if (stateFlags.isValidTimeRxFromGPS) {
				if (!(stateFlags.setTimeCommandSent)) {
					// only send signal once
					sendSetTimeSignal();
					stateFlags.setTimeCommandSent = 1; // flag that it is done
					machineState = TurningOffGPS;
					goto nextIteration;
				} 
			} // end stateFlags.isValidTimeRxFromGPS
			
			// no valid time within 3 minutes, begin shutdown
			if (!(stateFlags.isRoused)) {
				machineState = TurningOffGPS;
				goto nextIteration;
			}			
		} // end machineState == ParsingNMEA
		
		if (machineState == TurningOffGPS) {
			// try 3 times to properly turn GPS off
			// if successful, shut down
			// if not, risk shutting down anyway, nothing else to do
			uint8_t GpsOffAttempts;
			for (Timer1 = 100; Timer1; );	// wait for 1 second initially
			for( GpsOffAttempts = 0; GpsOffAttempts <= 3; GpsOffAttempts++ ){
				PORTB |= (1<<PULSE_GPS); // set high
				for (Timer1 = 20; Timer1; );	// wait for 200ms
				PORTB &= ~(1<<PULSE_GPS); // set low
				for (Timer1 = 200; Timer1; );	// wait for 2 second
				// Prog_status.gps_serial_Received flag cleared automatically by timeout
				if (!Prog_status.gps_serial_Received) { // GPS is no longer sending NMEA
					PORTB &= ~(1<<GPS_PWR); // turn off physical power to GPS module
					machineState = ShuttingDown; // OK to shut down
					goto nextIteration;
				}
			}
			if (GpsOffAttempts >= 3) { // failed to shut down properly
				machineState = ShuttingDown; // risk shut down anyway, nothing else to do
				PORTB &= ~(1<<GPS_PWR); // turn off physical power to GPS module
			}
		} // end machineState == TurningOffGPS

		if (machineState == ShuttingDown) {
			// go to sleep
			endRouse();
			PORTA &= ~(1<<LED); // turn off pilot light blinkey
			
			// shut down UARTs
			UCSR0B = 0;
			UCSR1B = 0;
						
			// before setting PRADC (PRR[0], below) assure ADC is disabled
			// may not be necessary, if ADC never enabled, but assures lowest power
			// ADCSRA  ADC Control and Status Register A
			// 7 - ADEN: ADC Enable. Writing this bit to zero turns off the ADC
			ADCSRA &= ~(1<<ADEN);
			
			// PRR  Power Reduction Register
			// 7  PRTWI: Power Reduction Two-Wire Interface
			// 6  PRUSART1: Power Reduction USART1
			// 5  PRUSART0: Power Reduction USART0
			// 4  PRSPI: Power Reduction SPI
			// 3  PRTIM2: Power Reduction Timer/Counter2
			// 2  PRTIM1: Power Reduction Timer/Counter1
			// 1  PRTIM0: Power Reduction Timer/Counter0
			// 0  PRADC: Power Reduction ADC
			
			// shut down any peripheral clocks by writing all 1s
			// may not be necessary in power-down sleep mode, but assures lowest power
			PRR = 0xff;
			
			// To enter a sleep mode, the SE bit in MCUCR must be set and 
			// a SLEEP instruction must be executed. The SMn bits in
			// MCUCR select which sleep mode will be activated by 
			// the SLEEP instruction.
			
			// MCUCR  MCU Control Register
			// (7, 6, 2 reserved)
			// 5 - SE: Sleep Enable
			// 4:3  SM[1:0]: Sleep Mode Select Bits 1 and 0
			//       SM[1:0]=(10) for Power-down
			// 1:0  ISC0[1:0]: Interrupt Sense Control 0 Bit 1 and Bit 0
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
		} // machineState == ShuttingDown

		// continue main program loop
    } // end of 'while(1)' main program loop
}

/*
enable ADC
do not initiate conversion yet
select Single Conversion Mode
enable ADC conversion complete interrupt
enter ADC Noise Reduction mode
The ADC will start a conversion once the CPU has been halted.
*/

uint16_t readCellVoltage() {
	uint16_t sumOfReadings = 0;
	// set initial conditions; conversion-complete interrupt will fill in these values
	cellVoltageReading.adcWholeWord = 0;
	// clear the ADC power reduction bit
	PRR &= ~(1<<PRADC);
	// enable the ADC
	ADCSRA |= (1<<ADEN);
	// cell voltage to be measured is only about 1.4V max, so
	// select ADC voltage reference, 2.2V internal reference, disconnected from external AREF pin
	// REFS[2:0] = 010, these are ADMUXB[7:5]
	// Gain Selection, set the gain for differential inputs, not relevant, use gain of 1
	// GSEL[1:0] = 00, these are ADMUXB[1:0]
//	ADMUXB |= (1<<REFS1);
	ADMUXB = 0b01000000;
	// After switching to internal voltage reference the ADC requires a settling time of 1ms before
	// measurements are stable. Conversions starting before this may not be reliable. The ADC must 
	// be enabled during the settling time.
	for(uint16_t i=(F_CPU/1000); i; i-- )
		asm ("nop"); // loop should elapse well over 1ms
	
	// select ADC0 as the input; PA0, pin 13 in SOIC package, pin 5 in QFN package
	// single ended, (voltage ref to ground)
	// MUX[5:0] = 000000, these are ADMUXA[5:0]
	ADMUXA = 0b00000000;
	
	// Digital Input Disable
	// ADC[7:0], these are DIDR0[7:0]
	// set ADCnD bit to reduce power consumption; ADC0D is for ADC0, the ADC we are using here
	DIDR0 |= (1<<ADC0D);
	
	// set ADC prescaler, ADPS bits in ADCSRA
	// ADPS[2:0], these are ADCSRA[2:0]
	// using 8MHz CPU clock:
	//	desired ADC clock	calculated division factor
	// min	  50kHz				160
	// max	 200kHz				 40
	// so, a division factor of either 64 (125kHz) or 128 (62.5kHz) would be OK
	// using 128, ADPS[2:0] = 111
	//ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
	// using 64, ADPS[2:0] = 110
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1));
	// enable ADC Conversion Complete Interrupt
	// ADIE (ADC Interrupt Enable, ADCSRA[3])
	ADCSRA |= (1<<ADIE);

	// ADC Left Adjust Result
	// ADLAR is ADCSRB[3], leave as default 0 to right-adjust
	// ADC Auto Trigger Source
	// ADTS[2:0], these are ADCSRB[2:0]; leave as default 000 for Free Running mode
	// ADTS[2:0] are ignored if ADATE (ADCSRA[5]) is clear; it is left as default clear in this usage
	// ADC will be done as a Single Conversion by setting ADSC (ADC Start Conversion, ADCSRA[6])
	// ADCSRB = 0b00000000;
	
	// Enter ADC Noise Reduction mode (or Idle mode). The ADC will start a conversion once the CPU has been halted.
	// Do ADC work before enabling any other interrupts. Otherwise, other ISRs may wake the CPU first and so the
	// ADC interrupt would complete with the CPU in active mode, higher noise.
	// When the ADC conversion completes, the ADC interrupt will wake up the CPU and
	// execute the ADC Conversion Complete interrupt routine.
	
	// get a number or readings, and average them
	// 26 ADC cycles for 1st conversion when ADEN 1st set, 15 cycles after that
	// for consistency, switch ADEN off/on for every conversion
	// (sec/125*10^3 cycles) * (26 cycles/conversion) = 208us
	// 64 conversions = 13.3ms
	// 32 conversions = 6.66ms
	// 16 conversions = 3.33ms
	// 8 conversions = 1.66ms	
	uint8_t samplesToAverage = (1<<ADC_SAMPLES_TO_AVERAGE_PWR_2);
	for (uint8_t ct=0; ct<samplesToAverage; ct++) {
		
		// temporarily disable ADC, to start fresh
		ADCSRA &= ~(1<<ADEN);
		// enable ADC conversion complete interrupt
		ADCSRA |= (1<<ADIE);
		// clear the conversion complete interrupt flag
		ADCSRA |= (1<<ADIF); // writing a 1 clears this flag
		// global enable interrupts
		sei();
		// enable ADC
		ADCSRA |= (1<<ADEN);
		// initiate a single conversion
		ADCSRA |= (1<<ADSC);
		// enter CPU Noise Reduction mode
		// SE bit in MCUCR must be written to logic one and a SLEEP
		//  instruction must be executed.
		// Sleep mode select
		// SM[1:0], these are MCUCR[4:3]; set to 01 for ADC Noise Reduction mode
		// MCUCR[1:0] are ICSR[1:0], Interrupt Sense Control, not relevant, can be 00
		// set Noise Reduction sleep mode, wait to set SE, don't care about any other bits
		MCUCR = 0b00001000;
		// set SE (sleep enable)
		MCUCR |= (1<<SE);
		// go intoPower-down mode SLEEP
		asm("sleep");
		// the ADC interrupt will wake up the CPU and execute the ADC Conversion Complete ISR
		
		// ADSC bit stays high as long as the conversion is in progress and will be 
		// cleared by hardware when the conversion is completed.
		while (ADCSRA & (1<<ADSC))
			; // should break out of this when conversion is complete
		// the ADC interrupt that occurred only disabled the ADC interrupt, so
		//  no more will occur until explicitly re-enabled
		
		// get the reading
		cellVoltageReading.adcLoByte = ADCL; // read ADCL first, then ADCH, to ensure that the content
		cellVoltageReading.adcHiByte = ADCH; // of the Data Registers belongs to the same conversion
		// formula from datasheet:
		// ADC = (Vin * 1024) / Vref
		// rearranges to:
		// Vin = (ADC * Vref) / 1024
		sumOfReadings += cellVoltageReading.adcWholeWord;
	} // finished getting all the readings we are going to average
	
	// done, disable the ADC
	ADCSRA &= ~(1<<ADEN);
	// done, set the ADC power reduction bit
	PRR |= (1<<PRADC);
	// done, clear SE to prevent SLEEP by accident
	MCUCR &= ~(1<<SE);
	// shift right to divide by 2-to-the-power and create the average
	return (uint16_t)(sumOfReadings >> ADC_SAMPLES_TO_AVERAGE_PWR_2);
}

void stayRoused(uint16_t dSec)
{
	cli(); // temporarily disable interrupts to prevent Timer1 from
	// changing the count partway through
	if ((dSec) > rouseCountdown) { // never trim the rouse interval, only extend it
		rouseCountdown = dSec;
	}
	stateFlags.isRoused = 1;
	PORTA |= (1<<LED); // set pilot light on
	sei();
}

void endRouse(void) {
	cli(); // temporarily disable interrupts to prevent Timer1 from
	// changing the count partway through
	rouseCountdown = 0;
	stateFlags.isRoused = 0;
	PORTA &= ~(1<<LED); // force pilot light off
	sei();
}

int parseNMEA(void) {
	// when GPS starts up, it gives the date string as "181210" and
	// time string as "235846.nnn", incrementing every second so
	// time soon rolls over to "000000.nnn" and date to "191210"
	// This means the GPS will be providing readable dates/times before they are valid
	// Parse them here, to use for diagnostics, but don't send the final set-time signal
	//  until the GPS signals that data are valid
	char ch;
	while (1) {
		if (circBufGet(&gps_recBuf, &ch)) { // buffer is empty
			return 1;
		}
		if (ch == '$') { // NMEA sentences begin with '$'
			NMEA_status.use_nmea = 1; // tentatively, the NMEA string is one to use
			fldCounter = sentenceType; // this first field is 'sentenceType'
			posCounter = -1; // start of field, but this initiator is not parsed yet
		}
		if (NMEA_status.use_nmea) { // most characters will be thrown away, skipping this loop
			if (ch == 0x0d) { // terminate at newline
				NMEA_status.use_nmea = 0; // skip characters till next '$' found
				// only do the following after a completed line that is flagged as valid
				if (NMEA_status.valid_data) { // GPS says data valid
					return 0; // we are done, begin shutdown
				} // else continue getting characters
			} else { // not end-of-line
				if (ch == ',') { // field delimiter
					fldCounter++; // refer to the next field in order
					posCounter = -1; // start of field, but this delimiter is not part of it
				} else { // a regular data character
					posCounter++; // refer to the next position within the current field
					// test various field/position details
					switch (fldCounter) {
						case sentenceType: // check that sentence type is "GPRMC"; all begin with "GP"
							// if not "GPRMC", ignore characters till next '$' found
							if ((posCounter == 3) && (ch != 'R')) NMEA_status.use_nmea = 0;
							if ((posCounter == 4) && (ch != 'M')) NMEA_status.use_nmea = 0;
							if ((posCounter == 5) && (ch != 'C')) NMEA_status.use_nmea = 0; 
							break;
						case timeStamp: // "hhmmss"; copy, in between ':' positions
							NMEA_status.valid_timestamp = 0; // flag off while getting current timestamp
							if (posCounter == 0) cmdOut[12] = ch;
							if (posCounter == 1) cmdOut[13] = ch;
							if (posCounter == 2) cmdOut[15] = ch;
							if (posCounter == 3) cmdOut[16] = ch;
							if (posCounter == 4) cmdOut[18] = ch;
							if (posCounter == 5) {
								cmdOut[19] = ch;
								// for now, always use Universal time; may calc timezone later from longitude
								cmdOut[21] = '+';
								cmdOut[22] = '0';
								cmdOut[23] = '0';
								NMEA_status.valid_timestamp = 1; // timestamp complete
								}
							break;
						case isValid:
							if (posCounter == 0) {
								if (ch == 'A') {
									NMEA_status.valid_data = 1; // GPS says data valid
								} else {
									NMEA_status.valid_data = 0; // data not valid
								}
							}
							break;
							
						case curLat:
							if (posCounter >= 0) cmdOut[28 + posCounter] = ch; // copy characters
							break;
							
						case isNorthOrSouth:
							if (posCounter == 0) cmdOut[39] = ch; // 'N' or 'S'
							break;
							
						case curLon:
							if (posCounter >= 0) cmdOut[40 + posCounter] = ch; // copy characters
							break;
							
						case isEastOrWest:
							if (posCounter == 0) cmdOut[52] = ch; // 'E' or 'W'
							break;
							
						case dateStamp: // "ddmmyy"; rearrange and copy, in between '-' positions
										// to make "20yy-mm-dd"
							NMEA_status.valid_datestamp = 0; // flag off while getting current date
							if (posCounter == 0) cmdOut[9] = ch;
							if (posCounter == 1) cmdOut[10] = ch;
							if (posCounter == 2) cmdOut[6] = ch;
							if (posCounter == 3) cmdOut[7] = ch;
							if (posCounter == 4) cmdOut[3] = ch;
							if (posCounter == 5) {
								cmdOut[4] = ch;
								NMEA_status.valid_datestamp = 1; // date is valid
								} 
							break;
//						case magVar: // don't need this field, or any after
//							NMEA_status.use_nmea = 0; // skip characters till next '$' found
//							break;
						default:
							break;
					} // end of switch (fldCounter)
				} // regular data character
			} // not end of line
		}
	}
}

void sendSetTimeSignal(void) {
	// this will be the usual tie-up point
	// transmit the set-time signal back to the main uC
	// then set flag(s) to signal this uC to shut down
	
	cmdClearPtr = cmdClear; // assure the main uC receive-command buffer is clear
	while (*cmdClearPtr != '\0') {
		while (!(UCSR1A & (1<<UDRE1))) { // Tx data register UDRn ignores any write unless UDREn=1
			;
		}
		UDR1 = *cmdClearPtr++; // put the character to be transmitted in the Tx buffer
	}	
	
	cmdOut[0]='t'; // set this command character only now
	cmdOutPtr = cmdOut;	
	while (*cmdOutPtr != '\0') {
		while (!(UCSR1A & (1<<UDRE1))) { // Tx data register UDRn ignores any write unless UDREn=1
			;
		}
		UDR1 = *cmdOutPtr++; // put the character to be transmitted in the Tx buffer
	}
}

void sendDebugSignal(void) {
	// for testing, send the current attempt at the set-time message
	cmdOutPtr = cmdOut;
	// debugging diagnostics, put flag characters into the output string
	if (Prog_status.gps_serial_Received) {
		*cmdOutPtr = 'r';
	}

	while (*cmdOutPtr != '\0') {
		while (!(UCSR1A & (1<<UDRE1))) { // Tx data register UDRn ignores any write unless UDREn=1
			;
		}
		UDR1 = *cmdOutPtr++; // put the character to be transmitted in the Tx buffer
	}
	// reset the following flag, to allow the next periodic diagnostics
	stateFlags.isTimeForDebugDiagnostics = 0;
	// after testing diagnostics, put the string back as it was
	restoreCmdDefault();
}

int circBufPut(circBuf_t *c, char d) {
	int nx = c->head + 1;
	if (nx >= c->maxLen)
		nx = 0;
	if (nx == c->tail)
		return 1; // buffer is full
	c->buffer[c->head] = d;
	c->head = nx;
	return 0;
}

int circBufGet(circBuf_t *c, char *d) {
	cli(); // don't let the Rx interrupt interfere
	if (c->head == c->tail) {
		sei();
		return 1; // buffer is empty
	}
	*d = c->buffer[c->tail];
	c->buffer[c->tail] = '\0';
	int nx = c->tail + 1;
	if (nx > c->maxLen)
		nx = 0;
	c->tail = nx;
	sei();
	return 0;
}

// Interrupt occurs when ADC conversion is complete
ISR(ADC_vect)
{
	// disable ADC conversion complete interrupt
	ADCSRA &= ~(1<<ADIE);
}

ISR(TIMER1_COMPA_vect) {
	// occurs when TCNT1 matches OCR1A
	// set to occur at 100Hz
	char n;
	int16_t t;
	uint8_t g;
	if (--ToggleCountdown <= 0) {
		PORTA ^= (1<<LED); // toggle bit 2, pilot light blinkey
		ToggleCountdown = TOGGLE_INTERVAL;
	}
	
	cli(); // don't let other interrupts interfere
	g = gpsTimeoutCountdown;
	if (g) gpsTimeoutCountdown = --g;
	if (!gpsTimeoutCountdown) {
		Prog_status.gps_serial_Received = 0;
	}
	sei();
	
	// for testing, show what's currently in output buffer
	// do this every 2.5 seconds
	if ((rouseCountdown % 250) == 0) {
		stateFlags.isTimeForDebugDiagnostics = 1;
	}
	
	t = rouseCountdown;
	if (t) rouseCountdown = --t;
	
	//	if (--rouseCountdown <= 0)
	if (!rouseCountdown)
	{
		stateFlags.isRoused = 0;
	}

	n = Timer1;						// 100Hz decrement timer 
	if (n) Timer1 = --n;
	// can use for in-loop delay by using following format:
	//	for (Timer1 = 3; Timer1; );	// Wait for 30ms
/*
	n = Timer2;
	if (n) Timer2 = --n;
	n = Timer3;
	if (n) Timer3 = --n;
*/
}

ISR(USART0_RX_vect) {
	// occurs when USART0 Rx Complete
	char gps_receiveByte;
	// ignore any "bad" characters
	// FE0: Frame Error
	// DOR0: Data OverRun
	// UPE0: USART Parity Error
	
	if (UCSR0A & ((1<<FE0) | (1<<DOR0) | (1<<UPE0))) { // bad "character" received
		gps_receiveByte = UDR0; // read register to clear it
		return;
	}

//	cli(); // don't let other interrupts interfere
	gpsTimeoutCountdown = GPS_RX_TIMEOUT; // while Rx normally, will always refresh within this interval
	Prog_status.gps_serial_Received = 1; // flag that serial is being received
//	sei();
	gps_receiveByte = UDR0; // read character
	if (stateFlags.setTimeCommandSent) // valid timestamp captured and sent
		return; // don't capture anything any more
	if (gps_receiveByte == '$') { // start of NMEA sentence
		NMEA_status.captureNMEA = 1; // flag to start capturing
		nmea_capture_counter = -1; // no chars counted yet
	}
	nmea_capture_counter++;
	if ((nmea_capture_counter == 3) && (gps_receiveByte != 'R')) // cannot be a "$GPRMC" sentence
		NMEA_status.captureNMEA = 0; // stop capturing
	if (NMEA_status.captureNMEA) { // try to store the character
		if (circBufPut(&gps_recBuf, gps_receiveByte)) {
			NMEA_status.bufferFull = 1; // if full, drop; ISR can't return anything; Rx data will be repeated
		} else {
			NMEA_status.bufferFull = 0;
		}		
	}
}

ISR(USART1_RX_vect) {
	// occurs when USART1 Rx Complete
	// for now, only flag that characters received, don't even store anything
	char main_receive_byte;
	// count 3 "good" characters; if board is totally unconnected, random noise interpreted as Rx, though with errors
	// FE1: Frame Error
	// DOR1: Data OverRun
	// UPE1: USART Parity Error	
	if (UCSR1A & ((1<<FE1) | (1<<DOR1) | (1<<UPE1))) { // bad "character" received
		cmd_capture_ctr = 0; // reset counter
	} else {
		cmd_capture_ctr++; // count that we got a good character
	}
	// we do not currently use main_receive_byte, but only load it from UDR1 to reset the UART
	main_receive_byte = UDR1; // clear any error flags, and buffer
	(void)main_receive_byte; // avoid compiler warning: variable 'main_receive_byte' set but not used [-Wunused-but-set-variable]
	// compiler warning clobbers all other build diagnostics
	if (cmd_capture_ctr >= 3) {
		Prog_status.main_serial_Received = 1; // flag that serial is received from the main uC
		UCSR1B = (1<<TXEN1); // enable only Tx; should flush the receive buffer and clear RXC1 flag, allowing Tx1
	}	
}

void delay_ms(uint16_t x) {
	uint8_t y, z;
	for ( ; x > 0 ; x--){
		for ( y = 0 ; y < 90 ; y++){
			for ( z = 0 ; z < 6 ; z++){
				asm volatile ("nop");
			}
		}
	}
}

void restoreCmdDefault(void) {
	// restore the output buffer to its default "x2016-03-19 20:30:01 -08"
	// followed by  " 0000.000000 00000.000000"
	cmdOut[0] = 'x';
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
	
	cmdOut[28] = '0';
	cmdOut[29] = '0';
	cmdOut[30] = '0';
	cmdOut[31] = '0';
	cmdOut[32] = '.';
	cmdOut[33] = '0';
	cmdOut[34] = '0';
	cmdOut[35] = '0';
	cmdOut[36] = '0';
	cmdOut[37] = '0';
	cmdOut[38] = '0';
	cmdOut[39] = ' ';
	cmdOut[40] = '0';
	cmdOut[41] = '0';
	cmdOut[42] = '0';
	cmdOut[43] = '0';
	cmdOut[44] = '0';
	cmdOut[45] = '.';
	cmdOut[46] = '0';
	cmdOut[47] = '0';
	cmdOut[48] = '0';
	cmdOut[49] = '0';
	cmdOut[50] = '0';
	cmdOut[51] = '0';
	cmdOut[52] = ' ';
	cmdOut[53] = '\n';
	cmdOut[54] = '\r';
	cmdOut[55] = '\n';
	cmdOut[56] = '\r';
	cmdOut[57] = '\0';
}
