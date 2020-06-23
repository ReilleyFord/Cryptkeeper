#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "include/uart.h"

#ifndef FALSE
#define FALSE 0
#define TRUE !FALSE
#endif

// Figure this out
#define BAUDRATE    38400L
#define BAUDREG     ((unsigned int)((F_CPU/(BAUDRATE*8UL))-1))

/*
 * SD Card Commands
 */
#define SD_IDLE        (0x40 + 0)   // CMD0: Set SD card to Idle
#define SD_INIT        (0x40 + 1)   // CMD1: Initialize SD Card
#define SD_INTER       (0x40 + 8)   // CMD8: Send Interface - Only for SDHC
#define SD_CSD         (0x40 + 9)   // CMD9: Send CSD Block
#define SD_CID         (0x40 + 10)  // CMD10: Send CID Bock
#define SD_STATUS      (0x40 + 13)  // CMD13: Send Card Status
#define SD_SET_BLK     (0x40 + 16)  // CMD16: CMD16: Set Block Size (Bytes)
#define SD_READ_BLK    (0x40 + 17)  // Read single block
#define SD_LOCK_UNLOCK (0x40 + 42)  // CMD42: PWD Lock/Unlock
#define CMD55          (0x40 + 55)  // Multi-byte preface command
#define SD_OCR         (0x40 + 58)  // Read OCR
#define SD_ADV_INIT    (0xc0 + 41)  // ACMD41 Advanced Initialization for SDHC
#define SD_GET_CSD     (0x40 + 27)  // CMD27: Get CSD Block

/*
 * Masks for CMD42 options
 */
#define MASK_ERASE        0x08 //
#define MASK_LOCK_UNLOCK  0x04

/*
 * Arduino is split into blocks of pins. Each block needs 3 Registers
 * DDR (Data Direction Register) - Dictates which pins are Input or output
 * PORT - Which block of pins is being used.
 * PIN - Reads input value when a pin is selected as Input mode.
 */

// Setting up SPI and DDR (Data Direction Register)
// DDR will decide whether the port is Input (0xFF) or output (Default and 0x00)
// For example. Setting the fifth bit of DDRB to 1 means we are indicating that
// we want to use the pin associated to the fifth bit in PORTB to be used as output.
#define SPI_PORT  PORTB
#define SPI_DDR   DDRB

// Bits used by the SPI port
#define MOSI  3
#define MISO  4
#define SCK   5
// Fourth called SS for Slave Select, used for multiple slaves.

// Definition for CS, port, and DDR for the SD Card. - Should match chip?
#define SD_PORT     PORTB
#define SD_DDR      DDRB
#define SD_CS       PORTB2
#define SD_CS_MASK  (1<<SD_CS)

// Error codes for functions
#define SD_OK         0
#define SD_NO_DETECT  1
#define SD_TIMEOUT    2
#define SD_RWFAIL    -1

// CMDs to run against SD Card
#define  CMD_LOCK		    1
#define  CMD_UNLOCK		  2
#define  CMD_NONE		    3
#define  CMD_INFO		    4
#define  CMD_READBLK		5
#define  CMD_PWD_LOCK	  6
#define  CMD_PWD_UNLOCK	7
#define  CMD_PWD_CHECK	8
#define  CMD_LOCK_CHECK	9
#define  CMD_ERASE		  10
#define  CMD_PWD_CLEAR  11

#define  CRC7_POLY		0x89		/* polynomial used for CSD CRCs */

uint8_t pwd[16];
uint8_t pwd_len;
uint8_t crctable[256];

/*
 * Local function declaration
 */
static void 	  Done(void);
static void     Select(void);
static void     Deselect(void);
static uint8_t  xchg(uint8_t  c);
static void     BuildCRCTable(void);
static void 		LoadEnteredPassword(void);
static uint8_t 	ExecuteCMD42(uint8_t mask);
static void     ProcessCommand(void);
static uint8_t  ReadCommand(void);
static int8_t   sd_send_command(uint8_t  command, uint32_t  arg);
static int8_t		sd_wait_for_data(void);


int main(void) {

  // First step, enable CS as output.
  SD_DDR  |= SD_CS_MASK; // Setting the 2nd pin of PORTB (Chip Select) as output via DDRB
  Deselect(); // Make sure card is not selected.

  SPI_PORT |= ((1<<MOSI) | (1<<SCK));   // Flip bits for MOSI and Serial Clock
  SPI_DDR  |= ((1<<MOSI) | (1<<SCK));   // Mark pins as output
  SPI_PORT |= (1<<MISO);                // Flipping MISO bit.

  /*
   * Enabling SPI via SPCR (Serial Peripheral Control Register)
   * SPE  - SPI Enable - Flip bit to enable SPI
   * MSTR - Master/Slave Select. If set Master mode is enabled.
   * SPR1 - Setting Clock Rate - Multiple options depending on SP
   * SPR0 - Setting Clock Rate - SPR0, SPR1 and SPI2X dictate Clock Rate based on which bits are set.
   * In this configuration Clock Rate is set to fosc/128.
   */
  SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0);

  // Initialize UART
  uart_init();
  stdout = &uart_output;
  stdin  = &uart_input;
  stderr = &uart_output;
  sei();  // Enable Global Interrupts

  printf_P(PSTR("%c[2J"), 27); // Send escape code to clear UART Terminal.
  printf_P(PSTR("\r\nPolyphemus SD Card Tool\r\n"));
  printf_P(PSTR("? - Read Card Status\r\n"));
  printf_P(PSTR("u - Attempt Unlock\r\n"));
  printf_P(PSTR("l - Lock\r\n"));
  printf_P(PSTR("c - Clear Password\r\n"));
  printf_P(PSTR("r - Read Card\r\n"));

  BuildCRCTable();

  while(1) ProcessCommand();

  return 0;
}

static void ProcessCommand(void) {
  uint8_t         cmd;
  static uint8_t  prevCMD = 0;
  uint8_t         response;

  response = ReadCommand();

  if((cmd != prevCMD) && (prevCMD == CMD_NONE)) prevCMD = cmd;
}

static uint8_t ReadCommand(void) {
  uint8_t response, cmd;

  _delay_ms(50);
  response = CMD_NONE;
  if(uart_pending_data()) {
    response = getchar();
    printf_P(PSTR("\n%c"), response);
  }

  return response;
}

// Not sure how this is working....
static void BuildCRCTable(void) {
    int i, j;
    // generate a table value for all 256 possible byte values
    for (i = 0; i < 256; i++) {
        crctable[i] = (i & 0x80) ? i ^ CRC7_POLY : i;
        for (j = 1; j < 8; j++) {
            crctable[i] <<= 1;
            if (crctable[i] & 0x80) crctable[i] ^= CRC7_POLY;
        }
    }
}

/*
 * Flipping CS bit -- Selecting card.
 */
static void Select(void) {
  SD_PORT &= ~SD_CS_MASK; //
}

/*
 * Flipping CS bit -- De-selecting card.
 */
static void Deselect(void) {
  SD_PORT |= SD_CS_MASK;
}

/*
 * Laziness.
 */
static void Done(void) {
	printf_P(PSTR("\ndone.\n"));
}

// Get user input for an attempted password and then Load that password into memory.
// Loop until terminating character of enter
static void LoadEnteredPassword(void) {
	uint8_t r;
	uint8_t i = 0;

	_delay_ms(50);
	printf_P(PSTR("\n\nPlease Enter Password:\r\n"));

	// Loop until enter key (\r) press. Build PWD and PWD_LEN. Backspace functionality.
	while(1) {
		if(uart_pending_data()) {
			r = getchar();
			printf_P(PSTR("%c"), r);

			if (r == 127) {
				i--;
				continue;
			} else if(r == '\r') {
				pwd_len = i;
				break;
			} else {
				pwd[i] = r;
				i++;
			}

		}
	}

}

/*
 * This function will handle all future CMD42 executions.
 */
static uint8_t ExecuteCMD42(uint8_t mask) {
	uint8_t response;
	uint16_t i;
	mask = mask & 0x07; // Bitwise operator, flip high bits.
	// 00000010 with bitwise AND 0x07 - Clears PWD NO ERASE.
	// 00000100 with bitwise AND 0x07- Unlocking for current session - NO ERASE.
	Deselect(); // Just in case.
	Select(); // CMD7 Select the card. Place in Transfer/Receive mode.

	// No need to set block size. BLK set in SDInit()
	response = sd_send_command(SD_LOCK_UNLOCK, 0); // Send unlock command.
	if(response != 0) return SD_RWFAIL; // Check response.

	xchg(0xfe);	// Data token marking start of block.
	xchg(mask); // Start with the correct command.
	xchg(pwd_len); // Send pwd length

 	// Sending 1 full 512 byte block.
	for(i = 0; i < 512; i++) {
		if(i < pwd_len) {
			printf_P(PSTR("\nExchaning Byte: %c"), pwd[i]);
			xchg(pwd[i]);
		} else xchg(0xff);
	}

	// Closing with 2x 8 clocks
	xchg(0xff);
	xchg(0xff);

	i = 0xffff;
	while(!xchg(0xFF) && (--i)); // Waiting for card.

	if(i) return SD_OK;
	else return SD_RWFAIL;
}
