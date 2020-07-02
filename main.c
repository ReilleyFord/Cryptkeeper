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
#define MASK_CLR_PWD      0x02
#define MASK_SET_PWD      0x01

/*
 * Options for Types of SD cards.
 */
#define  SDTYPE_UNKNOWN		0				/* card type not determined */
#define  SDTYPE_SD				1				/* SD v1 (1 MB to 2 GB) */
#define  SDTYPE_SDHC			2				/* SDHC (4 GB to 32 GB) */
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
uint8_t sdtype;
uint8_t block[512];
uint8_t cardstatus[2];
uint8_t csd[16];
uint8_t cid[16];
uint8_t ocr[4];

/*
 * Local function declaration
 */
static void 	  Done(void);
static void     Select(void);
static void     Deselect(void);
static uint8_t  SendByte(uint8_t  c);
static void 		LoadEnteredPassword(void);
static uint8_t 	ExecuteCMD42(uint8_t mask);
static void     ProcessCommand(void);
static uint8_t  ReadCommand(void);
static int8_t   SendCommand(uint8_t  command, uint32_t  arg);
static int8_t   InitializeSD(void);
static int8_t   ReadSD(void);
static int8_t   ReadOCR(void);
static int8_t   ReadCSD(void);
static int8_t   ReadCID(void);
static int8_t   ReadStatus(void);
static void     DisplayStatus(void);
static int8_t   WaitForData(void);
static void     DisplayBlock(void);
static int8_t   ReadBlock(uint32_t  blocknum, uint8_t *buffer);

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
   * SPR1 - Setting Clock Rate - Multiple options depending on SPX
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

  while(1) ProcessCommand();

  return 0;
}

/*
 * Flipping CS bit -- Selecting card.
 */
static void Select(void) {
  SD_PORT &= ~SD_CS_MASK;
}

/*
 * Flipping CS bit -- De-selecting card.
 */
static void Deselect(void) {
  SD_PORT |= SD_CS_MASK;
}

/*
 * ProcessCommand function
 * Beginning of code flow, kicked off by main(). This process loops awaiting
 * user input, either through switches or UART -- Some form of user input.
 */
static void ProcessCommand(void) {
  uint8_t         cmd, i;
  static uint8_t  prevCMD = 0;
  uint8_t         response;

  cmd = ReadCommand();

  if((cmd != prevCMD) && (prevCMD == CMD_NONE)) {

  response = InitializeSD();
  if(response != SD_OK) printf_P(PSTR("\n\r\n\rUnable to initialize card."));

  /*
   * If card passes init vibe check, begin processing command.
   */
   if(cmd == CMD_INFO) {
     printf_P(PSTR("\r\nCard Type: %d"), sdtype);
     response = ReadSD();
     if(response == SD_OK) {
       printf_P(PSTR("\r\nOCR: "));
       for(i = 0; i < 4; i++) printf_P(PSTR("%02X "), ocr[i]);
       printf_P(PSTR("\r\nCSD: "));
       for(i = 0; i < 16; i++) printf_P(PSTR("%02X "), csd[i]);
       printf_P(PSTR("\r\nCID: "));
       for(i = 0; i < 16; i++) printf_P(PSTR("%02X "), cid[i]);
       DisplayStatus();
     } else printf_P(PSTR("\r\nCard Registers could not be read."));
   } else if(cmd == CMD_PWD_CLEAR) {
     ReadStatus();
     if(cardstatus[1] & 0x01) {
       LoadEnteredPassword();
       response = ExecuteCMD42(MASK_CLR_PWD);
       ReadStatus();
       if(cardstatus[1] & 0x01) {
         printf_P(PSTR("\nFailed! Retrying..."));
         response = ExecuteCMD42(MASK_CLR_PWD);
         ReadStatus();
         if(cardstatus[1] & 0x01) printf_P(PSTR("\nFailed: The card is still locked."));
       } else Done();
     } else printf_P(PSTR("\nThe card is not locked."));
   } else if(cmd == CMD_READBLK) {

     //ToDo
     response = ReadBlock(0, block);

     if(response != SD_OK) printf_P(PSTR("\nError: Unable to read block."));
     else DisplayBlock();
   } else if(cmd == CMD_PWD_LOCK) {
     ReadStatus();

     if((cardstatus[1] & 0x01) == 0) {
       LoadEnteredPassword();
       printf_P(PSTR("\r\nAttempting to set password."));
       response = ExecuteCMD42(MASK_SET_PWD);
       ReadStatus();

       printf_P(PSTR("\nAttempting to lock card."));
       response = ExecuteCMD42(MASK_LOCK_UNLOCK);
       ReadStatus();
       if((cardstatus[1] & 0x01) == 0) printf_P(PSTR("\nFailed: there was an error attempting to lock card."));
       else Done();
     } else printf_P(PSTR("\nThe card is already locked."));
   } else if(cmd == CMD_PWD_UNLOCK) {
     ReadStatus();

     if(cardstatus[1] & 0x01) {
       LoadEnteredPassword();
       printf_P(PSTR("\nAttempting to unlock card."));
       response = ExecuteCMD42(00000100);
       ReadStatus();
       if(cardstatus[1] & 0x01) {
         printf_P(PSTR("\nUnlock Failed: Attempting unlock again."));
         response = ExecuteCMD42(00000100);
         ReadStatus();
         if(cardstatus[1] & 0x01) printf_P(PSTR("\nUnlock Failed: Unable to unlock card."));
       } else Done();
     } else printf_P(PSTR("\nCard is already unlocked."));
   }

  }
  prevCMD = cmd;
}

/*
 * ReadCommand function
 * This is called during ProcessCommand and is used to determine CMD options/state
 * Returns CMD selected as response.
 */
static uint8_t ReadCommand(void) {
  uint8_t response;

  _delay_ms(50);
  response = CMD_NONE;
  // Wait for data from UART.
  if(uart_pending_data()) {
    response = getchar();
    printf_P(PSTR("\n%c"), response);

    switch (response) {
      case '?' :
        response = CMD_INFO;
        break;
      case 'r' :
        response = CMD_READBLK;
        break;
      case 'u' :
        response = CMD_PWD_UNLOCK;
        break;
      case 'l' :
        response = CMD_PWD_LOCK;
        break;
      case 'c' :
        response = CMD_PWD_CLEAR;
        break;
      default  :
        response = CMD_NONE;
    }
  }

  return response;
}

/*
 * SD Card Initialization function.
 * This will begin by setting SD to idle mode.
 * Then it will probe the card to check for SDHC which requires ACMD41 interface
 * and advanced intialization methods.
 * Returns SD_OK or SD_NO_DETECT.
 */
static int8_t InitializeSD(void) {
  int i;
  int8_t response;

  sdtype = SDTYPE_UNKNOWN;

  Deselect();

  // Send bytes while card stabilizes.
  for(i=0; i < 10; i++) SendByte(0xff);

  for(i = 0; i < 0x10; i++) {
    response = SendCommand(SD_IDLE, 0); // Try SD_IDLE until success or timeout.
    if(response == 1) break;
  }
  if(response != 1) return SD_NO_DETECT;

  SendCommand(SD_SET_BLK, 512); // Set block length to 512 bytes.

  // Always attempt ACMD41 first for SDC then drop to CMD1
  response = SendCommand(SD_INTER, 0x1aa);
  if(response == 0x01) {
    for(i = 0; i < 4; i++) SendByte(0xff);          // Clock through 4 bytes to burn 32 bit lower response.
    for(i = 20000; i > 0; i--) {                    // Send Advanced init cmd until initialization complete and response is 0x00
      response = SendCommand(SD_ADV_INIT, 1UL<<30); // Send advanced init with HCS bit 30 set.
      if(response == 0) break;
    }
    sdtype = SDTYPE_SDHC;
  } else { // Begin initializing SDSC -- CMD1
    response = SendCommand(SD_OCR, 0); // Not necessary if voltage is set correctly.
    if(response == 0x01) {
      for(i = 0; i < 4; i++) SendByte(0xff); // Burn the next 4 bytes returned (OCR)
    }
    for(i = 20000; i > 0; i--) {
      response = SendCommand(SD_INIT, 0);
      if(response == 0) break;
    }
    SendCommand(SD_SET_BLK, 512); // SDSC might reset block length to 1024, reinit to 512.
    sdtype = SDTYPE_SD;
  }

  SendByte(0xff); // End initialization with 8 clocks.

  // Initialization should be completed. The SPI clock rate can be set to maximum, usually 20MHz. Depends on card.
  return SD_OK;
}

/*
 * ReadSD function
 * Kicks off a basic read of the available data registers.
 * OCR, CSD, CID.
 */
static int8_t ReadSD(void) {
   int8_t response;

   response = ReadOCR();
   response = ReadCSD();

   if(response == SD_OK) response = ReadCID();
   if(response == SD_OK) response = ReadStatus();

   return response;
}

/*
 * ReadOCR function
 * Requests a read of the card OCR.
 * Method of read is based on SD Card Type.
 */
static int8_t ReadOCR(void) {
  uint8_t i;
  int8_t  response;

  if(sdtype == SDTYPE_SDHC) {
    response = SendCommand(SD_INTER, 0x1aa);
    if(response != 0) return SD_RWFAIL;
    for(i=0; i < 4; i++) ocr[i] = SendByte(0xff);
    SendByte(0xff);                                // Burn the remaining CRC bits.
  } else {
    response = SendCommand(SD_OCR, 0);
    if(response != 0x00) return SD_RWFAIL;         // Check response returned from CMD.
    for(i=0; i < 4; i++)  ocr[i] = SendByte(0xff); // Next four bytes will be the OCR.
    SendByte(0xff);                                // Burn the remaining byte.
  }

  return SD_OK;
}

/*
 * ReadCSD function
 * Requests a read of the Card Specific Data (CSD)
 */
static int8_t ReadCSD(void) {
  int8_t  response;

  SendCommand(SD_CSD, 0);
  response = WaitForData();
  if (response != (int8_t)0xfe) return SD_RWFAIL;

  // CSD returns 16 Bytes. -- Grab those.
  for(int i=0; i < 16; i++) csd[i] = SendByte(0xff);
  SendByte(0xff); // Burn the CRC.

  return SD_OK;
}

/*
 * ReadCID function
 * Requests a read of the card Card Identification Data.
 */
static int8_t ReadCID(void) {
  int8_t response;

  SendCommand(SD_CID, 0);
  response = WaitForData();
	if(response != (int8_t)0xfe) return SD_RWFAIL;

  // CID returns R1 response and 16 bytes.
  for(uint8_t i = 0; i < 16; i++) cid[i] = SendByte(0xff);

  SendByte(0xff); //Burn CRC

  return SD_OK;
}

/*
 * ReadStatus function
 * Reads the card status via CMD13
 */
static int8_t ReadStatus(void) {
  cardstatus[0] = SendCommand(SD_STATUS, 0);
  cardstatus[1] = SendByte(0xff);

  SendByte(0xff);
  return  SD_OK;
}

/*
 * ReadBlock function
 * This will execute CMD17 - Read Block command to obtain the first 512 block of data from the card.
 */
static int8_t ReadBlock(uint32_t startblock, uint8_t *buffer) {
  uint8_t   status;
  uint16_t  i;
  uint32_t  address;

  /*
   * CMD17:
   * SDSC uses a Byte Address
   * SDHC uses block number
   */

   if(sdtype == SDTYPE_SD) address = startblock << 9; // Convert to Byte Address.

   status = SendCommand(SD_READ_BLK, address); // Send CMD17
   if(status != SD_OK) return SD_RWFAIL; // Check returned status from CMD

   status = WaitForData(); // Wait for 0xFE marking start of block read.
   if(status != 0xFE) return SD_RWFAIL; // Check status.

   for(i = 0; i < 512; i++) block[i] = SendByte(0xFF); // Grab the next 512 bytes.

   // Send dummy data to complete process.
   SendByte(0xFF);
   SendByte(0xFF);

   return SD_OK;
}

/*
 * DisplayStatus() function
 * Determines lock status.
 */
static void DisplayStatus(void) {
  ReadStatus();

  printf_P(PSTR("\r\nPassword Status: "));
  if((cardstatus[1] & 0x01) == 0) printf_P(PSTR("Unlocked\n"));
  else printf_P(PSTR("Locked\n"));
}

/*
 * DisplayStatus function
 * Function to format and display the Data Block obtained from ReadBlock function.
 */
static void DisplayBlock(void) {
  uint32_t i;
  uint8_t  str[17];

	str[16] = 0;
	str[0] = 0;			// only need for first newline, overwritten as chars are processed

	printf_P(PSTR("\n\rContents of block buffer:"));
	for (i=0; i<512; i++) {
		if ((i % 16) == 0) printf_P(PSTR(" %s\n\r%04X: "), str, i);

		printf_P(PSTR("%02X "), (uint8_t)block[i]);

		if (isalpha(block[i]) || isdigit(block[i]))  str[i%16] = block[i];
		else str[i%16] = '.';
	}
	printf_P(PSTR(" %s\n\r"), str);
}

/*
 * SendCommand
 * Function accepts an SD CMD and 4 byte argument.
 * Exchanges CMD and arg with CRC and 0xff filled bytes with card.
 * Returns the response provided by the card.
 * For advanced initilization and commands this will send the required preface CMD55
 * Error codes will be 0xff for no response, 0x01 for OK, or CMD specific responses.
 */
static int8_t SendCommand(uint8_t cmd, uint32_t arg) {
  uint8_t response, crc;

  /*
   * Needed for SDC and advanced initilization.
   * ACMD(n) requires CMD55 to be sent first.
   */
  if(cmd & 0x80) {
    cmd = cmd & 0x7f; // Stripping high bit.
    response = SendCommand(CMD55, 0);
    if (response > 1) return response;
  }

  Deselect();
  SendByte(0xff);
  Select();
  SendByte(0xff);

  /*
   * Begin sending command
   * Command structure is 48 bits??
   */
   SendByte(cmd | 0x40);
   SendByte((unsigned char)(arg>>24));
   SendByte((unsigned char)(arg>>16));
   SendByte((unsigned char)(arg>>8));
   SendByte((unsigned char)(arg&0xff));
   if(cmd == SD_IDLE)  crc = 0x95;
   if(cmd == SD_INTER) crc = 0x87;
   SendByte(crc);

   // Send clocks waiting for timeout.
   do {
      response = SendByte(0xff);
    } while((response & 0x80) != 0); // High bit cleared means OK

   // Switch statement with fall through and default. Deselecting card if no more R/W operations required.
   switch (cmd) {
     case SD_ADV_INIT :
     case SD_SET_BLK :
     case SD_IDLE :
     case SD_INIT :
     case CMD55 :
       Deselect();
       SendByte(0xff);
     default :
       break;
   }

   return response;
}

/*
 * This function will handles all CMD42 executions.
 * Seperate from SendCommand for building CMD42 specific data blocks
 * PWD, PWD_LEN, CRC (Theoretically should not matter as CRC is not checked in SPI mode).
 */
static uint8_t ExecuteCMD42(uint8_t mask) {
	uint8_t response;
	uint16_t i;
	mask = mask & 0x07; // Bitwise operator, flip high bits.
	// 00000010 with bitwise AND 0x07 - Clears PWD NO ERASE.
	// 00000100 with bitwise AND 0x07- Unlocking for current session - NO ERASE.
	Deselect(); // Just in case.
	Select();   // CMD7 Select the card. Place in Transfer/Receive mode.

	// No need to set block size. BLK set in SDInit()
	response = SendCommand(SD_LOCK_UNLOCK, 0); // Send unlock command.
	if(response != 0) return SD_RWFAIL;        // Check response.

	SendByte(0xfe);	   // Data token marking start of block.
	SendByte(mask);    // Start with the correct command.
	SendByte(pwd_len); // Send pwd length

 	// Sending 1 full 512 byte block.
	for(i = 0; i < 512; i++) {
		if(i < pwd_len) {
			printf_P(PSTR("\nExchaning Byte: %c"), pwd[i]);
			SendByte(pwd[i]);
		} else SendByte(0xff);
	}

	// Closing with 2x 8 clocks
	SendByte(0xff);
	SendByte(0xff);

	i = 0xffff;
	while(!SendByte(0xFF) && (--i)); // Waiting for card.

	if(i) return SD_OK;
	else return SD_RWFAIL;
}

/*
 * SendByte function.
 * ToDo: comment this.
 */
static unsigned char SendByte(unsigned char c) {
  SPDR = c; // Write to SPI Data Register - Writes out to MOSI via Hosts SPI Bus
  while((SPSR & (1<<SPIF)) == 0); // Wait for SPSR and SPIF registers to clear.
  return SPDR;
}

/*
 * Get user input for an attempted password and then Load that password into memory.
 * Loop until terminating character of enter
 */
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
 * Laziness.
 */
static void Done(void) {
	printf_P(PSTR("\ndone.\n"));
}

/*
 * WaitForData function
 * Used for commands that require processing and timeouts while awaiting response
 * that is not 0xff.
 */
static int8_t WaitForData(void) {
	int16_t				i;
	uint8_t				response;

	for (i = 0; i < 100; i++) {
		response = SendByte(0xff);
		if (response != 0xff) break;
	}

	return  (int8_t) response;
}
