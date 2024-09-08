

#include "reg_defs.h"
#include "flash_shield.h"
#include <stdint.h>
#include "common.h"

/* The W25Q80DV array is organized into 4096 programmable pages of 256-bytes each.
 * Up to 256 bytes can be programmed at a time.
 * 16 pages crerates one sector of 4 kB and 16 sectors create one 64 kB block
 * Pages can be erased in groups of 16 pages (One 4KB sector erase)
 * The W25Q80DV has 256 erasable sectors.
 * The W25Q80DV supports the standard Serial Peripheral Interface (SPI)
 *
 * Our partition of the Flash is to keep sector 0 as metadata container
 * to keep track of latest trackPointSerie and other stuff
 * Layout of Sector 0 is first page is just string metadata about
 * SW version and corresponding git sha
 *
 * Since we only will write one page each time we need to keep track of
 * latest page written in order to be able to start a new track.
 * For each new track is the first page a track metadata page
 *
 * Page Program (02h)
 * The Page Program instruction allows from one byte to 256 bytes (a page) of data to be programmed at
 * previously erased (FFh) memory locations. A Write Enable instruction must be executed before the
 * device will accept the Page Program Instruction (Status Register bit WEL= 1). The instruction is initiated
 * by driving the /CS pin low then shifting the instruction code “02h” followed by a 24-bit address (A23-A0)
 * and at least one data byte, into the DI pin. The /CS pin must be held low for the entire length of the
 * instruction while data is being sent to the device. The Page Program instruction sequence is shown in
 * figure 19.
 * If an entire 256 byte page is to be programmed, the last address byte (the 8 least significant address
 * bits) should be set to 0. If the last address byte is not zero, and the number of clocks exceed the
 * remaining page length, the addressing will wrap to the beginning of the page. In some cases, less than
 * 256 bytes (a partial page) can be programmed without having any effect on other bytes within the same
 * page. One condition to perform a partial page program is that the number of clocks cannot exceed the
 * remaining page length. If more than 256 bytes are sent to the device the addressing will wrap to the
 * beginning of the page and overwrite previously sent data.
 *
 * Sector Erase (20h)
 * The Sector Erase instruction sets all memory within a specified sector (4K-bytes) to the erased state of
 * all 1s (FFh). A Write Enable instruction must be executed before the device will accept the Sector Erase
 * Instruction (Status Register bit WEL must equal 1). The instruction is initiated by driving the /CS pin low
 * and shifting the instruction code “20h” followed a 24-bit sector address (A23-A0) (see Figure 2). The
 * Sector Erase instruction sequence is shown in figure 21a
 *
 * Read Data (03h)
 * The Read Data instruction allows one or more data bytes to be sequentially read from the memory. The
 * instruction is initiated by driving the /CS pin low and then shifting the instruction code “03h” followed
 * by a 24-bit address (A23-A0) into the DI pin. The code and address bits are latched on the rising edge
 * of the CLK pin. After the address is received, the data byte of the addressed memory location will be
 * shifted out on the DO pin at the falling edge of CLK with most significant bit (MSB) first. The address is
 * automatically incremented to the next higher address after each byte of data is shifted out allowing for
 * a continuous stream of data. This means that the entire memory can be accessed with a single
 * instruction as long as the clock continues. The instruction is completed by driving /CS high.
 */

// Prototypes for private functions
static void write_track_start(uint8_t *data);
static void read_flash_header(void);
static void init_flash_header(void);
static void enable_write(void);

// local typedefs and defines
#define WRITE_ENABLE 0x06U
#define WRITE_DISABLE 0x04U
#define READ_STATUS_REG1 0x05U
#define READ_STATUS_REG2 0x35U
#define WRITE_STATUS_REG 0x01U
#define PAGE_PROGRAM 0x02U
#define SECTOR_ERASE 0x20U
#define BLOCK_ERASE 0x52U
#define CHIP_ERASE 0xC7U
#define ERASE_PROG_SUSPEND 0x75U
#define ERASE_PROG_RESUME 0x7AU
#define POWER_DOWN 0xB9U
#define READ_DATA 0x03U
#define READ_DATA_CMD_SIZE 0x04U
#define FAST_READ 0x0BU
#define ENABLE_RESET 0x66U
#define RESET 0x99U
#define PAGE_ADDR_SIZE 0x03U
#define PAGE_SIZE 0x100
#define SECTOR_SIZE 0x1000
#define FLASH_HEADER_SZ 0x0DU
typedef enum
{
   FLASH_ERASE_ALL,
   FLASH_SECTOR_ERASE,
   FLASH_PROGRAM
} prog_states_t;

static boolean gpsTrackStarted;
static uint16_t gpsTrackId;
static volatile uint8_t *payloadPtr;
static volatile uint8_t byteCount;
static volatile uint8_t offset;
static volatile boolean normalTrackData;
static volatile boolean progStartSent;
static volatile boolean progAddressSent;
static volatile boolean writeEnabled;
static volatile boolean flashInitialized;
static volatile boolean readFromFlash;
static volatile boolean readFlashStatus;
static volatile boolean readFlashMeta;
static volatile boolean metaCorrect;
static volatile boolean transactionDone;
static uint8_t pageAddressArray[PAGE_ADDR_SIZE];
static volatile uint8_t payloadCount;
static volatile uint8_t payloadSize;
static volatile prog_states_t progState;

// Read flash meta data command from address 0x00
const uint8_t read_metadata[] = {0x03, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t flash_header[] = {2, 4, 0, 9, 0, 1, 11, 3, 11, 13, 6, 0, 10};

// Currently focusing on writing data and need redesign to handle read data as well
// Since we run with a hich SPI clock compared to osc clock (prescaled by 2) means
// the ISR must finish within 16/17 clock cycles (asm instructions)
ISR(SPI_STC_vect)
{
   if (readFromFlash)
   {
      // Handle read commands
      // write next byte unles already written previos ISR
      if (payloadSize > payloadCount + 1)
      {
         SPDR = payloadPtr[payloadCount + 1];
      }

      if (readFlashMeta)
      {
         // Not transfer any read data, just compare with expected
         if (READ_DATA_CMD_SIZE <= payloadCount)
         {
            if (SPDR != flash_header[payloadCount])
            {
               metaCorrect = FALSE;
            }
         }
      }
      else if (readFlashStatus)
      {
         // Check if BUSY bit is cleared
         if (0 == (SPDR & 0x1))
         {
            // Erase is done, clear flag
            readFlashStatus = FALSE;
         }
      }
      // increment for next byte
      payloadCount++;
      if (payloadSize == payloadCount)
      {
         transactionDone = TRUE;
         // Pull SS high to indicate end of transmission
         PORTB |= _BV(PORTB2);
      }
   }
   else
   {
      // Do the write commands
      if (writeEnabled)
      {
         if (FLASH_PROGRAM == progState)
         {
            if (progStartSent)
            {
               if (progAddressSent)
               {
                  // Gets called after each byte has ben sent
                  byteCount++;
                  if (normalTrackData)
                  {
                     if (TRK_BYTES > byteCount)
                     {
                        // Write next byte
                        SPDR = payloadPtr[byteCount + offset];
                     }
                     else
                     {
                        // Pull SS high to indicate end of transmission
                        PORTB |= _BV(PORTB2);
                        // Clear Write Enabled flag
                        writeEnabled = FALSE;
                     }
                  }
                  else
                  {
                     if (TRK_HEADER_DATA_BYTES > byteCount)
                     {
                        // Write next byte
                        SPDR = payloadPtr[byteCount];
                     }
                     else if (TRK_BYTES > byteCount)
                     {
                        SPDR = 0xFF;
                     }
                     else
                     {
                        // Pull SS high to indicate end of transmission
                        PORTB |= _BV(PORTB2);
                        // Clear Write Enabled flag
                        writeEnabled = FALSE;
                     }
                  }
               }
               else
               {
                  // Send page address for programming
                  SPDR = pageAddressArray[payloadCount++];
                  if (PAGE_ADDR_SIZE <= payloadCount)
                  {
                     // Address has been sent, now we start with track datra payload
                     progAddressSent = TRUE;
                  }
               }
            }
            else
            {
               SPDR = PAGE_PROGRAM;
               progStartSent = TRUE;
            }
         }
         else if (FLASH_SECTOR_ERASE == progState)
         {
         }
         else if (FLASH_ERASE_ALL == progState)
         {
            // Pull SS high to indicate end of transmission
            PORTB |= _BV(PORTB2);
            // Set flag to indicate transaction done
            transactionDone = TRUE;
         }
      }
      else
      {
         // Pull SS high to indicate end of transmission
         PORTB |= _BV(PORTB2);
         // Set the Write Enabled flag
         writeEnabled = TRUE;
      }
   }
}

void init_flash(void)
{
   // First step is to check if the Flash has been used at all, e.g. does the first
   // sector contain proper metadata.
   gpsTrackStarted = FALSE;
   read_flash_header();
}

void store_track_data(uint8_t *data)
{
   writeEnabled = FALSE;
   normalTrackData = FALSE;
   progStartSent = FALSE;
   progAddressSent = FALSE;
   progState = FLASH_PROGRAM;
   offset = 0;
   payloadPtr = data;
   payloadCount = 0;
   if (!gpsTrackStarted)
   {
      /// First track point needs a metadata page first to indicate start of track
      write_track_start(data);
      gpsTrackStarted = TRUE;
      offset = TRK_HEADER_DATA_BYTES;
   }
   // Now write the payload point
   byteCount = 0;
   normalTrackData = TRUE;
   enable_write();
   // Pull SS low to indicate start of page write
   PORTB &= ~_BV(PORTB2);
   // Write first byte to SPDR
   SPDR = payloadPtr[byteCount + offset];
}

// Private functions
static void write_track_start(uint8_t *data)
{
   // Write a page with track start meta data
   normalTrackData = FALSE;
   enable_write();
   // Pull SS low to indicate start of page write
   PORTB &= ~_BV(PORTB2);
   // Write first byte to SPDR
   SPDR = payloadPtr[byteCount + offset];
}

static void set_page_address(uint8_t A23_A16, uint8_t A15_A8, uint8_t A7_A0)
{
   pageAddressArray[0] = A23_A16;
   pageAddressArray[1] = A15_A8;
   pageAddressArray[2] = A7_A0;
}

static void read_flash_header(void)
{
   // Get header data
   readFromFlash = TRUE;
   readFlashMeta = TRUE;
   transactionDone = FALSE;
   // assume correct meta data since ISR will set to FALSE if not
   metaCorrect = TRUE;

   payloadPtr = read_metadata;
   payloadSize = READ_DATA_CMD_SIZE + FLASH_HEADER_SZ;
   payloadCount = 0;
   // Set SS to start SPI
   PORTB &= ~_BV(PORTB2);
   // Write first byte to SPDR
   SPDR = payloadPtr[payloadCount];
   // Wait until read is done
   while (!transactionDone)
   {
      my_nop();
   }
   // Handle not initialzied Flash
   if (!metaCorrect)
   {
      init_flash_header();
   }
}

static void init_flash_header(void)
{
   // Erase Flash
   readFromFlash = FALSE;
   progState = FLASH_ERASE_ALL;
   transactionDone = FALSE;
   enable_write();
   SPDR = CHIP_ERASE;
   while (!transactionDone)
   {
      my_nop();
   }
   // ISR will pull SS high to indicate valid comand so now we just wait until
   // Flash indicates erase finnished by reading the BUSY bit (bit 0 in SR-1)
   // which is set during erase and cleared after erase is finished
   readFromFlash = TRUE;
   readFlashStatus = TRUE;
   payloadSize = 1;
   payloadCount = 0;
   transactionDone = FALSE;
   // Send Read Status command
   SPDR = READ_STATUS_REG1;
   // Loop until chip is erased, e.g. readFlashStatus is set to FALSE
   while (!readFlashStatus)
   {
      if (transactionDone)
      {
         payloadCount = 0;
         transactionDone = FALSE;
         // Send new Read Status command
         SPDR = READ_STATUS_REG1;
      }
   }
   // Chip has been erased, Write header.
   // Fill in correct Header
   set_page_address(0, 0, 0);
   enable_write();
}

static void enable_write(void)
{
   // Pull SS low to indicate start of Write Enable command
   PORTB &= ~_BV(PORTB2);
   SPDR = WRITE_ENABLE;
   // Wait until enabled
   while (!writeEnabled)
   {
      my_nop();
   }
}