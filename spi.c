
#include "spi.h"
#include "reg_defs.h"
#include <stdio.h>

/* When configured as a Master, the SPI interface has no automatic control of the SS line. This must be handled
 * by user software before communication can start. When this is done, writing a byte to the SPI Data Register
 * starts the SPI clock generator, and the hardware shifts the eight bits into the Slave. After shifting one byte, the
 * SPI clock generator stops, setting the end of Transmission Flag (SPIF). If the SPI Interrupt Enable bit (SPIE) in
 * the SPCR Register is set, an interrupt is requested. The Master may continue to shift the next byte by writing it
 * into SPDR, or signal the end of packet by pulling high the Slave Select, SS line. The last incoming byte will be
 * kept in the Buffer Register for later use.
 * 
 * The TinyFlash W25Q80DV support SPI modes 0 and 3 so we will set CPHA 1 and CPOL 1
 *  Instructions are initiated with the falling edge of Chip Select
 *  (/CS). The first byte of data clocked into the DI input provides the instruction code. Data on the DI input
 *  is sampled on the rising edge of clock with most significant bit (MSB) first.
 * 
 * The W25Q80DV are accessed through an SPI compatible bus consisting of four signals: Serial Clock
 *  (CLK), Chip Select (/CS), Serial Data Input (DI) and Serial Data Output (DO). Standard SPI instructions
 *  use the DI input pin to serially write instructions, addresses or data to the device on the rising edge of
 *  CLK. The DO output pin is used to read data or status from the device on the falling edge CLK.
 */

void init_spi()
{
   // Enable SPI and its interrupt, MSB first -> 0 on DORD, and set as master -> 1 on MSTR
   // The clock will be 1 MHz hence since we have 8 MHz clock we prescale by 1/8 -> 1 in SPR0, 0 on SPR1 and 1 on SPI2X
   SPSR |= _BV(SPI2X);
   SPCR = _BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA) | _BV(SPR0);
}
