#ifndef _REG_DEFS_H
#define _REG_DEFS_H

/*
 * From reference manual and pad number to function:
 * 1:  PD3 (PCINT19/OC2B/INT1)
 * 2:  PD4 (PCINT20/XCK/T0)
 * 3:  GND
 * 4:  VCC
 * 5:  GND
 * 6:  VCC
 * 7:  PB6 (PCINT6/XTAL1/TOSC1)
 * 8:  PB7 (PCINT7/XTAL2/TOSC2)
 * 9:  PD5 (PCINT21/OC0B/T1)
 * 10: PD6 (PCINT22/OC0A/AIN0)
 * 11: PD7 (PCINT23/AIN1)
 * 12: PB0 (PCINT0/CLKO/ICP1)
 * 13: PB1 (PCINT1/OC1A)
 * 14: PB2 (PCINT2/SS/OC1B)
 * 15: PB3 (PCINT3/OC2A/MOSI)
 * 16: PB4 (PCINT4/MISO)
 * 17: PB5 (SCK/PCINT5)
 * 18: AVCC
 * 19: ADC6
 * 20: AREF
 * 21: GND
 * 22: ADC7
 * 23: PC0 (ADC0/PCINT8)
 * 24: PC1 (ADC1/PCINT9)
 * 25: PC2 (ADC2/PCINT10)
 * 26: PC3 (ADC3/PCINT11)
 * 27: PC4 (ADC4/SDA/PCINT12)
 * 28: PC5 (ADC5/SCL/PCINT13)
 * 29: PC6 (RESET/PCINT14
 * 30: PD0 (RXD/PCINT16)
 * 31: PD1 (TXD/PCINT17)
 * 32: PD2 (INT0/PCINT18)
 *
 * The TinuDuino schematics shows what MCU pins are connected to the expansion port J2
 * J2:1  28 PC5 (ADC5/SCL)
 * J2:3  27 PC4 (ADC4/SDA)
 * J2:5  26 PC3 (ADC3)
 * J2:7  25 PC2 (ADC2)
 * J2:9  24 PC1 (ADC1)
 * J2:10 (Note!! Schematics gives signal VIN on J2:10 but VIN is not defined anywhere else)
 * J2:11 23 PC0 (ADC0)
 * J2:13 20 AREF
 * J2:15 17 PB5 (SCK)
 * J2:16 (Note!! Schematics gives signal RSV2 on J2:16 but RSV2 is not defined anywhere else)
 * J2:17 16 PB4 (MISO)
 * J2:18 (Note!! Schematics gives signal RSV1 on J2:18 but RSV1 is not defined anywhere else)
 * J2:19 15 PB3 (OC2A/MOSI)
 * J2:20 (Note!! Schematics gives signal RSV0 on J2:20 but RSV0 is not defined anywhere else)
 * J2:21 14 PB2 (SS/OC1B)
 * J2:22 29 RESET
 * J2:23 13 PB1 (OC1A)
 * J2:24 30 PD0 (RXD)
 * J2:25 12 PB0 (CLKO/ICP1)
 * J2:26 31 PD1 (TXD)
 * J2:27 11 PD7 (AIN1)
 * J2:28 32 PD2 (INT0)
 * J2:29 10 PD6 (AIN0)
 * J2:30 1  PD3 (OC2B/INT1)
 * J2:31 9  PD5 (OC0B/T1)
 * J2:32 2  PD4 (XCK/T0)
 *
 *
 */
/*
 * The tiny stack consist of the TinyDuo processor board, TinyFlash board and TinyGPS board (TinyUSBShield is not allways connected ...)
 *
 * -----------------------------------------------------------------------------
 * TinyFlash (W25Q80DVSNIG MB85RS1MTPNF 23LC1024T) has an SPI interface:
 * SCK    J2:15 PB5
 * MOSI   J2:19 PB3
 * MISO   J2:17 PB4 (Input is automatically configured when SPI enabled)
 * SS     J2:31 PD5
 *
 * The TinyFlash reference manual specifies that SPI bus operation Modes 0 (0,0) and 3 (1,1) are supported.
 * Where they differ between 0 and 3 by:
 *  For Mode 0 the CLK signal is normally low on the falling and rising edges of /CS.
 *  For Mode 3 the CLK signal is normally high on the falling and rising edges of /CS.
 *
 * When the SPI is enabled, the data direction of the MOSI, MISO, SCK, and SS pins is overridden according to
 * Table 19-1 on page 171. For more details on automatic port overrides, refer to ”Alternate Port Functions” on
 * page 89.
 * The PRSPI bit in
 * ”Minimizing Power Consumption” on page 51 must be written to zero to enable SPI module.
 * When the SPI is configured as a Master (MSTR in SPCR is set), the user can determine the direction of the SS pin.
 * If SS is configured as an output, the pin is a general output pin which does not affect the SPI system. Typically,
 * the pin will be driving the SS pin of the SPI Slave.
 * Input pins are automatically configured. (MISO)
 * The output pins must be initialized manually by the software.
 * SPI Data Modes:
 * CPOL  CPHA  MODE   Leading Edge      Trailing edgw
 * 0     0     0      Sample (rising)   Setup (falling)
 * 0     1     1      Setup (rising)    Sample (falling)
 * 1     0     2      Sample (falling)  Setup (rising)
 * 1     1     3      Setup (falling)   Sample (rising)
 * From the net:
 * Single SPI
 * Single mode SPI works for most use cases such as rapid prototyping, device programming, and automated testing.
 * SPI is fast, with most single SPI serial throughput rates reaching around 10 Mbps.
 * Single SPI parallel throughput rates ranges from 10 – 24 Mbps. However, a single data line will not be able to
 * send data at SPI’s fastest speed.
 * Dual SPI
 * Dual SPI has a dual I/O interface that enables transfer rates to double compared to the standard serial Flash memory devices.
 * The MISO and MOSI data pins operate in half-duplex mode to send two bits per clock cycle.
 * The MOSI line become IO0 and the MISO line becomes IO1. Dual SPI serial throughput rates reach around 20 Mbps.
 * !!! Looks complicated with DUAL SPI. The ATMega328 specifies that SPI supports: Double Speed (CK/2) Master SPI Mode
 * But reading the register specifications:
 * 19.5.2 SPSR – SPI Status Register
 *  Bit 0 – SPI2X: Double SPI Speed Bit:
 *   When this bit is written logic one the SPI speed (SCK Frequency) will be doubled when the SPI is in Master
 *   mode (see Table 19-5). This means that the minimum SCK period will be two CPU clock periods. When the SPI
 *   is configured as Slave, the SPI is only ensured to work at fosc/4 or lower.
 * Means we will only run in normal Single SPI and we will use SPI Mode 3 CPOL:1, CPHA:1
 *
 * -----------------------------------------------------------------------------
 * TinyGPS Telit SE868 V2/JF2
 * Default interface is SW UART (R2/R3 populated -> ADS0/AD1) but we need higher speed, e.g. HW UART
 * so we need to move the R2/R3 resistors to R4/R1 -> IO1/IO0
 * RX     J2:26 PD1
 * TX     J2:24 PD0
 * ON/OFF J2:5  PC3  Turns ON the GPS with active high once SYS-ON indicated it is ready for it
 * SYS-ON J2:7  AD2  Is active high when ready to receive the ON pulse
 * Both RX and TX pins will override normal port operation when enabled so no need to configure them but we will do that anyway
 * NMEA Frames:
 * RMC = 1 second update
 * GGA = 1 second update
 * GSA = 1 second update
 * GSV = 5 second update
 *
 * From the JF" Hardware Guide it states that the Baudrate can be changed from default 4800 by:
 * Use the Set Serial Port (PSRF100) NMEA command to change the baud rate.
 * $PSRF100,1,19200,8,1,0*38
 * The field of checksum consists of two hex digits representing the exclusive or of all characters between,
 * but not including, the $ and * and the End of message termination is <CR><LF> (0x0D 0x0A or '\n\r')
 *
 * We can also limit NMEA sentences sent by GPS to lower the cpuload of handling not needed messages:
 * $PSRF103,00,01,00,01*25
 *
 * -----------------------------------------------------------------------------
 * USB TinyShield  FT231XQ-R
 * RX     J2:26 PD1
 * TX     J2:24 PD0
 * Since we have a conflict here we need to act differently depending on if USB shield is connected or not.
 * Default case is not connected which must be verified during setup and then can we turn on the GPS.
 * Only when USB is connected do we need to turn off the GPS to avoid UART from both at the same time.
 */

// part of sfr_defs.h and other avr includes from Arduino15 install folder, ArduinoIDE install (stdint instead of inttypes)
#include <stdint.h>

#define sei()  __asm__ __volatile__ ("sei" ::: "memory")
#define cli()  __asm__ __volatile__ ("cli" ::: "memory")

#ifndef _VECTOR
#define _VECTOR(N) __vector_##N
#endif

#define ISR(vector, ...)                                                \
   void vector(void) __attribute__((signal, __INTR_ATTRS)) __VA_ARGS__; \
   void vector(void)

#define SPI_STC_vect_num 17
#define SPI_STC_vect _VECTOR(17) /* SPI Serial Transfer Complete */

#define USART_RX_vect_num 18
#define USART_RX_vect _VECTOR(18) /* USART Rx Complete */

#define USART_UDRE_vect_num 19
#define USART_UDRE_vect _VECTOR(19) /* USART, Data Register Empty */

#define USART_TX_vect_num 20
#define USART_TX_vect _VECTOR(20) /* USART Tx Complete */

#define TIMER1_COMPA_vect_num 11
#define TIMER1_COMPA_vect _VECTOR(11) /* Timer/Counter1 Compare Match A */

#define TIMER1_COMPB_vect_num 12
#define TIMER1_COMPB_vect _VECTOR(12) /* Timer/Counter1 Compare Match B */

#define _BV(bit) (1 << (bit))
#define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & _BV(bit)))

#define __SFR_OFFSET 0x20U
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define _MMIO_WORD(mem_addr) (*(volatile uint16_t *)(mem_addr))
#define _MMIO_DWORD(mem_addr) (*(volatile uint32_t *)(mem_addr))

#define _SFR_MEM8(mem_addr) _MMIO_BYTE(mem_addr)
#define _SFR_MEM16(mem_addr) _MMIO_WORD(mem_addr)
#define _SFR_MEM32(mem_addr) _MMIO_DWORD(mem_addr)
#define _SFR_IO8(io_addr) _MMIO_BYTE((io_addr) + __SFR_OFFSET)
#define _SFR_IO16(io_addr) _MMIO_WORD((io_addr) + __SFR_OFFSET)

#define _SFR_MEM_ADDR(sfr) ((uint16_t) & (sfr))
#define _SFR_IO_ADDR(sfr) (_SFR_MEM_ADDR(sfr) - __SFR_OFFSET)
#define _SFR_IO_REG_P(sfr) (_SFR_MEM_ADDR(sfr) < 0x40U + __SFR_OFFSET)

#define _SFR_ADDR(sfr) _SFR_MEM_ADDR(sfr)
// End copy from sfr_defs.h

// Copy from iom328p.

// End copy from iom328p.

// MCU Control Register
#define MCUCR _SFR_IO8(0x35U)
#define IVCE 0
#define IVSEL 1
#define PUD 4
#define BODSE 5
#define BODS 6

// Power Reduction Register
#define PRR _SFR_MEM8(0x64U)
#define PRADC 0
#define PRUSART0 1
#define PRSPI 2
#define PRTIM1 3
#define PRTIM0 5
#define PRTIM2 6
#define PRTWI 7

// SPI Registers
#define SPCR _SFR_IO8(0x2C)
#define SPR0 0
#define SPR1 1
#define CPHA 2
#define CPOL 3
#define MSTR 4
#define DORD 5
#define SPE 6
#define SPIE 7

#define SPSR _SFR_IO8(0x2D)
#define SPI2X 0
#define WCOL 6
#define SPIF 7

#define SPDR _SFR_IO8(0x2E)
#define SPDR0 0
#define SPDR1 1
#define SPDR2 2
#define SPDR3 3
#define SPDR4 4
#define SPDR5 5
#define SPDR6 6
#define SPDR7 7

// USART Registers
#define UCSR0A _SFR_MEM8(0xC0U)
#define MPCM0 0
#define U2X0 1
#define UPE0 2
#define DOR0 3
#define FE0 4
#define UDRE0 5
#define TXC0 6
#define RXC0 7

#define UCSR0B _SFR_MEM8(0xC1U)
#define TXB80 0
#define RXB80 1
#define UCSZ02 2
#define TXEN0 3
#define RXEN0 4
#define UDRIE0 5
#define TXCIE0 6
#define RXCIE0 7

#define UCSR0C _SFR_MEM8(0xC2U)
#define UCPOL0 0
#define UCSZ00 1
#define UCPHA0 1
#define UCSZ01 2
#define UDORD0 2
#define USBS0 3
#define UPM00 4
#define UPM01 5
#define UMSEL00 6
#define UMSEL01 7

#define UBRR0 _SFR_MEM16(0xC4U)

#define UBRR0L _SFR_MEM8(0xC4U)
#define UBRR0_0 0
#define UBRR0_1 1
#define UBRR0_2 2
#define UBRR0_3 3
#define UBRR0_4 4
#define UBRR0_5 5
#define UBRR0_6 6
#define UBRR0_7 7

#define UBRR0H _SFR_MEM8(0xC5U)
#define UBRR0_8 0
#define UBRR0_9 1
#define UBRR0_10 2
#define UBRR0_11 3

#define UDR0 _SFR_MEM8(0xC6U)

// PORT B
#define PINB _SFR_IO8(0x03)
#define PINB0 0
#define PINB1 1
#define PINB2 2
#define PINB3 3
#define PINB4 4
#define PINB5 5
#define PINB6 6
#define PINB7 7

#define DDRB _SFR_IO8(0x04)
#define DDB0 0
#define DDB1 1
#define DDB2 2
#define DDB3 3
#define DDB4 4
#define DDB5 5
#define DDB6 6
#define DDB7 7

#define PORTB _SFR_IO8(0x05)
#define PORTB0 0
#define PORTB1 1
#define PORTB2 2
#define PORTB3 3
#define PORTB4 4
#define PORTB5 5
#define PORTB6 6
#define PORTB7 7

// Port C
#define PINC _SFR_IO8(0x06)
#define PINC0 0
#define PINC1 1
#define PINC2 2
#define PINC3 3
#define PINC4 4
#define PINC5 5
#define PINC6 6

#define DDRC _SFR_IO8(0x07)
#define DDC0 0
#define DDC1 1
#define DDC2 2
#define DDC3 3
#define DDC4 4
#define DDC5 5
#define DDC6 6

#define PORTC _SFR_IO8(0x08)
#define PORTC0 0
#define PORTC1 1
#define PORTC2 2
#define PORTC3 3
#define PORTC4 4
#define PORTC5 5
#define PORTC6 6

// Port D
#define PIND _SFR_IO8(0x09)
#define PIND0 0
#define PIND1 1
#define PIND2 2
#define PIND3 3
#define PIND4 4
#define PIND5 5
#define PIND6 6
#define PIND7 7

#define DDRD _SFR_IO8(0x0A)
#define DDD0 0
#define DDD1 1
#define DDD2 2
#define DDD3 3
#define DDD4 4
#define DDD5 5
#define DDD6 6
#define DDD7 7

#define PORTD _SFR_IO8(0x0B)
#define PORTD0 0
#define PORTD1 1
#define PORTD2 2
#define PORTD3 3
#define PORTD4 4
#define PORTD5 5
#define PORTD6 6
#define PORTD7 7

// Counter 1
#define TCCR1A _SFR_MEM8(0x80)
#define WGM10 0
#define WGM11 1
#define COM1B0 4
#define COM1B1 5
#define COM1A0 6
#define COM1A1 7

#define TCCR1B _SFR_MEM8(0x81)
#define CS10 0
#define CS11 1
#define CS12 2
#define WGM12 3
#define WGM13 4
#define ICES1 6
#define ICNC1 7

#define TCCR1C _SFR_MEM8(0x82)
#define FOC1B 6
#define FOC1A 7

#define TCNT1 _SFR_MEM16(0x84)

#define TCNT1L _SFR_MEM8(0x84)
#define TCNT1L0 0
#define TCNT1L1 1
#define TCNT1L2 2
#define TCNT1L3 3
#define TCNT1L4 4
#define TCNT1L5 5
#define TCNT1L6 6
#define TCNT1L7 7

#define TCNT1H _SFR_MEM8(0x85)
#define TCNT1H0 0
#define TCNT1H1 1
#define TCNT1H2 2
#define TCNT1H3 3
#define TCNT1H4 4
#define TCNT1H5 5
#define TCNT1H6 6
#define TCNT1H7 7

#define ICR1 _SFR_MEM16(0x86)

#define ICR1L _SFR_MEM8(0x86)
#define ICR1L0 0
#define ICR1L1 1
#define ICR1L2 2
#define ICR1L3 3
#define ICR1L4 4
#define ICR1L5 5
#define ICR1L6 6
#define ICR1L7 7

#define ICR1H _SFR_MEM8(0x87)
#define ICR1H0 0
#define ICR1H1 1
#define ICR1H2 2
#define ICR1H3 3
#define ICR1H4 4
#define ICR1H5 5
#define ICR1H6 6
#define ICR1H7 7

#define OCR1A _SFR_MEM16(0x88)

#define OCR1AL _SFR_MEM8(0x88)
#define OCR1AL0 0
#define OCR1AL1 1
#define OCR1AL2 2
#define OCR1AL3 3
#define OCR1AL4 4
#define OCR1AL5 5
#define OCR1AL6 6
#define OCR1AL7 7

#define OCR1AH _SFR_MEM8(0x89)
#define OCR1AH0 0
#define OCR1AH1 1
#define OCR1AH2 2
#define OCR1AH3 3
#define OCR1AH4 4
#define OCR1AH5 5
#define OCR1AH6 6
#define OCR1AH7 7

#define OCR1B _SFR_MEM16(0x8A)

#define OCR1BL _SFR_MEM8(0x8A)
#define OCR1BL0 0
#define OCR1BL1 1
#define OCR1BL2 2
#define OCR1BL3 3
#define OCR1BL4 4
#define OCR1BL5 5
#define OCR1BL6 6
#define OCR1BL7 7

#define OCR1BH _SFR_MEM8(0x8B)
#define OCR1BH0 0
#define OCR1BH1 1
#define OCR1BH2 2
#define OCR1BH3 3
#define OCR1BH4 4
#define OCR1BH5 5
#define OCR1BH6 6
#define OCR1BH7 7

#define TIMSK1 _SFR_MEM8(0x6F)
#define TOIE1 0
#define OCIE1A 1
#define OCIE1B 2
#define ICIE1 5

#endif