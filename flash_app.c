#include "reg_defs.h"
#include "flash_shield.h"
#include "common.h"

volatile boolean store_track;

void init_system(void)
{
   // Configure PD6/PD7 Input with Pull-Up
   PORTD |= _BV(PORTD6)|_BV(PORTD7);
   // Deactivate TWI, Counter2, Counter0, ADC
   PRR = _BV(PRADC) | _BV(PRTIM0) | _BV(PRTIM2) | _BV(PRTWI);

   // Configure Counter 1 Mode to CTC. We will use OCR1A for Match A in the error_handler ISR switch to new value
   // and using OCR1B for Match B in the GPS ON/OFF pulse ISR
   // WGM Mode is 4 hence WGM bits in TCCR1A should be kept at 0 and sice we use low frequencies
   // we will prescale the clock with 8 -> 1 us tick (OCR1B used to start GPS) Prescaler will be set to 1024 for error handler
   TCCR1B = _BV(WGM12) | _BV(CS11);

   // Disable interrupts when initiating drivers
   cli();

   // Init the SPI
   init_spi();

   // Init the Flash
   init_flash();

   sei();
}

int main(void)
{
   // Perform initialisation of the modules
   init_system();
   // Main loop
   while (TRUE)
   {
      if (store_track)
      {
         // call shield to store track
         uint8_t data[6] = {1, 2, 3, 4, 5, 6};
         store_track_data(data);
      }
   }

}



