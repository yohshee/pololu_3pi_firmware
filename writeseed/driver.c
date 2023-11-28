/*
 * Used to write the initial random seed and an agent ID to the Pololu 3pi.
 *
 * Author: Rick Coogle
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/fuse.h>
#include <avr/eeprom.h>
#include <pololu/3pi.h>

#define ID_ADDR    ((uint8_t*)(0x0F0))
#define SEED_ADDR  ((uint16_t*)(0x1F0))
#define INIT_SEED  0x5EED

int main(void)
{
  unsigned char button;
  uint16_t uValue;
  uint8_t uId;

  lcd_init_printf();

  eeprom_busy_wait();
  uId = eeprom_read_byte(ID_ADDR);
 
  /* Print confirmation, wait for button press */
  lcd_goto_xy(0,0);
  printf("oid=%u", uId);
 
  uId = 1;
  while(1) {
    lcd_goto_xy(0,1);
    printf("id=%03u", uId);

    button = get_single_debounced_button_press(ANY_BUTTON);

    if(button & BUTTON_A)
      uId--;
    else if(button & BUTTON_B)
      uId++;
    else if(button & BUTTON_C)
      break;
  }

  green_led(HIGH);
  red_led(HIGH);
  
  /* Write the seed */
  eeprom_busy_wait();
  eeprom_write_word(SEED_ADDR, INIT_SEED);

  /* Write the ID */
  eeprom_busy_wait();
  eeprom_write_byte(ID_ADDR, uId);

  /* Confirm the write worked */
  eeprom_busy_wait();
  uValue = eeprom_read_word(SEED_ADDR);
  
  green_led(LOW);
  red_led(LOW);

  /* Print stop */
  clear();
  if(uValue == INIT_SEED)
    print("Done.");
  else
    print("Fail.");

  return 0;
}

