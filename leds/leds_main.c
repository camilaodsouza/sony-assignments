#include <sdk/config.h>
#include <stdio.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

//Blinks LEDs 1 to 4 in sequence a couple of times.

int leds_main(int argc, char *argv[])
{
  printf("Blink LEDS!!!\n");
  int count = 0;
  for (int i = 0; i < 4; i++)
  {
    switch (i)
    {
    case 0:
      board_gpio_write(PIN_I2S1_BCK, 1);
      sleep(1);
      board_gpio_write(PIN_I2S1_BCK, 0);
      break;
    case 1:
      board_gpio_write(PIN_I2S1_LRCK, 1);
      sleep(1);
      board_gpio_write(PIN_I2S1_LRCK, 0);
      break;
    case 2:
      board_gpio_write(PIN_I2S1_DATA_IN, 1);
      sleep(1);
      board_gpio_write(PIN_I2S1_DATA_IN, 0);
      break;
    case 3:
      board_gpio_write(PIN_I2S1_DATA_OUT, 1);
      sleep(1);
      board_gpio_write(PIN_I2S1_DATA_OUT, 0);
      i = -1;
      break;
    default:
      break;
    }
    count++;
    if (count == 8)
      break;
  }
  return 0;
}
