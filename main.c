#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>


/*
 * 24C64 has 16bit addresing via 2 addres bytes i2c
 * but 24C04, 24C08, 24C16 uses b3-b1 in addres as A10-A8
 *
 * for 24C64 you can page write up to 32bytes if A15 = A5; if not, it will roll over to addr 0 of page
 *
 * TODO: formula for page number
 */


void clock_setup(void)
{
  rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ])
  
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_USART1);

}

void gpio_setup(void)
{
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);

  /*
   * I2C2
   * PA11 = SCL; PA12 = SDA
   */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11 | GPIO12);
  gpio_set_af(GPIOa, GPIO_AF6, GPIO11 | GPIO12);

  i2c_set_speed(I2C2, i2c_speed_fm_400k, 64);
}

int main(void)
{
  clock_setup();
  gpio_setup();
  
  //uint8_t page_buffer[32] = {0};
  
  gpio_set(GPIOA, GPIO4);
  
  return 0;
}
