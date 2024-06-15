#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>


/*
 * SI5351 addr = 0x60
 * 24C64 has 16bit addresing via 2 addres bytes i2c
 * but 24C04, 24C08, 24C16 uses b3-b1 in addres as A10-A8
 *
 * for 24C64 you can page write up to 32bytes if A15 = A5; if not, it will roll over to addr 0 of page
 *
 * TODO: formula for page number
 */


void clock_setup(void)
{
  rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);
  
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_USART1);
}

void gpio_setup(void)
{
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO4);
  
  /*
   * I2C2
   * PA11 = SCL; PA12 = SDA
   */

  i2c_peripheral_disable(I2C2);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF6, GPIO11 | GPIO12);

  i2c_set_speed(I2C2, i2c_speed_sm_100k, (rcc_apb1_frequency / 1000000));

  i2c_peripheral_enable(I2C2);
}

void uart_setup(void)
{
  /*
   * USART1
   * PB6 = TX; PB7 = RX
   */

  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
  gpio_set_af(GPIOB, GPIO_AF0, GPIO6 | GPIO7);
  
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  usart_enable(USART1);
}

int main(void)
{
  clock_setup();
  gpio_setup();

  gpio_clear(GPIOA, GPIO4);
  
  uart_setup();

  uint8_t buffer[32] = {0};
  uint8_t command_buffer[12] = {0};

  
  for(uint32_t i = 0; i <= 10000000; i++)
    __asm__("NOP");

  command_buffer[0] = 0x00;
  command_buffer[1] = 0x00;
  command_buffer[2] = 0x43;

  i2c_transfer7(I2C2, 0x50, command_buffer, 2, buffer, 32);
  
  
  for(uint32_t i = 0; i <= 10000000; i++)
    __asm__("NOP");
  

  
  for(uint16_t i = 0; i <= 31; i++)
    {
      usart_send_blocking(USART1, buffer[i]);
    }
  
  
  usart_send_blocking(USART1, 0x00);
 
  gpio_set(GPIOA, GPIO4);
  
  return 0;
}

/*
  //i2c addres scanner
  for(uint16_t addr = 0; addr <= 127; addr++)
    {
      i2c_set_7bit_address(I2C2, 0x50);
      i2c_set_read_transfer_dir(I2C2);
      i2c_set_bytes_to_transfer(I2C2, 0);
      i2c_send_start(I2C2);
      i2c_enable_autoend(I2C2);

      while (i2c_received_data(I2C2) == 0)
	{
	  if(i2c_nack(I2C2))
	    {
	      break;
	    }
	  else
	    {
	      for(uint32_t j = 0; j <= 6400000; j++)
		__asm__("NOP");
	    }
	}
      i2c_get_data(I2C2);
      i2c_send_stop(I2C2);

      for(uint32_t i = 0; i <= 64000; i++)
	__asm__("NOP");
	}
  */
