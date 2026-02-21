#include "i2c_lcd.h"

static I2C_HandleTypeDef *lcd_i2c;
static uint8_t lcd_addr8; // (addr_7bit << 1)

#define LCD_BL  0x08
#define LCD_EN  0x04
#define LCD_RS  0x01

// petit delay "micro" approximatif (sans timer)
static void dly_short(void)
{
  for (volatile int i = 0; i < 2000; i++) { __NOP(); }
}

static void i2c_write(uint8_t data)
{
  // envoie 1 octet, et on laisse respirer
  HAL_I2C_Master_Transmit(lcd_i2c, lcd_addr8, &data, 1, 100);
  dly_short();
}

static void pulse(uint8_t data)
{
  i2c_write(data | LCD_EN);
  dly_short();
  i2c_write(data & ~LCD_EN);
  dly_short();
}

static void write4(uint8_t nibble, uint8_t mode)
{
  // nibble doit être déjà dans le haut (xxxx0000)
  uint8_t data = (nibble & 0xF0) | mode | LCD_BL;
  i2c_write(data);
  pulse(data);
}

static void send8(uint8_t value, uint8_t mode)
{
  write4(value & 0xF0, mode);
  write4((value << 4) & 0xF0, mode);
  HAL_Delay(2);
}

static void cmd(uint8_t c) { send8(c, 0); }
static void dat(uint8_t c) { send8(c, LCD_RS); }

void LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
  lcd_i2c = hi2c;
  lcd_addr8 = (uint8_t)(addr_7bit << 1);

  HAL_Delay(50);

  // séquence init classique HD44780 en 4-bit
  write4(0x30, 0); HAL_Delay(5);
  write4(0x30, 0); HAL_Delay(5);
  write4(0x30, 0); HAL_Delay(5);
  write4(0x20, 0); HAL_Delay(5);

  cmd(0x28); // 4-bit, 2 lignes, 5x8
  cmd(0x0C); // display ON, cursor OFF
  cmd(0x06); // entry mode
  cmd(0x01); // clear
  HAL_Delay(5);
}

void LCD_Clear(void)
{
  cmd(0x01);
  HAL_Delay(5);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t a = (row == 0) ? (0x00 + col) : (0x40 + col);
  cmd(0x80 | a);
}

void LCD_Print(const char *s)
{
  while (*s)
  {
    dat((uint8_t)*s++);
    HAL_Delay(1);
  }
}
