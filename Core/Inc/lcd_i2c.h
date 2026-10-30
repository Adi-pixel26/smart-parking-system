#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

/* ---------------- Configuration ---------------- */

/* 7-bit I2C address of the PCF8574T backpack.
 * PCF8574T  (A0-A2 open)  -> 0x27
 * PCF8574AT (A0-A2 open)  -> 0x3F                                   */
#define LCD_I2C_ADDR     0x27U

/* Clock values (HSI = 16 MHz, APB1 prescaler = 1).
 * Change these if you switch to the PLL / HSE.                       */
#define LCD_PCLK1_MHZ    16U    /* APB1 clock feeding I2C1 */
#define LCD_CPU_MHZ      16U    /* Core clock (used for us delays) */

/* ---------------- Pins (I2C1, AF4) -------------- */
/*   PB6 -> SCL
 *   PB7 -> SDA                                                       */

/* ---------------- API --------------------------- */

void    I2C1_Init(void);                              /* GPIO + I2C1 peripheral   */
void    LCD_Init(void);                               /* calls I2C1_Init() itself */
void    LCD_Clear(void);
void    LCD_Home(void);
void    LCD_SetCursor(uint8_t row, uint8_t col);      /* row 0..1, col 0..15      */
void    LCD_Print(const char *str);
void    LCD_PrintChar(char c);
void    LCD_PrintUInt(uint32_t value, uint8_t width); /* zero-padded to 'width'   */
void    LCD_Backlight(uint8_t on);                    /* 1 = on, 0 = off          */

#endif /* LCD_I2C_H */
