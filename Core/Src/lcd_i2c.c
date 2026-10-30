#include "lcd_i2c.h"
#include "stm32f411xe.h"

/* PCF8574 -> LCD pin mapping (standard backpack):
 *   P0 = RS, P1 = RW (kept 0), P2 = EN, P3 = Backlight, P4..P7 = D4..D7 */
#define LCD_RS          0x01U
#define LCD_EN          0x04U
#define LCD_BL          0x08U

/* Fail fast: ~3ms worst case per byte at 16MHz, not 6ms */
#define I2C_TIMEOUT     50000U

static uint8_t lcd_bl = LCD_BL;     /* backlight state */

/* ------------------------------------------------------------------ */
/*  DWT-based microsecond delay (self-contained, no TIM2 dependency)   */
/* ------------------------------------------------------------------ */
static void delay_us_lcd(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * LCD_CPU_MHZ;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks);
}

static void delay_ms_lcd(uint32_t ms)
{
    while (ms--)
        delay_us_lcd(1000U);
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ------------------------------------------------------------------ */
/*  Hardware I2C1 (PB6 = SCL, PB7 = SDA, AF4)                         */
/* ------------------------------------------------------------------ */
void I2C1_Init(void)
{
    /* 1. Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    delay_us_lcd(10);   /* let clock stabilise before touching registers */

    /* 2. PB6 (SCL), PB7 (SDA): AF4, open-drain, high speed, no pull */
    GPIOB->MODER   &= ~((3U << 12) | (3U << 14));
    GPIOB->MODER   |=  ((2U << 12) | (2U << 14));   /* Alternate Function */
    GPIOB->OTYPER  |=  (1U << 6) | (1U << 7);       /* Open-drain */
    GPIOB->OSPEEDR |=  (3U << 12) | (3U << 14);     /* Very high speed */
    GPIOB->PUPDR   &= ~((3U << 12) | (3U << 14));   /* No pull (backpack has external) */
    GPIOB->AFR[0]  &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB->AFR[0]  |=  ((4U  << 24) | (4U  << 28)); /* AF4 = I2C1 */

    /* 3. Software reset the peripheral to clear any stuck BUSY flag */
    I2C1->CR1 |=  I2C_CR1_SWRST;
    delay_us_lcd(10);               /* hold reset for at least a few us */
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    delay_us_lcd(10);               /* settle before programming registers */

    /* 4. Configure for Standard mode 100 kHz (APB1 = 16 MHz) */
    I2C1->CR2   = LCD_PCLK1_MHZ;                     /* FREQ = 16         */
    I2C1->CCR   = (LCD_PCLK1_MHZ * 1000U) / 200U;    /* CCR = 80 → 100kHz */
    I2C1->TRISE = LCD_PCLK1_MHZ + 1U;                /* TRISE = 17        */

    /* 5. Enable peripheral */
    I2C1->CR1 |= I2C_CR1_PE;
    delay_us_lcd(10);
}

/* ------------------------------------------------------------------ */
/*  I2C write helper — returns 1 on success, 0 on any error/timeout    */
/* ------------------------------------------------------------------ */
#define WAIT_FOR(cond)                          \
    do {                                        \
        t = I2C_TIMEOUT;                        \
        while (!(cond)) {                       \
            if (--t == 0U) goto fail;           \
        }                                       \
    } while (0)

static uint8_t i2c_write(uint8_t addr7, const uint8_t *data, uint8_t len)
{
    uint32_t t;
    uint8_t  i;

    /* If bus stuck busy, reset and bail */
    t = I2C_TIMEOUT;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if (--t == 0U) { I2C1_Init(); return 0; }
    }

    I2C1->CR1 |= I2C_CR1_START;
    WAIT_FOR(I2C1->SR1 & I2C_SR1_SB);

    I2C1->DR = (uint8_t)(addr7 << 1);   /* write direction */

    WAIT_FOR(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF));
    if (I2C1->SR1 & I2C_SR1_AF) goto fail;   /* NACK: no device at address */

    (void)I2C1->SR1;
    (void)I2C1->SR2;   /* reading SR2 clears ADDR */

    for (i = 0; i < len; i++) {
        WAIT_FOR(I2C1->SR1 & I2C_SR1_TXE);
        I2C1->DR = data[i];
    }

    WAIT_FOR(I2C1->SR1 & I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
    return 1;

fail:
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |=  I2C_CR1_STOP;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  PCF8574 → LCD 4-bit protocol                                       */
/* ------------------------------------------------------------------ */

/*
 * Send a single nibble (upper 4 bits of 'n') to the LCD with an EN pulse.
 * Used ONLY during the power-on reset sequence (LCD still in 8-bit mode).
 */
static void lcd_write_nibble(uint8_t n)
{
    uint8_t buf[2];
    n = (uint8_t)((n & 0xF0U) | lcd_bl);   /* keep backlight, clear RS/EN/RW */
    buf[0] = n | LCD_EN;                    /* EN high  */
    buf[1] = n;                             /* EN low   */
    i2c_write(LCD_I2C_ADDR, buf, 2);
    delay_us_lcd(100);                      /* extra settling after EN pulse */
}

/*
 * Send a full byte as two nibbles in ONE I2C transaction.
 * rs = 0 for command, rs = LCD_RS (0x01) for data.
 */
static void lcd_write_byte(uint8_t val, uint8_t rs)
{
    uint8_t hi = (uint8_t)((val & 0xF0U)        | lcd_bl | rs);
    uint8_t lo = (uint8_t)(((val << 4) & 0xF0U) | lcd_bl | rs);
    uint8_t buf[4];

    buf[0] = hi | LCD_EN;   /* high nibble, EN high */
    buf[1] = hi;             /* high nibble, EN low  */
    buf[2] = lo | LCD_EN;   /* low  nibble, EN high */
    buf[3] = lo;             /* low  nibble, EN low  */

    i2c_write(LCD_I2C_ADDR, buf, 4);
    delay_us_lcd(50);        /* > 37 us command execution time */
}

static void lcd_cmd(uint8_t cmd)  { lcd_write_byte(cmd, 0);      }
static void lcd_data(uint8_t dat) { lcd_write_byte(dat, LCD_RS); }

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void LCD_Init(void)
{
    dwt_init();
    I2C1_Init();

    /*
     * KS0066 / HD44780 power-on 4-bit initialisation sequence.
     * Delays are from the JHD162A datasheet — they are intentionally
     * generous to guarantee correct operation across all units.
     */
    delay_ms_lcd(100);      /* Wait > 40 ms after Vcc reaches 2.7 V */

    /* Step 1 — three 0x30 nibbles to force the LCD into a known 8-bit state */
    lcd_write_nibble(0x30);
    delay_ms_lcd(10);       /* Must be > 4.1 ms */

    lcd_write_nibble(0x30);
    delay_ms_lcd(5);        /* Must be > 100 µs (use 5 ms for reliability) */

    lcd_write_nibble(0x30);
    delay_ms_lcd(5);        /* Must be > 100 µs */

    /* Step 2 — switch to 4-bit interface */
    lcd_write_nibble(0x20);
    delay_ms_lcd(5);

    /* Step 3 — configure with full 2-nibble commands (now in 4-bit mode) */
    lcd_cmd(0x28);          /* Function Set: 4-bit, 2 lines, 5x8 font */
    delay_ms_lcd(5);

    lcd_cmd(0x08);          /* Display OFF */
    delay_ms_lcd(5);

    lcd_cmd(0x01);          /* Display Clear */
    delay_ms_lcd(5);        /* Clear command needs > 1.6 ms */

    lcd_cmd(0x06);          /* Entry Mode: auto-increment, no display shift */
    delay_ms_lcd(5);

    lcd_cmd(0x0C);          /* Display ON, cursor OFF, blink OFF */
    delay_ms_lcd(5);
}

void LCD_Clear(void)
{
    lcd_cmd(0x01);
    delay_ms_lcd(5);        /* > 1.6 ms execution time */
}

void LCD_Home(void)
{
    lcd_cmd(0x02);
    delay_ms_lcd(5);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    /* DDRAM addresses: row 0 starts at 0x00, row 1 starts at 0x40 */
    static const uint8_t row_addr[2] = {0x00U, 0x40U};
    if (row > 1U)  row = 1U;
    if (col > 15U) col = 15U;
    lcd_cmd((uint8_t)(0x80U | (row_addr[row] + col)));
}

void LCD_PrintChar(char c)
{
    lcd_data((uint8_t)c);
}

void LCD_Print(const char *str)
{
    while (*str)
        lcd_data((uint8_t)*str++);
}

void LCD_PrintUInt(uint32_t value, uint8_t width)
{
    char    buf[11];
    uint8_t n = 0;

    do {
        buf[n++] = (char)('0' + (value % 10U));
        value   /= 10U;
    } while (value && n < (uint8_t)sizeof(buf));

    /* Left-pad with zeros if width requested */
    while (width > n && width <= 10U) {
        lcd_data('0');
        width--;
    }

    /* Print digits in reverse (most-significant first) */
    while (n)
        lcd_data((uint8_t)buf[--n]);
}

void LCD_Backlight(uint8_t on)
{
    uint8_t b;
    lcd_bl = on ? LCD_BL : 0U;
    b = lcd_bl;
    i2c_write(LCD_I2C_ADDR, &b, 1);
}
