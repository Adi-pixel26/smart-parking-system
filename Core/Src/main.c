/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Pure Bare-Metal Smart Parking System (STM32F411)
  *                   - 2x HC-SR04 with EXTI Hardware Interrupts (PA0-PA3)
  *                   - 2x Servo Motors (TIM3 PWM on PA6 and PA7)
  *                   - JHD162A LCD via PCF8574T on PB6/PB7 (Hardware I2C1)
  ******************************************************************************
  */
/* USER CODE END Header */

#include <stdint.h>
#include "stm32f411xe.h"
#include "lcd_i2c.h"

#define MAX_SLOTS 5

/* ---- SENSOR 1 (Entry: Trig=PA0, Echo=PA1 via EXTI1) ---- */
volatile uint32_t start_time1  = 0;
volatile float    distance_cm1 = -1.0f;
volatile uint8_t  echo_state1  = 0;   /* 0=await rising, 1=await falling */
volatile uint8_t  echo_ready1  = 0;   /* set by ISR when distance is valid */

/* ---- SENSOR 2 (Exit:  Trig=PA2, Echo=PA3 via EXTI3) ---- */
volatile uint32_t start_time2  = 0;
volatile float    distance_cm2 = -1.0f;
volatile uint8_t  echo_state2  = 0;
volatile uint8_t  echo_ready2  = 0;

/* ---- SysTick ms counter ---- */
volatile uint32_t ms_ticks = 0;

/* ---- Parking state ---- */
volatile int8_t available_slots = MAX_SLOTS;

/* ------------------------------------------------------------------ */
/*  Function prototypes                                                 */
/* ------------------------------------------------------------------ */
void     SystemClock_Config(void);
void     SysTick_Init(void);
uint32_t get_tick(void);
void     delay_us(uint32_t us);   /* TIM2-based, used for sensor pulses */
void     delay_ms(uint32_t ms);
void     Error_Handler(void);

/* ------------------------------------------------------------------ */
/*  SysTick — 1 ms timebase                                            */
/* ------------------------------------------------------------------ */
void SysTick_Init(void)
{
    SysTick->LOAD = (16000000U / 1000U) - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk    |
                    SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) { ms_ticks++; }

uint32_t get_tick(void) { return ms_ticks; }

/* ------------------------------------------------------------------ */
/*  TIM2-based delays (sensor timing only — NOT used by lcd_i2c.c)    */
/* ------------------------------------------------------------------ */
void delay_us(uint32_t us)
{
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
        delay_us(1000U);
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */
int main(void)
{
    SystemClock_Config();
    SysTick_Init();

    /* 1. Peripheral clocks */
    RCC->AHB1ENR |= (1U << 0);               /* GPIOA */
    RCC->AHB1ENR |= (1U << 1);               /* GPIOB */
    RCC->APB1ENR |= (1U << 0) | (1U << 1);  /* TIM2, TIM3 */
    RCC->APB2ENR |= (1U << 14);              /* SYSCFG (EXTI) */

    /* 2. Ultrasonic sensor GPIO
     *    PA0 = Trig1 (Output), PA1 = Echo1 (Input + pull-down)
     *    PA2 = Trig2 (Output), PA3 = Echo2 (Input + pull-down)   */
    GPIOA->MODER &= ~((3U<<0)|(3U<<2)|(3U<<4)|(3U<<6));
    GPIOA->MODER |=  (1U<<0) | (1U<<4);      /* PA0, PA2 = output */
    /* PA1, PA3 remain input (00) */
    GPIOA->PUPDR &= ~((3U<<2)|(3U<<6));
    GPIOA->PUPDR |=  (2U<<2) | (2U<<6);      /* PA1, PA3 pull-down */

    /* 3. Servo PWM pins: PA6 (TIM3_CH1) and PA7 (TIM3_CH2) → AF2 */
    GPIOA->MODER   &= ~((3U<<12)|(3U<<14));
    GPIOA->MODER   |=  (2U<<12)|(2U<<14);
    GPIOA->OSPEEDR |=  (3U<<12)|(3U<<14);
    GPIOA->AFR[0]  &= ~((0xFU<<24)|(0xFU<<28));
    GPIOA->AFR[0]  |=  (2U<<24)|(2U<<28);    /* AF2 = TIM3 */

    /* 4. TIM2 — 1 MHz free-running counter for echo timing (1 tick = 1 µs) */
    TIM2->PSC = 16U - 1U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->EGR = 1U;
    TIM2->CR1 = 1U;

    /* 5. TIM3 — 50 Hz PWM for servos (20 ms period, 1 MHz counter clock) */
    TIM3->PSC   = 16U - 1U;
    TIM3->ARR   = 20000U - 1U;
    TIM3->CCR1  = 1000U;         /* 1.0 ms = 0° (closed) */
    TIM3->CCR2  = 1000U;
    TIM3->CCMR1 = (6U<<4)|(1U<<3)|(6U<<12)|(1U<<11); /* PWM1 + preload CH1&CH2 */
    TIM3->CCER  = (1U<<0)|(1U<<4);                    /* enable CH1 & CH2 */
    TIM3->EGR   = 1U;
    TIM3->CR1   = 1U;

    /* Power-on servo sweep so you can confirm hardware is alive */
    TIM3->CCR1 = 2000U; TIM3->CCR2 = 2000U;
    delay_ms(500U);
    TIM3->CCR1 = 1000U; TIM3->CCR2 = 1000U;
    delay_ms(300U);

    /* 6. EXTI — hardware interrupts on PA1 (EXTI1) and PA3 (EXTI3)
     *    Both start armed on the rising edge.                         */
    SYSCFG->EXTICR[0] &= ~((0xFU<<4)|(0xFU<<12)); /* PA1, PA3 → Port A */
    EXTI->IMR  |=  (1U<<1)|(1U<<3);
    EXTI->RTSR |=  (1U<<1)|(1U<<3);
    EXTI->FTSR &= ~((1U<<1)|(1U<<3));
    EXTI->PR    =  (1U<<1)|(1U<<3);               /* clear pending */
    NVIC_SetPriority(EXTI1_IRQn, 1U);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI3_IRQn, 2U);
    NVIC_EnableIRQ(EXTI3_IRQn);

    /* 7. LCD — hardware I2C1, DWT timer, full KS0066 init sequence */
    LCD_Init();

    LCD_SetCursor(0, 0);
    LCD_Print("    WELCOME!    ");
    LCD_SetCursor(1, 0);
    LCD_Print(" SMART PARKING  ");
    delay_ms(2000U);

    LCD_Clear();

    /* Initial display */
    LCD_SetCursor(0, 0);
    LCD_Print("Slots Left: 5/5 ");
    LCD_SetCursor(1, 0);
    LCD_Print("Status: OPEN    ");

    /* ----------------------------------------------------------------
     * Main loop state
     * ---------------------------------------------------------------- */
    uint32_t last_trigger_ms    = 0;
    uint8_t  active_sensor      = 1;
    uint8_t  car1_detected      = 0;
    uint8_t  car2_detected      = 0;
    int8_t   prev_slots         = MAX_SLOTS;

    while (1)
    {
        /* ============================================================
         * A. SENSOR TRIGGER — every 25 ms, alternating sensors
         *    Each sensor gets a fresh trigger every 50 ms.
         *    HC-SR04 max echo = 38 ms so this is safe.
         * ============================================================ */
        if (get_tick() - last_trigger_ms >= 25U)
        {
            last_trigger_ms = get_tick();

            if (active_sensor == 1)
            {
                /* Reset state for sensor 1 */
                if (!echo_ready1) distance_cm1 = -1.0f;
                echo_ready1 = 0;
                echo_state1 = 0;

                /* Re-arm rising edge on PA1 */
                EXTI->RTSR |=  (1U<<1);
                EXTI->FTSR &= ~(1U<<1);
                EXTI->PR    =  (1U<<1);

                /* 10 µs trigger pulse on PA0 */
                GPIOA->BSRR = (1U<<0);
                delay_us(10U);
                GPIOA->BSRR = (1U<<16);

                active_sensor = 2;
            }
            else
            {
                /* Reset state for sensor 2 */
                if (!echo_ready2) distance_cm2 = -1.0f;
                echo_ready2 = 0;
                echo_state2 = 0;

                /* Re-arm rising edge on PA3 */
                EXTI->RTSR |=  (1U<<3);
                EXTI->FTSR &= ~(1U<<3);
                EXTI->PR    =  (1U<<3);

                /* 10 µs trigger pulse on PA2 */
                GPIOA->BSRR = (1U<<2);
                delay_us(10U);
                GPIOA->BSRR = (1U<<18);

                active_sensor = 1;
            }

            /* --------------------------------------------------------
             * B. SERVO CONTROL — checked every 25 ms (instant response)
             * -------------------------------------------------------- */

            /* Entry gate (Sensor 1 / Servo 1) */
            if (distance_cm1 > 0.0f && distance_cm1 < 10.0f)
            {
                if (available_slots > 0)
                {
                    TIM3->CCR1 = 2000U;   /* open  (90°) */
                    if (!car1_detected)
                    {
                        available_slots--;
                        car1_detected = 1;
                    }
                }
                else
                {
                    TIM3->CCR1 = 1000U;   /* stay closed — FULL */
                }
            }
            else
            {
                TIM3->CCR1 = 1000U;       /* close gate */
                car1_detected = 0;
            }

            /* Exit gate (Sensor 2 / Servo 2) */
            if (distance_cm2 > 0.0f && distance_cm2 < 10.0f)
            {
                TIM3->CCR2 = 2000U;       /* open  (90°) */
                if (!car2_detected)
                {
                    if (available_slots < MAX_SLOTS) available_slots++;
                    car2_detected = 1;
                }
            }
            else
            {
                TIM3->CCR2 = 1000U;       /* close gate */
                car2_detected = 0;
            }
        }

        /* ============================================================
         * C. LCD UPDATE — ONLY when the slot count changes.
         *    This keeps I2C calls completely out of the sensor cycle.
         * ============================================================ */
        if (available_slots != prev_slots)
        {
            prev_slots = available_slots;

            if (available_slots <= 0)
            {
                LCD_SetCursor(0, 0);
                LCD_Print("PARKING FULL!   ");
                LCD_SetCursor(1, 0);
                LCD_Print("PLEASE WAIT...  ");
            }
            else
            {
                LCD_SetCursor(0, 0);
                LCD_Print("Slots Left: ");
                LCD_PrintChar('0' + (char)available_slots);
                LCD_Print("/5 ");

                LCD_SetCursor(1, 0);
                LCD_Print("Status: OPEN    ");
            }
        }
    }
}

/* ------------------------------------------------------------------ */
void SystemClock_Config(void) {}

/* ================================================================== */
/*  EXTI ISRs — ultrasonic echo measurement                            */
/* ================================================================== */

/* EXTI1 → PA1 = Echo 1 */
void EXTI1_IRQHandler(void)
{
    if (!(EXTI->PR & (1U<<1))) return;
    EXTI->PR = (1U<<1);

    if (echo_state1 == 0)
    {
        /* Rising edge — start timing */
        start_time1 = TIM2->CNT;
        echo_state1 = 1;

        EXTI->RTSR &= ~(1U<<1);
        EXTI->FTSR |=  (1U<<1);
    }
    else
    {
        /* Falling edge — compute distance */
        uint32_t pulse = TIM2->CNT - start_time1;
        if (pulse >= 100U && pulse <= 30000U)
            distance_cm1 = (pulse * 0.0343f) / 2.0f;

        echo_ready1  = 1;
        echo_state1  = 0;

        EXTI->FTSR &= ~(1U<<1);
        EXTI->RTSR |=  (1U<<1);
    }
}

/* EXTI3 → PA3 = Echo 2 */
void EXTI3_IRQHandler(void)
{
    if (!(EXTI->PR & (1U<<3))) return;
    EXTI->PR = (1U<<3);

    if (echo_state2 == 0)
    {
        start_time2 = TIM2->CNT;
        echo_state2 = 1;

        EXTI->RTSR &= ~(1U<<3);
        EXTI->FTSR |=  (1U<<3);
    }
    else
    {
        uint32_t pulse = TIM2->CNT - start_time2;
        if (pulse >= 100U && pulse <= 30000U)
            distance_cm2 = (pulse * 0.0343f) / 2.0f;

        echo_ready2  = 1;
        echo_state2  = 0;

        EXTI->FTSR &= ~(1U<<3);
        EXTI->RTSR |=  (1U<<3);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
