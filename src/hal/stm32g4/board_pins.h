/*
 * board_pins.h - STM32G474RE pin assignment for SK2V-rev1 (LQFP64).
 * Alternate-function numbers from the STM32G474 datasheet (DS12288),
 * table "Alternate function". Re-check when porting to another package.
 *
 *  Function          Pin    Periph / AF
 *  ---------------   ----   --------------------------
 *  PWM A high/low    PA8  / PB13   TIM1_CH1  AF6 / TIM1_CH1N AF6
 *  PWM B high/low    PA9  / PB14   TIM1_CH2  AF6 / TIM1_CH2N AF6
 *  PWM C high/low    PA10 / PB15   TIM1_CH3  AF6 / TIM1_CH3N AF4
 *  nFAULT -> BKIN    PB12          TIM1_BKIN AF6 (also read as GPIO)
 *  Gate enable       PB5           GPIO out
 *  I_A               PA0           ADC1_IN1  (injected)
 *  I_B               PA1           ADC2_IN2  (injected)
 *  I_C               PA6           ADC2_IN3  (injected)
 *  V_BUS             PC0           ADC1_IN6  (injected)
 *  Throttle          PC1           ADC1_IN7  (regular)
 *  NTC FET           PC2           ADC1_IN8  (regular)
 *  NTC motor         PC3           ADC1_IN9  (regular)
 *  Brake analog      PB1           ADC1_IN12 (regular)
 *  Hall A/B/C        PB6/PB7/PB8   GPIO in, pull-up
 *  Brake switch      PB9           GPIO in, pull-up, active low
 *  PAS               PB10          GPIO in, pull-up
 *  SPI3 SCK/MISO/MOSI PC10/PC11/PC12 AF6
 *  CS gate driver    PA15          GPIO out
 *  CS encoder        PC9           GPIO out
 *  USART2 TX/RX      PA2/PA3       AF7
 *  FDCAN1 RX/TX      PA11/PA12     AF9
 *  LED               PC6           GPIO out
 *  AUX (lights)      PC7           GPIO out
 *  SWDIO/SWCLK       PA13/PA14
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#define ADC_CH_IA        1u   /* ADC1 */
#define ADC_CH_VBUS      6u   /* ADC1 */
#define ADC_CH_IB        2u   /* ADC2 */
#define ADC_CH_IC        3u   /* ADC2 */
#define ADC_CH_THROTTLE  7u
#define ADC_CH_TEMP_FET  8u
#define ADC_CH_TEMP_MOT  9u
#define ADC_CH_BRAKE     12u
#define ADC_CH_VREFINT   18u

#define HSE_HZ           8000000u
#define SYSCLK_HZ        170000000u

#endif
