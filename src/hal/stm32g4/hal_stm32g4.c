/*
 * hal_stm32g4.c - register-level HAL for STM32G474 (CMSIS headers only).
 *
 * Timing architecture:
 *   TIM1 runs center-aligned at BOARD_PWM_FREQ_HZ. CH4 (PWM2, CCR4 = ARR-1)
 *   produces OC4REF rising just before the counter peak, routed to TRGO2,
 *   which triggers the injected sequences of ADC1 and ADC2 simultaneously.
 *   At the peak all three low-side FETs are on, so shunt current = phase
 *   current. ADC1 JEOS -> ADC1_2_IRQHandler -> app_fast_loop_isr().
 *   CCR preload is transferred at the next update event (the counter
 *   underflow), half a period after the sample (BOARD_PWM_OUTPUT_DELAY_PERIODS).
 *
 * BRING-UP NOTE: this file is written against RM0440 rev 7 but has not yet
 * been validated on hardware. Follow docs/BRINGUP.md step by step with a
 * current-limited supply before connecting a battery.
 */
#include <string.h>

#include "board_config.h"
#include "board_pins.h"
#include "hal.h"
#include "ring_buffer.h"
#include "stm32g4xx.h"

static volatile uint32_t s_millis;
static uint32_t s_pwm_arr;
static hal_reset_cause_t s_reset_cause;
static uint8_t s_uart_rx_mem[512], s_uart_tx_mem[1024];
static ring_buffer_t s_uart_rx, s_uart_tx;

/* ------------------------------------------------------------------ GPIO */

typedef enum { GPIO_IN = 0, GPIO_OUT = 1, GPIO_AF = 2, GPIO_ANALOG = 3 } gpio_mode_t;
typedef enum { PULL_NONE = 0, PULL_UP = 1, PULL_DOWN = 2 } gpio_pull_t;

static void gpio_cfg(GPIO_TypeDef *g, uint32_t pin, gpio_mode_t mode, gpio_pull_t pull, uint32_t af)
{
    g->MODER = (g->MODER & ~(3u << (pin * 2))) | ((uint32_t)mode << (pin * 2));
    g->PUPDR = (g->PUPDR & ~(3u << (pin * 2))) | ((uint32_t)pull << (pin * 2));
    g->OSPEEDR |= (3u << (pin * 2)); /* very high speed: sharp PWM edges */
    g->OTYPER &= ~(1u << pin);
    volatile uint32_t *afr = &g->AFR[pin >> 3];
    uint32_t sh = (pin & 7u) * 4u;
    *afr = (*afr & ~(0xFu << sh)) | ((af & 0xFu) << sh);
}

static inline void gpio_write(GPIO_TypeDef *g, uint32_t pin, bool v)
{
    g->BSRR = v ? (1u << pin) : (1u << (pin + 16u));
}

static inline bool gpio_read(GPIO_TypeDef *g, uint32_t pin) { return (g->IDR & (1u << pin)) != 0u; }

/* ----------------------------------------------------------------- clock */

static void clock_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB1ENR1;

    /* Range 1 boost mode is required above 150 MHz. */
    PWR->CR5 &= ~PWR_CR5_R1MODE;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    FLASH->ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS) {}

    /* 8 MHz / M=2 * N=85 / R=2 = 170 MHz */
    RCC->PLLCFGR = (3u << RCC_PLLCFGR_PLLSRC_Pos) |        /* HSE */
                   ((2u - 1u) << RCC_PLLCFGR_PLLM_Pos) |
                   (85u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLR_Pos) |          /* /2 */
                   RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* Step through AHB/2 for >= 1 us when switching to >80 MHz (RM0440 6.2.7). */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | (8u << RCC_CFGR_HPRE_Pos);
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (3u << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS) != (3u << RCC_CFGR_SWS_Pos)) {}
    for (volatile int i = 0; i < 200; i++) {}
    RCC->CFGR &= ~RCC_CFGR_HPRE;

    SystemCoreClock = SYSCLK_HZ;

    /* Cycle counter for timestamps. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Freeze PWM (outputs to idle) and watchdog while halted in the debugger:
     * a breakpoint must never leave a FET switched on. */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
}

/* ----------------------------------------------------------------- PWM */

static void pwm_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    TIM1->CR1 = 0;
    TIM1->BDTR = 0;
    s_pwm_arr = (uint32_t)((float)SYSCLK_HZ / (2.0f * BOARD_PWM_FREQ_HZ));
    TIM1->PSC = 0;
    TIM1->ARR = s_pwm_arr;
    TIM1->RCR = 0;
    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;              /* center-aligned mode 1 */

    /* CH1..3: PWM mode 1 with preload. CH4: PWM mode 2 -> ADC trigger. */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = s_pwm_arr / 2u;
    TIM1->CCR4 = s_pwm_arr - 1u;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;

    /* TRGO2 = OC4REF */
    TIM1->CR2 = (7u << TIM_CR2_MMS2_Pos);

    /* Dead time: t_DTS = 1/170 MHz = 5.88 ns. DTG[7]=0 range covers up to 747 ns. */
    uint32_t dtg = (uint32_t)(BOARD_DEADTIME_NS * 1e-9f * (float)SYSCLK_HZ + 0.5f);
    if (dtg > 127u) dtg = 127u;
    /* Break: active low (gate driver nFAULT / comparator), filtered, outputs
     * forced to idle (both FETs off) by OSSI/OSSR while MOE = 0. */
    TIM1->BDTR = dtg | TIM_BDTR_OSSR | TIM_BDTR_OSSI | TIM_BDTR_BKE |
                 (3u << TIM_BDTR_BKF_Pos);                   /* BKP = 0: active low */

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0;

    /* PWM pins last, with MOE still 0 -> idle-low. */
    gpio_cfg(GPIOA, 8, GPIO_AF, PULL_DOWN, 6);
    gpio_cfg(GPIOA, 9, GPIO_AF, PULL_DOWN, 6);
    gpio_cfg(GPIOA, 10, GPIO_AF, PULL_DOWN, 6);
    gpio_cfg(GPIOB, 13, GPIO_AF, PULL_DOWN, 6);
    gpio_cfg(GPIOB, 14, GPIO_AF, PULL_DOWN, 6);
    gpio_cfg(GPIOB, 15, GPIO_AF, PULL_DOWN, 4);
    gpio_cfg(GPIOB, 12, GPIO_AF, PULL_UP, 6);   /* BKIN */
}

float hal_pwm_frequency_hz(void) { return BOARD_PWM_FREQ_HZ; }

void hal_pwm_set_duty(float a, float b, float c)
{
    float arr = (float)s_pwm_arr;
    TIM1->CCR1 = (uint32_t)(a * arr);
    TIM1->CCR2 = (uint32_t)(b * arr);
    TIM1->CCR3 = (uint32_t)(c * arr);
}

void hal_pwm_outputs_enable(void)
{
    if (TIM1->SR & TIM_SR_BIF) return; /* break latched: stay off */
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void hal_pwm_outputs_disable(void) { TIM1->BDTR &= ~TIM_BDTR_MOE; }

void hal_pwm_low_sides_on(void)
{
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->EGR = TIM_EGR_UG; /* apply now, not at next update */
    if (!(TIM1->SR & TIM_SR_BIF)) TIM1->BDTR |= TIM_BDTR_MOE;
}

bool hal_pwm_outputs_enabled(void) { return (TIM1->BDTR & TIM_BDTR_MOE) != 0u; }
bool hal_pwm_hw_break_tripped(void) { return (TIM1->SR & TIM_SR_BIF) != 0u; }
void hal_pwm_hw_break_clear(void) { TIM1->SR &= ~TIM_SR_BIF; }

/* ----------------------------------------------------------------- ADC */

static void adc_enable(ADC_TypeDef *adc)
{
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    hal_delay_us(30);                               /* tADCVREG_STUP = 20 us */
    adc->CR &= ~ADC_CR_ADCALDIF;
    adc->CR |= ADC_CR_ADCAL;
    while (adc->CR & ADC_CR_ADCAL) {}
    hal_delay_us(5);
    adc->ISR = ADC_ISR_ADRDY;
    adc->CR |= ADC_CR_ADEN;
    while (!(adc->ISR & ADC_ISR_ADRDY)) {}
}

static void adc_set_smp(ADC_TypeDef *adc, uint32_t ch, uint32_t smp)
{
    if (ch < 10u) adc->SMPR1 = (adc->SMPR1 & ~(7u << (ch * 3u))) | (smp << (ch * 3u));
    else adc->SMPR2 = (adc->SMPR2 & ~(7u << ((ch - 10u) * 3u))) | (smp << ((ch - 10u) * 3u));
}

static void adc_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    (void)RCC->AHB2ENR;
    /* HCLK/4 = 42.5 MHz (max 60 MHz), synchronous to TIM1 -> no trigger jitter. */
    ADC12_COMMON->CCR = (3u << ADC_CCR_CKMODE_Pos) | ADC_CCR_VREFEN;

    gpio_cfg(GPIOA, 0, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOA, 1, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOA, 6, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOC, 0, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOC, 1, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOC, 2, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOC, 3, GPIO_ANALOG, PULL_NONE, 0);
    gpio_cfg(GPIOB, 1, GPIO_ANALOG, PULL_NONE, 0);

    adc_enable(ADC1);
    adc_enable(ADC2);

    /* Sample times: 6.5 cycles (smp=1) for low-impedance CSA outputs,
     * 47.5 cycles (smp=4) for the divided / NTC channels. */
    adc_set_smp(ADC1, ADC_CH_IA, 1);
    adc_set_smp(ADC1, ADC_CH_VBUS, 2);
    adc_set_smp(ADC2, ADC_CH_IB, 1);
    adc_set_smp(ADC2, ADC_CH_IC, 1);
    adc_set_smp(ADC1, ADC_CH_THROTTLE, 4);
    adc_set_smp(ADC1, ADC_CH_TEMP_FET, 4);
    adc_set_smp(ADC1, ADC_CH_TEMP_MOT, 4);
    adc_set_smp(ADC1, ADC_CH_BRAKE, 4);
    adc_set_smp(ADC1, ADC_CH_VREFINT, 6);

    /* Injected: 2 conversions each, JEXTSEL = 8 (TIM1_TRGO2, RM0440 table 163),
     * rising edge. Both ADCs sample their first channel at the same instant. */
    const uint32_t trig = (8u << ADC_JSQR_JEXTSEL_Pos) | (1u << ADC_JSQR_JEXTEN_Pos) | (1u << ADC_JSQR_JL_Pos);
    ADC1->JSQR = trig | (ADC_CH_IA << ADC_JSQR_JSQ1_Pos) | (ADC_CH_VBUS << ADC_JSQR_JSQ2_Pos);
    ADC2->JSQR = trig | (ADC_CH_IB << ADC_JSQR_JSQ1_Pos) | (ADC_CH_IC << ADC_JSQR_JSQ2_Pos);

    ADC1->IER = ADC_IER_JEOSIE;
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
}

void hal_adc_read_fast(hal_adc_fast_t *o)
{
    /* ADC2 finishes within a few ns of ADC1; bounded wait just in case. */
    for (int i = 0; i < 100 && !(ADC2->ISR & ADC_ISR_JEOS); i++) {}
    ADC2->ISR = ADC_ISR_JEOS;
    o->ia = (uint16_t)ADC1->JDR1;
    o->vbus = (uint16_t)ADC1->JDR2;
    o->ib = (uint16_t)ADC2->JDR1;
    o->ic = (uint16_t)ADC2->JDR2;
}

uint16_t hal_adc_read_slow(hal_adc_slow_ch_t ch)
{
    static const uint8_t map[HAL_ADC_SLOW_COUNT] = {
        [HAL_ADC_SLOW_THROTTLE] = ADC_CH_THROTTLE,
        [HAL_ADC_SLOW_BRAKE] = ADC_CH_BRAKE,
        [HAL_ADC_SLOW_TEMP_FET] = ADC_CH_TEMP_FET,
        [HAL_ADC_SLOW_TEMP_MOTOR] = ADC_CH_TEMP_MOT,
        [HAL_ADC_SLOW_VREF] = ADC_CH_VREFINT,
    };
    /* Regular conversion, software triggered. Injected conversions pre-empt it
     * and it resumes automatically. ~2 us worst case. */
    ADC1->SQR1 = ((uint32_t)map[ch] << ADC_SQR1_SQ1_Pos);
    ADC1->ISR = ADC_ISR_EOC;
    ADC1->CR |= ADC_CR_ADSTART;
    for (int i = 0; i < 2000 && !(ADC1->ISR & ADC_ISR_EOC); i++) {}
    return (uint16_t)ADC1->DR;
}

float hal_adc_vref(void) { return BOARD_ADC_VREF; }

void ADC1_2_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_JEOS) {
        ADC1->ISR = ADC_ISR_JEOS;
        app_fast_loop_isr();
    }
}

/* ----------------------------------------------------------------- GPIO misc */

static void gpio_init(void)
{
    gpio_cfg(GPIOB, 6, GPIO_IN, PULL_UP, 0);
    gpio_cfg(GPIOB, 7, GPIO_IN, PULL_UP, 0);
    gpio_cfg(GPIOB, 8, GPIO_IN, PULL_UP, 0);
    gpio_cfg(GPIOB, 9, GPIO_IN, PULL_UP, 0);
    gpio_cfg(GPIOB, 10, GPIO_IN, PULL_UP, 0);
    gpio_write(GPIOB, 5, false);
    gpio_cfg(GPIOB, 5, GPIO_OUT, PULL_NONE, 0);
    gpio_cfg(GPIOC, 6, GPIO_OUT, PULL_NONE, 0);
    gpio_cfg(GPIOC, 7, GPIO_OUT, PULL_NONE, 0);
}

uint8_t hal_hall_read(void)
{
    uint32_t idr = GPIOB->IDR;
    return (uint8_t)((idr >> 6) & 7u); /* PB6=A bit0, PB7=B bit1, PB8=C bit2 */
}
bool hal_brake_switch_active(void) { return !gpio_read(GPIOB, 9); }
bool hal_pas_read(void) { return gpio_read(GPIOB, 10); }
void hal_gate_enable(bool on) { gpio_write(GPIOB, 5, on); }
bool hal_gate_fault_pin(void) { return !gpio_read(GPIOB, 12); }
void hal_led_set(bool on) { gpio_write(GPIOC, 6, on); }
void hal_aux_output_set(bool on) { gpio_write(GPIOC, 7, on); }

/* ----------------------------------------------------------------- SPI3 */

static void spi_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
    (void)RCC->APB1ENR1;
    gpio_cfg(GPIOC, 10, GPIO_AF, PULL_NONE, 6);
    gpio_cfg(GPIOC, 11, GPIO_AF, PULL_UP, 6);
    gpio_cfg(GPIOC, 12, GPIO_AF, PULL_NONE, 6);
    gpio_write(GPIOA, 15, true);
    gpio_cfg(GPIOA, 15, GPIO_OUT, PULL_UP, 0);
    gpio_write(GPIOC, 9, true);
    gpio_cfg(GPIOC, 9, GPIO_OUT, PULL_UP, 0);

    /* Master, mode 1 (CPOL 0, CPHA 1), fPCLK/32 = 5.3 MHz, 16-bit frames. */
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_CPHA | (4u << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI3->CR2 = (15u << SPI_CR2_DS_Pos);
    SPI3->CR1 |= SPI_CR1_SPE;
}

uint16_t hal_spi_transfer16(hal_spi_dev_t dev, uint16_t tx)
{
    /* SPI3 is shared: the encoder is read in the fast ISR, so a gate-driver
     * transaction from a lower priority context must not be interrupted. */
    uint32_t irq = (dev == HAL_SPI_DEV_GATE_DRIVER) ? hal_irq_save() : 0u;
    GPIO_TypeDef *cs_port = (dev == HAL_SPI_DEV_GATE_DRIVER) ? GPIOA : GPIOC;
    uint32_t cs_pin = (dev == HAL_SPI_DEV_GATE_DRIVER) ? 15u : 9u;

    gpio_write(cs_port, cs_pin, false);
    for (volatile int i = 0; i < 10; i++) {} /* CS setup > 50 ns */
    while (!(SPI3->SR & SPI_SR_TXE)) {}
    *(volatile uint16_t *)&SPI3->DR = tx;
    uint32_t guard = 0;
    while (!(SPI3->SR & SPI_SR_RXNE) && ++guard < 10000u) {}
    uint16_t rx = *(volatile uint16_t *)&SPI3->DR;
    while (SPI3->SR & SPI_SR_BSY) {}
    gpio_write(cs_port, cs_pin, true);
    for (volatile int i = 0; i < 40; i++) {} /* CS high time > 400 ns (DRV835x) */

    if (dev == HAL_SPI_DEV_GATE_DRIVER) hal_irq_restore(irq);
    return rx;
}

/* ----------------------------------------------------------------- USART2 */

static void uart_init(uint32_t baud)
{
    rb_init(&s_uart_rx, s_uart_rx_mem, sizeof(s_uart_rx_mem));
    rb_init(&s_uart_tx, s_uart_tx_mem, sizeof(s_uart_tx_mem));
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    (void)RCC->APB1ENR1;
    gpio_cfg(GPIOA, 2, GPIO_AF, PULL_NONE, 7);
    gpio_cfg(GPIOA, 3, GPIO_AF, PULL_UP, 7);
    USART2->BRR = SYSCLK_HZ / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    USART2->CR1 |= USART_CR1_UE;
    NVIC_SetPriority(USART2_IRQn, 6);
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    if (isr & USART_ISR_RXNE_RXFNE) (void)rb_put(&s_uart_rx, (uint8_t)USART2->RDR);
    if ((USART2->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
        uint8_t b;
        if (rb_get(&s_uart_tx, &b)) USART2->TDR = b;
        else USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    }
}

size_t hal_uart_write(const uint8_t *d, size_t n)
{
    size_t i = 0;
    while (i < n && rb_put(&s_uart_tx, d[i])) i++;
    USART2->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return i;
}

bool hal_uart_read_byte(uint8_t *b) { return rb_get(&s_uart_rx, b); }

/* ----------------------------------------------------------------- FDCAN1 */

/* Message RAM layout is fixed on STM32G4 (RM0440 44.3.3), per instance: */
#define CAN_RAM            ((uint32_t *)SRAMCAN_BASE)
#define CAN_RAM_RXF0_OFF   (0x0B0u / 4u)
#define CAN_RAM_TXB_OFF    (0x278u / 4u)
#define CAN_ELEM_WORDS     18u

#define CAN_SEG1_TQ        135u
#define CAN_SEG2_TQ        34u
#define CAN_BIT_TQ         (1u + CAN_SEG1_TQ + CAN_SEG2_TQ)

/* The slower supported rates divide 1 Mbps, so exact timing at the highest
 * rate proves that the prescaler calculation is exact for all four rates. */
_Static_assert(SYSCLK_HZ % (1000000u * CAN_BIT_TQ) == 0u,
               "CAN timing must support an exact 1 Mbps bitrate");

void hal_can_init(uint32_t bitrate)
{
    /* Classic CAN rates that divide 170 MHz / 170 tq exactly; else 500k. */
    if (bitrate != 125000u && bitrate != 250000u && bitrate != 500000u && bitrate != 1000000u)
        bitrate = 500000u;
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL) | (2u << RCC_CCIPR_FDCANSEL_Pos); /* PCLK1 */
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;
    gpio_cfg(GPIOA, 11, GPIO_AF, PULL_UP, 9);
    gpio_cfg(GPIOA, 12, GPIO_AF, PULL_NONE, 9);

    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);   /* classic CAN */
    FDCAN1->CCCR |= FDCAN_CCCR_DAR;                        /* no auto-retransmit: status is periodic */

    /* 170 time quanta: 1 sync + 135 seg1 + 34 seg2 -> 80 % sample point.
     * Prescalers 8/4/2/1 give exact 125k/250k/500k/1M bitrates. */
    uint32_t brp = SYSCLK_HZ / (bitrate * CAN_BIT_TQ);
    FDCAN1->NBTP = ((CAN_SEG2_TQ - 1u) << FDCAN_NBTP_NSJW_Pos) | ((brp - 1u) << FDCAN_NBTP_NBRP_Pos) |
                   ((CAN_SEG1_TQ - 1u) << FDCAN_NBTP_NTSEG1_Pos) | ((CAN_SEG2_TQ - 1u) << FDCAN_NBTP_NTSEG2_Pos);

    memset((void *)SRAMCAN_BASE, 0, 0x350);                 /* clear instance 1 message RAM */
    FDCAN1->RXGFC = 0; /* accept all std + ext into FIFO0; filter in software */
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
}

bool hal_can_send(const hal_can_frame_t *f)
{
    if (FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) return false;
    uint32_t idx = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    volatile uint32_t *e = CAN_RAM + CAN_RAM_TXB_OFF + idx * CAN_ELEM_WORDS;
    e[0] = f->extended ? ((f->id & 0x1FFFFFFFu) | (1u << 30)) : ((f->id & 0x7FFu) << 18);
    e[1] = ((uint32_t)(f->dlc & 0xFu)) << 16;
    uint32_t w0 = 0, w1 = 0;
    memcpy(&w0, &f->data[0], 4);
    memcpy(&w1, &f->data[4], 4);
    e[2] = w0;
    e[3] = w1;
    FDCAN1->TXBAR = 1u << idx;
    return true;
}

bool hal_can_receive(hal_can_frame_t *f)
{
    if ((FDCAN1->RXF0S & FDCAN_RXF0S_F0FL) == 0u) return false;
    uint32_t idx = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    volatile uint32_t *e = CAN_RAM + CAN_RAM_RXF0_OFF + idx * CAN_ELEM_WORDS;
    uint32_t r0 = e[0], r1 = e[1];
    f->extended = (r0 & (1u << 30)) != 0u;
    f->id = f->extended ? (r0 & 0x1FFFFFFFu) : ((r0 >> 18) & 0x7FFu);
    f->dlc = (uint8_t)((r1 >> 16) & 0xFu);
    uint32_t w0 = e[2], w1 = e[3];
    memcpy(&f->data[0], &w0, 4);
    memcpy(&f->data[4], &w1, 4);
    FDCAN1->RXF0A = idx;
    return true;
}

/* ----------------------------------------------------------------- flash */

/* Dual-bank mode (DBANK=1, factory default): 2 KB pages. Config lives in the
 * last two pages of bank 2, so erasing never stalls code running from bank 1. */
#define FLASH_CFG_PAGE_SIZE  2048u
#define FLASH_CFG_BASE       0x0807F000u
#define FLASH_CFG_FIRST_PAGE 126u      /* page index within bank 2 */

uint32_t hal_flash_slot_size(void) { return FLASH_CFG_PAGE_SIZE; }
const void *hal_flash_slot_ptr(int slot) { return (const void *)(FLASH_CFG_BASE + (uint32_t)slot * FLASH_CFG_PAGE_SIZE); }

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
}

static bool flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {}
    bool ok = !(FLASH->SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                             FLASH_SR_SIZERR | FLASH_SR_PGSERR));
    FLASH->SR = FLASH->SR; /* clear flags (write 1 to clear) */
    return ok;
}

bool hal_flash_slot_erase(int slot)
{
    if (slot < 0 || slot >= HAL_FLASH_CONFIG_SLOTS) return false;
    flash_unlock();
    flash_wait();
    /* The erase blocks the main loop (not the ISRs - bank 2 is erased while
     * code runs from bank 1). Start it with a full watchdog window. */
    hal_watchdog_kick();
    FLASH->CR = FLASH_CR_PER | FLASH_CR_BKER |
                ((FLASH_CFG_FIRST_PAGE + (uint32_t)slot) << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    bool ok = flash_wait();
    FLASH->CR = FLASH_CR_LOCK;
    return ok;
}

bool hal_flash_slot_write(int slot, uint32_t offset, const void *data, uint32_t len)
{
    if (slot < 0 || slot >= HAL_FLASH_CONFIG_SLOTS || (offset & 7u) ||
        offset + len > FLASH_CFG_PAGE_SIZE) return false;
    const uint8_t *src = (const uint8_t *)data;
    volatile uint32_t *dst = (volatile uint32_t *)(FLASH_CFG_BASE + (uint32_t)slot * FLASH_CFG_PAGE_SIZE + offset);

    flash_unlock();
    flash_wait();
    hal_watchdog_kick();
    FLASH->CR = FLASH_CR_PG;
    bool ok = true;
    for (uint32_t i = 0; i < len && ok; i += 8u) {
        uint8_t dw[8];
        memset(dw, 0xFF, sizeof(dw));
        memcpy(dw, &src[i], (len - i) >= 8u ? 8u : (len - i));
        uint32_t lo, hi;
        memcpy(&lo, &dw[0], 4);
        memcpy(&hi, &dw[4], 4);
        dst[0] = lo;          /* double-word programming: two consecutive writes */
        __ISB();
        dst[1] = hi;
        ok = flash_wait();
        dst += 2;
    }
    FLASH->CR = FLASH_CR_LOCK;
    return ok;
}

/* ----------------------------------------------------------------- watchdog */

void hal_watchdog_start(uint32_t timeout_ms)
{
    IWDG->KR = 0xCCCCu;                      /* start (also starts LSI) */
    IWDG->KR = 0x5555u;                      /* unlock PR/RLR */
    IWDG->PR = 3u;                           /* /32 -> 1 kHz tick from 32 kHz LSI */
    IWDG->RLR = timeout_ms > 4095u ? 4095u : timeout_ms;
    while (IWDG->SR != 0u) {}
    IWDG->KR = 0xAAAAu;
}

void hal_watchdog_kick(void) { IWDG->KR = 0xAAAAu; }

/* ----------------------------------------------------------------- RNG / UID */

static void rng_init(void)
{
    /* RNG kernel clock = HSI48 (CCIPR.CLK48SEL = 00, reset default). */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {}
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB2ENR;
    RNG->CR = RNG_CR_RNGEN | RNG_CR_CED;  /* clock error detection on */
}

uint32_t hal_random_u32(void)
{
    for (;;) {
        uint32_t sr = RNG->SR;
        if (sr & (RNG_SR_SECS | RNG_SR_SEIS)) {
            /* Seed error: restart the generator (RM0440 26.3.7). */
            RNG->SR &= ~RNG_SR_SEIS;
            RNG->CR &= ~RNG_CR_RNGEN;
            RNG->CR |= RNG_CR_RNGEN;
            continue;
        }
        if (sr & RNG_SR_DRDY) {
            uint32_t v = RNG->DR;
            if (v != 0u) return v; /* 0 can indicate a seed error on some revisions */
        }
    }
}

void hal_device_uid(uint8_t uid[12])
{
    const uint8_t *p = (const uint8_t *)UID_BASE;
    for (int i = 0; i < 12; i++) uid[i] = p[i];
}

/* ----------------------------------------------------------------- system */

void SysTick_Handler(void)
{
    s_millis++;
    app_medium_loop_isr();
}

static void capture_reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    if (csr & RCC_CSR_IWDGRSTF) s_reset_cause = HAL_RESET_WATCHDOG;
    else if (csr & RCC_CSR_SFTRSTF) s_reset_cause = HAL_RESET_SOFTWARE;
    else if (csr & RCC_CSR_BORRSTF) s_reset_cause = HAL_RESET_BROWNOUT;
    else if (csr & RCC_CSR_PINRSTF) s_reset_cause = HAL_RESET_PIN;
    else s_reset_cause = HAL_RESET_OTHER;
    RCC->CSR |= RCC_CSR_RMVF;
}

void hal_init(void)
{
    capture_reset_cause();
    clock_init();
    pwm_init();            /* first: gets PWM pins into a defined OFF state */
    gpio_init();
    adc_init();
    spi_init();
    uart_init(115200u);
    rng_init();
    /* CAN is started by the app via hal_can_init() with the configured bitrate. */
}

void hal_start_scheduler(void)
{
    /* ADC IRQ above everything; SysTick (medium loop) below it. */
    NVIC_SetPriorityGrouping(3); /* 4 bits preemption */
    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    SysTick_Config(SYSCLK_HZ / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 4);

    TIM1->CR1 |= TIM_CR1_CEN; /* counter runs, triggers ADC; MOE still 0 */
}

uint32_t hal_millis(void) { return s_millis; }
uint32_t hal_cycles(void) { return DWT->CYCCNT; }
uint32_t hal_cycles_per_us(void) { return SYSCLK_HZ / 1000000u; }

void hal_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t n = us * (SYSCLK_HZ / 1000000u);
    while ((DWT->CYCCNT - start) < n) {}
}

void hal_system_reset(void)
{
    hal_pwm_outputs_disable();
    NVIC_SystemReset();
}

hal_reset_cause_t hal_reset_cause(void) { return s_reset_cause; }

uint32_t hal_irq_save(void)
{
    uint32_t s = __get_PRIMASK();
    __disable_irq();
    return s;
}

void hal_irq_restore(uint32_t s) { __set_PRIMASK(s); }

/* Any fault handler: kill the bridge first, then let the watchdog reset us. */
void HardFault_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    GPIOB->BSRR = (1u << (5u + 16u)); /* gate enable low */
    for (;;) {}
}
