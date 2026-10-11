/*
 * Minimal C startup for STM32G474: vector table, .data/.bss init, FPU enable.
 * Unused IRQs go to Default_Handler, which disables the bridge and waits for
 * the watchdog.
 */
#include <stdint.h>

#include "stm32g4xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void ADC1_2_IRQHandler(void);
void USART2_IRQHandler(void);

void NMI_Handler(void) __attribute__((weak, alias("HardFault_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("HardFault_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("HardFault_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("HardFault_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));

#define NUM_IRQS 102

/* Range-fill then override specific slots (GCC extension). */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((section(".isr_vector"), used))
void (*const g_vectors[16 + NUM_IRQS])(void) = {
    [0] = (void (*)(void))(&_estack),
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 ... 16 + NUM_IRQS - 1] = Default_Handler,
    [16 + ADC1_2_IRQn] = ADC1_2_IRQHandler,
    [16 + USART2_IRQn] = USART2_IRQHandler,
};
#pragma GCC diagnostic pop

void Default_Handler(void) { HardFault_Handler(); }

void Reset_Handler(void)
{
    /* FPU full access before any float code runs. */
    SCB->CPACR |= (0xFu << 20);
    __DSB();
    __ISB();

    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;) *dst++ = *src++;
    for (uint32_t *dst = &_sbss; dst < &_ebss;) *dst++ = 0u;

    SCB->VTOR = (uint32_t)g_vectors;
    main();
    for (;;) {}
}

/* Required by CMSIS system header. */
uint32_t SystemCoreClock = 16000000u;
