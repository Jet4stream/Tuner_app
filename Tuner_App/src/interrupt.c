#include "ee14lib.h"

void SysTick_initialize(void)
{
    // Enable the SysTick interrupt
    SysTick->CTRL = 0;
    // Set the reload value to the maximum value of a 24-bit counter
    SysTick->LOAD = 3999;
    // Set the priority of the SysTick interrupt to the highest level
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    // Set the current value of the SysTick counter to 0
    SysTick->VAL = 0;
    // Chooses the processor clock as the clock source
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    // Enable the SysTick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    // Enable the SysTick counter
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void config_gpio_interrupt(void)
{
    // 1. Enable SYSCFG and GPIOA clocks
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // 2. Set PA0 and PA1 as input (default after reset, but can be safe to set)
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);  // 00: Input mode

    // 3. Map EXTI0 and EXTI1 to PA0 and PA1
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0_Msk | SYSCFG_EXTICR1_EXTI1_Msk); // Map to Port A

    // 4. Select falling edge trigger (or rising if needed)
    EXTI->FTSR1 |= (1 << 0) | (1 << 1);  // Falling edge on EXTI0, EXTI1
    // EXTI->RTSR1 |= (1 << 0) | (1 << 1);  // Optional: rising edge too

    // 5. Unmask EXTI lines
    EXTI->IMR1 |= (1 << 0) | (1 << 1);

    // 6. Enable NVIC interrupts for EXTI0 and EXTI1
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);

    NVIC_SetPriority(EXTI1_IRQn, 2);
    NVIC_EnableIRQ(EXTI1_IRQn);
}
