#include <stm32l432xx.h>
#include "ee14lib.h"
#include <stdio.h>
#include <string.h>

volatile unsigned int bpm = 120;
volatile unsigned int threshold = 500;
volatile unsigned int beat_flag = 0;

void increase_bpm(void)
{
    if (bpm < 300)
    {
        bpm++;
        threshold = 60000 / bpm;
    }
}

void decrease_bpm(void)
{
    if (bpm > 20)
    {
        bpm--;
        threshold = 60000 / bpm;
    }
}


// SysTick interrupt handler
void SysTick_Handler(void)
{
    static uint32_t ms_counter = 0;

    ms_counter++;

    if (ms_counter >= threshold)
    {
        ms_counter = 0;
        beat_flag = 1;
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF0)
    {
        increase_bpm();
    }
    EXTI->PR1 |= EXTI_PR1_PIF0;
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF1)
    {
        decrease_bpm();
    }
    EXTI->PR1 |= EXTI_PR1_PIF1;
}

int main()
{
    SysTick_initialize();

    host_serial_init();

    gpio_config_mode(A0, 0b00);
    gpio_config_pullup(A0, 0b01);
    gpio_config_mode(A1, 0b00);
    gpio_config_pullup(A1, 0b01);
    config_gpio_interrupt();

    char buffer[100];

    while (1)
    {
        if (beat_flag)
        {
            beat_flag = 0;
            sprintf(buffer, "BPM: %d\n", bpm);
            serial_write(USART2, buffer, strlen(buffer));
        }
    }
}