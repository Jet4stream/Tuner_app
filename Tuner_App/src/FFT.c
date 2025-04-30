#include <stm32l432xx.h>
#include "ee14lib.h"
#include <stdio.h>


#define FFT_SIZE 2048
#define SAMPLE_RATE 10000
#define SAMPLE_PIN A0
volatile u_int32_t adcBuffer[FFT_SIZE];
volatile int sampleIndex = 0;
volatile int bufferReady = 0;


void setup_timer2_10kHz(void)
{
    // Timer2 setup for 16kHz update event
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock

    TIM2->PSC = 0; // Prescaler
    TIM2->ARR = 7999; // 10kHz sample

    TIM2->DIER |= TIM_DIER_UIE;  // Update interrupt enable
    TIM2->CR1 |= TIM_CR1_CEN;    // Enable Timer

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);   // Enable interrupt in NVIC
}


// Timer2 interrupt handler
void TIM2_IRQHandler(void) {
    // Clear interrupt flag
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        // Read ADC sample
        adc_config_single(SAMPLE_PIN);
        u_int32_t sample = adc_read_single();
        adcBuffer[sampleIndex++] = sample;

        // Check if buffer is full
        if (sampleIndex >= FFT_SIZE) {
            sampleIndex = 0;
            bufferReady = 1;
        }
    }
}


int main()
{
    host_serial_init();    // serial communication
    __enable_irq();
    gpio_config_mode(A1, INPUT); // for picking up whether noise is detected

    while (1) {
        int noise_detection = gpio_read(A1);
        // timer for sampling at 10kHz
        while (noise_detection == 0) {
            noise_detection = gpio_read(A1);
        }

        setup_timer2_10kHz();

        // buffer is filled
        if (bufferReady) {
            bufferReady = 0;

            // send samples to run FFT
            for (int i = 0; i < FFT_SIZE; i++) {
                char buffer[32];
                int n = snprintf(buffer, sizeof(buffer), "%d\n", adcBuffer[i]);
                serial_write(USART2, buffer, n);
            }
        }
    }
}


