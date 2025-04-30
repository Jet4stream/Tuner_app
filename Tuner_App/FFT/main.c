#define ARMCM4
#include <arm_math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ee14lib.h"
#include <stm32l432xx.h>
#include <math.h>

#define SAMPLE_RATE 1000  // Sampling rate in Hz
#define NUM_SAMPLES 256   // Number of samples
#define FREQ 440          // Sine wave frequency in Hz
#define SAMPLE_PIN A0

volatile u_int32_t adcBuffer[NUM_SAMPLES];
volatile int sampleIndex = 0;
volatile int bufferReady = 0;

uint8_t fftFlag = 0;
arm_rfft_instance_q15 fftHandler; // instance of rfft
uint32_t dataLength = NUM_SAMPLES; // length of data
uint32_t ifftFlag; // set to 0 to do FFT
uint32_t bitReverse; // set to 0 to not reverse order of bits
uint32_t sampleRate = SAMPLE_RATE;

q15_t sample[NUM_SAMPLES];
q15_t result[NUM_SAMPLES * 2];
char buffer[100];
float sine_wave[NUM_SAMPLES];


void setup_timer2_10kHz(void)
{
    // Timer2 setup for 16kHz update event
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock

    TIM2->PSC = 0; // Prescaler
    TIM2->ARR = 79999; // 10kHz sample

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
        // sprintf(buffer, "ADC Sample: %lu\n", sample);
        // serial_write(USART2, buffer, strlen(buffer));

        // Check if buffer is full
        if (sampleIndex >= NUM_SAMPLES) {
            sampleIndex = 0;
            bufferReady = 1;
        }
    }
}

int main() {
    host_serial_init();
    __enable_irq();
    gpio_config_mode(A1, INPUT); // for picking up whether noise is detected
    dataLength = NUM_SAMPLES;
    ifftFlag = 0;
    bitReverse = 1;
    arm_rfft_init_q15(&fftHandler, dataLength, ifftFlag, bitReverse);
    float peakVal = 0.0f;
    uint32_t peakHz = 0;
    


    while (1) {
        // // sine wave generator
        // for (int i = 0; i < NUM_SAMPLES; i++) {
        //     sine_wave[i] = sinf(2.0f * PI * FREQ * i / SAMPLE_RATE);
        // }
        // int noise_detection = gpio_read(A1);
        // timer for sampling at 10kHz
        // while (noise_detection == 0) {
        //     noise_detection = gpio_read(A1);
        // }

        setup_timer2_10kHz();

        // buffer is filled
        if (bufferReady) {
            bufferReady = 0;

            // send samples to run FFT
            for (int i = 0; i < NUM_SAMPLES; i++) {
                // char buffer[32];
                // serial_write(USART2, buffer, n);
                // sprintf(buffer, "adcBuffer[%d]: %lu    ", i, adcBuffer[i]);
                // serial_write(USART2, buffer, strlen(buffer));
                // float adcBufferNum = adcBuffer[i];
                // int test = adcBufferNum * 3;
                // sprintf(buffer, "adcBufferNum: %lu  ", test);
                // serial_write(USART2, buffer, strlen(buffer));
                // float Result = adcBufferNum / 4095.0f;
                // sprintf(buffer, "adcBufferNum: %f\n", Result);
                // serial_write(USART2, buffer, strlen(buffer));
                sine_wave[i] = (((float) adcBuffer[i]) - 2047.0f) / 2048.0f;
                
                if (sine_wave[i] < -1.0f || sine_wave[i] > 1.0f) {
                    sprintf(buffer, "Invalid sine_wave[%d]: %.4f\n", i, sine_wave[i]);
                    serial_write(USART2, buffer, strlen(buffer));
                }
                sprintf(buffer, "adcBuffer[%d]: %lu, sine_wave[%d]: %.4f\n", i, adcBuffer[i], i, sine_wave[i]);
                serial_write(USART2, buffer, strlen(buffer));
                sample[i] = (q15_t)(sine_wave[i] * 32767);
            }

        // for (int i = 0; i < NUM_SAMPLES; i++) {
        //     sine_wave[i] = sinf(2.0f * PI * FREQ * i / SAMPLE_RATE);
        // }

        //     // input sine wave into the sample 
        // for (int i = 0; i < NUM_SAMPLES; i++) {
        //     // sine_wave[i] = sine_wave[i] / 4095.0f;
        //     sample[i] = (q15_t)(sine_wave[i] * 32767);
        // }

            fftFlag = 1; 

            if (fftFlag) {
                peakVal = 0.0f;
                peakHz = 1;
    
                arm_rfft_q15(&fftHandler, sample, result);
    
                uint32_t freqIndex = 0;
    
                for (uint32_t index = 2; index < dataLength; index += 2) {
                    float real = result[index] / 32768.0f;
                    float imag = result[index + 1] / 32768.0f;
                    float curVal = sqrtf(real * real + imag * imag);
    
                    if (curVal > peakVal) {
                        peakVal = curVal;
                        freqIndex = index / 2;
                        peakHz = (uint32_t) (freqIndex * sampleRate / ((float) dataLength));
                    }
    
                    freqIndex++;
                }
                sprintf(buffer, "peakHz: %ld\n", peakHz);
                serial_write(USART2, buffer, strlen(buffer));
    
                fftFlag = 0;
            }
        }

        // // input sine wave into the sample 
        // for (int i = 0; i < NUM_SAMPLES; i++) {
        //     sine_wave[i] = sine_wave[i] / 4096.0f;
        //     sample[i] = (q15_t)(sine_wave[i] * 32767);
        // }

        // Flag only set when the sample is new data
        
    }
}

// int main() {
//         host_serial_init();
//         __enable_irq();
//         gpio_config_mode(A1, INPUT); // for picking up whether noise is detected
//         dataLength = NUM_SAMPLES;
//         ifftFlag = 0;
//         bitReverse = 1;
//         arm_rfft_init_q15(&fftHandler, dataLength, ifftFlag, bitReverse);
//         float peakVal = 0.0f;
//         uint32_t peakHz = 0;
        
//         float sine_wave1[NUM_SAMPLES];
//         float sine_wave2[NUM_SAMPLES];
//         float sine_wave3[NUM_SAMPLES];
    
//         while (1) {
//             // sine wave generator
//             for (int i = 0; i < NUM_SAMPLES; i++) {
//                 sine_wave1[i] = sinf(2.0f * PI * FREQ * i / SAMPLE_RATE);
//             }
    
//             for (int i = 0; i < NUM_SAMPLES; i++) {
//                 sine_wave2[i] = sinf(2.0f * PI * FREQ * 10 * i / SAMPLE_RATE);
//             }

//             for (int i = 0; i < NUM_SAMPLES; i++) {
//                 sine_wave3[i] = sinf(2.0f * PI * FREQ * 3 * i / SAMPLE_RATE);
//             }
    
//             for (int i = 0; i < NUM_SAMPLES; i++) {
//                 sine_wave[i] = (sine_wave1[i] * 3 + sine_wave2[i] * 2 + sine_wave3[i]) / 6;
//             }

//                 // input sine wave into the sample 
//             for (int i = 0; i < NUM_SAMPLES; i++) {
//                 // sine_wave[i] = sine_wave[i] / 4095.0f;
//                 sample[i] = (q15_t)(sine_wave[i] * 32767);
//             }
    
//                 fftFlag = 1; 
    
//                 if (fftFlag) {
//                     peakVal = 0.0f;
//                     peakHz = 0;
        
//                     arm_rfft_q15(&fftHandler, sample, result);
        
//                     uint32_t freqIndex = 0;
        
//                     for (uint32_t index = 0; index < dataLength; index += 2) {
//                         float real = result[index] / 32768.0f;
//                         float imag = result[index + 1] / 32768.0f;
//                         float curVal = sqrtf(real * real + imag * imag);
        
//                         if (curVal > peakVal) {
//                             peakVal = curVal;
//                             freqIndex = index / 2;
//                             peakHz = (uint32_t) (freqIndex * sampleRate / ((float) dataLength));
//                         }
        
//                         freqIndex++;
//                     }
//                     sprintf(buffer, "peakHz: %ld\n", peakHz);
//                     serial_write(USART2, buffer, strlen(buffer));
        
//                     fftFlag = 0;
//                 }
            
    
//             // // input sine wave into the sample 
//             // for (int i = 0; i < NUM_SAMPLES; i++) {
//             //     sine_wave[i] = sine_wave[i] / 4096.0f;
//             //     sample[i] = (q15_t)(sine_wave[i] * 32767);
//             // }
    
//             // Flag only set when the sample is new data
            
//         }
//     }