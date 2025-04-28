#define ARMCM4
#include <arm_math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ee14lib.h"
#include <stm32l432xx.h>
#include <math.h>

#define SAMPLE_RATE 10000  // Sampling rate in Hz
#define NUM_SAMPLES 2048   // Number of samples
#define FREQ 440          // Sine wave frequency in Hz

uint8_t fftFlag = 1;
arm_rfft_instance_q15 fftHandler; // instance of rfft
uint32_t dataLength = 2048; // length of data
uint32_t ifftFlag; // set to 0 to do FFT
uint32_t bitReverse; // set to 0 to not reverse order of bits
uint32_t sampleRate = SAMPLE_RATE;

q15_t sample[NUM_SAMPLES];
q15_t result[NUM_SAMPLES * 2];//[8192];



int main() {
    host_serial_init();
    char buffer[100];
    sprintf(buffer, "Hello World\n");
    serial_write(USART2, buffer, strlen(buffer));
    dataLength = NUM_SAMPLES; // set to 8192 arbitrarily 
    ifftFlag = 0;
    bitReverse = 1;
    arm_rfft_init_q15(&fftHandler, dataLength, ifftFlag, bitReverse);
    float peakVal = 0.0f;
    uint32_t peakHz = 0;

    // signal generator
    float sine_wave[NUM_SAMPLES];
    

    while (1) {
        for (int i = 0; i < NUM_SAMPLES; i++) {
            sine_wave[i] = sinf(2.0f * PI * FREQ * i / SAMPLE_RATE);
        }
    
    
        for (int i = 0; i < NUM_SAMPLES; i++) {
            sample[i] = (q15_t)(sine_wave[i] * 32767);
        }

        if (fftFlag) {
            peakVal = 0.0f;
            peakHz = 0;

            arm_rfft_q15(&fftHandler, sample, result);

            uint32_t freqIndex = 0;

            for (uint32_t index = 0; index < dataLength; index += 2) {
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
        }
    }
}