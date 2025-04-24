#include "arm_math.h"

#include <math.h>
#include <stdio.h>

#define FFT_SIZE 1024

float32_t input[FFT_SIZE * 2]; // Interleaved real and imaginary parts
float32_t output[FFT_SIZE];
arm_cfft_instance_f32 fft_instance;

int main(void) {
    // Initialize FFT instance
    arm_cfft_init_f32(&fft_instance, FFT_SIZE);

    // Prepare a cosine wave as input
    for (int i = 0; i < FFT_SIZE; i++) {
        input[2 * i] = cos(2 * M_PI * i / FFT_SIZE); // Real part
        input[2 * i + 1] = 0;                       // Imaginary part
    }

    // Perform FFT
    arm_cfft_f32(&fft_instance, input, 0, 1);

    // Compute magnitude
    arm_cmplx_mag_f32(input, output, FFT_SIZE);

    // Print results
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        printf("Frequency bin %d: %f\n", i, output[i]);
    }

    while (1);
}
