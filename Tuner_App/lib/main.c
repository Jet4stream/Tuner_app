#define ARMCM4
#include <arm_math.h>
#include <stm32l432xx.h>
#include <stdlib.h>

// void* aligned_malloc(size_t size, size_t alignment);

// int main(void) {
//     while(1) {

//     }
//     return 0;
// }

#define AUDIO_BUFFER_SIZE 256
#define FFT_BUFFER_SIZE 1024
#define SAMPLE_RATE_HZ 1024.0f

float *fft_buffer;
arm_rfft_fast_instance_f32 fftHandler;

float fftBufIn[FFT_BUFFER_SIZE];
float fftBufOut[FFT_BUFFER_SIZE];
uint16_t BUFFER_GETTING_AUDIO = AUDIO_BUFFER_SIZE;
uint16_t INT16_TO_FLOAT = 0;
float inBufPtr[AUDIO_BUFFER_SIZE];
uint8_t fftFlag = 0;
uint8_t processHalfFlag = 1;
uint32_t fftLenReal = 8192;
uint32_t ifftFlagR = 0;
uint32_t bitReverseFlag = 0;

// // // Function called when half of buffer is filled in
// // void Process_HalfBuffer() {

// //     static float leftIn, rightIn;
// //     static float leftProcessed, rightProcessed;
// //     static int16_t leftOut, rightOut;

// //     //Loop through half of double buffer
// //     for (uint8_t n = 0; n < (BUFFER_GETTING_AUDIO / 2) - 1; n += 2) {
// //         // convert to float
// //         leftIn = INT16_TO_FLOAT * ((float) inBufPtr[n]);
// //         rightIn = INT16_To_Float * ((float) inBufPtr[n+1]);

// //         // apply processing to input
// //         leftProcessed = leftIn;
// //         rightProcessed = rightIn;

// //         // convert processed float to 16 bit integers for output buffer
// //         leftOut = (int16_t) (FLOAT_TO_INT16 * outputVolume * leftProcessed);
// //         rightOut = (int16_t) (FLOAT_TO_INT16 * outputVolume * rightProcessed);

// //         // Set output buffer size 
// //         outBufPtr[n] = leftOut;
// //         outBufPtr[n + 1] = rightOut;
// //     }
// // }

// void FFT_ready() {

//     static float leftIn, rightIn;
//     static float leftProcessed, rightProcessed;
//     static int16_t leftOut, rightOut;
//     static int16_t fftIndex = 0;

//     //Loop through half of double buffer
//     for (uint8_t n = 0; n < (BUFFER_GETTING_AUDIO / 2) - 1; n += 2) {
//         // convert to float
//         leftIn = INT16_TO_FLOAT * ((float) inBufPtr[n]);
//         rightIn = INT16_TO_FLOAT * ((float) inBufPtr[n+1]);

//         // Fill FFt buffer
//         fftBufIn[fftIndex] = leftIn;
//         fftIndex++;

//         if (fftIndex == FFT_BUFFER_SIZE) {
//             // perform FFT
//             //arm_rfft_fast_f32(&fftHandler, &fftBufIn, &fftBufOut, 0);

//             // set FFT flag
//             fftFlag = 1;

//             // Reset FFT array index
//             fftIndex = 0;
//         }
//     }   
// }

int main(void) { 
    // initialize FFT
    //arm_rfft_init_q15(&fftHandler, fftLenReal, ifftFlagR, bitReverseFlag);
    arm_rfft_fast_init_32_f32(&fftHandler);
    float peakVal = 0.0f;
    uint16_t peakHz = 0;

    while (1) {

        if (processHalfFlag) {
            //Process_HalfBuffer();
            processHalfFlag = 0;
        }

        if (fftFlag) {
            // compute absolute value of complex FFT results to get peak
            peakVal = 0.0f;
            peakHz = 0.0f;

            uint32_t freqIndex = 0;

            for (uint32_t index = 0; index < FFT_BUFFER_SIZE; index += 2) {
                float curVal = sqrtf(fftBufOut[index] * fftBufOut[index] + (fftBufOut[index + 1]) * fftBufOut[index + 1]);

                if (curVal > peakVal) {
                    peakVal = curVal;
                    peakHz = (uint16_t) (freqIndex * SAMPLE_RATE_HZ / ((float) FFT_BUFFER_SIZE));
                }

                freqIndex++;
            }
        }

    }
}