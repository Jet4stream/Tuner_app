#define ARMCM4
#include <stm32l432xx.h>
#include "ee14lib.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <arm_math.h>
#include <stdlib.h>

#define SAMPLE_RATE 5000 // Sampling rate in Hz
#define NUM_SAMPLES 256  // Number of samples
#define FREQ 440          // Sine wave frequency in Hz
#define SAMPLE_PIN A4

// For ADC Sampling
volatile int adcBuffer[NUM_SAMPLES];
volatile int sampleIndex = 0;
volatile int bufferReady = 0;

// For FFT
uint8_t fftFlag = 0;
arm_rfft_instance_q15 fftHandler;  // instance of rfft
uint32_t dataLength = NUM_SAMPLES; // length of data
uint32_t ifftFlag;                 // set to 0 to do FFT
uint32_t bitReverse;               // set to 0 to not reverse order of bits
uint32_t sampleRate = SAMPLE_RATE;

q15_t sample[NUM_SAMPLES];
q15_t result[NUM_SAMPLES * 2];
char buffer[100];
float sine_wave[NUM_SAMPLES];


// For debouncing
#define DEBOUNCE_DELAY_MS 500
volatile uint32_t last_press_time_0 = 0;
volatile uint32_t last_press_time_1 = 0;

// For note calculations
const char *note_names[] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
volatile unsigned int prev_freq = 0;
    

// For metronome
volatile unsigned int bpm = 120;
volatile unsigned int threshold = 500;
volatile unsigned int metronome_flag = 0;
volatile unsigned int beat_flag = 0;
volatile unsigned int update_flag = 0;
volatile unsigned int interrupt_count = 0;
volatile float frequency = 440.0f;
volatile unsigned int encoder_count = 120;

// For LCD display
volatile int LCDAddr = 0x27;
volatile int BLEN = 1;

void delay_ms(unsigned int ms)
{
    unsigned int start = interrupt_count;
    while (interrupt_count - start < ms)
        ;
}

// LCD Display Logic
void write_word(int data)
{
    int temp = data;
    if (BLEN == 1)
        temp |= 0x08;
    else
        temp &= 0xF7;
    i2c_write(I2C3, LCDAddr, &temp, 1);
}

void send_command(int comm)
{
    int buf;
    // Send bit7-4 firstly
    buf = comm & 0xF0;
    buf |= 0x04; // RS = 0, RW = 0, EN = 1
    write_word(buf);
    delay_ms(2);
    buf &= 0xFB; // Make EN = 0
    write_word(buf);

    // Send bit3-0 secondly
    buf = (comm & 0x0F) << 4;
    buf |= 0x04; // RS = 0, RW = 0, EN = 1
    write_word(buf);
    delay_ms(2);
    buf &= 0xFB; // Make EN = 0
    write_word(buf);
}

void send_data(int data)
{
    int buf;
    // Send bit7-4 firstly
    buf = data & 0xF0;
    buf |= 0x05; // RS = 1, RW = 0, EN = 1
    write_word(buf);
    delay_ms(2);
    buf &= 0xFB; // Make EN = 0
    write_word(buf);

    // Send bit3-0 secondly
    buf = (data & 0x0F) << 4;
    buf |= 0x05; // RS = 1, RW = 0, EN = 1
    write_word(buf);
    delay_ms(2);
    buf &= 0xFB; // Make EN = 0
    write_word(buf);
}

void init()
{
    send_command(0x33); // Must initialize to 8-line mode at first
    delay_ms(5);
    send_command(0x32); // Then initialize to 4-line mode
    delay_ms(5);
    send_command(0x28); // 2 Lines & 5*7 dots
    delay_ms(5);
    send_command(0x0C); // Enable display without cursor
    delay_ms(5);
    send_command(0x01); // Clear Screen
                        // wiringPiI2CWrite(fd, 0x08);
    int i = 0x08;
    i2c_write(I2C3, LCDAddr, &i, 1);
}

void clear()
{
    send_command(0x01); // clear Screen
}

void write(int x, int y, const char *format, ...)
{
    char buffer[32]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    int addr;
    if (x < 0)
        x = 0;
    if (x > 15)
        x = 15;
    if (y < 0)
        y = 0;
    if (y > 3)
        y = 3;

    // Move cursor
    addr = 0x80 + 0x40 * y + x;
    send_command(addr);

    for (int i = 0; buffer[i] != '\0'; i++)
    {
        send_data(buffer[i]);
    }
}

// Metronome Logic
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
    interrupt_count++;

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
        uint32_t now = interrupt_count;
        if ((now - last_press_time_0) >= DEBOUNCE_DELAY_MS)
        {
            last_press_time_0 = now;
            increase_bpm();
            update_flag = 1;
        }

        EXTI->PR1 = EXTI_PR1_PIF0; // Clear pending flag
    }
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF1)
    {
        uint32_t now = interrupt_count;
        if ((now - last_press_time_1) >= DEBOUNCE_DELAY_MS)
        {
            last_press_time_1 = now;
            decrease_bpm();
            update_flag = 1;
        }

        EXTI->PR1 = EXTI_PR1_PIF1; // Clear pending flag
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF3)
    {
        uint32_t now = interrupt_count;
        if ((now - last_press_time_1) >= DEBOUNCE_DELAY_MS)
        {
            last_press_time_1 = now;
            if (metronome_flag == 0)
            {
                metronome_flag = 1;
                
            }
            else
            {
                metronome_flag = 0;
            }
            update_flag = 1;
        }

        EXTI->PR1 = EXTI_PR1_PIF3; // Clear pending flag
    }
}

// Note calculation logic
void display_tuner(float freq)
{
    if (freq <= 0)
        return; // invalid
    
    // if (freq == prev_freq)
    //     return; // no change


    if ((int)freq == 19)
    {
         prev_freq = freq;
         write(0, 1, "--------------------");
         return;
    }

    // Calculate the note number relative to A4 (440 Hz)
    float note_float = 12.0f * log2f(freq / 440.0f) + 69.0f;
    int note_number = (int)(note_float + 0.5f); // nearest note

    int note_index = note_number % 12; // 0 = C, 1 = C#, 2 = D, ..., 11 = B

    const char *note = note_names[note_index];

    // Calculate how many cents off we are (-50 to +50 max)
    float exact_freq_of_note = 440.0f * powf(2.0f, (note_number - 69) / 12.0f);
    float cents = 1200.0f * log2f(freq / exact_freq_of_note);

    // Position on LCD
    const int CENTER_COL = 9; // Centered in 20 cols
    const int MAX_SHIFT = 9;  // Max left/right range

    int shift = (int)(cents / 6.0f); // 6 cents per column shift roughly
    if (shift > MAX_SHIFT)
        shift = MAX_SHIFT;
    if (shift < -MAX_SHIFT)
        shift = -MAX_SHIFT;

    int note_pos = CENTER_COL + shift;
    if (note_pos < 0)
        note_pos = 0;
    if (note_pos > 19)
        note_pos = 19;

    // Build the display string
    char tuner_line[21];
    for (int i = 0; i < 20; i++)
    {
        tuner_line[i] = '-';
    }
    tuner_line[20] = '\0'; // Null terminate

    // Put first character of note
    tuner_line[note_pos] = note[0];
    if (note[1] == '#')
    {
        // If sharp, show #
        if (note_pos + 1 < 20)
        {
            tuner_line[note_pos + 1] = '#';
        }
    }

    // Write to row 2
    write(0, 1, "%s", tuner_line);
    prev_freq = freq; // Update previous frequency
}

void setup_timer2_10kHz(void)
{
    // Timer2 setup for 16kHz update event
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock

    TIM2->PSC = 0;    // Prescaler
    TIM2->ARR = 799; // 10kHz sample

    TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable
    TIM2->CR1 |= TIM_CR1_CEN;   // Enable Timer

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt in NVIC
}

// Timer2 interrupt handler
void TIM2_IRQHandler(void)
{
    // Clear interrupt flag
    if ((TIM2->SR & TIM_SR_UIF))
    {

        // Read ADC sample
        adc_config_single(SAMPLE_PIN);
        int sample = adc_read_single();
        adcBuffer[sampleIndex++] = sample;
        // sprintf(buffer, "ADC Sample: %lu\n", sample);
        // serial_write(USART2, buffer, strlen(buffer));

        // Check if buffer is full
        if (sampleIndex >= NUM_SAMPLES)
        {
            sampleIndex = 0;
            bufferReady = 1;
        }

        TIM2->SR &= ~TIM_SR_UIF;
    }
}

int main()
{
    SysTick_initialize();
    delay_ms(40);
    i2c_init(I2C3, D12, A6);
    delay_ms(100);

    init();
    clear();
    write(0, 0, "        Tuner:");
    write(0, 1, "--------------------");

    gpio_config_mode(A0, 0b00);
    gpio_config_pullup(A0, 0b01);
    gpio_config_mode(A1, 0b00);
    gpio_config_pullup(A1, 0b01);
    gpio_config_mode(A2, 0b00);
    gpio_config_pullup(A2, 0b01);
    config_gpio_interrupt();

    gpio_config_mode(A3, 0b01);
    host_serial_init();
    __enable_irq();
    gpio_config_mode(A4, INPUT);

    dataLength = NUM_SAMPLES;
    ifftFlag = 0;
    bitReverse = 1;
    arm_rfft_init_q15(&fftHandler, dataLength, ifftFlag, bitReverse);

    q15_t magnitudes[NUM_SAMPLES / 2];
    float peakVal = 0.0f;
    uint32_t peakHz = 0;
    setup_timer2_10kHz();

    while (1)
    {
        if (beat_flag)
        {
            beat_flag = 0;
            if (metronome_flag)
            {
                gpio_write(A3, 1);
                delay_ms(10);
                gpio_write(A3, 0);
            }
        }

        if (update_flag)
        {
            update_flag = 0;
            clear();
            if (metronome_flag)
            {
                write(0, 0, "     Metronome:");
                write(0, 1, "      BPM: %d", bpm);
            }
            else
            {
                write(0, 0, "        Tuner:");
                display_tuner(peakHz);
            }
        }

        if (!metronome_flag && bufferReady)
        {
            bufferReady = 0;

            for (int i = 0; i < NUM_SAMPLES; i++)
            {
                sine_wave[i] = (((float)adcBuffer[i]) / 2048.0f) - 1.0f;

                // Apply Hann window
                float hann = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (NUM_SAMPLES - 1)));
                sine_wave[i] *= hann;

                sample[i] = (q15_t)(sine_wave[i] * 32767);
            }

            arm_rfft_q15(&fftHandler, sample, result);
            arm_cmplx_mag_q15(result, magnitudes, NUM_SAMPLES / 2);

            uint32_t peakIndex = 1;
            q15_t peakValue = 0;

            for (uint32_t i = 1; i < NUM_SAMPLES / 2; i++)
            {
                if (magnitudes[i] > peakValue)
                {
                    peakValue = magnitudes[i];
                    peakIndex = i;
                }
            }

            peakHz = (peakIndex * SAMPLE_RATE) / NUM_SAMPLES;
            sprintf(buffer, "peakHz: %lu\n", peakHz);
            serial_write(USART2, buffer, strlen(buffer));
            if (prev_freq != peakHz) {
                prev_freq = peakHz;
                update_flag = 1;
            }
        }
    }
}