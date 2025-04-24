#include <stm32l432xx.h>
#include "ee14lib.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>


volatile unsigned int bpm = 120;
volatile unsigned int threshold = 500;
volatile unsigned int beat_flag = 0;
volatile unsigned int update_flag = 0;
volatile unsigned int interrupt_count = 0;

volatile int LCDAddr = 0x27;
volatile int BLEN = 1;

void delay_ms(unsigned int ms)
{
    unsigned int start = interrupt_count;
    while (interrupt_count - start < ms)
        ;
}

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

void write_word(int data){
	int temp = data;
	if ( BLEN == 1 )
		temp |= 0x08;
	else
		temp &= 0xF7;
	i2c_write(I2C3, LCDAddr, &temp, 1);
}

void send_command(int comm){
	int buf;
	// Send bit7-4 firstly
	buf = comm & 0xF0;
	buf |= 0x04;			// RS = 0, RW = 0, EN = 1
	write_word(buf);
	delay_ms(2);
	buf &= 0xFB;			// Make EN = 0
	write_word(buf);

	// Send bit3-0 secondly
	buf = (comm & 0x0F) << 4;
	buf |= 0x04;			// RS = 0, RW = 0, EN = 1
	write_word(buf);
	delay_ms(2);
	buf &= 0xFB;			// Make EN = 0
	write_word(buf);
}

void send_data(int data){
	int buf;
	// Send bit7-4 firstly
	buf = data & 0xF0;
	buf |= 0x05;			// RS = 1, RW = 0, EN = 1
	write_word(buf);
	delay_ms(2);
	buf &= 0xFB;			// Make EN = 0
	write_word(buf);

	// Send bit3-0 secondly
	buf = (data & 0x0F) << 4;
	buf |= 0x05;			// RS = 1, RW = 0, EN = 1
	write_word(buf);
	delay_ms(2);
	buf &= 0xFB;			// Make EN = 0
	write_word(buf);
}

void init(){
	send_command(0x33);	// Must initialize to 8-line mode at first
	delay_ms(5);
	send_command(0x32);	// Then initialize to 4-line mode
	delay_ms(5);
	send_command(0x28);	// 2 Lines & 5*7 dots
	delay_ms(5);
	send_command(0x0C);	// Enable display without cursor
	delay_ms(5);
	send_command(0x01);	// Clear Screen
	// wiringPiI2CWrite(fd, 0x08);
    int i = 0x08;
    i2c_write(I2C3, LCDAddr, &i, 1);

}

void clear(){
	send_command(0x01);	//clear Screen
}

void write(int x, int y, const char* format, ...) {
    char buffer[32]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    int addr;
    if (x < 0)  x = 0;
    if (x > 15) x = 15;
    if (y < 0)  y = 0;
    if (y > 3)  y = 3;

    // Move cursor
    addr = 0x80 + 0x40 * y + x;
    send_command(addr);

    for (int i = 0; buffer[i] != '\0'; i++) {
        send_data(buffer[i]);
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF0)
    {
        increase_bpm();
        update_flag = 1;
    }
    EXTI->PR1 |= EXTI_PR1_PIF0;
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF1)
    {
        decrease_bpm();
        update_flag = 1;
    }
    EXTI->PR1 |= EXTI_PR1_PIF1;
}

int main()
{
    SysTick_initialize();

    delay_ms(40);
    i2c_init(I2C3, D12, A6);
    delay_ms(100);

    // lcd_init();
    // lcd_set_cursor(0, 0);
    // lcd_print("No library here!");
    init();
	write(0, 0, "     Metronome:");
	write(0, 1, "      BPM: %d", bpm);
	// write(0, 2, "20 cols, 4 rows");
	// write(0, 3, "www.sunfounder.com");

    gpio_config_mode(A0, 0b00);
    gpio_config_pullup(A0, 0b01);
    gpio_config_mode(A1, 0b00);
    gpio_config_pullup(A1, 0b01);
    config_gpio_interrupt();

    gpio_config_mode(A2, 0b01);


    

    while (1)
    {
        if (beat_flag)
        {
            beat_flag = 0;
            gpio_write(A2, 1);
            delay_ms(10);
            gpio_write(A2, 0);
        }
        if (update_flag)
        {
            update_flag = 0;
            clear();
            write(0, 0, "     Metronome:");
            write(0, 1, "      BPM: %d", bpm);
        }
    }
}