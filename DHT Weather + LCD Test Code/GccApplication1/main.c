/*
 * Example test bench for DHT22 on an Atmega 1284 
 *
 * Author      : David Feng
 * Usage       : Compiled on AVR-GCC using Atmel Studio 7 
 */

#ifndef F_CPU
#define F_CPU 8000000UL //define microcontroller clock speed
#endif

#include <util/delay.h>
#include "lcd_io.c" //library that controls LCD display
#include "DHT.c" //library that works with DHT sensor

unsigned char receivedData;

uint16_t temperature_int = 0;
uint16_t humidity_int = 0;

//function to convert number values into characters
void LCD_print(uint16_t temp, unsigned char cur_count) {
	
	unsigned char ten_count = 0;
	unsigned char hun_count = 0;

	LCD_Cursor(cur_count);
	while( temp >= 100 ) {
		hun_count++;
		temp -= 100;
	}	
	
	LCD_WriteData(hun_count + '0');
	cur_count++;

	LCD_Cursor(cur_count);
	while( temp >= 10 ) {
		ten_count++;
		temp -= 10;
	}
	
	LCD_WriteData(ten_count + '0');
	cur_count++;
	
	LCD_Cursor(cur_count);
	LCD_WriteData('.');
	cur_count++;	
	
	LCD_Cursor(cur_count);
	LCD_WriteData((unsigned char) temp + '0');
	
	LCD_Cursor(0);
}

void SPI_ServantInit(void) {
	// set DDRB to have MISO line as output and MOSI, SCK, and SS as input
	DDRB = 0x4F; PORTB = 0xB0;
	// set SPCR register to enable SPI and enable SPI interrupt (pg. 168)
	// don't forget to add SPIE (interrupt)
	SPCR |= (1<<SPE) |  (1<<SPIE);
	// make sure global interrupts are enabled on SREG register (pg. 9)
	SREG |= 0x80;
}

ISR(SPI_STC_vect) { // this is enabled in with the SPCR register’s “SPI
	// this is enabled in with the SPCR register’s “SPI Interrupt Enable”
	while(!(SPSR & (1<<SPIF)))
	;
	// SPDR contains the received data, e.g. unsigned char receivedData = SPDR;
	receivedData = SPDR;
}


int main(void) {
	
	//Initializing LCD Display to output data from DHT sensor
	
	DDRC = 0xFF; PORTC = 0x00;   // set PORTC to output
	DDRD = 0xFF; PORTD = 0x00;   // set PORTD to output
	DDRA = 0xFF; PORTA = 0x00;

	unsigned char* template = "Temp:   . C     Rh:   . %";
	unsigned char* error = "Error";
	
	LCD_init();
	LCD_ClearScreen ();
	LCD_DisplayString (1, template);

	SPI_ServantInit();
	
	_delay_ms(1000);
		
    while (1) {
		//call DHT sensor function defined in DHT.c
		PORTA = receivedData;
		if (dht_GetTempUtil(&temperature_int, &humidity_int) != -1) {
			
			LCD_print(temperature_int, 7);
			LCD_print(humidity_int, 21);
		}
		
		else {LCD_DisplayString (1, error);}
		
		_delay_ms(1500);
    }
}