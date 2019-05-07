/*
 * rvm4.c
 *
 * Created: 6/5/2562 15:28:59
 * Author : Admin
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <mymfrc522.h>


#define LCD_Dir  DDRD			/* Define LCD data port direction */
#define LCD_Port PORTD			/* Define LCD data port */
#define RS PORTD0				/* Define Register Select pin */
#define EN PORTD1 				/* Define Enable signal pin */

#define SPI_DDR		DDRB
#define SPI_PORT	PORTB
#define SPI_PIN		PINB
#define SPI_MOSI	PORTB3
#define SPI_MISO	PORTB4
#define SPI_SS		PORTB2
#define SPI_SCK		PORTB5


void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble (4 bit high) */
	LCD_Port &= ~ (1<<RS);		/* RS = 0 (command reg.) */
	LCD_Port |= (1<<EN);		/* Enable pulse to send */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(100);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble (4 bit low) */
	LCD_Port |= (1<<EN);		/* Enable pulse to send */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(1);
}


void LCD_Data( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble (4 bit high) */
	LCD_Port |= (1<<RS);		/* RS=1 (data reg.) */
	LCD_Port|= (1<<EN);			/* Enable pulse to send */
	_delay_us(100);
	LCD_Port &= ~ (1<<EN);

	_delay_us(100);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble (4 bit low) */
	LCD_Port |= (1<<EN);		/* Enable pulse to send */
	_delay_us(100);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(1);
}


void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Dir = 0xFF;				/* Make LCD port direction as output */
	_delay_ms(20);				/* LCD Power ON delay always > 15ms */
	
	LCD_Command(0x02);			/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);          /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0F);          /* Display on cursor off*/
	LCD_Command(0x06);          /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);          /* Clear display screen*/
	_delay_ms(1);
}


void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i = 0; str[i] != 0; i++)		/* Send each char of string till the NULL */
	{
		LCD_Data (str[i]);
	}
}

void LCD_Clear()
{
	LCD_Command (0x01);		/* Clear display */
	_delay_ms(1);
	LCD_Command (0x80);		/* Cursor at home position */
}


int main(void)
{
	uint8_t byte;
	uint8_t str[MAX_LEN];
	char buffer[5];
	_delay_ms(50);
	
    spi_init();
    mfrc522_init();
	LCD_Init();
	
    while (1) 
    {
		byte = mfrc522_read(VersionReg);
		if(byte == 0x92)
		{
			LCD_String("MIFARE RC522v2");
			LCD_String("Detected");
		}else if(byte == 0x91 || byte==0x90)
		{
			LCD_String("MIFARE RC522v1");
			LCD_String("Detected");
		}else
		{
			LCD_String("No reader found");
		}
		_delay_ms(100);
		
		byte = mfrc522_read(ComIEnReg);
		mfrc522_write(ComIEnReg,byte|0x20);
		byte = mfrc522_read(DivIEnReg);
		mfrc522_write(DivIEnReg,byte|0x80);
		
		_delay_ms(1500);
		LCD_Clear();
		
		while(1){
			byte = mfrc522_request(PICC_REQALL,str);
			sprintf(buffer, "%u", byte);
			LCD_String(buffer);
			_delay_ms(1000);
		}

    }
}

