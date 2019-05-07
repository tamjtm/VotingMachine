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
#include "mfrc522.h"
#include "spi.h"
#include "lcd.h"


#define RS PORTC4
#define E PORTC5
#define CTRL_DDR DDRC
#define CTRL_PORT PORTC
#define DATA_DDR DDRD
#define DATA_PORT PORTD

void LCD_Command( unsigned char cmnd )
{
	DATA_PORT = (DATA_PORT & 0x0F) | (cmnd & 0xF0); /* sending upper nibble (4 bit high) */
	CTRL_PORT &= ~ (1<<RS);		/* RS = 0 (command reg.) */
	CTRL_PORT |= (1<<E);		/* Enable pulse to send */
	_delay_us(1);
	CTRL_PORT &= ~ (1<<E);

	_delay_us(100);

	DATA_PORT = (DATA_PORT & 0x0F) | (cmnd << 4);  /* sending lower nibble (4 bit low) */
	CTRL_PORT |= (1<<E);		/* Enable pulse to send */
	_delay_us(1);
	CTRL_PORT &= ~ (1<<E);
	_delay_ms(1);
}


void LCD_Data( unsigned char data )
{
	DATA_PORT = (DATA_PORT & 0x0F) | (data & 0xF0); /* sending upper nibble (4 bit high) */
	CTRL_PORT |= (1<<RS);		/* RS=1 (data reg.) */
	CTRL_PORT|= (1<<E);			/* Enable pulse to send */
	_delay_us(100);
	CTRL_PORT &= ~ (1<<E);

	_delay_us(100);

	DATA_PORT = (DATA_PORT & 0x0F) | (data << 4); /* sending lower nibble (4 bit low) */
	CTRL_PORT |= (1<<E);		/* Enable pulse to send */
	_delay_us(100);
	CTRL_PORT &= ~ (1<<E);
	_delay_ms(1);
}


void LCD_Init (void)			/* LCD Initialize function */
{
	CTRL_DDR |= (1<<RS) | (1<<E);
	CTRL_PORT &= ~(1<<RS) & ~(1<<E);
	DDRD |= (1<<PORTD7)|(1<<PORTD6)|(1<<PORTD5)|(1<<PORTD4);
	DATA_PORT = (DATA_PORT & 0x0F);
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

void LCD_Cursor(unsigned char x, unsigned char y)
{
	unsigned char firstcharadd[]={0x80, 0xC0};
	LCD_Command(firstcharadd[y] + x);
}

int main(void)
{
	char buffer[5];
	uint8_t byte;
	uint8_t str[MAX_LEN];
	_delay_ms(50);
	LCD_Init();
	LCD_String("RFID Reader");
	//LCDWriteStringXY(5,1,VERSION_STR);
	
	
	spi_init();
	_delay_ms(1000);
	LCD_Clear();
	
	//init reader
	mfrc522_init();
	
	//check version of the reader
	byte = mfrc522_read(VersionReg);
	if(byte == 0x92)
	{
		LCD_String("MIFARE RC522v2");
		LCD_Command(0xC0);
		LCD_String("Detected");
	}else if(byte == 0x91 || byte==0x90)
	{
		LCD_String("MIFARE RC522v1");
		LCD_Command(0xC0);
		LCD_String("Detected");
	}else
	{
		LCD_String("No reader found");
	}
	
	byte = mfrc522_read(ComIEnReg);
	mfrc522_write(ComIEnReg,byte|0x20);
	byte = mfrc522_read(DivIEnReg);
	mfrc522_write(DivIEnReg,byte|0x80);
	
	_delay_ms(1500);
	LCD_Clear();
	
	while(1){
		byte = mfrc522_request(PICC_REQALL,str);
		//sprintf(buffer,"%u",byte);
		//LCDHexDumpXY(0,0,byte);
		
		if(byte == CARD_FOUND)
		{
			LCD_String("q");
			byte = mfrc522_get_card_serial(str);
			if(byte == CARD_FOUND)
			{
				for(byte=0;byte<8;byte++)
				{
					sprintf(buffer,"%u",str[byte]);
					LCD_String(buffer);					
				}
				_delay_ms(2500);
			}
			else
			{
				LCDWriteStringXY(0,1,"Error");
			}
		}
		LCD_String("o");
		_delay_ms(1000);
	}
	
	
}

