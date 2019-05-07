#define F_CPU 8000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>				/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */
#include <string.h>
#include <stdio.h>


#define KEYPAD_R_DDR DDRC
#define KEYPAD_R_PORT PORTC
#define KEYPAD_C_DDR DDRD
#define KEYPAD_C_PORT PORTD
#define KEYPAD_C_PIN PIND

#define RS PORTC4
#define E PORTC5
#define CTRL_DDR DDRC
#define CTRL_PORT PORTC
#define DATA_DDR DDRD
#define DATA_PORT PORTD

//#define LCD_Dir  DDRD			/* Define LCD data port direction */
//#define LCD_Port PORTD			/* Define LCD data port */
//#define RS PORTD2				/* Define Register Select pin */
//#define EN PORTD3 				/* Define Enable signal pin */

int score[16] = { 0 };

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


uint16_t keypad_read()
{
	uint16_t key = 0;
	int i = 0;
	
	for(i = 0; i < 4; i++)
	{
		key = key << 4;					// shift left 4-bit
		
		// scan a row and collect in key 
		KEYPAD_R_DDR |= (1 << i);		// set DDRB0 to output
		KEYPAD_R_PORT &= ~(1 << i);		// set to GND
		
		_delay_ms(1);
		
		key |= (KEYPAD_C_PIN & 0x0F);		// read only 4-bit MSB (clear 4-bit LSB by 0)
		
		_delay_ms(5);
		KEYPAD_R_DDR &= ~(1 << i);		// reset to input (pull up)
		KEYPAD_R_PORT |= (1 << i);
	}
	
	return key;
}

void keypad_init()
{
	DDRC &= ~(1<<PORTC0) & ~(1<<PORTC1) & ~(1<<PORTC2) & ~(1<<PORTC3) ;		// set PORTB as input
	PORTC |= (1<<PORTC0)|(1<<PORTC1)|(1<<PORTC2)|(1<<PORTC3);		// enable pull up
	DDRD &= ~(1<<PORTD0) & ~(1<<PORTD1) & ~(1<<PORTD2) & ~(1<<PORTD3) ;		// set PORTB as input
	PORTD |= (1<<PORTD0)|(1<<PORTD1)|(1<<PORTD2)|(1<<PORTD3);		// enable pull up
}


void counting(uint16_t candidate)
{
	score[candidate-1]++;
}

uint16_t voting_section()
{
	char buff[5];
	
	LCD_Command (0x01);				// Clear display
	LCD_String ("PLEASE VOTE.."); 	//sending string
	LCD_Command(0xC0);				//moving courser to second line
	uint16_t key=0;
	
	do
	{
		key = ~keypad_read();
		key = 1 + log(key)/log(1.99);
		
	} while (!key);					// wait for voting
	
	counting(key);					// calculate score
	return key;
}

void result_section(uint16_t key)
{
	LCD_Command (0x01);				// Clear display
	LCD_String ("THANKS YOU :)");	//sending string
	LCD_Command(0xC0);				//moving courser to second line
	
	int i = 0;
	char buffer[20];
	sprintf(buffer, "No.%d = %d", key, score[key-1]);		// collect keys to buffer
	for (i = 0; i < strlen(buffer); i++)
	{
		LCD_Data(buffer[i]);					// send input character to LCD for displaying
	}
}


int main(void)
{
	unsigned char cardID[8] ;
	uint8_t id=0;
	char buffer[8];
	int i;
	
	
	keypad_init();
	LCD_Init();
	//I2C_Init();
	//spi_init();
	
	while (1)
	{
		_delay_ms(1);
		//id = I2C_ReadwithAck();
		
		
		uint16_t key = voting_section();
		result_section(key);
		
		_delay_ms(200);
	}
}