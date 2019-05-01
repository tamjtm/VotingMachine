/** Voting Machine LCD & Keypad */
	
#define F_CPU 8000000UL			/* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h>				/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */
#include <string.h>
#include <stdio.h>

#define KEYPAD_DDR DDRB
#define KEYPAD_PORT PORTB
#define KEYPAD_PIN PINB

#define LCD_Dir  DDRD			/* Define LCD data port direction */
#define LCD_Port PORTD			/* Define LCD data port */
#define RS PORTD2				/* Define Register Select pin */
#define EN PORTD3 				/* Define Enable signal pin */

int score[16] = { 0 };

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


uint16_t keypad_read()
{
	uint16_t key = 0;
	int i = 0;
	
	for(i = 0; i < 4; i++)
	{
		key = key << 4;					// shift left 4-bit
		
		_delay_ms(1);
		/* scan a row and collect in key */
		KEYPAD_DDR |= (1 << i);			// set DDRB0 to output
		KEYPAD_PORT &= ~(1 << i);		// set to GND
		_delay_ms(1);
		
		key |= (KEYPAD_PIN & 0xF0) >> 4;		// read only 4-bit MSB (clear 4-bit LSB by 0)
		
		KEYPAD_DDR &= ~(1 << i);		// reset to input (pull up)
		KEYPAD_PORT |= (1 << i);
	}
	
	return key;
}

void keypad_init()
{
	KEYPAD_DDR = 0x00;		// set PORTB as input
	KEYPAD_PORT = 0xFF;		// enable pull up
}

void counting(uint16_t candidate)
{
	score[candidate-1]++;
}

uint16_t voting_section()
{
	LCD_Command (0x01);				// Clear display
	LCD_String ("PLEASE VOTE.."); 	//sending string
	LCD_Command(0xC0);				//moving courser to second line
	uint16_t key;
	
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

void I2C_Init()
{
	TWSR=0x00; //set presca1er bits to zero
	TWBR=0x32; //SCL frequency is 100K for 8Mhz
	TWCR=0x04; //enab1e TWI module
}

void I2C_Start()
{
	TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));
	while (!(TWCR & (1<<TWINT)));
}

void I2C_Stop(void)
{
	TWCR = ((1<< TWINT) | (1<<TWEN) | (1<<TWSTO));
	_delay_us(100) ; //wait for a short time
}

void I2C_Write(uint8_t v_i2cData_u8)
{
	TWDR = v_i2cData_u8 ;
	TWCR = ((1<< TWINT) | (1<<TWEN));
	while (!(TWCR & (1 <<TWINT)));
}

uint8_t I2C_ReadwithAck()
{
	TWCR = ((1<< TWINT) | (1<<TWEN) | (1<<TWEA));
	while ( !(TWCR & (1 <<TWINT)));
	return TWDR;
}

unsigned char RFID_Read()
{
	uint8_t id;
	unsigned char cardID[8];
	int i;
	char buffer[8];
	
	I2C_Start();
	for(i=0;i<8;i++)
	{
		id = I2C_ReadwithAck();
		cardID[i] = id;
		sprintf(buffer, "%u", id);
		for (i = 0; i < strlen(buffer); i++)
		{
			LCD_Data(buffer[i]);					// send input character to LCD for displaying
		}
	}
	I2C_Stop();
	
	return *cardID;
}

int main(void)
{
	unsigned char cardID[8] ;
	
	LCD_Init();
	keypad_init();
	I2C_Init();
		
	while (1)
	{
		LCD_Command (0x01);				// Clear display
		LCD_String ("RFID NUMBER..");	//sending string
		LCD_Command(0xC0);				//moving courser to second line
		
		*cardID = RFID_Read();
		
		uint16_t key = voting_section();
		result_section(key);
		
			
		_delay_ms(200);
	}
}

