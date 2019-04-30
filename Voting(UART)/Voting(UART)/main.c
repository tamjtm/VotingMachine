/*
 * Voting(UART).c
 *
 * Created: 27/4/2562 21:25:13
 * Author : Admin
 */ 

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


void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Port &= ~ (1<<RS);		/* RS=0, command reg. */
	LCD_Port |= (1<<EN);		/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(100);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(1);
}


void LCD_Data( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Port |= (1<<RS);		/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(100);
	LCD_Port &= ~ (1<<EN);

	_delay_us(100);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(100);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(1);
}


void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Dir = 0xFF;				/* Make LCD port direction as o/p */
	_delay_ms(20);				/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x02);			/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);          /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0c);          /* Display on cursor off*/
	LCD_Command(0x06);          /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);          /* Clear display screen*/
	_delay_ms(1);
}


void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
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

/* 4x4 MATRIX KEYPAD initialize */
void keypad_Init()
{
	KEYPAD_DDR = 0x00;
	KEYPAD_PORT = 0xFF;
}

uint16_t readKeypad()
{
	uint16_t key = 0;
	int vote = 0;
	int i;
	//char buffer[5];
	
	//while(vote == 0);
	//{
	for(i=0;i<4;i++)
	{
		key = key<<4;
		_delay_ms(200);
			
		// scan 1st row: set PB0 as ground
		KEYPAD_DDR |= (1<<i);
		KEYPAD_PORT &= ~(1<<i);
			
		//read data from PIN4:7 (so AND for delete 4LSB and shift right 4 positions)
		key |= (KEYPAD_PIN & 0xF0) >> 4;
			
		KEYPAD_DDR &= ~(1<<i);
		KEYPAD_PORT |= (1<<i);
		
		if(vote!=key)
		{
			vote = 1;
		}
		else
		{
			vote = 0;
		}
	}
		/*
		if(key!=16)
		{
			vote = 1;
		}
	}*/
				
	return key;
}
/*
void USART_Init( unsigned int ubrr)
{
	UCSR0B |=(1<<RXEN0)|(1<<RXCIE0); 	// enabling data receive complete interrupt,enabling data receive pin
	UCSR0C |=|(1<<UCSZ00)|(1<<UCSZ01); 	//changing other bits by first setting URSEL, setting for 8 bit communication
	UBRR0=ubrr;			//setting the baud rate
}

unsigned char USART_Receive( void )
{
	 Wait for data to be received */
	//while ( !(UCSR0A & (1<<RXC0)) )	;
	/* Get and return received data from buffer */
	//return UDRn;
//}

/*
char *RFID_Read()
{
	char mem[4];
	char showID[4];
	int16_t id;		//allotting memory for storing ID 
	while(!(UCSRA&(1<<RXC)));	//wait till first eight bit data is received
	id=UDR;
	mem[0] = id;
	sprintf(showID, "%u", id);
	LCD_String(showID);
	while(!(UCSRA&(1<<RXC)));	//wait till first eight bit data is received
	id=UDR;
	mem[1] = id;
	sprintf(showID, "%u", id);
	LCD_String(showID);
	while(!(UCSRA&(1<<RXC)));	//wait till first eight bit data is received
	id=UDR;
	mem[2] = id;
	sprintf(showID, "%u", id);
	LCD_String(showID);
	while(!(UCSRA&(1<<RXC)));	//wait till first eight bit data is received
	id=UDR;
	mem[3] = id;
	sprintf(showID, "%u", id);
	LCD_String(showID);

	return showID;
}
*/
void Initialize()
{
	//DDRC = 0b00000011;
	
	//keypad_Init();
	//USART_Init(6);
	LCD_Init();
}

int main(void)
{
	int vote[16];
	//char showID[4];
	int i=0;
	uint16_t key;
	
	for(i=0;i<16;i++)
	{
		vote[i] = 0;
	}
		
	Initialize();
	
	LCD_String ("RFID NUMBER"); 	//sending string
	//LCD_Command(0x80 + 0x40 + 0);	//moving courser to second line
	
	while (1) 
    {
		
		//showID = RC522_Read();
		PORTC |= (1<<PINC0);
		LCD_String ("II");
		_delay_ms(50);
		
		LCD_Command(0x80 + 0);
		LCD_String("PLEASE VOTE");
		
		key = ~readKeypad();
		PORTC |= (1<<PINC1);
		key = 1 + log(key)/log(1.99);
						
		for(i=1;i<=16;i++)
		{
			if(key == i)
			{
				vote[i-1]++;
				break;
			}
		}
		
		LCD_Command(0x80 + 0);
		LCD_String("THANK YOU");
		_delay_ms(20);
		
		LCD_Clear();
		char buffer[5];
		char buffer2[5];
		for(i=0;i<16;i=i+4)
		{
			LCD_Command(0x80 + 0);
			sprintf(buffer, "%u", i+1);
			LCD_String(buffer);
			LCD_String("=");
			LCD_Command(0x80 + 3);
			sprintf(buffer2, "%u", vote[i]);
			LCD_String(buffer2);
			_delay_ms(10);
			LCD_Command(0x80 + 9);
			sprintf(buffer, "%u", i+2);
			LCD_String(buffer);
			LCD_String("=");
			LCD_Command(0x80 + 11);
			sprintf(buffer2, "%u", vote[i+1]);
			LCD_String(buffer2);
			_delay_ms(10);
			LCD_Command(0x80 + 0x40 + 0);
			sprintf(buffer, "%u", i+3);
			LCD_String(buffer);
			LCD_String("=");
			LCD_Command(0x80 + 0x40 + 3);
			sprintf(buffer2, "%u", vote[i+2]);
			LCD_String(buffer2);
			_delay_ms(10);
			LCD_Command(0x80 + 0x40 + 9);
			sprintf(buffer, "%u", i+4);
			LCD_String(buffer);
			LCD_String("=");
			LCD_Command(0x80 + 0x40 + 11);
			sprintf(buffer2, "%u", vote[i+3]);
			LCD_String(buffer2);
			_delay_ms(400);
		}
		
		PORTC &= ~(1<<PINC0) & ~(1<<PINC1);
		_delay_ms(250);
		
		LCD_Clear();
		LCD_String ("RFID NUMBER"); 	
		LCD_Command(0x80 + 0x40 + 0);
		
    }
}


