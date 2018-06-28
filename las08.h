/*
 * lsa08.h
 *
 *  Created on: Dec 10, 2017
 *      Author: dkrocks
 */
 
#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define baudrate 19200
#define BRX ((F_CPU/(16UL*baudrate))-1)	//baudrate prescaler calculation

volatile uint8_t data;/*data to be sent or recieved*/

void uart_init()
{
	//LSA1
	DDRE|=(1<<2);//~UEN
	DDRE&=~(1<<0);	//RXD0
	DDRE|=(1<<1);	//TXD0
	DDRE&=~(1<<4);	//INT4
	EICRB|=3;//PE4 junction pulse RAISING EDGE
	EIMSK|=(1<<4);
	UBRR0L=BRX;//bps DIVISOR VALUE FOR 14.74MHz
	UBRR0H=BRX>>8;
    UCSR0B|=(1<<RXEN0)|(1<<TXEN0);//TRANSFER AND RECIEVE ENABLE
	UCSR0C|=(1<<UCSZ00)|(1<<UCSZ01);//8 BIT MOD
	//LSA2
	DDRD|=(1<<4);//~UEN
	DDRD&=~(1<<2);	//RXD0
	DDRD|=(1<<3);	//TXD0
	DDRE&=~(1<<5);	//INT5
	EICRB|=3<<2;//PE4 junction pulse RAISING EDGE
	EIMSK|=(1<<5);
	UBRR1L=BRX;//bps DIVISOR VALUE FOR 14.74MHz
	UBRR1H=BRX>>8;
    UCSR1B|=(1<<RXEN1)|(1<<TXEN1);//TRANSFER AND RECIEVE ENABLE
	UCSR1C|=(1<<UCSZ10)|(1<<UCSZ11);//8 BIT MOD
	//LSA3
	DDRH|=(1<<2);//~UEN
	DDRH&=~(1<<0);	//RXD2
	DDRH|=(1<<1);	//TXD2
	DDRE&=~(1<<6);	//INT6
	EICRB|=3<<4;//PE6 junction pulse RAISING EDGE
	EIMSK |=(1<<6);
	UBRR2L=BRX;// bps DIVISOR VALUE FOR 14.74MHz
	UBRR2H=BRX>>8;
    UCSR2B|=(1<<RXEN2)|(1<<TXEN2);//TRANSFER AND RECIEVE ENABLE
	UCSR2C|=(1<<UCSZ20)|(1<<UCSZ21);//8 BIT MODE
	//LSA4
	DDRJ|=(1<<2);//~UEN
	DDRJ&=~(1<<0);	//RXD2
	DDRJ|=(1<<1);	//TXD2
	DDRE&=~(1<<7);	//INT7
	EICRB|=3<<6;//PE6 junction pulse RAISING EDGE
	EIMSK |=(1<<7);
	UBRR3L=BRX;//bps DIVISOR VALUE FOR 14.74MHz
	UBRR3H=BRX>>8;
    UCSR3B|=(1<<RXEN3)|(1<<TXEN3);//TRANSFER AND RECIEVE ENABLE
	UCSR3C|=(1<<UCSZ30)|(1<<UCSZ31);//8 BIT MODE
}
uint8_t get_lsa(int cn)
{
	switch (cn)
	{
	case 1:
		return receive1();
		break;
	case 3:
		return receive3();
		break;
	case 2:
		return receive4();
		break;
	case 4:
		return receive2();
		break;
	}
}
void disablelsa1()
{
	EIMSK &=~(1<<4);
}
void disablelsa2()
{
	EIMSK &=~(1<<5);
}
void disablelsa3()
{
	EIMSK &=~(1<<6);
}
void disablelsa4()
{
	EIMSK &=~(1<<7);
}
void enablelsa1()
{
	EIMSK |=(1<<4);
}
void enablelsa2()
{
	EIMSK |=(1<<5);
}
void enablelsa3()
{
	EIMSK |=(1<<6);
}
void enablelsa4()
{
	EIMSK |=(1<<7);
}
// ----------------- UART ----------------- //
uint8_t uart_read1()
{
	uint8_t b;
	while(!(UCSR0A &(1<<RXC0)));
	b=UDR0;
	return b;
}
void uart_send1(uint8_t data)
{
	UDR0=data;
	while(!(UCSR0A &(1<<TXC0)));
}
uint8_t uart_read2()
{
	uint8_t b;
	while(!(UCSR1A &(1<<RXC1)));
	b=UDR1;
	return b;
}
void uart_send2(uint8_t data)
{
	UDR1=data;
	while(!(UCSR1A &(1<<TXC1)));
}
uint8_t uart_read3()
{
	uint8_t b;
	b=UDR2;
	while(!(UCSR2A &(1<<RXC2)));
	b=UDR2;
	return b;
}
void uart_send3(uint8_t data)
{
	while ( !( UCSR2A & (1<<UDRE2)) );
	UDR2=data;
	while(!(UCSR2A &(1<<TXC2)));
}
uint8_t uart_read4()
{
	uint8_t b;
	while(!(UCSR3A &(1<<RXC3)));
	b=UDR3;
	return b;
}
void uart_send4(uint8_t data)
{
	UDR3=data;
	while(!(UCSR3A &(1<<TXC3)));
}

void calib()
{
		//PORTE&=~(1<<2);
		uart_send1(0x01);
		uart_send1(0x50);
		uart_send1(0x01);
		uart_send1(0x52);
		//
}
int receive1()
{
		PORTE&=~(1<<2);
		int b=uart_read1();
		PORTE|=(1<<2);
		return b;
}
int receive2()
{
		PORTD&=~(1<<4);
		int b=uart_read2();
		PORTD|=(1<<4);
		return b;
}
int receive3()
{
		PORTH&=~(1<<2);
		int b=uart_read3();
		PORTH|=(1<<2);
		return b;
}
int receive4()
{
		PORTJ&=~(1<<2);
		int b=uart_read4();
		PORTJ|=(1<<2);
		return b;
}
