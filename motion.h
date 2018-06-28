/*
 * motion.h
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

#define motord PORTL //to be defined
#define inc 10
#define min_pwm 600
#define max_pwm 800
#define setvalue 35
#define kp 2
#define ki 0
#define kd 2
uint8_t slow=0x00;
uint8_t slow_fa = 0x01;
uint8_t medium=0x02;
uint8_t fast=0x03;
#define ackw_1		0x01
#define ckw_1		0x02
#define ackw_2		0x04
#define ckw_2		0x08
#define ackw_3		0x10
#define ckw_3		0x20
#define ackw_4		0x40
#define ckw_4		0x80
#define fwd1 ckw_4|ackw_2
#define rev1 ackw_4|ckw_2
#define fwd2 ckw_3|ackw_1
#define rev2 ackw_3|ckw_1
#define sp1_s slow
#define sp1_m medium
#define sp1_f fast
#define sp1_sm slow_fa
#define sp2_s (slow<<2)
#define sp2_m (medium<<2)
#define sp2_f (fast<<2)
#define sp2_sm (slow_fa << 2)
#define sp3_s (slow<<4)
#define sp3_m (medium<<4)
#define sp3_f (fast<<4)
#define sp3_sm (slow_fa << 4)
#define sp4_s (slow<<6)
#define sp4_m (medium<<6)
#define sp4_f (fast<<6)
#define sp4_sm (slow_fa << 6)
#define rotate_cw ckw_1|ckw_2|ckw_3|ckw_4
#define rotate_acw ackw_1|ackw_2|ackw_3|ackw_4
#define stop 0
#define start1 50//447cm
#define start2 68//596cm
#define tz12 25//200cm
#define tz1 25//269cm
#define tz2 25//269cm
#define tz3 87//595cm
void initpwm()
{
	DDRL=0xff;			//Locomotion Direction Bits		1|2|3|4
	DDRB=0xf0;			//PWM(4-Motor1	5,6,7-LOCO 1,2,3)
	DDRH=0xff;			//2-Chip Select	PWM(3-LOCO 4)
	//FAST PWM && INVERTING MODE && NO PRESCALER
	OCR1A=00;
	OCR1B=00;
	OCR1C=00;
	OCR4A=00;
	TCCR1A=0xff;
	TCCR1C=0x00;
	TCCR1B=0x09;
	TCCR4A=0xff;
	TCCR4C=0x00;
	TCCR4B=0x09;
	OCR1A=	OCR1B=	OCR1C=	OCR4A=min_pwm;
}
// -------------- SPI ----------------- //

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB = ((1<< 2 )|(1<< 1) | (1<<0) );
	/* Enable SPI, Master, set clock rate fck/16 */		
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}
void SPI_MasterTransmit(char cData)
{
	PORTB &= ~(1<<0);
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	PORTB |= (1<<0);
}
void movement(uint8_t dir,uint8_t speed,uint8_t distance)
{
	SPI_MasterTransmit(speed);
	_delay_us(500);
	SPI_MasterTransmit(dir);
	_delay_us(500);
	SPI_MasterTransmit(distance);
	_delay_us(500);
}
// -------------- Motion without spi ----------------- //
void set_direction(uint8_t dir)
{
	motord = dir;
}
void setpwm(int x)
{
	//int inc=10;
	OCR1A=	OCR1B=	OCR1C=	OCR4A=min_pwm;
	for(;OCR1A<x;)
	{
		OCR1A+=inc;//1n
		OCR1B+=inc;//2
		OCR1C+=inc;//3
		OCR4A+=inc;//4
		_delay_us(20);
	}
}
void PWM(signed int curr, signed int prev,int p)
{
	//signed int err, deviation,prev_err;
	unsigned int left,right;
	//int P=0,I=0,D=0;

	/*prev_err = setvalue - prev;
	deviation = setvalue - curr;

	P = deviation * kp;
	I = (I + deviation) * ki;
	D = (deviation - prev_err) * kd;

	err = P + I + D;
	*/
	right = 900+prev ;
	left = 900 +curr;

	if(right > 1024)
	right = 1024;
	if(left > 1024)
	left=1024;

	if(right<600)
	right=600;
	if(left<600)
	left=600;
	int temp=min(left,right);
	if(p==1)
	{
		OCR1B=OCR4A=min_pwm;
		while(OCR1B<temp)
		{
			OCR1B += inc;			// LEFT MOTOR
			OCR4A += inc;			// RIGHT MOTOR
			_delay_us(10);
		}
		OCR1B = left;			// LEFT MOTOR
		OCR4A = right;			// RIGHT MOTOR
	}
	else if(p==2)
	{
		OCR1C=OCR1A=min_pwm;
		while(OCR1A<=temp)
		{
			OCR1C += inc;			// LEFT MOTOR
			OCR1A += inc;			// RIGHT MOTOR
			_delay_us(10);
		}
		OCR1A = left;			// LEFT MOTOR
		OCR1C = right;			// RIGHT MOTOR
	}
}

void stopm(int p)
{
	unsigned int left=0,right=0;
	if(p==1)
	{
		left=OCR1B ;			// LEFT MOTOR
		 right=OCR4A;			// RIGHT MOTOR
	}
	else if(p==2)
	{
		left=OCR1A;			// LEFT MOTOR
		right=OCR1C;			// RIGHT MOTOR
	}
	while(left > 600 && right > 600)
	{
		if(left>600)
			left-=10;
		if(right>600)
			right-=10;
		if(p==1)
		{
			OCR1B = left;			// LEFT MOTOR
			OCR4A = right;			// RIGHT MOTOR
		}
		else if(p==2)
		{
			OCR1A = left;			// LEFT MOTOR
			OCR1C = right;			// RIGHT MOTOR
		}
		_delay_us(50);
	}
}
