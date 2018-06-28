/*
 * line.c
 *
 *  Created on: Dec 10, 2017
 *      Author: dkrocks
 * distance in  2560
 *  __________________________-----------final code------------________________________
 */

#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/iom2560.h>

#define MAX 4000
#define min(a, b) (a < b ? a : b)
#define count1 TCNT4
#define count2 TCNT5
volatile uint8_t TOP = -1, stack[MAX];
volatile signed int curr, prev;

volatile int f = 0 /*sensor and dir flag*/, junction_counter = 0 /*total current junction counter*/, k = 1 /*total expected junction count*/, zone = 0 /*latest completed zone*/, start = 0 /*start type TZ1 or TZ2*/, re = -1 /*current motion type*/, prev = 255 /*prev LSA08 value*/, slow_flag = 0 /*flag to reduce the speed of the bot*/;
volatile uint8_t distance = 0; /*PID distance*/
volatile uint8_t back = 0, med_flag = 0, fa_flag = 0;
volatile uint8_t v1 = 0, v2 = 0, v3 = 0, v4 = 0, counter1 = 0, counter2 = 0;
volatile uint8_t first = 1, gotz12 = 0, gotz21 = 0;
volatile uint8_t get = 0, dis_cover = 0, inside = 0;
volatile uint8_t align_com = 0, last_zone = -1;
// ------------------ Include ----------------- //

#include "lsa08.h"
#include "motion.h"

// ------------------ Initialize ----------------- //
// void task()
// {
// 	//TASK Complete
// 	//DDRD&=~(1);	//INT0
// 	//EICRA|=3;//PD0 junction pulse RAISING EDGE
// 	//EIMSK |=(1<<0);
// 	//DDRD|=(1<<5);
// 	DDRD &= ~(1 << 1); //INT1
// 	EICRA |= 3 << 2;   //PD1 junction pulse RAISING EDGE*/
// 	EIMSK |= (1 << 1);
// }
void counter_init()
{
    DDRL &= ~(1 << 2);
    TCCR5B |= (1 << CS50) | (1 << CS51) | (1 << CS52);
    TIMSK5 |= (1 << TOIE5);
    DDRH &= ~(1 << 7);
    TCCR4B |= (1 << CS40) | (1 << CS41) | (1 << CS42);
    TIMSK4 |= (1 << TOIE4);
}
void init()
{
    cli();
    DDRA = 0XFF; //output testing LED
    DDRC = 0xff; //piston operation
    PORTA = 0;
    DDRD &= ~(7 << 5); //limit switch
    //DDRK = 0xff;	   //PPR
    DDRH &= ~(1 << 3); //start
    PORTK = 0;
    PORTJ = 0;
    DDRJ &= ~(1 << 7);           //pushbutton input
    DDRL |= (1 << 3) | (3 << 0); //nano output
    uart_init();
    SPI_MasterInit();
    disablelsa1();
    disablelsa2();
    disablelsa4();
    disablelsa3();
    counter_init();
}
void nano(uint8_t bit)
{
    if (bit != 0)
        PORTL = (bit & 0x3) | ((bit & 0x4) << 1);
    else
        PORTL &= ~(0xb);
}
double getpulses()
{
    double revol = (4 * distance * 7) / (12.7 * 22);
    double val = (revol * 1000);
    return val;
    //25.0637706
}
// -------------- Value Stack ----------------- //
void push(int x)
{
    /*	if(TOP==(MAX-1))*/
    /*		TOP=0;						//CIRCULAR STACK*/
    /*	else*/
    TOP++;
    stack[TOP] = x;
}

int pop()
{
    if (TOP >= 0)
    {
        int x = stack[TOP];
        /*	if(TOP==0)*/
        /*		TOP=(MAX-1);				//CIRCULAR STACK*/
        /*	else*/
        TOP--;
        return (x);
    }
    return 35;
}
// -------------- Motion ----------------- //
void set_last_lsa()
{
    int tp;
    tp = receive1();
    if (tp != 255)
        v1 = tp;
    tp = receive2();
    if (tp != 255)
        v2 = tp;
    tp = receive3();
    if (tp != 255)
        v3 = tp;
    tp = receive4();
    if (tp != 255)
        v4 = tp;
}
uint8_t get_last_lsa(int cn)
{
    switch (cn)
    {
    case 1:
        return v1;
        break;
    case 3:
        return v3;
        break;
    case 2:
        return v4;
        break;
    case 4:
        return v2;
        break;
    }
}
void delay(int y)
{
    int t = 0;
    y = (y / 10) - 1;
    while (t < y)
    {
        set_last_lsa();
        _delay_ms(10);
        t++;
    }
}
void align(int cn)
{
    int y = 0, k = 0;
    int a = get_last_lsa(cn); //back;
    int f = 1;
    // if (get_lsa(cn) == 255 || get == 1 || align_com == 1)
    {
        while ((f == 1))
        {
            PORTA = a;
            //movement(rotate_cw,sp1_s|sp2_s|sp3_s|sp4_s,distance);
            //set_direction(rotate_cw);
            //setpwm(700)
            if (a > 35)
            {
                movement((ckw_1 << (2 * (cn - 1))), sp1_sm << (2 * (cn - 1)), distance);
                //_delay_ms(100);
                delay(80);
                movement(0xff, 0, distance);
            }
            //movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
            else if (a < 35)
            {
                movement((ackw_1 << (2 * (cn - 1))), sp1_sm << (2 * (cn - 1)), distance);
                //_delay_ms(100);
                delay(80);
                movement(0xff, 0, distance);
            }
            _delay_ms(10);
            k = get_lsa(cn);
            if (k != 255)
            {
                a = k;
                f = 0;
            }
            set_last_lsa();
        }
        while ((a > 45 || a < 25))
        {
            PORTA = a;
            //movement(rotate_cw,sp1_s|sp2_s|sp3_s|sp4_s,distance);
            //set_direction(rotate_cw);
            //setpwm(700)
            if (a > 45)
            {
                movement((ckw_1 << (2 * (cn - 1))), sp1_sm << (2 * (cn - 1)), distance);
                //_delay_ms(100);
                delay(50);
                movement(0xff, 0, distance);
            }
            //movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
            else if (a < 25)
            {
                movement((ackw_1 << (2 * (cn - 1))), sp1_sm << (2 * (cn - 1)), distance);
                //_delay_ms(100);
                delay(50);
                movement(0xff, 0, distance);
            }
            _delay_ms(10);
            k = get_lsa(cn);
            if (k != 255)
            {
                a = k;
                f = 0;
            }
            set_last_lsa();
        }
    }
}
void curve1()
{
    count2 = count1 = 0;
    counter1 = counter2 = 0;
    movement(rev2, sp1_sm | sp3_sm, 15);
    while (count2 <= 500)
    {
        set_last_lsa();
        PORTA = count2;
    }
    movement(0xff, 0xff, 15);
    movement(rev2 | ackw_2 | ackw_4, sp1_sm | sp3_sm | sp2_m | sp4_m, 12);
    while (count1 <= 1000)
    {
        set_last_lsa();
        PORTA = count1;
    }
    movement(rev2, sp1_s | sp3_s, 15);
    int a = receive1();
    while (a != 255)
    {
        set_last_lsa();
        a = receive1();
        _delay_ms(10);
    }
    while (a == 255)
    {
        set_last_lsa();
        a = receive1();
        _delay_ms(10);
    }
    movement(0xff, 0xff, 15);
    count2 = 0;
    // while(count2<=50)
    // {
    // 	set_last_lsa();
    // 	PORTA=count2;
    // }
    // movement(rev2 | fwd1, sp1_sm | sp3_sm | sp2_m | sp4_m, 12);
    // while (count2 <= 900)
    // {
    //     set_last_lsa();
    //     PORTA = count2;
    // }
    movement(rev2 | fwd1, sp3_sm | sp2_sm | sp4_m, 12);
    while (count2 <= 500)
    {
        set_last_lsa();
        PORTA = count2;
    }
    movement(fwd1 , sp2_sm | sp4_sm | sp3_s, 15);
    a = receive1();
    while (a == 255)
    {
        set_last_lsa();
        a = receive1();
        _delay_ms(10);
    }
    while (a != 255)
    {
        set_last_lsa();
        a = receive1();
        _delay_ms(10);
    }
    movement(fwd1 | ackw_3, sp2_sm | sp4_sm | sp3_s, 15);
    while (a == 255)
    {
        set_last_lsa();
        a = receive1();
        _delay_ms(10);
    }
    movement(0xff, 0xff, 15);
    delay(50);
    align(1);
}
void rotate_acw90()
{
    movement(0xff, 0xff, 0xff);
    count2 = 0;
    counter2 = 0;
    movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
    while (count2 <= 400)
    {
        set_last_lsa();
    }
    v1 = 255;
    while (count2 <= 1153)
    {
        set_last_lsa();
    }
    movement(0xff, 0, distance);
    delay(200);
    align(1);
}
void go(int first)
{
    //cli();
    count2 = 0;
    counter2 = 0;
    count1 = 0;
    counter1 = 0;
    PORTA = 0xff;
    if (first == 1)
    {
        movement(fwd2, sp1_s | sp3_s, 15);
        while (count2 <= 300)
        {
            set_last_lsa();
            PORTA = count2;
        }
        sei();
        junction_counter = 0;
        enablelsa4();
        while (count2 <= 1750)
        {
            set_last_lsa();
            PORTA = count2;
        }
        movement(0xff, 0xff, 0xff);
        //_delay_ms(300);
        // delay(500);
        // align(4);
        // align(2);
        //first=0;
    }
    else if (first == 2)
    {
        movement(0xff, 0xff, 0xff);
        movement(rev1, sp2_s | sp4_s, 155);
        while (count1 <= 300)
        {
            set_last_lsa();
            PORTA = count1;
        }
        while (count1 <= 1150)
        {
            set_last_lsa();
            PORTA = count1;
        }
        while (count1 <= 3000 && (receive4() == 255))
            set_last_lsa();
        movement(0xff, 0xff, 0xff);
        //_delay_ms(300);
        delay(500);
        if (gotz12 == 1)
            align(2);
        else
            align(4);
        //first=0;
    }
    else
    {
        //first=1;
        movement(rev2, sp1_s | sp3_s, 15);
        while (count2 <= 300)
            set_last_lsa();
        sei();
        junction_counter = 0;
        enablelsa4();
        while (count2 <= 1750)
            set_last_lsa();
        v1 = v2 = v3 = v4 = 255;
        while (count2 <= 3000 && (receive1() == 255))
            set_last_lsa();
        movement(0xff, 0xff, 0xff);
        //_delay_ms(300);
        delay(300);
        align(1);
    }
    junction_counter = 0;
    disablelsa4();
}
void rotate_cw90()
{
    movement(0xff, 0xff, 0xff);
    count2 = 0;
    counter2 = 0;
    movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
    while (count2 <= 400)
    {
        set_last_lsa();
    }
    v1 = v3 = 0;
    v2 = v4 = 255;
    while (count2 <= 1153)
    {
        set_last_lsa();
    }
    // movement(rotate_cw, sp1_sm | sp2_sm | sp3_sm | sp4_sm, distance);
    // //set_direction(rotate_acw);
    // //setpwm(800);
    int a = receive1();
    // while (a != 255)
    // {
    // 	set_last_lsa();
    // 	a = receive1();
    // }
    // movement(0xff, 0, distance);
    // while (a == 255)
    // {
    // 	movement(rotate_cw, sp1_sm | sp2_sm | sp3_sm | sp4_sm, distance);
    // 	//_delay_ms(40);
    // 	delay(5);
    // 	movement(0xff, 0, distance);
    // 	a = receive1();
    // 	//_delay_ms(5);
    // }
    movement(0xff, 0, distance);
    //set_direction(0xff);
    //_delay_ms(500);
    delay(200);
    align(2);
    // int y = 0, k = 0;
    // for (y = 0; y < 30; y++)
    // {
    //     set_last_lsa();
    //     a = get_last_lsa(2);
    // }
    // a = k;
    // while (a == 255 || a > 65 || a < 5)
    // {
    //     PORTA = a;
    //     //movement(rotate_cw,sp1_s|sp2_s|sp3_s|sp4_s,distance);
    //     //set_direction(rotate_cw);
    //     //setpwm(700);
    //     // if (a == 255)
    //     // {
    //     // 	movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
    //     // 	_delay_ms(50);
    //     // 	movement(0xff, 0, distance);
    //     // }
    //     // else
    //     if (a > 55)
    //     {
    //         movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
    //         //_delay_ms(50);
    //         delay(50);
    //         movement(0xff, 0, distance);
    //     }
    //     //movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
    //     else if (a < 15)
    //     {
    //         movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
    //         //_delay_ms(50);
    //         delay(50);
    //         movement(0xff, 0x55, distance);
    //     }
    //     set_last_lsa();
    //     k = receive4();
    //     if (k != 255)
    //         a = k;
    //     // k = receive2();
    //     // if (k != 255)
    //     // 	back = k;
    //     _delay_ms(10);
    // }
}
//TASK COMPLETE's
//ISR(INT0_vect)
void change()
{
    movement(0xff, 0xff, 0xFF);
    //cli();
    count1 = count2 = 0;
    counter1 = counter2 = 0;
    re = -1;
    prev = 255;
    slow_flag = 0;
    med_flag = 0;
    dis_cover = 0;
    fa_flag = 0;
    if (zone == 0)
    {
        if (f == -1 && inside == 0)
        {
            //f = 1;
            f = 0;
            junction_counter = 0;
            k = 2;
            TOP = -1;
            PORTA = 0x66;
            //_delay_ms(500);
            //set_direction(fwd2);
            //setpwm(800);
            distance = tz1;
            inside = 1;
            if (!gotz21)
            {
                rotate_acw90();
                align(1);
            }
            gotz21 = 0;
            movement(fwd1, sp2_s | sp4_s, distance);
            //movement(fwd2, sp1_s | sp3_s, distance);
            //_delay_ms(200);
            delay(200);
            v1 = v2 = v3 = v4 = 0;
            junction_counter = 0;
        }
        else if (f == -1 && inside == 1)
        {
            f = 2;
            //f = 0;
            junction_counter = 0;
            k = 2;
            zone = 1;
            PORTA = 0x66;
            //_delay_ms(500);
            //set_direction(rev2);
            //setpwm(800);
            inside = 0;
            distance = tz1;
            //movement(fwd1, sp2_s | sp4_s, distance);
            movement(rev2, sp1_s | sp3_s, distance);
            _delay_ms(200);
            junction_counter = 0;
        }
        if (f == -3)
        {
            f = 0;
            junction_counter = 0;
            k = 2;
            TOP = -1;
            zone = 1;
            inside = 0;
            PORTA = 0x66;
            //_delay_ms(500);
            //set_direction(fwd1);
            //setpwm(800);
            movement(fwd1, sp2_s | sp4_s, distance);
            _delay_ms(200);
            junction_counter = 0;
        }
    }
    else if (zone == 1)
    {
        if (f == -3)
        {
            f = 0;
            junction_counter = 0;
            k = 1;
            PORTA = zone;
            //_delay_ms(500);
            //set_direction(fwd1);
            //setpwm(800);
            distance = tz12;
            movement(fwd1, sp2_s | sp4_s, distance);
            //_delay_ms(200);
            delay(200);
            slow_flag = 1;
            junction_counter = 0;
        }
        else if (f == -1 && inside == 0)
        {
            //f = 1;
            f = 0;
            junction_counter = 0;
            k = 2;
            TOP = -1;
            PORTA = zone;
            // _delay_ms(500);
            inside = 1;
            //set_direction(fwd2);
            //setpwm(800);
            distance = tz2;
            if (!gotz12)
            {
                rotate_acw90();
                align(1);
            }
            gotz12 = 0;
            movement(fwd1, sp1_s | sp3_s, distance);
            //_delay_ms(200);
            delay(200);
            v1 = v2 = v3 = v4 = 0;
            junction_counter = 0;
        }
        else if (f == -1 && inside == 1)
        {
            f = 2;
            junction_counter = 0;
            k = 2;
            //TODO
            zone = 2;
            PORTA = zone;
            // _delay_ms(500);
            //set_direction(rev2);
            //setpwm(800);
            distance = tz2;
            inside = 0;
            movement(rev2, sp1_s | sp3_s, distance);
            //_delay_ms(200);
            delay(200);
            junction_counter = 0;
        }
    }
    else if (zone == 2)
    {
        if (f == -1 && inside == 0)
        {
            //f = 1;
            f = 0;
            junction_counter = 0;
            k = 5;
            TOP = -1;
            PORTA = zone;
            // _delay_ms(500);
            //set_direction(fwd2);
            //setpwm(800);
            distance = tz3;
            inside = 1;
            {
                rotate_acw90();
                align(1);
            }
            movement(fwd1, sp2_s | sp4_s, distance);
            //_delay_ms(200);
            delay(200);
            v1 = v2 = v3 = v4 = 0;
            junction_counter = 0;
        }
        else if (f == -1 && inside == 1)
        {
            f = 2;
            junction_counter = 0;
            k = 5;
            //zone=4;
            PORTA = zone;
            //set_direction(rev2);
            //setpwm(800);
            distance = tz3;
            inside = 0;
            movement(rev2, sp1_s | sp3_s, distance);
            delay(200);
            //_delay_ms(200);
            junction_counter = 0;
        }
    }
    else if (zone == 4)
    {
        if (f == -4)
        {
            //f = 1;
            f = 2;
            junction_counter = 0;
            k = 1;
            TOP = -1;
            PORTA = zone;
            zone = 1;
            //_delay_ms(500);
            //set_direction(fwd2);
            //setpwm(800);
            distance = tz12;
            align(2);
            movement(rev2, sp1_s | sp3_s, distance);
            //_delay_ms(200);
            delay(400);
            v1 = v2 = v3 = v4 = 255;
            junction_counter = 0;
        }
        else if (f == -2)
        {
            f = 3;
            junction_counter = 0;
            k = 1;
            //zone=4;
            PORTA = zone;
            //set_direction(rev2);
            //setpwm(800);
            zone = 4;            distance = tz12;

            distance = tz12;
            inside = 0;
            med_flag = 1;
            align_com = 1;
            align(3);
            align_com = 0;
            movement(rev1, sp2_s | sp4_s, distance);
            delay(200);
            //_delay_ms(200);
            v1 = 255;
            v3 = v4 = 0;
            v2 = 255;
            junction_counter = 0;
        }
    }
    else if (zone == 5)
    {
        if (f == -4)
        {
            //f = 1;
            f = 1;
            junction_counter = 0;
            k = 1;
            TOP = -1;
            PORTA = zone;
            zone = 0;
            //_delay_ms(500);
            //set_direction(fwd2);
            //setpwm(800);
            distance = tz12;
            align(4);
            movement(fwd2, sp1_s | sp3_s, distance);
            //_delay_ms(200);
            delay(400);
            v1 = v2 = v3 = v4 = 0;
            junction_counter = 0;
        }
        else if (f == -2)
        {
            f = 3;
            junction_counter = 0;
            k = 0;
            //zone=4;
            PORTA = zone;
            //set_direction(rev2);
            //setpwm(800);
            distance = tz12;
            inside = 0;
            med_flag = 1;
            align_com = 1;
            align(3);
            align_com = 0;
            movement(rev1, sp2_s | sp4_s, distance);
            delay(200);
            //_delay_ms(200);
            v1 = v2 = v3 = v4 = 0;
            junction_counter = 0;
        }
    }
    count1 = count2 = 0;
    counter1 = counter2 = 0;
}
int main(void)
{
    cli();
    // DDRB=0;

    // PORTB=0xFF;
    // while(1)
    // {
    // 	DDRA=255;
    // 	Adc_init();
    // 	PORTA=present();
    // }
    //DDRD=0xff;
    //DDRD=0XFF;
    // {
    // 	_delay_ms(2000);
    // 	//rotate_cw90();
    // 	//PORTC |= (1 << 4);
    // 	//PORTC |= (1 << 3);
    // 	PORTC |= (1 << 3)|(1 << 4);
    // 	_delay_ms(1);
    // 	//PORTC |= (1 << 4);
    // 	//PORTC |= (1 << 3);
    // 	_delay_ms(2000);
    // 	//PORTC&=~(1<<3);
    // 	_delay_ms(1000);
    // 	PORTC &= ~(1 << 4);
    // 	PORTC &= ~(1 << 3);
    // 	//PORTC|=(1<<2);
    // 	_delay_ms(500);
    // 	//PORTC&=~(1<<2);|= (1 << 4)
    // }
    // while(1);
    // while (1)
    // {
    // 	{
    // 		if ((PIND >> 5 & (0x01)) == 0x01)
    // 			PORTA = 0X01;
    // 		else
    // 			PORTA &= ~(1 << 0);
    // 		if ((PIND >> 6 & (0x01)) == 0x01)
    // 			PORTA = 0X02;
    // 		else
    // 			PORTA &= ~(1 << 1);
    // 		if ((PIND >> 7 & (0x01)) == 0x01)
    // 			PORTA = 0X04;
    // 		else
    // 			PORTA &= ~(1 << 2);
    // 	}
    // }
    // enablelsa1();
    // enablelsa2();
    // rotate_cw90();
    // align();
    //  while(1);
    //f=2;

    /*	enablelsa3();*/
    // enablelsa3();
    // while(1)
    // 	{
    // 		//_delay_ms(5000);
    // 		PORTA=junction_counter;
    // 	}
    // movement(rotate_cw,sp1_m|sp2_m|sp3_m|sp4_m,0xa0);
    // while(1);
    /*		//PORTA=0;*/
    /*		//PORTA=0xaa;*/
    /*		_delay_ms(2000);*/
    // movement(rotate_cw,sp1_m|sp2_m|sp3_m|sp4_m,0xff);
    // _delay_ms(2000);
    /*		*/
    /*		_delay_ms(2000);*/
    /*		movement(rotate_cw,sp1_f|sp2_f|sp3_f|sp4_f,0xff);*/
    /*		_delay_ms(2000);*/
    /*		//movement(0xff,0);*/
    /*		//PORTA=0x77;*/
    /*	}*/
    /*DDRB=0xff;*/
    /*PORTB=0;*/
    // movement(0xff, 0, distance);
    //PORTA=junction_counter;
    slow_flag = 0;
    //f=-2;
    //PORTC |= (1 << 5);
    // while(1)
    // 	PORTA=count2;
    // movement(fwd1 | 0x3, sp2_f | sp4_f, distance);

    // movement(0xff, 0, distance);
    // go(1);
    // _delay_ms(1000);
    // go(0);
    // while (1)
    // 	;
    volatile uint8_t curr, a, b;
    init();
    // while (1) //((count1 + (counter * 65536)) <= getpulses())
    // {

    //     //count2 = 0;
    //     //counter = 0;
    //     PORTA = count2;
    //     re = -1;
    //     slow_flag = 1;
    // }
    PORTC = (1 << 0);
    nano(6);
    PORTC |= (1 << 3);
    _delay_ms(500);
    while (((PINH >> 3) & 0x1) == 0)
        ;
    //_delay_ms(1000);
    sei();
    int complete = 0;
    nano(5);
    // while(1)
    // {
    // 	if (((PINH >> 3) & 0x1) == 1)
    // 		PORTA=0xff;
    // 	else
    // 		PORTA=0;
    // }
    // ;
    SPI_MasterTransmit(0x05); // 	kp
    _delay_us(500);
    SPI_MasterTransmit(0x02); //	ki
    _delay_us(500);
    SPI_MasterTransmit(0x04); //	kd
    _delay_us(500);

    if (((PINJ >> 7) & 0x1) == 1)
        start = 1;
    else
        start = 0;
    distance = start1;
    if (start == 1)
    {
        f = -3;
        zone = 0;
        change();
        // PORTD |= (1 << 5);
        // _delay_us(50);
        // PORTD &= ~(1 << 5);
        _delay_ms(500);
        distance = start2;
    }
    //cli();
    // rotate_acw90();
    // curve1();
    // while (1)
    // 	;
    int tp;
    movement(fwd1 | fwd2, sp1_s | sp3_s | sp2_m | sp4_m, distance);
    tp = receive1();
    while (tp != 255)
    {
        tp = receive1();
        _delay_ms(10);
    }
    tp = receive1();
    while (tp == 255)
    {
        tp = receive1();
        _delay_ms(10);
    }
    nano(4);
    movement(0x30, 0x30, distance);
    PORTC = 0;
    // movement(0xff, 0x55, distance);
    // _delay_ms(300);
    // tp = receive1();
    // while (tp == 255)
    // {
    // 	movement(rev2 | 0xc0, sp1_s | sp3_s, distance);
    // 	_delay_ms(100);
    // 	movement(0xff, 0, distance);
    // 	tp = receive1();
    // 	_delay_ms(10);
    // }
    // nano(2);
    // _delay_ms(4000);
    // nano(4);
    // _delay_ms(4000);
    // nano(2);
    // _delay_ms(4000);
    // nano(0);
    // _delay_ms(8000);
    // while(1);
    // go(2);
    // while(1);
    dis_cover = 0;
    //f=3;
    count1 = 0;
    count2 = 0;
    while (1)
    {
        PORTA = junction_counter;
        //PORTA = f;
        if (dis_cover == 0 && ((count2 + (counter2 * 65536)) >= 200))
        {
            junction_counter = 0;
            dis_cover = 1;
            //sei();
        }
        if ((dis_cover == 0 && ((count1 + (counter1 * 65536)) >= 200)) || (inside==1 && dis_cover == 0))
        {
            junction_counter = 0;
            dis_cover = 1;
            //sei();
        }
        if (f == 0)
        {
            if (inside == 1)
            {
                if ((fa_flag == 0) && (count1 + (counter1 * 65536)) >= (getpulses() - 3000))
                {
                    fa_flag = 1;
                    PORTA = 0xff;
                    movement(0xff, 0x55, distance);
                    delay(30);
                    re = -1;
                }
            }
            if (med_flag == 0 && (count1 + (counter1 * 65536)) >= (getpulses() - 1000))
            {
                med_flag = 1;
                PORTA = 0xff;
                movement(0xff, 0xff, distance);
                delay(50);
                re = -1;
                count1 = 0;
                counter1 = 0;
                // v1 = v2 = v3 = v4 = 255;
            }
            // if (inside == 1)
            if (slow_flag == 0 && (count1 + (counter1 * 65536)) >= (getpulses() - 500))
            {
                count1 = 0;
                counter1 = 0;
                PORTA = 0x55;
                re = -1;
                slow_flag = 1;
                movement(0xff, 0xff, distance);
                delay(50);
            }
        }
        else if ((f > 0))
        {

            if (fa_flag == 0 && (count2 + (counter2 * 65536)) >= (getpulses() - 3000))
            {
                fa_flag = 1;
                movement(0xff, 0x55, distance);
                delay(30);
                re = -1;
            }
            if (med_flag == 0 && (count2 + (counter2 * 65536)) >= (getpulses()) - 1000)
            {
                med_flag = 1;
                movement(0xff, 0x55, distance);
                re = -1;
                delay(30);
            }
            if (slow_flag == 0 && (count2 + (counter2 * 65536) - 1000) >= getpulses())
            {
                count2 = 0;
                counter2 = 0;
                PORTA = 0x55;
                re = -1;
                slow_flag = 1;
                movement(0xff, 0x55, distance);
                delay(80);
            }
        }
        if (f == 0)
        {
            set_last_lsa();
            enablelsa1();
            // if(slow_flag)
            // 	PORTA = count2;
            //PORTA=a;
            if (junction_counter >= k && dis_cover == 1)
            {
                // b = receive2();
                if (receive2() <= 70 || receive4() <= 70)
                {
                    f = -1;
                    junction_counter = 0;
                    PORTA = junction_counter;
                    disablelsa1();
                    movement(rev1 | 0x3, sp4_sm | sp2_sm, distance);
                    //while(1);
                    //_delay_ms(200);
                    delay(50);
                    movement(0xff, 0, distance);
                    //_delay_ms(200);
                    delay(50);
                    b = get_last_lsa(4); //receive2();
                    prev = 255;
                    int to = 0;
                    while (b == 255)
                    {
                        if (get_last_lsa(4) < 35)
                            movement(rev1 | 0x3, sp4_sm | sp2_sm, distance);
                        else
                            movement(fwd1 | 0x3, sp4_sm | sp2_sm, distance);
                        //set_direction(rev1);
                        //PWM(-400,-400,1);
                        //_delay_ms(320);
                        delay(50);
                        PORTA = 0xa;
                        movement(0xff, 0x55, distance);
                        b = receive2();
                    }
                    delay(200);
                    b = get_last_lsa(4);
                    while (b > 55 || b < 15)
                    {
                        if (b == 255)
                        {
                            movement(rev1 | 0x3, sp4_sm | sp2_sm, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(100);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else if (b < 15)
                        {
                            movement(rev1 | 0x3, sp4_sm | sp2_sm, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(100);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else
                        {
                            movement(fwd1 | 0x3, sp4_sm | sp2_sm, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(100);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        b = get_last_lsa(4);
                        //receive2();
                        //_delay_ms(10);
                        // a = receive2();
                        // if (a != 255)
                        // 	back = a;
                        set_last_lsa();
                    }
                    re = -1;
                    a = b = 255;
                    prev = 255;
                    /*					set_direction(0xff);*/
                    /*					_delay_ms(500);*/
                    /*					set_direction(0x00);*/
                    continue;
                }
            }
            a = receive1();
            if (a != 255)
            {
                prev = a;
                //c=1;
            }
            else if (prev != 255)
            {
                re = -1;
                if (prev >= 35)
                {
                    //clock
                    movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_cw,sp1_s);
                    //_delay_ms(60);
                    delay(60);
                    movement(0xff, 0, distance);
                }
                else
                {
                    //anti-clock
                    movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_acw,sp1_s);
                    //_delay_ms(60);
                    delay(60);
                    movement(0xff, 0, distance);
                }
                re = -1;
            }
            //set_direction(fwd1);
            if (a > 70)
            {
                //set_direction(stop);
                if (re != 0)
                {
                    movement(0xff, 0, distance);
                    re = 0;
                }
            }
            else if (a < 25)
            {
                if (re != 1)
                {
                    if (slow_flag)
                    {
                        movement(fwd1 | ackw_1, sp1_s | sp2_s | sp4_s, distance);
                        //movement(0xff,0,distance);
                        //movement(fwd1,sp2_m|sp4_s);
                    }
                    else if (med_flag)
                    {
                        movement(fwd1 | ackw_1, sp1_s | sp2_sm | sp4_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(fwd1 | ackw_1, sp1_s | sp2_m | sp4_m, distance);
                    }
                    else
                        //movement(fwd1,sp2_f|sp4_m);
                        movement(fwd1 | ackw_1, sp1_m | sp2_f | sp4_f, distance); //sp4_m
                    re = 1;
                }
                //PWM(100,-100,1);
            }
            else if (a > 45)
            {
                if (re != 2)
                {
                    if (slow_flag)
                    {
                        movement(fwd1 | ckw_1, sp1_s | sp2_s | sp4_s, distance);
                        //movement(0xff,0,distance);
                        //movement(fwd1,sp2_s|sp4_m);
                    }
                    else if (med_flag)
                    {
                        movement(fwd1 | ckw_1, sp1_s | sp2_sm | sp4_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(fwd1 | ckw_1, sp1_s | sp2_m | sp4_m, distance);
                    }
                    else
                        movement(fwd1 | ckw_1, sp1_m | sp2_f | sp4_f, distance); //sp2_m
                    //movement(fwd1,sp2_m|sp4_f);
                    re = 2;
                }
                //PWM(-100,100,1);
            }
            else
            {
                if (re != 3)
                {
                    if (slow_flag)
                    {
                        //movement(0xff,0);
                        //movement(0xff,0,distance);
                        movement(fwd1 | 0x3, sp2_s | sp4_s, distance);
                    }
                    else if (med_flag)
                    {
                        movement(fwd1 | 0x03, sp2_sm | sp4_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(fwd1 | 0x03, sp2_m | sp4_m, distance);
                    }
                    else
                    {
                        //movement(0xff,0);
                        movement(fwd1 | 0x3, sp2_f | sp4_f, distance);
                    }
                    re = 3;
                }
                //PWM(100,100,1);
            }
        }
        else if (f == 1)
        {
            enablelsa2();
            if (junction_counter >= (k) && dis_cover == 1)
            {
                // if (slow_flag == 0)
                // {
                // 	movement(0xff, 0, distance);
                // 	//_delay_ms(400);
                // 	delay(400);
                // 	re = -1;
                // }
                slow_flag = 1;
                b = receive1();
                if (b < 70)
                {
                    if (gotz21 == 1)
                        f = -1;
                    else
                        f = -2;
                    junction_counter = 0;
                    PORTA = junction_counter;
                    disablelsa2();
                    movement(rev2 | 0xc0, sp1_s | sp3_s, distance);
                    //_delay_ms(100);
                    delay(100);
                    movement(0xff, 0, distance);
                    //_delay_ms(200);
                    delay(200);
                    b = receive1();
                    prev = 255;
                    int to = 0;
                    while (b == 255)
                    {
                        movement(rev2 | 0xc0, sp1_s | sp3_s, distance);
                        //set_direction(rev1);
                        //PWM(-400,-400,1);
                        //_delay_ms(320);
                        // to = 0;
                        // while (to < 20)
                        // {
                        // 	set_last_lsa();
                        // 	_delay_ms(10);
                        // 	to++;
                        // }
                        delay(210);
                        PORTA = 0xa;
                        movement(0xff, 0, distance);
                        b = receive1();
                    }
                    while (b > 45 || b < 25)
                    {
                        if (b == 255)
                        {
                            movement(rev2 | 0xc0, sp1_s | sp3_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            delay(210);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else if (b < 25)
                        {
                            movement(fwd2 | 0xc0, sp1_s | sp3_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(200);
                            // to = 0;
                            // while (to < 15)
                            // {
                            // 	set_last_lsa();
                            // 	_delay_ms(10);
                            // 	to++;
                            // }
                            delay(170);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else
                        {
                            movement(rev2 | 0xc0, sp1_s | sp3_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(200);
                            delay(170);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        b = get_last_lsa(1);
                        //_delay_ms(10);
                        // a = receive2();
                        // if (a != 255)
                        // 	back = a;
                        set_last_lsa();
                    }
                    movement(0xff, 0, distance);
                    re = -1;
                    a = b = 255;
                    prev = 255;
                    //set_direction(0xff);
                    //_delay_ms(500);
                    //set_direction(0x00);
                    continue;
                }
            }
            a = receive2();
            if (a != 255)
            {
                prev = a;
                //c=1;
            }
            else if (prev != 255)
            {
                re = -1;
                if (prev >= 35)
                {
                    //clock
                    movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_cw,sp1_s);
                    //_delay_ms(30);
                    delay(30);
                    movement(0xff, 0, distance);
                }
                else
                {
                    //anti-clock
                    movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_acw,sp1_s);
                    //_delay_ms(30);
                    delay(30);
                    movement(0xff, 0, distance);
                }
            }
            if (a > 70)
            {
                if (re != 0)
                {
                    movement(0xff, 0, distance);
                    re = 0;
                }
                //set_direction(stop);
            }
            else if (a < 25)
            {
                if (re != 1)
                {
                    if (slow_flag)
                    {
                        movement(fwd2 | ackw_4, sp1_s | sp3_s | sp4_s, distance);
                    }
                    else if (med_flag)
                    {
                        movement(fwd2 | ackw_4, sp1_sm | sp3_sm | sp4_s, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(fwd2 | ackw_4, sp1_m | sp3_m | sp4_sm, distance);
                    }
                    else
                        movement(fwd2 | ackw_4, sp1_f | sp3_m | sp4_s, distance);
                    re = 1;
                }
                //PWM(100,-100,2);
            }
            else if (a > 45)
            {
                if (re != 2)
                {
                    if (slow_flag)
                    {
                        movement(fwd2 | ckw_4, sp1_s | sp3_s | sp4_s, distance);
                    }
                    else if (med_flag)
                    {
                        movement(fwd2 | ckw_4, sp1_sm | sp3_sm | sp4_s, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(fwd2 | ckw_4, sp1_m | sp3_m | sp4_sm, distance);
                    }
                    else
                        movement(fwd2 | ckw_4, sp1_m | sp3_f | sp4_s, distance);
                    re = 2;
                }
                //PWM(-100,100,2);
            }
            else
            {
                if (re != 3)
                {
                    if (slow_flag)
                    {
                        movement(fwd2 | 0xc0, sp1_s | sp3_s, distance);
                    }
                    else if (med_flag)
                    {
                        movement(fwd2 | 0xc0, sp1_sm | sp3_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(fwd2 | 0xc0, sp1_m | sp3_m, distance);
                    }
                    else
                    {
                        movement(fwd2 | 0xc0, sp1_f | sp3_f, distance);
                    }
                    re = 3;
                }
                //PWM(100,100,2);
            }
        }
        else if (f == 2)
        {

            if (gotz12 == 1)
                enablelsa4();
            else
                enablelsa2();
            PORTA = junction_counter;
            if (junction_counter >= (k) && dis_cover == 1)
            {
                if (!gotz12)
                {
                    movement(0xff, 0xff, distance);
                    f = -3;
                    re = -1;
                    a = b = 255;
                    prev = 255;
                    continue;
                }
                b = receive1();
                if (gotz12 == 1)
                    disablelsa4();
                else
                    disablelsa2();
                if (b < 70)
                {
                    f = -1;
                    junction_counter = 0;
                    PORTA = junction_counter;
                    //disablelsa4();
                    movement(fwd2 | 0x0c, sp1_s | sp3_s, distance);
                    //_delay_ms(100);
                    delay(100);
                    movement(0xff, 0, distance);
                    //_delay_ms(300);
                    delay(300);
                    b = receive1();
                    prev = 255;
                    while (b > 45 || b < 25)
                    {
                        if (b == 255)
                        {
                            movement(fwd2 | 0xc0, sp1_s | sp3_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(200);
                            delay(200);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else if (b < 25)
                        {
                            movement(fwd2 | 0xc0, sp1_s | sp3_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(200);
                            delay(200);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else
                        {
                            movement(rev2 | 0xc0, sp1_s | sp3_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(200);
                            delay(200);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        b = receive1();
                        _delay_ms(10);
                        // a = receive2();
                        // if (a != 255)
                        // 	back = a;
                        set_last_lsa();
                    }
                    movement(0xff, 0, distance);
                    re = -1;
                    continue;
                }
            }
            a = receive4();
            if (a != 255)
            {
                prev = a;
                //c=1;
            }
            else if (prev != 255)
            {
                re = -1;
                if (prev >= 35)
                {
                    //clock
                    movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_cw,sp1_s);
                    //_delay_ms(30);
                    delay(30);
                    movement(0xff, 0, distance);
                }
                else
                {
                    //anti-clock
                    movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_acw,sp1_s);
                    //_delay_ms(30);
                    delay(30);
                    movement(0xff, 0, distance);
                }
            }
            //set_direction(rev2);
            if (a > 70)
            {
                if (re != 0)
                {
                    movement(0xff, 0, distance);
                    re = 0;
                }
                //set_direction(stop);
            }
            else if (a < 25)
            {
                if (re != 1)
                {
                    if (slow_flag)
                    {
                        //movement(rev2|,sp1_m|sp3_s);
                        movement(rev2 | ackw_2, sp1_s | sp3_s | sp2_s, distance);
                    }
                    else
                        //movement(rev2,sp1_f|sp3_m);
                        movement(rev2 | ackw_2, sp1_m | sp3_f | sp2_s, distance);
                    re = 1;
                }
                //PWM(100,-100,2);
            }
            else if (a > 45)
            {
                if (re != 2)
                {
                    if (slow_flag)
                    {
                        //movement(rev2,sp1_s|sp3_m);
                        movement(rev2 | ckw_2, sp1_s | sp3_s | sp2_s, distance);
                    }
                    else
                        //movement(rev2,sp1_m|sp3_f);
                        movement(rev2 | ckw_2, sp1_f | sp3_m | sp2_s, distance);
                    re = 2;
                }
                //PWM(-100,100,2);
            }
            else
            {
                if (re != 3)
                {
                    if (slow_flag)
                    {
                        movement(rev2 | 0x0c, sp1_s | sp3_s, distance);
                    }
                    else
                        movement(rev2 | 0x0c, sp1_f | sp3_f, distance);
                    re = 3;
                }
                //PWM(100,100,2);
            }
        }
        else if (f == 3)
        {
            enablelsa3();
            // if(slow_flag)
            // 	PORTA = count2;
            //PORTA=a;
            if (junction_counter >= k && dis_cover == 1)
            {
                if (gotz12 == 1)
                    b = receive4();
                else
                    b = receive2();
                if (b <= 70)
                {
                    f = -4;
                    junction_counter = 0;
                    PORTA = junction_counter;
                    disablelsa3();
                    movement(fwd1 | 0x30, sp4_sm | sp2_sm, distance);
                    //while(1);
                    //_delay_ms(200);
                    delay(200);
                    movement(0xff, 0x55, distance);
                    //_delay_ms(200);
                    delay(200);
                    int to = 2;
                    if (gotz12 == 1)
                    {
                        b = receive4();
                        while (b == 255)
                        {
                            if (get_last_lsa(2) > 35)
                                movement(fwd1 | 0x30, sp4_sm | sp2_sm, distance);
                            else
                                movement(rev1 | 0x30, sp4_sm | sp2_sm, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(220);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                            b = receive4();
                        }
                    }
                    else
                    {
                        b = receive2();
                        prev = 255;
                        while (b == 255)
                        {
                            if (get_last_lsa(4) < 35)
                                movement(rev1 | 0x30, sp4_sm | sp2_sm, distance);
                            else
                                movement(fwd1 | 0x30, sp4_sm | sp2_sm, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(100);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                            b = receive2();
                        }
                    }
                    delay(200);
                    if (gotz12 == 1)
                        b = receive4();
                    else
                        b = receive2();
                    while (b > 55 || b < 15)
                    {
                        // if (b == 255)
                        // {
                        // 	movement(fwd1 | 0x30, sp4_s | sp2_s, distance);
                        // 	//set_direction(rev1);
                        // 	//PWM(-400,-400,1);
                        // 	//_delay_ms(320);
                        // 	delay(250);
                        // 	PORTA = 0xa;
                        // 	movement(0xff, 0, distance);
                        // }
                        //else
                        if (b < 15)
                        {
                            if (gotz12 == 1)
                                movement(fwd1 | 0x30, sp4_s | sp2_s, distance);
                            else
                                movement(rev1 | 0x30, sp4_s | sp2_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(250);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        else
                        {
                            if (gotz12 == 1)
                                movement(rev1 | 0x30, sp4_s | sp2_s, distance);
                            else
                                movement(fwd1 | 0x30, sp4_s | sp2_s, distance);
                            //set_direction(rev1);
                            //PWM(-400,-400,1);
                            //_delay_ms(320);
                            delay(320);
                            PORTA = 0xa;
                            movement(0xff, 0, distance);
                        }
                        if (gotz12 == 1)
                            b = get_last_lsa(2);
                        else
                            b = get_last_lsa(4);
                        //receive2();
                        //_delay_ms(10);
                        // a = receive2();
                        // if (a != 255)
                        // 	back = a;
                        set_last_lsa();
                    }
                    re = -1;
                    a = b = 255;
                    prev = 255;
                    /*					set_direction(0xff);*/
                    /*					_delay_ms(500);*/
                    /*					set_direction(0x00);*/
                    continue;
                }
            }
            a = receive3();
            if (a != 255)
            {
                prev = a;
                //c=1;
            }
            else if (prev != 255)
            {
                re = -1;
                if (prev >= 35)
                {
                    //clock
                    movement(rotate_cw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_cw,sp1_s);
                    //_delay_ms(60);
                    delay(60);
                    movement(0xff, 0, distance);
                }
                else
                {
                    //anti-clock
                    movement(rotate_acw, sp1_s | sp2_s | sp3_s | sp4_s, distance);
                    //movement(rotate_acw,sp1_s);
                    //_delay_ms(60);
                    delay(60);
                    movement(0xff, 0, distance);
                }
                re = -1;
            }
            //set_direction(fwd1);
            if (a > 70)
            {
                //set_direction(stop);
                if (re != 0)
                {
                    movement(0xff, 0, distance);
                    re = 0;
                }
            }
            else if (a < 25)
            {
                if (re != 1)
                {
                    if (slow_flag)
                    {
                        movement(rev1 | ackw_3, sp3_s | sp4_s | sp2_s, distance);
                        //movement(0xff,0,distance);
                        //movement(rev1,sp2_m|sp4_s);
                    }
                    else if (med_flag)
                    {
                        movement(rev1 | ackw_3, sp3_s | sp4_sm | sp2_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(rev1 | ackw_3, sp3_s | sp4_m | sp2_m, distance);
                    }
                    else
                        //movement(rev1,sp2_f|sp4_m);
                        movement(rev1 | ackw_3, sp3_s | sp4_f | sp2_m, distance);
                    re = 1;
                }
                //PWM(100,-100,1);
            }
            else if (a > 45)
            {
                if (re != 2)
                {
                    if (slow_flag)
                    {
                        movement(rev1 | ckw_3, sp3_s | sp4_s | sp2_s, distance);
                        //movement(0xff,0,distance);
                        //movement(rev1,sp2_s|sp4_m);
                    }
                    else if (med_flag)
                    {
                        movement(rev1 | ckw_3, sp3_s | sp4_sm | sp2_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(rev1 | ckw_3, sp3_s | sp4_m | sp2_m, distance);
                    }
                    else
                        movement(rev1 | ckw_3, sp3_s | sp4_m | sp2_f, distance);
                    //movement(rev1,sp2_m|sp4_f);
                    re = 2;
                }
                //PWM(-100,100,1);
            }
            else
            {
                if (re != 3)
                {
                    if (slow_flag)
                    {
                        //movement(0xff,0);
                        //movement(0xff,0,distance);
                        movement(rev1 | 0x30, sp2_s | sp4_s, distance);
                    }
                    else if (med_flag)
                    {
                        movement(rev1 | 0x30, sp2_sm | sp4_sm, distance);
                    }
                    else if (fa_flag)
                    {
                        movement(rev1 | 0x30, sp2_m | sp4_m, distance);
                    }
                    else
                    {
                        //movement(0xff,0);
                        movement(rev1 | 0x30, sp2_f | sp4_f, distance);
                    }
                    re = 3;
                }
                //PWM(100,100,1);
            }
        }
        else if (f < 0)
        {
            //throw
            if (f == -1 && inside == 0 && gotz12 != 1 && gotz21 != 1)
            {
                align_com = 1;
                align(2);
                align(4);
                align_com = 0;
                go(1);
            }
            if ((f == -1 || f == -3) && inside == 0 && get == 0)
            {
                //Adc_init();
                //int wo;
                //PORTA=0x55;
                // for(wo=0;(wo<500) && (!present());wo++);
                // _delay_ms(200);
                // for(wo=0;wo<500 && (!present()); wo++);
                // if(wo<500 && zone==1)
                // {
                // 	zone=0;
                // 	f=-1;
                // }
                // PORTA=0x05;
                // ADC_stop();
                //limit switch
                delay(500);
                int get1 = 0, get2 = 0, get3 = 0;
                if (get == 0)
                {
                    align_com = 1;
                    align(2);
                    align(4);
                    align_com = 0;
                    nano(4);
                    delay(200);
                    while (get1 < 2 && get2 < 2 && get3 < 2)
                    {
                        PORTA = (0x80 | (PIND >> 5 & 0x07));
                        if (((PIND >> 5 & (0x01)) == 0x01)) // & zone == 0)
                        {
                            get1++;
                            _delay_ms(200);
                            if ((PIND >> 5 & (0x01)) == 0x01)
                                get1++;
                            else
                                get1 = 0;
                        }
                        if ((PIND >> 7 & (0x01)) == 0x01)
                        {
                            get2++;
                            _delay_ms(200);
                            if ((PIND >> 7 & (0x01)) == 0x01)
                                get2++;
                            else
                                get2 = 0;
                        }
                        if ((PIND >> 6 & (0x01)) == 0x01)
                        {
                            get3++;
                            _delay_ms(200);
                            if ((PIND >> 6 & (0x01)) == 0x01)
                                get3++;
                            else
                                get3 = 0;
                        }
                    }
                    PORTA = 0xff;
                    if (get1 == 2)
                    {
                        last_zone=1;
                        if (zone == 1 && f == -3) //go to tz1 again
                        {
                            zone = 0;
                            f = -1;
                        }
                        else if (zone == 2 && f == -3) //its a mistake go to tz2 and take the shot
                        {
                            gotz21 = 1;
                            nano(1);
                            rotate_acw90();
                            align(3);
                            zone = 5; //1;
                            f = -2;   //-1;
                        }
                        else if (zone == 3 && f == -3) //go to tz2
                        {
                            // zone = 1;
                            // f = -1;
                            gotz21 = 1;
                            nano(1);
                            rotate_acw90();
                            align(3);
                            zone = 5; //1;
                            f = -2;   //-1;
                        }
                        nano(1);
                        //go to tz1
                    }
                    else if (get2 == 2)
                    {
                        last_zone = 2;
                        if (zone == 2 && f == -3) //go to tz2 again
                        {
                            zone = 1;
                            f = -1;
                        }
                        else if (zone == 3 && f == -3) //go to tz2 from tz3
                        {
                            zone = 1;
                            f = -1;
                            nano(2);
                        }
                        else if (zone == 1 && f == -1) //go to tz2 from tz3
                        {
                            zone = 1;
                            f = -1;
                            nano(2);
                        }
                        else if (zone != 0) //go to tz2 from tz1
                        {
                            gotz12 = 1;
                            nano(2);
                            align(4);
                            //curve1();
                            //movement(0xff, 0, 0xff);
                            delay(300);
                            rotate_acw90();
                            align(3);
                            //go(2);
                            zone = 4;//1;
                            f = -2;//-1;
                        }
                        nano(2);
                    }
                    else if (get3 == 2)
                    {
                        last_zone = 3;
                        if (zone == 3 && f == -3) //repeat tz3
                        {
                            zone = 2;
                            f = -1;
                        }
                        else if (zone == 2 && f == -3)
                        {
                            zone = 2;
                            f = -1;
                        }
                        else if (zone == 1 && f == -3) //go to tz1 again
                        {
                            zone = 0;
                            f = -1;
                        }
                        else if (zone == 0 && f == -1) //go to tz1 again
                        {
                            zone = 0;
                            f = -1;
                        }
                        else
                        {
                            zone = 2;
                            f = -1;
                        }
                        nano(3);
                    }
                    
                    get = 1;
                    _delay_ms(500);
                }
            }
            if (f == -1 && inside == 1)
            {
                DDRC = 0xff;
                PORTC = 0;
                get = 0;
                if (zone == 0)
                {
                    PORTC |= (1 << 0);
                    //PORTK = 0b01010100; //tz1
                }
                else if (zone == 1)
                {
                    PORTC |= (1 << 1);
                    //PORTK = 0b11001010; //tz2
                }
                else if (zone == 2)
                {
                    PORTC |= (1 << 2);
                    //PORTK = 0b01000110; //tz3
                }
                cli();
                // align(1);
                // align(2);
                // align(4);
                // movement(0xff, 0xff, 0xff);
                // rotate_acw90();
                align_com = 1;
                // align(1);
                // align(3);
                align(2);
                align(4);
                align(2);
                align(4);
                align_com = 0;
                nano(4);
                delay(300);
                nano(last_zone);
                delay(600);
                nano(6);
                movement(0xff, 0xff, 0xff);
                int p=0;
                while(p<6)
                {
                    PORTC |= (1 << 5);
                    _delay_ms(50);
                    PORTC &= ~(1 << 5);
                    _delay_ms(50);
                    p++;
                }
                //rotate_cw90();
                PORTC |= (1 << 4);
                //PORTC |= (1 << 3);
                //PORTC |= (1 << 3) | (1 << 4);
                _delay_ms(15);
                //PORTC |= (1 << 4);
                PORTC |= (1 << 3);
                _delay_ms(500);
                //PORTC&=~(1<<3);
                // PORTK = 0b00000100;
                // _delay_ms(1000);
                // PORTC = 0;
                PORTC &= ~(1 << 4);
                //PORTC &= ~(1 << 3);
                //PORTC|=(1<<2);
                _delay_ms(200);
                //PORTC&=~(1<<2);
                movement(0xff, 0xff, 0xff);
                rotate_cw90();
                align_com = 1;
                // align(4);
                //align(2);
                align_com = 0;
                movement(0xff, 0xff, 0xff);
                PORTC = 0;
            }
            sei();
            //while(1);
            //throw complete
            /*			if(f==-2)*/
            /*			{*/
            /*				_delay_ms(2000);*/
            /*				rotate_acw90();*/
            /*			}*/
            /*			break;*/
            _delay_ms(50);
            change();
            // PORTD|=(1<<5ch);
            // _delay_us(50);
            // PORTD&=~(1<<5);
            _delay_ms(50);
            junction_counter = 0;
        }
        set_last_lsa();
        //_delay_ms(10);
    }
}
//LSA1
ISR(INT4_vect)
{
    junction_counter++;
    v2 = 255;
    //set_direction(0xff);
    //PORTA=junction_counter;
}
//LSA2
ISR(INT5_vect)
{
    junction_counter++;
    //set_direction(0xff);
    //PORTA=junction_counter;
}
ISR(INT6_vect)
{
    junction_counter++;
    v4=0;
    //set_direction(0xff);
    //PORTA=junction_counter;
}
ISR(INT7_vect)
{
    junction_counter++;
    //set_direction(0xff);
    //PORTA=junction_counter;
}
// ISR(INT1_vect)
// {
// 	//sei();
// 	//junction_counter++;
// 	PORTA=0x55;
// 	re=-1;
// 	slow_flag=1;
// 	//movement(0xff,0xff,0xFF);
// }
ISR(TIMER3_OVF_vect)
{
    counter1++;
}
ISR(TIMER5_OVF_vect)
{
    counter2++;
}
