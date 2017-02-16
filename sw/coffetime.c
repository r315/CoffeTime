/**
 * @file coffetime.c
 * @brief This code is intended to controll all functions of
 *        Dolche Gusto KP2106 expresso machine
 **/

#include <htc.h>
#ifndef _16F76
__CONFIG(0x3F30); // pwm on RB2
#else
__CONFIG(0x3F1A);
#endif

#define ON			1
#define OFF			0

#define PUMP		RB2
#define HEATER 		RB3
#define CAPSULE 	RA4
#define VALVE_HOT	RA2
#define VALVE_COLD	RA1
#define NTC			RA0
#define BUTTON_ON	RB0
#define LED_GREEN	RB4
#define LED_RED		RB5

#define LED_ON		0
#define LED_OFF		1
#define LED_RED_ON     LED_RED   = LED_ON
#define LED_GREEN_ON   LED_GREEN = LED_ON
#define LED_RED_OFF    LED_RED   = LED_OFF
#define LED_GREEN_OFF  LED_GREEN = LED_OFF

#define HEATER_ON   HEATER = ON
#define PUMP_ON     PUMP   = OFF
#define HEATER_OFF  HEATER = OFF
#define PUMP_OFF    PUMP   = ON

#define EN_SENSORS	RB5 = 0
#define DIS_SENSORS	RB5 = 1

#ifdef _16F76
#define TMR0_RELOAD -183
#define PWM_PERIOD  15000    		//10ms@1.5Mhz Timer1
#define GATE_PULSE_WIDTH -20			//Turn-on time ~20us
#else
#define TMR0_RELOAD -248			//write inibits two instruction cycles
#define PWM_PERIOD  10000			//10ms@1Mhz Timer1
#endif

#define PWM_OFF_VALUE (PWM_PERIOD * 2)
#define HEATER_MAX_POWER 100
#define POWER_MULTIPLIER  (PWM_PERIOD/HEATER_MAX_POWER)
#define ONE_MINUTE  60000
#define TEN_MINUTES (10 * ONE_MINUTE)
#define AUTO_OFF_TIME TEN_MINUTES

#define TRIS_SLEEP  0xFF

typedef struct _Timer{
	unsigned long start;
	unsigned long end;
}Timer;

enum STATES{
	IDLE,
	HEATING,
	READY,
	POURING,
	SLEEP
};

bank1 unsigned long ms_ticks;
bank1 char bstate;

void PID_Init(void);
float PID_Update(float curTemp);
void NTC_Init(void);
float BOILER_getCurrentTemp(void);
char BOILER_Ready(void);

/* -------------------------------------------------------------
 * @brief interrupt vector, the timer's and compare interrupts 
 *        are precessed here
 *------------------------------------------------------------- */
void interrupt vector(void){
	if(TMR0IF){                         // timer0 clock = 250khz
		TMR0 = TMR0_RELOAD;
		ms_ticks++;	
		//PUMP ^= 1;
		TMR0IF = OFF;
	}

	if(RBIF){                           // portb missmatch flag
		TMR1H = 0;                      // reset timer and start timer
		TMR1L = 0;		                // the heater will be turned-on on 
		TMR1ON = ON;                    // timer match
		//HEATER ^= 1;
		//PUMP ^= 1;
		asm("movf 6,w");                // dummy read of portb to end missmatch condition
		RBIF = OFF;
	}	
	
	if(CCP1IF){                         // match with timer1 flag
		TMR1H = GATE_PULSE_WIDTH >>8;   // set trigger pulse width
		TMR1L = GATE_PULSE_WIDTH & 255; // the pulse is cleared on timer overflow
		HEATER_ON;                      // turn-on heater
		CCP1IF = OFF;
	}
	
	if(TMR1IF){		
		HEATER_OFF;                     // on timer overflow turn off heater
		TMR1ON = OFF;                   // and stop timer
		TMR1IF = OFF;
	}	
}
/**
 *  Time Related stuff
 **/ 
unsigned long elapsedTimer(Timer *tim){
	return ms_ticks - tim->start;    
}

void setTimer(Timer *tim, unsigned long ms){
    tim->start = ms_ticks;
	tim->end = ms;
}

char isTimerExpired(Timer *tim){
	return (elapsedTimer(tim) < tim->end)? OFF : ON;
}

/**
 *  power management
 **/ 
void setHeaterPower(char val){
unsigned int reload_value;
		
	if(!val)
		reload_value = PWM_OFF_VALUE;
	else{
		if(val > HEATER_MAX_POWER)
			val = HEATER_MAX_POWER;
		reload_value = PWM_PERIOD - (val * POWER_MULTIPLIER);
	}
	
	CCPR1H = reload_value >> 8;
	CCPR1L = reload_value & 255;	
}

void powerDown(void){
	//TRISA = TRIS_SLEEP;
	//TRISB = TRIS_SLEEP;
	setHeaterPower(0);
	//TMR1ON = OFF;	
	HEATER_OFF;	
	PUMP_OFF;
	LED_RED_OFF;
	LED_GREEN_OFF;	
}


/**
 *   misc stuff
 */
char getButtonON(void){
	if(BUTTON_ON){
		if(bstate)
			return OFF;
		bstate = ON;
	}else
		bstate = OFF;
	return bstate;
}

void testPins(void){
Timer pid_timer;
	while(ON){
		if(isTimerExpired(&pid_timer)){	
			HEATER ^= 1;	
			setTimer(&pid_timer,500);
		}
	}
}
//---------------------------------------------------
//
//---------------------------------------------------
void main(void)
{ 
unsigned char state = IDLE;
Timer state_timer, pid_timer, auto_off;
#ifndef _16F76 
/* 8mhz internal oscillator */
	OSCCON = 0x70;
	while(!IOFS);
#endif
/* configure port pins*/
	TRISA = 0b11111111;
	TRISB = 0b11000001;
	
	EN_SENSORS;	
	LED_RED_OFF;
	LED_GREEN_OFF;
	HEATER_OFF;
	PUMP_OFF;	
	
/* Configure Timer0 for 1ms interrupts */
	ms_ticks = 0;
#ifndef _16F76
	OPTION = 0x50 | 2 ;  /* assign prescaler to TMR0 rate 1:8 no pull-up*/
#else
	OPTION = 0xD0 | 3 ;   /* for 12Mhz XTAL */
#endif
	TMR0IE = ON;
		
/* Configure Timer1 for generating PWM for HEATER */	
#ifndef _16F76
	T1CON = (1<<4);     /* 1:2 prescaler => 1Mhz(1us) clock*/
#else
	T1CON = (1<<4);     /* 1:2 prescaler => 1500Khz(666ns) clock*/
#endif
	CCP1CON = 0x0A;     /* Compare mode, generate interrupt on match */	
	TMR1IE = ON;
	CCP1IE = ON;
	TMR1ON = OFF;
	
/* Configure PORTB input change intr */		
	RBIE = ON;

	PEIE = ON;
	GIE = ON;	
	
	setHeaterPower(0);
	
	PID_Init();
	NTC_Init();	
	
	//testPins();

	while(1){
		if(isTimerExpired(&state_timer)){
			switch(state){		
				case IDLE:				
					if(getButtonON()){
						state = HEATING;
						setHeaterPower(100);
						setTimer(&auto_off, AUTO_OFF_TIME);
						break;
					}				
					break;
					
				case HEATING:				
					if(getButtonON()){
						state = IDLE;
						powerDown();
						break;
					}
					
					if(isTimerExpired(&pid_timer)){
						if(BOILER_Ready()){							
							if(CAPSULE && !(VALVE_HOT | VALVE_COLD)){ // avoid pouring if handle is accidently on
								state = READY;
								LED_GREEN_ON;
								LED_RED_OFF;
								break;
							}
							LED_RED_ON;
						}else{
							LED_RED ^= 1;
						}
						setTimer(&pid_timer,500);
					}
					break;
					
				case READY:	
					if(getButtonON() || isTimerExpired(&auto_off)){
						state = IDLE;
						powerDown();
						break;
					}
					
					if(BOILER_Ready()){
						if(CAPSULE){
							LED_GREEN_ON;
							LED_RED_OFF;
							if(VALVE_HOT || VALVE_COLD){
								state = POURING;
								PUMP_ON;
								LED_GREEN_OFF;
								LED_RED_ON;
								break;
							}
						}else{						
							LED_GREEN_OFF;
							LED_RED_ON;
						}
					}else {	
						state = HEATING;
						LED_GREEN_OFF;
						LED_RED_ON;
						break;
					}
					break;
			
			    case POURING:
					if( !(VALVE_HOT | VALVE_COLD) ){
						state = READY;
						PUMP_OFF;
						LED_RED_OFF;
						LED_GREEN_ON;
						break;						
					}
			        break;
				default: 
					break;
			}
			setTimer(&state_timer, 10);  //100Hz update
		}
	
	}
}
