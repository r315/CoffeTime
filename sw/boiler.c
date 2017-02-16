#include <htc.h>

#define ON 1
#define OFF 0
#define ANIN 0    //RA0

void NTC_Init(void){
	TRISA |= (1<<ANIN);
	ADCON0 = 0xC1 | (ANIN << 3);	// RCosc, ADON	
	ADCON1 = 0x0E;                  // left justified, one AN channel, refs (VDD,VSS)
}

char NTC_Read(void){
#ifndef _16F76
	GODONE = 1;			// initiate conversion on the selected channel
	while(GODONE);		
return ADRESH;          //left justified ignore 2 lsb bits
#else
	ADGO = 1;
	while(!ADGO);
	return ADRES;
#endif
}

float BOILER_getCurrentTemp(void){
	return 0;
}

char BOILER_Ready(void){
float current_temp = BOILER_getCurrentTemp();
	//boilerSetPower(PID_Update(current_temp));
	//if(current_temp >= getTargetTemp())
	//	return ON;
	//return OFF;
	return ON;
}

