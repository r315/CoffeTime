/**
 * @file pid.c
 * @brief  PID control code
 *         
 *         This module implements a PID control loop, the pid is
 *         initialize with p,i,d values from eeprom
 *
 *         the pid code was written by Tim Tim Hirzel based on Tim Wescott pid algorithm.
 *         http://playground.arduino.cc/Main/BarebonesPIDForEspresso#pid
 *
 *         Creative Commons Attribution-Noncommercial-Share Alike 3.0
 +
 * 		   Hugo Reis
 *         Febuary 2017
 **/


#ifdef __PICC__
	#include <htc.h>
	#ifdef _16F76
		#define eeprom_read(a) 0      //implement this
		#define eeprom_write(a,d)  //implement this
	#endif
#else
	#define bank1
	#define eeprom_read(a)  0     //implement this
	#define eeprom_write(a,d)  0  //implement this
#endif

#define ADDR_PGAIN 0
#define ADDR_IGAIN ADDR_PGAIN + sizeof(float)
#define ADDR_DGAIN ADDR_IGAIN + sizeof(float)
#define ADDR_TARGET_TEMP ADDR_DGAIN + sizeof(float)

#define WINDUP_GUARD_GAIN 100.0

typedef struct _pid{
	float pgain, igain, dgain;
	float targetTemp;
	float pTerm, iTerm, dTerm;	
	float iState;
	float lastTemp;	
}Pid;

bank1 Pid pid;

void PID_Init(void) {
	pid.pgain = eeprom_read(ADDR_PGAIN);
	pid.igain = eeprom_read(ADDR_IGAIN);
	pid.dgain = eeprom_read(ADDR_DGAIN);
	pid.targetTemp = eeprom_read(ADDR_TARGET_TEMP);
	pid.iState = 0;
	pid.lastTemp = 0;	
}

void PID_Save(void){
char *p,i;
	p = (char*)&pid;
	for(i = 0; i < 4 * sizeof(float); i++){ //only the first 4 floats are needed to save
		eeprom_write(ADDR_PGAIN + i, *(p+i));
	}	
}

float getTargetTemp(void){ return pid.targetTemp;}
float getP(void) { return pid.pgain; }
float getI(void) { return pid.igain; }
float getD(void) { return pid.dgain; }
void setP(float p) { pid.pgain = p; }
void setI(float i) { pid.igain = i; }
void setD(float d) { pid.dgain = d; }

float PID_Update(float curTemp)
{
  // these local variables can be factored out if memory is an issue,
  // but they make it more readable
  double result;
  float error;
  float windupGaurd;

  // determine how badly we are doing
  error = pid.targetTemp - curTemp;

  // the pTerm is the view from now, the pgain judges
  // how much we care about error we are this instant.
  pid.pTerm = pid.pgain * error;

  // iState keeps changing over time; it's
  // overall "performance" over time, or accumulated error
  pid.iState += error;

  // to prevent the iTerm getting huge despite lots of
  //  error, we use a "windup guard"
  // (this happens when the machine is first turned on and
  // it cant help be cold despite its best efforts)

  // not necessary, but this makes windup guard values
  // relative to the current iGain
  windupGaurd = WINDUP_GUARD_GAIN / pid.igain;  

  if (pid.iState > windupGaurd)
    pid.iState = windupGaurd;
  else if (pid.iState < -windupGaurd)
    pid.iState = -windupGaurd;
  pid.iTerm = pid.igain * pid.iState;

  // the dTerm, the difference between the temperature now
  //  and our last reading, indicated the "speed,"
  // how quickly the temp is changing. (aka. Differential)
  pid.dTerm = (pid.dgain* (curTemp - pid.lastTemp));

  // now that we've use lastTemp, put the current temp in
  // our pocket until for the next round
  pid.lastTemp = curTemp;

  // the magic feedback bit
  return  pid.pTerm + pid.iTerm - pid.dTerm;
}
