/*******************************************************************
    Copyright (C) 2016 cri-s (phone.cri@gmail.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by chris aka cri-s
    Please post support questions to github.com/cri-s issues.

*******************************************************************/
#include <string.h>
//#include "SoftI2CMaster.h"		// SoftI2CMaster lib 


#define timeMode	1	// 1=31.25 Khz , 2 =3.9khz
#define timeAdj		32	// 1=32	,	2 = 4  -- value for delay multiply
#define 		LINE_SZ	60

#define INIT_STRING "$VERSION OpenpPnP-Mate 0.01 [use HELP for command list]"

//========================================
// Commandds Macro, 
//----------------------------------------
//  add commands here, and then
//  add cmd_XXX(byte) function
//  cmd_XXX(byte); declaration
//  cmd_XXX(value); init if required
//  and add code to section at end of file
//////////////////////////////////////////
#define CMD(id,req,opt,help)	else if (help&&(Serial.println("\t\t" help),0)||!strcmp(line_p,#id)) if(argc<req||argc>opt) report(3); else report(cmd_##id(argc))
//////////////////////////////////////////
#define COMMANDS() 	\
	CMD(PWM,3,3,"PWM <id> <value>");	\
	CMD(ADC,2,2,"ADC <id>	");	\
	CMD(OUT,2,3,"OUT <value>\n\t	out <pin> <state>	-- 2==toggle");

#define CMD_INIT() 	\
	cmd_PWM(2);	\
	cmd_ADC(2);	\
	cmd_OUT(2);	\
	;
//////////////////////////////////////////
	byte cmd_PWM(byte);
	byte cmd_ADC(byte);
	byte cmd_OUT(byte);
//////////////////////////////////////////
//////////////////////////////////////////

word adc_raw[6][9];	// ADC average from 10 samples
word adc_med[6][9];	// ADC average from 10 samples
long adc_avg[6];	// oversampling adc variable
word adc_val[6];	// 16bit adc reusult
word adc_cnt;		// averaging counter
word pwm[6];		// pwm values
//--------------------------------------------
byte i,j,b;		// general purpose vars
word w;	

//========================================
// special macros that return 
//----------------------------------------
//  it return immediatly from actual 
//  function.
//  just return, no lonjump
//////////////////////////////////////////

char line[LINE_SZ+1];
char*line_p=line;

void report(byte);

//========================================
// arg() Return the arguments 
//----------------------------------------
//  like argv[] just use arg() instead
//  with -1 as argument, it return number 
//  of arguments, argc
//========================================
char *arg(unsigned char arg) { 
    byte i; char*s;
    for(i=0,s=line;*s;s+=strlen(s)+1) if(arg==i) break;
    if(arg==-1) return (char*) i;
    return s;
}

//========================================
// output utility functions
//========================================
void header(char*str) {
// should be smarter
	Serial.println("********************************");
	if(!str) return;
	Serial.println(str);
	header(NULL);
}

//========================================
// cmdParse()
//----------------------------------------
//  use shared char* line_p
//========================================
void cmdParse()
{   byte argc,help=0;
    fflush(stdout);
    if(!*line_p) { report(0); return; }	// OK , nothing to do

 // search for valid commands
	argc=(unsigned int)arg(-1);
	if(!strcmp(line_p,"HELP")) help++;
	if(help) 	header("	List of Supported Commands");
	if(0);
	COMMANDS()
	else if(help) header(NULL);
	else report(2);
}

//========================================
// serial_handler 
//----------------------------------------
//  core function to parse commands
//  it read on line of commands and split
//  it as list of strings terminated by 0
//========================================
void serial_handler()
{
    char c;
    static char old;

	c=Serial.read();
   if(c=='\n'||c=='\r') {
	line_p[0]=0;
	line_p[1]=0;	
        line_p = line;
        cmdParse();
        line_p = line;
	old=0;
	return;
   } 
   if(c==' ')
	if(old==c) return;	// no duplicate spaces
   else if(c<' ')  return;	// no special chars
   if(c>='a'&&c<='z') c^=' ';	// check case
	old=c;			// store last char
   if(c==' ') c=0;		// and patch it
	if(line_p==line+LINE_SZ-2); else
        *line_p++ = c;
}



void setup()
{

// SETUP PWM

	TCCR0A=_BV(COM0A1)|_BV(COM0B1)|_BV(WGM00);   // phase corrected PWM
	TCCR0B=TCCR0B&0b11111000|timeMode;  // PIN 5+6
	TCCR1B=TCCR1B&0b11111000|timeMode;  // PIN 9+10
	TCCR2B=TCCR2B&0b11111000|timeMode;  // PIN 3+11

// PORT SETUP
	PORTD=4;	// D2 is input
	PORTB=_BV(5);	// led is on D13
	PORTC=0;

	DDRD=0xff;
	DDRB=0xff;
	DDRC=0;		// ADC is input


	delay(1000*timeAdj);
	PORTB=0;	// LED = off

// INIT SERIAL 
   Serial.begin(19200);
	Serial.println(INIT_STRING);
	

// preload ADC values
	for(i=0;i<6;i++) { 
		w=analogRead(A0+i);
		for(j=0;j<9;j++) adc_med[i][j]=adc_raw[i][j]=w;
	}

	CMD_INIT();
}





void loop()
{	byte i; uint32_t t;
    	while (Serial.available()) serial_handler();	// Serial.check
	for(i=0;i<6;i++) { 
		//  otherwise do ADC 
		for(w=0,j=0;j<9;j++) w+=adc_raw[i][j];
		w+=analogRead(A0+i); w/=10;
		for(j=1;j<9;j++) adc_raw[i][j]=adc_raw[i][j-1];
		*adc_raw[i]=w;
		adc_avg[i]+=*adc_raw[i];
		if(++adc_cnt==128) { adc_cnt=0;
			for(t=0,j=0;j<9;j++) t+=adc_med[i][j];
			t+=adc_avg[i]>>1; adc_avg[i]=0; 
			for(j=1;j<9;j++) adc_raw[i][j]=adc_med[i][j-1];
			*adc_med[i]=t/10;
			adc_val[i]=adc_med[i][0];
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
/*****************************************************************************
******************************************************************************
   BEGIN OF COMMAND FUNCTIONS  
   CORE CODE IS ABOVE , COMMANDS AND RELATED UTILITY FUNCTIONS BELOW
******************************************************************************
*****************************************************************************/
//============================================================================
// VAR SECTION
//============================================================================
word output;		// out variable
uint32_t auxiliar;	// aux variable

//============================================================================
// MACRO SECTION
//============================================================================
#define ERR_RANGE	3

//============================================================================
// UTILITY SECTION
//============================================================================
         
 void report(byte id) { 
  if(!id--) { Serial.println("OK"); return; }			// 0
  Serial.print("ERROR#"); Serial.print(id); Serial.print(":\t");
  if	  (!id--) Serial.println("Syntax Error");		// 1
  else if (!id--) Serial.println("Wrong number of Arguments");	// 2
  // end of fixed number of assignment of error number
  else if (!id--) Serial.println("Argument out of Range ");	// 3
  else 	Serial.println("System Error");
 }

void set_out(word output) {
	if(output&1) digitalWrite( 4,HIGH); else digitalWrite( 4,LOW);
	if(output&2) digitalWrite( 7,HIGH); else digitalWrite( 7,LOW);
	if(output&4) digitalWrite( 8,HIGH); else digitalWrite( 8,LOW);
	if(output&8) digitalWrite(12,HIGH); else digitalWrite(12,LOW);
}

void set_aux(uint32_t aux) {
}

//============================================================================
// COMMAND SECTION
//============================================================================


//========================================
// PWM id value
//----------------------------------------
// id 	 = 0 for all or 1-6 for pwm 1-6
// value = pwm value
//========================================
byte cmd_PWM(byte i)
{ byte pin;
 static const byte pins[]={ 5,6,9,10,3,11 };
  if(i>6) return(ERR_RANGE);
  else 	if(!i) for(i=6;i--;) 	analogWrite( pins[i],pwm[i]=atoi(arg(2)));
	else  i--,		analogWrite( pins[i],pwm[i]=atoi(arg(2)));
  return 0;
}

//========================================
// ADC pin 
// ADC  0 
//----------------------------------------
// pin 	 = 0 for all or 1-6 for pwm 1-6
//========================================
byte cmd_ADC(byte i)
{ 
  i=atoi(arg(1));
  if(i>6) return(ERR_RANGE);
  if(!i) 
  	while(i<6) 	Serial.println(adc_val[i++]); 
  else 			Serial.println(adc_val[--i]);
  return 0;
}

//========================================
// OUT 	   value 
// OUT pin  0|1|2	2==toggle 
//========================================
byte cmd_OUT(byte i)
{ 
  if(i==2) { // direct output value
	output=atoi(arg(1));
  } else     {
	i=atoi(arg(1));
  	if(i>16) return ERR_RANGE;
	if(!i) w=-1; else w=1<<--i;
	i=atoi(arg(2));
	if(i>2) 	return 1;	
	if(i!=2)	output|=w; 
	if(i!=1) 	output^=w; 
  }
	set_out(output);

  return 0;
}



//========================================
// OUT 	   value 
// OUT pin  0|1|2	2==toggle 
//========================================
byte cmd_AUX(byte i)
{ 
  if(i==2) { // direct output value
	auxiliar=atoi(arg(1));
  } else     {
	i=atoi(arg(1));
  	if(i>16) return ERR_RANGE;
	if(!i) w=-1; else w=1<<--i;
	i=atoi(arg(2));
	if(i>2) 	return 1;	
	if(i!=2)	auxiliar|=w; 
	if(i!=1) 	auxiliar^=w; 
  }

	set_out(auxiliar);
  return 0;
}

