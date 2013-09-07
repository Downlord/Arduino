#define PIN_VSS_SENSOR       2
#define INTERRUPT_VSS        0

#define TOUCH_SENS           3

#define PIN_SERVO_LINKS      5 // 5
#define PIN_SERVO_RECHTS     6 // 6  // 9

#define PIN_BT_RX             10 // 8
#define PIN_BT_TX             11 // 7


#define BAUD_USB              115200
#define BAUD_BT               9600


#define PIN_SPI_CLK          13
#define PIN_SPI_DATA         12

#define NUM_LEDS               130 // 130

#define LED_STRIP_RIMR_START   0
#define LED_STRIP_RIMR_END     20

#define LED_STRIP_BODY_START   20
#define LED_STRIP_BODY_END     80

#define LED_STRIP_RIMF_START   80
#define LED_STRIP_RIMF_END     100

#define LED_STRIP_MASK_START   100
#define LED_STRIP_MASK_END     120

#define LED_STRIP_EYE_START    120
#define LED_STRIP_EYE_END      130

int LEDpart[][2] = {
	{LED_STRIP_RIMR_START, LED_STRIP_RIMR_END},
  {LED_STRIP_RIMF_START, LED_STRIP_RIMF_END},
  {LED_STRIP_BODY_START, LED_STRIP_BODY_END},
  {LED_STRIP_MASK_START, LED_STRIP_MASK_END},
  {LED_STRIP_EYE_START, LED_STRIP_EYE_END}
};
unsigned int my_PIN[16]; //={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

int Speed,lastSpeed;
int ServoPos = 0,ServoPosL=0,ServoPosR=0;
unsigned int ledMode=0;
int maxbright=30;
int minFreq=1000;
int maxFreq=2000;
int minFreqTest,maxFreqTest;
int minFreqTestVal,maxFreqTestVal;
unsigned long time, Tlocked, lastPulseRead, nextUpdate, debugUntil, debugThrottle;
volatile int HallSensorTicks = 0; 

boolean printDEBUG=false;

#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <SoftwareSerialCommand.h>

SoftwareSerial SerialBT(PIN_BT_RX,PIN_BT_TX);

SerialCommand sCmd;     // The demo SerialCommand object

SoftwareSerialCommand sCmdBT;     // The demo SerialCommand object


#include <avr/pgmspace.h>

void StreamPrint_progmem(PGM_P format,...)
{ 

	// program memory version of printf - copy of format string and result share a buffer
	// so as to avoid too much memory use
	char formatString[128], *ptr;
	strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem
	// null terminate - leave last char since we might need it in worst case for result's \0
	//  strcat(formatString,LINEEND);
	formatString[ sizeof(formatString)-2 ]='\0'; 
	ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...
	va_list args;
	va_start (args,format);
	vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
	va_end (args);
	formatString[ sizeof(formatString)-1 ]='\0'; 
	Serial.println(ptr);
	SerialBT.println(ptr);
}

#define debug(format, ...) StreamPrint_progmem(PSTR(format),##__VA_ARGS__)
#define dbstream(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)


boolean counterwise=true;

char old_c_KTMode, c_KTMode='f';

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h> 
#include <string.h>   // String function definitions 


/** `========================================================================================================================================
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 **
 **                                                L  E D 
 **
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 **/
 

#include <FastSPI_LED2.h>
CRGB leds[NUM_LEDS];

void all_off() { //-SET ALL LEDS TO ONE COLOR
    for(int i = 0 ; i < NUM_LEDS; i++ ) {
        leds[i].setRGB( 0,0,0);
    }  
    LEDS.show();       
    delay(15);
}

void one_color_all(int cred, int cgrn, int cblu) { //-SET ALL LEDS TO ONE COLOR

    for(int i = 0 ; i < NUM_LEDS; i++ ) {
      leds[i].setRGB(cred, cgrn, cblu);
    }  
    LEDS.show();       
    delay(15);
}

//------------------------LED EFFECT FUNCTIONS------------------------

void breathe(){
	//rims back
	int time=millis();
	for( int idx = LEDpart[0][0] ; idx < LEDpart[0][1]; idx++ ) {
	  int rval=map((exp(sin(((millis()+(idx*1000))-3500)/1000.0*(75)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
    if(my_PIN[1] == true) leds[idx].setRGB( rval,rval,rval);
		else leds[idx].setRGB( 0,0,0 );
  } 
	//rims front
  for( int idx = LEDpart[1][0] ; idx < LEDpart[1][1]; idx++ ) {
	  int rval=map((exp(sin(((millis()+(idx*1000))-3500)/1000.0*(75)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
    if(my_PIN[2] == true) leds[idx].setRGB( rval,rval,rval);
		else leds[idx].setRGB( 0,0,0 );
  }

	//body
  for( int idx = LEDpart[2][0] ; idx < LEDpart[2][1]; idx++ ) {
		int bval = map((exp(sin((millis()-3500)/1000.0*25/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
    if(my_PIN[3] == true) leds[idx].setRGB( 0,0,bval);
		else leds[idx].setRGB( 0,0,0 );
  }
  
	//mask
	for( int idx = LEDpart[3][0] ; idx < LEDpart[3][1]; idx++ ) {
		int mval = map((exp(sin((millis()-3500)/1000.0*(85)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
		if(my_PIN[4] == true) leds[idx].setRGB( 0,0,mval);
		else leds[idx].setRGB( 0,0,0 );
  }

	// eyes
	for( int idx = LEDpart[4][0] ; idx < LEDpart[4][1]; idx++ ) {
		int eval = map((exp(sin((millis()-3500)/1000.0*(55)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
		int rval = eval*170/maxbright;
		int gval = eval*43/maxbright;
		int bval = eval;
		if(my_PIN[5] == true) leds[idx].setRGB( rval,gval,bval);
		else leds[idx].setRGB( 0,0,0 );
  } 
  
	if(time > nextUpdate){
		//debug("PINS %d %d %d %d %d %d",my_PIN[1],my_PIN[2],my_PIN[3],my_PIN[4],my_PIN[5],my_PIN[6]);
		nextUpdate=time+2000;
	}
  if(my_PIN[6] == true) flashing();
  LEDS.show(); 
}

void flashing(){
  int i,factor=1;
  for(int flaeschess=100;flaeschess>0;flaeschess--){
    if(random(500/factor)==0){
      factor=10;
      int ovalr=leds[LED_STRIP_EYE_START].r;
      int ovalg=leds[LED_STRIP_EYE_START].g;
      int ovalb=leds[LED_STRIP_EYE_START].b;  
      for( i = LED_STRIP_EYE_START ; i < LED_STRIP_EYE_END; i++ ) {
        leds[i].setRGB( maxbright/3,maxbright/3,maxbright/3);    
      }
      LEDS.show(); 
      delay(random(7));
      for( i = LED_STRIP_EYE_START ; i < LED_STRIP_EYE_END; i++ ) {
        leds[i].setRGB( ovalr,ovalg,ovalb);
      }
      LEDS.show(); 
      delay(random(5));
    }
  }
}


long colorchange;
int colorstep;
void do_led_effects(){ 
  breathe();
	/**
  for( int idx = 0 ; idx < 9; idx++ ) {
    int bval = map((exp(sin((millis()+(1000/14*idx)-3500)/1000.0*(85)/60*PI)) - 0.36787944)*10.38,0,255,0,maxbright);
      if(millis()-5000 > colorchange) {
        if(bval==0){
          colorchange=millis();colorstep++;
        }
      }
      if(colorstep>1) colorstep=0;
      if(colorstep<1){
        leds[idx].setRGB( bval,0,0);
      }else{
        leds[idx].setRGB( 0,0,bval);
      }
   }**/
   LEDS.show(); 
   delay(random(5));


}

/** `========================================================================================================================================
**
**                                  H a r d w a r e   B u t t o n s
**
** `========================================================================================================================================
**/
#include <Button.h>

#define BUT_COUNT 1

byte lastButStatus[BUT_COUNT]={'r'};
byte ButStatus[BUT_COUNT]={'r'};

Button But[BUT_COUNT] = { Button(TOUCH_SENS, LOW), };


/** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**
**                                                S E R V O S
**
** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**/

#include <Servo.h>

Servo myservo_links; // erzeugt ein Servo-Objekt
Servo myservo_rechts; // erzeugt ein Servo-Objekt

int specify_angular_speed=210; // msec per 60°

void servo_attach(){
	myservo_links.attach(PIN_SERVO_LINKS, minFreq, maxFreq);
	myservo_rechts.attach(PIN_SERVO_RECHTS, minFreq, maxFreq);
}

void servo_detach(){
	myservo_links.detach();
	myservo_rechts.detach();
}

void getCurrPos(){
	ServoPos=myservo_links.read();
	ServoPosL=ServoPos;
	ServoPosL=ServoPos;
}

void servo_write(int pos){

	servo_attach();
	getCurrPos();  
	int myway=ServoPos-pos;
	int distance=abs(myway);
	//debug("di %d",distance);
	
	int mydelay=abs(200+(specify_angular_speed/60*distance));
	//debug("D %d",mydelay);
	delay(50);
	//if(pos > 160) pos=180;
	//else if(pos < 10) pos=5;
	myservo_links.write(pos);
	myservo_rechts.write(pos);
	delay(mydelay); // 0,11 ms/10 degrees
	ServoPos=pos;
	ServoPosL=pos;
	ServoPosR=(pos);
	debug("*S %d",ServoPos);
	//debug("#S %d\n",ServoPos);
	//	debug("#SL %d#",ServoPosL);
	//	debug("#SR %d#",ServoPosR);
	delay(50);
	servo_detach();
	//delay(150);
	//debug("moved Servos\n",ServoPos);
}

void move_pos(int pos) {
	getCurrPos();
	if(ServoPos == pos) return;
	servo_write(pos);
	c_KTMode='m';
}

void move_close() {
	if(ServoPos <= 90) return;
	servo_write(0);
}

void move_open() {
	if(ServoPos >= 90) return;
	servo_write(180);
}

void Etoggle(){
	int newpos = (ServoPos > 90)?0:180;	
	move_pos(newpos);
}

void Eclose(){
	if (ServoPos == 0) return;
	if (millis() <= Tlocked) {
		return;
	}
	//CLOSE
	move_close();
}

void Eopen(){
	Tlocked=millis()+3000;
	if (ServoPos >= 90) return;
	// OPEN
	move_open();
}

/** `========================================================================================================================================
**
**                                  helper
**
** `========================================================================================================================================
**/

void rpm()      //This is the function that the interupt calls 
{ 
	++HallSensorTicks; 
}

void rotate_KTMode(){
	old_c_KTMode=c_KTMode;
	if(c_KTMode == 'a'){
		c_KTMode='c';
		move_close();
	}else if (c_KTMode == 'c'){
		c_KTMode='a';  
	}else if (c_KTMode == 'm'){
		c_KTMode='c';
		move_close();
	}else if (c_KTMode == 'f'){
		c_KTMode='c';
		move_close();
	}
	//debug("--- mode now %s ---\n",c_KTMode);
}

/** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**
**                                                 A N D R O I D
**
** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**/

void  sCmd_do_command(const char *comm){
	uint8_t i = 1;
	char *arg[30];
	strcpy(arg[0],comm);i=1;
	do
	{
		arg[i++] = sCmd.next();
	} while ((i < 30) && (arg[i] != NULL));  
	//sCmd.clearBuffer();
	//printf_P("running %s",arg[0]);
	do_command(i,arg);
}

void  sCmdBT_do_command(const char *comm){
	uint8_t i = 1;
	char *arg[30];
	strcpy(arg[0],comm);i=1;
	do
	{
		arg[i++] = sCmdBT.next();
	} while ((i < 30) && (arg[i] != NULL));  
	//sCmd.clearBuffer();
	//printf_P("running %s",arg[0]);
	do_command(i,arg);
}

/** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**
**                                                 C O M M A N D S
**
** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**/

void do_command(int argC,char **arg){
	int i=0;
	char *cmd=arg[0];


	if(strcmp(cmd,"ex")==0){
		if(arg[1] != NULL) {
			servo_write(atoi(arg[1]));
			c_KTMode='m';
		}else{
			Etoggle();
		}
	}else

		
		if(strcmp(cmd,"PIN")==0){
			if(arg[1] != NULL && arg[2] != NULL){
				my_PIN[atoi(arg[1])]=(boolean)atoi(arg[2]);
				debug("*P %s %s",arg[1],arg[2]);
//				debug("set my_PIN %s to %s",arg[1],arg[2]);
			}else{
				my_PIN[atoi(arg[1])]=!my_PIN[atoi(arg[1])];
//				debug("set my_PIN to ?, toggeling");				
				debug("*P %s %s",arg[1],my_PIN[atoi(arg[1])]);
			}
		}else

		if(strcmp(cmd,"print")==0){
			debug("Hello World");
		}else

			if(strcmp(cmd,"*INIT*")==0){
				debug("*L"); 
				for(int idx=1;idx<13;idx++){
					debug("*P %d %d",idx,my_PIN[idx]);    
				}
				debug("*SPD %d",Speed);    
				debug("*S %d",ServoPos);    
				debug("Android connected over BT");
			}else

				if(strcmp(cmd,"minf")==0){
					minFreq=atoi(arg[1]);
					servo_write(0);
				}else

					if(strcmp(cmd,"maxf")==0){
						maxFreq=atoi(arg[1]);
						servo_write(180);
					}else{
						debug("unknown command:  %s",cmd);
						//sCmdBT.clearBuffer();
					}  
}

/** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**
**                                                S E T U P
**
** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**/
void setup() 
{ 

	sCmd.setDefaultHandler(sCmd_do_command);
	sCmdBT.setDefaultHandler(sCmdBT_do_command);

	myservo_links.write(180);
	myservo_rechts.write(0);

	pinMode(PIN_VSS_SENSOR, INPUT); 
	attachInterrupt(INTERRUPT_VSS, rpm, RISING); 

	pinMode(TOUCH_SENS,INPUT_PULLUP);


	int newDebounceDelay=150;  
	for(int i=0; i<BUT_COUNT; i++){
		But[i].setDebounceDelay(newDebounceDelay);
	}

	LEDS.addLeds<WS2801, PIN_SPI_CLK, PIN_SPI_DATA, BGR, DATA_RATE_MHZ(1)>(leds, NUM_LEDS);

	Serial.begin(BAUD_USB);
	SerialBT.begin(9600);  


} 


/** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**
**                                                L  O  O  P
**
** `========================================================================================================================================
** `========================================================================================================================================
** `========================================================================================================================================
**/
void loop () 
{
	time=millis();

	sCmd.readSerial();
	sCmdBT.readSerial(&SerialBT); 

	if ( (time-lastPulseRead) > 1000 ) {
		lastSpeed=Speed;
		lastPulseRead=time;
		Speed = (HallSensorTicks/9.09888888888888);
		if(Speed!=lastSpeed) debug("*SPD %d",Speed);

		HallSensorTicks=0;
	}

	for(int i=0; i<BUT_COUNT; i++){
		lastButStatus[i]=ButStatus[i];
		if(lastButStatus[i]== 'H') ButStatus[i]='h';
		if(lastButStatus[i]== 'P') ButStatus[i]='p';
		if(lastButStatus[i]== 'R') ButStatus[i]='r';
		if(lastButStatus[i]== 'D') ButStatus[i]='d';
	}
	for(int i=0; i<BUT_COUNT; i++){
		But[i].listen();
	}

	if(time >5000){
		for(int i=0; i<BUT_COUNT; i++){
			if(But[i].isHold()){  
				if(lastButStatus[i]!= 'H') c_KTMode='f';
				ButStatus[i]='H';
			}else
				if(But[i].onPress()){
					if(lastButStatus[i]!= 'P') rotate_KTMode();  
					ButStatus[i]='P';
				}else
					if(But[i].onRelease()){
						ButStatus[i]='R';
					}else
						if(But[i].onDoubleClick()){
							ButStatus[i]='D';
						}    
		}
	}



	if ( c_KTMode == 'f'){
		move_open();
	}else if (c_KTMode == 'a'){
		if(lastSpeed!=Speed){
			switch (Speed) {
			case 0 ... 1:
				Eclose();          
				break;
			case 45 ... 65:
				Eclose();
				break;
			default:
				Eopen();        
				break;
			}
		}
	}else if (c_KTMode == 'c'){
		move_close();
	}else{
		// 'm'
	}
	if(time > nextUpdate){
//		debug("*SPD %d",Speed);
		nextUpdate=time+2000;
	}
	do_led_effects();
}



