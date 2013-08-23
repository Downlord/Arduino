
#define PIN_TOUCH_BUTTON_S1  A2
#define PIN_TOUCH_BUTTON_S2  A3
#define PIN_TOUCH_BUTTON_S3  A4
#define PIN_BUTTON_TOGGLE    A5
#define PIN_SWITCH_FORCE     A6 //defekt
#define PIN_SWITCH_DEBUG     A7//defekt

#define PIN_VSS_SENSOR       2
#define PIN_HIGH_BEAM        4

#define PIN_SERVO_LINKS      5
#define PIN_SERVO_RECHTS     9

/**
The circuit: 
 Two devices which communicate serially are needed.
 * HC-06 device's TX attached to digital pin 8, RX to pin 7
 
 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts, 
 so only the following can be used for RX: 
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
**/

#define PIN_BT_RX            11
#define PIN_BT_TX            10

#define PIN_PWM1             3
#define PIN_PWM2             6 //defekt
#define PIN_PWM3             7

#define NUM_LEDS             130

#define SERIALCOMMAND_DEBUG

#define LED_STRIP_RIMR_START 0
#define LED_STRIP_RIMR_END 20

#define LED_STRIP_BODY_START 20
#define LED_STRIP_BODY_END 80

#define LED_STRIP_RIMF_START 80
#define LED_STRIP_RIMF_END 100

#define LED_STRIP_MASK_START 100
#define LED_STRIP_MASK_END 120

#define LED_STRIP_EYE_START 120
#define LED_STRIP_EYE_END 130

typedef struct {
  char *name;
  int PINSTART;
  int PINEND;
  int rval;
  int gval;
  int bval;
  int set;
} LED_segment;

LED_segment LED_seg[]={
  {"rims_rear",    LED_STRIP_RIMR_START, LED_STRIP_RIMR_END,0,0,0,0},
  {"body",        LED_STRIP_BODY_START, LED_STRIP_BODY_END,0,0,0,0},
  {"rims_front",   LED_STRIP_RIMF_START, LED_STRIP_RIMF_END,0,0,0,0},
  {"mask",        LED_STRIP_MASK_START, LED_STRIP_MASK_END,0,0,0,0},
  {"eyes",        LED_STRIP_EYE_START, LED_STRIP_EYE_END,0,0,0,0}
};
  

char old_c_KTMode, c_KTMode='f';

int maxbright=10;

int minFreq=650;
int maxFreq=2400;
int minFreqTest,maxFreqTest;
int minFreqTestVal,maxFreqTestVal;

volatile int HallSensorTicks = 0; 
int Speed,lastSpeed;
int ServoPos = 0;
boolean printDEBUG=false;

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h> 
#include <string.h>   // String function definitions 

#include <MeetAndroid.h>
#include <SoftwareSerial.h>
MeetAndroid meetAndroid(PIN_BT_RX, PIN_BT_TX,9600); 


/** `========================================================================================================================================
 **
 **                                  H a r d w a r e   B u t t o n s
 **
 ** `========================================================================================================================================
 **/
#include <Button.h>

#define BUT_COUNT 4
  
Button But[BUT_COUNT] = { Button(A2, LOW),
                          Button(A3, LOW),
                          Button(A4, LOW),
                          Button(A5, LOW),
                        };
byte lastButStatus[BUT_COUNT]={'r','r','r','r'};
byte ButStatus[BUT_COUNT]={'r','r','r','r'};

#include <Servo.h>
Servo myservo_links; // erzeugt ein Servo-Objekt
Servo myservo_rechts; // erzeugt ein Servo-Objekt

//#include <Cmd.h>
#include <SerialCommand.h>
SerialCommand sCmd;

unsigned long time, Tlocked, lastPulseRead, nextUpdate;
boolean  attachedAndroid,attachedServos,attachedSerial,attachedLEDFunctions = false;
int ledMode=1;

void rpm ()      //This is the function that the interupt calls 
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
  printf_P(PSTR("new mode from: %c to %c\n"),old_c_KTMode,c_KTMode);
}

#include <FastSPI_LED2.h>
CRGB leds[NUM_LEDS];


void print_debug(){
  int i;
  printf_P(PSTR("Speed [%d], ServoPosL [%d], ServoPosR [%d], c_KTMode [%c], LedMode [%d]\n"),Speed,myservo_links.read(),myservo_rechts.read(),c_KTMode,ledMode);
}
#include "Stream.h"
#define ByteBufferLenght 128
int ma_bufferCount;
char ma_buffer[ByteBufferLenght];

static FILE uartout = {0} ;

// create a output function
// This works because Serial.write, although of
// type virtual, already exists.
static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c) ;  Serial.flush();

    if(ma_bufferCount >= ByteBufferLenght)     ma_bufferCount=0;  

    ma_buffer[ma_bufferCount++]=c;
    
    if(c ==  '\n'){
      ma_buffer[ma_bufferCount++]='\0';
      meetAndroid.send((char*)ma_buffer);
      ma_bufferCount=0; delete[] ma_buffer;
    }
    return 0 ;
}


/** `========================================================================================================================================
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 **
 **                                                E E P R O M 
 **
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 **/
#include <EEPROM.h>
 
int eepromReadInt(int address){
   int value = 0x0000;
   value = value | (EEPROM.read(address) << 8);
   value = value | EEPROM.read(address+1);
   return value;
}
 
void eepromWriteInt(int address, int value){
//  while (!eeprom_is_ready());
   cli();
   EEPROM.write(address, (value >> 8) & 0xFF );
   EEPROM.write(address+1, value & 0xFF);
   sei();
}
 
void saveSettings(void){
  if(eepromReadInt(0) != ServoPos) eepromWriteInt(0,ServoPos);
}

void loadSettings(void){
//  while (!eeprom_is_ready());
  ServoPos=eepromReadInt(0);
}



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
int ledsX[NUM_LEDS][3]; //-ARRAY FOR COPYING WHATS IN THE LED STRIP CURRENTLY (FOR CELL-AUTOMATA, ETC)

//-PERISTENT VARS
int idex = 0;        //-LED INDEX (0 to NUM_LEDS-1
int idx_offset = 0;  //-OFFSET INDEX (BOTTOM LED TO ZERO WHEN LOOP IS TURNED/DOESN'T REALLY WORK)

const int dblflashPattern[][30] PROGMEM = { 
    {128,0,0,1},
    {255,0,0,1},
    {255,0,128,1},
    {255,0,255,1},
    {255,255,255,30},
    {255,0,255,1},
    {255,0,128,1},
    {255,0,0,1},
    {128,0,0,1},
    {0,0,0,130},

    {128,0,0,1},
    {255,0,0,1},
    {255,0,128,1},
    {255,0,255,1},
    {255,255,255,30},
    {255,0,255,1},
    {255,0,128,1},
    {255,0,0,1},
    {128,0,0,1},
    {0,0,0,130},

    {128,0,0,1},
    {255,0,0,1},
    {255,0,128,1},
    {255,0,255,1},
    {255,255,255,30},
    {255,0,255,1},
    {255,0,128,1},
    {255,0,0,1},
    {128,0,0,1},
    {0,0,0,800},
  };    

/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}


//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void setColor(int pix, uint32_t c){
  leds[pix].r = (c >> 16) & 0xFF;
  leds[pix].g = (c >> 8) & 0xFF;
  leds[pix].b = c & 0xFF;
}

//-SET THE COLOR OF A SINGLE RGB LED
void set_color_led(int adex, int cred, int cgrn, int cblu) {  
  int bdex;
  
  if (idx_offset > 0) {  //-APPLY INDEX OFFSET 
    bdex = (adex + idx_offset) % NUM_LEDS;
  }
  else {bdex = adex;}
  leds[idx_offset].setRGB( cred, cgrn, cblu);
}

void all_off() { //-SET ALL LEDS TO ONE COLOR
    for(int i = 0 ; i < NUM_LEDS; i++ ) {
        leds[i].setRGB( 0,0,0);
    }  
    LEDS.show();       
    delay(15);
    ledMode=0;
}

void one_color_all(int cred, int cgrn, int cblu) { //-SET ALL LEDS TO ONE COLOR

    for(int i = 0 ; i < NUM_LEDS; i++ ) {
      leds[i].setRGB(cred, cgrn, cblu);
    }  
    LEDS.show();       
    delay(15);
}


//------------------------LED EFFECT FUNCTIONS------------------------

void LED_set(char *part,int rval, int gval, int bval=0,int force_update=0){
  for (int i=0;i<5;i++){
    //printf_P(PSTR("LEDset: matching %s with %s\n"),part,LED_seg[i].name);
    
    if(strcmp(LED_seg[i].name,part)==0){
      //printf_P(PSTR("LEDset: match, setting %s to %d,%d,%d\n"),LED_seg[i].name,rval,gval,bval);
      for( int idx = LED_seg[i].PINSTART ; idx < LED_seg[i].PINEND; idx++ ) {
        leds[idx].setRGB( rval,gval,bval);
      }
      LED_seg[i].set=1;
      if(rval==0 && gval==0 && bval==0) LED_seg[i].set=0;
      if(force_update==1) LEDS.show(); 
    }
  }
}


void breathe(){
  int bval = map((exp(sin((millis()-3500)/1000.0*25/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
  LED_set("body",0,0,bval);
  
  int eval = map((exp(sin((millis()-3500)/1000.0*(55)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
  if(ServoPos<=10) LED_set("eyes",eval,0,0);
  else if(ServoPos>=170) LED_set("eyes",0,eval,0);
  
  int mval = map((exp(sin((millis()-3500)/1000.0*(85)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
  LED_set("mask",0,0,mval);

  int rval=map((exp(sin((millis()-3500)/1000.0*(75)/60*PI)) - 0.36787944)*107.38,0,255,0,maxbright);
  LED_set("rims_front",rval/3,rval/3,rval/3);
  LED_set("rims_back",rval/3,rval/3,rval/3);

  flashing();
  LEDS.show(); 
}

void flashing(){
  int i,factor=1;
  for(int flaeschess=10;flaeschess>0;flaeschess--){
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

void dblflash() {
  if(time > nextUpdate){
    printf_P(PSTR("LED: dblflash\n"));
    nextUpdate=millis()+1000;
  }
  
  for(int cyc=0; cyc<31; cyc++){
    //pl("cyc: %d, (%d,%d,%d) wait: %d",dblflashPattern[cyc][0], dblflashPattern[cyc][1], dblflashPattern[cyc][2], dblflashPattern[cyc][3]);
    for(int x=0; x<(NUM_LEDS); x++){
      leds[x].setRGB(pgm_read_byte(&dblflashPattern[cyc][0]), pgm_read_byte(&dblflashPattern[cyc][1]),pgm_read_byte(&dblflashPattern[cyc][2]));
    }
    LEDS.show();
    delay(pgm_read_byte(&dblflashPattern[cyc][3]));
  }  
}


void rainbow(uint8_t wait) {
  if(time > nextUpdate){
    printf_P(PSTR("LED: rainbow\n"));
    nextUpdate=millis()+1000;
  }
  int i, j;
   
  for (j=0; j < 256; j++) {     // 3 cycles of all 256 colors in the wheel
    for (i=0; i < NUM_LEDS; i++) {
      setColor(i, Wheel( (i + j) % 255));
    }  
    LEDS.show();   // write all the pixels out
    delay(wait);
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(uint8_t wait) {
  if(time > nextUpdate){
    printf_P(PSTR("LED: rainbowCycle\n"));
    nextUpdate=millis()+1000;
  }

  int i, j;
  
  for (j=0; j < 256 * 5; j++) {     // 5 cycles of all 25 colors in the wheel
    for (i=0; i < NUM_LEDS; i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      setColor(i, Wheel( ((i * 256 / NUM_LEDS) + j) % 256) );
    }  
    LEDS.show();   // write all the pixels out
    delay(wait);
  }
}



void antialisedPoint(int r, int g, int b, int step, int dscale, int sleep)
{
  int screenOffset = int(1.0/(step*dscale/100))+1;
  for (int j=-screenOffset; j<(NUM_LEDS/step + screenOffset);j++){
    for (int i=0;i<NUM_LEDS;i++){
      int delta = 1-abs(i-j*step)/1000*dscale;
      if(delta<0) delta=0;
      leds[i].setRGB( (delta*r), (delta*g), (delta*b));
      //printf_P(PSTR("LED: set Nr %d to (%d,%d,%d)\n"), i, (delta*r), (delta*g), (delta*b));
    }
    LEDS.show();      
  }
}

signed int larson_pos=0;
signed int larson_dir=1;

// "Larson scanner" = Cylon/KITT bouncing light effect
void larson_scanner() {
  int i, j;
  int r=0,  g=0, b=maxbright, wait=0;

  //if(printDEBUG==1) printf_P(PSTR("LED: NEW larson pos is now %d, dir is now %d\n"),larson_pos,larson_dir);
  for(i=0; i<((NUM_LEDS-1) ); i++) {
    //if(printDEBUG==1) printf_P(PSTR("LED: FRESH1 larson pos is now %d\n"),larson_pos);
    // Draw 5 pixels centered on pos. setPixelColor() will clip
    // any pixels off the ends of the strip, no worries there.
    // we'll make the colors dimmer at the edges for a nice pulse
    // look
    leds[(larson_pos - 3)].setRGB(r/16, g/16, b/16);
    leds[(larson_pos - 2)].setRGB(r/8, g/8, b/8);
    leds[(larson_pos - 1)].setRGB(r/4, g/4, b/4);
    leds[(larson_pos)].setRGB(maxbright, maxbright, b);
    leds[(larson_pos + 1)].setRGB(r/4, g/4, b/4);
    leds[(larson_pos + 2)].setRGB(r/8, g/8, b/8);
    leds[(larson_pos + 3)].setRGB(r/16, g/16, b/16);

    int opos=(int)larson_pos;
    //if(printDEBUG==1) printf_P(PSTR("LED: FRESH2 larson pos is now %d %d\n"),larson_pos,opos);
    
    LEDS.show();
    delay(1);
    
    larson_pos=(int)opos;
    //if(printDEBUG==1) printf_P(PSTR("LED: FRESH3 larson pos is now %d %d\n"),larson_pos,opos);
    
    //delay(wait);
    // If we wanted to be sneaky we could erase just the tail end
    // pixel, but it's much easier just to erase the whole thing
    // and draw a new one next time.
    for(j=-2; j<= 2; j++)
        leds[(larson_pos + j)].setRGB(0,0,0);
    // Bounce off ends of strip

    //if(printDEBUG==1) printf_P(PSTR("LED: FRESH4 larson pos is now %d\n"),larson_pos);
    
    if(larson_dir == 1){
      //if(printDEBUG==1) printf_P(PSTR("LED: INCDEC larson dir is positive inc %d\n"),larson_pos);
      larson_pos=larson_pos+1;
      //if(printDEBUG==1) printf_P(PSTR("LED: INCDEC larson pos is now %d\n"),larson_pos);
    }else
    if(larson_dir == -1){
      //if(printDEBUG==1) printf_P(PSTR("LED: INCDEC larson dir is negative dec %d\n"),larson_pos);
      larson_pos=larson_pos-1;
      //if(printDEBUG==1) printf_P(PSTR("LED: INCDEC larson pos is now %d\n"),larson_pos);
    }
    
    //if(printDEBUG==1) printf_P(PSTR("LED: BETWEEN larson at newpos %d %d\n"),larson_pos,larson_dir);

    if(larson_pos < 0) {
      //if(printDEBUG==1) printf_P(PSTR("LED: WRAP larson turning dir positive at %d (newpos 1)\n"),larson_pos);
      larson_pos = 1;
      larson_dir = 1;
      //if(printDEBUG==1) printf_P(PSTR("LED: WRAP larson pos is now %d, dir is now %d\n"),larson_pos,larson_dir);
    } else if(larson_pos >= (NUM_LEDS-2)) {
      //if(printDEBUG==1) printf_P(PSTR("LED: WRAP larson turning dir negative at %d (newpos 128)\n"),larson_pos);
      larson_pos = 128;
      larson_dir = -1;
      //if(printDEBUG==1) printf_P(PSTR("LED: WRAP larson pos is now %d, dir is now %d\n"),larson_pos,larson_dir);
    }
  }
}
    
void rotate_ledMode(){
  ledMode++;
  printf_P(PSTR("LED: ledMode now %d\n"),ledMode);
  if(ledMode >5) ledMode=0;
}

void do_led_effects(){ 
  //printf_P(PSTR("LED: running Effect id %d\n"),ledMode);
  if (ledMode == 0) { all_off();}
  else if (ledMode == 1) {breathe();}                //---STRIP RAINBOW FADE 
  else if (ledMode == 2) {rainbow(1);}       //--- SIN WAVE BRIGHTNESS  
  else if (ledMode == 3) {rainbowCycle(1);}       //--- SIN WAVE BRIGHTNESS  
  else if (ledMode == 4) {dblflash();}       //--- SIN WAVE BRIGHTNESS  
  else if (ledMode == 5) {antialisedPoint(0,0,100,3,3,0);}
  else if (ledMode == 6) {larson_scanner();}
  else breathe();
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
void and_do_COMMAND(byte flag, byte numOfValues)
{
  uint8_t argc, i = 0;
  char *arg[30],*cmd;
  int length = meetAndroid.stringLength();
  char data[length];
  meetAndroid.getString(data);
  arg[0] = strtok(data, " ");i=1;
  do
  {
      arg[i] = strtok(NULL, " ");
      i++;
  } while ((i < 30) && (arg[i] != NULL));  
  do_command(i,arg);
}
/** `========================================================================================================================================
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 **
 **                                                SERIAL COMMANDS
 **
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 ** `========================================================================================================================================
 **/
void  sCmd_do_command(const char *comm){
  uint8_t i = 1;
  char *arg[30];
  strcpy(arg[0],comm);
  do
  {
      arg[i++] = sCmd.next();
  } while ((i < 30) && (arg[i] != NULL));   
  do_command(i,arg);
}

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

void servo_attach(){ 
  printf_P(PSTR("Exaust: attaching Servos [%d,%d]\n"),minFreq,maxFreq);

  myservo_links.attach(PIN_SERVO_LINKS, minFreq, maxFreq);
  myservo_rechts.attach(PIN_SERVO_RECHTS, minFreq, maxFreq);
  delay(100);
}

void servo_detach(){ 
  delay(250);
  myservo_links.detach();
  myservo_rechts.detach();
}

void servo_write(int pos){
  servo_attach();
  myservo_links.write(pos);
  myservo_rechts.write(pos);
  ServoPos=pos;
  printf_P(PSTR("Exaust: set to %d, pos is now (%d,%d) [%d,%d]\n"),pos,ServoPos,ServoPos,minFreq,maxFreq);
  servo_detach();
}

void move_pos(int pos) {
  if(ServoPos == pos) return;
  servo_write(pos);
  c_KTMode='m';
}

void move_close() {
  if(ServoPos == 0) return;
  servo_write(0);
}

void move_open() {
  if(ServoPos >= 179) return;
  servo_write(180);
}
void Etoggle(){
  move_pos((ServoPos==0)?179:0);
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
  if (ServoPos == 179) return; 
  // OPEN
  move_open();
}

void do_servo_freq_adjust(){
    if(minFreqTest==1){
    minFreq=500;
    c_KTMode='m';
    maxFreqTest=0;
    myservo_links.detach();
    myservo_rechts.detach();
    myservo_links.attach(PIN_SERVO_LINKS, minFreq+minFreqTestVal, maxFreq);
    myservo_rechts.attach(PIN_SERVO_RECHTS, minFreq+minFreqTestVal, maxFreq);
    delay(100);
    myservo_links.write(0);
    myservo_rechts.write(0);
    delay(250);
    printf_P(PSTR("Servo Min Freq: %d\n"),(minFreq+minFreqTestVal));
    minFreqTestVal+=10;
    if((minFreq+minFreqTestVal) > 1000) minFreqTest=0;
  }else{
    minFreqTestVal=0;
  }
    
   if(maxFreqTest==1){
     maxFreq=2400;
     c_KTMode='m';
    minFreqTest=0;
    myservo_links.detach();
    myservo_rechts.detach();
    myservo_links.attach(PIN_SERVO_LINKS, minFreq, maxFreq-maxFreqTestVal);
    myservo_rechts.attach(PIN_SERVO_RECHTS, minFreq, maxFreq-maxFreqTestVal);
    delay(100);
    myservo_links.write(180);
    myservo_rechts.write(180);
    delay(250);
    printf_P(PSTR("Servo Max Freq: %d\n"),(maxFreq-maxFreqTestVal));
    maxFreqTestVal+=10;
    if((maxFreq-maxFreqTestVal) < 1800) maxFreqTest=0;
  }    else{
    maxFreqTestVal=0;
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
  loadSettings();
  pinMode(2, INPUT); 
  attachInterrupt(0, rpm, RISING); 
  digitalWrite(A2,HIGH);
  digitalWrite(A3,HIGH);
  digitalWrite(A4,HIGH);
  digitalWrite(A5,HIGH);

  int newDebounceDelay=250;  
  for(int i=0; i<BUT_COUNT; i++){
    But[i].setDebounceDelay(newDebounceDelay);
  }
  
  delay(2000);

  Serial.begin(57600);
  // fill in the UART file descriptor with pointer to writer.
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  // The uart is the standard output device STDOUT.
  stdout = &uartout ;
   
 // LEDS.addLeds<WS2801, 11, 13, BGR, DATA_RATE_MHZ(1)>(leds, NUM_LEDS);

  meetAndroid.registerFunction(and_do_COMMAND, 'z');  
  sCmd.setDefaultHandler(sCmd_do_command);

  printf_P(PSTR("--- INIT COMPLETE ---\n"));
//  print_debug();
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

  ServoPos=((myservo_links.read()+myservo_rechts.read())/2);
  int loopDebug=0;
  
  time=millis();
  sCmd.readSerial();
  meetAndroid.receive(); // you need to keep this in your loop() to receive events    

  if ( (time-lastPulseRead) > 1000 ) {
    lastSpeed=Speed;
    lastPulseRead=time;
    Speed = (HallSensorTicks/9.09888888888888);
    if(lastSpeed != Speed){
      printf_P(PSTR("Speed: %d km/h => %d km/h\tServo %d\tHallSensorTicks %d\n"),lastSpeed,Speed,ServoPos,HallSensorTicks);
    }
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

//  if (loopDebug==1) printf_P(PSTR("-- Button react\n"));
  if(time >5000){
    for(int i=0; i<BUT_COUNT; i++){
      if(But[i].isHold()){  
        if(lastButStatus[i]!= 'H'){
          printf_P(PSTR("But[%d] hold\n"),i,lastButStatus[i]);
          if(i == 0) c_KTMode='f';
        }
        ButStatus[i]='H';
      }else
      if(But[i].onPress()){
        ButStatus[i]='P';
        if(i == 0) rotate_KTMode();
        if(i == 1) rotate_ledMode();
        if(i == 3) ledMode=1;
        if(i == 3) ledMode=0;

        printf_P(PSTR("But[%d] pressed\n"),i);
      }else
      if(But[i].onRelease()){
        ButStatus[i]='R';
        printf_P(PSTR("But[%d] released\n"),i);
      }else
      if(But[i].onDoubleClick()){
        ButStatus[i]='D';
        printf_P(PSTR("But[%d] dblclicked\n"),i);
      }    
    }
   }
  
//  if (loopDebug==1) printf_P(PSTR("-- KTMode react\n"));
  // rotate modi automatic > closed  / forced => closed
   
  if ( c_KTMode == 'f'){
     move_open();
  }else if (c_KTMode == 'a'){
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
  }else if (c_KTMode == 'c'){
     move_close();
  }

//  if (loopDebug==1)   printf_P(PSTR("-- check print debug listen\n"));
  if(time > nextUpdate){
    if(printDEBUG==true) print_debug();
    nextUpdate=millis()+5000;
  }
//  if (loopDebug==1) printf_P(PSTR("-- do LED Effects \n"));

// do_servo_freq_adjust();

  do_led_effects();
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
  
  if(strcmp(cmd,"mode")==0){
    ledMode = atoi(arg[1]);
    printf_P(PSTR("new LED mode: %d\n"),ledMode);
  }else
  if(strcmp(cmd,"ex")==0){
    if(arg[1] != NULL) {
      servo_write(atoi(arg[1]));
      c_KTMode='m';
      printf_P(PSTR("Exaust Servo pos: %d\n"),atoi(arg[1]));
    }else{
      Etoggle();
      printf_P(PSTR("toggled exaust\n"));
    }
  }else

  if(strcmp(cmd,"toggle")==0){
    if(arg[1] != NULL ){
      c_KTMode = arg[1][0];
    }else{
       rotate_KTMode();
    }    
    printf_P(PSTR("new c_KTMode: %s\n"),c_KTMode);
  }else
  if(strcmp(cmd,"bri")==0){
    if(arg[1] != NULL ){
      maxbright=atoi(arg[1]);
      printf_P(PSTR("new LED brightness: %d\n"),atoi(arg[1]));
    }
  }else

  if(strcmp(cmd,"led")==0){
    LED_set(arg[1],atoi(arg[2]),atoi(arg[3]),atoi(arg[4]));
    printf_P(PSTR("LED set: %s to %d/%d/%d\n"),arg[1],atoi(arg[2]),atoi(arg[3]),atoi(arg[4]) );
  }else

  if(strcmp(cmd,"print")==0){
    print_debug();
  }else

  if(strcmp(cmd,"mintest")==0){
    minFreqTest=!minFreqTest;
  }else

  if(strcmp(cmd,"maxtest")==0){
    maxFreqTest=!maxFreqTest;
  }else

  if(strcmp(cmd,"minf")==0){
    minFreq=atoi(arg[1]);
    printf_P(PSTR("minFreq is now: %d\n"),minFreq);
    servo_write(0);
  }else

  if(strcmp(cmd,"maxf")==0){
    maxFreq=atoi(arg[1]);
    printf_P(PSTR("maxFreq is now: %d\n"),maxFreq);
    servo_write(180);
  }else

  if(strcmp(cmd,"debug")==0){
    printDEBUG=!printDEBUG;
    printf_P(PSTR("print debug is now: %d\n"),printDEBUG);
  }else{
    printf_P(PSTR("unknown command: %s\n"),arg[0]);
    printf_P(PSTR("valid commands:\nmode,ex,toggle,bri,mintest,maxtest,minf,maxf,print,debug\n"));
  }  
}
  
