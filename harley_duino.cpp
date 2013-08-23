#define MeetAndroid_USE

#define PIN_VSS_SENSOR       2

#define PIN_SERVO_LINKS      5
#define PIN_SERVO_RECHTS     6


#define PIN_BT_RX            11 // 8
#define PIN_BT_TX            10 // 7

#define BAUD_USB              57600
#define BAUD_BT               9600

char old_c_KTMode, c_KTMode='f';

int minFreq=650;
int maxFreq=2400;
int minFreqTest,maxFreqTest;
int minFreqTestVal,maxFreqTestVal;
unsigned long time, Tlocked, lastPulseRead, nextUpdate;

volatile int HallSensorTicks = 0; 
int Speed,lastSpeed;
int ServoPos = 0;
boolean printDEBUG=false;

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h> 
#include <string.h>   // String function definitions 

/** `========================================================================================================================================
 **
 **                                  M e e t A n d r o i d
 **
 ** `========================================================================================================================================
 **/
#ifdef MeetAndroid_USE
#include <MeetAndroid.h>
#include <SoftwareSerial.h>

MeetAndroid meetAndroid(PIN_BT_RX, PIN_BT_TX,BAUD_BT); 
#endif
/** `========================================================================================================================================
 **
 **                                  debug
 **
 ** `========================================================================================================================================
 **/

#include <avr/pgmspace.h>
  
void StreamPrint_progmem(PGM_P format,...)
{
  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[128], *ptr;
  strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem
  // null terminate - leave last char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString)-2 ]='\0'; 
  ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...
  va_list args;
  va_start (args,format);
  vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString)-1 ]='\0'; 
  Serial.print(ptr); Serial.flush();
#ifdef MeetAndroid_h
meetAndroid.send(ptr);
#endif
}
 
#define debug(format, ...) StreamPrint_progmem(PSTR(format),##__VA_ARGS__)
#define debugstream(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)



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


/** `========================================================================================================================================
 **
 **                                  Serial Command
 **
 ** `========================================================================================================================================
 **/
#include <SerialCommand.h>
SerialCommand sCmd;

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


void servo_attach(){ 
  debug(("Exaust: attaching Servos [%d,%d]\n"),minFreq,maxFreq);

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
  debug(("Exaust: set to %d, pos is now (%d,%d) [%d,%d]\n"),pos,ServoPos,ServoPos,minFreq,maxFreq);
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


/** `========================================================================================================================================
 **
 **                                  helper
 **
 ** `========================================================================================================================================
 **/

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
  debug(("new mode from: %c to %c\n"),old_c_KTMode,c_KTMode);
}

void print_debug(){
  int i;
  debug(("Speed [%d], ServoPosL [%d], ServoPosR [%d], c_KTMode [%c]\n"),Speed,myservo_links.read(),myservo_rechts.read(),c_KTMode);
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
#ifdef MeetAndroid_h 
void and_do_COMMAND(byte flag, byte numOfValues)
{
  uint8_t argc, i = 0;
  char *arg[30],*cmd;
  int length = meetAndroid.stringLength();
  char data[length];
  meetAndroid.getString(data);
  arg[0] = strtok(data, " ");i=1;
  if(printDEBUG==true) debug("Andr: got %s\n",arg[0]);
  do
  {
      arg[i] = strtok(NULL, " ");
      if(printDEBUG==true) debug("Andr: found argument %s at %d\n",arg[i],i);
  } while ((i < 30) && (arg[i++] != NULL));  
  do_command(i,arg);
}
#endif
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
#ifdef SerialCommand_h 
void  sCmd_do_command(const char *comm){
  uint8_t i = 1;
  char *arg[30];
  strcpy(arg[0],comm);i=1;
  if(printDEBUG==true) debug("sCmd: got %s\n",comm);
  do
  {
      arg[i] = sCmd.next();
      if(printDEBUG==true) debug("sCmd: found argument %s at %d\n",arg[i],i);
  } while ((i < 30) && (arg[i++] != NULL));   
  do_command(i,arg);
}
#endif

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
  
  if(printDEBUG==true) debug(("try to guess command: %s (%s,%s,%s)\n"),arg[0],arg[1],arg[2],arg[3]);
  
  if(strcmp(cmd,"mode")==0){
    int ledMode = atoi(arg[1]);
    debug(("new LED mode: %d\n"),ledMode);
  }else
  if(strcmp(cmd,"ex")==0){
    if(arg[1] != NULL) {
      servo_write(atoi(arg[1]));
      c_KTMode='m';
      debug(("Exaust Servo pos: %d\n"),atoi(arg[1]));
    }else{
      Etoggle();
      debug(("toggled exaust\n"));
    }
  }else

  if(strcmp(cmd,"toggle")==0){
    if(arg[1] != NULL ){
      c_KTMode = arg[1][0];
    }else{
       rotate_KTMode();
    }    
    debug(("new c_KTMode: %s\n"),c_KTMode);
  }else
  if(strcmp(cmd,"bri")==0){
    if(arg[1] != NULL ){
      int maxbright=atoi(arg[1]);
      debug(("new LED brightness: %d\n"),atoi(arg[1]));
    }
  }else

  if(strcmp(cmd,"led")==0){
    //LED_set(arg[1],atoi(arg[2]),atoi(arg[3]),atoi(arg[4]));
    debug(("LED set: %s to %d/%d/%d\n"),arg[1],atoi(arg[2]),atoi(arg[3]),atoi(arg[4]) );
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
    debug(("minFreq is now: %d\n"),minFreq);
    servo_write(0);
  }else

  if(strcmp(cmd,"maxf")==0){
    maxFreq=atoi(arg[1]);
    debug(("maxFreq is now: %d\n"),maxFreq);
    servo_write(180);
  }else

  if(strcmp(cmd,"debug")==0){
    printDEBUG=!printDEBUG;
    debug(("print debug is now: %d\n"),printDEBUG);
  }else{
    debug(("unknown command: %s\n"),arg[0]);
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
#ifdef Eeprom_h
  loadSettings();
#endif
  pinMode(PIN_VSS_SENSOR, INPUT); 
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

  Serial.begin(BAUD_USB);
  
#ifdef MeetAndroid_h
  meetAndroid.registerFunction(and_do_COMMAND, 'z');  
#endif

#ifdef SerialCommand_h
  sCmd.setDefaultHandler(sCmd_do_command);
#endif
    debug(("--- INIT COMPLETE ---\n"));
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

  int loopDebug=0;
  time=millis();
#ifdef MeetAndroid_h
  meetAndroid.receive(); // you need to keep this in your loop() to receive events    
#endif

#ifdef SerialCommand_h
  sCmd.readSerial();
#endif
  if ( (time-lastPulseRead) > 1000 ) {
    lastSpeed=Speed;
    lastPulseRead=time;
    Speed = (HallSensorTicks/9.09888888888888);
    if(lastSpeed != Speed){
      debug(("Speed: %d km/h => %d km/h\tServo %d\tHallSensorTicks %d\n"),lastSpeed,Speed,ServoPos,HallSensorTicks);
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
  if(time >5000){
    for(int i=0; i<BUT_COUNT; i++){
      if(But[i].isHold()){  
        if(lastButStatus[i]!= 'H'){
          debug(("But[%d] hold\n"),i,lastButStatus[i]);
          if(i == 0) c_KTMode='f';
        }
        ButStatus[i]='H';
      }else
      if(But[i].onPress()){
        ButStatus[i]='P';
        if(i == 0) rotate_KTMode();
//        if(i == 1) rotate_ledMode();
        debug(("But[%d] pressed\n"),i);
      }else
      if(But[i].onRelease()){
        ButStatus[i]='R';
        debug(("But[%d] released\n"),i);
      }else
      if(But[i].onDoubleClick()){
        ButStatus[i]='D';
        debug(("But[%d] dblclicked\n"),i);
      }    
    }
   }
  
   
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

  if(time > nextUpdate){
    if(printDEBUG==true) print_debug();
    nextUpdate=millis()+5000;
  }
}
    
