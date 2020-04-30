/*
- Default state is : PAUSE, Press Enter Button to start!
- The operational mode can only be changed at "PAUSE" state!
- Add MyMotion86.h and MyMotion86.cpp at : 86Duino_Coding_318_WIN\hardware\86duino\x86\libraries\Motion86
- Download Adafruit_PCD8544.h library for SPI LCD control
*/

#include "Arduino.h"
#include "MyMotion86.h"
#include "TimerOne.h"

/*LCD*/
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
Adafruit_PCD8544 display = Adafruit_PCD8544(4, 3, 2);

/*Machine Setting*/
MyMachine machine(0);
#define MACHINE_PPU 50

/*State Define*/
enum{
  PAUSE=0,
  PLAY=1
}CNC_STATE;

/*Button Define*/
#define BTN_NUM 6 
enum{
  BTN_MODE=0,  
  BTN_ENTER,   
  BTN_UP,      
  BTN_AXIS,
  BTN_SCAL,
  BTN_DOWN
};
const int btnPin[BTN_NUM]={39,40,41,14,15,16}; //set button pin 
int btnCount[BTN_NUM];
bool btnIsActive[BTN_NUM];
bool btnPrevActive[BTN_NUM];
bool btnRiseTrigger[BTN_NUM];
const int filterTimes=3;

/*Mode Define*/
#define MODE_NUM 5
enum{
  HOME=0,
  KERORO=1,
  JOG=2,
  INCJOG=3,
  MPG=4,
  // GCODE=5
}CNC_MODE;
int mode; //variable for CNC_MODE

/*JOG,MPG Variables*/
int control_axis=0; //AXIS_X by default
double mpg_scale[3]={1, 1.8, 3.5}; //Low, Medium, High
double jog_scale[3]={5,10,15};
int scale_index=0;

void setup() {  
  Serial.begin(9600);
  /*machine setting*/
  machine.config_ReverseDirection(AXIS_Z);
  machine.config_PPU(AXIS_X, MACHINE_PPU);
  machine.config_PPU(AXIS_Y, MACHINE_PPU);
  machine.config_PPU(AXIS_Z, MACHINE_PPU);
  machine.config_PosLimit(AXIS_X, 0, 70);
  machine.config_PosLimit(AXIS_Y, 0, 70);
  machine.config_PosLimit(AXIS_Z, -25,0);
  machine.config_HomePins(7, 8, 10);
  machine.machineOn();
  machine.setDefaultFeedrate(400);
  machine.setHomeSpeed(400, 400, 400);
  machine.enableSoftLimit();
  /*button detection*/
  Timer1.initialize(10000);// 10,000 micros
  Timer1.attachInterrupt(btnCallback);  
  pinMode(BTN_MODE ,INPUT);
  pinMode(BTN_ENTER,INPUT);
  pinMode(BTN_UP   ,INPUT);
  pinMode(BTN_AXIS ,INPUT);
  pinMode(BTN_SCAL ,INPUT);
  pinMode(BTN_DOWN ,INPUT);  
  /*LCD initialize*/
  display.begin();
  display.setContrast(60);// best value for display
  display.setTextSize(1);
  display.setTextWrap(false);// no wrap if words exceeds the screen 
  display.display(); // show logo
  delay(2000);
  display.clearDisplay();// clears the screen and buffer
  /*Timer setting*/
  TimerRTC.initialize(30000);// 30,000 micros
  TimerRTC.attachInterrupt(lcdUpdate);
  /*Initial State*/
  CNC_STATE=PAUSE;
  CNC_MODE=HOME;
  mode=0;
}

void loop() {  
  while(CNC_STATE==PAUSE){
    machine.stop();//stop moving and clear all plan buffer
    if(btnRiseTrigger[BTN_MODE]){
      btnRiseTrigger[BTN_MODE]=false;
      mode=(mode+1) % MODE_NUM;        
      Serial.print("mode :");Serial.println(mode);
    }              
  }

  switch(mode){
    case HOME:
      Serial.println("Home mode!");
      machine.myHome();      
      CNC_STATE=PAUSE;
      break;
    case KERORO:
      Serial.println("Keroro mode!");
      keroro();      
      CNC_STATE=PAUSE;
      break;
    case JOG:
      Serial.println("Jog mode!");
      jog();
      break;
    case INCJOG:
      Serial.println("Incjog mode!");
      incJog();
      break;
    case MPG:
      Serial.println("MPG mode!");
      mpg();
      break;
    default:
      break;
  }
  delay(10);
}

/*button callback: Read and filtering the button signal*/
void btnCallback(){
  for(int i=0 ; i<BTN_NUM ; i++){
    if(digitalRead(btnPin[i])){      
      if(btnCount[i]<filterTimes) 
        ++btnCount[i];
      else
        btnIsActive[i]=true;        
    } 
    else{      
      btnCount[i]=0;
      btnIsActive[i]=false;      
    } 
    if(!btnPrevActive[i] && btnIsActive[i]){
      btnRiseTrigger[i]=true;
    }
    btnPrevActive[i]=btnIsActive[i];
  }
  if(btnRiseTrigger[BTN_ENTER]){ //toggle state
    btnRiseTrigger[BTN_ENTER]=false;
    CNC_STATE=(CNC_STATE==PAUSE) ? PLAY : PAUSE;
    (CNC_STATE==PAUSE)? Serial.println("PAUSE!") : Serial.println("PLAY");
  }  
  if(btnRiseTrigger[BTN_AXIS]){  
    btnRiseTrigger[BTN_AXIS]=false;            
    if(mode==JOG | mode==INCJOG | mode==MPG){
      control_axis=(control_axis+1)%3; //change axis
      if(mode==JOG | mode ==INCJOG){        
        io_DisableINT();
        machine.setJogAxis(control_axis);
        io_RestoreINT();        
      }        
      else {
        if(control_axis==AXIS_X) machine.setMpgAxis(AXIS_X);
        else if(control_axis==AXIS_Y) machine.setMpgAxis(AXIS_Y);
        else machine.setMpgAxis(AXIS_Z); 
      }
    }          
  }
  if(btnRiseTrigger[BTN_SCAL]){
    btnRiseTrigger[BTN_SCAL]=false;
    if(mode==INCJOG | mode==MPG){
      scale_index=(scale_index+1)%3;
      if(mode==INCJOG)
        machine.setJogOffset(jog_scale[scale_index]);
      else
        machine.setMpgRatio(mpg_scale[scale_index]);            
    }    
  }          
}

void lcdUpdate(){  
  double x,y,z;
  bool isManualMode;
  display.clearDisplay();
  display.setTextColor(BLACK);  
  if(mode==JOG | mode==INCJOG | mode==MPG) 
    isManualMode=true;
  else
    isManualMode=false;    
  display.setCursor(7,0);
  display.println("Hello CNC :)");  
  display.setCursor(0,10);
  display.println("STATE:");
  display.setCursor(0,18);
  if(CNC_STATE==PAUSE) 
    display.println("pause");
  else 
    display.println("play");
  display.setCursor(3,30);
  display.println("MODE:");
  display.setCursor(0,40);
  if(mode==HOME) display.println("home");
  else if(mode==KERORO) display.println("keroro");
  else if(mode==JOG) display.println("jog");
  else if(mode==INCJOG) display.println("incjog");
  else if(mode==MPG) display.println("mpg");
  if(isManualMode){
    display.setCursor(40,10);
    display.print("Scale:");display.print(scale_index);
  }
  else{
    display.setCursor(45,10);
    display.print("POS:");
  }
  machine.getActualPos(x, y, z);  
  display.setCursor(40,20);
  if(isManualMode && control_axis==AXIS_X) 
    display.setTextColor(WHITE, BLACK);
  else
    display.setTextColor(BLACK);
  display.print("x:");display.print(x);
  display.setCursor(40,30);
  if(isManualMode && control_axis==AXIS_Y) 
    display.setTextColor(WHITE, BLACK);
  else
    display.setTextColor(BLACK);
  display.print("y:");display.print(y);
  display.setCursor(40,40);
  if(isManualMode && control_axis==AXIS_Z) 
    display.setTextColor(WHITE, BLACK);
  else
    display.setTextColor(BLACK);
  display.print("z:");display.print(z);
  display.display(); // show splashscreen
}

void keroro(){
  double draw_depth =-23.7; //the height from origin of z-axis to the plotting platform
  int factor =3;
  int slow_speed =400;
  int high_speed =800;
  int lift_height =5;

  machine.myHome();// homing
  machine.line(29, 42, 0, slow_speed);// go to center
  machine.line(29, 42, draw_depth, slow_speed);
  machine.machineOff();//for resize the keroro gcode drawing
  machine.config_PPU(AXIS_X, (MACHINE_PPU) / (factor));
  machine.config_PPU(AXIS_Y, (MACHINE_PPU) / (factor));
  machine.config_PPU(AXIS_Z, (MACHINE_PPU) / (factor));
  machine.machineOn();
  machine.gcode("G92 X0 Y0 Z0"); //set new original
  machine.disableSoftLimit(); 
  /*start drawing*/
  machine.line(0, 0, lift_height, slow_speed);
  machine.line(45, 40, lift_height, high_speed);
  machine.line(45, 40, 0, slow_speed);
  machine.arcXY(60, 60, 0, true, high_speed);// r
  machine.arcXY(60, 0, -60, true, high_speed);// r
  machine.arcXY(60, -60, 0, true, high_speed);// r
  machine.arcXY(60, -45, 40, true, high_speed);// r
  machine.arcXY(120, 45, 40, true, high_speed);// r
  machine.line(45, 40, lift_height, slow_speed);
  machine.line(-42, 18, lift_height, high_speed);
  machine.line(-42, 18, 0, slow_speed);
  machine.circleXY(14, 0, true, high_speed);//circle
  machine.line(-42, 18, lift_height, slow_speed);
  machine.line(-51, 18, lift_height, high_speed);
  machine.line(-51, 18, 0, slow_speed);
  machine.circleXY(23, 0, true, high_speed);//cicle
  machine.line(-51, 18, lift_height, slow_speed);
  machine.line(42, 18, lift_height, high_speed);
  machine.line(42, 18, 0, slow_speed);
  machine.circleXY(-14, 0, true, high_speed);//cicle
  machine.line(42, 18, lift_height, slow_speed);
  machine.line(51, 18, lift_height, high_speed);
  machine.line(51, 18, 0, slow_speed);
  machine.circleXY(-23, 0, true, high_speed);//cicle
  machine.line(51, 18, lift_height, slow_speed);
  machine.line(-34, -50, lift_height, high_speed);
  machine.line(-34, -50, 0, slow_speed);
  machine.arcXY(50, 34, -50, true, high_speed);//r
  machine.line(34, -50, lift_height, slow_speed);
  machine.line(-18, -40, lift_height, high_speed);
  machine.line(-18, -40, 0, slow_speed);
  machine.arcXY(20, 18, -40, false, high_speed);//r
  machine.line(18, -40, lift_height, slow_speed);
  machine.line(-60, 40, lift_height, high_speed);
  machine.line(-60, 40, 0, slow_speed);
  machine.arcXY(70, 60, 40, true, high_speed);//r
  machine.line(84, -40, 0, high_speed);
  machine.arcXY(30, 40, -44, true, high_speed);//r
  machine.line(40,-44, lift_height,slow_speed);
  machine.line(-60, 40, lift_height, high_speed);
  machine.line(-60, 40, 0, slow_speed);
  machine.line(-84, -40, 0, high_speed);
  machine.arcXY(30, -40, -44, false, high_speed);//r
  machine.line(-40, -44, lift_height, slow_speed);
  machine.line(0, 66, lift_height, high_speed);
  machine.line(0, 66, 0, slow_speed);
  machine.line(-6, 52, 0, high_speed);
  machine.line(10, 60, 0, high_speed);
  machine.line(-10, 60, 0, high_speed);
  machine.line(6, 52, 0, high_speed);
  machine.line(0, 66, 0, high_speed);
  machine.line(0, 66, lift_height, slow_speed);
  // Wait until the movement is finished.
  while (machine.isMoving() && CNC_STATE!=PAUSE);
  machine.stop();
  machine.machineOff();
  machine.config_PPU(AXIS_X, MACHINE_PPU);
  machine.config_PPU(AXIS_Y, MACHINE_PPU);
  machine.config_PPU(AXIS_Z, MACHINE_PPU);
  machine.machineOn();
  machine.enableSoftLimit(); 
  machine.myHome();// go home
}

void jog(){  
  machine.beginJog(41, 16, false);
  machine.setJogAxis(control_axis);
  machine.setJogSpeed(600);
  while(CNC_STATE==PLAY){    
    Serial.print("JOG Axis= ");Serial.println(control_axis);
    Serial.print("Jog position = ");
    Serial.print(machine.getJogPos(AXIS_X));
    Serial.print(", ");
    Serial.print(machine.getJogPos(AXIS_Y));
    Serial.print(", ");
    Serial.println(machine.getJogPos(AXIS_Z));
    delay(1);
  }
  machine.endJog();
}

void incJog(){
  machine.beginJog(41, 16, true);
  machine.setJogAxis(control_axis);
  machine.setJogOffset(jog_scale[scale_index]);
  machine.setJogSpeed(600);
  while(CNC_STATE==PLAY){     
    Serial.print("IncJOG Scale= ");Serial.println(jog_scale[scale_index]);
    Serial.print("IncJOG Axis= ");Serial.println(control_axis);
    Serial.print("incJog position = ");
    Serial.print(machine.getJogPos(AXIS_X));
    Serial.print(", ");
    Serial.print(machine.getJogPos(AXIS_Y));
    Serial.print(", ");
    Serial.println(machine.getJogPos(AXIS_Z));
    delay(1);
  }
  machine.endJog();
}

void mpg(){
  Enc1.begin(MODE_AB_PHASE);
  machine.beginMpg(Enc1);
  machine.setMpgSpeed(600);
  machine.setMpgRatio(mpg_scale[0]);
  Enc1.setDigitalFilter(5000);//filter: 5,000 * 10 ns = 50 ms
  if(control_axis==AXIS_X)
    machine.setMpgAxis(AXIS_X);
  else if(control_axis==AXIS_Y)
    machine.setMpgAxis(AXIS_Y);
  else if(control_axis==AXIS_Z)
    machine.setMpgAxis(AXIS_Z);
  while(CNC_STATE==PLAY){       
    Serial.print("MPG Scale= ");Serial.println(mpg_scale[scale_index]);
    Serial.print("MPG Axis= ");Serial.println(control_axis);
    Serial.print("Mpg position = ");
    Serial.print(machine.getJogPos(AXIS_X));
    Serial.print(", ");
    Serial.print(machine.getJogPos(AXIS_Y));
    Serial.print(", ");
    Serial.println(machine.getJogPos(AXIS_Z));
    delay(1);
  }  
  machine.endMpg();
}
