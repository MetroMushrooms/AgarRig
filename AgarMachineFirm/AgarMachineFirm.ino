//Accel Library for driving stepper motors 
#include <AccelStepper.h>
#include <math.h>   

//Set homeswitches
#define home_switch                      3
#define lower_carousel_homeSensor        2
#define higher_carousel_homeSensor      14
#define KILL_PIN                        41

#include <Arduino.h>

// ----------------------------------------------------------------------------
// PID & Thermistor
// ----------------------------------------------------------------------------

#include <PID_v1_bc.h>
#include <GyverNTC.h>
#define AgarVessel A14
#define PIN_INPUT A13
#define PIN_OUTPUT 8

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
GyverNTC hotPlateTherm(PIN_INPUT, 100000, 3950, 25, 4700);  
GyverNTC agarVesselTherm(AgarVessel, 100000, 3950, 25, 4700); 
// pin, thermistor resistance at base temperature, beta of thermistor, base temperature, pull down resistor

//specify delay time(1/3 of second)
int delayTime = 333;
unsigned long serialTime = 0;

// ----------------------------------------------------------------------------
// LCD STUFF START 
// ----------------------------------------------------------------------------


/********************
Arduino generic menu system
Arduino menu using clickEncoder and I2C LCD
Sep.2014 Rui Azevedo - ruihfazevedo(@rrob@)gmail.com
Feb.2018 Ken-Fitz - https://github.com/Ken-Fitz
LCD library:
https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
http://playground.arduino.cc/Code/LCD3wires
*/

#include <Wire.h>
#include <U8g2lib.h>
#include <menu.h>//menu macros and objects
#include <menuIO/u8g2Out.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <menuIO/clickEncoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <SPI.h>
using namespace Menu;


#define BEEPER_PIN 37
#define fontName u8g2_font_7x13_mf
#define fontX 7
#define fontY 16
#define offsetX 0
#define offsetY 3
#define U8_Width 128
#define U8_Height 64
#define fontMarginX 2
#define fontMarginY 2



// defines Actuator pin numbers
const int ACstepPin = 54;
const int ACdirectionPin = 55;
const int ACenablePin = 38;
//Create Actuator Stepper 
AccelStepper Actuator(AccelStepper::DRIVER, ACstepPin, ACdirectionPin);

// defines Lowcar pin numbers
const int LowcarstepPin = 60;
const int LowcardirectionPin = 61;
const int LowcarenablePin = 56;
//Create Actuator Stepper 
AccelStepper Lowcar(AccelStepper::DRIVER, LowcarstepPin, LowcardirectionPin);

// defines Highcar pin numbers
const int HighcarstepPin = 46;
const int HighcardirectionPin = 48;
const int HighcarenablePin = 62;
//Create Actuator Stepper 
AccelStepper Highcar(AccelStepper::DRIVER, HighcarstepPin, HighcardirectionPin);

// defines PUMP pin numbers
const int PumpstepPin = 26;
const int PumpdirectionPin = 28;
const int PumpenablePin = 24;
//Create Actuator Stepper 
AccelStepper Pump(AccelStepper::DRIVER, PumpstepPin, PumpdirectionPin);

//Actuator stepper variables 
String dataIn = "";
String manualStatus = "";
int count = 0;
int dist;
long initial_homing=-1;

int Lcount = 0;
int Ldist;
long Linitial_homing=1;

//////////////////////////////////////////////////////////
// BEGIN LCD STUFF
//////////////////////////////////////////////////////////

U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0,  23 , 17, 16);

const colorDef<uint8_t> colors[6] MEMMODE={
  {{0,0},{0,1,1}},//bgColor
  {{1,1},{1,0,0}},//fgColor
  {{1,1},{1,0,0}},//valColor
  {{1,1},{1,0,0}},//unitColor
  {{0,1},{0,0,1}},//cursorColor
  {{1,1},{1,0,0}},//titleColor
};

// Encoder /////////////////////////////////////
#define encA 33
#define encB 31
//this encoder has a button here
#define encBtn 35


ClickEncoder clickEncoder(encA,encB,encBtn,2);
// The number following clickEncoder sets the sensitivity
ClickEncoderStream encStream(clickEncoder,2);
MENU_INPUTS(in,&encStream);
void timerIsr() {clickEncoder.service();}

//////////////////////////////////////////////////////////
// Opertations 
//////////////////////////////////////////////////////////
int exitMenuOptions = 0; //Forces the menu to exit and cut the copper tape

void startCycle() {
  Serial.println("Yeet");
  delay(20000);
  exitMenuOptions = 0; // Return to the menu
} 

int upperCarouselDistance; 
void jogUpperCarousel() {
      int dir = 1;
      while (Highcar.currentPosition() != upperCarouselDistance) {
        if (upperCarouselDistance < 0) {
          dir = -1;
        }
        Highcar.setMaxSpeed(500*dir);
        Highcar.setSpeed(500*dir);
        Highcar.run();
        Serial.println(dir);
      }
      Highcar.setCurrentPosition(0);
      delay(100);
      exitMenuOptions = 0;
    }

int lowerCarouselDistance;
void jogLowerCarousel() {
      int dir = 1;
      while (Lowcar.currentPosition() != lowerCarouselDistance) {
        if (upperCarouselDistance < 0) {
          dir = -1;
        }
       Lowcar.setMaxSpeed(500*dir);
       Lowcar.setSpeed(500*dir);
       Lowcar.run();
      }
      Lowcar.setCurrentPosition(0);
      delay(100);
      exitMenuOptions = 0;
    }

int ActuatorDistance;
void jogActuator() {
      int dir = 1;
        if (upperCarouselDistance < 0) {
          dir = -1;
        }
        Actuator.setMaxSpeed(500*dir);
        Actuator.setSpeed(500*dir);
        Actuator.moveTo(ActuatorDistance);
        Actuator.runToPosition();
      
      Actuator.setCurrentPosition(0);
      delay(100);
      exitMenuOptions = 0;
    }




//////////////////////////////////////////////////////////
// Start ArduinoMenu
//////////////////////////////////////////////////////////

result doAlert(eventMask e, prompt &item);

result showEvent(eventMask e,navNode& nav,prompt& item) {
  Serial.print("event: ");
  Serial.println(e);
  return proceed;
}


// Starts Filling Cycle
result doStartCycle() {
  delay(500);
  exitMenuOptions = 1;
  return proceed;
}

//Jogs Upper carousel
result doJogUpperCarousel() {
  delay(500);
  exitMenuOptions = 2;
  return proceed;
}

result doJogLowerCarousel() {
  delay(500);
  exitMenuOptions = 3;
  return proceed;
}

result doJogActuator() {
  delay(500);
  exitMenuOptions = 4;
  return proceed;
}

//Menu Vars 
int plateCount=50;

//Motor Tuning 
MENU(motorTuning,"Motor Tuning",showEvent,anyEvent,noStyle
  // ,OP("Actuator   Speed",showEvent,anyEvent)
  // ,OP("Carousel 1 Speed",showEvent,anyEvent)
  // ,OP("Carousel 2 Speed",showEvent,anyEvent)
  ,FIELD(upperCarouselDistance,"^Carousel JOG: ","",-5000,5000,20,1,doNothing,noEvent,
  wrapStyle)
  ,OP("Jog ^carousel", doJogUpperCarousel, enterEvent)

  ,FIELD(lowerCarouselDistance,"^Low Carousel JOG: ","",-5000,5000,20,1,doNothing,noEvent, wrapStyle)
  ,OP("Jog low carousel", doJogLowerCarousel, enterEvent)

  ,FIELD(ActuatorDistance,"Act JOG: ","",-20000,20000,1000,100,doNothing,noEvent, wrapStyle)
  ,OP("Jog Actuator", doJogActuator, enterEvent)

  ,EXIT("<Back")
);

// Main Settings 
MENU(settingsMenu,"SETTINGS",showEvent,anyEvent,noStyle
  , OP("Plate  Count ",showEvent,anyEvent)
  , SUBMENU(motorTuning)
  , OP("Homing Tuning",showEvent,anyEvent)
  ,EXIT("<Back")
);

char* constMEM hexDigit MEMMODE="0123456789ABCDEF";
char* constMEM hexNr[] MEMMODE={"0","x",hexDigit,hexDigit};
char buf1[]="0x11";

MENU(mainMenu,"MAIN MENU",doNothing,noEvent,wrapStyle
  ,OP("Start", doStartCycle, enterEvent)
  ,FIELD(plateCount,"Plate Count","",0,50,10,1,doNothing,noEvent,wrapStyle)
  ,SUBMENU(settingsMenu)
);

#define MAX_DEPTH 5

MENU_OUTPUTS(out,MAX_DEPTH
  ,U8G2_OUT(u8g2,colors,fontX,fontY,offsetX,offsetY,{0,0,U8_Width/fontX,U8_Height/fontY})
  ,SERIAL_OUT(Serial)
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);//the navigation root object


result idle(menuOut& o,idleEvent e) {
  switch(e) {
    case idleStart:o.print("suspending menu!");break;
    case idling:o.print("suspended...");break;
    case idleEnd:o.print("resuming menu.");break;
  }
  return proceed;
}

//////////////////////////////////////////////////////////
// Bouncey Boy Frames
//////////////////////////////////////////////////////////
#define bounce1_width 53
#define bounce1_height 48
const unsigned char bounce1_bits[]PROGMEM = {
  0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x01, 
  0x00, 0x00, 0x00, 0x00, 0x7E, 0xE4, 0x07, 0x00, 0x00, 0x00, 0xC0, 0xAF, 
  0x39, 0x1E, 0x00, 0x00, 0x00, 0xF8, 0x52, 0x2A, 0x79, 0x00, 0x00, 0x00, 
  0x1E, 0xA5, 0x75, 0xF6, 0x00, 0x00, 0x80, 0x93, 0x3F, 0xEA, 0xAE, 0x03, 
  0x00, 0xC0, 0xED, 0xE8, 0x55, 0x51, 0x07, 0x00, 0x70, 0x6D, 0x80, 0xAA, 
  0x67, 0x0C, 0x00, 0x90, 0x35, 0x05, 0x55, 0x2E, 0x3F, 0x00, 0xC8, 0x11, 
  0xF8, 0x2B, 0xF9, 0x7B, 0x00, 0x6C, 0xBA, 0xFF, 0xFF, 0xDC, 0xED, 0x00, 
  0x5C, 0xF5, 0xFD, 0xFF, 0xB7, 0xBB, 0x01, 0xAE, 0xFE, 0xFF, 0x7F, 0x5F, 
  0x6A, 0x01, 0x56, 0xDF, 0x07, 0xF0, 0xFB, 0x65, 0x03, 0xA6, 0xF3, 0x00, 
  0x80, 0x77, 0xFF, 0x07, 0x9A, 0x3D, 0x00, 0x00, 0xFE, 0xAF, 0x07, 0xA6, 
  0x1F, 0x00, 0x00, 0xB8, 0x53, 0x0F, 0xDE, 0x0E, 0x00, 0x00, 0xF0, 0xA7, 
  0x0E, 0xF4, 0x77, 0x00, 0xC0, 0xE4, 0x5E, 0x0D, 0xCC, 0xFF, 0x00, 0xE0, 
  0xC1, 0xAE, 0x1C, 0x98, 0xFB, 0x98, 0xF0, 0xC1, 0xDF, 0x1F, 0xB0, 0x73, 
  0xFC, 0xF0, 0x91, 0x2F, 0x0F, 0xE0, 0x31, 0xFC, 0xC0, 0x80, 0xAB, 0x0D, 
  0xC0, 0x01, 0xFC, 0x00, 0x00, 0xEF, 0x06, 0x80, 0x00, 0xFE, 0x00, 0x00, 
  0x7E, 0x03, 0x80, 0x00, 0xFC, 0x00, 0x00, 0xDE, 0x03, 0xC0, 0x00, 0xFC, 
  0x00, 0xA0, 0xEC, 0x00, 0x40, 0x00, 0xF8, 0x00, 0x00, 0x7D, 0x00, 0x40, 
  0x00, 0x00, 0x00, 0xA0, 0x0C, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x08, 
  0x00, 0x60, 0x00, 0x00, 0x00, 0x80, 0x0A, 0x00, 0x60, 0x00, 0x00, 0x00, 
  0x40, 0x19, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x20, 0x00, 
  0x00, 0x00, 0x40, 0x09, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 
  0x60, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 
  0x0C, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x00, 0xC0, 0x00, 0x00, 
  0x00, 0x50, 0x04, 0x00, 0x80, 0x00, 0x00, 0x00, 0x50, 0x02, 0x00, 0x00, 
  0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0xC0, 0x01, 
  0x00, 0x00, 0x0C, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 
  0x3C, 0x00, 0x00, 0x00, 0xE0, 0x01, 0xA0, 0x0F, 0x00, 0x00, 0x00, 0x80, 
  0x9F, 0xFD, 0x01, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x00, 0x00, 
  };

#define bounce2_width 53
#define bounce2_height 53
const unsigned char bounce2_bits[]PROGMEM = {
  0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x7F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xFC, 0xF3, 0x03, 0x00, 0x00, 0x00, 0x80, 0x1F, 
  0xA5, 0x0F, 0x00, 0x00, 0x00, 0xE0, 0x93, 0x3A, 0x1D, 0x00, 0x00, 0x00, 
  0x78, 0x64, 0xA5, 0x72, 0x00, 0x00, 0x00, 0x9F, 0x9D, 0x6A, 0xD5, 0x00, 
  0x00, 0x80, 0x67, 0x7B, 0xD5, 0xAA, 0x03, 0x00, 0xC0, 0xD5, 0x61, 0xAA, 
  0xA9, 0x07, 0x00, 0x70, 0x69, 0x9A, 0x55, 0x57, 0x0E, 0x00, 0xB0, 0x25, 
  0x80, 0xAD, 0x3E, 0x1B, 0x00, 0xC8, 0xBA, 0xA9, 0x45, 0xD9, 0x3E, 0x00, 
  0x6C, 0x15, 0xFF, 0x7F, 0xEC, 0xE7, 0x00, 0xAC, 0xF6, 0xEF, 0xFE, 0xB7, 
  0xDD, 0x00, 0x5E, 0xF9, 0xFD, 0xFF, 0x6F, 0xB9, 0x01, 0xA2, 0x9E, 0x5F, 
  0x7E, 0x5E, 0x6B, 0x03, 0xAA, 0xFE, 0x01, 0xC0, 0x7F, 0xFB, 0x03, 0x56, 
  0x7B, 0x00, 0x80, 0xF7, 0xBD, 0x07, 0xAB, 0x37, 0x00, 0x00, 0xDC, 0x47, 
  0x07, 0x96, 0x1F, 0x00, 0x00, 0xBC, 0x3B, 0x05, 0xDE, 0x0C, 0x00, 0x00, 
  0xF0, 0xB7, 0x0E, 0xF4, 0x3F, 0x00, 0x40, 0xE0, 0x56, 0x1D, 0xCC, 0x7F, 
  0x00, 0xF0, 0xC8, 0xCC, 0x1D, 0xC8, 0xFB, 0x00, 0xF0, 0xD1, 0xAF, 0x0E, 
  0x30, 0xFB, 0xFE, 0xE0, 0x81, 0xAF, 0x0F, 0xF0, 0x73, 0xFC, 0xE0, 0x90, 
  0x4B, 0x0D, 0xE0, 0x01, 0xFC, 0x00, 0x20, 0xBF, 0x07, 0x80, 0x00, 0xFC, 
  0x00, 0x00, 0xF7, 0x06, 0x80, 0x00, 0xDC, 0x00, 0x60, 0xDE, 0x03, 0x80, 
  0x00, 0xFC, 0x00, 0x00, 0x96, 0x01, 0xC0, 0x00, 0xFC, 0x00, 0x40, 0xFD, 
  0x00, 0x40, 0x00, 0x78, 0x00, 0x00, 0x7C, 0x00, 0x40, 0x00, 0x00, 0x00, 
  0xA0, 0x18, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x60, 0x00, 
  0x00, 0x00, 0x80, 0x09, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 
  0x1A, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x60, 0x00, 0x00, 
  0x00, 0x40, 0x08, 0x00, 0x60, 0x00, 0x00, 0x00, 0xA0, 0x09, 0x00, 0x40, 
  0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x00, 0xC0, 0x00, 0x00, 0x00, 0x50, 0x06, 0x00, 0x80, 0x00, 0x00, 0x00, 
  0x40, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x20, 0x03, 0x00, 0x00, 0x03, 
  0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x06, 0x00, 0x00, 0xE4, 0x00, 0x00, 
  0x00, 0x1C, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x1E, 
  0x00, 0x00, 0x00, 0xC0, 0x03, 0xE0, 0x07, 0x00, 0x00, 0x00, 0x00, 0xFE, 
  0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x00, 0x00, };

#define bounce3_width 53
#define bounce3_height 46
const unsigned char bounce3_bits[]PROGMEM = {
  0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xFC, 0xE3, 0x03, 0x00, 0x00, 0x00, 0xC0, 0x9F, 
  0x7A, 0x0F, 0x00, 0x00, 0x00, 0xF8, 0x55, 0xA6, 0x39, 0x00, 0x00, 0x00, 
  0x5E, 0xA2, 0x6D, 0xF6, 0x00, 0x00, 0x80, 0x57, 0x3F, 0x56, 0x96, 0x01, 
  0x00, 0xC0, 0xE9, 0xD9, 0xA9, 0x59, 0x07, 0x00, 0x70, 0x6B, 0xC0, 0xAA, 
  0xA7, 0x0E, 0x00, 0x90, 0x25, 0x05, 0x55, 0x5E, 0x3E, 0x00, 0x48, 0x3A, 
  0x78, 0x3F, 0xFD, 0x7B, 0x00, 0xEC, 0xD5, 0xFF, 0xFD, 0x61, 0xEE, 0x00, 
  0x5C, 0xFA, 0xFF, 0xEF, 0xBF, 0xA7, 0x01, 0xA6, 0xDD, 0xFF, 0xFF, 0xDF, 
  0x5D, 0x03, 0x5E, 0xFE, 0x01, 0xE0, 0x7B, 0xFB, 0x03, 0xA6, 0x7B, 0x00, 
  0x00, 0xEF, 0xDC, 0x06, 0x96, 0x3F, 0x00, 0x00, 0xBC, 0x5F, 0x07, 0xCE, 
  0x1F, 0x00, 0x00, 0xF8, 0xA3, 0x0E, 0xFE, 0x0C, 0x00, 0x00, 0xF4, 0x5F, 
  0x0D, 0xE4, 0x7F, 0x00, 0xE0, 0xC0, 0xA6, 0x1E, 0x98, 0xFF, 0x10, 0xF0, 
  0xC1, 0x6D, 0x1F, 0x98, 0xFB, 0xFC, 0xF0, 0x91, 0x8F, 0x0E, 0xF0, 0x71, 
  0xFC, 0xE0, 0x80, 0x3F, 0x0F, 0xC0, 0x01, 0xFC, 0x00, 0x20, 0xDF, 0x06, 
  0x80, 0x00, 0xFC, 0x00, 0x40, 0xFE, 0x03, 0x80, 0x00, 0xFE, 0x00, 0x00, 
  0xAE, 0x03, 0xC0, 0x00, 0xFC, 0x00, 0xA0, 0xEE, 0x00, 0x40, 0x00, 0x78, 
  0x00, 0x00, 0x3C, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x40, 
  0x00, 0x00, 0x00, 0x40, 0x09, 0x00, 0x60, 0x00, 0x00, 0x00, 0x80, 0x08, 
  0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x20, 0x00, 0x00, 0x00, 
  0x00, 0x1A, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x08, 0x00, 0x60, 0x00, 
  0x00, 0x00, 0x80, 0x08, 0x00, 0x60, 0x00, 0x00, 0x00, 0x20, 0x0D, 0x00, 
  0x40, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 
  0x04, 0x00, 0x80, 0x00, 0x00, 0x00, 0x30, 0x06, 0x00, 0x80, 0x01, 0x00, 
  0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 
  0x06, 0x00, 0x00, 0xE4, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x3C, 0x00, 
  0x00, 0x00, 0x70, 0x00, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x80, 0x1F, 0xFC, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0x00, };

#define bounce4_width 53
#define bounce4_height 44
const unsigned char bounce4_bits[]PROGMEM = {
  0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xBC, 0xE2, 0x01, 0x00, 0x00, 0x00, 0x80, 0x4F, 
  0xB5, 0x07, 0x00, 0x00, 0x00, 0xF0, 0x91, 0x16, 0x1E, 0x00, 0x00, 0x00, 
  0x3E, 0x7F, 0x73, 0x3A, 0x00, 0x00, 0xC0, 0xAF, 0xFB, 0x6E, 0xFA, 0x00, 
  0x00, 0x70, 0xD1, 0x80, 0xC9, 0x94, 0x03, 0x00, 0x98, 0x6C, 0x80, 0xA9, 
  0xA5, 0x07, 0x00, 0xCC, 0x36, 0xDD, 0x5F, 0x5F, 0x1F, 0x00, 0x7E, 0x91, 
  0xFF, 0xFE, 0x78, 0xF9, 0x00, 0xA6, 0xFA, 0xFD, 0xCF, 0xEF, 0xEF, 0x01, 
  0x6B, 0xDD, 0x5F, 0xFE, 0x5F, 0x24, 0x07, 0x57, 0xFF, 0x03, 0xF0, 0xF7, 
  0xFF, 0x06, 0xAE, 0x73, 0x00, 0x00, 0xFF, 0x9D, 0x0F, 0x96, 0x3F, 0x00, 
  0x00, 0x9C, 0xD7, 0x0F, 0x96, 0x0E, 0x00, 0x00, 0xF0, 0x57, 0x0E, 0xEC, 
  0x2F, 0x00, 0x40, 0xE0, 0xAE, 0x1F, 0xC8, 0xFF, 0x00, 0xF0, 0xD1, 0x6C, 
  0x0E, 0x98, 0xFB, 0x94, 0xF0, 0xC1, 0xAF, 0x0E, 0xF0, 0xFB, 0xFC, 0xF0, 
  0x80, 0x8F, 0x07, 0xC0, 0x01, 0xFC, 0x40, 0x30, 0xBF, 0x07, 0x80, 0x00, 
  0xFC, 0x00, 0x00, 0xFF, 0x03, 0x80, 0x00, 0xFC, 0x00, 0x60, 0xAE, 0x01, 
  0xC0, 0x00, 0xFC, 0x00, 0x00, 0xEE, 0x00, 0x40, 0x00, 0xFC, 0x00, 0x40, 
  0x7C, 0x00, 0x40, 0x00, 0x30, 0x00, 0x00, 0x1D, 0x00, 0x60, 0x00, 0x00, 
  0x00, 0x40, 0x08, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x08, 0x00, 0x20, 
  0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x18, 
  0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x20, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x00, 0x60, 0x00, 0x00, 0x00, 0xA0, 0x0D, 0x00, 0x40, 0x00, 
  0x00, 0x00, 0x80, 0x0C, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x10, 0x04, 0x00, 
  0x80, 0x00, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 
  0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x00, 0x0C, 0x00, 
  0x00, 0x78, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 
  0xE0, 0x03, 0xE0, 0x07, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0x00, };

#define bounce5_width 53
#define bounce5_height 42
const unsigned char bounce5_bits[]PROGMEM = {
  0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x01, 
  0x00, 0x00, 0x00, 0x00, 0x7F, 0xF3, 0x07, 0x00, 0x00, 0x00, 0xE0, 0xAB, 
  0x85, 0x1E, 0x00, 0x00, 0x00, 0x7C, 0x50, 0x35, 0x73, 0x00, 0x00, 0x00, 
  0x9F, 0xBF, 0xEB, 0xF6, 0x01, 0x00, 0xC0, 0x91, 0x6B, 0x96, 0x68, 0x07, 
  0x00, 0xE0, 0xED, 0xC0, 0xA9, 0x97, 0x0E, 0x00, 0xB0, 0x2A, 0x84, 0xB1, 
  0x2E, 0x3E, 0x00, 0xCC, 0x36, 0xFB, 0x5F, 0xFE, 0x7B, 0x00, 0x6C, 0xD2, 
  0xDF, 0xFF, 0xE9, 0xD7, 0x00, 0xD6, 0xFD, 0xFD, 0xDF, 0x9F, 0xF5, 0x01, 
  0x6A, 0xDE, 0x0F, 0xFC, 0xFF, 0x3B, 0x03, 0x16, 0xFF, 0x01, 0xC0, 0xF7, 
  0xFA, 0x07, 0xAF, 0x73, 0x00, 0x00, 0xDE, 0xDE, 0x05, 0xD6, 0x1F, 0x00, 
  0x00, 0xBC, 0x27, 0x0F, 0xAE, 0x0E, 0x00, 0x00, 0xF0, 0xA7, 0x0E, 0xEC, 
  0x77, 0x00, 0xE0, 0xE0, 0x5E, 0x1E, 0xD8, 0xFF, 0x00, 0xF0, 0xC1, 0xAE, 
  0x1F, 0x98, 0xFB, 0xFC, 0xF0, 0xA1, 0x5F, 0x0E, 0xE0, 0x63, 0xFC, 0x60, 
  0x90, 0x8F, 0x0D, 0xC0, 0x01, 0xFC, 0x00, 0x00, 0xFF, 0x07, 0x80, 0x00, 
  0xFC, 0x00, 0x20, 0xBE, 0x03, 0xC0, 0x00, 0xFC, 0x00, 0x40, 0xEE, 0x01, 
  0x40, 0x00, 0xFC, 0x00, 0x00, 0x7D, 0x00, 0x40, 0x00, 0x70, 0x00, 0x40, 
  0x3C, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x60, 0x00, 0x00, 
  0x00, 0x40, 0x09, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x20, 
  0x00, 0x00, 0x00, 0x40, 0x08, 0x00, 0x20, 0x00, 0x00, 0x00, 0x80, 0x08, 
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x40, 0x00, 0x00, 0x00, 
  0x20, 0x09, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0xC0, 0x00, 
  0x00, 0x00, 0x20, 0x06, 0x00, 0x80, 0x00, 0x00, 0x00, 0x20, 0x03, 0x00, 
  0x00, 0x03, 0x00, 0x00, 0x88, 0x01, 0x00, 0x00, 0x06, 0x00, 0x00, 0xE0, 
  0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x70, 0x00, 
  0x80, 0x0F, 0x00, 0x00, 0x00, 0x80, 0x5F, 0xFE, 0x01, 0x00, 0x00, 0x00, 
  0x00, 0xE0, 0x0B, 0x00, 0x00, 0x00, };



// ----------------------------------------------------------------------------
// LCD STUFF END 
// ----------------------------------------------------------------------------

// // ----------------------------------------------------------------------------
// // Thermister and heated bed  
// // ----------------------------------------------------------------------------
// #define TEMP_0_PIN          13   // ANALOG NUMBERING
// #define TEMP_1_PIN          14   // ANALOG NUMBERING
// #define HEATER_0_PIN        8
// double Output = 0.2;

void setup()
{
  Serial.begin(9600);
//////////////////////////////////////////////////////////
// LCD SETUP 
//////////////////////////////////////////////////////////
  pinMode(encBtn,INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);

  Serial.println("Arduino Menu Library");Serial.flush();
  SPI.begin();
  u8g2.begin();
  int slow = 2;
    // u8g2.firstPage();
  do {
    u8g2.clearBuffer();
    u8g2.drawXBMP(37, 8,53,42, bounce5_bits);
    u8g2.sendBuffer();
    delay(slow);

    u8g2.clearBuffer();
    u8g2.drawXBMP(37, 8,53,44, bounce4_bits);
    u8g2.sendBuffer();
    delay(slow);


    u8g2.clearBuffer();
    u8g2.drawXBMP(37, 8,53,46, bounce3_bits);
    u8g2.sendBuffer();
    delay(slow);
    
    u8g2.clearBuffer();
    u8g2.drawXBMP(37, 8,53,53, bounce2_bits);
    u8g2.sendBuffer();
    delay(slow);

    u8g2.clearBuffer();
    u8g2.drawXBMP(37, 8,53,48, bounce1_bits);
    u8g2.sendBuffer();
    delay(slow);
  } while ( u8g2.nextPage() );
  
  u8g2.setFont(fontName);
  nav.idleTask=idle;
  nav.showTitle=true;

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  delay(2000);

  //LCD Beeper 
  pinMode(BEEPER_PIN, OUTPUT);
  digitalWrite(BEEPER_PIN, LOW);
//////////////////////////////////////////////////////////
// LCD SETUP END
//////////////////////////////////////////////////////////

   //Actuator Limit Switch
  pinMode(home_switch, INPUT_PULLUP);
  delay(5);

  //Lower Carousel homing sensor
  pinMode(lower_carousel_homeSensor, INPUT);
  delay(5);

  //Upper Carousel homing sensor 
  pinMode(higher_carousel_homeSensor, INPUT);
  delay(5);

  //BEGIN ACTUATOR SETUP
  Actuator.setEnablePin(ACenablePin);
  Actuator.setPinsInverted(false, false, true);
  Actuator.enableOutputs();
  Actuator.setMaxSpeed(1000);
  Actuator.setSpeed(700);
  Actuator.setAcceleration(100);

  Serial.print("Actuator is Homing...........");

  // while (!digitalRead(home_switch)) { // Move Actuator CCW until switch triggered
  //   // Serial.println("Moving to switch...........");
  //   Actuator.moveTo(initial_homing); //Set the position to move to
  //   initial_homing--; 
  //   Actuator.run();
  //   delay(5);
  // }

  // Actuator.setCurrentPosition(0);
  // Actuator.setMaxSpeed(1000);
  // Actuator.setAcceleration(100);
  // initial_homing=1;
  
  // while (digitalRead(home_switch)) { //Move CW until limit is released
  //   //Serial.println("Moving away from switch...........");
  //   Actuator.moveTo(initial_homing);
  //   Actuator.run();
  //   initial_homing++;
  //   delay(5);
  // }

  // Actuator.setCurrentPosition(0);
  // Serial.println("Actuator homing Completed");
  // Serial.println("");
  // Actuator.setMaxSpeed(1000);
  // Actuator.setAcceleration(1000);

  // int count = 0;
  // int dist;
  // long initial_homing=-1;
  // delay(500);

//END ACTUATOR SETUP 

//BEGIN LOWER CAROUSEL SETUP  
  Serial.println("Homing Lower Carousel");

  Lowcar.setEnablePin(LowcarenablePin);
  Lowcar.setPinsInverted(false, false, true);
  Lowcar.enableOutputs();
  Lowcar.setMaxSpeed(1000);
  Lowcar.setSpeed(400);
  Lowcar.setAcceleration(100);

  Serial.print("Lowcar is Homing...........");

  // while (!digitalRead(lower_carousel_homeSensor)) { 
  //   Lowcar.moveTo(Linitial_homing); //Set the position to move to
  //   Linitial_homing++; 
  //   Lowcar.run();
  //   delay(5);
  // }

  // Lowcar.setCurrentPosition(0);
  // Lowcar.setMaxSpeed(1000);
  // Lowcar.setAcceleration(100);
  // Linitial_homing=-1;
  
  // while (digitalRead(lower_carousel_homeSensor)) { 
  //   Lowcar.moveTo(Linitial_homing);
  //   Lowcar.run();
  //   Linitial_homing--;
  //   delay(5);
  // }

  // Lowcar.setCurrentPosition(0);

  // while (Lowcar.currentPosition()!= 30) {
  // Lowcar.setSpeed(200);
  // Lowcar.run();

  // }
  // Lowcar.setCurrentPosition(0);
  // Serial.println("Lowcar homing Completed");
  // Serial.println("");
  // Lowcar.setMaxSpeed(1000);
  // Lowcar.setAcceleration(1000);
  // delay(500);

//BEGIN Higher CAROUSEL SETUP  
  Serial.println("Homing Lower Carousel");

  Highcar.setEnablePin(HighcarenablePin);
  Highcar.setPinsInverted(false, false, true);
  Highcar.enableOutputs();
  Highcar.setMaxSpeed(1000);
  Highcar.setSpeed(400);
  Highcar.setAcceleration(100);

  Serial.print("Highcar is Homing...........");

  // while (!digitalRead(higher_carousel_homeSensor)) { 
  //   Highcar.moveTo(initial_homing); //Set the position to move to
  //   initial_homing--; 
  //   Highcar.run();
  //   delay(5);
  // }

  // Highcar.setCurrentPosition(0);
  // Highcar.setMaxSpeed(1000);
  // Highcar.setAcceleration(100);
  // initial_homing=1;
  
  // while (digitalRead(higher_carousel_homeSensor)) { 
  //   Highcar.moveTo(initial_homing);
  //   Highcar.run();
  //   initial_homing++;
  //   delay(5);
  // }

  // Highcar.setCurrentPosition(0);
  // while (Highcar.currentPosition() != 100) {
  // Highcar.setSpeed(200);
  // Highcar.run();

  // }
  // Highcar.setCurrentPosition(0);
  // while (Highcar.currentPosition() != -400) {
  // Highcar.setSpeed(-200);
  // Highcar.run();

  // }
  // Highcar.setCurrentPosition(0);
  // Serial.println("Highcar homing Completed");
  // Serial.println("");
  // Highcar.setMaxSpeed(1000);
  // Highcar.setAcceleration(1000);
  // delay(500);

  // // Enable Pump 
  // Pump.setEnablePin(PumpenablePin);
  // Pump.setPinsInverted(false, false, true);
  // Pump.enableOutputs();
  // Pump.setMaxSpeed(1000);
  // Pump.setSpeed(400);
  // Pump.setAcceleration(100);

  Serial.println("Pump Enabled");  

  //PID 
    //initialize the variables we're linked to
  Input = hotPlateTherm.getTemp();
  Setpoint = 105;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
//PRINT MESSAGE TO LOOP AFTER SETUP 
  Serial.println("Enter a mode: ");
}


void loop() {

  // analogWrite(HEATER_0_PIN, 230);
  Input = hotPlateTherm.getTemp();

  double gap = abs(Setpoint - Input);  //distance away from setpoint
  if (gap < 2) {                      //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  } else {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);

  //Loop to only send a serial message every so often without using delay
  if (millis() > serialTime) {
    Serial.print("Heat plate ");
    Serial.print(hotPlateTherm.getTemp());
    Serial.print(" *C");

    Serial.print(" Agar Vessel ");
    Serial.print(agarVesselTherm.getTemp());
    Serial.println(" *C");
    serialTime += delayTime;
  }
  // switch (exitMenuOptions) {
  //   case 1: {
  //     delay(500); // Pause to allow the button to come up 
  //     startCycle(); 
  //     break;
  //   }
  //   case 2: {
  //     delay(500); // Pause to allow the button to come up 
  //     jogUpperCarousel(); 
  //     break;
  //   }
  //   case 3: {
  //     delay(500); // Pause to allow the button to come up 
  //     jogLowerCarousel(); 
  //     break;
  //   }
  //   case 4: {
  //     delay(500); // Pause to allow the button to come up 
  //     jogActuator();
  //     break;
  //   }    
  // }

  // nav.doInput();
  // if (nav.changed(0)) {//only draw if menu changed for gfx device
  //   u8g2.firstPage();
  //   do nav.doOutput(); while(u8g2.nextPage());
  // }
}


// ----------------------------------------------------------------------------
// MOTOR FUNCTIONS 
// ----------------------------------------------------------------------------

//Go Mode 
void actuationRotation() {
    Serial.println("AcutationRotation Cycle Start");

    boolean firstPlate = true;
    int plateCount = 50;    
    int platesFilled = 0;

    Actuator.setMaxSpeed(6000);
    Actuator.setAcceleration(1000); 
    
    Lowcar.setMaxSpeed(200);
    Lowcar.setAcceleration(800); 

    Highcar.setMaxSpeed(1000);
    Highcar.setAcceleration(800); 

    Pump.setMaxSpeed(6000);
    Pump.setAcceleration(1000);
    
    //Load our First Plate 
    while (firstPlate == true) {
      //Actuator Rises to top to recieve plate from High Carousel 
      Actuator.moveTo(31700);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      // if (encoder->getButton() == 3) break;
      delay(5);
      //High Carousel rotates empty plate slot to sit on Actuator 
      Highcar.moveTo(400);
      Highcar.runToPosition();
      Highcar.setCurrentPosition(0);
      // if (encoder->getButton() == 3) break;
      delay(5);
      //Actuator drop just below high carousel platform 
      Actuator.moveTo(-5800);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      // if (encoder->getButton() == 3) break;
      delay(5);
      //High Carousel rotates to awkward position to prevent plate falling
      Highcar.moveTo(-400);
      Highcar.runToPosition();
      Highcar.setCurrentPosition(0);
      // if (encoder->getButton() == 3) break;
      delay(5);
      //Actuator drop below Lower Carousel and deposit first plate 
      Actuator.moveTo(-25900);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      plateCount--;
      // if (encoder->getButton() == 3) break;
      delay(5);
      //Lower Carousel Rotate, placing a empty plate in filling position 
      Lowcar.moveTo(800);
      Lowcar.runToPosition();
      Lowcar.setCurrentPosition(0);
      // if (encoder->getButton() == 3) break;
      delay(5);
      //update frist plate state 
      firstPlate = false;       
    }

    //Continue loading plates 
    while (firstPlate == false) {
      Serial.println("Loading additional plates, accounting for filled plate");
      
      // Fill plate 
      while (Pump.currentPosition() != 100000) {
        Pump.setMaxSpeed(5000);
        Pump.setSpeed(5000);
        Pump.run();
        // if (encoder->getButton() == 3) break;
      }

      // if (encoder->getButton() == 3) break;
      Pump.setCurrentPosition(0);
      platesFilled++;

      //Actuator Pushes plate up to platform level of High carousel
      Serial.println("Actuator Pushes plate up to platform level of High carousel");
      Actuator.moveTo(25000);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //High Carousel rotates filled plate stack over newly filled plate 
      Serial.println("Placing Filled Stack Over Filled Plate");
      Highcar.moveTo(-400);
      Highcar.runToPosition();
      Highcar.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //Actuator pushes plate up stack, now actuator is at platform level 33k
      Serial.println("Pushing Plate into stack");
      Actuator.moveTo(7500);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //High Carousel rotates to awkward position for filled plates
      Serial.println("Sweeping filled plate");
      Highcar.moveTo(400);
      Highcar.runToPosition();
      Highcar.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //High Carousel rotates to empty plate stack
      Serial.println("Placing empty plate on Actuator");
      Highcar.moveTo(400);
      Highcar.runToPosition();
      Highcar.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //Actuator drop plate to platform level 
      Actuator.moveTo(-7500);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //High Carousel rotates Holding Position
      Highcar.moveTo(400);
      Highcar.runToPosition();
      Highcar.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //Actuator Pushes plate up to platform level of High carousel
      Actuator.moveTo(-25000);
      Actuator.runToPosition();
      Actuator.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
      //Lower Carousel Rotate, placing a empty plate in filling position 
      Lowcar.moveTo(800);
      Lowcar.runToPosition();
      Lowcar.setCurrentPosition(0);
      delay(1000);
      // if (encoder->getButton() == 3) break;
    }
  Serial.println("Exiting cycle and returning to main loop");
  return;
}

//Mode for manual jogging
// void manual() {
//   //29500 is the top 
//   String dataInS;
//   dataIn ="";
//   Serial.println("Entered Manual Mode, Type end to exit");
//    while (!dataIn.startsWith("end")) { //Stay in manual mode until "exit"
//      dataIn = Serial.readString();

//       //ACTUATOR Up Functionality      
//       if (dataIn.startsWith("u")) {
//       dataInS = dataIn.substring(1, dataIn.length()); // Reads up value "u13"
//       dist = dataInS.toInt();
//       Serial.print("Move up ");
//       Serial.print(dist);
//       Serial.println(" steps");
//       while (Actuator.currentPosition() != dist) {
//         Actuator.setSpeed(1000);
//         Actuator.run();
//       }
//       Actuator.setCurrentPosition(0);
//       delay(100);
//     }

//     //ACTUATOR Down Functionality Functionality      
//       if (dataIn.startsWith("d")) {
//       dataInS = dataIn.substring(1, dataIn.length()); // Reads up value "u13"
//       dist = ((dataInS.toInt()) * -1);
//       Serial.print("Move Down ");
//       Serial.print(dist);
//       Serial.println(" steps");
//       while (Actuator.currentPosition() != dist) {
//         Actuator.setSpeed(-1000);
//         Actuator.run();
//         // if (!digitalRead(home_switch)) {
//         //   return;
//         // }
//       }
//       Actuator.setCurrentPosition(0);
//       delay(100);
//     }

//         //LOWER CAROUSEL CW Functionality      
//       if (dataIn.startsWith("l")) {
//       dataInS = dataIn.substring(1, dataIn.length()); // Reads up value "l13"
//       dist = ((dataInS.toInt()) * 1);
//       Serial.print("Move Down ");
//       Serial.print(dist);
//       Serial.println(" steps");
//       while (Lowcar.currentPosition() != dist) {
//         Lowcar.setMaxSpeed(2000);
//         Lowcar.setSpeed(500);
//         Lowcar.run();
//         // if (!digitalRead(home_switch)) {
//         //   return;
//         // }
//       }
//       Lowcar.setCurrentPosition(0);
//       delay(100);
//     }

//     //High CAROUSEL CW Functionality      
//       if (dataIn.startsWith("h")) {
//       dataInS = dataIn.substring(1, dataIn.length()); // Reads up value "h13"
//       dist = ((dataInS.toInt()) * 1);
//       Serial.print("Move Down ");
//       Serial.print(dist);
//       Serial.println(" steps");
//       while (Highcar.currentPosition() != dist) {
//         Highcar.setMaxSpeed(2000);
//         Highcar.setSpeed(500);
//         Highcar.run();
//         // if (!digitalRead(home_switch)) {
//         //   return;
//         // }
//       }
//       Highcar.setCurrentPosition(0);
//       delay(100);
//     }

//     //Pump Functionality      
//       if (dataIn.startsWith("p")) {
//       dataInS = dataIn.substring(1, dataIn.length()); // Reads pump value "p13"
//       dist = ((dataInS.toDouble()) * 1);
//       Serial.print("Pumping ");
//       Serial.print(dist);
//       Serial.println(" steps");
//       while (Pump.currentPosition() != dist) {
//         Pump.setMaxSpeed(5000);
//         Pump.setSpeed(5000);
//         Pump.run();
//         // if (!digitalRead(home_switch)) {
//         //   return;
//         // }
//       }
//       // Pump.setMaxSpeed(4000);
//       // Pump.setAcceleration(500); 
//       // Pump.moveTo(29000);
//       // Pump.runToPosition();
//       // Pump.setCurrentPosition(0);
//       // delay(5);
//       delay(100);
//     }
//    }
//    Serial.println("exiting Manual mode");
  
//    return;
// }

// //Homing Function. Dog shit rn. 
// void homeAc() {
//   dataIn ="";
//   Actuator.setSpeed(250);
//   Actuator.setAcceleration(100);

//   Serial.print("Actuator is Homing...........");

//   while (!digitalRead(home_switch)) { // Move Actuator CCW until switch triggered
//     // Serial.println("Moving to switch...........");
//     Actuator.moveTo(initial_homing); //Set the position to move to
//     initial_homing--; 
//     Actuator.run();
//     delay(5);
//   }

//   Actuator.setCurrentPosition(0);
//   Actuator.setMaxSpeed(1000);
//   Actuator.setAcceleration(100);
//   initial_homing=1;
  
//   while (digitalRead(home_switch)) { //Move CW until limit is released
//     //Serial.println("Moving away from switch...........");
//     Actuator.moveTo(initial_homing);
//     Actuator.run();
//     initial_homing++;
//     delay(5);
//   }

//   Actuator.setCurrentPosition(0);
//   Serial.println("Homing Completed");
//   Actuator.setMaxSpeed(1000);
//   Actuator.setAcceleration(100);

//   Serial.println("Returning to mode Selection");

//   return; 
// }

// // Test mode for Hall sensors 
// void hall() {
  
//   //
//   while (!dataIn.startsWith("end")) {

//     while (digitalRead(lower_carousel_homeSensor)) {
//       Serial.println("no magnet detected");
//       if (!digitalRead(lower_carousel_homeSensor)) {
//         Serial.println("Magnet Detected!");
//       }    
//     }

//     // while (magnetDetected == true) {
//     //   Serial.println("Magnet Detected!");
//     //   if (digitalRead(lower_carousel_homeSensor)) {
//     //     magnetDetected = false;
//     //   }
//     // }
//   }
  
// }