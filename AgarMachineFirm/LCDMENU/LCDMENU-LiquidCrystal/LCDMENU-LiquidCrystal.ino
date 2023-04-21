#include <Arduino.h>

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
using namespace Menu;


#include <SPI.h>
#define fontName u8g2_font_7x13_mf
#define fontX 7
#define fontY 16
#define offsetX 0
#define offsetY 3
#define U8_Width 128
#define U8_Height 64
#define fontMarginX 2
#define fontMarginY 2
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


//Motor Tuning 
MENU(motorTuning,"Motor Tuning",showEvent,anyEvent,noStyle
  ,OP("Actuator   Speed",showEvent,anyEvent)
  ,OP("Carousel 1 Speed",showEvent,anyEvent)
  ,OP("Carousel 2 Speed",showEvent,anyEvent)
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

int plateCount=50;

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








void setup() {
  pinMode(encBtn,INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  // u8g2.setBitmapMode(1);
  Serial.begin(9600);
  // while(!Serial);
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
}
void loop() {
  switch (exitMenuOptions) {
  case 1: {
      delay(500); // Pause to allow the button to come up 
      startCycle(); 
      break;
    }
  }

  nav.doInput();
  if (nav.changed(0)) {//only draw if menu changed for gfx device
    u8g2.firstPage();
    do nav.doOutput(); while(u8g2.nextPage());
  }
}