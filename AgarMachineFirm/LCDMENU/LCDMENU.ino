#include <Arduino.h>

//Encoder library to read reprap embedded encoder 
#include <ClickEncoder.h>
#include <TimerOne.h>

//Arduino Menu Includes, from u8g2 example 
#include <menu.h>
#include <menuIO/u8g2Out.h>
// #include <menuIO/encoderIn.h>
// #include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>

using namespace Menu;

// Define LCD pins 
#define encA    23
#define encB    17
#define encBtn  16

// Designate Display ST7920 in this case 
// SPI allows us to communicate with SPI devices with the arduino as the controller 
#include <SPI.h>

// Not sure of the specifics here consult https://github.com/olikraus/u8g2
#define fontName u8g2_font_5x7_tf
#define fontX 5
#define fontY 9
#define offsetX 0
#define offsetY 0
#define U8_Width 84
#define U8_Height 48
// INIT lcd 
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, encA , encB, encBtn);

// Create encoder instance
ClickEncoder *encoder;
// Value of encoder
int16_t last, value;

void timerIsr() {
  encoder->service();
}

// define menu colors --------------------------------------------------------
//each color is in the format:
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
// this is a monochromatic color table
const colorDef<uint8_t> colors[6] MEMMODE={
  {{0,0},{0,1,1}},//bgColor
  {{1,1},{1,0,0}},//fgColor
  {{1,1},{1,0,0}},//valColor
  {{1,1},{1,0,0}},//unitColor
  {{0,1},{0,0,1}},//cursorColor
  {{1,1},{1,0,0}},//titleColor
};

// NO IDEA WHAT THIS DOES 
result doAlert(eventMask e, prompt &item);

// Menu Variables 
int test=55;
int ledCtrl=HIGH;

result myLedOn() {
  ledCtrl=HIGH;
  return proceed;
}

result myLedOff() {
  ledCtrl=LOW;
  return proceed;
}

// TOGGLES ledCtril value 
TOGGLE(ledCtrl,setLed,"Led: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("On",HIGH,doNothing,noEvent)
  ,VALUE("Off",LOW,doNothing,noEvent)
);

//Selection Test? 
int selTest=0;
SELECT(selTest,selMenu,"Select",doNothing,noEvent,noStyle
  ,VALUE("Zero",0,doNothing,noEvent)
  ,VALUE("One",1,doNothing,noEvent)
  ,VALUE("Two",2,doNothing,noEvent)
);

// Choose Test? 
int chooseTest=-1;
CHOOSE(chooseTest,chooseMenu,"Choose",doNothing,noEvent,noStyle
  ,VALUE("First",1,doNothing,noEvent)
  ,VALUE("Second",2,doNothing,noEvent)
  ,VALUE("Third",3,doNothing,noEvent)
  ,VALUE("Last",-1,doNothing,noEvent)
);

// Defines a menu? 
MENU(subMenu,"Sub-Menu",doNothing,noEvent,noStyle
  ,OP("Sub1",doNothing,noEvent)
  // ,altOP(altPrompt,"",doNothing,noEvent)
  ,EXIT("<Back")
);

uint16_t hrs=0;
uint16_t mins=0;

//define a pad style menu (single line menu)
//here with a set of fields to enter a date in YYYY/MM/DD format
altMENU(menu,timeMenu,"Time",doNothing,noEvent,noStyle,(systemStyles)(_asPad|Menu::_menuData|Menu::_canNav|_parentDraw)
  ,FIELD(hrs,"",":",0,11,1,0,doNothing,noEvent,noStyle)
  ,FIELD(mins,"","",0,59,10,1,doNothing,noEvent,wrapStyle)
);

// No Clue 
char* constMEM hexDigit MEMMODE="0123456789ABCDEF";
char* constMEM hexNr[] MEMMODE={"0","x",hexDigit,hexDigit};
char buf1[]="0x11";

// Defines main menu 
MENU(mainMenu,"Main menu",doNothing,noEvent,wrapStyle
  ,OP("Op1",doNothing,noEvent)
  ,OP("Op2",doNothing,noEvent)
  //,FIELD(test,"Test","%",0,100,10,1,doNothing,noEvent,wrapStyle)
  ,SUBMENU(timeMenu)
  ,SUBMENU(subMenu)
  ,SUBMENU(setLed)
  ,OP("LED On",myLedOn,enterEvent)
  ,OP("LED Off",myLedOff,enterEvent)
  ,SUBMENU(selMenu)
  ,SUBMENU(chooseMenu)
  ,OP("Alert test",doAlert,enterEvent)
  ,EDIT("Hex",buf1,hexNr,doNothing,noEvent,noStyle)
  ,EXIT("<Exit")
);

// No idea 
#define MAX_DEPTH 2

// Wacky shit going on Here, I think encoder input to the menu is being defined? 

// encoderIn<encA,encB> encoder;//simple quad encoder driver
// encoderInStream<encA,encB> encStream(encoder,4);// simple quad encoder fake Stream

//a keyboard with only one key as the encoder button
// keyMap encBtn_map[]={{-encBtn,defaultNavCodes[enterCmd].ch}};//negative pin numbers use internal pull-up, this is on when low
// keyIn<1> encButton(encBtn_map);//1 is the number of keys

// menuIn* inputsList[]={&encBuitton,&Serial};
// chainStream<2> in(inputsList);//1 is the number of inputs

serialIn serial(Serial);
MENU_INPUTS(in,&serial);
// MENU_INPUTS(in,&encStream,&encButton);//,&serial);

MENU_OUTPUTS(out,MAX_DEPTH
  ,U8G2_OUT(u8g2,colors,fontX,fontY,offsetX,offsetY,{0,0,U8_Width/fontX,U8_Height/fontY})
  ,SERIAL_OUT(Serial)
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

result alert(menuOut& o,idleEvent e) {
  if (e==idling) {
    o.setCursor(0,0);
    o.print("alert test");
    o.setCursor(0,1);
    o.print("press [select]");
    o.setCursor(0,2);
    o.print("to continue...");
  }
  return proceed;
}

result doAlert(eventMask e, prompt &item) {
  nav.idleOn(alert);
  return proceed;
}

//when menu is suspended
result idle(menuOut& o,idleEvent e) {
  o.clear();
  switch(e) {
    case idleStart:o.println("suspending menu!");break;
    case idling:o.println("suspended...");break;
    case idleEnd:o.println("resuming menu.");break;
  }
  return proceed;
}

void setup() {
  Serial.begin(9600);
  
  // Initialize Encoder
  encoder = new ClickEncoder(33, 31, 35);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  last = -1;

  SPI.begin();
  u8g2.begin();

  u8g2.setFont(fontName);

  // disable second option
  mainMenu[1].enabled=disabledStatus;
  nav.idleTask=idle;//point a function to be used when menu is suspended
  Serial.println("setup done.");Serial.flush();
}




void loop() {  
  //Get Current value of encoder 
  value += encoder->getValue();
  
  //MENU HERE? 
  
  nav.doInput();
  // digitalWrite(LEDPIN, ledCtrl);
  if (nav.changed(0)) {//only draw if menu changed for gfx device
    //change checking leaves more time for other tasks
    u8g2.firstPage();
    do nav.doOutput(); while(u8g2.nextPage());
  }

  //END MENU 

  //Update Global value "last" with current value of encoder
  if (value != last) {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);
  }
  
  //Encoder button handler 
  ClickEncoder::Button b = encoder->getButton();
  // Open is the default unpressed state of the button 
  if (b != ClickEncoder::Open) {
    Serial.print("Button: ");
    #define VERBOSECASE(label) case label: Serial.println(#label); break;
    switch (b) {
      VERBOSECASE(ClickEncoder::Pressed);
      VERBOSECASE(ClickEncoder::Held)
      VERBOSECASE(ClickEncoder::Released)
      VERBOSECASE(ClickEncoder::Clicked)
      case ClickEncoder::DoubleClicked:
          Serial.println("ClickEncoder::DoubleClicked");
          encoder->setAccelerationEnabled(!encoder->getAccelerationEnabled());
          Serial.print("  Acceleration is ");
          Serial.println((encoder->getAccelerationEnabled()) ? "enabled" : "disabled");

        break;
    }
  }    
}
