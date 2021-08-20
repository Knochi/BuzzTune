#include <Adafruit_DotStar.h>
#include "ArduinoLowPower.h"
#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define HighPIN     4
#define LowPIN      1
#define onPointPIN  3
#define wakePIN     HighPIN

//effects (see https://www.ti.com/lit/ds/slos854d/slos854d.pdf page 63)
#define EFF_2LOW    18 //Strong Click 2
#define EFF_POINT   52 //Pulsing Strong 1
#define EFF_2HIGH   28 //Short Double Click Strong 2
#define EFF_DLY_MS  500 //delay between effect playbacks
//DotStar PIN Definitions
#define DATAPIN    7
#define CLOCKPIN   8

#define DBG_LVL   0 //0: no Serial, 1: one time messages, 2: cyclic messages

//init dotStar LED
Adafruit_DotStar dot(1, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

//init Haptic driver
Adafruit_DRV2605 drv;

//globals
bool High, Low, onPoint;
  
void setup() {
  //turn off dotstar
  dot.begin(); // Initialize pins for output
  dot.setBrightness(80);
  dot.clear();
  dot.show();  // Turn all LEDs off ASAP

  //setup pins
  pinMode(HighPIN, INPUT);
  pinMode(LowPIN, INPUT);
  pinMode(onPointPIN, INPUT);
  
  //setup LowPower
  LowPower.attachInterruptWakeup(wakePIN, wake_CB, CHANGE);
  //setup haptic driver
  drv.begin();
  drv.selectLibrary(1);

  #if (DBG_LVL)
    Serial.begin(9600);
    while(!Serial){
      ;
    }
  #endif
  AutoCal_LRA(2.0,2.5,204);
  Init_LRA();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  High = digitalRead(HighPIN);
  Low = digitalRead(LowPIN);
  onPoint = digitalRead(onPointPIN);

  #if (DBG_LVL>1)
    Serial.print("High:");
    Serial.print(High); //blue
    Serial.print("\t");
    Serial.print("Low:");
    Serial.print(Low); //red
    Serial.print("\t");
    Serial.print("onPoint:");
    Serial.println(onPoint); //green
  #endif
  
  //sleep when all pins are high level 
  if (High&&Low&&onPoint){
    digitalWrite(LED_BUILTIN,LOW);
    LowPower.sleep();
    
  }
  else{ //one of the leds are lit
    if (!High)
      drv.setWaveform(0, EFF_2HIGH);  // play effect 
    else if (!onPoint)
      drv.setWaveform(0, EFF_POINT);  // play effect
    else if (!Low)
      drv.setWaveform(0, EFF_2LOW);  // play effect
    
    drv.setWaveform(1, 0);       // end waveform
    drv.go();
    delay(2500); //wait 5s
  }
  
  delay(500);
  
}

void wake_CB(){
  digitalWrite(LED_BUILTIN,HIGH);
}

//initialiaze LRA Vibra after AutoCal
void Init_LRA(){                                  //(int Comp, int BEMF) {
  #if (DBG_LVL)
    Serial.println("Init LRA");
  #endif  
  //drv.writeRegister8(DRV2605_REG_RATEDV, 0x90); //3V
  //drv.writeRegister8(DRV2605_REG_CLAMPV, 0xA4); //3.6V Overdrive
  drv.writeRegister8(DRV2605_REG_FEEDBACK, 0xB6); //ERM default settings
  //drv.writeRegister8(DRV2605_REG_AUTOCALCOMP, LRA_COMP); //AutoCal Compensation results
  //drv.writeRegister8(DRV2605_REG_AUTOCALEMP, Comp); //Autocal Back EMF results
  drv.writeRegister8(DRV2605_REG_CONTROL1, 0x13); //Boost off, DC Coupling, DriveTime=19
  drv.writeRegister8(DRV2605_REG_CONTROL2, 0xF5); //default settings
  drv.writeRegister8(DRV2605_REG_CONTROL3, 0x80); //default settings, PWM input, Auto-Resonance
  drv.writeRegister8(DRV2605_REG_LIBRARY, 0x06); // LRA Library 
  drv.writeRegister8(DRV2605_REG_MODE, 0x00); // Mode Ready
}

// rated Voltage, Overdrive Voltage, LRA resonat Freq
void AutoCal_LRA(float f_rVolt, float f_ovVolt, int fLRA) {
  //sampletime 300µs
  int rVolt = f_rVolt * sqrt(1-(4*(300*1e-6)+300*1e-6)*fLRA) / (20.71*1e-3); //calculate rated Voltage
  int ovVolt = f_ovVolt / ((21.33*1e-3)*sqrt(1-fLRA*800*1e-6)); //calculate Overdrive Voltage
  #if (DBG_LVL)
    Serial.print("Calibrating with rVolt=");Serial.print(f_rVolt);Serial.print("V and ovVolt=");Serial.print(f_ovVolt); Serial.println("V.");
    Serial.println("----- Starting Autocal for LRA ------");
  #endif  
  int x=0;
  drv.writeRegister8(DRV2605_REG_RATEDV, rVolt); 
  drv.writeRegister8(DRV2605_REG_CLAMPV, ovVolt);
  drv.writeRegister8(DRV2605_REG_FEEDBACK, 0xB6); //Feedback Control (LRA, 4x Brake, LoopGain Medium, BEMFGain 1.8x/20x
  
  drv.writeRegister8(DRV2605_REG_CONTROL1, 0x93); //Boost on, DC Coupling, Drive Time=19
  drv.writeRegister8(DRV2605_REG_CONTROL2, 0x1C); //default settings
  drv.writeRegister8(DRV2605_REG_CONTROL3, 0x80); //ClosedLoop default
 
  drv.writeRegister8(DRV2605_REG_MODE, 0x07); // Mode AutoCal
  drv.writeRegister8(DRV2605_REG_CONTROL4, 0xA0); //Time 500ms, OTP Read Only, do not programm OTP

  drv.go(); //start the Autocal

  //wait for autocal to complete
  do{
      x = drv.readRegister8(DRV2605_REG_GO); //Poll GO Bit for "0"
      delay(100);
    }while(x);
    
  x = drv.readRegister8(DRV2605_REG_STATUS); //read status

  #if (DBG_LVL)
    //-- error Detection --
    if (x & 0x08) Serial.println ("Failed");
    else Serial.println("Suceeded");
    
    if (x & 0x04) Serial.println ("Feedback TimeOut");
    if (x & 0x02) Serial.println ("OverTemp");
    if (x & 0x01) Serial.println ("OverCurrent");
    
    
    //-- Results
    Serial.print("Compensation: "); Serial.println(drv.readRegister8(DRV2605_REG_AUTOCALCOMP));
    Serial.print("Back EMF: "); Serial.println(drv.readRegister8(DRV2605_REG_AUTOCALEMP));
    Serial.print("Feedback: "); Serial.println(drv.readRegister8(DRV2605_REG_FEEDBACK));
    
    Serial.println("----- End of Autocal for LRA ------");
  #endif  
  drv.writeRegister8(DRV2605_REG_MODE, 0x00); // Mode Ready
}
