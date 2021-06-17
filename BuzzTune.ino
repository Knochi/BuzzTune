#include <Adafruit_DotStar.h>
#include "ArduinoLowPower.h"
#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define HighPIN     4
#define LowPIN      1
#define onPointPIN  3
#define wakePIN     HighPIN

//DotStar PIN Definitions
#define DATAPIN    7
#define CLOCKPIN   8

//init dotStar LED
Adafruit_DotStar dot(1, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

//init Haptic driver
Adafruit_DRV2605 drv;


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
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  High = digitalRead(HighPIN);
  Low = digitalRead(LowPIN);
  onPoint = digitalRead(onPointPIN);
  Serial.print("High:");
  Serial.print(High); //blue
  Serial.print("\t");
  Serial.print("Low:");
  Serial.print(Low); //red
  Serial.print("\t");
  Serial.print("onPoint:");
  Serial.println(onPoint); //green

  //sleep when all pins are high level 
  if (High&&Low&&onPoint){
    digitalWrite(LED_BUILTIN,LOW);
    LowPower.sleep();
    
  }
  delay(100);
}

void wake_CB(){
  digitalWrite(LED_BUILTIN,HIGH);
}

//initialiaze LRA Vibra after AutoCal
void Init_LRA(){                                  //(int Comp, int BEMF) {
  Serial.println("Init LRA");
  //drv.writeRegister8(DRV2605_REG_RATEDV, 0x90); //3V
  //drv.writeRegister8(DRV2605_REG_CLAMPV, 0xA4); //3.6V Overdrive
  drv.writeRegister8(DRV2605_REG_FEEDBACK, 0xB6); //ERM default settings
  //drv.writeRegister8(DRV2605_REG_AUTOCALCOMP, Comp); //AutoCal Compensation results
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
  
  Serial.print("Calibrating with rVolt=");Serial.print(f_rVolt);Serial.print("V and ovVolt=");Serial.print(f_ovVolt); Serial.println("V.");
  Serial.println("----- Starting Autocal for LRA ------");
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

  //-- error Detection --
  if (x & 0x08) Serial.println ("Failed");
  if (x & 0x04) Serial.println ("Feedback TimeOut");
  if (x & 0x02) Serial.println ("OverTemp");
  if (x & 0x01) Serial.println ("OverCurrent");
  
  //-- Results
  Serial.print("Compensation: "); Serial.println(drv.readRegister8(DRV2605_REG_AUTOCALCOMP));
  Serial.print("Back EMF: "); Serial.println(drv.readRegister8(DRV2605_REG_AUTOCALEMP));
  Serial.print("Feedback: "); Serial.println(drv.readRegister8(DRV2605_REG_FEEDBACK));
  
  drv.writeRegister8(DRV2605_REG_MODE, 0x00); // Mode Ready
  Serial.println("----- End of Autocal for LRA ------");
}
