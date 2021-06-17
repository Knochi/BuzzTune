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
