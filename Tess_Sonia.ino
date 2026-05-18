/******************************************************************************
Ultra-sound HC-SR04 + FastLED Arduino
For UCA21
Fabien Ferrero
Aug, 2021

Distributed as-is; no warranty is given.

Use HC-SR04
Trig is connected on A2
Echo is connected on A3

Objective : This code will :
    * Measure the distance with HC-SR04
    * use the LED to illustrate the distance (RED is close, blue is far)
******************************************************************************/

// Your sketch must #include this library, and the Wire library
// (Wire is a standard library included with Arduino):

#include <FastLED.h> // http://librarymanager/All#FASTLED
#include "pitches.h"
#define Buzzer 8


#define LED_PIN     4
#define NUM_LEDS    21
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;



const int trigPin = A3;  
const int echoPin = A2;
float duration, distance;   

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
  delay( 1000 ); // power-up safety delay
  // Initialize the Serial port:
  
  Serial.begin(115200);
  Serial.println("HC-SR04 example sketch");

  // Initialize the HC-SR04 pin
  
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 


// Setup LED

 FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
 FastLED.setBrightness(  BRIGHTNESS );
    
 currentPalette = RainbowColors_p;
 currentBlending = LINEARBLEND;

}

void loop() {
 
  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW); 

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;   // The speed of sound is approximately 340 meters per second, but since the pulseIn() function returns the time in microseconds
  Serial.println(distance);
  if(distance<50.00){
    Serial.println("Présence");

     // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(Buzzer, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(Buzzer);
  }
  }
  
  uint8_t dist_temp = map(distance,0,200,0,255); // Map value from distance sensor to LED

  // FastLED's built-in rainbow generator
  fill_solid( leds, NUM_LEDS, ColorFromPalette(RainbowColors_p,dist_temp,BRIGHTNESS, LINEARBLEND));

  
// send the 'leds' array out to the actual LED strip
  FastLED.show();
  delay( 100 ); // power-up safety delay

}






