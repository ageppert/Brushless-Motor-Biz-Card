//Program to run a brushless motor in open loop mode
//By Juan Pablo Angulo
//Ask me questions: https://www.patreon.com/randomaccessprojects
//A simple DIY circuit to run it can be found in my Patreon: https://www.patreon.com/randomaccessprojects
//Thanks for your support!
// ST L6234 Brushless Driver (like SimpleFOC shield). Three outputs to control high and low side FETs. When high, high side FET is enabled. When low, low side FET is enabled.

// Hacked a bunch by Andy Geppert for Brushless Motor and Driver Business Card
// V0.1 board
// Using 328PB from Arduino Uno clone, 16 MHz
// Additional Boards Manager URL: https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
// https://github.com/MCUdude/MiniCore?tab=readme-ov-file
// Board/MiniCore/AtMega 328


#include <Adafruit_NeoPixel.h>
#define PIN_LED_GRB   13
#define NUM_LEDS       1
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED_GRB, NEO_GRB + NEO_KHZ800);

const int potPin = A1;  // INPUT pot control for speed or position
//use ports 9, 10, 11 


// Arduino Pin Assignments
uint8_t pinGateAL =  9;
uint8_t pinGateBL = 10;
uint8_t pinGateCL = 11;
uint8_t pinGateAH =  5;
uint8_t pinGateBH =  3;
uint8_t pinGateCH =  2;


// Variables
int pwmSin[] = {127,110,94,78,64,50,37,26,17,10,4,1,0,1,4,10,17,26,37,50,64,78,94,110,127,144,160,176,191,204,217,228,237,244,250,253,255,253,250,244,237,228,217,204,191,176,160,144,127}; // array of PWM duty values for 8-bit timer - sine function
int currentStepA=0; //initial pointer at 0   degrees for coil A
int currentStepB=16;//initial pointer at 120 degrees for coil B
int currentStepC=32;//initial pointer at 240 degrees for coil C
int pos;


//SETUP
void setup() {
  // Serial Port
    Serial.begin(115200);
  // EEPROM built in to AT328PB
  // LEDs
  pixels.begin();
  pixels.clear();

}
 
void loop() {
  // Detect board type through built-in MCU EEPROM

  // Configure hardware per EEPROM
    // Default state of pins should be floating, and external (on-board) hardware should provide safe starting states
    // Inputs
      pinMode(potPin, INPUT);

    // Outputs: Six PWMs for the three MOSFET half-bridges
      TCCR0B = TCCR0B & 0b11111000 | 0x03; // changing this will also affect millis() and delay(), better to leave it default (0x03).
      TCCR1B = TCCR1B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 9 and 10, (0x03 is default value, gives 490 Hz).
      TCCR2B = TCCR2B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 11 and 3, (0x03 is default value, gives 490 Hz).
      // ensure pin 2 and 5 are configured as well.
      
      ICR1 = 255 ; // 8 bit resolution for PWM
 
      pinMode(pinGateAL, OUTPUT);
      pinMode(pinGateBL, OUTPUT);
      pinMode(pinGateCL, OUTPUT);
      pinMode(pinGateAH, OUTPUT);
      pinMode(pinGateBH, OUTPUT);
      pinMode(pinGateCH, OUTPUT);

  twinkle();

  // MAIN APPLICATION MODE
    // Housekeeping tasks
      // check the buttons
      // read the potentiometer (if active)
      // read the PWM input (if active)
      // serial port management
    // State Manager
      move();
}

void twinkle()
{
  pixels.setPixelColor(0, pixels.Color(3, 0, 0));
  pixels.show();
  delay(1);
  pixels.setPixelColor(0, pixels.Color(0, 3, 0));
  pixels.show();
  delay(1);
  pixels.setPixelColor(0, pixels.Color(0, 0, 3));
  pixels.show();
  delay(1);
}


void move()
{
  currentStepA = currentStepA + 1;  //Add 1 to make the motor move step by step.
  currentStepB = currentStepA + 16; //add 120 deg of phase to whatever position StepA is. 
  currentStepC = currentStepA + 32; //add 240 deg of phase to whatever position StepA is.
  
  currentStepA = currentStepA%48; //I used remainder operation or modulo to "wrap" the values between 0 and 47
  currentStepB = currentStepB%48;
  currentStepC = currentStepC%48;

  analogWrite(pinGateAL, pwmSin[currentStepA]*1); //multipliying by 0.5 to reduce output torque to half as its being supplied with 12V and can get pretty warm. make this 1 if supply is 5V or if you know what you are doing ;)
  analogWrite(pinGateBL, pwmSin[currentStepC]*1);
  analogWrite(pinGateCL, pwmSin[currentStepB]*1);

  //Following send data to PLX-DAQ macro for Excel
  Serial.print("DATA,");
  Serial.print(pwmSin[currentStepA]);
  Serial.print(","); 
  Serial.print(pwmSin[currentStepB]);
  Serial.print(","); 
  Serial.println(pwmSin[currentStepC]);
  

  //Read pot value
  int sensorValue = analogRead(potPin); 

// Select ONLY ONE of the following lines for constant speed, speed control or position control:

  //This will give you constant speed, remember if you changed TCCR0B to 0x01, then delay(64000) = ~1 second
  //delay(5); 

  //This will give you open loop speed control with the potentiometer
  //delay(sensorValue/10);

  //This will give you open loop position control with the potentiometer
  currentStepA = sensorValue/5; //divide by a number to affect the ratio of pot position : motor position

 ////////////

   
//Serial.println(currentStepA);
  //pos=pulseIn(encoder,HIGH);
  //Serial.println(pos);
}
