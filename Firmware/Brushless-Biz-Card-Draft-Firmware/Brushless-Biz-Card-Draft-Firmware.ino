/*    
    Brushless Motor and Driver Business Card
    Andy Geppert, Machine Ideas, June 2024
    V0.1 board, S/N 1
    https://hackaday.io/project/196576-brushless-motor-and-driver-business-card-kit
    https://github.com/ageppert/Brushless-Motor-Biz-Card
      
    Using 328PB from Arduino Uno clone, 16 MHz resonator
    Additional Boards Manager URL: https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
    https://github.com/MCUdude/MiniCore?tab=readme-ov-file
    Arduino IDE 1.8.19 / Board / MiniCore / AtMega 328
    Variant PB

    Code started life as:
      Program to run a brushless motor in open loop mode, three approximated sine wave PWM sequences, 120 degrees out of phase
      By Juan Pablo Angulo https://www.patreon.com/randomaccessprojects
      ST L6234 Brushless Driver (like SimpleFOC shield). Three PWM outputs to control high and low side FETs. When high, high side FET is enabled. When low, low side FET is enabled.
    Hacked a bunch by Andy Geppert for Brushless Motor and Driver Business Card
*/

// USER RGB LED
  #define PIN_LED_GRB   13
  #include <Adafruit_NeoPixel.h>
  #define NUM_LEDS       1
  Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED_GRB, NEO_GRB + NEO_KHZ800);
  int ledBlinkState = 0;
  unsigned long previousMillis = 0;
  unsigned long currentMillis = 0;
  const long interval = 500;

// SAO OLED I2C 0x3C
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  // On an arduino UNO:       A4(SDA), A5(SCL)
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Analog input option, not used
const int potPin = A1;  // INPUT pot control for speed or position

// Digital Write FAST!!!
  #include <digitalWriteFast.h>
  uint8_t pinGateAH =  5; // Set high to disable upper PFET, low to enable. Using only LOW/HIGH digital control. No need to PWM these.
  uint8_t pinGateBH =  3;
  uint8_t pinGateCH =  2;

// PWM
  uint8_t pinGateAL =  9; // Set low to disable lower NFET, high to enable. These are modulated with PWM.
  uint8_t pinGateBL = 10;
  uint8_t pinGateCL = 11;

// MOTOR CONTROL Variables
  #define POWER_SCALAR 0.5          // Between 0 and 1 to reduce the peak PWM value, aka voltage.
  uint8_t stepDelay = 5; // Number of milliseconds between steps through the electrical cycle sine array
  #define PWM_SINE_ARRAY_LENGTH 49    // from 0 to 48
  int pwmSin[PWM_SINE_ARRAY_LENGTH] = {127,110,94,78,64,50,37,26,        17,10,4,1,0,1,4,10,              17,26,37,50,64,78,94,110,  127,144,160,176,191,204,217,228,   
                  237,244,250,253,255,253,250,244,  237,228,217,204,191,176,160,144  ,127}; // array of PWM duty values for 8-bit timer - sine function
  int currentStepA=0; //initial pointer at 0   degrees for coil A
  int currentStepB=16;//initial pointer at 120 degrees for coil B
  int currentStepC=32;//initial pointer at 240 degrees for coil C
  int pos;

void setup() {
  // Serial Port
    Serial.begin(115200);
  // EEPROM built in to AT328PB

  // LEDs
    pixels.begin();
    pixels.clear();
    pixels.show();
    ledGreen();
    delay(500);

  // Detect board type through built-in MCU EEPROM
  
  // Configure hardware per EEPROM
    // Default state of pins should be floating, and external (on-board) hardware should provide safe starting states
    // Inputs
      pinMode(potPin, INPUT);

    // Capacitive touch buttons

    // PWM Frequency
      TCCR0B = TCCR0B & 0b11111000 | 0x03; // changing this will also affect millis() and delay(), better to leave it default (0x03).
      TCCR1B = TCCR1B & 0b11111000 | 0x03; // set PWM frequency @ 31250 Hz for Pins 9 and 10, (0x03 is default value, gives 490 Hz).
      TCCR2B = TCCR2B & 0b11111000 | 0x03; // set PWM frequency @ 31250 Hz for Pins 11 and 3, (0x03 is default value, gives 490 Hz).
      ICR1 = 255 ; // 8 bit resolution for PWM

    // OUTPUT PINS
      pinMode(pinGateAL, OUTPUT);
      pinMode(pinGateBL, OUTPUT);
      pinMode(pinGateCL, OUTPUT);
      // pinMode(pinGateAH, OUTPUT);
      // pinMode(pinGateBH, OUTPUT);
      // pinMode(pinGateCH, OUTPUT);
      pinModeFast(pinGateAH, OUTPUT);
      pinModeFast(pinGateBH, OUTPUT);
      pinModeFast(pinGateCH, OUTPUT);
      pwmAllDisable();

  // OLED SETUP
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
    }
    else {
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F(" Brushless"));
      display.println(F("   Motor"));
      display.setCursor(7, 32);
      display.println(F(" Biz Card"));
      display.setTextSize(1);
      display.setCursor(22, 48);
      display.println(F("By Andy Geppert")); 
      display.println(F(" www.MachineIdeas.com")); 
      display.display();      
    }
    ledBlue();
    delay(500);
} // END OF SETUP FUNCTION
 
void loop() {
  // MAIN APPLICATION MODE
    // HOUSEKEEPING
      // Blink to look alive
        currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;
          if (ledBlinkState == 0) { ledBlinkState = 1; ledRed();}
          else {ledBlinkState = 0; ledOff(); }
        }
      // check the buttons
      // read the potentiometer (if active)
      // read the PWM input (if active)
      // serial port management
    
    // STATE MANAGER
    // pwmAllDisable();
    // move();
    // phaseABrampUpDownTriangle();
    // phaseABrampUpDownSine(stepDelay);
    PhaseAHighSideSine(stepDelay);
    // spinTheMotorBlindlySineWave(stepDelay);

    delay (10);
    
} // END OF MAIN LOOP FUNCTION

void spinTheMotorBlindlyTrapezoidal(uint8_t value)
{
  // Step One
  
}

void spinTheMotorBlindlySineWave(uint8_t value)
{
  /* H-Bridges and 6 step sequence
      AH Q1    BH Q3   CH  Q5  (high side transistors are only on or off, no PWM needed)
          U        V        W
      AL Q2    BL Q4   CL  Q6  (low side transistors are PWM modulated)

  Step  Q1  Q2  Q3  Q4  Q5  Q6
        +   -   +   -   +   -
  1     on          pwm
  2     on                  pwm
  3             on          pwm
  4         pwm on
  5         pwm         on
  6                 pwm on
  */
  currentStepA = currentStepA + 1;  // Add 1 to make the motor move step by step.
  currentStepB = currentStepA + 16; // add 120 deg of phase to whatever position StepA is. 
  currentStepC = currentStepA + 32; // add 240 deg of phase to whatever position StepA is.
  
  currentStepA = currentStepA%48; // I used remainder operation or modulo to "wrap" the values between 0 and 47
  currentStepB = currentStepB%48;
  currentStepC = currentStepC%48;

  analogWrite(pinGateAL, pwmSin[currentStepA]*POWER_SCALAR);
  analogWrite(pinGateBL, pwmSin[currentStepC]*POWER_SCALAR);
  analogWrite(pinGateCL, pwmSin[currentStepB]*POWER_SCALAR);

}

void PhaseAHighSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < PWM_SINE_ARRAY_LENGTH; i++) {
    analogWrite(pinGateCH,pwmSin[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWriteFast(pinGateCH, HIGH);
}

void phaseABrampUpDownSine(uint8_t value)
{
  for( uint8_t i = 0; i < PWM_SINE_ARRAY_LENGTH; i++) {
    pwmAtoBEnable(pwmSin[i]);
    delay(value);
  }
  pwmAtoBEnable(0);
}

void phaseABrampUpDownTriangle()
{
  for( uint8_t i = 0; i < 255; i++) {
    pwmAtoBEnable(i);
    delay(1);
  }
  for( uint8_t i = 255; i > 0; i--) {
    pwmAtoBEnable(i);
    delay(1);
  }
  pwmAtoBEnable(0);

  for( uint8_t i = 0; i < 255; i++) {
    pwmBtoAEnable(i);
    delay(1);
  }
  for( uint8_t i = 255; i > 0; i--) {
    pwmBtoAEnable(i);
    delay(1);
  }
  pwmBtoAEnable(0);
}

void pwmAtoBEnable(uint8_t value)
{
  if (value>0) {
    // analogWrite(pinGateAH,0);       // A high side enable solid
    digitalWriteFast(pinGateAH, LOW);
    analogWrite(pinGateBL,value);   // B low side enable PWM
  }
  else {
    // analogWrite(pinGateAH,255);     // A high side disable solid
    digitalWriteFast(pinGateAH, HIGH);
    analogWrite(pinGateBL,0);       // B low side disable PWM
  }
}

void pwmBtoAEnable(uint8_t value)
{
  if (value>0) {
    // analogWrite(pinGateBH,0);       // A high side enable solid
    digitalWriteFast(pinGateBH, LOW);
    analogWrite(pinGateAL,value);   // B low side enable PWM
  }
  else {
    // nalogWrite(pinGateBH,255);     // A high side disable solid
    digitalWriteFast(pinGateBH, HIGH);
    analogWrite(pinGateAL,0);       // B low side disable PWM
  }
}

void pwmAllDisable()
{
  // analogWrite(pinGateAH,255);
  // analogWrite(pinGateBH,255);
  // analogWrite(pinGateCH,255);
  digitalWriteFast(pinGateAH, HIGH);
  digitalWriteFast(pinGateBH, HIGH);
  digitalWriteFast(pinGateCH, HIGH);
  analogWrite(pinGateAL,0);
  analogWrite(pinGateBL,0);
  analogWrite(pinGateCL,0);  
}

void move()
{
  // By Juan Pablo Angulo https://www.patreon.com/randomaccessprojects
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
  delay(5); 

  //This will give you open loop speed control with the potentiometer
  //delay(sensorValue/10);

  //This will give you open loop position control with the potentiometer
  currentStepA = sensorValue/5; //divide by a number to affect the ratio of pot position : motor position

  //Serial.println(currentStepA);
  //pos=pulseIn(encoder,HIGH);
  //Serial.println(pos);
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

void ledOff()
{
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
}

void ledRed()
{
  pixels.setPixelColor(0, pixels.Color(3, 0, 0));
  pixels.show();
}

void ledGreen()
{
  pixels.setPixelColor(0, pixels.Color(0, 3, 0));
  pixels.show();
}

void ledBlue()
{
  pixels.setPixelColor(0, pixels.Color(0, 0, 3));
  pixels.show();
}
