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

#define DEBUG 0

// USER RGB LED
  #define PIN_LED_GRB   13
  #include <Adafruit_NeoPixel.h>
  #define NUM_LEDS       1
  Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED_GRB, NEO_GRB + NEO_KHZ800);
  uint8_t  ledBlinkState = 0;
  uint32_t previousMillis = 0;
  uint32_t currentMillis = 0;
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

// PWM
  uint8_t pinGateAH =  5;         // Set high to disable upper PFET, low to enable.
  uint8_t pinGateBH =  3;
  uint8_t pinGateCH =  2;
  uint8_t pinGateAL =  9;         // Set low to disable lower NFET, high to enable.
  uint8_t pinGateBL = 10;
  uint8_t pinGateCL = 11;

// MOTOR CONTROL Variables
  #define POWER_SCALAR 1          // Between 0 and 1 to reduce the peak PWM value, aka voltage.
  uint32_t stepDelay = 1000;          // Number of milliseconds between steps through the electrical cycle sine array

  // STARTED WITH 73 ELEMENTS, BACK TO BACK SINE WAVES.
  int16_t pwmSinePosNeg[] = {0,22,44,65,87,107,127,146,163,180,195,208,220,231,239,246,251,254,255,254,251,246,239,231,220,208,195,180,163,146,127,107,87,65,44,22,0,-23,-45,-66,-88,-108,-128,-147,-164,-181,-196,-209,-221,-232,-240,-247,-252,-255,-255,-255,-252,-247,-240,-232,-221,-209,-196,-181,-164,-147,-128,-108,-88,-66,-45,-23,-1};
  // TESTING WITH 36 MORE ELEMENTS, PRE-PENDED ZEROS.
  // int16_t pwmSinePosNeg[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,22,44,65,87,107,127,146,163,180,195,208,220,231,239,246,251,254,255,254,251,246,239,231,220,208,195,180,163,146,127,107,87,65,44,22,0,-23,-45,-66,-88,-108,-128,-147,-164,-181,-196,-209,-221,-232,-240,-247,-252,-255,-255,-255,-252,-247,-240,-232,-221,-209,-196,-181,-164,-147,-128,-108,-88,-66,-45,-23,-1};
  uint8_t SineArrayElementCount = sizeof(pwmSinePosNeg) / sizeof(pwmSinePosNeg[0]);
  uint8_t currentStepA  =                (SineArrayElementCount *  90 / 360);  // initial array position at 90 degrees for coil A
  uint8_t currentStepB  = currentStepA + (SineArrayElementCount * 120 / 360);  // initial array position at 90 + 120 = 210 degrees for coil B
  uint8_t currentStepC  = currentStepB + (SineArrayElementCount * 120 / 360);  // initial array position at 90 + 240 = 330 degrees for coil C

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

    // Capacitive touch buttons

    // PWM Frequency, assuming ATmega 328PB at 16 MHz.
      TCCR0B = TCCR0B & 0b11111000 | 0x01; // Pins 5 (AH) & 6 (FEEDBACK_COM).                              0x03=977 Hz default. Changing this will also affect millis() and delay(), better to leave it default.
      TCCR1B = TCCR1B & 0b11111000 | 0x01; // Pins 9 (AL) & 10 (BL).          0x01=31763 Hz, 0x02=3921 Hz, 0x03=490 Hz default.
      TCCR2B = TCCR2B & 0b11111000 | 0x01; // Pins 3 (BH) & 11 (CL).          0x01=31763 Hz, 0x02=3921 Hz, 0x03= 1 kHz default.
      TCCR4B = TCCR4B & 0b11111000 | 0x01; // Pin  2 (CH)                     0x01=31763 Hz, 0x02=3921 Hz, 0x03=490 Hz default.
      ICR1 = 255 ; // 8 bit resolution for PWM

    // OUTPUT PINS
      pinMode(pinGateAL, OUTPUT);
      pinMode(pinGateBL, OUTPUT);
      pinMode(pinGateCL, OUTPUT);
      pinMode(pinGateAH, OUTPUT);
      pinMode(pinGateBH, OUTPUT);
      pinMode(pinGateCH, OUTPUT);
      // pinModeFast(pinGateAH, OUTPUT);
      // pinModeFast(pinGateBH, OUTPUT);
      // pinModeFast(pinGateCH, OUTPUT);
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

  // DISPLAY CONFIGURATION PARAMETERS
    Serial.print("Sine Wave Array Element Count: ");
    Serial.println(SineArrayElementCount);
    Serial.print("Sine Wave Array Element Starting Positions (A,B,C): ");
    Serial.print(currentStepA);
    Serial.print(", ");
    Serial.print(currentStepB);
    Serial.print(", ");
    Serial.print(currentStepC);
    Serial.println();
    Serial.print("Sine Wave Array Element Starting PWM Values (A,B,C): ");
    Serial.print(pwmSinePosNeg[currentStepA]);
    Serial.print(", ");
    Serial.print(pwmSinePosNeg[currentStepB]);
    Serial.print(", ");
    Serial.print(pwmSinePosNeg[currentStepC]);
    Serial.println();
    
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
    // PhaseCLowSideSine(stepDelay);
    spinTheMotorBlindlySineWave(stepDelay);

    // delay (10);
    #if DEBUG == 1
      Serial.print(currentStepA);
      Serial.print(", ");
      Serial.print(currentStepB);
      Serial.print(", ");
      Serial.println(currentStepC);
    #endif    
    
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
  currentStepA = currentStepA + 1;  // Once the phasing is set, just increment all of the phases through the array.
  currentStepB = currentStepB + 1;
  currentStepC = currentStepC + 1;
  
  currentStepA = currentStepA % SineArrayElementCount; // Remainder operation or modulo to "wrap" the values between 0 and # of elements in the array.
  currentStepB = currentStepB % SineArrayElementCount;
  currentStepC = currentStepC % SineArrayElementCount;

  SetPwmPhaseA(pwmSinePosNeg[currentStepA]);
  SetPwmPhaseB(pwmSinePosNeg[currentStepB]);
  SetPwmPhaseC(pwmSinePosNeg[currentStepC]);
  #if DEBUG == 2
    Serial.print(pwmSinePosNeg[currentStepA]);
    Serial.print(", ");
    Serial.print(pwmSinePosNeg[currentStepB]);
    Serial.print(", ");
    Serial.print(pwmSinePosNeg[currentStepC]);
    Serial.println();
  #endif
  delay(value);
  delay(value);
  delay(value);
}

void SetPwmPhaseA (int16_t value)
{
  // disable low side, pwm high side, with inverted logic (PWM value 5 converts to 250)
  if ((value > 0) && (value <  256)) {
    digitalWrite(pinGateAL,LOW ); 
    analogWrite(pinGateAH,(255-value) * POWER_SCALAR);
    // Serial.println("     LOW");
    }
  // disable high side, pwm low side, with direct logic (PWM value 5 stays 5)
  if ((value < 0) && (value > -256)) {
    digitalWrite(pinGateAH,HIGH); 
    analogWrite(pinGateAL,abs(value) * POWER_SCALAR);
    // Serial.println("              HIGH)");
    } 
  // disable low and high side
  if (value == 0) { 
    digitalWrite(pinGateAL,LOW ); 
    digitalWrite(pinGateAH,HIGH);
    // Serial.println("                      FLOAT)");
    }
}

void SetPwmPhaseB (int16_t value)
{
  // disable low side, pwm high side, with inverted logic (PWM value 5 converts to 250)
  if ((value > 0) && (value <  256)) {
    digitalWrite(pinGateBL,LOW ); 
    analogWrite(pinGateBH,(255-value) * POWER_SCALAR);
    }
  // disable high side, pwm low side, with direct logic (PWM value 5 stays 5)
  if ((value < 0) && (value > -256)) {
    digitalWrite(pinGateBH,HIGH); 
    analogWrite(pinGateBL,abs(value) * POWER_SCALAR);
    } 
  // disable low and high side
  if (value == 0) { 
    digitalWrite(pinGateBL,LOW ); 
    digitalWrite(pinGateBH,HIGH);
    }
}

void SetPwmPhaseC (int16_t value)
{
  // disable low side, pwm high side, with inverted logic (PWM value 5 converts to 250)
  if ((value > 0) && (value <  256)) {
    digitalWrite(pinGateCL,LOW ); 
    analogWrite(pinGateCH,(255-value) * POWER_SCALAR);
    }
  // disable high side, pwm low side, with direct logic (PWM value 5 stays 5)
  if ((value < 0) && (value > -256)) {
    digitalWrite(pinGateCH,HIGH); 
    analogWrite(pinGateCL,abs(value) * POWER_SCALAR);
    } 
  // disable low and high side
  if (value == 0) { 
    digitalWrite(pinGateCL,LOW ); 
    digitalWrite(pinGateCH,HIGH);
    }
}

void PhaseAHighSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    analogWrite(pinGateAH,pwmSinePosNeg[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWrite(pinGateAH, HIGH);
}

void PhaseBHighSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    analogWrite(pinGateBH,pwmSinePosNeg[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWrite(pinGateBH, HIGH);
}

void PhaseCHighSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    analogWrite(pinGateCH,pwmSinePosNeg[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWrite(pinGateCH, HIGH);
}


void PhaseALowSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    analogWrite(pinGateAL,pwmSinePosNeg[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWrite(pinGateAL, LOW);
}
void PhaseBLowSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    analogWrite(pinGateBL,pwmSinePosNeg[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWrite(pinGateBL, LOW);
}

void PhaseCLowSideSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    analogWrite(pinGateCL,pwmSinePosNeg[i]*POWER_SCALAR);
    delay(value);
  }
  digitalWrite(pinGateCL, LOW);
}

void phaseABrampUpDownSine(uint8_t value)
{
  for( uint8_t i = 0; i < SineArrayElementCount; i++) {
    pwmAtoBEnable(pwmSinePosNeg[i]);
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
    digitalWrite(pinGateAH, LOW);
    analogWrite(pinGateBL,value);   // B low side enable PWM
  }
  else {
    // analogWrite(pinGateAH,255);     // A high side disable solid
    digitalWrite(pinGateAH, HIGH);
    analogWrite(pinGateBL,0);       // B low side disable PWM
  }
}

void pwmBtoAEnable(uint8_t value)
{
  if (value>0) {
    // analogWrite(pinGateBH,0);       // A high side enable solid
    digitalWrite(pinGateBH, LOW);
    analogWrite(pinGateAL,value);   // B low side enable PWM
  }
  else {
    // nalogWrite(pinGateBH,255);     // A high side disable solid
    digitalWrite(pinGateBH, HIGH);
    analogWrite(pinGateAL,0);       // B low side disable PWM
  }
}

void pwmAllDisable()
{
  // analogWrite(pinGateAH,255);
  // analogWrite(pinGateBH,255);
  // analogWrite(pinGateCH,255);
  digitalWrite(pinGateAH, HIGH);
  digitalWrite(pinGateBH, HIGH);
  digitalWrite(pinGateCH, HIGH);
  analogWrite(pinGateAL,0);
  analogWrite(pinGateBL,0);
  analogWrite(pinGateCL,0);  
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
