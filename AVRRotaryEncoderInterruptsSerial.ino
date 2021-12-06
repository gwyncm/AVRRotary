/***************************************************************************************************/
/*
   This is an Arduino sketch for RotaryEncoder library using interrupts

   written by : enjoyneering79
   modified by: gwyncm
   sourse code: https://github.com/enjoyneering/

   This sketch uses interrupts, specials pins are required to interface
   Board:                                    int.0  int.1  int.2  int.3  int.4  int.5            Level
   Uno, Mini, Pro, ATmega168, ATmega328..... 2      3      x       x      x     x                5v
   Mega2560................................. 2      3      21      20     19    18               5v
   Leonardo, Micro, ATmega32U4.............. 3      2      0       1      7     x                5v
   Digistump, Trinket, ATtiny85............. 2/physical pin 7                                    5v
   Due, SAM3X8E............................. all digital pins                                    3v
   Zero, ATSAMD21G18........................ all digital pins, except pin 4                      3v
   Blue Pill, STM32F103xxxx boards.......... all digital pins, maximun 16 pins at the same time  3v
   ESP8266.................................. all digital pins, except gpio6 - gpio11 & gpio16    3v/5v
   ESP32.................................... all digital pins                                    3v

   NOTE:
   - LOW     interrupt trigges whenever the pin is low
   - HIGH    interrupt triggers whenever the pin is high (Arduino Due, Zero, MKR1000 only)
   - CHANGE  interrupt triggers whenever the pin changes value
   - RISING  interrupt triggers when the pin goes from low to high
   - FALLING interrupt triggers when the pin goes from high to low

   Frameworks & Libraries:
   TimerOne AVR          - https://github.com/PaulStoffregen/TimerOne
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#pragma GCC optimize ("Os") //code optimization controls, "O2" or "O3" code performance & "Os" code size

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#include <Joystick.h>

#include <TimerOne.h>       //https://github.com/PaulStoffregen/TimerOne
#include <RotaryEncoder.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  1, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering

#define PIN_A   5           //ky-040 clk pin,             add 100nF/0.1uF capacitors between pin & ground!!!
#define PIN_B   4           //ky-040 dt  pin,             add 100nF/0.1uF capacitors between pin & ground!!!
#define BUTTON  3           //ky-040 sw  pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!

int16_t position = 0;

RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);


void encoderISR()
{
  encoder.readAB();
}

void encoderButtonISR()
{
  encoder.readPushButton();
}

void setup()
{
  Timer1.initialize();                                                       //optionally timer's period can be set here in usec, default 1 sec, this breaks analogWrite() for pins 9 & 10

  encoder.begin();                                                           //set encoders pins as input & enable built-in pullup resistors

  Timer1.attachInterrupt(encoderISR, 10000);                                 //call encoderISR()       every 10000 microseconds/0.01 seconds
  attachInterrupt(digitalPinToInterrupt(BUTTON), encoderButtonISR, FALLING); //call encoderButtonISR() every high to low changes

  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();

  // Initialize Joystick Library
  Joystick.begin();
  Joystick.setXAxisRange(-100, 100);
  Joystick.setYAxisRange(-100, 100);
}

void loop()
{
  if (position != encoder.getPosition())
  {
    position = encoder.getPosition();

    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Altitude");
    display.println(position);
    display.display();

    Joystick.setXAxis(position);

    Serial.println(position);
  }
  
  if (encoder.getPushButton() == true) Serial.println(F("PRESSED"));         //(F()) saves string to flash & keeps dynamic memory free
}
