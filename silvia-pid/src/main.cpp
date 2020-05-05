/*
  Silvia Rancilio PID

  PID
  https://playground.arduino.cc/Code/PIDLibraryRelayOutputExample/
  https://github.com/br3ttb/Arduino-PID-Library/

  smoothing readings:
  https://www.arduino.cc/en/Tutorial/Smoothing

  OLED displej SSD1306 přes I2C
  https://navody.arduino-shop.cz/navody-k-produktum/oled-displej-ssd1306.html

  https://www.arduinotech.cz/inpage/wifi-teplomer-385/

 */

#include <Arduino.h>
#include <PID_v1.h>
#include <max6675.h>
#include <U8g2lib.h>

// debug
// #define DEBUG

// Relay GPIO pin
// pin 13 = internal led, change to 8
#define HEAT_RELAY_PIN 6
#define PUMP_RELAY_PIN 8
// D2 button start timer
#define BUTTON_PIN 2

// cilova teplota pro PID, stupne celsia
const double set_brew_point = 93;

// doba extrakce kavy - dle toho nastavit timer
const unsigned long shot_timer = 30 * 1000;

// MAX6675
#define thermo_DO 3    // MAX31855 Data to Arduino digital 3
#define thermo_CS 4    // MAX31855 Chip Select to Arduino digital 4
#define thermo_CLK 5   // MAX31855 Clock to Arduino digital 5

// the temperature averaging
const int numReadings = 8;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// MAX6675(CLK, CS, DO)
MAX6675 termoclanek(thermo_CLK, thermo_CS, thermo_DO);

// MAX31865 SPI(CS, DI, DO, CLK)
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

// define PID variables we'll be connecting to
double Setpoint, Input, Output;

float Kp = 16.16;   // User inserts PID values obtaines from Autotune here
float Ki = 0.14;    //
float Kd = 480.10;  //

// heating ON/OFF
bool heating_on = false;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// espresso machines that a 1000ms window does well
unsigned long WindowSize = 1000;
unsigned long windowStartTime;

unsigned long current_time;
unsigned long last_time;

// button
// internal pull up = GND + D2 -> trigger to LOW
// prvni stisk START, druhy stisk STOP
int last_button_state = HIGH;
bool switch_relay_on = false;
unsigned long start_timer;
int timer_sec = 0;      // zbyvajici cas do konce shot_timer

// inicializace OLED displeje z knihovny U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// bitmapa vypinace, 16x16px
#define u8g_switch_width 16
#define u8g_switch_height 16
static const unsigned char u8g_switch_bitmap[] PROGMEM = {
0xfe, 0x7f, 0xfe, 0x7f, 0xf6, 0x6f, 0xe6, 0x67, 0xc6, 0x63, 0x8e, 0x71, 0x8e, 0x71, 0x9e, 0x79,
0x9f, 0xf9, 0x9f, 0xf9, 0x9f, 0xf9, 0x8f, 0xf1, 0xc7, 0xe3, 0xc1, 0x83, 0xf0, 0x0f, 0xf8, 0x1f
};


// zjisti aktualni teplotu termoclanku
double getTemperature() {

  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = termoclanek.readCelsius();
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  // set PID Input to avg
  return average;
}

// funkce vykresli pro nastavení výpisu informací na OLED
// Setpoint, Input, heating_on, timer_sec
void oledDisplay()
{

  // draw switch on bitmap as heating ON
  if (heating_on == true) {
    u8g2.drawXBMP(100, 0, u8g_switch_width, u8g_switch_height, u8g_switch_bitmap);
    /*
    for (int i = 0; i <= 5; i++) {
      u8g2.drawHLine(10, i, 108);
    }
    */
  }

  // display setpoint a actual temp value
  u8g2.setCursor(0, 36);
  u8g2.print(Setpoint, 1);
  u8g2.setCursor(0, 64);
  u8g2.print(Input, 1);

  // display odpocet timeru, pokud je timer_sec nenulova
  if (timer_sec > 0) {
    u8g2.setCursor(80, 64);
    u8g2.print(timer_sec);
  }
}

// detekce stisknuteho tlacitko, pouze "edge"
void checkButton() {

  int current_button_state = digitalRead(BUTTON_PIN);

  // pokud doslo ke stisku tlacitko
  if (current_button_state == LOW && last_button_state == HIGH)
  {
    if (switch_relay_on == false)
    {
      start_timer = current_time;
      #ifdef DEBUG
      Serial.println("timer start");
      #endif
    }
    else {
      // 2x tlacitko = stop timer immediately
      start_timer = current_time - shot_timer;
      #ifdef DEBUG
      Serial.println("timer stop");
      #endif
    }

    // invert relayOn ON/OFF
    switch_relay_on = !switch_relay_on;

    // Delay a little bit to avoid bouncing
    delay(10);
  }

  last_button_state = current_button_state;
}

// nastav rele dle timeru hodnoty start_timer
void setPumpRelay() {

  // set brew pump ON or OFF
  if (shot_timer > (current_time - start_timer)) {
    digitalWrite(PUMP_RELAY_PIN, HIGH);
    // nastav odpocet sekund do konce timeru
    timer_sec = (int) ((shot_timer - current_time + start_timer)/1000);
  }
  else {
    digitalWrite(PUMP_RELAY_PIN, LOW);
    // reset switch_relay_on state
    switch_relay_on = false;
    timer_sec = 0;
  }

  #ifdef DEBUG
  Serial.println(timer_sec);
  #endif
}

// switch heat relay
void setHeatRelay() {

  // turn the output pin on/off based on pid output
  if (current_time - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > current_time - windowStartTime) {

    #ifdef DEBUG
    Serial.println("ON");
    #endif

    heating_on = true;
    digitalWrite(HEAT_RELAY_PIN, HIGH);
  }
  else {

    #ifdef DEBUG
    Serial.println("OFF");
    #endif

    heating_on = false;
    digitalWrite(HEAT_RELAY_PIN, LOW);
  }
}

void setup(void)
{
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  // set mode pro tlacitko a pro relay
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // vypni rele pro heating
  pinMode(HEAT_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, LOW);

  // vypni rele pro pumpu
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, LOW);

  // inicializace pro PID
  current_time = millis();
  windowStartTime = current_time;
  last_time = current_time;

  Setpoint = set_brew_point;  //Default Setpoint for Rancilio Silvia
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  // reset casovace pro pumpu na OFF
  start_timer = current_time - shot_timer;

  // OLED display
  // fontsize - pozor, ma vliv na velikost program size, TN pouze digits
  // https://github.com/olikraus/u8g2/wiki/fntlistall#20-pixel-height
  u8g2.begin();
  u8g2.setFontPosTop();
  u8g2.setFont(u8g2_font_fub20_tn);

  // initialize all the readings to 0:
  for (int i = 0; i < numReadings; i++)
  {
    readings[i] = 0;
  }

  // wait for MAX chip to stabilize
  delay(1000);
}

// loop forever
void loop(void)
{
  // recalculate current time
  current_time = millis();

  // calc PID 8 times per second
  if ((current_time - last_time) >= 125)
  {
    last_time = current_time;

    // Input = read temp pro PID
    Input = getTemperature();

    // kontrola stisku tlacitek
    checkButton();

    // nastav rele pro pumpu
    setPumpRelay();

    // PID Input = aktualni_teplota v C;
    myPID.Compute();

    #ifdef DEBUG
    Serial.print(Output); Serial.print(";");
    Serial.print(Input);  Serial.print(";");
    #endif

    // switch on/off heat relay
    setHeatRelay();

    // vykresli text na OLED
    u8g2.firstPage();
    do {
      oledDisplay();
    } while( u8g2.nextPage() );
  }

  // delay() nepouzivat, arduino zamrzne a nefunguje pak PID
}
