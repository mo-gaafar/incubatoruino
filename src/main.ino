#include <Arduino.h>

#include <DHT.h>
#define USE_ARDUINO_INTERRUPTS true
//#include <ESP8266WiFi.h>

// includes
#include <PulseSensorPlayground.h>
// DHT sensor library
#include <math.h>
#include <LiquidCrystal.h>

// Debug mode (print out stuff)
#define DEBUG_MODE 1

// Wifi credentials
const char *ssid = "gaafer 2.4";
const char *password = "01027099005";

// Google sheet id
String GOOGLE_SHEET_ID = "";

// Pin definitions
#define COLOR_0 7 // Module pins wiring
#define COLOR_1 8
#define COLOR_2 9
#define COLOR_3 10
#define COLOR_OUT 11

int data = 0; // This is where we're going to stock our values

// #define LCD_D4 11
// #define LCD_D5 10
// #define LCD_D6 9
// #define LCD_D7 8
// #define LCD_E 7
// #define LCD_RW 6
// #define LCD_RS 5

#define BUZZER_PIN 5
#define RELAY_HEATER 4
#define RELAY_FAN 2
#define DHT_PIN 3
#define DHT_TYPE DHT11
#define ESP_TX_PIN 1
#define ESP_RX_PIN 0

#define TEMP_SKIN A0
//#define HEART_RATE A1
#define TEMP_SKIN_THRESHOLD_L 36.4
#define TEMP_SKIN_THRESHOLD_H 38.2
#define HM_THRESHOLD 00
#define TEMP_AIR_THRESHOLD_H 35
#define TEMP_AIR_THRESHOLD_L 25
#define HEART_RATE_THRESHOLD 120

int alarmState = 0;
int currentMillis = millis();
int previousMillis = 0;
#define BUZ_INTERVAL_MS 500

#include <SoftwareSerial.h>

DHT dht(DHT_PIN, DHT_TYPE);

PulseSensorPlayground pulseSensor;
const int OUTPUT_TYPE = SERIAL_PLOTTER;

const int PULSE_INPUT = A1;
const int PULSE_BLINK = LED_BUILTIN;
const int PULSE_FADE = 5;
const int THRESHOLD = 550; // Adjust this number to avoid noise when idle
                           // Configure the PulseSensor manager.

int alarmCounter = 0;

SoftwareSerial ESP8266(10, 11); // RX,TX

void setup()
{

  Serial.begin(9600);
  pinMode(RELAY_HEATER, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize the PulseSensor object
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Double-check the "pulseSensor" object was created and "began" seeing a signal.
  if (pulseSensor.begin())
  {
    Serial.println("We created a pulseSensor Object !"); // This prints one time at Arduino power-up,  or on Arduino reset.
  }

  pinMode(COLOR_0, OUTPUT);
  pinMode(COLOR_1, OUTPUT);
  pinMode(COLOR_2, OUTPUT);
  pinMode(COLOR_3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  digitalWrite(COLOR_0, HIGH); // Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
  digitalWrite(COLOR_1, HIGH);

  // Init the DHT11 sensor
  Serial.println("DHT Started");
  dht.begin();

  // Start the software serial for communication with the ESP8266
  ESP8266.begin(9600); // this assumes default baud rate is used by the module

  Serial.println("");
  Serial.println("Ready");

  Serial.println("Establishing connection type…");
  ESP8266.println("AT+CIPMUX=1");
  delay(1000);
  Serial.println("Setting WiFI mode…");
  ESP8266.println("AT+CWMODE=3");
  delay(1000);
  Serial.println("Connecting to WiFi");
  ESP8266.println("\"gaafer 2.4\", \"01027099005\""); // insert your own SSID and password here
  delay(1000);
  Serial.println("Establishing TCP connection");
  ESP8266.println("\"AT+CIPSTART=0\",\"TCP\",\"www.teachmemicro.com\",\"80\"");
  delay(1000);
}

void loop()
{

  // alarm block
  if (alarmState == 1)
  {
    // blink without delay using micros
    unsigned long currentMillis = millis();
    int state = digitalRead(BUZZER_PIN);

    if (currentMillis - previousMillis >= BUZ_INTERVAL_MS)
    {
      previousMillis = currentMillis;

      if (state == LOW)
        state = HIGH;
      else
        state = LOW;

      digitalWrite(BUZZER_PIN, state);
    }
  }
  else
  {
    digitalWrite(BUZZER_PIN, LOW);
  }
  // end alarm block

  // thermistor skin read
  float reading = thermistorRead();
#if DEBUG_MODE == 1
  Serial.println();
  Serial.println("Temp skin: ");
  Serial.println(reading);
#endif

  // temperature and humidity air read
  float h = dht.readHumidity();
  float t = dht.readTemperature();
#if DEBUG_MODE == 1
  Serial.println("Temp air: ");
  Serial.println(t);
  Serial.println("Humidity air: ");
  Serial.println(h);
#endif
  if (isnan(reading))
  {
    Serial.println("Sensors failed!");
    alarmState = 1;
    return;
  }
  else
  {
    alarmState = 0;
  }

  // heart rate read

  Serial.println("Reading Pulse");
  Serial.println(analogRead(PULSE_INPUT));
  Serial.println(pulseSensor.getBeatsPerMinute());

  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
   */
  if (pulseSensor.sawStartOfBeat())
  {
    pulseSensor.outputBeat();
  }

  // color read
  Serial.println("Color:");
  Serial.println(getColor());

  // lcd print?

  // heater and fan control
  if (reading < TEMP_SKIN_THRESHOLD_L)
  {
    digitalWrite(RELAY_HEATER, HIGH);
    digitalWrite(RELAY_FAN, LOW);
  }
  else if (reading > TEMP_SKIN_THRESHOLD_H)
  {
    digitalWrite(RELAY_HEATER, LOW);
    digitalWrite(RELAY_FAN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_HEATER, LOW);
    digitalWrite(RELAY_FAN, LOW);
  }

  // google sheet post
  // int adcval = analogRead(A0);
#if DEBUG_MODE == 1
  Serial.println("Sending out data…");
#endif
  ESP8266.println("AT+CIPSEND=0,16");
  ESP8266.println("POST https://script.google.com/macros/s/AKfycbzPC5yj-H_a1P03F7VZIvknGoh-oZzunQiIb6jEi0q2gYw7maxfD7vtV8IcFv9WG4Ml/exec?heartrate=35&incubatortemp=25&infanttemp=35&humidity=5&juandice=1 HTTPS/1.1");
  ESP8266.println();
  ESP8266.println();
  delay(20);
}

// thermistor module

int thermistor_adc_val;
float temp;
double output_voltage, thermistor_resistance, therm_res_ln, temperature;

float thermistorRead(void)
{
  thermistor_adc_val = analogRead(TEMP_SKIN);
  output_voltage = ((thermistor_adc_val * 5.0) / 1023.0);
  thermistor_resistance = ((5 * (10.0 / output_voltage)) - 10); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000;         /* Resistance in ohms   */
  therm_res_ln = log(thermistor_resistance);
  temperature = (1 / (0.001129148 + (0.000234125 * therm_res_ln) + (0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln))); /* Temperature in Kelvin */
  temperature = temperature - 273.15;
  return temperature;
}

// color module
int COLOR_data = 0; // This is where we're going to stock our values
int red;
int grn;
int blu;
String color = "";

////////////////////////////////////////////////////////////////////////////////
String getColor()
{
  readRGB();

  if (red > 8 && red < 18 && grn > 9 && grn < 19 && blu > 8 && blu < 16)
    color = "WHITE";
  else if (red > 80 && red < 125 && grn > 90 && grn < 125 && blu > 80 && blu < 125)
    color = "BLACK";
  else if (red > 12 && red < 30 && grn > 40 && grn < 70 && blu > 33 && blu < 70)
    color = "RED";
  else if (red > 50 && red < 95 && grn > 35 && grn < 70 && blu > 45 && blu < 85)
    color = "GREEN";
  else if (red > 10 && red < 20 && grn > 10 && grn < 25 && blu > 20 && blu < 38)
    color = "YELLOW";
  else if (red > 65 && red < 125 && grn > 65 && grn < 115 && blu > 32 && blu < 65)
    color = "BLUE";
  else
    color = "NO_COLOR";

  return color;
}

/* read RGB components */
void readRGB()
{
  red = 0;
  grn = 0;
  blu = 0;
  int n = 10;
  for (int i = 0; i < n; ++i)
  {
    // read red component
    digitalWrite(COLOR_2, LOW);
    digitalWrite(COLOR_3, LOW);
    red = red + pulseIn(COLOR_OUT, LOW);
    delay(10);

    // read green component
    digitalWrite(COLOR_2, HIGH);
    digitalWrite(COLOR_3, HIGH);
    grn = grn + pulseIn(COLOR_OUT, LOW);
    delay(10);

    // let's read blue component
    digitalWrite(COLOR_2, LOW);
    digitalWrite(COLOR_3, HIGH);
    blu = blu + pulseIn(COLOR_OUT, LOW);
    delay(10);
  }
  red = red / n;
  grn = grn / n;
  blu = blu / n;
}
