#include <Arduino.h>

#include <DHT.h>
#define USE_ARDUINO_INTERRUPTS true
//#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// includes
#include <PulseSensorPlayground.h>
// DHT sensor library
#include <math.h>
#include <LiquidCrystal.h>

// Debug mode (print out stuff)
/*
0- No debug
1- Debug
2 - Debug esp wifi
*/

#define DEBUG_MODE 2

// Wifi credentials
const char *ssid = "gaafer 2.4";
const char *password = "01027099005";

// Google sheet id
String GOOGLE_SHEET_ID = "";

// Pin definitions
// #define COLOR_0 8 // Module pins wiring
// #define COLOR_1 9
#define COLOR_2 6
#define COLOR_3 7
#define COLOR_OUT 8
int redfrequency = 0;
int greenfrequency = 0;
int bluefrequency = 0;

int data = 0; // This is where we're going to stock our values

#define LCD_D4 A2
#define LCD_D5 A3
#define LCD_D6 A4
#define LCD_D7 A5
#define LCD_E 9
#define LCD_RW 12
#define LCD_RS 13
#define MAX_SLIDES 4
int slideNumber = 0;

#define BUZZER_PIN 2
#define RELAY_HEATER 3
#define RELAY_FAN 4
#define DHT_PIN 5
#define DHT_TYPE DHT11
#define ESP_TX_PIN 11
#define ESP_RX_PIN 10

#define TEMP_SKIN A0
//#define HEART_RATE A1
#define TEMP_SKIN_THRESHOLD_L 36
#define TEMP_SKIN_THRESHOLD_H 38
#define HM_THRESHOLD 60
#define TEMP_AIR_THRESHOLD_H 35
#define TEMP_AIR_THRESHOLD_L 25
#define HEART_RATE_THRESHOLD 120

// no of main loops before jaundice detection
#define JAUNDICE_THRESHOLD 10

/*
ALARM CODE GUIDE
1 - High temperature
2 - Low temperature
3 - High heart rate
4 - Low heart rate
5 - High humidity
6 - Low humidity
7 - High air temperature
8 - Low air temperature
9 - Sensor error
404 - baby kidnapped
*/

int alarmState = 0;
int alarmCounter = 0;
uint32_t countJaundice = 0;

int currentAlarmMillis = millis();
int currentMillis = millis();
int previousMillis = 0;
#define BUZ_INTERVAL_MS 500

DHT dht(DHT_PIN, DHT_TYPE);

PulseSensorPlayground pulseSensor;
const int OUTPUT_TYPE = SERIAL_PLOTTER;

const int PULSE_INPUT = A1;
// const int PULSE_BLINK = LED_BUILTIN;
// const int PULSE_FADE = 13;
const int THRESHOLD = 550; // Adjust this number to avoid noise when idle
                           // Configure the PulseSensor manager.

LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

SoftwareSerial ESP8266(ESP_RX_PIN, ESP_TX_PIN); // RX,TX

void setup()
{

  Serial.begin(9600);

  // init lcd and test
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Loading..");

  // init hardware actuator pins
  pinMode(RELAY_HEATER, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize the PulseSensor object
  pulseSensor.analogInput(PULSE_INPUT);
  // pulseSensor.blinkOnPulse(PULSE_BLINK);
  // pulseSensor.fadeOnPulse(PULSE_FADE);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Double-check the "pulseSensor" object was created and "began" seeing a signal.
  if (pulseSensor.begin())
  {
    Serial.println("We created a pulseSensor Object !"); // This prints one time at Arduino power-up,  or on Arduino reset.
  }

  // pinMode(COLOR_0, OUTPUT);
  // pinMode(COLOR_1, OUTPUT);
  pinMode(COLOR_2, OUTPUT);
  pinMode(COLOR_3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  // Done in hardware
  //  digitalWrite(COLOR_0, HIGH); // Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
  //  digitalWrite(COLOR_1, HIGH);

  // Init the DHT11 sensor
  Serial.println("DHT Started");
  dht.begin();

  // Start the software serial for communication with the ESP8266
  ESP8266.begin(115200); // this assumes default baud rate is used by the module

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
  if (alarmState >= 1)
  {
    // blink without delay using micros
    unsigned long currentAlarmMillis = millis();
    int state = digitalRead(BUZZER_PIN);

    if (currentAlarmMillis - previousMillis >= BUZ_INTERVAL_MS)
    {
      previousMillis = currentAlarmMillis;

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

  // display lcd block
  lcdSlidesLoop();

  // thermistor skin read
  float tempSkinReading = thermistorRead();
#if DEBUG_MODE == 1
  Serial.println();
  Serial.println("Temp skin: ");
  Serial.println(tempSkinReading);
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

  if (isnan(tempSkinReading) || isnan(h) || isnan(t))
  {
    alarmCounter++;
    // debouncing alarm
    if (alarmCounter > 10)
    {
      Serial.println("Sensor failure!");
      alarmState = 9;
    }
    // return;
  }
  else
  {
    alarmState = 0;
    alarmCounter = 0;
  }

  // heart rate read
#if DEBUG_MODE == 1
  Serial.println("Reading Pulse");
  // Serial.println(analogRead(PULSE_INPUT));
  Serial.println(pulseSensor.getBeatsPerMinute());
#endif
  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
   */
  if (pulseSensor.sawStartOfBeat())
  {
    pulseSensor.outputBeat();
  }

// color read
#if DEBUG_MODE == 1
  Serial.println("Color:");
#endif
  // Serial.println(getColor());
  colorLoop();

  // lcd print?

  // heater and fan control
  if (tempSkinReading < TEMP_SKIN_THRESHOLD_L)
  {
    digitalWrite(RELAY_HEATER, HIGH);
    digitalWrite(RELAY_FAN, LOW);
  }
  else if (tempSkinReading > TEMP_SKIN_THRESHOLD_H || h > HM_THRESHOLD)
  {
    digitalWrite(RELAY_HEATER, LOW);
    digitalWrite(RELAY_FAN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_HEATER, LOW);
    digitalWrite(RELAY_FAN, LOW);
  }

  // if (h > HM_THRESHOLD)
  // {
  //   // TODO: fix fan electiricty
  //   digitalWrite(RELAY_FAN, HIGH);
  // }

  // google sheet post

#if DEBUG_MODE == 2
  // listen for communication from the ESP8266 and then write it to the serial monitor
  if (ESP8266.available())
  {
    Serial.write(ESP8266.read());
  }

  // listen for user input and send it to the ESP8266
  if (Serial.available())
  {
    ESP8266.write(Serial.read());
  }
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

void colorLoop()
{

  // Setting red filtered photodiodes to be read
  digitalWrite(COLOR_2, LOW);
  digitalWrite(COLOR_3, LOW);
  // Reading the output frequency
  redfrequency = pulseIn(COLOR_OUT, LOW);

  delay(100);
  // Setting Green filtered photodiodes to be read
  digitalWrite(COLOR_2, HIGH);
  digitalWrite(COLOR_3, HIGH);
  // Reading the output frequency
  greenfrequency = pulseIn(COLOR_OUT, LOW);
  // Printing the value on the serial monitor

  delay(100);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(COLOR_2, LOW);
  digitalWrite(COLOR_3, HIGH);
  // Reading the output frequency
  bluefrequency = pulseIn(COLOR_OUT, LOW);
  // Printing the value on the serial monitor

  if (redfrequency > 25 && redfrequency < 77)
  {
#if DEBUG_MODE == 1
    Serial.println("RED COLOUR");
#endif
  }

  else if (bluefrequency > 25 && bluefrequency < 77)
  {
#if DEBUG_MODE == 1
    Serial.println("BLUE COLOUR");
#endif
  }
  else if (greenfrequency > 25 && greenfrequency < 77)

  {
#if DEBUG_MODE == 1
    Serial.println("GREEN COLOUR");
#endif
  }
  else if (redfrequency > 77 && redfrequency < 130)
  {
    countJaundice++;
#if DEBUG_MODE == 1
    Serial.println("YELLOW COLOUR");
#endif
  }
  else if (redfrequency > 130 && redfrequency < 180)
  {
    countJaundice++;
#if DEBUG_MODE == 1
    Serial.println("ORANGE COLOUR");
#endif
  }
  else if (redfrequency > 180 && redfrequency < 230)
  {
#if DEBUG_MODE == 1
    Serial.println("PINK COLOUR");
#endif
  }
  else if (redfrequency > 230 && redfrequency < 280)
  {
#if DEBUG_MODE == 1
    Serial.println("PURPLE COLOUR");
#endif
  }
  else if (redfrequency > 280 && redfrequency < 330)
  {
#if DEBUG_MODE == 1
    Serial.println("BROWN COLOUR");
#endif
  }
  else if (redfrequency > 330 && redfrequency < 380)
  {
#if DEBUG_MODE == 1
    Serial.println("WHITE COLOUR");
#endif
  }
  else
  {
#if DEBUG_MODE == 1
    Serial.println("NO COLOUR");
#endif
  }
}

void lcdSlidesLoop()
{
  // display slideshow block
  if (alarmState >= 1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ALARM");
    lcd.setCursor(0, 1);
    switch (alarmState)
    {
    case 1:
      lcd.print("HYPERTHERMIA");
      break;
    case 2:
      lcd.print("HYPOTHERMIA");
      break;
    case 3:
      lcd.print("TACHYCARDIA");
      break;
    case 4:
      lcd.print("BRADYCARDIA");
      break;
    case 5:
      lcd.print("HIGH HUMIDITY");
      break;
    case 6:
      lcd.print("LOW HUMIDITY");
      break;
    case 7:
      lcd.print("HIGH AIR TEMP");
      break;
    case 8:
      lcd.print("LOW AIR TEMP");
      break;
    case 9:
      lcd.print("SENSOR ERROR");
      break;

    default:
      break;
    }
    lcd.print("CHECK SENSOR");
  }
  else
  {
    // check if 1000 millis passed since last check
    if (millis() - currentMillis >= 2000)
    {
      currentMillis = millis();
      if (slideNumber == MAX_SLIDES)
      {
        slideNumber = 0;
        lcd.begin(16, 2);
      }
      else
      {
        switch (slideNumber)
        {
        case 0:
          // display dht air temp and humidity
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Air Temp:");
          lcd.print(dht.readTemperature());
          lcd.setCursor(0, 1);
          lcd.print("Humidity:");
          lcd.print(dht.readHumidity());

          break;
        case 1:
          // display skin temp
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Skin Temp:");
          lcd.print(thermistorRead());
          break;

        case 2:
          // display heart rate
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Heart Rate:");
          lcd.print(pulseSensor.getBeatsPerMinute());

          break;
        case 3: // display is jaundiced?
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Jaundice:");
          if (countJaundice > JAUNDICE_THRESHOLD)
          {
            lcd.print("YES");
          }
          else
          {
            lcd.print("NO");
          }

          break;

        default:
          break;
        }

        slideNumber++;
      }
    }
  }
}