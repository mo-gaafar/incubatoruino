#include <Arduino.h>

#include <dht.h>

//#include <ESP8266WiFi.h>

// includes
#include <PulseSensorPlayground.h>
// DHT sensor library
#include <math.h>
#include <LiquidCrystal.h>

// Wifi credentials
const char *ssid = "gaafer 2.4";
const char *password = "01027099005";

// Google sheet id
String GOOGLE_SHEET_ID = "";

// Pin definitions
#define BUZZER_PIN 12
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

#define RELAY_HEATER 4
#define DHT_PIN 3
#define RELAY_FAN 2
#define ESP_TX_PIN 1
#define ESP_RX_PIN 0

#define TEMP_SKIN A0
#define HEART_RATE A1
#define TEMP_SKIN_THRESHOLD 00
#define HM_THRESHOLD 00
#define TEMP_AIR_THRESHOLD 00
#define HEART_RATE_THRESHOLD 00

int ALARM_STATE = 0;

DHT dht;
PulseSensorPlayground pulseSensor;


//////////////////////////////////////////////////////////////////





void WifiLoop(void)
{
  // if (WiFi.status() == WL_CONNECTED)
  // { // Check WiFi connection status

  //   HTTPClient http; // Declare an object of class HTTPClient

  //   // http.begin("");  //Specify request destination
  //   // int httpCode = http.POST();                                  //Send the request
  //   Serial.print("wifi connected");
  //   // if (httpCode > 0) { //Check the returning code

  //   //   String payload = http.getString();   //Get the request response payload
  //   //   Serial.println(payload);             //Print the response payload

  //   // }

  //   http.end(); // Close connection
  // }
  // // delay(500);    //Send a request every 30 seconds
}

/////////////////////////////////////////////////////
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("hello world");
  //  pulseSensor.analogInput(HEART_RATE);
  pulseSensor.setThreshold(Threshold);
  pinMode(RELAY_FAN, HIGH);

  // Wifi setup
  WiFi.begin(ssid, password) while (WiFi.status() != WL_CONNECTED)
  {
    delay(50);
  }
}

void loop()
{
  Serial.println(".");
  Serial.println("temp");
  Serial.println(dht.readTemperature(), 1);
  Serial.println("hm");
  Serial.println(dht.readHumidity(), 1);
  // myBPM = pulseSensor.getBeatsPerMinute(); // Calculates BPM

  //      // Constantly test to see if a beat happened
  //  Serial.println("♥  A HeartBeat Happened ! "); // If true, print a message
  // Serial.print("BPM: ");
  // Serial.println(myBPM);// Print the BPM value
  // int v=bmp();
  if (V > HEART_RATE_THRESHOLD)
  {
    BUZZER_PIN
  }

  // Serial.println(analogRead(HEART_RATE));

  delay(200);

  /*int myBPM = pulseSensor.getBeatsPerMinute();      // Calculates BPM

  if (pulseSensor.sawStartOfBeat()) {               // Constantly test to see if a beat happened
    Serial.println("♥  A HeartBeat Happened ! "); // If true, print a message
    Serial.print("BPM: ");
    Serial.println(myBPM);                        // Print the BPM value
    }*/

  // put your main code here, to run repeatedly:

  // Wifi
  WifiLoop();
}