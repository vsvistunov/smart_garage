#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27,16,2); // set the LCD address to 0x27 for a 16 chars and 2 line display

const int ONE_WIRE_BUS=2;
const int trig = 6;
const int echo = 7;
const long temperature_interval = 5000;
const long mqtt_interval = 5000;
const long thingspeak_interval = 60000;
//const long lcd_interval = 1000;
long time, dist;
unsigned long prev_mqtt_send = 0;
unsigned long prev_thingspeak_send = 0;
//unsigned long prev_lcd_send = 0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress insideThermometer;

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}



void setup() {
  // put your setup code here, to run once:
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  Serial.begin(9600);

  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");
  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();
  sensors.setResolution(insideThermometer, 10);
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();
  //lcd.init();
  //lcd.backlight();

  delay(500);

}


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  time = pulseIn(echo, HIGH);
  dist = (time/2) / 29.1;

  sensors.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensors.getTempC(insideThermometer);

  if (millis() - prev_mqtt_send > mqtt_interval) {
    prev_mqtt_send = millis();
    if (dist > 500 or dist <= 0) dist = -10;
    Serial.println ("Publish /ESP_Easy_garage/sensors/distance/," + String(dist));
    Serial.println ("Publish /ESP_Easy_garage/sensors/distance/," + String(tempC));
  }
/*
  if (millis() - prev_lcd_send > lcd_interval) {
    lcd.clear();
    prev_lcd_send = millis();
    if (dist > 500 or dist <= 0) lcd.print("Out of Range");
    else lcd.print(dist);
  }
*/
  if (millis() - prev_thingspeak_send > thingspeak_interval) {
    prev_thingspeak_send = millis();
    if (dist > 500 or dist <= 0) dist = -10;
    Serial.println ("SendToHTTP 18.214.44.70,80,/update?api_key=JXQQ3PQWSTJK87EY&field1=" + String(dist)+"&field2=" + String(tempC));
  }
 
}