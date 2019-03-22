#include <Arduino.h>
#include <OneWire.h>

//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27,16,2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//пин шины 1-Wire
const int ONE_WIRE_BUS=2;

//пины УЗ-датчика
const int trig = 6;
const int echo = 7;

//интервал отправки данных по mqtt (default = 5000 ms)
const long mqtt_interval = 5000;
//интервал отправки данных на thingspeak (default = 60000 ms)
const long thingspeak_interval = 60000;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire ds(ONE_WIRE_BUS);

//массив для хранения адресов датчиков
byte addr[1][8] = {0x28,0xFF,0x85,0xD0,0x90,0x15,0x01,0x35};
//массив для хранения температур
float Temp[1];
//



//***Функция считывания температуры c Далласов*****
void dallRead(unsigned long interval){
  static unsigned long prevTime = 0;
  if (millis() - prevTime > interval) { //Проверка заданного интервала
  static boolean flagDall = 0; //Признак операции
  prevTime = millis();
  flagDall =! flagDall; //Инверсия признака
  if (flagDall) {
    ds.reset();
    ds.write(0xCC); //Обращение ко всем датчикам
    ds.write(0x44); //Команда на конвертацию
  }
  else {
    byte i;
     int temp;
    for (i = 0; i < 1; i++){ //Перебор количества датчиков
     ds.reset();
     ds.select(addr[i]);
     ds.write(0xBE); //Считывание значения с датчика
     temp = (ds.read() | ds.read()<<8); //Принимаем два байта температуры
     Temp[i] = (float)temp / 16.0;
     }
   }
  }
}
//--------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  Serial.begin(9600);

  //lcd.init();
  //lcd.backlight();

  delay(100);
}

void loop() {
  long time, dist;
  static unsigned long prev_mqtt_send = 0;
  static unsigned long prev_thingspeak_send = 0;
  
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  time = pulseIn(echo, HIGH);
  dist = (time/2) / 29.1;

  dallRead(2000);

  if (millis() - prev_mqtt_send > mqtt_interval) {
    prev_mqtt_send = millis();
    if (dist > 500 or dist <= 0) dist = -10;
    Serial.print (F("Publish /ESP_Easy_garage/sensors/distance/,"));
    Serial.println (String(dist));
    Serial.print (F("Publish /ESP_Easy_garage/sensors/temperature_01/,"));
    Serial.println (String(Temp[0]));
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
    Serial.print(F("SendToHTTP 18.214.44.70,80,/update?api_key=JXQQ3PQWSTJK87EY&field1="));
    Serial.print(String(dist));
    Serial.print(F("&field2="));
    Serial.println (String(Temp[0]));
  }
}