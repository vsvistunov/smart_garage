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

//пины реле
const int rel_01 = 3;
const int rel_02 = 4;

//максимальный и минимальный уровни жидости
const long dist_max = 30;
const long dist_min = 20;

//максимальный и минимальный уровни температуры
const float temp_max = 30;
const float temp_min = 26;


//интервал отправки данных по mqtt (default = 5000 ms)
const long mqtt_interval = 5000;
//интервал отправки данных на thingspeak (default = 60000 ms)
const long thingspeak_interval = 60000;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire ds(ONE_WIRE_BUS);

//массив для хранения адресов датчиков
byte addr[2][8] = {{0x28,0xFF,0x85,0xD0,0x90,0x15,0x01,0x35},{0x28,0xD5,0xBF,0x7F,0x6,0x0,0x0,0xF2}};
//массив для хранения температур
float Temp[2];
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
    for (i = 0; i < 2; i++){ //Перебор количества датчиков
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

  pinMode(rel_01, OUTPUT);
  digitalWrite(rel_01,HIGH);
  pinMode(rel_02, OUTPUT);
  digitalWrite(rel_02,HIGH);

  Serial.begin(9600);

  //lcd.init();
  //lcd.backlight();

  delay(100);
}


void relay_control (int rel_ID, int state) {
  rel_ID = rel_ID + 2;
  if (digitalRead(rel_ID) != state) {
    digitalWrite(rel_ID,state);
  }
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


  //переключение реле по уровню жидкости
  if (dist < dist_min && dist > 0) relay_control (1,LOW);
  else if (dist > dist_max && dist > 0) relay_control (1,HIGH); 

  //переключение реле по датчику температуры
  if (Temp[1] < temp_min) relay_control (2,LOW);
  else if (Temp[1] > temp_max) relay_control (2,HIGH);


  if (millis() - prev_mqtt_send > mqtt_interval) {
    prev_mqtt_send = millis();
    if (dist > 500 or dist <= 0) dist = -10;
    Serial.print (F("Publish /ESP_Easy_garage/sensors/distance/,"));
    Serial.println (String(dist));
    Serial.print (F("Publish /ESP_Easy_garage/sensors/temperature_01/,"));
    Serial.println (String(Temp[0]));
    Serial.print (F("Publish /ESP_Easy_garage/sensors/temperature_02/,"));
    Serial.println (String(Temp[1]));
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
    Serial.print(String(Temp[0]));
    Serial.print(F("&field3="));
    Serial.println(String(Temp[1]));
  }
}