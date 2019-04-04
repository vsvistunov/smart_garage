/*
Топики показаний:
/smart_garage/sensors/dist/        - расстояние до препятствия в см
/smart_garage/sensors/level/           - процентное выражение уровня ()
/smart_garage/sensors/temp_01/  - температура наружнего датчика
/smart_garage/sensors/temp_02/  - температура внутреннего датчика

Настройка мин/макс уровня - послать в uart строку вида "$<min_value> <max_value>;""
Символ "$" - начало команды, значения разделяются пробелами, в конце ставится символ ";".
Можно послать из интерфейса ESPEasy командой "SerialSend $<min_value> <max_value>;""
Например команда "SerialSend $40 50;" задаст мин/макс уровни 40 и 50 см.
Принятые значения записываются в dist_min и dist_max и сохраняются в EEPROM.


*/



#include <Arduino.h>
#include <OneWire.h>
#include <EEPROM.h>

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



unsigned int dist_min_address = 0;
unsigned int dist_max_address = 1;
//максимальный и минимальный уровни жидости
byte dist_min = 0;
byte dist_max = 0;

//максимальный и минимальный уровни температуры
const float temp_max = 5;
const float temp_min = 3;

String stringPublish = "Publish /smart_garage";

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

#define PARSE_AMOUNT 2         // число значений в массиве, который хотим получить
unsigned int intData[PARSE_AMOUNT];     // массив численных значений после парсинга
boolean recievedFlag;
boolean getStarted;
byte index;
String string_convert = "";

void parsing() {
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();        // обязательно ЧИТАЕМ входящий символ
    if (getStarted) {                         // если приняли начальный символ (парсинг разрешён)
      if (incomingByte != ' ' && incomingByte != ';') {   // если это не пробел И не конец
        string_convert += incomingByte;       // складываем в строку
      } else {                                // если это пробел или ; конец пакета
        intData[index] = string_convert.toInt();  // преобразуем строку в int и кладём в массив
        string_convert = "";                  // очищаем строку
        index++;                              // переходим к парсингу следующего элемента массива
      }
    }
    if (incomingByte == '$') {                // если это $
      getStarted = true;                      // поднимаем флаг, что можно парсить
      index = 0;                              // сбрасываем индекс
      string_convert = "";                    // очищаем строку
    }
    if (incomingByte == ';') {                // если таки приняли ; - конец парсинга
      getStarted = false;                     // сброс
      recievedFlag = true;                    // флаг на принятие
    }
  }
}

// медианный фильтр по трем значениям
float middle_of_3(int a, int b, int c) {
  int middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}


//***Функция считывания температуры c Далласов*****
void dallRead(unsigned long interval){
  static unsigned long prevTime = 0;
  static int temp_unfiltered[3][2]; //массив для хранения температур с двух датчиков, в каждый столбец по три значения
  static byte i = 0; // индекс строки массива неотфильтрованных температур temp_unfiltered[i][j]
  int temp_filtered; //результат работы медианного фильтра по  

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
      byte j; //индекс столбца массива temp_unfiltered - выбор датчика
      
      //Перебор количества датчиков - выбираем столбец массива temp_unfiltered[i][j]
      for (j = 0; j < 2; j++){ 
        ds.reset();
        ds.select(addr[j]);
        ds.write(0xBE); //Считывание значения с датчика
        //переключаем индекс строки с 0 до 2 (0, 1, 2, 0, 1, 2…)
        if (++i >2) i = 0; 
        //Принимаем два байта температуры и пишем их в массив
        temp_unfiltered[i][j] = (ds.read() | ds.read()<<8);
        //производим фильтрацию по каждому столбцу значений массива temp_unfiltered
        temp_filtered = middle_of_3 (temp_unfiltered[0][j], temp_unfiltered[1][j], temp_unfiltered[2][j]);
        
        //В массив Temp заносим отфильтрованные значения с каждого датчика
        Temp[j] = (float)temp_filtered / 16.0;
        
        /*
        Serial.print("Sensor_");
        Serial.print(j);
        Serial.print(";");
        Serial.print ((float)temp_unfiltered[i][j] / 16.0);
        Serial.print (";");
        Serial.println (Temp[j]);
        */

      }
    }
  }
}
//--------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  dist_min = EEPROM.read(0);
  dist_max = EEPROM.read(1);


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
  unsigned int time_filtered, dist, level;
  static unsigned int time_unfiltered[3];
  static byte i;
  static unsigned long prev_pulse_send = 0;
  static unsigned long prev_mqtt_send = 0;
  static unsigned long prev_thingspeak_send = 0;
  
  //замеряем дистанцию каждые 500 мс
  if (millis() - prev_pulse_send > 500) {
    prev_pulse_send = millis();
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    //меняем индекс массива 0..1..2..0..1..2..0..1..
    if (++i > 2) i = 0;
    //запоминаем в массив измеренное эхо
    time_unfiltered [i] = pulseIn(echo, HIGH);
    //производим фильтрацию значений массива time_unfiltered
    time_filtered = middle_of_3(time_unfiltered [0], time_unfiltered [1], time_unfiltered [2]);
    //вычисяем расстояние до объекта
    dist = (time_filtered/2) / 29.1;
    //вычисляем процентный уровень
    level = constrain (map (dist, dist_max, dist_min, 0, 100), 0, 100);
  }


  dallRead(1000);

  //переключение реле по уровню жидкости
  if (dist < dist_min && dist > 0) relay_control (1,LOW);
  else if (dist > dist_max && dist > 0) relay_control (1,HIGH); 

  //переключение реле по датчику температуры
  if (Temp[1] < temp_min) relay_control (2,LOW);
  else if (Temp[1] > temp_max) relay_control (2,HIGH);


  if (millis() - prev_mqtt_send > mqtt_interval) {
    prev_mqtt_send = millis();
    if (dist > 500 or dist <= 0) dist = -10;
    Serial.print(stringPublish);
    Serial.print (F("/sensors/dist/,"));
    Serial.println(dist);
    Serial.print(stringPublish);
    Serial.print (F("/sensors/level/,"));
    Serial.println (level);
    Serial.print(stringPublish);
    Serial.print (F("/sensors/temp_01/,"));
    Serial.println (Temp[0],1);
    Serial.print(stringPublish);
    Serial.print (F("/sensors/temp_02/,"));
    Serial.println (Temp[1],1);
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
    Serial.print(F("SendToHTTP api.thingspeak.com,80,/update?api_key=JXQQ3PQWSTJK87EY&field1="));
    Serial.print(dist);
    Serial.print(F("&field2="));
    Serial.print(level);
    Serial.print(F("&field3="));
    Serial.print(Temp[0],1);
    Serial.print(F("&field4="));
    Serial.println(Temp[1],1);
  }

  parsing();            // функция парсинга
  if (recievedFlag) {   // если получены данные
    recievedFlag = false;
    if (intData[0] >= 10 && intData[0] < 200 && intData[1] > 10 && intData[1] < 200 && intData[0] < intData[1]) {
      dist_min = intData[0];
      dist_max = intData[1];

      EEPROM.write(0,dist_min);
      EEPROM.write(1,dist_max);
      
      Serial.print(stringPublish);
      Serial.print (F("/debug/,wr_to_EEPROM_dist_min/max:"));
      Serial.print (dist_min);
      Serial.print (F("/"));
      Serial.println (dist_max);
    }
    else {
      Serial.print(stringPublish);
      Serial.println (F("/debug/,ERR_dist_min/max"));
    }
  }
}