#include <math.h>
#include <DoubleLinkedList.h>
#include <LiquidCrystal_SoftI2C.h>
#include <SoftwareWire.h>
#include <SPI.h>
#include <SD.h>
#include <Regexp.h>

#define CHECK Serial.println("✓");
#define DEFAULT_TIMEOUT 2000
#define RED_NAME "Obj1" // RED - Refrigeration Exporter of Data
#define MAX_PARTS 10 // Adjust this based on the maximum expected split parts

const int thermistor_output1 = A0;
const int reset_pin = 11; // with multiplexer is A3
const int chipSelect_pin = 10;
const int SDA_pin = 6;
const int SCL_pin = 7;

const uint8_t RTC_ADDRESS = 0x68;
const uint8_t LCD_ADDRESS = 0x27;

SoftwareWire myWire(SDA_pin, SCL_pin);
LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2, &myWire);
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;
MatchState ms; // regexp

struct MyDateTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  MyDateTime(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s)
      : year(y), month(mo), day(d), hour(h), minute(mi), second(s) {}
};

void arduinoReset();
void sendData(const String& command, const int timeout = DEFAULT_TIMEOUT, void(*funcIfNotOk)() = arduinoReset);
String getData(const String& command, const int timeout = DEFAULT_TIMEOUT);

void sendTempreature(const String& targetURL, float tempreature, String termistorpPath);
float TempreatureFromAdc(const int16_t& thermistor_adc_val);

bool findOk(const String& txt);
void plug() {}

uint8_t bcdToDec(uint8_t val);
uint8_t decToBcd(uint8_t val);
void setRTC(const MyDateTime &dt);
MyDateTime getRTC();
int split(String data, char delimiter, String result[]);
void SetupRTC();

void printLCD(const String& txt, const String& txt2 = "");

bool isURLOK = 0;
bool isRTCOK = 0;

String URL = "";
String ID = "";

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  while(!Serial);
#endif
  myWire.begin();
  lcd.begin();                      
  lcd.backlight();
  printLCD("LCD and wire OK");

  Serial1.begin(9600);
  while(!Serial1);
  printLCD("Serial1 OK");  
  
  //Power on the SIM800C
  pinMode(9,OUTPUT);
  digitalWrite(9,HIGH);
  delay(3000);
  digitalWrite(9,LOW);
  delay(1000);
  printLCD("SIM800C OK");

  while(!Serial1.availableForWrite());
  printLCD("Serial1 is write", "available");

  printLCD("AT commands", "initialization.. ");    
  sendData("AT");
  printLCD("AT OK");

  printLCD("Sim-card setup..");    
  sendData("AT+CSQ"); // checking signal level
  printLCD("AT+CSQ OK");
  sendData("AT+CREG?"); // checking web registration 
  printLCD("AT+CREG? OK");
  sendData("AT+CSTT=\"internet.kyivstar.net\",\"\",\"\""); // installing APN
  printLCD("Kyivstar OK");
  sendData("AT+CIICR"); // Raising a wireless connection
  printLCD("AT+CIICR OK");
  sendData("AT+SAPBR=3,1,\"Contype\",\"GPRS\""); // Setting media profile parameters
  printLCD("GPRS OK");
  sendData("AT+SAPBR=3,1,\"APN\",\"internet.kyivstar.net\""); // installing APN
  printLCD("APN OK");
  sendData("AT+SAPBR=1,1"); // Opening the carrier profile
  printLCD("SAPBR1.1 OK");
  sendData("AT+SAPBR=2,1"); // Checking the carrier profile status
  printLCD("SAPBR2.1 OK");
  sendData("AT+CMGF=1"); // Set the SMS in text mode
  printLCD("SMS OK");

  printLCD("Initializing", "SD card..");
  if (!card.init(SPI_HALF_SPEED, chipSelect_pin)) {
    printLCD("SD module BAD", "Resetting...");
    delay(5000);
    arduinoReset();
  } else {
    printLCD("SD module OK");
  }

  printLCD("HTTP", "initialization..");    
  sendData("AT+HTTPINIT");
  printLCD("AT+HTTPINIT OK");

  printLCD("HTTP parameters", "setting...");  
  sendData("AT+HTTPPARA=\"CID\", 1");
  printLCD("CID OK");
  // sendData("AT+HTTPPARA=\"URL\", \"http://92.43.81.152:1488/data\"");
  // printLCD("URL OK");
  sendData("AT+HTTPPARA=\"CONTENT\", \"application/json\"");
  printLCD("CONTENT OK");
  sendData("AT+HTTPPARA?");
  printLCD("AT+HTTPPARA? OK");

  if(SD.exists("url.txt")) {
    myFile = SD.open("url.txt", FILE_READ);
    while (myFile.available()) {
      URL += myFile.read();
    }
    //sendData("AT+HTTPPARA=\"URL\", \""+url+"\"");
    printLCD("URL OK");
    isURLOK = 1;
    myFile.close();
  }

  if(SD.exists("id.txt")) {
    myFile = SD.open("id.txt", FILE_READ);
    while (myFile.available()) {
      ID += myFile.read();
    }
    printLCD("ID OK");
    myFile.close();
  } else {
    printLCD("id.txt BAD", "Resetting..");
    delay(5000);
    arduinoReset();
  }

  
  if(getRTC().year < 2025){
    printLCD("RTC BAD");
    SetupRTC();
  } else{
    printLCD("RTC OK");
    isRTCOK = 1;
  }
  
  if(isURLOK){
    printLCD("Data transfering", "started");  
  } else {
    printLCD("Waiting URL via","SMS (user app)");  
  }

  pinMode(thermistor_output1, INPUT);  
}
  
void loop() {
  if (Serial1.available()) {
    String message = Serial1.readString();

    // Проверка на наличие заголовка SMS
    if (message.indexOf("+CMT:") != -1) {
      // Извлечение текста SMS
      int index = message.indexOf("\r\n") + 2; // Найти начало текста SMS
      String command = message.substring(index); // Извлечь текст сообщения

      // changing url
      const char* pattern = "^id:(%w%w%w%w%w%w%w%w%w%w);url:https?://.*$";
      ms.Target(command.c_str());
      if (ms.Match((char*)pattern) == REGEXP_MATCHED) {
        URL="http";
        int i = 10+7+4+1; // 10 - id, 7 - id:;url:, 4 - http, 1 - possible s
        for(;i<command.length();++i){
          URL += command[i];
        }
        printLCD("URL OK");
        isURLOK = 1;

        if(getRTC().year < 2025){
          SetupRTC();
        }
      }
    }
  }
  if(isURLOK){
    // сделать трансферинг данных с датчиков на сервер/SD-карту 
    // с (возможно "красивыми") часовыми метками 
  }
}

void sendData(const String& command, const int timeout = DEFAULT_TIMEOUT, void(*funcIfNotOk)() = arduinoReset) { //Send command function

  String response = ""; 
  Serial1.println(command); 
  long int time = millis();
  while( (time+timeout) > millis()){
    while(Serial1.available()){       
      response += (char)Serial1.read(); 
    }  
  }
  if(!findOk(response)) {
      lcd.clear();
      lcd.print("BAD");
      if(funcIfNotOk == arduinoReset){
        lcd.setCursor(0, 1);
        lcd.print("Resetting...");
        lcd.setCursor(0, 0);
        delay(5000);  
      }
      funcIfNotOk();
  }

  #ifdef DEBUG
    Serial.println();
    Serial.print(response);
    CHECK

    if(!findOk(response)) {
      Serial.println("Start Resetting... \n");
      funcIfNotOk();
    }

    Serial.println();
  #endif
}

String getData(const String& command, const int timeout = DEFAULT_TIMEOUT) {
  String response = ""; 
  Serial1.println(command); 
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial1.available()){       
        response += (char)Serial1.read(); 
      }  
    }    
    #ifdef DEBUG
      Serial.println();
      Serial.print(response);
      CHECK
      Serial.println();
    #endif

    return response;
}

//TODO: slightly remake this function for server purpose
void sendTempreature(float tempreature, String termistorpPath) {
  sendData("AT+HTTPPARA=\"URL\", \""+URL+"\"");
  String jsonData = "{\"t\":\"" + String(tempreature) + "\", \"p\":\"" + RED_NAME + "/" + termistorpPath +"\"}";

  sendData("AT+HTTPDATA="+ String(jsonData.length())+",10000");
  delay(100);
  Serial1.print(jsonData);
  delay(100);
  Serial1.write(26); // Ctrl+Z in ASCII
  delay(100);
  sendData("AT+HTTPACTION=1", DEFAULT_TIMEOUT, plug);
  delay(1000);
  
  String response = getData("AT+HTTPREAD");
#ifdef DEBUG
  Serial.println(response);
#endif
}

float TempreatureFromAdc(const int16_t& thermistor_adc_val) {
  float output_voltage, thermistor_resistance, therm_res_ln, temperature;  
  
  output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  thermistor_resistance = ( ( 5 * ( 10.0 / output_voltage ) ) - 10 ); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000 ; /* Resistance in ohms   */
  therm_res_ln = log(thermistor_resistance);
  temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
  return temperature - 273.15; /* Temperature in degree Celsius */
}

bool findOk(const String& txt) {
  for(int i = 0; i+1 < txt.length(); ++i) {
    if (txt.substring(i, i+2) == "OK") {
      return true;
    }
  }

  return false;
}

void arduinoReset() {
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, LOW);
  delay(100);  
  digitalWrite(reset_pin, HIGH);  
}

// Преобразование из двоично-десятичного формата в десятичный
uint8_t bcdToDec(uint8_t val) {
  return (val / 16 * 10) + (val % 16);
}

// Преобразование из десятичного формата в двоично-десятичный
uint8_t decToBcd(uint8_t val) {
  return (val / 10 * 16) + (val % 10);
}

// Функция для записи времени в RTC
void setRTC(const MyDateTime &dt) {
  myWire.beginTransmission(RTC_ADDRESS);
  myWire.write(0x00); // Указываем адрес регистра секунд
  myWire.write(decToBcd(dt.second));
  myWire.write(decToBcd(dt.minute));
  myWire.write(decToBcd(dt.hour));
  myWire.write(decToBcd(0)); // День недели (не используется)
  myWire.write(decToBcd(dt.day));
  myWire.write(decToBcd(dt.month));
  myWire.write(decToBcd(dt.year - 2000));
  myWire.endTransmission();
}

// Функция для чтения времени из RTC
MyDateTime getRTC() {
  myWire.beginTransmission(RTC_ADDRESS);
  myWire.write(0x00); // Указываем адрес регистра секунд
  myWire.endTransmission();

  myWire.requestFrom(RTC_ADDRESS, SCL_pin);
  uint8_t second = bcdToDec(myWire.read() & 0x7F);
  uint8_t minute = bcdToDec(myWire.read());
  uint8_t hour = bcdToDec(myWire.read());
  myWire.read(); // Пропускаем день недели
  uint8_t day = bcdToDec(myWire.read());
  uint8_t month = bcdToDec(myWire.read());
  uint16_t year = 2000 + bcdToDec(myWire.read());

  return MyDateTime(year, month, day, hour, minute, second);
}

void printLCD(const String& txt, const String& txt2 = "") {
  lcd.clear();
  lcd.print(txt);
  if(txt != "") {
    lcd.setCursor(0, 1);
    lcd.print(txt2);
  }
  lcd.setCursor(0, 0);
}

int split(String data, char delimiter, String result[]) {
  int count = 0;

  while (data.length() > 0) {
    int index = data.indexOf(delimiter);
    if (index == -1) {
      // No more delimiters, add the last part
      result[count++] = data;
      break;
    }

    // Add the substring to the result array
    result[count++] = data.substring(0, index);

    // Remove the processed part
    data = data.substring(index + 1);

    // Prevent overflow
    if (count >= MAX_PARTS) {
      break;
    }
  }

  return count; // Return the number of parts
}

void SetupRTC() {
  if(isURLOK){
    sendData("AT+HTTPPARA=\"URL\", \""+URL+"/time\"");
    // String jsonData = "{\"t\":\"" + String(tempreature) + "\", \"p\":\"" + RED_NAME + "/" + termistorpPath +"\"}";
    // sendData("AT+HTTPDATA="+ String(jsonData.length())+",10000");
    // delay(100);
    // Serial1.print(jsonData);
    delay(100);
    Serial1.write(26); // Ctrl+Z in ASCII
    delay(100);
    sendData("AT+HTTPACTION=0", DEFAULT_TIMEOUT, plug);
    delay(1000);
    String response = getData("AT+HTTPREAD");

    // checking dateTime
    const char* pattern = "^(%d%d%d%d) (%d%d) (%d%d) (%d%d) (%d%d) (%d%d)$";
    ms.Target(response.c_str());
    if (ms.Match((char*)pattern) != REGEXP_MATCHED) {
      printLCD("BAD URL or SD", "Resetting..");
      delay(5000);
      arduinoReset();
    } 

    //spliting
    int dt[6];
    String buf = "";
    for (int i = 0, j=0; i < response.length(); ++i) {
      if (response[i] == " ") {
        if(i == (response.length()-1)){
          buf += response[i];
          dt[j] = buf.toInt();
          break;
        }
        dt[j] = buf.toInt();
        buf = "";
        ++j;
      }
      buf += response[i];
    } 

    // Setting RTC
    setRTC(MyDateTime(dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]));
    printLCD("RTC OK");
    isRTCOK = 1;
  }
}

//const int DanfossAK32R_Pin = A1;
//typedef int16_t ia; // adc in integer
//ia U0, U100, P100;
// inline double barFromAdc(const int16_t& adc) {
//   return (double(adc)-U0)/(U100-U0)*P100; 
// }
//  U0 = 162, U100 = 1012, P100 = 14;  // preassure param. that depends on special sensors.
  //ia DanfossAK32R_Output = analogRead(DanfossAK32R_Pin);


  // Serial.println("Enabling server settings... ");    
  // Serial1.println("AT+CIPMUX=1");            // Включаем мультиканальный режим
  // delay(1000);
  // Serial1.println("AT+CIPSERVER=1,80");      // Открываем сервер на порту 80
  // delay(1000);