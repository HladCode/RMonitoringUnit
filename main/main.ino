#include <LiquidCrystal_I2C.h>
#include "Wire.h"
#include "FS.h"

#include <Regexp.h>
#include "esp_system.h"

#define DEBUG
//#define SD_OFF

#ifndef SD_OFF
#include <SPI.h>
#include <SD.h>
#endif

#define DEFAULT_TIMEOUT 5000
#define MODEM_RST             5
#define MODEM_PWRKEY          4
#define MODEM_POWER_ON       23
#define LED_GPIO             13
#define MODEM_TX 27
#define MODEM_RX 26
#define LED_ON               HIGH
#define LED_OFF              LOW

#define thermistor_output1 35
// #define SDA_pin 6
 #define SCL_pin 22

// Software SPI for SD card module
int sck = 14;
int miso = 2;
int mosi = 15;
int cs = 13;

#define RTC_ADDRESS 0x68
#define LCD_ADDRESS 0x27

// SoftwareWire Wire(SDA_pin, SCL_pin);
LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2);
// Sd2Card card;
// SdVolume volume;
// SdFile root;

 // regexp

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

void programReset();
bool findOk(const String& txt);
void sendData(const String& command, const int timeout = DEFAULT_TIMEOUT, void(*funcIfNotOk)() = programReset) { //Send command function
  String response = ""; 
  Serial1.println(command); 
  long int time = millis();
  while( (time+timeout) > millis()){
    while(Serial1.available()){       
      response += (char)Serial1.read(); 
    }  
  }

#ifdef DEBUG
  Serial.println(response);
#endif

  if(!findOk(response)) {
      if(funcIfNotOk = plug) {
        return;
      }

      lcd.clear();
      lcd.print(command);
      funcIfNotOk();
  }
}

String getData(const String& command, const int timeout = DEFAULT_TIMEOUT) {
  String response = ""; 
  Serial1.print(command + "\r\n");
  delay(500);
  
  long int startTime = millis();
  while (!Serial1.available() && millis() - startTime < timeout) {
    delay(10);  // Ждем, пока появятся данные
  }
  Serial.println("Serial1.available: " + String(Serial1.available()));

  Serial1.flush();


  long int time = millis();
  while ((time + timeout) > millis()) {
    while (Serial1.available()) {       
      char c = (char)Serial1.read(); 
      response += c;
      Serial.print(c); 
    }
  }    

  Serial.println("\ngetData end");

  return response;
}

void setupModem();
void sendFloatToServer(float valueToSend, MyDateTime dt, String SensorPinNumber, String Purpose);
float TempreatureFromAdc(const int16_t& thermistor_adc_val);


void plug() {}

uint8_t bcdToDec(uint8_t val);
uint8_t decToBcd(uint8_t val);

void setRTC(const MyDateTime &dt);
MyDateTime getRTC();
void SetupRTC();

void printLCD(const String& txt, const String& txt2 = "") {
  lcd.clear();
  lcd.print(txt);
  if(txt != "") {
    lcd.setCursor(0, 1);
    lcd.print(txt2);
  }
  lcd.setCursor(0, 0);
}


bool createDir(fs::FS &fs, const char *path);
bool writeFile(fs::FS &fs, const char *path, const char *message);
bool appendFile(fs::FS &fs, const char *path, const char *message);

bool isURLOK = 0;
bool isRTCOK = 0;

String URL;
char ID[10];

#ifdef DEBUG
void checkMemory() {
  Serial.print("Свободная память (heap): ");
  Serial.println(esp_get_free_heap_size());
}
#endif 

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif 

  lcd.init();                      
  lcd.backlight();
  printLCD("LCD and wire OK"); 
  
  //Power on the SIM800H
  setupModem();
  printLCD("SIM800H", "Powered");
  delay(5000);
 
  printLCD("AT and Serial1", "initialization.. ");   
  Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  Serial1.println("AT");
  delay(1000);
  if (Serial1.available()) {
      String response = Serial1.readString();
      Serial.println(response);
      if (response.indexOf("OK") >= 0) {
          printLCD("Serial1&AT OK");
          delay(3000);
      } else {
          printLCD("No OK response");
          programReset();
      }
  } else {
      printLCD("No response");
      programReset();
  }
  printLCD("AT commands", "initialization.. ");   

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
  printLCD("HTTP", "initialization..");  
  sendData("AT+HTTPINIT");
  sendData("AT+HTTPTERM");
  sendData("AT+HTTPINIT");
  printLCD("AT+HTTPINIT OK");
  printLCD("HTTP parameters", "setting...");  
  sendData("AT+HTTPPARA=\"CID\", 1");
  printLCD("CID OK");
  sendData("AT+HTTPPARA=\"CONTENT\", \"application/json\"");
  printLCD("CONTENT OK");
  sendData("AT+HTTPPARA?");
  printLCD("AT+HTTPPARA? OK");
  sendData("AT+CIPRXGET?");
  
  
#ifndef SD_OFF
  printLCD("Initializing", "SD card..");
  SD.end();

  SPI.begin(sck, miso, mosi, cs);
  if (!SD.begin(cs, SPI)) {
    printLCD("SD module BAD", "Resetting...");
    delay(5000);
    programReset();
  } else {
    printLCD("SD module OK");
  }

  //Example: 12345abcde http://www.example.com
  printLCD("Reading", "config.txt");
  if(!SD.exists("/Config.txt")) {
    printLCD("Config.txt BAD", "RAAF"); // Reset after adding file
    programReset();
  }
  File myFile = SD.open("/Config.txt", FILE_READ);
  int i = 0;
  while (i < 10) {
    ID[i] = char(myFile.read());
    ++i;
  }
  myFile.read();

  while (myFile.available()) {
    URL += char(myFile.read());
  }

  int bufCount = URL.indexOf("\n");
  if(bufCount >= 0){
    URL.remove(bufCount);
  }
  printLCD("Config OK");
  myFile.close();
#endif
  //TODO: сделать перепроверку URL, рабочий ли он
  isURLOK = 1;

#ifdef SD_OFF
  URL = "URL";
  ID[0]='D';
  ID[1]='_';
  ID[2]='_';
  ID[3]='_';
  ID[4]='_';
  ID[5]='_';
  ID[6]='_';
  ID[7]='_';
  ID[8]='_';
  ID[9]='_';
#endif

  if(getRTC().year < 2025){
    printLCD("RTC BAD");
    SetupRTC();
  } else {
    printLCD("RTC OK");
    isRTCOK = 1;
  }
  isRTCOK = 1;
  
  if(isURLOK){
    printLCD("Data transfering", "started");  
  } else {
    printLCD("Waiting URL via","SMS (user app)");  
  }
}
  
void loop() {
  if (Serial1.available()) {
    String message = Serial1.readString();

    // Проверка на наличие заголовка SMS
    if (message.indexOf("+CMT:") != -1) {

      // TODO: сделать код более понятней и сделать так чтоб ссылка на сервер сохранялась

      // Извлечение текста SMS
      int index = message.indexOf("\r\n") + 2; // Найти начало текста SMS
      String command = message.substring(index); // Извлечь текст сообщения

      // changing url
      const char* pattern = "^id:(%w%w%w%w%w%w%w%w%w%w);url:https?://.*$";
      MatchState ms;
      char* command_pchar = strdup(command.c_str());
      ms.Target(command_pchar);
      if (ms.Match((char*)pattern) == REGEXP_MATCHED && command.startsWith(String("id:")+String(ID))) {
        URL="http";
        uint8_t i = 10+7+4+1; // 10 - id, 7 - id:;url:, 4 - http, 1 - possible s
        for(;i<command.length();++i){
          URL += command[i];
        }
        printLCD("URL OK");
        isURLOK = 1;

        if(getRTC().year < 2025){
          SetupRTC();
          isRTCOK = 1;
        }
      }
    }
  }

  //TODO: write time on LCD here

  if(isURLOK && isRTCOK){
    MyDateTime dt = getRTC();
    String dataPath = "/t(C)/0/"+String(dt.year)+"/"+String(dt.month)+"/"+String(dt.day);

//    sendData("AT+HTTPSTATUS?");
    sendFloatToServer(TempreatureFromAdc(analogRead(thermistor_output1)), dt, "35", "t(C)");
#ifndef SD_OFF
    if(!SD.exists(dataPath+"/data.txt")){
      SD.mkdir("/t(C)");
      SD.mkdir("/t(C)/0/");
      SD.mkdir("/t(C)/0/"+String(dt.year));
      SD.mkdir("/t(C)/0/"+String(dt.year)+"/"+String(dt.month));
      SD.mkdir(dataPath);
    }
    File d1 = SD.open(dataPath+"/data.txt", FILE_APPEND);
    d1.println(String(dt.hour)+":"+String(dt.minute)+":"+String(dt.second)+" "+String(TempreatureFromAdc(analogRead(thermistor_output1))));
    d1.close();
#endif
  }
}

void sendFloatToServer(float valueToSend, MyDateTime dt, String SensorPinNumber, String Purpose) {
  
  // Serial.println("AT+HTTPPARA=\"URL\",\""+URL+"/data\"");

  sendData("AT+HTTPPARA=\"URL\",\""+URL+"/data\"", DEFAULT_TIMEOUT, plug);
  String jsonData = "{\"id\":\""+String(ID)+
  "\",\"p\":\"" + Purpose + 
  "\",\"n\":\""+SensorPinNumber+
  "\",\"t\":\""+String(dt.year)+" "+String(dt.month)+" "+String(dt.day)+" "+String(dt.hour)+" "+String(dt.minute)+" "+String(dt.second)+
  "\",\"v\":"+String(valueToSend, 2)+"}";

#ifdef DEBUG
  Serial.println(jsonData);
  // Serial.println(jsonData.length());
#endif

  delay(100);
  Serial.println(getData("AT+HTTPDATA=" + String(jsonData.length()) + ",10000\r\n"));

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
  if (thermistor_adc_val == 0) return NAN; // Проверка на 0, чтобы избежать деления на ноль

  float output_voltage = (thermistor_adc_val * 3.3) / 4095.0;  // Используем 3.3V и 12-битный АЦП ESP32

  if (output_voltage == 0) return NAN; // Проверка на нулевое напряжение

  float thermistor_resistance = ( ( 3.3 * ( 10.0 / output_voltage ) ) - 10 ); // 10kΩ номинал
  thermistor_resistance *= 1000; // Перевод в Ом

  if (thermistor_resistance <= 0) return NAN; // Проверка на отрицательное сопротивление

  float therm_res_ln = log(thermistor_resistance);
  float temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln ) ) );

  return temperature - 273.15; // Конвертация в градусы Цельсия
}

bool findOk(const String& txt) {
  return txt.indexOf("OK") >= 0;
}

void programReset() {
  while(1){}
  // if(SD.exists("Fail_count.txt")) {
  //   // File myFile = SD.open("/Fail_count.txt", FILE_READ);
  //   // char fail_count = myFile.read();
  //   // myFile.close();
  //   //lcd.setCursor(0, 1);
  //   if(fail_count < '3'){
  //     //lcd.setCursor(0, 1);
  //     //lcd.print("Resetting...");
  //     //lcd.setCursor(0, 0);
  //     ++fail_count;

  //     // myFile = SD.open("/Fail_count.txt", FILE_WRITE);
  //     // myFile.seek(0);  // Указатель в начало файла
  //     // myFile.print(fail_count); // Перезаписываем файл
  //     // myFile.close();

  //     delay(5000);
  //     // esp_restart();
  //     while(1){}
  //   } else {
  //     // myFile = SD.open("/Fail_count.txt", FILE_WRITE);
  //     // myFile.seek(0);  // Указатель в начало файла
  //     // myFile.print(0); // Перезаписываем файл
  //     // myFile.close();

  //     //lcd.print("Fatal!!");
  //     //lcd.setCursor(0, 0);
  //     delay(5000);
  //     while(1){}
  //   }
  // } else {
  //   // File myFile = SD.open("/Fail_count.txt", FILE_WRITE);
  //   // myFile.seek(0);  // Указатель в начало файла
  //   // myFile.print(1); // Перезаписываем файл
  //   // myFile.close();

  //   //esp_restart();
  //   while(1){}
  // }
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
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0x00); // Указываем адрес регистра секунд
  Wire.write(decToBcd(dt.second));
  Wire.write(decToBcd(dt.minute));
  Wire.write(decToBcd(dt.hour));
  Wire.write(decToBcd(0)); // День недели (не используется)
  Wire.write(decToBcd(dt.day));
  Wire.write(decToBcd(dt.month));
  Wire.write(decToBcd(dt.year - 2000));
  Wire.endTransmission();
}

// Функция для чтения времени из RTC
MyDateTime getRTC() {
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0x00); // Указываем адрес регистра секунд
  Wire.endTransmission();

  Wire.requestFrom(RTC_ADDRESS, SCL_pin);
  uint8_t second = bcdToDec(Wire.read() & 0x7F);
  uint8_t minute = bcdToDec(Wire.read());
  uint8_t hour = bcdToDec(Wire.read());
  Wire.read(); // Пропускаем день недели
  uint8_t day = bcdToDec(Wire.read());
  uint8_t month = bcdToDec(Wire.read());
  uint16_t year = 2000 + bcdToDec(Wire.read());

  return MyDateTime(year, month, day, hour, minute, second);
}

void SetupRTC() {
  if(isURLOK){

    sendData("AT+HTTPPARA=\"URL\",\""+URL+"/time\"", DEFAULT_TIMEOUT, plug);
    delay(100);
    Serial1.write(26); // Ctrl+Z in ASCII
    delay(100);
    sendData("AT+HTTPACTION=0", DEFAULT_TIMEOUT, plug);
    delay(2000);
    String response = getData("AT+HTTPREAD");
    delay(2000);

    Serial.println("Response: ");
    Serial.println(response);

    // Serial.println("\nCheck:");
    // Serial.println(int('\n'));
    // Serial.println(int(' '));
    // Serial.println();

    // for(int i = 0; i < response.length();++i){
    //   Serial.println(int(response[i]));
    // }
    // Serial.println("\n\nEnd");

    // checking dateTime
    const char* pattern = ".-\n(%d%d%d%d) (%d%d) (%d%d) (%d%d) (%d%d) (%d%d)\r.*";
    MatchState ms;
    char* pResponse = strdup(response.c_str());
    ms.Target(pResponse);
    if (ms.Match((char*)pattern) != REGEXP_MATCHED) {
      printLCD("BAD URL or SD", "Resetting..");
      delay(5000);
      programReset();
    } 

    //spliting
    uint16_t dt[6];
    String buf = "";

    int start_index = response.indexOf("2");
    for (int i = start_index, j=0; i < response.length(); ++i) {
      if (response[i] == ' ') {
        if(i == (response.length()-1)){
          buf += response[i];
          dt[j] = buf.toInt();
          break;
        }
        dt[j] = buf.toInt();
        buf = "";
        ++j;
        continue;
      }
      buf += response[i];
    } 

    // Setting RTC
    setRTC(MyDateTime(dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]));
    printLCD("RTC OK");
    isRTCOK = 1;
  }
}

void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
   
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    // digitalWrite(MODEM_PWRKEY, HIGH);
    // delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(2000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LED_OFF);
}

// bool createDir(fs::FS &fs, const char *path) {
//   return fs.mkdir(path);
// }

// bool writeFile(fs::FS &fs, const char *path, const char *message) {
//   File file = fs.open(path, FILE_WRITE);
//   if (!file) {
//     return 0;
//   }
//   bool res = 0;
//   if (file.print(message)) {
//     res = 1;
//   }
//   file.close();
//   return res;
// }

// bool appendFile(fs::FS &fs, const char *path, const char *message) {
//   File file = fs.open(path, FILE_APPEND);
//   if (!file) {
//     return 0;
//   }
//   bool res = 0;
//   if (file.print(message)) {
//     res = 1;
//   } 
//   file.close();
//   return res;
// }

//const int DanfossAK32R_Pin = A1;
//typedef int16_t ia; // adc in integer
//ia U0, U100, P100;
// inline double barFromAdc(const int16_t& adc) {
//   return (double(adc)-U0)/(U100-U0)*P100; 
// }
//  U0 = 162, U100 = 1012, P100 = 14;  // preassure param. that depends on special sensors.
  //ia DanfossAK32R_Output = analogRead(DanfossAK32R_Pin);