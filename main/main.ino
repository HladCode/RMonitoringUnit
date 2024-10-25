#include <math.h>
#include <DoubleLinkedList.h>

#define DEBUG 1
#define CHECK Serial.println("✓");
#define DEFAULT_TIMEOUT 2000
#define RED_NAME "Obj1" // RED - Refrigeration Exporter of Data

const int thermistor_output1 = A0;
const int reset_pin = 6;

void arduinoReset();
void sendData(const String& command, const int timeout = DEFAULT_TIMEOUT, void(*funcIfNotOk)() = arduinoReset);
String getData(const String& command, const int timeout = DEFAULT_TIMEOUT);
void sendTempreature(const String& targetURL, float tempreature, String termistorpPath);
float TempreatureFromAdc(const int16_t& thermistor_adc_val);
bool findOk(const String& txt);
void plug() {}



void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  while(!Serial);
  while(!Serial1);

  //Power on the SIM800C
  pinMode(9,OUTPUT);
  digitalWrite(9,HIGH);
  delay(3000);
  digitalWrite(9,LOW);
  delay(1000);
      

  Serial.println(" -- Program start -- ");  
  Serial.println("Serial initialization... ✓");  
  Serial.print("Serial1 initialization... ");  
  while(!Serial1.availableForWrite());
  CHECK
  Serial.println("AT commands initialization... ");    
  sendData("AT");
  Serial.println("\n");

  Serial.println("Sim-card setup... ");    
  sendData("AT+CSQ"); // checking signal level
  sendData("AT+CREG?"); // checking web registration 
  sendData("AT+CSTT=\"internet.kyivstar.net\",\"\",\"\""); // installing APN
  sendData("AT+CIICR"); // Raising a wireless connection
  sendData("AT+SAPBR=3,1,\"Contype\",\"GPRS\""); // Setting media profile parameters
  sendData("AT+SAPBR=3,1,\"APN\",\"internet.kyivstar.net\""); // installing APN
  sendData("AT+SAPBR=1,1"); // Opening the carrier profile
  sendData("AT+SAPBR=2,1"); // Checking the carrier profile status
  Serial.println("\n");

  // Serial.println("Enabling server settings... ");    
  // Serial1.println("AT+CIPMUX=1");            // Включаем мультиканальный режим
  // delay(1000);
  // Serial1.println("AT+CIPSERVER=1,80");      // Открываем сервер на порту 80
  // delay(1000);


  Serial.println("HTTP initialization... ");    
  sendData("AT+HTTPINIT");
  Serial.println("\n");

  Serial.println("HTTP parameters setting... ");  
  sendData("AT+HTTPPARA=\"CID\", 1");
  //sendData("AT+HTTPPARA=\"URL\", \"http://92.43.81.152:1488/data\"");
  sendData("AT+HTTPPARA=\"CONTENT\", \"application/json\"");
  sendData("AT+HTTPPARA?");

  Serial.println("\n");
  Serial.println(" -- Data transfer start -- ");  

  pinMode(thermistor_output1, INPUT);  
}
  
void loop() {
  if (Serial1.available()) {
        String message = Serial1.readString();
        if (message.indexOf("+CMT:") != -1) {
            // Извлечение текста SMS
            int index = message.indexOf("\r\n") + 2;
            String command = message.substring(index);
            Serial.println(message)
            Serial.println(command)
        }
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
  if(DEBUG){
    Serial.println();
    Serial.print(response);
    CHECK

    if(!findOk(response)) {
      Serial.println("Start reseting... \n");
      funcIfNotOk();
    }

    Serial.println();
  }  
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
    if(DEBUG){
      Serial.println();
      Serial.print(response);
      CHECK
      Serial.println();
    }

    return response;
}

void sendTempreature(const String& targetURL, float tempreature, String termistorpPath) {
  sendData("AT+HTTPPARA=\"URL\", \""+targetURL+"\"");
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
  Serial.println(response);
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

//const int DanfossAK32R_Pin = A1;
//typedef int16_t ia; // adc in integer
//ia U0, U100, P100;
// inline double barFromAdc(const int16_t& adc) {
//   return (double(adc)-U0)/(U100-U0)*P100; 
// }
//  U0 = 162, U100 = 1012, P100 = 14;  // preassure param. that depends on special sensors.
  //ia DanfossAK32R_Output = analogRead(DanfossAK32R_Pin);
