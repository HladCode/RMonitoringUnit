#include <math.h>
#define DEBUG 1
#define CHECK Serial.println("✓");
#define DEFAULT_TIMEOUT 2000

//typedef int16_t ia; // adc in integer

//const int DanfossAK32R_Pin = A1;
const int thermistor_output = A0;

//ia U0, U100, P100;

void sendData(const String& command, const int timeout = DEFAULT_TIMEOUT);
String getData(const String& command, const int timeout = DEFAULT_TIMEOUT);
float TempreatureFromAdc(const int16_t& thermistor_adc_val);

// inline double barFromAdc(const int16_t& adc) {
//   return (double(adc)-U0)/(U100-U0)*P100; 
// }

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
  sendData("AT+CSQ"); // Перевірка рівня сигналу
  sendData("AT+CREG?"); // Перевірка реєстрації в мережі
  sendData("AT+CSTT=\"internet.kyivstar.net\",\"\",\"\""); // Встановлення APN
  sendData("AT+CIICR"); // Підняття бездротового з'єднання
  sendData("AT+SAPBR=3,1,\"Contype\",\"GPRS\""); // Встановлення параметрів профілю носія
  sendData("AT+SAPBR=3,1,\"APN\",\"internet.kyivstar.net\""); // Встановлення APN
  sendData("AT+SAPBR=1,1"); // Відкриття профілю носія
  sendData("AT+SAPBR=2,1"); // Перевірка статусу профілю носія
  Serial.println("\n");

  Serial.println("HTTP initialization... ");    
  sendData("AT+HTTPINIT");
  Serial.println("\n");

  Serial.println("HTTP parameters setting... ");  
  sendData("AT+HTTPPARA=\"CID\", 1");
  sendData("AT+HTTPPARA=\"URL\", \"http://91.244.23.42:1488/data\"");
  sendData("AT+HTTPPARA=\"CONTENT\", \"application/json\"");
  sendData("AT+HTTPPARA?");

  Serial.println("\n");
  Serial.println(" -- Data transfer start -- ");  

  pinMode(thermistor_output, INPUT);
//  U0 = 162, U100 = 1012, P100 = 14;  // в дальнейшем эти значения может задавать сервер или считывание из файла из-за разости параметров сенсоров

  

}
  
void loop() {
  //ia DanfossAK32R_Output = analogRead(DanfossAK32R_Pin);
  float thermistor_Output = TempreatureFromAdc(analogRead(thermistor_output));
  String jsonData = "{\"t\":\"" + String(thermistor_Output) + "\", \"p\":\"Ukraine/Cherkassy/CityMarket/Address/R2\"}";
  //Serial.println("\n"+ jsonData.length() +"\n");
  sendData("AT+HTTPDATA="+ String(jsonData.length())+",10000");
  delay(100);
  Serial1.print(jsonData);
  delay(100);
  Serial1.write(26); // ASCII код Ctrl+Z
  delay(100);
  sendData("AT+HTTPACTION=1");
  delay(1000);
  
  String response = getData("AT+HTTPREAD");
  Serial.println(response);
  delay(1000);

  //Serial.print("Temperature: " + String(TempreatureFromAdc(thermistor_Output), 4)+"\n\n\n");

}

void sendData(const String& command, const int timeout = DEFAULT_TIMEOUT) { //Send command function
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

float TempreatureFromAdc(const int16_t& thermistor_adc_val) {
  float output_voltage, thermistor_resistance, therm_res_ln, temperature;  
  
  output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  thermistor_resistance = ( ( 5 * ( 10.0 / output_voltage ) ) - 10 ); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000 ; /* Resistance in ohms   */
  therm_res_ln = log(thermistor_resistance);
  temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
  return temperature - 273.15; /* Temperature in degree Celsius */
}