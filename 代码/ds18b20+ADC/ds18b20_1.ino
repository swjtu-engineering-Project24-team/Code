/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <OneWire.h>
#include <DallasTemperature.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 0;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);
  // Start the DS18B20 sensor
  sensors.begin();

  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
}

void loop() {
  static u_int64_t count=1;
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.printf("第%ld次温度采集结果\n:",count);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");

  // read the analog / millivolts value for pin 2:
  int analogValue = analogRead(1);
  int analogVolts = analogReadMilliVolts(1);

  Serial.printf("第%ld次光照强度采集结果:\n",count);
  // print out the values you read:
  Serial.printf("ADC value(光照强度) = %d\n",4095-analogValue);
  Serial.printf("ADC millivolts value(电压值) = %d毫伏\n",analogVolts);
  count+=1;
  delay(2000);
}