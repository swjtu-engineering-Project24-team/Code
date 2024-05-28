/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// GPIO where the DS18B20 is connected to
const int oneWireBus = 0;
#define Led1Pin 3
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

#define DHTPIN 2  // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11  // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);
  // Start the DS18B20 sensor
  sensors.begin();
  // Initialize device.
  dht.begin();
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
  pinMode(Led1Pin, OUTPUT);
}

void loop() {
  static u_int64_t count = 1;
  //湿度测量
  // Wait a few seconds between measurements.
  delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  Serial.printf("----------------------------------------\n");
  Serial.printf("第%ld次湿度采集结果:\n", count);
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.printf("\n");
  //温度测量

  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.printf("第%ld次温度采集结果:\n", count);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");

  //光强测量
  // read the analog / millivolts value for pin 2:
  int analogValue = analogRead(1);
  int analogVolts = analogReadMilliVolts(1);

  Serial.printf("第%ld次光照强度采集结果:\n", count);
  // print out the values you read:
  Serial.printf("ADC value(光照强度) = %d\n", 4095 - analogValue);
  Serial.printf("ADC millivolts value(电压值) = %d毫伏\n", analogVolts);

  if (4095 - analogValue < 2000)
    digitalWrite(Led1Pin, HIGH);
  else
    digitalWrite(Led1Pin, LOW);
  count += 1;
  delay(2000);
}
