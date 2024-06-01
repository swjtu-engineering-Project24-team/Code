/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ArduinoJson.h>
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


#define LED 12 //LED灯连接到GPIO12,用LED灯指示设备状态


#define product_id "wQ1H3gUGA3" //产品ID，改为自己的产品ID
#define device_id "ds18b20_dht11" //设备ID，改为自己的设备ID
#define token "version=2018-10-31&res=products%2FwQ1H3gUGA3%2Fdevices%2Fds18b20_dht11&et=1717569887&method=md5&sign=cSTkV3u1iTT8HUG%2Bs%2BJC%2Fg%3D%3D" //token，改为自己的token

const char* ssid = "iPhone12";//WiFi名称，改为自己的WiFi名称
const char* password = "1234567876";//WiFi密码，改为自己的WiFi密码



const char* mqtt_server = "mqtts.heclouds.com";//MQTT服务器地址
const int mqtt_port = 1883;//MQTT服务器端口

#define ONENET_TOPIC_PROP_POST "$sys/" product_id "/" device_id "/thing/property/post"
//设备属性上报请求,设备---->OneNET
#define ONENET_TOPIC_PROP_SET "$sys/" product_id "/" device_id "/thing/propertyt"
//设备属性设置请求,OneNET---->设备
#define ONENET_TOPIC_PROP_POST_REPLY "$sys/" product_id "/" device_id "/thing/property/post/reply"
//设备属性上报响应,OneNET---->设备
#define ONENET_TOPIC_PROP_SET_REPLY "$sys/" product_id "/" device_id "/thing/propertyt_reply"
//设备属性设置响应,设备---->OneNET
#define ONENET_TOPIC_PROP_FORMAT "{\"id\":\"%u\",\"version\":\"1.0\",\"params\":%s}"
//设备属性格式模板
int postMsgId = 0;//消息ID,消息ID是需要改变的,每次上报属性时递增

//按照自己的设备属性定义，可以通过DHT11传感器获取温湿度，也可以通过开关控制LED，这里只是模拟
float tempc = 28.0;//温度
float tempf = 97.0;//温度
int humi = 60;//湿度
int light_ad=0;
bool LED_Status = false;//LED状态


WiFiClient espClient;//创建一个WiFiClient对象
PubSubClient client(espClient);//创建一个PubSubClient对象
Ticker ticker;//创建一个定时器对象

void LED_Flash(int time);
void WiFi_Connect();
void OneNet_Connect();
void OneNet_Prop_Post();
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  pinMode(LED, OUTPUT);//LED灯设置为输出模式
  Serial.begin(115200);//串口初始化,波特率9600,用于输出调试信息，这里串口波特率要与串口监视器设置的一样，否则会乱码
  WiFi_Connect();//连接WiFi
  OneNet_Connect();//连接OneNet
  ticker.attach(10, OneNet_Prop_Post);//定时器,每10ms执行一次OneNet_Prop_Post函数


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
  if (isnan(h) || isnan(t) || isnan(f))
  {
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
  humi=h;
  Serial.printf("\n");
  //温度测量

  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  tempc=temperatureC;
  tempf=temperatureF;
  Serial.printf("第%ld次温度采集结果:\n", count);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");

  //光强测量
  // read the analog / millivolts value for pin 2:
  int analogValue = analogRead(1);
  int analogVolts = analogReadMilliVolts(1);
  light_ad=4095 - analogValue;
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




  if(WiFi.status() != WL_CONNECTED) {//如果WiFi连接断开
    WiFi_Connect();//重新连接WiFi
  }
  if (!client.connected()) {//如果MQTT连接断开
    OneNet_Connect();//重新连接OneNet
  }
  client.loop();//保持MQTT连接
}

void LED_Flash(int time) {
  digitalWrite(LED, HIGH);//点亮LED
  delay(time);//延时time
  digitalWrite(LED, LOW);//熄灭LED
  delay(time);//延时time
}

void WiFi_Connect()
{
  WiFi.begin(ssid, password);//连接WiFi
  while (WiFi.status() != WL_CONNECTED) {//等待WiFi连接,WiFI.status()返回当前WiFi连接状态,WL_CONNECTED为连接成功状态
    LED_Flash(500);//LED闪烁,循环等待
    Serial.println("\nConnecting to WiFi...");
  }
  Serial.println("Connected to the WiFi network");//WiFi连接成功
  Serial.println(WiFi.localIP());//输出设备IP地址
  digitalWrite(LED, HIGH);//点亮LED,表示WiFi连接成功
}

void OneNet_Connect()
{
  client.setServer(mqtt_server, mqtt_port);//设置MQTT服务器地址和端口
  client.connect(device_id, product_id, token);//连接OneNet
  if(client.connected()) //如果连接成功
  {
    LED_Flash(500);
    Serial.println("Connected to OneNet!");
  }
  else
  {
    Serial.println("Failed to connect to OneNet!");
  }
  client.subscribe(ONENET_TOPIC_PROP_SET);//订阅设备属性设置请求, OneNET---->设备
  client.subscribe(ONENET_TOPIC_PROP_POST_REPLY);//订阅设备属性上报响应,OneNET---->设备
  client.setCallback(callback);//设置回调函数
}
//上报设备属性
void OneNet_Prop_Post()
{
  if(client.connected()) 
  {
    char parmas[256];//属性参数
    char jsonBuf[256];//JSON格式数据,用于上报属性的缓冲区
    sprintf(parmas, "{\"tempc\":{\"value\":%.1f},\"tempf\":{\"value\":%.1f},\"humi\":{\"value\":%d},\"light_ad\":{\"value\":%d}}", tempc,tempf,humi,light_ad);//设置属性参数
    Serial.println(parmas);
    sprintf(jsonBuf,ONENET_TOPIC_PROP_FORMAT,postMsgId++,parmas);//设置JSON格式数据,包括消息ID和属性参数
    Serial.println(jsonBuf);
    if(client.publish(ONENET_TOPIC_PROP_POST, jsonBuf))//上报属性
    {
      LED_Flash(500);
      Serial.println("Post property success!");
    }
    else
    {
      Serial.println("Post property failed!");
    }
  }
}
//回调函数，当订阅的主题有消息时，会调用此函数
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);//打印主题
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);//打印消息内容
  }
  Serial.println();//换行
  LED_Flash(500);//LED闪烁,表示收到消息
  if(strcmp(topic, ONENET_TOPIC_PROP_SET) == 0)
  {
    DynamicJsonDocument doc(100);//创建一个JSON文档,用于解析消息内容

    DeserializationError error = deserializeJson(doc, payload);//解析消息内容,将消息内容存储到doc中,返回解析结果,成功返回0,失败返回其他值
    if (error) {//解析失败,打印错误信息,返回
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    JsonObject setAlinkMsgObj = doc.as<JsonObject>();//获取JSON文档的根对象
    JsonObject params = setAlinkMsgObj["params"];//获取params对象
    if(params.containsKey("LED"))//判断params对象是否包含LED属性
    {
      LED_Status = params["LED"];//获取LED属性值
      Serial.print("Set LED:");
      Serial.println(LED_Status);
    }
    serializeJsonPretty(setAlinkMsgObj, Serial);//打印JSON文档
    String str = setAlinkMsgObj["id"];//获取消息ID
    char SendBuf[100];//发送缓冲区
    sprintf(SendBuf, "{\"id\":\"%s\",\"code\":200,\"msg\":\"success\"}", str.c_str());//设置响应消息
    Serial.println(SendBuf);
    delay(100);
    if(client.publish(ONENET_TOPIC_PROP_SET_REPLY, SendBuf))//发送响应消息,不知道为什么，这里发送成功的，但是OneNET平台上没有收到
    {
      Serial.println("Send set reply success!");
    }
    else
    {
      Serial.println("Send set reply failed!");
    }
  }
}
