// Blynk configuration - must be at the very top of the page!
#define BLYNK_TEMPLATE_ID "TMPL6uopzbp08"
#define BLYNK_TEMPLATE_NAME "IoTFinalEx"
#define BLYNK_AUTH_TOKEN "zQ5uaroVXgdYnBvntzvW8sZ3E-wi_Q06"

#define MQTT_USERNAME  "msassignments4"
#define MQTT_PASSWORD  "aio_vbwF28ELzr7aXhTiwn5NIdzjnnOu"
/*=================================================
    Dog Keeper - Arduino & Blynk & adafruitversion:
  =================================================
  - The System:
==================================================================================================================================================================================================================
This Arduino code is designed to create an interactive and dynamic smart dog keeper using an ESP32 controller and Blynk, make.com, and adafruit.io services. 
The system uses a servo motor to turn the AC ON/OFF, temperature, and humidity sensor for measuring the room's temperature and humidity, and a water sensor for measuring the water level within a bowl. 
Our two scenarios controlled by the ESP32 are:
  1. The ESP32 monitors the room's temperature. We created an automation with the ESP32 such that when it senses that the gap between the room's temperature and outside temperature
  is more than 2 degrees, it activates the servo motor and turns on the AC. 
  Once the gap has minimized to less than 2 degrees, it activates the servo again and turns the AC OFF.
  2. The ESP32 checks and update the water level in the dog's water bowl. !!!!! shahar, pls continue !!!!!!

Our two scenarios coming to the ESP32 from outside services are:
  1.The user city's outside temperature value coming from make.com weather services is being updated every 15 minutes within the make.com, which sends the value to the virtual pin of the outside temperature,
  that triggers an update off the outside temperature variable that ESP32 saves.
  This update can trigger the automation mentioned above in section 1. 
  2.  The user pressed the ON/OFF button in the Blynk interface – the ESP32 turns ON/OFF accordingly using the servo motor. 

Also, we create a dashboard with Adafruit using MQTT requests to notify the user by email once the water level is low and the room's temperature is too high.
Adafruit also creates graphs and statistics on the above information so the user can track its dog water consumption and room temperature.
==================================================================================================================================================================================================================
  - The Circuit :
  ===============
  - input sensor: Temperature Sensor and Water Sensor.
  - output sensor: Servo motor.
  - Video's link: https://drive.google.com/file/d/1lfHAuW2Bzsm6A9jI-Yk3YtplLfj9te91/view?usp=share_link

  - Created By :
  ==============
    Matan Yaniv   #315144436
    Shahar Hahami #318623568
=================================================================================================================================================================================================================*/
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <PubSubClient.h>
#include "SPIFFS.h"

// Temprature Sensor
#define DHTPIN 15
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);

// Water Level Sensor
#define WATER_SENSOR_PIN 34

//Servo
#define OFF_POSITION 140
#define ON_POSITION 60
#define SERVO_PIN 23

// UltraSonic
#define TRIGGER_PIN  16
#define ECHO_PIN     17
#define MAX_DISTANCE 100

// Tempeerature Sensor
uint32_t delayMS;
sensors_event_t tempEvent;
double roomTemp;
int outsideTemp;
double roomHumidityLevel;

//Servo variables
Servo myservo; 
int pos = ON_POSITION;

// UltraSonic variables
int DEFAULT_DISTANCE = 0;
bool is_SomeoneEntered = false;
bool isFanOn = false;

// Water Sensor
int waterLevel;

// WiFi credentials
char ssid[] = "";
char pass[] = "";

// MQTT
const char MQTT_CLIENTID[] = MQTT_USERNAME  __DATE__  __TIME__;
const char* mqtt_server = "io.adafruit.com";
const char temperatureTopic[] = MQTT_USERNAME "/feeds/temperature";
const char humidityTopic[] = MQTT_USERNAME "/feeds/humidity";
const char waterTopic[] = MQTT_USERNAME "/feeds/water-level";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
#define MSG_BUFFER_SIZE (50)
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;
char temp[50];
String tempStr;

unsigned long previousMillis = 0;  // Stores the last time the function was called
const long interval = 3600000;      // One hour in milliseconds

void MQTT(){
  if (!mqttClient.connected()) {
    reconnect();
  }
  
  mqttClient.loop();
  unsigned long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    Serial.println("publishing MQTT Data");
    sampleSensors();
    tempStr = String(roomTemp);
    mqttClient.publish(temperatureTopic, tempStr.c_str());
    tempStr = String(roomHumidityLevel);
    mqttClient.publish(humidityTopic, tempStr.c_str());
    waterLevel = (waterLevel/4095.0) * 10;
    tempStr = String(waterLevel);
    Serial.println(waterLevel);
    mqttClient.publish(waterTopic, tempStr.c_str());
  }
}

void initializeServo(){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(SERVO_PIN, 100, 3000); // attaches the servo onto pin
  myservo.write(OFF_POSITION);
}

// Initialize temperature sensor object and determine the delay for sampling.
void initializeTempSensor(){
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  sampleSensors();
  outsideTemp = 0;
}

// Measures the temperature & humidity and water sensors
void sampleSensors(){
  getWaterLevel();
  getTemperature();
  getHumidity();
  
  updateBlynk();
}

void updateBlynk(){
  Blynk.run();
  Blynk.virtualWrite(V2, roomTemp);
  Blynk.virtualWrite(V3, roomHumidityLevel);
  Blynk.virtualWrite(V0, (waterLevel/4095.0) * 10);
  Blynk.run();
  delay(10);
}

void getWaterLevel() {
  waterLevel = analogRead(WATER_SENSOR_PIN);
  Serial.printf("The current water level is: %d\n", waterLevel);
}

// Measures the temperature from it's sensor.
// Once it failed, it returns 0, and letting the esp32 know that an error occurred.
void getTemperature(){
  dht.temperature().getEvent(&tempEvent);
  if (isnan(tempEvent.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    roomTemp = tempEvent.temperature;
    Serial.printf("Temperature: %.2f°C\n", roomTemp);
  }
}

// Measures the humidity from it's sensor.
// Once it failed, it returns 0, and letting the esp32 know that an error occurred.
void getHumidity(){
  dht.humidity().getEvent(&tempEvent);
  if(isnan(tempEvent.relative_humidity)){
    Serial.println(F("Error reading temperature!"));
  } else {
    roomHumidityLevel = tempEvent.relative_humidity;
    Serial.printf("Humidity: %.2f%%\n", roomHumidityLevel);
  }
}

// Checks whether the room's temperature is above 2 degress or more than the  oustside temperature 
bool isRoomTooHot() {
  return (outsideTemp - roomTemp >= 2); 
}

// Moves the servo to the ON_POSITION and returns to OFF_POSITION.
void moveServo(){
  myservo.write(ON_POSITION);
  delay(600);
  myservo.write(OFF_POSITION);
  delay(200);
}

// Updates the Blynk's fan ON/OFF button.
void setFanSwitchOnOff(){
  Blynk.run();
  if(!isFanOn){
      Blynk.virtualWrite(V1, 1);
      isFanOn = true;
      moveServo();
  }else if(is_SomeoneEntered == false && isFanOn){
      Blynk.virtualWrite(V1, 0);
      isFanOn = false;
      moveServo();
  }
}

void controlFanBasedOnTemperature() {
  Serial.println("Checking temperature differences....");
  Serial.printf("\nOutside temperature: %d\n", outsideTemp);
  if (isRoomTooHot()) {
    if (!isFanOn) {
      Blynk.virtualWrite(V1, 1); // Turn the Blynk switch ON
      isFanOn = true;
      Serial.println("Temperature differences triggered the fan to ON state");
      moveServo(); // Move the servo to ON position
    }
  } else {
    if (isFanOn) {
      Blynk.virtualWrite(V1, 0); // Turn the Blynk switch OFF
      isFanOn = false;
      Serial.println("Temperature differences triggered the fan to OFF state");
      moveServo(); // Move the servo to OFF position
    }
  }
}

//Once the Blynk btn pressed update if the fan is ON/OFF, and moves the servo.
BLYNK_WRITE(V1){
  isFanOn = param.asInt() == 1;
  Serial.println("User turned ON the fan via Blynk app");
  moveServo();
}

// updates the value of the outside temperature coming from the make.com using the blynk urls.
BLYNK_WRITE(V4){
  outsideTemp = param.asInt();
  Serial.printf("\nOutside temperature: %d\n", outsideTemp);
  Blynk.virtualWrite(V4, outsideTemp);
}

void setup_wifi() {
  delay(10);
  Serial.printf("\nConnecting to %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.printf("\nWiFi connected\nIP address: %s\n", WiFi.localIP().toString().c_str());
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.printf("Attempting MQTT connection...");
    String mqttClientId = "ESP32Client-";
    mqttClientId += String(random(0xffff), HEX);
    if (mqttClient.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.printf("connected\n");
    }
    else {
      Serial.printf("failed, rc=%d try again in 5 seconds\n", mqttClient.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "sgp1.blynk.cloud");
  initializeTempSensor(); // temperature sensor initialization
  initializeServo();
  Blynk.virtualWrite(V1, 0); // initialize the fan button to 0
  mqttClient.setServer(mqtt_server, 1883);
  outsideTemp = roomTemp;
}

void loop() {
  Blynk.run();
  MQTT(); 
  sampleSensors();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Update the timer
    previousMillis = currentMillis;
    Serial.printf("Timer reset!");
    controlFanBasedOnTemperature();
  }
}