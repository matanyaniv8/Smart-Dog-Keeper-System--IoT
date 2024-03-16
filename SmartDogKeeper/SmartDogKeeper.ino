//************************************************************//
// Blynk configuration - must be at the very top of the page!
// Replace with your tokens from Blynk
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""
// Replace with your tokens from adafruit.
#define MQTT_USERNAME  ""
#define MQTT_PASSWORD  ""
//************************************************************//
// Libaries in use.
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

// Tempeerature Sensor
uint32_t delayMS;
sensors_event_t tempEvent;
double roomTemp;
int outsideTemp;
double roomHumidityLevel;

//Servo variables
Servo myservo; 
int pos = ON_POSITION;
bool isFanOn = false;

// Water Sensor
int waterLevel;

// WiFi credentials
// Please fill in your WiFi credentials in order to connect the controller to the internet.
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
const long interval = 3600000;     // One hour in milliseconds

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
