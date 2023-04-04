#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <WiFi.h>
#include <FirebaseESP32.h>

#define FIREBASE_HOST "https://yongyang-project-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyBk6MUEUi6Fny3JEmP5peKKgnJ3NE1fdQg"

#define WIFI_SSID "Chitralada Technology Institute"
#define WIFI_PASSWORD "Chitralada_2021"

//#define WIFI_SSID "iPhone ของ Pon"
//#define WIFI_PASSWORD "ponBamkyrr"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme1; // I2C
//Adafruit_BME680 bme2; // I2C
Adafruit_BME680 bme3; // I2C
//Adafruit_BME680 bme4; // I2C OUTSIDE

FirebaseData firebaseData; //Firebase

float avgTemp, avgHumi, outTemp, outHumi;

const int fan1 = 12; //FanF
const int fan2 = 14; //FanB
const int heater = 27;
const int adjustHeat = 26;

int modeState = 1;
int checkFlag = 1;
int heaterState = 1;
float inTemp, inHumi;
int calTemp, calHumi;

unsigned long lastMillis = 0; //Timer

// Connect WIFI
void WiFiConnection() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
}

// Select I2C BUS
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// BME SetUp
void bmeSetup(){
//  TCA9548A(2);
//  bool status1 = bme1.begin(0x77);  
//  if (!status1) {
//    Serial.println("Could not find a valid BME680_1 sensor, check wiring!");
//    ESP.restart();
////    while (1);
//  }
//  bme1.setTemperatureOversampling(BME680_OS_8X);
//  bme1.setHumidityOversampling(BME680_OS_2X); 
//  bme1.setPressureOversampling(BME680_OS_4X);
//  bme1.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme1.setGasHeater(320, 150); // 320*C for 150 ms
  
//  TCA9548A(3);
//  bool status2 = bme2.begin(0x77);  
//  if (!status1) {
//    Serial.println("Could not find a valid BME280_2 sensor, check wiring!");
//    ESP.restart();
////    while (1);
//  }
//  // Set up oversampling and filter initialization
//  bme2.setTemperatureOversampling(BME680_OS_8X);
//  bme2.setHumidityOversampling(BME680_OS_2X);
//  bme2.setPressureOversampling(BME680_OS_4X);
//  bme2.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme2.setGasHeater(320, 150); // 320*C for 150 ms

  TCA9548A(7);
  bool status3 = bme3.begin(0x77);  
  if (!status3) {
    Serial.println("Could not find a valid BME280_3 sensor, check wiring!");
    ESP.restart();
//    while (1);
  }
  // Set up oversampling and filter initialization
  bme3.setTemperatureOversampling(BME680_OS_8X);
  bme3.setHumidityOversampling(BME680_OS_2X);
  bme3.setPressureOversampling(BME680_OS_4X);
  bme3.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme3.setGasHeater(320, 150); // 320*C for 150 ms
}

//Read DATA
void printValues(Adafruit_BME680 bme, int bus) {
  TCA9548A (bus);
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    ESP.restart();
    return;
  }
  delay(50);
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading for bme:("));
    ESP.restart();
    return;
  }
  if(bme.readTemperature() <= 0.00){
    Serial.println(F("Failed temp <= 0"));
    ESP.restart();
  }
  avgTemp += bme.readTemperature();
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  avgHumi += bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void openFan1(){
  if(modeState = 1){
    if (Firebase.setInt(firebaseData, "State/devices/FanF", 1)) {
      Serial.println("OK1");
      digitalWrite(fan1, HIGH);
    }
  }
  else{
     Serial.println("manualOK1");
     digitalWrite(fan1, HIGH);
  }
}

void closeFan1(){
  if(modeState = 1){
    if (Firebase.setInt(firebaseData, "State/devices/FanF", 0)) {
      Serial.println("close1");
      digitalWrite(fan1, LOW);
    }
  }
  else{
     Serial.println("manualClose1");
     digitalWrite(fan1, LOW);
  }
}

void openFan2(){
  if(modeState = 1){
    if (Firebase.setInt(firebaseData, "State/devices/FanB", 1)) {
      Serial.println("OK2");
      digitalWrite(fan2, HIGH);
    }
  }
  else{
     Serial.println("manualOK2");
     digitalWrite(fan2, HIGH);
  }
}

void closeFan2(){
  if(modeState = 1){
    if (Firebase.setInt(firebaseData, "State/devices/FanB", 0)) {
      Serial.println("close2");
      digitalWrite(fan2, LOW);
    }
  }
  else{
     Serial.println("manualClose2");
     digitalWrite(fan2, LOW);
  }
}

void openHeat(){
  if(modeState = 1){
    if (Firebase.setInt(firebaseData, "State/devices/Heater", 1)) {
      Serial.println("OK3");
      if(heaterState == 1){
        digitalWrite(heater, HIGH);
        delay(100);
        digitalWrite(heater, LOW);
        heatOn();
        heaterState = !heaterState;
      }
    }
  }
  else{
     Serial.println("manualOK3");
     if(heaterState == 1){
       digitalWrite(heater, HIGH);
       delay(100);
       digitalWrite(heater, LOW);
       heatOn();
       heaterState = !heaterState;
     }
  }
}

void closeHeat(){
 if(modeState = 1){
    if (Firebase.setInt(firebaseData, "State/devices/Heater", 0)) {
      Serial.println("close3");
      if(heaterState == 0){
       digitalWrite(heater, HIGH);
       delay(100);
       digitalWrite(heater, LOW);
       heaterState = !heaterState;
     }
    }
  }
  else{
     Serial.println("manualClose3");
     if(heaterState == 0){
       digitalWrite(heater, HIGH);
       delay(100);
       digitalWrite(heater, LOW);
       heaterState = !heaterState;
     }
  }
}

void heatOn(){
  Serial.println("HeatOnComplete");
  digitalWrite(adjustHeat, HIGH);
  delay(100);
  digitalWrite(adjustHeat, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(fan1, OUTPUT);
  pinMode(fan2, OUTPUT);
  pinMode(heater, OUTPUT);
  pinMode(adjustHeat, OUTPUT);
  
  digitalWrite(fan1, LOW);
  digitalWrite(fan2, LOW);
  digitalWrite(heater, LOW);
  digitalWrite(adjustHeat, LOW);
  WiFiConnection();
  
  Firebase.setInt(firebaseData, "State/devices/FanF", 0);
  Firebase.setInt(firebaseData, "State/devices/FanB", 0);
  Firebase.setInt(firebaseData, "State/devices/Heater", 0);
  // Start I2C communication with the Multiplexer
  Wire.begin();

  // Init sensor on bus number 2
  bmeSetup();
  Serial.println();
 // getBmeData();
}

void loop() { 
  //Print values for sensor 1
  avgTemp = 0;
  avgHumi = 0;
  //Time every 5 minute
  if((millis() - lastMillis) >= 1*60*1000UL){
    lastMillis = millis();
    //getBmeData();
//    printValues(bme1, 2);
//    printValues(bme2, 3);
    printValues(bme3, 7);
  
    calTemp = (int((avgTemp / 2) * 100));
    inTemp = float(calTemp) / 100;
  
    calHumi = (int((avgHumi / 2) * 100));
    inHumi = float(calHumi) / 100;

    if (Firebase.setFloat(firebaseData, "Sensor1/inside/temp", inTemp)) {
      Serial.print("Average Temp:");
      Serial.println(inTemp);
    }
  
    if (Firebase.setFloat(firebaseData, "Sensor1/inside/humi", inHumi)) {
      Serial.print("Average Humi:");
      Serial.println(inHumi);
  
    }
  }
  //check Mode
  if(Firebase.getInt(firebaseData, "/Mode")){
    if(firebaseData.dataType() == "int"){
      if(firebaseData.intData() == 0){ //Manual Mode
        modeState = 0;
        Serial.println("ManualMode");
        if(checkFlag == 1){
          Firebase.setInt(firebaseData, "State/devices/FanF", 0);
          Firebase.setInt(firebaseData, "State/devices/FanB", 0);
          Firebase.setInt(firebaseData, "State/devices/Heater", 0);
          checkFlag = 0;
        }
        if(Firebase.getInt(firebaseData, "State/devices/FanF")){
          if(firebaseData.intData() == 1){
            openFan1();
          }
          else{
            closeFan1();
          }
        }
        if(Firebase.getInt(firebaseData, "State/devices/FanB")){
          if(firebaseData.intData() == 1){
            openFan2();
          }
          else{
            closeFan2();
          }
        }
//        if(Firebase.getInt(firebaseData, "State/devices/Heater")){
//          if(firebaseData.intData() == 1){
//            openHeat();
//          }
//          else{
//            closeHeat();
//          }
//        }
    }
    else{ //Auto Mode
      modeState = 1;
      Serial.println("AutoMode");
//      Serial.println(inTemp);
      checkFlag = 1;
      if(inTemp <= 40){
        closeFan1();
        openFan2();
//        openHeat();
      }
      else if(inTemp >= 57 && inTemp < 60){
        closeFan1();
        openFan2();
//        closeHeat();
      }
      else if(inTemp >= 65){
        openFan2();
        openFan1();
//        closeHeat();
      }
      else{
        Serial.println("Temperatur not mush and not less");
      }
    }
   }
  }
  //  delay(300000);
}
