#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

/*#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

//#define SDA_2 14
//#define SCL_2 27


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme2; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI

//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("BME680 async test"));
  Wire.begin();
//  Wire1.begin(SDA_2, SCL_2);

  bool status1 = bme.begin(0x77);  
  if (!status1) {
    Serial.println("Could not find a valid BME680_1 sensor, check wiring!");
    while (1);
  }
  
//  bool status2 = bme2.begin(0x77, &Wire1);  
//  if (!status2) {
//    Serial.println("Could not find a valid BME680_2 sensor, check wiring!");
//    while (1);
//  }


  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

//  bme2.setTemperatureOversampling(BME680_OS_8X);
//  bme2.setHumidityOversampling(BME680_OS_2X);
//  bme2.setPressureOversampling(BME680_OS_4X);
//  bme2.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme2.setGasHeater(320, 150); // 320*C for 150 ms

}

void loop() {
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
//  unsigned long endTime2 = bme2.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
//  if (endTime2 == 0) {
//    Serial.println(F("Failed to begin reading :("));
//    return;
//  }

  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading for bme1:("));
    return;
  }

//  if (!bme2.endReading()) {
//    Serial.println(F("Failed to complete reading for bme2:("));
//    return;
//  }
  Serial.println("--------------------");
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.println("BME1 is sending data");
  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));


//  Serial.println("--------------------");
//  Serial.println("BME2 is sending data");
//  Serial.print(F("Temperature = "));
//  Serial.print(bme2.temperature);
//  Serial.println(F(" *C"));
//
//  Serial.print(F("Pressure = "));
//  Serial.print(bme2.pressure / 100.0);
//  Serial.println(F(" hPa"));
//
//  Serial.print(F("Humidity = "));
//  Serial.print(bme2.humidity);
//  Serial.println(F(" %"));

  Serial.println();
  delay(2000);
}
