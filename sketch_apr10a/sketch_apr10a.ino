#include <ACS712.h>  // Library for current sensors
#include <Adafruit_SHT31.h>  // Library for Temp and Humidity Sensor(SHT30 is compatible)
#include <Arduino.h>
#include <DS3231_Simple.h>    // Library for Real Time Clock
#include <SPI.h>              // Library for communication (may be unused)
#include <SparkFunLSM9DS1.h>  // Library for accelerometer and compass
#include <Wire.h>    

Adafruit_SHT31 sht30 = Adafruit_SHT31();

LSM9DS1 Comp;
LSM9DS1 Accel;

#define LSM9DS1_C_C 0x1E  // Comp's compass
#define LSM9DS1_A_C 0x6B  // Comp's accelerometer
#define LSM9DS1_C_A 0x1C  // Accel's compass
#define LSM9DS1_A_A 0x6A  // Accel's accelerometer

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
    Wire.setClock(10000);
    Serial.println("startup");


    if (!sht30.begin(0x44)) {
        Serial.println("Couldn't find SHT30");
    }
    // Begin Communication with LSM9DS1 accelerometer
    if (!Comp.begin(LSM9DS1_A_C, LSM9DS1_C_C)) {
        Serial.println("Couldn't Find LSM9DS1 COMP");
    }
    if (!Accel.begin(LSM9DS1_A_A, LSM9DS1_C_A)) {
        Serial.println("Couldn't Find LSM9DS1 ACCEL");
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);

  Comp.readMag();
      Accel.readAccel();
      Serial.print("Compass\t X: ");
      Serial.print(Comp.mx);
      Serial.print(" Y: ");
      Serial.print(Comp.my);
      Serial.println();
    Serial.print("Accel\t X: ");
    Serial.print(Accel.ax);
    Serial.print(" Y: ");
    Serial.print(Accel.ay);
    Serial.print(" Z: ");
    Serial.print(Accel.az);
    Serial.println();

}
