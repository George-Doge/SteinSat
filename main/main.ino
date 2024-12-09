#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>



#define SEALEVELPRESSURE_HPA (1013.25)


Adafruit_BME280 bme; // I2C (default pins for Raspberry Pi Pico: GPIO 4 (SDA), GPIO 5(SCL)

// variables
unsigned long delayTime;
float internalTemp;

void setup() {
  Serial.begin(9600);
  Serial.println("IJFAOIAJFPOIJAFS");
  
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("BME280 with Raspberry Pi Pico");

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  delayTime = 1000;

  Serial.println();
}

void loop() { 
  printValues();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delayTime);
  digitalWrite(LED_BUILTIN, LOW);
  delay(delayTime);
  internalTemperature();
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void internalTemperature() {
  internalTemp = analogReadTemp();
  Serial.print("Internal temperature Celsius (ºC): ");
  Serial.println(tempC);
}