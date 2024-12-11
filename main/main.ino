#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
// MAIN CODE FOR SATELLITE

#define SEALEVELPRESSURE_HPA (1013.25)


Adafruit_BME280 bme; // I2C (default pins for Raspberry Pi Pico: GPIO 4 (SDA), GPIO 5(SCL)


// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

float readInternalTemp();
void toggleLED();
void sendPacket();
void printValues();
void loraSetup();
void bmeSetup();

// variables
int counter = 0;
bool led_state = false;
unsigned long delayTime;
float internalTemp;

void setup() {
  Serial.begin(115200);
  
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Raspberry Pi Pico powered Satellite SteinSat");

  // TODO UNCOMMENT WHEN WORKING BME MODULE IS INSTALLED
  // bmeSetup(); 

  delayTime = 1000;

  Serial.println();

  loraSetup();
}


void loop() { 
  sendPacket();
  delay(delayTime);
}

// for now there will be only hard-coded data which will be sent, later I'll add more modularity 
void sendPacket() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // Start LoRa packet
  LoRa.beginPacket();
  LoRa.print(readInternalTemp());
  LoRa.print("°C, counter: ");
  LoRa.print(counter);
  LoRa.endPacket();
  toggleLED();
  delay(300);
  toggleLED();

  counter++;
  delay(1000); // Send every second
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");
  
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

float readInternalTemp() {
  return analogReadTemp();
}

void toggleLED() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);
}

void bmeSetup() {
  Serial.println("Running BME280 initialization!");
  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loraSetup() {
  Serial.println("Running LoRa initialization!");
  // Initialize LoRa module
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  // Set frequency (433 MHz for Asia/Europe or 915 MHz for North America)
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed. Check your connections.");
    while (true);
  }

  // Set sync word to ensure communication is only between these devices
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa initialization successful!");
}
