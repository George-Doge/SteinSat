#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
#include <Arduino_JSON.h>

// MAIN CODE FOR SATELLITE WITH ESP8266 (ESP12E Motor Shield Compatible)

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_RUNNING false

// Pin definitions for LoRa module (ESP8266)
#define SS_PIN 15   // Chip Select (GPIO 15 / D8)
#define RST_PIN 0  // Reset (GPIO 0 / D3)
#define DIO0_PIN 4  // IRQ (GPIO 4 / D2)

// BME280 I2C Pins for ESP8266
#define SDA_PIN 5 // D1
#define SCL_PIN 2 // D4

Adafruit_BME280 bme; // I2C connection

// Function declarations
JSONVar getSensorData();
void toggleLED();
void sendPacket();
void loraSetup();
void bmeSetup();

// Global variables
const unsigned int ledTimeout = 300;
const unsigned int delayTime = 1000 - ledTimeout;
unsigned int counter = 0;
bool led_state = false;
float internalTemp;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("ESP8266 powered Satellite SteinSat\n");

  if (BME280_RUNNING) {
    Wire.begin(SDA_PIN, SCL_PIN);
    bmeSetup();
  }

  loraSetup();
}

void loop() {
  sendPacket();
  delay(delayTime);
}

void sendPacket() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // Start LoRa packet
  LoRa.beginPacket();

  // Get sensor data & add packet count to payload
  JSONVar jsonData = getSensorData();
  jsonData["counter"] = counter;

  // Convert JSON object to string
  String payload = JSON.stringify(jsonData);

  LoRa.print(payload);

  LoRa.endPacket();

  // Signal sent packet
  toggleLED();
  delay(ledTimeout);
  toggleLED();

  // Count packets sent
  counter++;
}

JSONVar getSensorData() {
  JSONVar jsonData;

  if (BME280_RUNNING) {
    jsonData["temperature"] = bme.readTemperature();
    jsonData["pressure"] = bme.readPressure() / 100.0F;
    jsonData["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    jsonData["humidity"] = bme.readHumidity();
  }

  // Replace with ESP8266 internal temperature sensor reading if available
  jsonData["onboard_temperature"] = readInternalTemp();

  return jsonData;
}

void toggleLED() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);
}

float readInternalTemp() {
  // Placeholder for ESP8266 internal temp (if required)
  return 25.0; // Replace with actual onboard temperature if available
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

void bmeSetup() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (true);
  }
  Serial.println("BME280 sensor initialization successful!");
}
