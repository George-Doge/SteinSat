#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
#include <Arduino_JSON.h>

// MAIN CODE FOR SATELLITE

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_RUNNING false

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

Adafruit_BME280 bme; // I2C (default pins for Raspberry Pi Pico: GPIO 4 (SDA), GPIO 5(SCL)


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
  Serial.println("Raspberry Pi Pico powered Satellite SteinSat\n");
  
  if(BME280_RUNNING) {
    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire.begin();
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
  delay(300);
  toggleLED();

  counter++;
}

JSONVar getSensorData() {
  // Read sensor data
  float onboardTemperature = analogReadTemp();
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();

  // Create JSON object
  JSONVar jsonData;

  jsonData["onboard_temperature"] = onboardTemperature;
  
  if(BME280_RUNNING) {
    jsonData["temperature"] = temperature;
    jsonData["pressure"] = pressure;
    jsonData["altitude"] = altitude;
    jsonData["humidity"] = humidity;
  }

  return jsonData;
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
    while (true);
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
