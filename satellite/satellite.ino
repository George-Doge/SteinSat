#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>

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

// Global variables
unsigned long ledStartTime = 0;
const unsigned int ledTimeout = 300;
bool led_state = false;
unsigned long counter = 1;
unsigned char sensorsNum = 1;
unsigned long lastPacketTime = 0; // Tracks the last time a packet was sent
const unsigned long interval = 500; // Interval in milliseconds (500ms = 0.5 seconds)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("ESP8266 powered Satellite SteinSat\n");

  if (BME280_RUNNING) {
    Wire.begin();
    bmeSetup();
    sensorsNum += 4;
  }

  loraSetup();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastPacketTime >= interval) {
    lastPacketTime = currentMillis;
    sendPacket();
  }

  handleLED();
}

void sendPacket() {
  unsigned char packetSize = sizeof(float) * sensorsNum + sizeof(unsigned long) + 1;
  uint8_t payload[packetSize];
  unsigned char arrayQueue = 0;
  uint8_t checksum = 0;

  // Cast float data into raw binary data
  float sensorData[sensorsNum];
  getSensorData(sensorData);

  for (int i = 0; i < sensorsNum; i++) {
    uint8_t binData[sizeof(float)];
    toBinary(sensorData[i], binData);

    if (arrayQueue + sizeof(binData) <= packetSize) {
      memcpy(&payload[arrayQueue], binData, sizeof(binData));
      arrayQueue += sizeof(binData);
    } else {
      Serial.println("Error: Payload array overflow.");
      break;
    }
  }

  uint8_t binPacketCounter[sizeof(unsigned long)];
  ulongToBinary(counter, binPacketCounter);

  if (arrayQueue + sizeof(binPacketCounter) <= packetSize) {
    memcpy(&payload[arrayQueue], binPacketCounter, sizeof(binPacketCounter));
    arrayQueue += sizeof(binPacketCounter);
  } else {
    Serial.println("Error: Payload array overflow.");
  }

  // Calculate checksum 
  for (int i = 0; i < packetSize - 1; i++) { 
    checksum ^= payload[i]; 
  }

  Serial.print("Calculated Checksum: ");
  Serial.println(checksum, HEX);

  // Append checksum to payload
  if (arrayQueue <= packetSize) {
    payload[arrayQueue] = checksum;
  } else {
    Serial.println("Error: Payload array overflow.");
  }

  Serial.print("Sending packet: ");
  Serial.println(counter);

  // Start LoRa packet
  LoRa.beginPacket();
  LoRa.write(payload, packetSize); // Send the raw binary data
  LoRa.endPacket();

  // Signal sent packet
  signalLED();

  // Count packets sent
  counter++;
}

void signalLED() {
  if (!led_state) {
    // Turn on the LED
    digitalWrite(LED_BUILTIN, HIGH);
    ledStartTime = millis(); // Record the time when the LED was turned on
    led_state = true;
  }
}

void handleLED() {
  if (led_state && millis() - ledStartTime >= ledTimeout) {
    // Turn off the LED after the timeout
    digitalWrite(LED_BUILTIN, LOW);
    led_state = false;
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

  LoRa.enableCrc();

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

void getSensorData(float* data) {
  unsigned char queue = 0;

  data[queue++] = 24.5;

  if (BME280_RUNNING) {
    data[queue++] = bme.readTemperature();
    data[queue++] = bme.readPressure() / 100.0F;
    data[queue++] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    data[queue++] = bme.readHumidity();
  }
}

union FloatToBinary {
    float dec;             // The float value
    uint8_t bin[sizeof(float)]; // Binary representation
};

// Function to convert float to binary
void toBinary(float dec, uint8_t* bin) {
    FloatToBinary converter;
    converter.dec = dec;  // Assign the float value
    memcpy(bin, converter.bin, sizeof(converter.bin));  // Copy binary data to output
}

union UlongToBinary {
  unsigned long dec;
  uint8_t bin[sizeof(unsigned long)];
};

void ulongToBinary(unsigned long dec, uint8_t* bin) {
  UlongToBinary converter;
  converter.dec = dec;
  memcpy(bin, converter.bin, sizeof(converter.bin));
}
