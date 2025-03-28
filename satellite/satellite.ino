#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <LoRa.h>

// MAIN CODE FOR SATELLITE WITH RPi Pico 2

#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP280_RUNNING true
#define MPU6050_RUNNING true
#define NEO6M_RUNNING false

#define LORA_SYNCWORD 0xF3

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

// BMP280 I2C Pins for RPi Pico 2
#define SDA_PIN 20 // GPIO 20
#define SCL_PIN 21 // GPIO 21


Adafruit_BMP280 bmp; // bmp object
Adafruit_MPU6050 mpu; // mpu object

// Global variables
unsigned long ledStartTime = 0;
const unsigned int ledTimeout = 300;
bool led_state = false;
unsigned long counter = 1;
unsigned char dataValuesNum = 1;
unsigned long lastPacketTime = 0; // Tracks the last time a packet was sent
const unsigned long interval = 500; // Interval in milliseconds (500ms = 0.5 seconds)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Raspberry Pi Pico 2 powered Satellite SteinSat\n");

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  if (BMP280_RUNNING) {
    bmpSetup();
    dataValuesNum += 3;
  }

  if (MPU6050_RUNNING) {
    mpuSetup();
    dataValuesNum += 4;
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
  unsigned char packetSize = sizeof(float) * dataValuesNum + sizeof(unsigned long) + 1;
  uint8_t payload[packetSize];
  unsigned char arrayQueue = 0;
  uint8_t checksum = 0;

  // Cast float data into raw binary data
  float sensorData[dataValuesNum];
  getSensorData(sensorData);

  for (int i = 0; i < dataValuesNum; i++) {
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
  LoRa.setSyncWord(LORA_SYNCWORD);
  Serial.println("LoRa initialization successful!");
}

void bmpSetup() {
  Serial.println("Initializing BMP280 sensor...");
  if (!bmp.begin(0x76)) {
    Serial.println("Error: Could not find a valid BMP280 sensor. Check wiring and I2C address!");
    // Optionally halt the system if the BMP280 is critical
    while (true);
  }
  Serial.println("BMP280 sensor initialization successful!");
}

void mpuSetup() {
  Serial.println("Initializing MPU6050 sensor...");
  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050 sensor...");
  }
  Serial.println("MPU6050 Sensor Initialized");
  // Set sensor range
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void getSensorData(float* data) {
  unsigned char queue = 0;

  data[queue++] = analogReadTemp();

  if (BMP280_RUNNING) {
    data[queue++] = bmp.readTemperature();
    data[queue++] = bmp.readPressure() / 100.0F;
    data[queue++] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

  if (MPU6050_RUNNING) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float acceleration_vector_components[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    float acceleration_vector_magnitude = calculate_magnitude(acceleration_vector_components);
    data[queue++] = acceleration_vector_magnitude;

    data[queue++] = g.gyro.x;
    data[queue++] = g.gyro.y;
    data[queue++] = g.gyro.z;
  }
}

float calculate_magnitude(float* acceleration_vector_components) {
  // This calculates the magnitude of the acceleration vector, presuming that z is facing down
  return sqrt(pow(acceleration_vector_components[0], 2) + pow(acceleration_vector_components[1], 2) + pow(acceleration_vector_components[2]-9.81, 2));
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
