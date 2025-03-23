#include <SPI.h>
#include <LoRa.h>
#include <Arduino_JSON.h>

#define BMP280_RUNNING true
#define MPU6050_RUNNING true
#define NEO8M_RUNNING false

#define LORA_SYNCWORD 0x185

// Pin definitions for LoRa module (ESP8266)
#define SS_PIN 15   // Chip Select (GPIO 15 / D8)
#define RST_PIN 0  // Reset (GPIO 0 / D3)
#define DIO0_PIN 4  // IRQ (GPIO 4 / D2)

// Variables
unsigned long ledStartTime = 0;
const unsigned int ledTimeout = 300;
bool led_state = false;
unsigned char dataValuesNum = 1;
unsigned int lastPacketCount = 0;
unsigned int packetsFailed = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  Serial.println("SteinSat Ground Station");
  
  dataValuesNum = 1;
  if (BMP280_RUNNING)
    dataValuesNum += 3;

  if (MPU6050_RUNNING)
    dataValuesNum += 4;

  if (NEO8M_RUNNING)
    dataValuesNum += 5;

  loraSetup();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    unsigned char payloadSize = sizeof(double) * dataValuesNum + sizeof(unsigned long) + 1;
    if (packetSize != payloadSize) {
      Serial.println("Error: Packet size mismatch.");
      return;
    }


    uint8_t message[payloadSize];
    
    // Read the received bytes
    for (int i = 0; i < payloadSize; i++) {
      message[i] = LoRa.read();
    }

    // Extract checksum
    uint8_t receivedChecksum = message[payloadSize - 1];

    // Calculate checksum
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < payloadSize - 1; i++) {
      calculatedChecksum ^= message[i];
    }

    bool isPacketValid = (receivedChecksum == calculatedChecksum);

    // Reconstruct the double array
    double receivedPayload[(payloadSize - 1) / sizeof(double)];
    for (int i = 0; i < dataValuesNum; i++) {
      memcpy(&receivedPayload[i], &message[i * sizeof(double)], sizeof(double));
    }

    unsigned long packetCount;
    memcpy(&packetCount, &message[dataValuesNum * sizeof(double)], sizeof(unsigned long));

    // Construct JSON output
    JSONVar output;
    unsigned char queue = 0;

    output["isValid"] = isPacketValid;
    output["onboardTemperature"] = receivedPayload[queue++];
    if (BMP280_RUNNING) {
      output["temperature"] = receivedPayload[queue++];
      output["pressure"] = receivedPayload[queue++];
      output["altitude"] = receivedPayload[queue++];
    }

    if (MPU6050_RUNNING) {
      output["acceleration_vector_magnitude"] = receivedPayload[queue++];

      output["gyro_x"] = receivedPayload[queue++];
      output["gyro_y"] = receivedPayload[queue++];
      output["gyro_z"] = receivedPayload[queue++];
    }

    if (NEO8M_RUNNING) {
      output["lat"] = receivedPayload[queue++];
      output["lng"] = receivedPayload[queue++];
      output["speed_over_ground"] = receivedPayload[queue++];
      output["alt_gps"] = receivedPayload[queue++];
      output["course_over_ground"] = receivedPayload[queue++];o
    }
    // calculate how many packets have been lost
    if (lastPacketCount != packetCount - 1) {
        packetsFailed += packetCount - lastPacketCount - 1;
    }

    output["packetCount"] = packetCount;
    lastPacketCount = packetCount;
    output["packetsFailed"] = packetsFailed;
    String printOutput = JSON.stringify(output);
    Serial.println(printOutput);

    // Signal received packet
    signalLED();
  }

  handleLED();
}

void loraSetup() {
  Serial.println("Running LoRa initialization!");
  
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  // Set frequency (433 MHz for Asia/Europe or 915 MHz for North America)
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed. Check your connections.");
    while (true);
  }

  LoRa.enableCrc();
  LoRa.setSyncWord(LORA_SYNCWORD); // Communication sync word
  Serial.println("LoRa initialization successful!");
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
