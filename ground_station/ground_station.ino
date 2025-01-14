#include <SPI.h>
#include <LoRa.h>
#include <Arduino_JSON.h>

#define BME280_RUNNING false

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

// Variables
unsigned long ledStartTime = 0;
const unsigned int ledTimeout = 300;
bool led_state = false;
unsigned char sensorsNum = 1;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial);

  Serial.println("SteinSat Ground Station");
  
  sensorsNum = BME280_RUNNING ? 5 : 1;
  loraSetup();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    unsigned char payloadSize = sizeof(float) * sensorsNum + sizeof(unsigned long) + 1;
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

    // Reconstruct the float array
    float receivedPayload[(payloadSize - 1) / sizeof(float)];
    for (int i = 0; i < sensorsNum; i++) {
      memcpy(&receivedPayload[i], &message[i * sizeof(float)], sizeof(float));
    }

    unsigned long packetCount;
    memcpy(&packetCount, &message[sensorsNum * sizeof(float)], sizeof(unsigned long));

    // Construct JSON output
    JSONVar output;
    unsigned char queue = 0;

    output["isValid"] = isPacketValid;
    output["onboardTemperature"] = receivedPayload[queue++];
    if (BME280_RUNNING) {
      output["temperature"] = receivedPayload[queue++];
      output["pressure"] = receivedPayload[queue++];
      output["altitude"] = receivedPayload[queue++];
      output["humidity"] = receivedPayload[queue++];
    }
    output["packetCount"] = packetCount;

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
  LoRa.setSyncWord(0xF3); // Communication sync word
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
