#include <SPI.h>
#include <LoRa.h>

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

bool led_state = 0;

void toggleLED();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

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

void loop() {
// Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");

    // Read packet data
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // Print RSSI (signal strength)
    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());

    toggleLED();
    delay(300);
    toggleLED();
  }
}

void toggleLED() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);
}
