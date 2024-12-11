#include <SPI.h>
#include <LoRa.h>

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

int counter = 0;
bool led_state = false;

float readTemp();
void toggleLED();

void setup() {
  Serial.begin(115200);

  Serial.println("LoRa Sender");

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
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // Start LoRa packet
  LoRa.beginPacket();
  LoRa.print(readTemp());
  LoRa.print("*C, counter: ");
  LoRa.print(counter);
  LoRa.endPacket();
  toggleLED();
  delay(300);
  toggleLED();

  counter++;
  delay(5000); // Send every 5 seconds
}

float readTemp() {
  return analogReadTemp();
}

void toggleLED() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);
}
