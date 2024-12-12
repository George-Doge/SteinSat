#include <SPI.h>
#include <LoRa.h>
#include <Arduino_JSON.h>

// LoRa MODUE CODE FOR GROUND STATION

#define BME280_RUNNING false

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

// Function declarations
void toggleLED();
void loraSetup();
void printPayload(JSONVar parsedJson);

// Variables
bool led_state = 0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial);

  Serial.println("SteinSat Ground Station");
  loraSetup(); 
}

void loop() {
  String receivedPayload = "";
  // Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");

    // Read packet data
    while (LoRa.available()) {
      receivedPayload += (char)LoRa.read();
    }

    JSONVar parsedPayload = JSON.parse(receivedPayload);

    // Check if parsing was successful
    if (JSON.typeof(parsedPayload) == "undefined") {
      Serial.println("Parsing JSON failed!");
    }

    printPayload(parsedPayload);
    
    // Signal received packet
    toggleLED();
    delay(300);
    toggleLED();
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

void printPayload(JSONVar parsedJson) {
  if(BME280_RUNNING) {
      String temperature = parsedJson["temperature"];
      String pressure = parsedJson["pressure"];
      String altitude = parsedJson["altitude"];
      String humidity = parsedJson["humidity"];
      
      Serial.println("The outside temperature is " + temperature + "°C.");
      Serial.println("The pressure is " + pressure + "hPa.");
      Serial.println("The estimated altitude is " + altitude + "m.");
      Serial.println("The humidity is " + humidity + "%.");
    }
    
    String onboardTemperature = parsedJson["onboard_temperature"];
    String counter = parsedJson["counter"];
    
    Serial.println("The onboard temperature is " + onboardTemperature + "°C.");
    Serial.println("Packet count: "+ counter);
}

void toggleLED() {
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state);
}
