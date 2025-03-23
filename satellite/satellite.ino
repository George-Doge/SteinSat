#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <LoRa.h>

// MAIN CODE FOR SATELLITE WITH RPi Pico 2

#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP280_RUNNING true
#define MPU6050_RUNNING true
#define NEO8M_RUNNING false

#define LORA_SYNCWORD 0x185

// Pin definitions for LoRa module
#define SS_PIN 17   // Chip Select (GPIO 17)
#define RST_PIN 27  // Reset (GPIO 27)
#define DIO0_PIN 28 // IRQ (GPIO 28)

// I2C Pins for RPi Pico 2
#define SDA_PIN 4 // GPIO 4
#define SCL_PIN 5 // GPIO 5

// UART Pins for GPS
#define RX_PIN 21
#define TX_PIN 20
#define GPSBaud 9600

// Pin definitions for LEDs and buzzer
#define buzzer 26
#define LEDS 22

Adafruit_BMP280 bmp; // bmp object
Adafruit_MPU6050 mpu; // mpu object
TinyGPSPlus gps; // gps object
SoftwareSerial ss(RX_PIN, TX_PIN); // Serial Connection object


// Global variables
unsigned long ledStartTime = 0;
const unsigned short ledTimeout = 300;
bool led_state = false;
unsigned long counter = 1; // Packet counter
unsigned char dataValuesNum = 1; // Tracks number of sent values
unsigned long lastPacketTime = 0; // Tracks the last time a packet was sent
const unsigned short interval = 500; // Interval in milliseconds (500ms = 0.5 seconds)
unsigned char buzzerLedIntensity = 0; // buzzerLed intensity var
const unsigned char buzzerLedInterval = 200; // buzzerLed interval

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(LEDS, OUTPUT);

  Serial.begin(115200);
  Serial.println("Raspberry Pi Pico 2 powered Satellite by team SteinSat\n");

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

  if (NEO8M_RUNNING) {
    neoSetup();
    dataValuesNum += 5;
  }

  loraSetup();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastPacketTime >= interval) {
    lastPacketTime = currentMillis;
    sendPacket();
  }

  handleLEDBuzzer(currentMillis);
  handleLED();
}

// gathers data and sends a packet
void sendPacket() {
  unsigned char packetSize = sizeof(double) * dataValuesNum + sizeof(unsigned long) + 1;
  uint8_t payload[packetSize];
  unsigned char arrayQueue = 0;
  uint8_t checksum = 0;

  // Cast double data into raw binary data
  double sensorData[dataValuesNum];
  getSensorData(sensorData);

  for (int i = 0; i < dataValuesNum; i++) {
    uint8_t binData[sizeof(double)];
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

// function to attract attention when Cansat is on the ground after the launch
void handleLEDBuzzer(int loopStartMillis) {
  unsigned long handleCurrentMillis = millis(); // Not to be confused with current millis in the main loop 

  if (loopStartMillis - handleCurrentMillis >= 200){
    analogWrite(buzzer, buzzerLedIntensity);
    analogWrite(LEDS, buzzerLedIntensity);

    if (buzzerLedIntensity < 255)
      buzzerLedIntensity++;

    else
      buzzerLedIntensity = 0;
  }

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

//starts lora
void loraSetup() {
  Serial.println("Running LoRa initialization!");

  // Initialize LoRa module
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  // Set frequency (433 MHz for Europe)
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed. Check your connections.");
    while (true);
  }

  LoRa.enableCrc();

  // Set sync word to ensure communication is only between these devices
  LoRa.setSyncWord(LORA_SYNCWORD);
  Serial.println("LoRa initialization successful!");
}

// starts bmp
void bmpSetup() {
  Serial.println("Initializing BMP280 sensor...");
  if (!bmp.begin(0x76)) {
    Serial.println("Error: Could not find a valid BMP280 sensor. Check wiring and I2C address!");
    // Optionally halt the system if the BMP280 is critical
    // while (true);
  }
  Serial.println("BMP280 sensor initialization successful!");
}

// starts mpu
void mpuSetup() {
  Serial.println("Initializing MPU6050 sensor...");
  if (!mpu.begin())
    Serial.println("Could not find MPU6050 sensor...");

  Serial.println("MPU6050 Sensor Initialized");
  // Set sensor range
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// starts gps
void neoSetup() {
  Serial.println("Initializing NEO8M GPS module");
  ss.begin(GPSBaud);
  Serial.println("GPS module started!");
}

void getSensorData(double* data) {
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

  if (NEO8M_RUNNING) {
    while (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        data[queue++] = gps.location.lat();
        data[queue++] = gps.location.lng();
        data[queue++] = gps.speed.knots();
        data[queue++] = gps.altitude.meters();
        data[queue++] = gps.course.deg();
      }
    }
  }
}

//  calculates the magnitude of the acceleration vector, assuming that 'z' is facing down
float calculate_magnitude(float* acceleration_vector_components) {
  return sqrt(pow(acceleration_vector_components[0], 2) + pow(acceleration_vector_components[1], 2) + pow(acceleration_vector_components[2]-9.81, 2));
}

union DoubleToBinary {
    double dec;             // The double value
    uint8_t bin[sizeof(double)]; // Binary representation
};

// Function to convert float to binary
void toBinary(double dec, uint8_t* bin) {
    DoubleToBinary converter;
    converter.dec = dec;  // Assign the double value
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
