#include <EEPROM.h>

#define EEPROM_SIZE 1
#define DELETE false // !!!change this to prevent accidental EEPROM data deletion!!!

int picture_counter = -1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  picture_counter = EEPROM.read(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Current picture counter: ");
  Serial.println(picture_counter);

  if (DELETE && picture_counter != 0) {
    EEPROM.write(0, 0);
    EEPROM.commit();
    picture_counter = 0;
    Serial.println("PICTURE COUNTER RESET!!!!");
  }
  delay(1000);
}
