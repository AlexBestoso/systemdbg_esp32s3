#include <Arduino.h>
#include <cstdint>
#include "./src/systemReg.class.h"

void setup() {
  Serial.begin(115200);
  Serial.printf("System Registers Test\n");
  delay(2000);
}

void loop() {
  delay(2000);
}