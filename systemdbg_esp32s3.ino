#include <Arduino.h>
#include <cstdint>
#include "./src/systemReg.class.h"

ES3SystemRegister sysReg;

void setup() {
  Serial.begin(115200);
  sysReg.getAll();
  sysReg.perip_clk_en1.system_dma_clk_en = true;
  sysReg.setSystem_perip_clk_en1();
  sysReg.getAll();
  Serial.printf("System Registers Test\n");
  delay(2000);
}

void loop() {
  Serial.printf("GDMA Clock : %s\n", sysReg.perip_clk_en1.system_dma_clk_en == true ? "true" : "false");
  delay(2000);
}