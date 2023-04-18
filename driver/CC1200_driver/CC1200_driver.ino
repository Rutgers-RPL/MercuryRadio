#include <SPI.h>
//#include <CC1200.h>
//#include <CC1101.h>

#define SS 10
#define MOSI 11
#define MISO 12
#define SCK 13
#define RESET 9



void setup() {
  Serial.begin(115200);

  SPI.begin();

  // MSB, Positive Clock Edge
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  byte command = 0b00000000;
  byte data = 0xFF;
  SPI.transfer(command);
  byte receivedVal = SPI.transfer(data);
  Serial.println("Received Byte: ");
  Serial.println(receivedVal);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  command = 0b10000000;
  receivedVal = SPI.transfer(command);
  Serial.println("Received Byte: ");
  Serial.println(receivedVal);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
  delay(100);
}

void loop() {
  delay(100);
}
