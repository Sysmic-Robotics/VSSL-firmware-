/*
 *  DEBUG: receptor nRF24 que solo imprime VRX y VRY
 *  Dirección   : "00001"
 *  Velocidad RF: 250 kbit/s
 *  Payload     : 4 bytes  (int16_t vrx, int16_t vry)
 *  Pines módulo: CE 8, CSN 10, SPI hardware (11-12-13)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN  8
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);

const byte ADDRESS[6] = "00001";      // 5 bytes ASCII + '\0'

struct Payload {
  int16_t vrx;
  int16_t vry;
};

Payload data;

void setup() {
  Serial.begin(115200);
  while (!Serial) ;                   // placas con USB nativo

  radio.begin();
  radio.enableDynamicPayloads();      // igual que en el TX
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);     // o LOW según alcance
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();

  Serial.println(F(">> Receptor listo, esperando datos…"));
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));  // 4 bytes
    Serial.print(F("VRX = "));
    Serial.print(data.vrx);
    Serial.print(F("   |   VRY = "));
    Serial.println(data.vry);
  }
}
