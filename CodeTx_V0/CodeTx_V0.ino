#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   8
#define CSN_PIN 10

const byte address[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);

int LeftValue = 0;  // A0 = eje X //LOS EJES XY CON LIMITES DE [0,1023] 
int RightValue = 0; // A1 = eje Y

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 100;

const int centro = 512;     // Valor central del joystick
const int umbral = 50;      // Margen muerto para ignorar pequeñas variaciones

void setup() {
  Serial.begin(115200);
  Serial.println("SimpleTx interpretado");

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(3, 5);
  radio.openWritingPipe(address);
}

void loop() {
  LeftValue = analogRead(A0);   // Eje X
  RightValue = analogRead(A1);  // Eje Y

  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    enviarJoystick();
    interpretarJoystick(); // ← Aquí imprimes “Eje X +100” o lo que quieras
    prevMillis = currentMillis;
  }
}

void enviarJoystick() {
  int paquete[2] = {LeftValue, RightValue};
  bool rslt = radio.write(paquete, sizeof(paquete));

  Serial.print("Enviando crudo: ");
  Serial.print(paquete[0]);
  Serial.print(", ");
  Serial.print(paquete[1]);
  Serial.println();

  if (rslt) {
    Serial.println("  Envío exitoso (ACK recibido)");
  } else {
    Serial.println("  Error en envío (sin ACK)");
  }
}

void interpretarJoystick() {
  int ejeX = LeftValue - centro;
  int ejeY = centro - RightValue;


  if (abs(ejeX) > umbral) {
    Serial.print("Eje X ");
    Serial.println(ejeX > 0 ? "+" + String(ejeX) : String(ejeX));
  }

  if (abs(ejeY) > umbral) {
    Serial.print("Eje Y ");
    Serial.println(ejeY > 0 ? "+" + String(ejeY) : String(ejeY));
  }
}
