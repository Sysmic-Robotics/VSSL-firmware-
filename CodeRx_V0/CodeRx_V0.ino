#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   8
#define CSN_PIN 10

#define MOT_A1_PIN 3
#define MOT_A2_PIN 5
#define MOT_B1_PIN 6
#define MOT_B2_PIN 9

#define LED_PIN 7      // LED que parpadea si no hay datos
#define DEADZONE 30    // Zona muerta en el eje X

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
int dataReceived[2];
bool newData = false;

unsigned long lastReceiveTime = 0;
unsigned long timeoutDuration = 500;  // milisegundos sin señal para parpadear LED

void setup() {
  Serial.begin(115200);
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, address);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&dataReceived, sizeof(dataReceived));
    lastReceiveTime = millis();
    newData = true;
  }

  if (newData) {
    int vrx = dataReceived[0];
    int vry = dataReceived[1];
    controlarMotores(vrx, vry);

    Serial.print("VRX: ");
    Serial.print(vrx);
    Serial.print(" | VRY: ");
    Serial.println(vry);

    newData = false;
  }

  // LED parpadea si no hay datos
  if (millis() - lastReceiveTime > timeoutDuration) {
    digitalWrite(LED_PIN, (millis() / 300) % 2);  // parpadeo
    stopAll();
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void controlarMotores(int vx, int vy) {
  int velY = map(vy, 0, 1023, -255, 255);
  int velX = map(vx, 0, 1023, -255, 255);

  // Comportamiento de avance/retroceso
  if (vy > 520) {  // Avanza
    set_motor_pwm(velY, MOT_A1_PIN, MOT_A2_PIN);
    set_motor_pwm(velY, MOT_B1_PIN, MOT_B2_PIN);

  } else if (vy < 500) {  // Retrocede
    set_motor_pwm(velY, MOT_A1_PIN, MOT_A2_PIN);
    set_motor_pwm(velY, MOT_B1_PIN, MOT_B2_PIN);

  } else {
    // Si no hay señal de avance ni retroceso, apagamos ambos motores
    set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
    set_motor_pwm(0, MOT_B1_PIN, MOT_B2_PIN);
  }

  // Comportamiento de giro
  if (vx > 520) {  // Girar a la derecha → activa solo rueda izquierda
    set_motor_pwm(150, MOT_A1_PIN, MOT_A2_PIN);  // Izquierda gira
    set_motor_pwm(0, MOT_B1_PIN, MOT_B2_PIN);    // Derecha apagada

  } else if (vx < 500) {  // Girar a la izquierda → activa solo rueda derecha
    set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);    // Izquierda apagada
    set_motor_pwm(150, MOT_B1_PIN, MOT_B2_PIN);  // Derecha gira
  }
}


void set_motor_pwm(int pwm, int IN1, int IN2) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(IN1, pwm);
    digitalWrite(IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, -pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void stopAll() {
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
}
