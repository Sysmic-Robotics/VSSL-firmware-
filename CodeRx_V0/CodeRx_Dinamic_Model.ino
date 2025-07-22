#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   8
#define CSN_PIN 10

#define MOT_A1_PIN 3  // Motor izquierda
#define MOT_A2_PIN 5
#define MOT_B1_PIN 6  // Motor derecha
#define MOT_B2_PIN 9

#define LED_PIN 7
#define DEADZONE 30

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
int dataReceived[2];
bool newData = false;

unsigned long lastReceiveTime = 0;
unsigned long timeoutDuration = 500;
unsigned long lastControlTime = 0;
const int controlPeriod = 10; // ms para control PI

// Parámetros físicos y de control (Paper)
float r = 0.025;      // Radio rueda [m]
float L = 0.07;       // Distancia entre ruedas [m]
float Kt = 0.1146;    // Constante de torque [Nm/A]
float J = 0.0002;     // Inercia total aproximada [kg.m^2]
float B = 0.0003;     // Fricción viscosa [Nm·s/rad]

// Controlador PI (velocidad angular ω)
float Kp = 0.2;
float Ki = 10;
float error_integral_L = 0;
float error_integral_R = 0;

// PWM actual aplicado a cada motor
int pwm_L = 0;
int pwm_R = 0;

// Velocidades estimadas (rad/s)
float omega_L = 0;
float omega_R = 0;

// Referencias de velocidad (rad/s)
float omega_ref_L = 0;
float omega_ref_R = 0;

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

  lastControlTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (radio.available()) {
    radio.read(&dataReceived, sizeof(dataReceived));
    lastReceiveTime = now;
    newData = true;
  }

  if (newData) {
    int vrx = dataReceived[0]; // Giro
    int vry = dataReceived[1]; // Avance
    procesarJoystick(vrx, vry);
    newData = false;
  }

  // Control PI a velocidad cada 10 ms
  if (now - lastControlTime >= controlPeriod) {
    actualizarControlVelocidad();
    lastControlTime = now;
  }

  // LED de fallo de comunicación
  if (now - lastReceiveTime > timeoutDuration) {
    digitalWrite(LED_PIN, (now / 300) % 2);
    stopAll();
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void procesarJoystick(int vx, int vy) {
  // Mapeo a velocidades lineales y angulares
  float vel_lineal = map(vy, 0, 1023, -100, 100) / 100.0 * 0.5; // m/s
  float vel_angular = map(vx, 0, 1023, -100, 100) / 100.0 * 3.0; // rad/s

  // Cálculo de velocidad de ruedas desde modelo cinemático
  omega_ref_R = (vel_lineal / r) + (L / (2 * r)) * vel_angular;
  omega_ref_L = (vel_lineal / r) - (L / (2 * r)) * vel_angular;

  Serial.print("wRef_L: "); Serial.print(omega_ref_L);
  Serial.print(" | wRef_R: "); Serial.println(omega_ref_R);
}

void actualizarControlVelocidad() {
  // Estimación ficticia de velocidad actual (idealmente usa encoder)
  omega_L = pwm_L * 0.1 / 255.0 * 10; // escala ficticia
  omega_R = pwm_R * 0.1 / 255.0 * 10;

  // Controlador PI para rueda izquierda
  float error_L = omega_ref_L - omega_L;
  error_integral_L += error_L * (controlPeriod / 1000.0);
  float uL = Kp * error_L + Ki * error_integral_L;
  pwm_L = constrain((int)(uL * 255.0 / 10.0), -255, 255);
  set_motor_pwm(pwm_L, MOT_A1_PIN, MOT_A2_PIN);

  // Controlador PI para rueda derecha
  float error_R = omega_ref_R - omega_R;
  error_integral_R += error_R * (controlPeriod / 1000.0);
  float uR = Kp * error_R + Ki * error_integral_R;
  pwm_R = constrain((int)(uR * 255.0 / 10.0), -255, 255);
  set_motor_pwm(pwm_R, MOT_B1_PIN, MOT_B2_PIN);
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
  set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(0, MOT_B1_PIN, MOT_B2_PIN);
  pwm_L = pwm_R = 0;
  error_integral_L = error_integral_R = 0;
}
