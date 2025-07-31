
#include <Arduino.h>


#define MOT_A1_PIN 3     // IN1 motor A
#define MOT_A2_PIN 5     // IN2 motor A
#define MOT_B1_PIN 6     // IN1 motor B
#define MOT_B2_PIN 9     // IN2 motor B

#define LED_PIN    13    


const int SPEED_LOW  = 100;   
const int SPEED_HIGH = 255;   


const unsigned long BLINK_LOW_MS  = 600UL;  // LED lento
const unsigned long BLINK_HIGH_MS = 150UL;  // LED rápido


bool testMode  = true;   // Activa/desactiva la prueba EDITAR PARA DESACTIVAR
bool highSpeed = false;  


void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN);
void set_motor_currents(int pwm_A, int pwm_B);
void test_motor(void);
void blink_status_led(void);


void setup() {
  
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);
}

/* -------------------------------------------------------------- */
void loop() {
  test_motor();       // rutina de 6 actos/5 s, alterna baja↔alta
  blink_status_led(); // parpadeo según velocidad

  /* Ejemplo para detener la prueba desde el monitor serie
     if (Serial.available() && Serial.read() == 'q') testMode = false; */
}

/* =================== Funciones auxiliares ===================== */
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN) {
  if (pwm < 0) {                       // atrás
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);
  } else {                             // adelante o paro
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void set_motor_currents(int pwm_A, int pwm_B) {
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  Serial.print(F("A=")); Serial.print(pwm_A);
  Serial.print(F("  B=")); Serial.println(pwm_B);
}

/* ---------------- Rutina de test de motores ------------------- */
void test_motor() {
  if (!testMode) return;

  static uint8_t       stage        = 0;   // 0-5
  static unsigned long stageStartMs = 0;

  unsigned long now = millis();
  if (now - stageStartMs >= 5000UL) {      // 5 s por acto
    stageStartMs = now;
    stage = (stage + 1) % 6;
    if (stage == 0) highSpeed = !highSpeed; // alterna velocidad
  }

  int pwm = highSpeed ? SPEED_HIGH : SPEED_LOW;

  switch (stage) {
    case 0: set_motor_currents( pwm ,  pwm ); break;  // ↑↑
    case 1: set_motor_currents( pwm ,    0 ); break;  // ↑·
    case 2: set_motor_currents(   0 ,  pwm ); break;  // ·↑
    case 3: set_motor_currents(-pwm , -pwm ); break;  // ↓↓
    case 4: set_motor_currents(-pwm ,    0 ); break;  // ↓·
    case 5: set_motor_currents(   0 , -pwm ); break;  // ·↓
  }
}

/* --------------- Parpadeo del LED indicador ------------------- */
void blink_status_led() {
  static unsigned long lastToggle = 0;
  static bool          ledState   = false;

  if (!testMode) {                    // LED apagado si no hay test
    digitalWrite(LED_PIN, LOW);
    return;
  }

  unsigned long interval = highSpeed ? BLINK_HIGH_MS : BLINK_LOW_MS;
  unsigned long now      = millis();

  if (now - lastToggle >= interval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastToggle = now;
  }
}
