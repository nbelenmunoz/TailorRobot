/*
 * Control de velocidad continua — Motor paso a paso
 * Wantai 42BYGHW811  (1.8°/paso, 200 pasos/vuelta, 2.5 A)
 * Arduino Mega 2560 + CNC Shield (A4988 / DRV8825)
 *
 * Pines (Mega 2560):
 *   STEP   = 2   →  Puerto PE4
 *   DIR    = 5   →  Puerto PE3
 *   ENABLE = 8   →  Puerto PH5
 * Comandos por Serial (9600 baud, terminador LF o CR):
 *   S500    →  fija velocidad a 500 pasos/s
 *   S0      →  DETIENE el motor
 *   R150    →  fija velocidad a 150 RPM
 *   E       →  habilita driver  (ENABLE = LOW)
 *   D       →  deshabilita driver (ENABLE = HIGH)
 *   +       →  sentido horario  (CW)
 *   -       →  sentido antihorario (CCW)
 *   ?       →  imprime estado actual

 */

#include <Arduino.h>

// ============================================================
//  CONFIGURACIÓN — ajusta aquí según tu hardware
// ============================================================
#define MICROSTEP_DIV   1          // 1 | 2 | 4 | 8 | 16 | 32
#define BASE_STEPS_REV  200        // pasos/vuelta del motor (1.8°/paso → 200)

const byte STEP_PIN   = 2;         // PE4 en Arduino Mega
const byte DIR_PIN    = 5;         // PE3 en Arduino Mega
const byte ENABLE_PIN = 8;         // PH5 en Arduino Mega

const float MIN_RPM = 0.5f;
const float MAX_RPM = 600.0f;      // límite conservador; ajusta según torque real

// ============================================================
//  Variables calculadas — no editar
// ============================================================
const float STEPS_PER_REV = (float)BASE_STEPS_REV * MICROSTEP_DIV;
const float MAX_SPS        = (MAX_RPM * STEPS_PER_REV) / 60.0f;
const float MIN_SPS        = (MIN_RPM * STEPS_PER_REV) / 60.0f;

// Estado del sistema
//  [FIX-3] currentSpeed_sps se escribe desde loop() y se lee en ISR → proteger
volatile float currentSpeed_sps = 0.0f;
bool motorEnabled  = false;
bool timerRunning  = false;
bool dirCW         = true;

// Buffer serie
char serialBuffer[32];
byte bufferIndex = 0;

// ============================================================
void setup() {
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // [FIX-4] Arranca deshabilitado para evitar movimientos inesperados
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIR_PIN,    HIGH);   // CW por defecto
  digitalWrite(STEP_PIN,   LOW);

  Serial.begin(9600);
  while (!Serial) { /* espera USB-CDC en Mega */ }

  Serial.println(F("=== Control Motor Paso a Paso ==="));
  Serial.print(F("Microstepping : 1/")); Serial.println(MICROSTEP_DIV);
  Serial.print(F("Pasos/vuelta  : ")); Serial.println(STEPS_PER_REV, 0);
  Serial.print(F("Rango pasos/s : "));
  Serial.print(MIN_SPS, 1); Serial.print(F(" – ")); Serial.print(MAX_SPS, 1);
  Serial.println(F(" pps"));
  Serial.println(F("Comandos: S500 | S0 | R150 | E | D | + | - | ?"));
  Serial.println(F("Motor: DESHABILITADO (envía 'E' para habilitar)"));
  Serial.println(F("================================="));

  // ---- Configurar Timer1: CTC, prescaler 8, tick = 0.5 µs ----
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 65535;
  TCCR1B |= (1 << WGM12);     // CTC
  TCCR1B |= (1 << CS11);      // Prescaler 8
  TIMSK1 &= ~(1 << OCIE1A);   // ISR desactivada hasta 'E' + velocidad
  interrupts();
}

// ============================================================
void loop() {
  leerComandoSerial();
}

// ============================================================
void leerComandoSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        procesarComando(serialBuffer);
        bufferIndex = 0;
      }
    } else if (bufferIndex < (byte)(sizeof(serialBuffer) - 1)) {
      serialBuffer[bufferIndex++] = c;
    }
  }
}

// ============================================================
void procesarComando(const char* cmd) {

  // ---- Comandos de un solo carácter ----
  switch (cmd[0]) {

    case 'E': case 'e':
      motorEnabled = true;
      digitalWrite(ENABLE_PIN, LOW);
      Serial.println(F("Motor HABILITADO"));
      // Re-aplicar velocidad si había una antes de deshabilitar
      {
        noInterrupts();
        float sps = currentSpeed_sps;
        interrupts();
        if (sps > 0.0f) setSpeedSPS(sps);
      }
      return;

    case 'D': case 'd':
      setSpeedSPS(0);                    // Detener pulsos primero
      motorEnabled = false;
      digitalWrite(ENABLE_PIN, HIGH);    // Deshabilitar bobinas
      Serial.println(F("Motor DESHABILITADO"));
      return;

    case '+':
      dirCW = true;
      digitalWrite(DIR_PIN, HIGH);
      Serial.println(F("Dirección: HORARIO (CW)"));
      return;

    case '-':
      dirCW = false;
      digitalWrite(DIR_PIN, LOW);
      Serial.println(F("Dirección: ANTIHORARIO (CCW)"));
      return;

    case '?':
      imprimirEstado();
      return;
  }

  // ---- Comandos con valor numérico: S<pps> o R<rpm> ----
  float value = atof(cmd + 1);

  switch (cmd[0]) {

    case 'S': case 's': {
      // [FIX-1] valor 0 → detener, sin clampear a MIN_SPS
      if (value <= 0.0f) {
        noInterrupts();
        currentSpeed_sps = 0.0f;
        interrupts();
        setSpeedSPS(0);
        Serial.println(F("Motor DETENIDO"));
      } else {
        if (value < MIN_SPS) value = MIN_SPS;
        if (value > MAX_SPS) value = MAX_SPS;
        noInterrupts();
        currentSpeed_sps = value;
        interrupts();
        setSpeedSPS(value);
        float rpm = (value * 60.0f) / STEPS_PER_REV;
        Serial.print(F("Vel: ")); Serial.print(value, 1);
        Serial.print(F(" pps | ")); Serial.print(rpm, 2); Serial.println(F(" RPM"));
      }
      break;
    }

    case 'R': case 'r': {
      float sps = (value * STEPS_PER_REV) / 60.0f;
      if (sps <= 0.0f) {
        noInterrupts();
        currentSpeed_sps = 0.0f;
        interrupts();
        setSpeedSPS(0);
        Serial.println(F("Motor DETENIDO"));
      } else {
        if (sps < MIN_SPS) sps = MIN_SPS;
        if (sps > MAX_SPS) sps = MAX_SPS;
        float rpm_real = (sps * 60.0f) / STEPS_PER_REV;
        noInterrupts();
        currentSpeed_sps = sps;
        interrupts();
        setSpeedSPS(sps);
        Serial.print(F("Vel: ")); Serial.print(sps, 1);
        Serial.print(F(" pps | ")); Serial.print(rpm_real, 2); Serial.println(F(" RPM"));
      }
      break;
    }

    default:
      Serial.print(F("Cmd desconocido: ")); Serial.println(cmd);
      Serial.println(F("Usa: S500 | S0 | R150 | E | D | + | - | ?"));
      break;
  }
}

// ============================================================
void imprimirEstado() {
  noInterrupts();
  float sps = currentSpeed_sps;
  interrupts();

  Serial.println(F("--- ESTADO ---"));
  Serial.print(F("Motor     : ")); Serial.println(motorEnabled ? F("Habilitado") : F("Deshabilitado"));
  Serial.print(F("Dirección : ")); Serial.println(dirCW ? F("Horario (CW)") : F("Antihorario (CCW)"));
  Serial.print(F("Velocidad : "));
  Serial.print(sps, 1); Serial.print(F(" pps | "));
  Serial.print((sps * 60.0f) / STEPS_PER_REV, 2); Serial.println(F(" RPM"));
  Serial.print(F("Timer ISR : ")); Serial.println(timerRunning ? F("Activo") : F("Inactivo"));
  Serial.print(F("Microstep : 1/")); Serial.println(MICROSTEP_DIV);
  Serial.println(F("--------------"));
}

// ============================================================
// ============================================================
void setSpeedSPS(float sps) {
  if (!motorEnabled || sps <= 0.0f) {
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);    // Deshabilitar ISR
    PORTE  &= ~(1 << PE4);       // [FIX-2] pin STEP = LOW directo
    interrupts();
    timerRunning = false;
    return;
  }

  // Calcular y clampear OCR
  float ocr_f = (1000000.0f / sps) - 1.0f;
  if (ocr_f <     1.0f) ocr_f =     1.0f;
  if (ocr_f > 65535.0f) ocr_f = 65535.0f;
  uint16_t newOCR = (uint16_t)(ocr_f + 0.5f);

  noInterrupts();
  OCR1A  = newOCR;
  TCNT1  = 0;                    // Reset: evita un primer ciclo incompleto
  TIMSK1 |= (1 << OCIE1A);      // Habilitar ISR
  interrupts();

  timerRunning = true;
}

ISR(TIMER1_COMPA_vect) {
  PORTE ^= (1 << PE4);    // Toggle pin 2 en Arduino Mega (PE4)
}
