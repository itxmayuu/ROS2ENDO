#include <Arduino.h>
#include <math.h>
#include "esp_timer.h"

// === IMU ===
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno(55);

// ----------- Pines Motor 1 -----------
#define ENC_A1   32
#define ENC_B1   33
#define PWM_PIN1 16
#define DIR11    17
#define DIR21    18

// ----------- Pines Motor 2 -----------
#define ENC_A2   34
#define ENC_B2   35
#define PWM_PIN2 23
#define DIR12    26
#define DIR22    27

// ================== PARÁMETROS DE SISTEMA ==================
const float CPR        = 11.0f * 226.0f * 4.0f;
const float Ts         = 0.005f;
const float zonaMuerta = 5.5f;
const float omegaMax   = 3.1f;
const float Vmax       = 12.0f;
const float K_est      = 0.179f;

bool useFeedforward = true;

// --- PWM ---
const int PWM_RES_BITS = 12;
const int PWM_MAX      = (1 << PWM_RES_BITS) - 1;
const int PWM_CH1      = 0;
const int PWM_CH2      = 1;
const int PWM_FREQ     = 5000;

// =============== CONTROL: PI + LEAD =========================
float Kp = 50.2f;
float Ki = 50.5f;
float Kd = 0.001f;
const float alpha_d = 0.7f;

// =============== FILTRO DE VELOCIDAD ========================
#define USE_EMA
#ifdef USE_EMA
const float fc = 20.0f;
const float a_filt = expf(-2.0f * PI * fc * Ts);
#endif

// ================== ESTADO Y VARIABLES ======================
volatile long enc1 = 0;
volatile long enc2 = 0;

float ref1 = 0.0f;
float ref2 = 0.0f;

float omega1 = 0.0f, omega1_f = 0.0f;
float omega2 = 0.0f, omega2_f = 0.0f;

float integ1 = 0.0f;
float integ2 = 0.0f;

float d_e1 = 0.0f, e1_prev = 0.0f;
float d_e2 = 0.0f, e2_prev = 0.0f;

volatile float g_omega1_f = 0.0f, g_u1 = 0.0f;
volatile float g_omega2_f = 0.0f, g_u2 = 0.0f;
volatile float g_imu_heading = 0;

esp_timer_handle_t ctrl_timer;

// ================== ISRs ENCODER ============================
void IRAM_ATTR encoderISR_A1() {
  bool A = digitalRead(ENC_A1), B = digitalRead(ENC_B1);
  enc1 += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoderISR_B1() {
  bool A = digitalRead(ENC_A1), B = digitalRead(ENC_B1);
  enc1 += (A != B) ? 1 : -1;
}

void IRAM_ATTR encoderISR_A2() {
  bool A = digitalRead(ENC_A2), B = digitalRead(ENC_B2);
  enc2 += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoderISR_B2() {
  bool A = digitalRead(ENC_A2), B = digitalRead(ENC_B2);
  enc2 += (A != B) ? 1 : -1;
}

// ================== HELPERS =================================
static inline float satf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float deadzoneSoftBi(float u) {
  const float u_min = 0.6f;
  if (fabsf(u) < u_min) return 0.0f;

  const float u0 = 3.0f;
  float s = (u >= 0.0f) ? 1.0f : -1.0f;
  float a = fabsf(u);
  float comp = zonaMuerta * (1.0f - expf(-(a - u_min) / u0));
  return s * (a + comp);
}

static inline void applyVoltageMotor1(float V) {
  if (V >= 0.0f) {
    digitalWrite(DIR11, HIGH);
    digitalWrite(DIR21, LOW);
  } else {
    digitalWrite(DIR11, LOW);
    digitalWrite(DIR21, HIGH);
  }

  int pwm = (int)lroundf((fabsf(V) / Vmax) * PWM_MAX);
  if (pwm < 0) pwm = 0;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  ledcWrite(PWM_CH1, pwm);
}

static inline void applyVoltageMotor2(float V) {
  if (V >= 0.0f) {
    digitalWrite(DIR12, HIGH);
    digitalWrite(DIR22, LOW);
  } else {
    digitalWrite(DIR12, LOW);
    digitalWrite(DIR22, HIGH);
  }

  int pwm = (int)lroundf((fabsf(V) / Vmax) * PWM_MAX);
  if (pwm < 0) pwm = 0;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  ledcWrite(PWM_CH2, pwm);
}

// ================== CALLBACK DE CONTROL (5 ms) ==============
void controlStep(void*) {
  static long lastCount1 = 0;
  static long lastCount2 = 0;

  long d1;
  noInterrupts();
  d1 = enc1 - lastCount1;
  lastCount1 = enc1;
  interrupts();

  omega1 = ((float)d1 / CPR) * (2.0f * PI / Ts);
#ifdef USE_EMA
  omega1_f = a_filt * omega1_f + (1.0f - a_filt) * omega1;
#else
  omega1_f = omega1;
#endif

  long d2;
  noInterrupts();
  d2 = enc2 - lastCount2;
  lastCount2 = enc2;
  interrupts();

  omega2 = ((float)d2 / CPR) * (2.0f * PI / Ts);
#ifdef USE_EMA
  omega2_f = a_filt * omega2_f + (1.0f - a_filt) * omega2;
#else
  omega2_f = omega2;
#endif

  // ------- Control M1 -------
  float e1 = ref1 - omega1_f;
  float de1_raw = (e1 - e1_prev) / Ts;
  d_e1 = alpha_d * d_e1 + (1.0f - alpha_d) * de1_raw;
  e1_prev = e1;

  const float w_quiet = 0.20f;
  const float r_quiet = 0.10f;
  bool stop1 = (fabsf(ref1) < r_quiet) && (fabsf(omega1_f) < w_quiet);

  float u_ff1 = useFeedforward ? (ref1 / K_est) : 0.0f;
  float u_unsat1 = u_ff1 + Kp * e1 + Ki * integ1 + Kd * d_e1;
  float u_cmd1 = deadzoneSoftBi(u_unsat1);
  float u_sat1 = satf(u_cmd1, -Vmax, Vmax);

  bool notSat1 = (u_cmd1 == u_sat1);
  bool helpUp1 = (u_cmd1 > Vmax) && (e1 < 0.0f);
  bool helpDn1 = (u_cmd1 < -Vmax) && (e1 > 0.0f);

  if (notSat1 || helpUp1 || helpDn1) integ1 += e1 * Ts;
  else integ1 *= 0.995f;

  if (stop1) {
    integ1 *= 0.90f;
    u_sat1 = 0.0f;
  }

  applyVoltageMotor1(u_sat1);

  // ------- Control M2 -------
  float e2 = ref2 - omega2_f;
  float de2_raw = (e2 - e2_prev) / Ts;
  d_e2 = alpha_d * d_e2 + (1.0f - alpha_d) * de2_raw;
  e2_prev = e2;

  bool stop2 = (fabsf(ref2) < r_quiet) && (fabsf(omega2_f) < w_quiet);

  float u_ff2 = useFeedforward ? (ref2 / K_est) : 0.0f;
  float u_unsat2 = u_ff2 + Kp * e2 + Ki * integ2 + Kd * d_e2;
  float u_cmd2 = deadzoneSoftBi(u_unsat2);
  float u_sat2 = satf(u_cmd2, -Vmax, Vmax);

  bool notSat2 = (u_cmd2 == u_sat2);
  bool helpUp2 = (u_cmd2 > Vmax) && (e2 < 0.0f);
  bool helpDn2 = (u_cmd2 < -Vmax) && (e2 > 0.0f);

  if (notSat2 || helpUp2 || helpDn2) integ2 += e2 * Ts;
  else integ2 *= 0.995f;

  if (stop2) {
    integ2 *= 0.90f;
    u_sat2 = 0.0f;
  }

  applyVoltageMotor2(u_sat2);

  g_omega1_f = omega1_f;
  g_u1 = u_sat1;
  g_omega2_f = omega2_f;
  g_u2 = u_sat2;
}

// ================== SETUP / LOOP =============================
void setup() {
  Serial.begin(921600);
  Serial.setTimeout(10);
  delay(200);

  Wire.begin();
  bool imu_ok = bno.begin();
  if (!imu_ok) {
    Serial.println("No BNO055 detected!");
  } else {
    delay(1000);
    bno.setExtCrystalUse(true);
  }

  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderISR_A1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderISR_B1, CHANGE);

  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR_A2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderISR_B2, CHANGE);

  pinMode(DIR11, OUTPUT);
  pinMode(DIR21, OUTPUT);
  digitalWrite(DIR11, HIGH);
  digitalWrite(DIR21, LOW);

  pinMode(DIR12, OUTPUT);
  pinMode(DIR22, OUTPUT);
  digitalWrite(DIR12, HIGH);
  digitalWrite(DIR22, LOW);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PWM_PIN1, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PWM_PIN2, PWM_CH2);

  const esp_timer_create_args_t targs = {
    .callback = &controlStep,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "ctrl5ms"
  };

  esp_err_t err = esp_timer_create(&targs, &ctrl_timer);
  if (err != ESP_OK) Serial.println("ERROR: esp_timer_create");

  err = esp_timer_start_periodic(ctrl_timer, (uint64_t)(Ts * 1e6));
  if (err != ESP_OK) Serial.println("ERROR: esp_timer_start_periodic");

  Serial.println(
    "Dual PI+lead bidireccional listo (M1 y M2). Envia 'w1,w2' en rad/s (puede ser negativo). Ej: 1.8,-1.2"
  );
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    int comma = line.indexOf(',');

    if (comma > 0) {
      float r1 = line.substring(0, comma).toFloat();
      float r2 = line.substring(comma + 1).toFloat();

      if (r1 >= -omegaMax && r1 <= omegaMax) ref1 = r1;
      if (r2 >= -omegaMax && r2 <= omegaMax) ref2 = r2;
    }
  }

  sensors_event_t event;
  if (bno.getEvent(&event)) {
    g_imu_heading = event.orientation.x;
  }

  static uint32_t last_tx = 0;
  uint32_t now = millis();

  if (now - last_tx >= 50) {
    last_tx = now;

    float w1 = g_omega1_f, u1 = g_u1;
    float w2 = g_omega2_f, u2 = g_u2;

    Serial.print(">omega1:"); Serial.println(w1, 3);
    Serial.print(">u1:");     Serial.println(u1, 3);
    Serial.print(">omega2:"); Serial.println(w2, 3);
    Serial.print(">u2:");     Serial.println(u2, 3);
    Serial.print(">imu:");    Serial.println(g_imu_heading);
  }
}
