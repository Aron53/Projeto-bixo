#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>

#include "secrets.h"

// ==========================================
// 1. DEFINICAO DE PINOS
// ==========================================
#define ENC_IN_ESQ_A 0
#define ENC_IN_ESQ_B 1
#define ENC_IN_DIR_A 2
#define ENC_IN_DIR_B 3

#define MOT_ESQ_PWM 4
#define MOT_ESQ_IN1 5
#define MOT_ESQ_IN2 6
#define MOT_DIR_PWM 7
#define MOT_DIR_IN1 8
#define MOT_DIR_IN2 9

// ==========================================
// 1B. GEOMETRIA DO ROBO E PARAMETROS (Do segundo código)
// ==========================================
const float RAIO_RODA = 0.0336f;
const float DIST_RODAS = 0.2140f;
const float PPR_ENCODER = 330.0f;

const float VEL_LINEAR_MAX_MS = 0.3f;   
const float VEL_ANGULAR_MAX_RADS = 2.0f; 

const float KP_LINEAR = 0.6f;
const float KP_ANGULAR = 1.5f;

// Ganhos do PID (Ajustar conforme necessário)
const float Kp = 1.0f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// ==========================================
// 2. VARIAVEIS GLOBAIS DE SISTEMA (Do segundo código)
// ==========================================
volatile long ticks_esq = 0;
volatile long ticks_dir = 0;
float ultimos_ticks_esq = 0;
float ultimos_ticks_dir = 0;

volatile float setpoint_esq = 0.0f;
volatile float setpoint_dir = 0.0f;

// Variáveis para compatibilidade com a cinemática do 1º código
volatile float setpoint_omega_esq = 0.0f;
volatile float setpoint_omega_dir = 0.0f;

// Posição
float pos_atual_x = 0.0f;
float pos_atual_y = 0.0f;
float pos_theta = 0.0f;

// Variáveis de Erro do PID
float erro_int_esq = 0.0f;
float erro_int_dir = 0.0f;
float erro_anterior_esq = 0.0f;
float erro_anterior_dir = 0.0f;

const float DT = 0.05f; 
unsigned long tempo_anterior = 0;
const int INTERVALO_AMOSTRAGEM_MS = 50; 

// ==========================================
// 3. MICRO-ROS (Estrutura do primeiro código)
// ==========================================
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t goal_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Point goal_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) error_loop(); \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    (void)rc; \
  }

void error_loop() {
  while (true) {
    Serial.println("error_loop: rcl init falhou");
    delay(500);
  }
}

// ==========================================
// 4. INTERRUPCOES DE HARDWARE (ISRs - Do segundo código)
// ==========================================
void IRAM_ATTR isr_encoder_esq() {
  if (digitalRead(ENC_IN_ESQ_B) == HIGH) {
    ticks_esq++;
  } else {
    ticks_esq--;
  }
}

void IRAM_ATTR isr_encoder_dir() {
  if (digitalRead(ENC_IN_DIR_B) == HIGH) {
    ticks_dir++;
  } else {
    ticks_dir--;
  }
}

// ==========================================
// 5. FUNCOES DE CALCULO E CONTROLE (Do segundo código)
// ==========================================
void calcula_odometria() {
  noInterrupts();
  long ticks_atuais_esq = ticks_esq;
  long ticks_atuais_dir = ticks_dir;
  interrupts();

  long delta_ticks_esq = ticks_atuais_esq - (long)ultimos_ticks_esq;
  long delta_ticks_dir = ticks_atuais_dir - (long)ultimos_ticks_dir;

  ultimos_ticks_dir = ticks_atuais_dir;
  ultimos_ticks_esq = ticks_atuais_esq;

  float Delta_rad_esq = ((float)delta_ticks_esq / PPR_ENCODER) * 2.0f * PI;
  float Delta_rad_dir = ((float)delta_ticks_dir / PPR_ENCODER) * 2.0f * PI;

  float V_Esq = Delta_rad_esq * RAIO_RODA / DT;
  float V_Dir = Delta_rad_dir * RAIO_RODA / DT;

  // Atualiza setpoints locais usando os setpoints angulares gerados pelo ROS (Cinemática do 1º código)
  setpoint_esq = setpoint_omega_esq * RAIO_RODA;
  setpoint_dir = setpoint_omega_dir * RAIO_RODA;

  float PWM_esquerdo = controle_pid(setpoint_esq, V_Esq, erro_int_esq, erro_anterior_esq);
  float PWM_direito  = controle_pid(setpoint_dir, V_Dir, erro_int_dir, erro_anterior_dir);

  aciona_motores(PWM_esquerdo, PWM_direito);
}

float controle_pid(float setpoint, float medida, float &erro_int, float &erro_anterior) {
  float erro = setpoint - medida;

  float proporcional = Kp * erro;
  float derivada = Kd * (erro - erro_anterior) / DT;
  erro_int += erro * DT;
  float integral = Ki * erro_int;

  erro_anterior = erro;

  float saida = proporcional + integral + derivada;
  return constrain(saida, -255.0f, 255.0f);
}

void aciona_motores(float PWM_esquerdo, float PWM_direito) {
  if (PWM_esquerdo < 0) {
    digitalWrite(MOT_ESQ_IN1, LOW);
    digitalWrite(MOT_ESQ_IN2, HIGH);
    analogWrite(MOT_ESQ_PWM, (int)abs(PWM_esquerdo));
  } else {
    digitalWrite(MOT_ESQ_IN1, HIGH);
    digitalWrite(MOT_ESQ_IN2, LOW);
    analogWrite(MOT_ESQ_PWM, (int)abs(PWM_esquerdo));
  }

  if (PWM_direito < 0) {
    digitalWrite(MOT_DIR_IN1, LOW);
    digitalWrite(MOT_DIR_IN2, HIGH);
    analogWrite(MOT_DIR_PWM, (int)abs(PWM_direito));
  } else {
    digitalWrite(MOT_DIR_IN1, HIGH);
    digitalWrite(MOT_DIR_IN2, LOW);
    analogWrite(MOT_DIR_PWM, (int)abs(PWM_direito));
  }
}

// ==========================================
// 6. CINEMATICA DIFERENCIAL + INTEGRACAO ROS 2 (Do primeiro código)
// ==========================================
void aplica_velocidade(float v, float w) {
  v = constrain(v, -VEL_LINEAR_MAX_MS, VEL_LINEAR_MAX_MS);
  w = constrain(w, -VEL_ANGULAR_MAX_RADS, VEL_ANGULAR_MAX_RADS);

  float vel_linear_esq = v - (w * DIST_RODAS / 2.0f);
  float vel_linear_dir = v + (w * DIST_RODAS / 2.0f);

  setpoint_omega_esq = vel_linear_esq / RAIO_RODA;
  setpoint_omega_dir = vel_linear_dir / RAIO_RODA;
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  aplica_velocidade((float)msg->linear.x, (float)msg->angular.z);
}

void coordenada_para_velocidade(float x, float y, float *v, float *w) {
  float distancia = sqrtf(x * x + y * y);
  float angulo_para_alvo = atan2f(x, y); 

  *w = KP_ANGULAR * angulo_para_alvo;
  *v = KP_LINEAR * distancia;

  if (fabsf(angulo_para_alvo) > 0.3f) {
    *v = 0.0f;
  }
}

void goal_callback(const void *msgin) {
  const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
  float v, w;
  coordenada_para_velocidade((float)msg->x, (float)msg->y, &v, &w);
  aplica_velocidade(v, w);
}

// ==========================================
// SETUP INICIAL
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_IN_ESQ_A, INPUT_PULLUP);
  pinMode(ENC_IN_ESQ_B, INPUT_PULLUP);
  pinMode(ENC_IN_DIR_A, INPUT_PULLUP);
  pinMode(ENC_IN_DIR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_ESQ_A), isr_encoder_esq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_DIR_A), isr_encoder_dir, RISING);

  pinMode(MOT_ESQ_PWM, OUTPUT);
  pinMode(MOT_ESQ_IN1, OUTPUT);
  pinMode(MOT_ESQ_IN2, OUTPUT);
  pinMode(MOT_DIR_PWM, OUTPUT);
  pinMode(MOT_DIR_IN1, OUTPUT);
  pinMode(MOT_DIR_IN2, OUTPUT);

  // Setup do micro-ROS (Lógica completa do primeiro código)
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASS,
                                agent_ip, AGENT_PORT);
  WiFi.setSleep(false);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "bixo_esp32c3_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &goal_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "bixo/goal"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
                                        &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &goal_subscriber, &goal_msg,
                                        &goal_callback, ON_NEW_DATA));

  Serial.println("Sistema Iniciado. Aguardando inicio dos ciclos de controle...");
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  unsigned long tempo_atual = millis();

  if (tempo_atual - tempo_anterior >= INTERVALO_AMOSTRAGEM_MS) {
    calcula_odometria();

    Serial.print("Ticks_Esq:");
    Serial.print(ticks_esq);
    Serial.print("\tTicks_Dir:");
    Serial.print(ticks_dir);
    Serial.print("\tSetpoint_Esq(rad/s):");
    Serial.print(setpoint_omega_esq);
    Serial.print("\tSetpoint_Dir(rad/s):");
    Serial.println(setpoint_omega_dir);

    tempo_anterior = tempo_atual;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}