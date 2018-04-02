

// --------------- Device code ----------------

// --------- LED ----------

#if defined (CONFIG_LED)

void led_red_on() {

  digitalWrite(LED_RED_PIN, HIGH);
}

void led_red_off() {

  digitalWrite(LED_YELLOW_PIN, LOW);
}

void led_yellow_on() {

  digitalWrite(LED_YELLOW_PIN, HIGH);
}

void led_yellow_off() {

  digitalWrite(LED_RED_PIN, LOW);
}

void led_setup() {

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
}

#endif // defined (CONFIG_LED)

// --------- Battery Voltage ----------

#if defined (CONFIG_POWER)

float power_get_voltage(int pin) {

  int val = analogRead(pin);

  return val * 0.0049;
}


#endif // defined (CONFIG_BATTERY)

// --------- Motor ----------

#if defined (CONFIG_MOTOR_2WD_CYTRON)

void m_set_pwm(int v_left_new, int v_left_direction, int v_right_new, int v_right_direction) {

  int v_left_pwm = 0;
  int v_left_dir = 0;
  int v_right_pwm = 0;
  int v_right_dir = 0;

  // Suggestion maybe 1 forward -1 backward 0 release

  if (v_left_direction == 1 || v_left_direction == '+') {
    v_left_dir = LOW;
    v_left_pwm = v_left_new;
  } else if (v_left_direction == -1 || v_left_direction == 2 || v_left_direction == '-') {
    v_left_dir = HIGH;
    v_left_pwm = v_left_new;
  } else {
    v_left_pwm = 0;
    v_right_pwm = 0;
  }

  if (v_right_direction == 1 || v_right_direction == '+') {
    v_right_dir = LOW;
    v_right_pwm = v_right_new;
  } else if (v_right_direction == -1 || v_right_direction == 2 || v_right_direction == '-') {
    v_right_dir = HIGH;
    v_right_pwm = v_right_new;
  } else {
    v_left_pwm = 0;
    v_right_pwm = 0;
  }

  digitalWrite(M_LEFT_DIR_PIN, v_left_dir);
  digitalWrite(M_RIGHT_DIR_PIN, v_right_dir);

  analogWrite(M_LEFT_PWM_PIN, v_left_pwm);
  analogWrite(M_RIGHT_PWM_PIN, v_right_pwm);
}

void m_stop() {

  digitalWrite(M_LEFT_DIR_PIN, 0);
  digitalWrite(M_RIGHT_DIR_PIN, 0);

  analogWrite(M_LEFT_PWM_PIN, 0);
  analogWrite(M_RIGHT_PWM_PIN, 0);
}

void m_setup() {
  // Need update for all hardware platform
  pinMode(M_LEFT_PWM_PIN, OUTPUT);
  pinMode(M_LEFT_DIR_PIN, OUTPUT);

  pinMode(M_RIGHT_PWM_PIN, OUTPUT);
  pinMode(M_RIGHT_DIR_PIN, OUTPUT);

}

#endif // defined (CONFIG_MOTOR_2WD_CYTRON)




#if defined (CONFIG_MOTOR_2WD_ADAFRUIT)

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m_left = AFMS.getMotor(M_LEFT_MOTOR_INDEX);
Adafruit_DCMotor *m_right = AFMS.getMotor(M_RIGHT_MOTOR_INDEX);

void m_set_pwm(int v_left_new, int v_left_direction, int v_right_new,
               int v_right_direction) {
// LEE only run forward at initializing at setup up 
// may simplely comment backward setting when not necessary if no sharp turn 
  if (v_left_direction == 1 || v_left_direction == '+') {
    m_left->setSpeed(v_left_new);
    m_left->run(FORWARD);
  } else if (v_left_direction == -1 || v_left_direction == 2
             || v_left_direction == '-') {
    m_left->setSpeed(v_left_new);
    m_left->run(BACKWARD);
  } else {
    m_left->run(RELEASE);
  }

  if (v_right_direction == 1 || v_right_direction == '+') {
    m_right->setSpeed(v_right_new);
    m_right->run(FORWARD);
  } else if (v_right_direction == -1 || v_right_direction == 2
             || v_right_direction == '-') {
    m_right->setSpeed(v_right_new);
    m_right->run(BACKWARD);
  } else {
    m_right->run(RELEASE);
  }

}

void m_stop() {
// LEE Release may not equal to setSpeed(0)
  m_left->run(RELEASE);
  m_right->run(RELEASE);
}

void m_setup() {

  AFMS.begin();
}

#endif // defined (CONFIG_MOTOR_2WD_ADAFRUIT)



// --------- Encoder ----------

#if defined (CONFIG_ENCODER_2WD)

// Encoder

#include <Encoder.h>

Encoder EncL(ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
Encoder EncR(ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

long encoder_left_read() {
  return EncL.read();
}

long encoder_right_read() {
  return EncR.read();
}

void encoder_left_write(long val) {
  EncL.write(val);
}

void encoder_right_write(long val) {
  EncR.write(val);
}

void encoder_reset() {

  EncL.write(0);
  EncR.write(0);
}

float encoder_calculate_angular_speed(long delta_tick, long delta_time) {

  return 1.0 * (delta_tick) / (delta_time / 1000.0) * 2 * PI / ENCODER_CPT_GEARED;
}

#endif // defined (CONFIG_ENCODER_2WD)

// --------- IMU ----------

#if defined (CONFIG_IMU_BNO055)

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 imu_bno055 = Adafruit_BNO055();

imu::Vector<3> euler_init;

imu::Vector<3> euler;
imu::Vector<3> acc;
imu::Vector<3> gyro;

void imu_setup() {

  imu_bno055.begin();

  delay(1000);

  euler_init = imu_bno055.getVector(Adafruit_BNO055::VECTOR_EULER);

}

void imu_read() {

  euler = imu_bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
  acc = imu_bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gyro = imu_bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

float imu_get_abs_theta() {

  float theta_degree = euler.x();

  if(theta_degree < -180) {

    theta_degree += 360;
  }

  if(theta_degree > 180) {

    theta_degree -= 360;
  }

  return theta_degree * 3.14 / 180.0;

  return theta_degree;
}

float imu_get_rel_theta() {

  float theta_degree = euler.x() - euler_init.x();

  if(theta_degree < -180) {

    theta_degree += 360;
  }

  if(theta_degree > 180) {

    theta_degree -= 360;
  }

  return theta_degree * 3.14 / 180.0;
}

float imu_get_accx() {

  return acc.x();
}

float imu_get_omega() {

  return gyro.z();
}

#endif

// --------- Ultrasonic sensor ----------

#if defined (CONFIG_ULTRASONIC_HC_SR04)

#include <NewPing.h>

NewPing sonar_left(ULTRASONIC_LEFT_TRIGGER_PIN, ULTRASONIC_LEFT_ECHO_PIN, ULTRASONIC_MAX_DISTANCE);
NewPing sonar_right(ULTRASONIC_RIGHT_TRIGGER_PIN, ULTRASONIC_RIGHT_ECHO_PIN, ULTRASONIC_MAX_DISTANCE);

float ultrasonic_get_delta_x(int side) {

  // get delta x from ultrasonic sensor
  int us = 0;

  if(side == 0) {

    us = sonar_left.ping();
  } else if (side == 1) {

    us = sonar_right.ping();
  } else {

    return 0;
  }
  
  
  float distance = us / 57.0 / 100.0;
  if(distance >= ULTRASONIC_MAX_DISTANCE) {
    distance = ULTRASONIC_MAX_DISTANCE;
  }

  return distance;
}

#endif


