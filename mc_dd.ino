// mc_dd.ino
//
// motion control (differential drive)


#include <Wire.h>
#include <math.h>
#include <TimerOne.h>

// Vehicle Setting Macro
#define CONFIG_PLATFORM_VC_TRUCK_V1
//#define CONFIG_PLATFORM_VC_BOX_V1


#include "config.h"
#include "serial.h"
#include "device.h"
#include "ctrl.h"

// system state can be 0 (stopped) or 1 (running), i.e. motor power state
int motor_power_state = 0;


long timestamps[8];



// robot state

float imu_abs_theta = 0;
float imu_rel_theta = 0;
float imu_accx = 0;
float imu_omega = 0;

void imu_get_theta_accx_omega() {

  imu_read();
  imu_abs_theta = imu_get_abs_theta();
  imu_rel_theta = imu_get_rel_theta();
  imu_accx = imu_get_accx();
  imu_omega = imu_get_omega();
}


float ultrasonic_left_delta_x = 0;
float ultrasonic_right_delta_x = 0;

void ultrasonic_get_delta_x() {

  ultrasonic_left_delta_x = ultrasonic_get_delta_x(0);
  ultrasonic_right_delta_x = ultrasonic_get_delta_x(1);
}

float power_voltage = 0;
float motor_voltage = 0;

void power_state_update() {

  power_voltage = power_get_voltage(POWER_VOLTAGE_PIN);
  motor_voltage = power_get_voltage(MOTOR_VOLTAGE_PIN);

}


// ----------------------------------- interrupt -------------------------

// Timer interrupt counter. It is reset when new loop period comes.
int timer_counter = 0;

// Timer flag for the controller loop
volatile int timer_inner_loop_flag = 0;

// CAUTION: This function is called in interrupt!!! Must make it short!!!
void timer_update() {

  timer_counter++;

  if (timer_counter >= ctrl_loop_period) {
    timer_inner_loop_flag = 1;
    timer_counter = 0;
  }

}

// ----------------------------------- setup --------------------------

void setup() {

  Serial.begin(115200);  // Serial Setup

  // delay(1000);

  // Serial.print("1\n");

  Timer1.initialize(1000); // NOTE: period is in us
  Timer1.attachInterrupt(timer_update);

  // Serial.print("2\n");

  // LED
  led_setup();

  // motor setup
  m_setup();

  imu_setup();

  // Serial.print("3\n");

  // long timestamp = millis();
  // send_ctrl_status_debug(timestamp);

  // in case of unexpected start state, reset everything
  m_stop(); 
  encoder_reset();
  ctrl_reset();

  // Serial.print("4\n");

  // TODO: Add some POST code

}

// ----------------------------------- loop -------------------------



unsigned long loop_count = 0;

void loop() {


  if (timer_inner_loop_flag > 0) {

    loop_count++;

    unsigned long timestamp = millis();
    timestamps[0] = millis() - timestamp;
    
    // sensor data gathering
    power_state_update();

    imu_get_theta_accx_omega(); // roughly 1 ~ 2 ms (Assume I2C bus runs at 400 KHz)

    timestamps[1] = millis() - timestamp;
    
    // TODO: poll the sensor in the timer interrupt instead of waiting here.
    // ultrasonic_get_delta_x(); // roughly 25 ms each sensor if maxmum distance is 3m
    
    timestamps[2] = millis() - timestamp;

    ctrl_get_current_wl_wr(); // less than 1 ms

    timestamps[3] = millis() - timestamp;
    
    serial_send_vehicle_status(timestamp); // at most 1 ms at 230400 bps
    
    timestamps[4] = millis() - timestamp;
 
    if (motor_power_state > 0) {

      // Run controller.
      ctrl_inner_loop(); // less than 1 ms
      
      timestamps[5] = millis() - timestamp;

      // Set controller output to motor.
      ctrl_set_pwm(); // less than 1 ms for directly write PWM to motor driver board
      
      timestamps[6] = millis() - timestamp;

      serial_send_ctrl_status_debug(timestamp); // roughly 2 ~ 3 ms at 230400 bps

      timestamps[7] = millis() - timestamp;

      ctrl_inner_loop_update(); 
    }

    timer_inner_loop_flag = 0;
  }

  // Run Serial State Machine
  // CAUTION: Never block or delay or spend too much time here
  if (Serial.available() > 0) {
    int c = Serial.read();
    serial_state_machine_proceed(c);
  }

  // Serial.print("loop\n");
  
}

