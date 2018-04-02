

// ----------------------------------- controller parameter --------------------------

// The controller mode can be either 0 (open_loop) or 1 (close_loop)
int ctrl_mode = 0;


// CAUTION: ctrl_loop_period is also accessed in interrupt
// CAUTION: ctrl_loop_period can not be changed at running time
// loop period is in ms
volatile long ctrl_loop_period = CTRL_LOOP_PERIOD;


// PID controller parameter

float prefilter_co = CTRL_LOOP_PREFILTER;

float kp_left = CTRL_LOOP_KP_LEFT;
float ki_left = CTRL_LOOP_KI_LEFT;
float kd_left = CTRL_LOOP_KD_LEFT;

float kp_right = CTRL_LOOP_KP_RIGHT;
float ki_right = CTRL_LOOP_KI_RIGHT;
float kd_right = CTRL_LOOP_KD_RIGHT;

float deadzone_threshold = 1.6;
float deadzone_saturation = 512;

float roll_off_co = CTRL_LOOP_ROLLOFF;

// ----------------------------------- control global variables --------------------------


long encoder_l = 0;
long encoder_r = 0;

// Current angular velocity
float wl = 0;
float wr = 0;

float wl_p = 0;
float wr_p = 0;

// Desired angular velocity
float wl_dsr = 0;
float wr_dsr = 0;

float wl_dsr_filtered = 0;
float wr_dsr_filtered = 0;

float wl_dsr_filtered_p = 0;
float wr_dsr_filtered_p = 0;

float err_wl = 0;
float err_wr = 0;

float err_wl_p = 0;
float err_wr_p = 0;

float err_wl_pp = 0;
float err_wr_pp = 0;

// controller output PWM
int pwml = 0;
int pwmr = 0;

// device output PWM
int pwml_out = 0;
int pwmr_out = 0;

int pwml_out_p = 0;
int pwmr_out_p = 0;

float pwml_up;
float pwml_ui;
float pwml_ud;

float pwmr_up;
float pwmr_ui;
float pwmr_ud;


// ----------------------------------- control utility function --------------------------

// PID controller
float ctrl_pid(float err, float err_sum, float err_p,
               float kp, float ki, float kd, 
               float *up_out, float *ui_out, float *ud_out,
               float ts) {

  float u = 0;
  float up = kp * err;
  float ui = ki * ts * err_sum;
  float ud = kd * (err - err_p) / ts;

  u = up + ui + ud;

  if(up_out != NULL)
    *up_out = up;
  if(ui_out != NULL)
    *ui_out = ui;
  if(ud_out != NULL)
    *ud_out = ud;

  return u;
}

// PID controller Incremental Style
int ctrl_pid_inc(int u_p, 
                 float err, float err_p, float err_pp,
                 float kp, float ki, float kd, 
                 float *up_out, float *ui_out, float *ud_out,
                 float ts) {

  float up = kp * (err - err_p);
  float ui = ki * ts * err;
  float ud = kd * ((err - err_p) - (err_p - err_pp)) / ts;

  float delta_u = (up + ui + ud) * PWM_D2A_FACTOR;
  
  int u = u_p + (int)delta_u;

  if(up_out != NULL)
    *up_out = up;
  if(ui_out != NULL)
    *ui_out = ui;
  if(ud_out != NULL)
    *ud_out = ud;

  return (int)u;
}


// If error is smaller than the threshold, then set error to 0
float ctrl_deadzone_threshold(float err_in) {

  float err_out = err_in;
  if (abs(err_in) < deadzone_threshold) {
    err_out = 0;
  }
  return err_out;
}

// Saturate the control variable change, i.e. we do not allow
// rapid change of PWM between interations.
int ctrl_deadzone_saturation(int u_in, int u_in_p) {

  int u_out = u_in;
  if (abs(u_in - u_in_p) > deadzone_saturation) {
    u_out = (int)(u_in + deadzone_saturation);
  }
  return u_out;
}


// roll off for controller output (plant (motor shield ) input)
// u_rf = (1-rf_coeff) u_rf_p + rf_coeff * u_in

float ctrl_output_rolloff (int u_rf_p, int u_in){
  float u_rf = (1 - roll_off_co)* u_rf_p + roll_off_co * u_in;
  return u_rf;
}

float ctrl_error_prefilter(float value, float value_p, float coefficient) {

  float value_filtered = (1 - coefficient) * value_p + coefficient * value;

  return value_filtered;
}









// ---------- control inner loop, i.e. (wL, wR) control ---------------


// compute angular velocity of each wheel through Encoder Measurement
void ctrl_get_current_wl_wr() {

  encoder_l = encoder_left_read();
  encoder_r = encoder_right_read();

  wl = encoder_calculate_angular_speed(encoder_l, ctrl_loop_period);
  wr = encoder_calculate_angular_speed(encoder_r, ctrl_loop_period);

  // average filter
  // wl = (wl + wl_p) / 2;
  // wr = (wr + wr_p) / 2;

  
  wl_p = wl;
  wr_p = wr;
  
  // reset encoder value, so that every time we get increment from last time
  encoder_left_write(0);
  encoder_right_write(0);

}

// Update desired angular velocity and use PID controller
// Thus control inner loop (angular velocity )

void ctrl_inner_loop() {

  if (ctrl_mode == 0) {

    // controller mode is open loop
    ctrl_set_pwm();
    return;
  }

  wl_dsr_filtered = (1 - prefilter_co) * wl_dsr_filtered_p
                    + prefilter_co * wl_dsr;
  wr_dsr_filtered = (1 - prefilter_co) * wr_dsr_filtered_p
                    + prefilter_co * wr_dsr;

  //calculate error between measured output and desire value
  err_wl = wl_dsr_filtered - wl;
  err_wr = wr_dsr_filtered - wr;


  // PID Inner loop Controller

  pwml_out = ctrl_pid_inc(pwml_out_p, 
                      err_wl, err_wl_p, err_wl_pp, 
                      kp_left, ki_left, kd_left,
                      &pwml_up, &pwml_ui, &pwml_ud,
                      (float)ctrl_loop_period / 1000.0);
  pwmr_out = ctrl_pid_inc(pwmr_out_p, 
                      err_wr, err_wr_p, err_wr_pp, 
                      kp_right, ki_right, kd_right, 
                      &pwmr_up, &pwmr_ui, &pwmr_ud,
                      (float)ctrl_loop_period / 1000.0);


  pwml = (int) ctrl_output_rolloff (pwml_out_p,pwml_out);
  pwmr = (int) ctrl_output_rolloff (pwmr_out_p,pwmr_out);


}

//Determine direction and control saturation
void ctrl_set_pwm() {

  int pwml_ctrl = pwml;
  int pwmr_ctrl = pwmr;
  int pwml_motor;
  int pwmr_motor;
  int right_dir = 0;
  int left_dir = 0;

  pwml_motor = pwml_ctrl;
  if (pwml_ctrl > PWM_MAX) {
    pwml_motor = PWM_MAX;
  }
  if (pwml_ctrl < PWM_MIN) {
    pwml_motor = PWM_MIN;
  }
  pwmr_motor = pwmr_ctrl;
  if (pwmr_ctrl > PWM_MAX) {
    pwmr_motor = PWM_MAX;
  }
  if (pwmr_ctrl < PWM_MIN) {
    pwmr_motor = PWM_MIN;
  }

  if (pwml_motor > 0) {
    left_dir = '+';
  } else if (pwml_motor < 0) {
    pwml_motor = -pwml_motor;
    left_dir = '-';
  } else {
    // note that set pwm as 0 is not release motor
    left_dir = '+';
  }

  if (pwmr_motor > 0) {
    right_dir = '+';
  } else if (pwmr_motor < 0) {
    pwmr_motor = -pwmr_motor;
    right_dir = '-';
  } else {
    right_dir = '+';
  }

  // call lower level interface
  m_set_pwm(pwml_motor, left_dir, pwmr_motor, right_dir);

}

void ctrl_inner_loop_update() {

  // Iteration
  pwml_out_p = pwml_out;
  pwmr_out_p = pwmr_out;
  err_wl_p = err_wl;
  err_wr_p = err_wr;
  err_wl_pp = err_wl_p;
  err_wr_pp = err_wr_p;
  wl_dsr_filtered_p = wl_dsr_filtered;
  wr_dsr_filtered_p = wr_dsr_filtered;


}


// ----------------------------------- control interface --------------------------

void ctrl_reset() {

  wl_dsr_filtered = 0;
  wr_dsr_filtered = 0;

  wl_dsr_filtered_p = 0;
  wr_dsr_filtered_p = 0;

  err_wl_p = 0;
  err_wr_p = 0;

  err_wl_pp = 0;
  err_wr_pp = 0;

  pwml_out_p = 0;
  pwmr_out_p = 0;
}

void ctrl_set_open_loop(int pwml_new, int pwmr_new) {

  pwml = pwml_new;
  pwmr = pwmr_new;

  ctrl_mode = 0;
}

void ctrl_set_wl_wr(float wl_dsr_new, float wr_dsr_new) {

  wl_dsr = wl_dsr_new;
  wr_dsr = wr_dsr_new;

  ctrl_mode = 1;
}




