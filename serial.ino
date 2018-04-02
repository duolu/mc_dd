// ------------ serial utility function -------

// CAUTION: this is blocking write!!!

void serial_put_char(byte c) {

  while (Serial.write(c) <= 0)
    ;
}

void serial_send_header(byte len, byte opcode) {

  serial_put_char(SERIAL_MAGIC_1);
  serial_put_char(SERIAL_MAGIC_2);
  serial_put_char(len);
  serial_put_char(opcode);
  Serial.flush();
}

void serial_send_int(int i) {

  byte *cp = (byte *) &i;

  byte c0 = cp[0];
  byte c1 = cp[1];

  serial_put_char(c0);
  serial_put_char(c1);
  Serial.flush();
}

void serial_send_long(long l) {

  byte *cp = (byte *) &l;

  //  byte c0 = (byte)(l & 0x000000FF);
  //  byte c1 = (byte)((l & 0x0000FF00) >> 8);
  //  byte c2 = (byte)((l & 0x00FF0000) >> 16);
  //  byte c3 = (byte)((l & 0xFF000000) >> 24);

  byte c0 = cp[0];
  byte c1 = cp[1];
  byte c2 = cp[2];
  byte c3 = cp[3];

  serial_put_char(c0);
  serial_put_char(c1);
  serial_put_char(c2);
  serial_put_char(c3);
  Serial.flush();
}

void serial_send_float(float f) {

  byte *cp = (byte *) &f;

  byte c0 = cp[0];
  byte c1 = cp[1];
  byte c2 = cp[2];
  byte c3 = cp[3];

  serial_put_char(c0);
  serial_put_char(c1);
  serial_put_char(c2);
  serial_put_char(c3);
  Serial.flush();
}

// CAUTION: this is blocking read!!!

int serial_get_char() {

  while (Serial.available() <= 0)
    ;
  return (char) Serial.read();

}
int serial_get_int() {

  char bytearray[2];

  bytearray[0] = serial_get_char();
  bytearray[1] = serial_get_char();

  return *((int *) bytearray);
}

long serial_get_long() {

  char bytearray[4];

  bytearray[0] = serial_get_char();
  bytearray[1] = serial_get_char();
  bytearray[2] = serial_get_char();
  bytearray[3] = serial_get_char();

  return *((long *) bytearray);
}

float serial_get_float() {

  char bytearray[4];

  bytearray[0] = serial_get_char();
  bytearray[1] = serial_get_char();
  bytearray[2] = serial_get_char();
  bytearray[3] = serial_get_char();

  return *((float *) bytearray);
}


int serial_state = SERIAL_STATE_INIT;
int serial_length = 0;

// -----------    dispatch serial message according to the opcode --------------


void serial_parse_command(int opcode) {

  switch (opcode) {

    case OPCODE_OPEN_LOOP: {

        // If direction variable is ‘+’, then it means rotating forward;
        // If direction variable is ‘-’, then it means rotating backward;
        // All other direction value is stop
        int pwm_left_dsr = serial_get_int();
        int pwm_right_dsr = serial_get_int();
        if (motor_power_state == 0 || (motor_power_state == 1 && ctrl_mode == 0))
          ctrl_set_open_loop(pwm_left_dsr, pwm_right_dsr);

        break;
      }

    case OPCODE_CTRL_WL_WR: {

        float wl_dsr_new = serial_get_float();
        float wr_dsr_new = serial_get_float();

        if (motor_power_state == 0 || (motor_power_state == 1 && ctrl_mode == 1))
          ctrl_set_wl_wr(wl_dsr_new, wr_dsr_new);
        break;
      }

    case OPCODE_SETUP: {

        // Setup only works if the system is stopped.
        if (motor_power_state == 0) {

          // The order must be exatly the same as h2l setup funciton 
          prefilter_co = serial_get_float();
          roll_off_co = serial_get_float();
          kp_left = serial_get_float();
          ki_left = serial_get_float();
          kd_left = serial_get_float();
          kp_right = serial_get_float();
          ki_right = serial_get_float();
          kd_right = serial_get_float();
          deadzone_threshold = serial_get_float();
          deadzone_saturation = serial_get_float();
          
        }

        break;
      }
    case OPCODE_START: {

        motor_power_state = 1;

        led_yellow_on();
         
        break;
      }
    case OPCODE_STOP: {

        motor_power_state = 0;
        m_stop();
        ctrl_reset();

        led_yellow_off();

        break;
      }
//    case OPCODE_DEBUG_ENABLE: {
//
//        debug_mode = 1;
//
//        break;
//      }
//    case OPCODE_DEBUG_DISABLE: {
//
//        debug_mode = 0;
//
//        break;
//      }

//    case OPCODE_XXX: {
//
//        // add new function here
//        // use these comments as template
//        break;
//      }
    default: {

        break;
      }

  }

}

// serial state machine
void serial_state_machine_proceed(int c) {

  //  Serial.print(c, HEX);
  //  Serial.print(' ');

  switch (serial_state) {
    case SERIAL_STATE_INIT: {
        if (c == SERIAL_MAGIC_1)
          serial_state = SERIAL_STATE_MAGIC1;
        else
          serial_state = SERIAL_STATE_INIT;
        break;
      }
    case SERIAL_STATE_MAGIC1: {
        if (c == SERIAL_MAGIC_2)
          serial_state = SERIAL_STATE_MAGIC2;
        else
          serial_state = SERIAL_STATE_INIT;
        break;
      }
    case SERIAL_STATE_MAGIC2: {
        serial_length = c;
        serial_state = SERIAL_STATE_PROTO;
        break;
      }
    case SERIAL_STATE_PROTO: {

        // opcode = c
        serial_parse_command(c);

        serial_state = SERIAL_STATE_INIT;
        break;
      }
    default: {
        serial_state = SERIAL_STATE_INIT;
        break;
      }

  }

}

// ---------- serial sending -----------

void serial_send_vehicle_status(long timestamp) {

  // CAUTION: modify the packet length if new field is added.
  // The first argument of send header is the total sum of payload below
  serial_send_header(52, OPCODE_VEHICLE_STATUS);

  serial_send_long(timestamp);
  serial_send_float(imu_abs_theta);
  serial_send_float(imu_rel_theta);
  serial_send_float(imu_accx);
  serial_send_float(imu_omega);
  serial_send_float(wl);
  serial_send_float(wr);
  serial_send_float(encoder_l);
  serial_send_float(encoder_r);
  serial_send_float(ultrasonic_left_delta_x);
  serial_send_float(ultrasonic_right_delta_x);
  serial_send_float(power_voltage);
  serial_send_float(motor_voltage);
}


void serial_send_ctrl_status_debug(long timestamp) {

  int i;

  serial_send_header(112, OPCODE_CTRL_STATUS_DEBUG);

  serial_send_long(timestamp);

  serial_send_float(wl_dsr);
  serial_send_float(wr_dsr);
  serial_send_float(wl_dsr_filtered);
  serial_send_float(wr_dsr_filtered);

  serial_send_int(pwml);
  serial_send_int(pwmr);
  serial_send_int(pwml_out);
  serial_send_int(pwmr_out);
  serial_send_int(pwml_out_p);
  serial_send_int(pwmr_out_p);

  serial_send_float(err_wl);
  serial_send_float(err_wl_p);
  serial_send_float(err_wl_pp);
  serial_send_float(err_wr);
  serial_send_float(err_wr_p);
  serial_send_float(err_wr_pp);

  serial_send_float(pwml_up);
  serial_send_float(pwml_ui);
  serial_send_float(pwml_ud);
  serial_send_float(pwmr_up);
  serial_send_float(pwmr_ui);
  serial_send_float(pwmr_ud);

  for(i = 0; i < 8; i++) {

    serial_send_long(timestamps[i]);
    
  }
}


