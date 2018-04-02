

// hardware abstraction layer
void m_set_pwm(int v_left_new, int v_left_direction, int v_right_new,
               int v_right_direction);
void m_stop();

long encoder_left_read();
long encoder_right_read();
long encoder_left_write();
long encoder_right_write();
