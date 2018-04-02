



extern volatile long ctrl_loop_period;


// control interface
void ctrl_reset();
void ctrl_set_open_loop();
void ctrl_set_wl_wr(float wl_dsr_new, float wr_dsr_new);



