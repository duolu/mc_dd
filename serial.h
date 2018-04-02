

// ------------------------------------- serial protocol -------------------------------

#define SERIAL_STATE_INIT      0
#define SERIAL_STATE_MAGIC1    1
#define SERIAL_STATE_MAGIC2    2
#define SERIAL_STATE_PROTO     3

#define SERIAL_MAGIC_1 'A'
#define SERIAL_MAGIC_2 'F'

// command sent from HLC to LLC

#define OPCODE_OPEN_LOOP                  0x00

#define OPCODE_CTRL_WL_WR                 0x10

//#define OPCODE_PAN_TILT                  0x20



#define OPCODE_SETUP          0xF0
#define OPCODE_START          0xF1
#define OPCODE_STOP               0xF2
#define OPCODE_DEBUG_ENABLE       0xF3
#define OPCODE_DEBUG_DISABLE      0xF4


// state report from LLC to HLC

#define OPCODE_VEHICLE_STATUS        0xE0
#define OPCODE_CTRL_STATUS_DEBUG    0xE1


void serial_parse_command(int opcode);
void serial_state_machine_proceed(int c);
void serial_send_header(byte len, byte opcode);



