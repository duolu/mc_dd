

// --------------- VC-TRUCK-V1 ----------------

#if defined(CONFIG_PLATFORM_VC_TRUCK_V1)

//LED
#define CONFIG_LED

#define LED_RED_PIN             6
#define LED_YELLOW_PIN          13

// POWER
#define CONFIG_POWER

#define POWER_VOLTAGE_PIN       0
#define MOTOR_VOLTAGE_PIN       1


// Motor
#define CONFIG_MOTOR_2WD_CYTRON

#define M_LEFT_PWM_PIN                  9
#define M_LEFT_DIR_PIN                  7
#define M_RIGHT_PWM_PIN                 10
#define M_RIGHT_DIR_PIN                 8

// Encoder
#define CONFIG_ENCODER_2WD

#define ENC_LEFT_PIN_A                  2
#define ENC_LEFT_PIN_B                  4
#define ENC_RIGHT_PIN_A                 3
#define ENC_RIGHT_PIN_B                 5

// Count Per Turn of Encoder 
#define ENCODER_CPT_GEARED              3592  // actually 3591.84

// IMU
#define CONFIG_IMU_BNO055

#define CONFIG_ULTRASONIC_HC_SR04

#define ULTRASONIC_LEFT_TRIGGER_PIN     22
#define ULTRASONIC_LEFT_ECHO_PIN        23

#define ULTRASONIC_RIGHT_TRIGGER_PIN    24
#define ULTRASONIC_RIGHT_ECHO_PIN       25

#define ULTRASONIC_MAX_DISTANCE   300

// Control
#define PWM_MAX                 255
#define PWM_MIN                 -255

#define CTRL_LOOP_PREFILTER     0.26

#define CTRL_LOOP_KP_LEFT       1.1
#define CTRL_LOOP_KI_LEFT       7.7
#define CTRL_LOOP_KD_LEFT       0

#define CTRL_LOOP_KP_RIGHT      1.1
#define CTRL_LOOP_KI_RIGHT      7.7
#define CTRL_LOOP_KD_RIGHT      0

#define CTRL_LOOP_ROLLOFF       0.83


#define PWM_D2A_FACTOR          42.5 // 6V motor

#define CTRL_LOOP_PERIOD        50 // 20 Hz inner loop

#endif // define (CONFIG_PLATFORM_VC_TRUCK_V1)

// --------------- BOX ----------------

#if defined (CONFIG_PLATFORM_VC_BOX_V1)

//LED
#define CONFIG_LED

#define LED_RED_PIN             6
#define LED_YELLOW_PIN          13

// POWER
#define CONFIG_POWER

#define POWER_VOLTAGE_PIN       0
#define MOTOR_VOLTAGE_PIN       1


// Motor
#define CONFIG_MOTOR_2WD_CYTRON

#define M_LEFT_PWM_PIN  9
#define M_LEFT_DIR_PIN  7
#define M_RIGHT_PWM_PIN  10
#define M_RIGHT_DIR_PIN  8

// Encoder
#define CONFIG_ENCODER_2WD

#define ENC_LEFT_PIN_A  2
#define ENC_LEFT_PIN_B  4
#define ENC_RIGHT_PIN_A  3
#define ENC_RIGHT_PIN_B  5

// Count Per Turn of Encoder
#define ENCODER_CPT_GEARED    3592

// IMU
#define CONFIG_IMU_BNO055

// TODO: fix ultrasonic sensor code

#define CONFIG_ULTRASONIC_HC_SR04

#define ULTRASONIC_LEFT_TRIGGER_PIN    22
#define ULTRASONIC_LEFT_ECHO_PIN       23

#define ULTRASONIC_RIGHT_TRIGGER_PIN    24
#define ULTRASONIC_RIGHT_ECHO_PIN       25

#define ULTRASONIC_MAX_DISTANCE   300

// Control
#define PWM_MAX                 255
#define PWM_MIN                 -255

#define CTRL_LOOP_PREFILTER     1

#define CTRL_LOOP_KP_LEFT       0.098
#define CTRL_LOOP_KI_LEFT       4.50
#define CTRL_LOOP_KD_LEFT       0

#define CTRL_LOOP_KP_RIGHT      0.098
#define CTRL_LOOP_KI_RIGHT      4.50
#define CTRL_LOOP_KD_RIGHT      0

#define CTRL_LOOP_ROLLOFF       1


#define PWM_D2A_FACTOR          42.5

#define CTRL_LOOP_PERIOD        25

#endif // defined (CONFIG_PLATFORM_VC-BOX_V1)


