
#include "Arduino.h"

// #define EXTERNAL_WIFI
// #define WIFI_SSID "FAST_347C"
// #define WIFI_PASSWORD "405405405"
// #define LOCAL_IP "192.168.1.201" // Force ROBOT IP LOCAL server port 2222
// #define TARGET_IP "192.168.1.101" // TARGET_IP server port 2223

#define LOCAL_UDP_PORT "8080"
#define TARGET_IP "192.168.4.2"

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 80     //140
#define MAX_TARGET_ANGLE 16 //14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780    // Max recommended value: 860
#define MAX_STEERING_PRO 260    // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26 // Max recommended value: 32

#define TELEMETRY_BATTERY 1
// #define TELEMETRY_ANGLE 1
// #define TELEMETRY_DEBUG 0

// 默认控制参数
// #define KP 0.32
// #define KD 0.050
// #define KP_THROTTLE 0.080
// #define KI_THROTTLE 0.1
// #define KP_POSITION 0.06
// #define KD_POSITION 0.45

#define KP 0.32          //平衡环PD控制器  0.52 0.08 80可以
#define KD 0.05          //平衡环PD控制器
#define KP_THROTTLE 0.08 //速度环PI控制器
#define KI_THROTTLE 0.1  //速度环PI控制器
#define KP_POSITION 0.06 //电机脉冲(电流环)PD控制器
#define KD_POSITION 0.45 //电机脉冲(电流环)PD控制器

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.16
#define KP_THROTTLE_RAISEUP 0 // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 3000 //30 // Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0 // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500 // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 500
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

// Telemetry
//#define TELEMETRY_BATTERY 1
//#define TELEMETRY_ANGLE 1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define ZERO_SPEED 65535
#define MAX_ACCEL 20 // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 16 // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

// AUX definitions
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

String MAC; // MAC address of Wifi module
uint8_t macAddr[6];
char macAddrChar[6];

uint16_t cascade_control_loop_counter = 0;
uint16_t loop_counter;        // To generate a medium loop 40Hz
uint16_t slow_loop_counter;   // slow loop 2Hz
uint16_t sendBattery_counter; // To send battery status
int16_t BatteryValue;

long long timer_old;
long long timer_value;
float debugVariable;
double dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered = 0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;

boolean positionControlMode = false;
uint8_t mode; // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1, steps2;
//volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2; // Actual speed of motors
int8_t dir_M1, dir_M2;      // Actual direction of steppers motors
int16_t actual_robot_speed; // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered; // Estimated robot speed

// OSC output variables
uint8_t OSCpage;
uint8_t OSCnewMessage;
float OSCfader[4];
float OSCxy1_x;
float OSCxy1_y;
float OSCxy2_x;
float OSCxy2_y;
uint8_t OSCpush[4];
uint8_t OSCtoggle[4];
uint8_t OSCmove_mode;
int16_t OSCmove_speed;
int16_t OSCmove_steps1;
int16_t OSCmove_steps2;

int packetSize = 0;
