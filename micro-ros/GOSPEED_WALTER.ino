#include <ArloRobot.h>
#include <SoftwareSerial.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int16.h>



#define RX_PIN 16  
#define TX_PIN 17

// ---------------- ROS entidades ----------------
rcl_subscription_t cmd_subscriber;
rcl_publisher_t left_wheel_publisher;
rcl_publisher_t right_wheel_publisher;
rcl_timer_t timer;

// --- NUEVO: publisher de prueba (heartbeat) ---
rcl_publisher_t heartbeat_publisher;
rcl_timer_t heartbeat_timer;
std_msgs__msg__Int16 heartbeat_msg;

geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Int16 left_wheel_msg;
std_msgs__msg__Int16 right_wheel_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

int motorSpeedleft = 0;
int motorSpeedRight = 0;

int LeftEncoderCount = 0;
int RightEncoderCount = 0;

const float K_LIN = 180.0f;
const float K_ANG = 150.0f;

void LeftEncoderCB();
void RightEncoderCB();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

ArloRobot Arlo; 
EspSoftwareSerial::UART myPort; //Comunicacion del ArloRobot por serial 

// ---------------- Util ----------------
void error_loop() {
  while (1) { delay(10); }
}

// ---------------- Callbacks ----------------
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
  (void)msg; // por ahora no usamos el cmd_vel
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&left_wheel_publisher, &left_wheel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_wheel_publisher, &right_wheel_msg, NULL));
    //right_encoder_msg.data = RightEncoderCount;
    //left_encoder_msg.data  = -LeftEncoderCount;
  }
}

// --- NUEVO: callback del heartbeat ---
void heartbeat_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  static int16_t counter = 0;
  if (timer != NULL) {
    heartbeat_msg.data = counter++;
    RCSOFTCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));
  }
}


// Si luego quieres controlar velocidades desde /cmd_vel:
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  float left  =  (K_LIN * linear) - (K_ANG * angular);
  float right =  (K_LIN * linear) + (K_ANG * angular);

  motorSpeedleft = (int)left;
  motorSpeedRight = (int)right;
  
  Arlo.writeSpeeds(motorSpeedleft, motorSpeedRight);
  delay(10);
  ESP.restart();
  //Arlo.clearCounts();
}

void setup() {
  delay(10);

  // UART por software para el DHB-10 / Arlo
  myPort.begin(19200, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN, TX_PIN);
  Arlo.begin(myPort);
  Arlo.clearCounts();

  // Transportes micro-ROS por WiFi
  // SSID, PASS, AGENT_IP, AGENT_PORT
  set_microros_wifi_transports("covibot", "covibot1", "192.168.1.9", 8888);

  // ---- Inicialización micro-ROS mínima (NODO + PUBLISHER + TIMER) ----
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Nodo
  RCCHECK(rclc_node_init_default(&node, "gospeed_walter_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));

  // Publishers: encoders (si ya los usarás luego)
  RCCHECK(rclc_publisher_init_default(
      &left_wheel_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "left_wheel"));
  RCCHECK(rclc_publisher_init_default(
      &right_wheel_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "right_wheel"));

  // --- NUEVO: publisher de prueba (heartbeat) ---
  RCCHECK(rclc_publisher_init_default(
      &heartbeat_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "walter_heartbeat"));

  // Timers
  // Timer de encoders (ej. 100 ms)
  RCCHECK(rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(100), timer_callback));

  // --- NUEVO: timer heartbeat (500 ms) ---
  RCCHECK(rclc_timer_init_default(
      &heartbeat_timer, &support, RCL_MS_TO_NS(500), heartbeat_timer_callback));

  // Executor (2 timers por ahora; ajusta si agregas más handles)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_vel_msg,&cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  // Haz girar el executor para que se publiquen los mensajes
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  //Arlo.clearCounts();
  // --- Tu lógica actual de prueba con Arlo ---
  //Arlo.writeSpeeds(144, -144);
  //delay(3000);
  //Arlo.clearCounts(); // Clear encoder counts
  //Arlo.writeSpeeds(0, 0);
  //delay(1000);
}