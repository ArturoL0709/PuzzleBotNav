// ======== Include necessary Micro-ROS and ESP32 libraries ========
#include <micro_ros_arduino.h>           // Micro-ROS library for ESP32
#include <rcl/rcl.h>                     // ROS 2 client library (Core)
#include <rcl/error_handling.h>          // ROS 2 error handling utilities
#include <rclc/rclc.h>                   // Micro-ROS client library
#include <rclc/executor.h>               // Executor to handle callbacks
#include <std_msgs/msg/float32.h>        // Float message type for PWM control
#include <std_msgs/msg/string.h>         // String message type for feedback
#include <rmw_microros/rmw_microros.h>   // Middleware functions for Micro-ROS


// ======== Micro-ROS Entity Declarations ========
rclc_support_t support;       // Micro-ROS execution context
rclc_executor_t executor;     // Manages execution of tasks (timers, subscribers)
rcl_allocator_t allocator;    // Handles memory allocation


rcl_node_t node;              // Defines the ROS 2 node
rcl_subscription_t pwm_subscriber;   // Subscribes to PWM control
rcl_publisher_t feedback_publisher;  // Publisher for motor feedback
rcl_publisher_t pwm_value_publisher;


std_msgs__msg__Float32 pwm_msg;      // Holds the PWM value (-1.0 to 1.0)
std_msgs__msg__String feedback_msg;  // Message for feedback
std_msgs__msg__Float32 pwm_value_msg;
char feedback_buffer[100];           // Buffer for feedback message


// ======== Macro Definitions ========
// Error handling macros
// Executes fn and returns false if it fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
 static volatile int64_t init = -1; \
 if (init == -1) { init = uxr_millis();} \
 if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


// ======== Hardware Pin Definitions ========
// For Hackerboard according to challenge requirements
#define M1_IN1 18     // Direction pin 1
#define M1_IN2 15     // Direction pin 2
#define M1_PWM 4      // PWM pin


// PWM Configuration
#define PWM_CHANNEL 0  // PWM channel (0-15)
#define PWM_FREQ 980   // PWM frequency (Hz)
#define PWM_RES 8      // PWM resolution (bits) - 0 to 255
#define PWM_MAX_VAL 255 // Maximum PWM value with 8-bit resolution


// ======== Motor Control Variables ========
float pwm_value = 0.0;  // Current PWM value from -1.0 to 1.0


// ======== Micro-ROS Connection State Machine ========
enum states {
 WAITING_AGENT,        // Waiting for ROS 2 agent connection
 AGENT_AVAILABLE,      // Agent detected
 AGENT_CONNECTED,      // Successfully connected
 AGENT_DISCONNECTED    // Connection lost
} state;


// ======== Function Prototypes ========
bool create_entities();
void destroy_entities();
void motor_control(float pwm_value);


// ======== Subscriber Callback: Adjusts Motor PWM ========
void subscription_callback(const void * msgin) { 
 const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
 // Constrain the value between -1.0 and 1.0
 pwm_value = (msg->data < -1.0) ? -1.0 : ((msg->data > 1.0) ? 1.0 : msg->data);
  // Apply the motor control with the new PWM value
 motor_control(pwm_value);
 pwm_value_msg.data = pwm_value;
 RCSOFTCHECK(rcl_publish(&pwm_value_publisher, &pwm_value_msg, NULL));
  // Prepare feedback message
 char buf[50];
 snprintf(buf, sizeof(buf), "Motor PWM: %.2f, Duty: %d",
          pwm_value, (int)(abs(pwm_value) * PWM_MAX_VAL));
 feedback_msg.data.data = buf;
 feedback_msg.data.size = strlen(buf);
  // Publish feedback
 RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
}


// ======== Motor Control Function ========
// Sets direction and PWM for the motor based on input in range [-1, 1]
void motor_control(float value) {
 // Determine direction based on sign
 if (value > 0) {
   // Forward direction
   digitalWrite(M1_IN1, HIGH);
   digitalWrite(M1_IN2, LOW);
 } else if (value < 0) {
   // Reverse direction
   digitalWrite(M1_IN1, LOW);
   digitalWrite(M1_IN2, HIGH);
 } else {
   // Stop motor
   digitalWrite(M1_IN1, LOW);
   digitalWrite(M1_IN2, LOW);
 }
  // Set PWM value (take absolute value to get magnitude)
 uint32_t pwm_duty = (uint32_t)(abs(value) * PWM_MAX_VAL);
 ledcWrite(PWM_CHANNEL, pwm_duty);
}


// ======== Setup Function ========
void setup() {
 // Initialize serial for debugging (optional)
 Serial.begin(115200);
 delay(2000);
  set_microros_transports();  // Initialize Micro-ROS communication
  // Setup motor control pins
 pinMode(M1_IN1, OUTPUT);
 pinMode(M1_IN2, OUTPUT);
  // Setup PWM for motor control
 ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
 ledcAttachPin(M1_PWM, PWM_CHANNEL);
  // Initialize motor to stopped state
 digitalWrite(M1_IN1, LOW);
 digitalWrite(M1_IN2, LOW);
 ledcWrite(PWM_CHANNEL, 0);
  // Prepare memory for feedback message
 feedback_msg.data.capacity = sizeof(feedback_buffer);
 feedback_msg.data.data = feedback_buffer;
 feedback_msg.data.size = 0;
  // Initial state is waiting for agent
 state = WAITING_AGENT;
 
}


// ======== Main Loop Function ========
void loop() {
 switch (state) {
   case WAITING_AGENT:
     EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
     break;


   case AGENT_AVAILABLE:
     state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
     if (state == WAITING_AGENT) {
       destroy_entities();
     }
     break;


   case AGENT_CONNECTED:
     EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
     if (state == AGENT_CONNECTED) {
       rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
     }
     break;


   case AGENT_DISCONNECTED:
     destroy_entities();
     state = WAITING_AGENT;
     break;
    
   default:
     break;
 }
}


// ======== ROS 2 Entity Creation and Cleanup Functions ========
bool create_entities()
{
 // Initialize Micro-ROS
 allocator = rcl_get_default_allocator();
 RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
 RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));


 // Initialize PWM Subscriber
 RCCHECK(rclc_subscription_init_default(
     &pwm_subscriber,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
     "/cmd_pwm"));
    
 // Initialize Feedback Publisher
 RCCHECK(rclc_publisher_init_default(
     &feedback_publisher,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
     "/motor_feedback"));


 // Initialize Executor
 executor = rclc_executor_get_zero_initialized_executor();
 RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
 RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA));


 return true;
}


void destroy_entities()
{
 rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
 (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);


 rcl_subscription_fini(&pwm_subscriber, &node);
 rcl_publisher_fini(&feedback_publisher, &node);
 rclc_executor_fini(&executor);
 rcl_node_fini(&node);
 rclc_support_fini(&support);
}
