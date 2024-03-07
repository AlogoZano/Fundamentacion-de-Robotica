#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>

#define LED_PIN 13

#define LED_DEBUG 5

#define PWM_PIN 15
#define PWM_CH 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define REDUCTION 56
#define PULSES 17
#define CONV 60000000

#define INT_ENCODER 32

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t publisher_wn;
rcl_subscription_t subscriber_setpoint;

std_msgs__msg__Int16 msg_wn;
std_msgs__msg__Float32 msg_setpoint;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node_controller;

rcl_timer_t timer1;
rcl_timer_t timer2;

float pwm_value = 0.0;

unsigned long actual = 0;
unsigned long anterior = 0;
unsigned long tiempo = 0;
int16_t rpm = 0;

bool estado = 0;


void subscription_callback(const void * msgin){  
  const std_msgs__msg__Float32 * msg_setpoint = (const std_msgs__msg__Float32 *)msgin;
  ledcWrite(PWM_CH, msg_setpoint->data);
}


void IRAM_ATTR Read_Encoder() {
  digitalWrite(LED_DEBUG, !digitalRead(LED_DEBUG));
  actual = esp_timer_get_time();
  tiempo = actual-anterior;
  anterior = actual;
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void publish_wn_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    rpm = CONV/(tiempo*PULSES*REDUCTION);
    if(rpm > 65){
      rpm = 65;
    }else if(rpm < 6){
      rpm = 0;
    }
    msg_wn.data = rpm;
    RCSOFTCHECK(rcl_publish(&publisher_wn, &msg_wn, NULL));

  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(INT_ENCODER, INPUT);
  pinMode(LED_DEBUG, OUTPUT);

  attachInterrupt(INT_ENCODER, Read_Encoder, RISING);

  ledcAttachPin(PWM_PIN, PWM_CH);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RESOLUTION);

  set_microros_transports();
  
  
  digitalWrite(LED_PIN, HIGH);  
  
  
  delay(2000);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node_controller, "micro_ros_esp32_controller_node", "", &support));

  //Subscribers
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber_setpoint,
    &node_controller,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "setpoint"
  ));

  //Timers
    const unsigned int timer_period = 20;
    RCCHECK(rclc_timer_init_default(
      &timer1,
      &support,
      RCL_MS_TO_NS(timer_period),
      publish_wn_callback
    ));

  //Publishers
  
    RCCHECK(rclc_publisher_init_default(
    &publisher_wn,
    &node_controller,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "angular_speed"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_setpoint, &msg_setpoint, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
