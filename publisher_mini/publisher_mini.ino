#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>

rcl_publisher_t publisher_raw_pot;
rcl_publisher_t publisher_voltage;

std_msgs__msg__Int16 msg_rawPot;
std_msgs__msg__Float32 msg_Voltage;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;

rcl_timer_t timer1;
rcl_timer_t timer2;

#define LED_PIN 13
#define ADC_PIN 34

int16_t adc_value = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void read_adc_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    adc_value = analogRead(ADC_PIN);
    
  }
 
}

void publish_adc_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    msg_rawPot.data = adc_value;
    msg_Voltage.data = (adc_value * 3.3)/4095;
    RCSOFTCHECK(rcl_publish(&publisher_raw_pot, &msg_rawPot, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_voltage, &msg_Voltage, NULL));
    }
}

void setup() {

  pinMode(ADC_PIN,INPUT);
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_raw_pot,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "raw_pot"));
  RCCHECK(rclc_publisher_init_default(
    &publisher_voltage,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "voltage"));
    

  // create timer,
  const unsigned int timer_1 = 10;
  RCCHECK(rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(timer_1),
    read_adc_callback));

  const unsigned int timer_2 = 100;
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer_2),
    publish_adc_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));

  //msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
