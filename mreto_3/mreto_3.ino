#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber_pwm;
rcl_publisher_t publisher_raw;
rcl_publisher_t publisher_volt;
rclc_executor_t executor_10;
rclc_executor_t executor_100;
rclc_executor_t executor_pwm;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_10;
rcl_timer_t timer_100;
std_msgs__msg__Float32 msg_raw;
std_msgs__msg__Float32 msg_volt;
std_msgs__msg__Float32 msg_pwm;

#define LED_PIN_1 12
#define LED_PIN_2 13
#define POT_PIN 15

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int val_pot = 0;
float voltage = 0;
int duty_cycle = 0;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN_1, !digitalRead(LED_PIN_1));
    delay(100);
  }
}

void timer_callback_10(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    val_pot = analogRead(POT_PIN);
    voltage = (float(val_pot)/4095.0)*3.3;
  }
}

void timer_callback_100(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher_raw, &msg_raw, NULL));
    msg_raw.data = val_pot;
    RCSOFTCHECK(rcl_publish(&publisher_volt, &msg_volt, NULL));
    msg_volt.data = voltage;
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if(msg->data <= 0){
    duty_cycle = 0;
  }else if(msg->data >= 100){
    duty_cycle = 100;
  }else{
    duty_cycle = msg->data;
  }
  analogWrite(LED_PIN_2, (float(duty_cycle)/100.0) * 255);  
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN_1, OUTPUT);
  digitalWrite(LED_PIN_1, HIGH);  
  pinMode(LED_PIN_2, OUTPUT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_duty_cycle"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_raw,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "raw_pot"));
  
  RCCHECK(rclc_publisher_init_default(
    &publisher_volt,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "voltage"));

  const unsigned int timer_timeout_10 = 10;
  
  RCCHECK(rclc_timer_init_default(
    &timer_10,
    &support,
    RCL_MS_TO_NS(timer_timeout_10),
    timer_callback_10));
  
  const unsigned int timer_timeout_100 = 100;
  
  RCCHECK(rclc_timer_init_default(
    &timer_100,
    &support,
    RCL_MS_TO_NS(timer_timeout_100),
    timer_callback_100));

  RCCHECK(rclc_executor_init(&executor_pwm, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_10, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_100, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_10, &timer_10));
  RCCHECK(rclc_executor_add_timer(&executor_100, &timer_100));
  RCCHECK(rclc_executor_add_subscription(&executor_pwm, &subscriber_pwm, &msg_pwm, &subscription_callback, ON_NEW_DATA));
  
  msg_raw.data = 0;
  msg_volt.data = 0;
  
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_10, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_100, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pwm, RCL_MS_TO_NS(100)));
}
