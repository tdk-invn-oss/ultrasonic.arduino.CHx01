/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/*
  MicroROS_Publisher
  
  This example aims to use MicroROS Arduino environment.
  Arduino library: micro_ros_arduino
  It publishes UltraSound range from CH201 to microROS
  Agent.
*/
#include <micro_ros_arduino.h>

#include "CH201.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/range.h>

rcl_publisher_t publisher;
sensor_msgs__msg__Range range_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1);}}

// Hardware config : INT1 on pin 2, INT_DIR on 8, Reset on 9 (not inverted) and Prog on 10
CH201 CHx01(Wire, 2, 8, 9 , 10, false);

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

void setup() {
  set_microros_transports();
  delay(2000);

  // Initializing the CHx01
  RCCHECK(CHx01.begin());

  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "Range_node", "", &support));
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "Range_publisher"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    
  // Start CHx01 in free run mode
  CHx01.free_run();
  // Sensor is generally used with 45 degree horn, but it can go up to 180 degrees.
  range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view = 23;
  range_msg.min_range = 0.005;
  range_msg.max_range = ((float)CHx01.get_measure_range())/1000;
  range_msg.variance = 0;
}

void loop() {
  if(CHx01.data_ready()) {
    float range;
    struct timespec tv = {0};
    clock_gettime(0, &tv);
    /* Get range and convert it from mm to m*/
    range = CHx01.get_range() / 1000;

    range_msg.header.stamp.nanosec = tv.tv_nsec;
    range_msg.header.stamp.sec = tv.tv_sec;
    
    if (range == 0)
    {
      range_msg.range = +INFINITY;
      rcl_publish(&publisher, &range_msg, NULL);
    } else if ((range > range_msg.min_range)&&(range < range_msg.max_range))
    {
      range_msg.range = range;
      rcl_publish(&publisher, &range_msg, NULL);
    } else {
      range_msg.range = -INFINITY;
      rcl_publish(&publisher, &range_msg, NULL);
    }
  }
}
