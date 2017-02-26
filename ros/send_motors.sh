#!/bin/sh

rostopic pub -r 1 /motors std_msgs/Float32MultiArray "layout:
  dim: []
  data_offset: 0
data: [0.5, 0.0]"

