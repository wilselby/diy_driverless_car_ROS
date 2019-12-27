#!/bin/bash

rosbag record --split --size 1024 --lz4 -o ml_training /imu/data /joy_teleop/cmd_vel_stamped /openmv_cam/image/raw /os1_node/imu_packets /os1_node/lidar_packets /racecar/ackermann_cmd_mux/output /rover_velocity_controller/cmd_vel /tf /tf_static /img_node/intensity_image /img_node/noise_image /img_node/range_image


