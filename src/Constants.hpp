#ifndef __SLAM_CONSTANTS_HPP__
#define __SLAM_CONSTANTS_HPP__

// Input Sensor Channels
#define GPS_CHANNEL "SENSOR_GPS"
#define IMU_CHANNEL "SENSOR_IMU"

// Simulator Input Channels
#define PERFECT_IMU_CHANNEL "SENSOR_IMU_PERFECT"
#define SIM_POSE_CHANNEL "SIM_ABSOLUTE_POSE"

// Preprocessing Output Channels
#define TRANSFORMED_IMU_CHANNEL "SENSOR_IMU_TRANSFORMED"

// Output Channels
#define SLAM_STATE_CHANNEL "SLAM_STATE"
#define GPS_STATE_CHANNEL "SLAM_GPS_STATE"
#define IMU_STATE_CHANNEL "SLAM_IMU_STATE"

//Map scale
const static double SQUARE_SIZE = 0.5;

#endif
