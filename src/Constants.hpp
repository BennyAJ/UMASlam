#ifndef __SLAM_CONSTANTS_HPP__
#define __SLAM_CONSTANTS_HPP__

#include <cmath> //for M_PI

// Input Sensor Channels
#define GPS_CHANNEL "SENSOR_GPS"
#define FOG_CHANNEL "SENSOR_FOG"
#define COMPASS_CHANNEL "SENSOR_COMPASS"
#define IMU_CHANNEL "SENSOR_IMU"

// Output Channels
#define SLAM_STATE_CHANNEL "SLAM_STATE"
#define SLAM_PARTICLE_CHANNEL "SLAM_PARTICLES"
#define TRANSFORMED_IMU_CHANNEL "SENSOR_IMU_TRANSFORMED"

// Simulator Only Channels
#define PERFECT_GPS_CHANNEL "SENSOR_GPS_PERFECT"
#define PERFECT_IMU_CHANNEL "SENSOR_IMU_PERFECT"
#define PERFECT_FOG_CHANNEL "SENSOR_FOG_PERFECT"
#define PERFECT_SLAM_STATE_CHANNEL "SLAM_STATE_PERFECT"

//GPS related localization coefficients
//unit is meters
//Data sheet says less than 3 meters 95% of the time
//So standard deviation must be 1.5 if GPS output is appx. normal
const static double DEFAULT_GPS_SIGMA = 1.5;

//FOG related localization coefficients
//unit is radians
//Data sheet value 0.5
const static double DEFAULT_FOG_SIGMA = 0.5*M_PI/180.0;

//IMU Related Constants
#define USE_IMU_PREDICTION 0 

//Simulation Constants
//When simulator is on, we always initialize north to 0 degrees
#define USING_SIMULATOR true

//Fake Compass Constants
#define ORIGIN_DIST_BEFORE_REINITIALIZATION 3

//Selects the NUM_AVERAGE_PARTICLES best matched particles, then averages them
const static int NUM_AVERAGE_PARTICLES = 1;
const static int NUM_PARTICLES = 1000;

//Localization coefficients related to predicting particles forward with gps
const static double PERCENT_PREDICTION_PARTICLES = 0;
const static double X_PREDICTION_SIGMA = 0.1;
const static double Y_PREDICTION_SIGMA = 0.1;

//Localization constants that relate the relative beliefs in the various sensors.
//Magnitudes are irrelevant as long as the ratios are kept consistent
const static double GPS_LIKELIHOOD_COEFFICIENT = 1.0;
const static double FOG_LIKELIHOOD_COEFFICIENT = 1.0;

//Map scale
const static double SQUARE_SIZE = 0.5;

#endif
