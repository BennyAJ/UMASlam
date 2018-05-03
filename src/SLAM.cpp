#include "SLAM.hpp"
#include "Constants.hpp"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

Slam::Slam() 
{
  llcm.subscribe(GPS_CHANNEL, &Slam::handleGPSData, this);
  llcm.subscribe(TRANSFORMED_IMU_CHANNEL, &Slam::handleIMUData, this);
}

void Slam::handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const gps_t * gps_data)
{
  localizer.handleGPSData(rbuf, chan, gps_data);
}

void Slam::handleIMUData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const imu_t * imu_data)
{
  localizer.handleIMUData(rbuf, chan, imu_data);
}

void Slam::run()
{
  while(true) {
    llcm.handle();
  }
}
