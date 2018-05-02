#include "SLAM.hpp"
#include "Constants.hpp"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

Slam::Slam() : 
         localizer(NUM_PARTICLES, PERCENT_PREDICTION_PARTICLES),
         num_mapped_scans(0),
         compass_initialized(false),
         compass_north(0),
         imu_north(0)
{
  llcm.subscribe(GPS_CHANNEL, &Slam::handleGPSData, this);
  llcm.subscribe(FOG_CHANNEL, &Slam::handleFOGData, this);
  llcm.subscribe(COMPASS_CHANNEL, &Slam::handleCompassData, this);
  llcm.subscribe(TRANSFORMED_IMU_CHANNEL, &Slam::handleIMUData, this);
}

void Slam::handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const gps_t * gps_data)
{
  localizer.handleGPSData(rbuf, chan, gps_data);

  //reinitialization of the fog
  if(!fog_initialized)
  {
    fake_compass.addGPS(*gps_data);
    if(USING_SIMULATOR) {
      fog_initialized = true;
      localizer.reset();
      localizer.reinitializeFOG(0);
    }
    else if(compass_initialized) {
      fog_initialized = true;
      localizer.reset();
      localizer.reinitializeFOG(compass_north);
    }
    else if(imu_initialized) {
      fog_initialized = true;
      localizer.reset();
      localizer.reinitializeFOG(imu_north);
    }
    else if(fake_compass.getDistFromOrigin() > ORIGIN_DIST_BEFORE_REINITIALIZATION) {
      fog_initialized = true;
      localizer.reset();
      localizer.reinitializeFOG(fake_compass.getNorthLocation(localizer.getFogInitialization()));
    }
  }
}

void Slam::handleFOGData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const fog_t * fog_data)
{
  localizer.handleFOGData(rbuf, chan, fog_data);
}

void Slam::handleCompassData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const compass_t * compass_data)
{
  compass_initialized = true;
  compass_north = compass_data->yaw;
}

void Slam::handleIMUData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const imu_t * imu_data)
{
  imu_initialized = true;
  imu_north = imu_data->yaw;

  localizer.handleIMUData(rbuf, chan, imu_data);
}

void Slam::run()
{
  while(true) {
    llcm.handle();
  }
}
