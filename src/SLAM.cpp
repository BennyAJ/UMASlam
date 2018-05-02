#include "SLAM.hpp"
#include "Constants.hpp"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

Slam::Slam() : mapper(MIN_X, MAX_X, MIN_Y, MAX_Y, SQUARE_SIZE),
         localizer(NUM_PARTICLES, PERCENT_PREDICTION_PARTICLES),
         num_mapped_scans(0),
         end_flag(false),
         reinitialized_fog(false),
         compass_north(COMPASS_DEFAULT),
         imu_north(COMPASS_DEFAULT)
{
  llcm.subscribe(SLAM_POINT_CLOUD_CHANNEL, &Slam::handlePointCloud, this);
  llcm.subscribe(GPS_CHANNEL, &Slam::handleGPSData, this);
  llcm.subscribe(FOG_CHANNEL, &Slam::handleFOGData, this);
  llcm.subscribe(COMPASS_CHANNEL, &Slam::handleCompassData, this);
  llcm.subscribe(TRANSFORMED_IMU_CHANNEL, &Slam::handleIMUData, this);
}

void Slam::handlePointCloud(const lcm::ReceiveBuffer * rbuf,
                const string & chan,
                const slam_pc_t * pc)
{

  if(num_mapped_scans >= NUM_ONLY_MAP_SCANS)
  {
    localizer.handlePointCloud(rbuf, chan, pc);
    mapper.addPose(localizer.getPose());
  }
  else
  {
    ++num_mapped_scans;
  }

  //sets scope for lock guard
  {
    std::lock_guard<std::mutex> map_lock(map_mut);
    mapper.handlePointCloud(rbuf, chan, pc);
    localizer.updateMap(mapper.getMap());
  }

  // Publish the updated map
  mapper.publishMap();

  #ifdef PROFILE
  if(num_mapped_scans >= NUM_ONLY_MAP_SCANS)
  {
    ++num_mapped_scans;
  }
  
  if(num_mapped_scans > NUM_PROFILED_SCANS)
  {
    end_flag = true;
  }
  #endif
}

void Slam::handleFOGData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const fog_t * fog_data)
{
  localizer.handleFOGData(rbuf, chan, fog_data);
  if(!reinitialized_fog)
  {
    fake_compass.addFOG(*fog_data);
  }
}

void Slam::handleCompassData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const compass_t * compass_data)
{
  compass_north = compass_data->yaw;
}

void Slam::handleGPSData(const lcm::ReceiveBuffer * rbuf,
             const string & chan,
             const gps_t * gps_data)
{
  localizer.handleGPSData(rbuf, chan, gps_data);

  //reinitialization of the fog
  if(!reinitialized_fog)
  {
    fake_compass.addGPS(*gps_data);
    if(NORTH_IS_ZERO) {
      reinitialized_fog = true;
      mapper.reset();
      localizer.reset();
      localizer.reinitializeFOG(0);
      localizer.updateMap(mapper.getMap());
    }
    else if(!USE_FAKE_COMPASS || compass_north != COMPASS_DEFAULT) 
    {
      // Use compass to initialize north
      reinitialized_fog = true;
      mapper.reset();
      localizer.reset();
      localizer.reinitializeFOG(compass_north);
      localizer.updateMap(mapper.getMap());
    }
    // Fall back to fake compass if compass is unavailable or 
    // priority is set to fake compass
    else 
    {
      if(fake_compass.getDistFromOrigin() > ORIGIN_DIST_BEFORE_REINITIALIZATION)
      {
        reinitialized_fog = true;
        mapper.reset();
        localizer.reset();
        localizer.reinitializeFOG(fake_compass.getNorthLocation(localizer.getFogInitialization()));
        localizer.updateMap(mapper.getMap());
      }
    }
  }

  // Publish the updated map
  mapper.publishMap();
}

void Slam::handleIMUData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const imu_t * imu_data)
{
  imu_north = 2 * M_PI - imu_data->yaw + M_PI / 2;

  while(imu_north < 0)
    imu_north += 2 * M_PI;
  while(imu_north > 2 * M_PI)
    imu_north -= 2 * M_PI;

  localizer.handleIMUData(rbuf, chan, imu_data);
}

const GridMap& Slam::getMap() 
{
  std::lock_guard<std::mutex> map_lock(map_mut);
  return mapper.getMap();
}

void Slam::run()
{
  while(!end_flag)
  {
    llcm.handle();
    //lock the map to avoid rewriting of the map while it's publishing
    {
      std::lock_guard<std::mutex> map_lock(map_mut);
    }
    //map_lock releases after the publish

  }
}
