#include "IMUCompare.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;
using namespace common::LCM::types;

void IMUCompare::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  if (chan == PERFECT_IMU_CHANNEL) 
  {
    last_perfect_imu_info = {imu_data->udot, imu_data->vdot};

    cout << "difference in udot is: " << last_perfect_imu_info.first - last_imu_info.first << endl;
    cout << "difference in vdot is: " << last_perfect_imu_info.second - last_imu_info.second << endl;
  }
  else if (chan == IMU_CHANNEL) 
  {
    last_imu_info = {imu_data->udot, imu_data->vdot};
  }
}


int main() {
  IMUCompare IMUCompare;
  lcm::LCM lcm;
  lcm.subscribe(PERFECT_IMU_CHANNEL, &IMUCompare::handleIMU, &IMUCompare);
  lcm.subscribe(IMU_CHANNEL, &IMUCompare::handleIMU, &IMUCompare);

  while(1) {
    lcm.handle();
  }
}
