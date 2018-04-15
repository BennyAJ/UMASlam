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
  /*if (chan == PERFECT_IMU_CHANNEL || chan == IMU_CHANNEL) {
    cout << "chan is " << chan << endl;
    cout << "perfect_imu is " << last_perfect_imu_info.first << " " << last_perfect_imu_info.second << endl;
    cout << "imu is " << last_imu_info.first << " " << last_imu_info.second << endl;
    cout << "________________________________" << endl;
  }*/

  if (chan == PERFECT_IMU_CHANNEL) 
  {
    last_perfect_imu_info = {imu_data->udot, imu_data->vdot};

    cout << "difference in udot is 1: " << last_perfect_imu_info.first - last_imu_info.first << endl;
    cout << "difference in vdot is 2: " << last_perfect_imu_info.second - last_imu_info.second << endl;
  }
  else if (chan == IMU_CHANNEL) 
  {

    last_imu_info = {imu_data->udot, imu_data->vdot};

    cout << "difference in udot is 3: " << last_perfect_imu_info.first - last_imu_info.first << endl;
    cout << "difference in vdot is 4: " << last_perfect_imu_info.second - last_imu_info.second << endl;
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