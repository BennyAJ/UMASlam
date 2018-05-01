#include "IMUGlobalCompare.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include <cmath>

using namespace std;

void IMUGlobalCompare::handleIMU(const lcm::ReceiveBuffer * rbuf,
                    const string & chan,
                    const imu_t * imu_data)
{
  if (chan == "SENSOR_IMU_GLOBAL") 
  {
    last_sim_imu_info = {imu_data->udot, imu_data->vdot};
    cout << "SIMULATOR utime: " << imu_data->utime << endl;
    cout << "SIMULATOR udot: " << imu_data->udot << endl;
    cout << "SIMULATOR vdot: " << imu_data->vdot << endl;
    cout << "--------------------" << endl;
  }
  else if (chan == TRANSFORMED_IMU_CHANNEL) 
  {
    //cout << "difference in udot is: " << last_sim_imu_info.first - imu_data->udot << endl;
    //cout << "difference in vdot is: " << last_sim_imu_info.second - imu_data->vdot << endl;
    cout << "TRANSFORMED utime: " << imu_data->utime << endl;
    cout << "TRANSFORMED udot: " << imu_data->udot << endl;
    cout << "TRANSFORMED vdot: " << imu_data->vdot << endl;
    cout << "--------------------" << endl;
  }
}

int main() {
  IMUGlobalCompare imuGlobalCompare;
  lcm::LCM lcm;
  lcm.subscribe(TRANSFORMED_IMU_CHANNEL, &IMUGlobalCompare::handleIMU, &imuGlobalCompare);
  lcm.subscribe("SENSOR_IMU_GLOBAL", &IMUGlobalCompare::handleIMU, &imuGlobalCompare);

  while(1) {
    lcm.handle();
  }
}
