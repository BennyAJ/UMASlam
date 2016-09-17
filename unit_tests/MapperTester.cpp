#include "../src/Mapper.hpp"
#include "../src/MapDrawer.hpp"

#include <lcm/lcm-cpp.hpp>
#include <thread>

using namespace std;
const char * LASER_SCAN_CHANNEL = "SENSOR_LASER"; 
const char * STATE_CHANNEL = "STATE_CHANNEL";   
const char * SERVO_CHANNEL = "SENSOR_LASER_SERVO";

int main()
{
	//initialize a giant map
	Mapper mapper(-30, 30, -30, 30, .25);

	lcm::LCM lcm;
	lcm.subscribe(LASER_SCAN_CHANNEL, &Mapper::handleLaserScan, &mapper);
	lcm.subscribe(SERVO_CHANNEL, &Mapper::handleServo, &mapper);
	lcm.subscribe(STATE_CHANNEL, &Mapper::handleState, &mapper);
	
	MapDrawer drawer;
	drawer.startDrawThread();

	while(0 == lcm.handle())
	{
		//handle any backlog of events before drawing
		while(lcm.handleTimeout(10) > 0)
		{
		}
		drawer.switchMap(mapper.getMapCopy());
		this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}