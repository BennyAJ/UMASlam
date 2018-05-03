#ifndef __UMA_MAP_DRAWER_HPP__
#define __UMA_MAP_DRAWER_HPP__
#include <unordered_map>
#include "Constants.hpp"
#include "../lcmtypes/slam_state_t.hpp"
#include "Pose.hpp"
#include <SFML/Graphics.hpp>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <vector>
#include <utility>

constexpr double WINDOW_HEIGHT = 1600.0;
constexpr double WINDOW_WIDTH = 1200.0;

constexpr size_t PIX_PER_SQUARE = 4;

class MapDrawer
{
public:
  MapDrawer() {
    possibleChans.push_back(std::pair<std::string,sf::Color> (GPS_STATE_CHANNEL,sf::Color::Red));
    possibleChans.push_back(std::pair<std::string,sf::Color> (IMU_STATE_CHANNEL,sf::Color::Blue));
  }
  void startDrawThread();
  void addPose(const SLAM::Pose& pose, std::string channel);
  void startDraw();
  void drawPoses(sf::RenderWindow & win);
  void drawBoat(sf::RenderWindow & win);

  void handleState(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const slam_state_t * state);

private:
  std::pair<double, double> convertToPixelCoords(double x, double y);

  std::unordered_map<std::string,std::vector<SLAM::Pose>> unOrdMap;
  std::vector<std::pair<std::string,sf::Color>>possibleChans;
};
#endif
