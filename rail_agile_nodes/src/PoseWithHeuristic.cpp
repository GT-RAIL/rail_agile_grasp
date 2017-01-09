#include "rail_agile_nodes/PoseWithHeuristic.h"

using namespace std;

PoseWithHeuristic::PoseWithHeuristic(geometry_msgs::Pose pose, double h)
{
  this->pose = pose;
  this->h = h;
}

bool PoseWithHeuristic::operator < (const PoseWithHeuristic obj) const
{
  return this->h < obj.h;
}
