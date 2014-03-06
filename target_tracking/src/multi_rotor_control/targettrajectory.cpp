#include "targettrajectory.h"
#include <cmath>

namespace ranav {

TargetTrajectory::TargetTrajectory() :
  type(TT_EIGHT),
  internalState(0),
  dt(0),
  vel(0),
  radius(0),
  length(0)
{
}

TargetTrajectory::~TargetTrajectory() {
}

void TargetTrajectory::init(const TParam &p) {
  std::string typestr = p("multi_rotor_control/targetTrajectoryType").toString();
  if (typestr == "eight") {
    type = TT_EIGHT;
  } else if (typestr == "eight_oval") {
    type = TT_EIGHT_OVAL;
  } else {
    std::cerr << "Error: Unkown target trajectory type '" << typestr << "' - exiting.\n";
    exit(-1);
  }
  dt = p("estimation/motionDt").toDouble();
  vel = 0.3;
  radius = 0.5;
  switch (type) {
    case TT_EIGHT:
      length = radius * (4 + 3*M_PI);
      break;
    case TT_EIGHT_OVAL:
      length = radius * 2*(2 + 2.5*M_PI + 2*sqrt(2));
      break;
    default: ;
  }

  int steps = ceil(length/vel/dt);
  trajectory.clear();
  for (int i=0; i<steps; ++i) {
    trajectory.push_back(get(i*vel*dt));
  }
}

Eigen::VectorXd TargetTrajectory::get(double t) const {
  if (type == TT_EIGHT) {
    double piece = 0;
    if (t < piece + radius)
      return Eigen::Vector2d((t-piece)/sqrt(2), (t-piece)/sqrt(2));
    piece += radius;
    if (t < piece + 1.5*M_PI*radius)
      return Eigen::Vector2d(sqrt(2) + cos(M_PI*0.75-(t-piece)/radius), sin(M_PI*0.75-(t-piece)/radius)) * radius;
    piece += 1.5*M_PI*radius;
    if (t < piece + 2*radius)
      return Eigen::Vector2d(radius/sqrt(2)-(t-piece)/sqrt(2), -radius/sqrt(2)+(t-piece)/sqrt(2));
    piece += 2*radius;
    if (t < piece + 1.5*M_PI*radius)
      return Eigen::Vector2d(-sqrt(2) + cos(M_PI*0.25+(t-piece)/radius), sin(M_PI*0.25+(t-piece)/radius)) * radius;
    piece += 1.5*M_PI*radius;
    if (t < piece + radius)
      return Eigen::Vector2d(-radius/sqrt(2) + (t-piece)/sqrt(2), -radius/sqrt(2) + (t-piece)/sqrt(2));
    piece += radius;

    return Eigen::Vector2d(0, 0);
  } else if (type == TT_EIGHT_OVAL) {
    double piece = 0;
    if (t < piece + radius)
      return Eigen::Vector2d((t-piece)/sqrt(2), (t-piece)/sqrt(2));
    piece += radius;
    if (t < piece + 1.25*M_PI*radius)
      return Eigen::Vector2d(sqrt(2) + cos(M_PI*0.75-(t-piece)/radius), sin(M_PI*0.75-(t-piece)/radius)) * radius;
    piece += 1.25*M_PI*radius;
    if (t < piece + 2*radius*sqrt(2))
      return Eigen::Vector2d(radius*sqrt(2)-(t-piece), -radius);
    piece += 2*radius*sqrt(2);
    if (t < piece + 1.25*M_PI*radius)
      return Eigen::Vector2d(-sqrt(2) + cos(-M_PI/2-(t-piece)/radius), sin(-M_PI/2-(t-piece)/radius)) * radius;
    piece += 1.25*M_PI*radius;
    if (t < piece + radius)
      return Eigen::Vector2d(-radius/sqrt(2) + (t-piece)/sqrt(2), radius/sqrt(2) - (t-piece)/sqrt(2));
    piece += radius;

    if (t < piece + radius)
      return Eigen::Vector2d((t-piece)/sqrt(2), -(t-piece)/sqrt(2));
    piece += radius;
    if (t < piece + 1.25*M_PI*radius)
      return Eigen::Vector2d(sqrt(2) + cos(M_PI*0.75-(t-piece)/radius), -sin(M_PI*0.75-(t-piece)/radius)) * radius;
    piece += 1.25*M_PI*radius;
    if (t < piece + 2*radius*sqrt(2))
      return Eigen::Vector2d(radius*sqrt(2)-(t-piece), radius);
    piece += 2*radius*sqrt(2);
    if (t < piece + 1.25*M_PI*radius)
      return Eigen::Vector2d(-sqrt(2) + cos(-M_PI/2-(t-piece)/radius), -sin(-M_PI/2-(t-piece)/radius)) * radius;
    piece += 1.25*M_PI*radius;
    if (t < piece + radius)
      return Eigen::Vector2d(-radius/sqrt(2) + (t-piece)/sqrt(2), -radius/sqrt(2) + (t-piece)/sqrt(2));
    piece += radius;

    return Eigen::Vector2d(0, 0);
  }
  return Eigen::Vector2d();
}


Eigen::VectorXd TargetTrajectory::step() {
  assert(!trajectory.empty());
  if (internalState == trajectory.size())
    internalState = 0;
  return trajectory[internalState++];
}

Eigen::VectorXd TargetTrajectory::randomJump() {
  assert(!trajectory.empty());
  internalState = rand()%trajectory.size();
  return trajectory[internalState++];
}

bool TargetTrajectory::atEnd() const {
  return internalState == trajectory.size();
}

} /* namespace ranav */
