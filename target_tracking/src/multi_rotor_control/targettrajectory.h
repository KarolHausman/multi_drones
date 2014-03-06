#ifndef TARGETTRAJECTORY_H_
#define TARGETTRAJECTORY_H_

#include <Eigen/Core>
#include "tparam.h"

namespace ranav {

//! Trajectory of a target with state [x, y] moving in two-dimensional space
class TargetTrajectory {
public:
  TargetTrajectory();
  virtual ~TargetTrajectory();

  virtual void init(const TParam &p);

  //! make a step and return the resulting state
  Eigen::VectorXd step();
  //! make a random jump on the trajectory (to any direction) and return the resulting state
  Eigen::VectorXd randomJump();
  //! returns whether the trajectory reached its end in the last step
  bool atEnd() const;
  //! returns the size of the discretized trajectory
  size_t size() const { return trajectory.size(); }

  //! returns the state specified by the (continuous) trajectory at the given time
  Eigen::VectorXd get(double t) const;
  //! returns the length of the trajectory (this is not the overall duration)
  double getLength() const { return length; }

protected:
  enum Type {
    TT_EIGHT,
    TT_EIGHT_OVAL,
  };
  Type type;
  unsigned int internalState;
  double dt;
  double vel;
  double radius;
  double length;
  std::vector<Eigen::VectorXd> trajectory;
};

} /* namespace ranav */

#endif /* TARGETTRAJECTORY_H_ */
