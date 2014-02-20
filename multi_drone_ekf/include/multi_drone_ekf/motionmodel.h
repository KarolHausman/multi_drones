#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_

#include <Eigen/Core>
//#include "tparam.h"

namespace ranav {

class MotionModel {
public:
  virtual ~MotionModel() {}
//  virtual void init(const TParam &p) = 0;

  //! returns the resulting state
  virtual Eigen::VectorXd move(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const = 0;
  virtual Eigen::VectorXd move(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const;

  //! default implementation ignores state and control and samples from noiseCov
  virtual Eigen::VectorXd sampleNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const;
  //! default implementation ignores state and control and returns noiseCov
  virtual Eigen::MatrixXd getNoiseCov(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const;

  virtual Eigen::MatrixXd jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const = 0;
  virtual Eigen::MatrixXd jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const = 0;
  virtual Eigen::MatrixXd jacobianNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const = 0;

  virtual Eigen::MatrixXd jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const;
  virtual Eigen::MatrixXd jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const;
  virtual Eigen::MatrixXd jacobianNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const;

  int getStateDim() const { return stateDim; }
  int getControlDim() const { return controlDim; }
  int getNoiseDim() const { return noiseDim; }

protected:
  int stateDim, controlDim, noiseDim;
  Eigen::MatrixXd noiseCov;
  Eigen::MatrixXd noiseCovSqrt; // cached
};

} /* namespace ranav */

#endif /* MOTIONMODEL_H_ */
