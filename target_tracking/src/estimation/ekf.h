#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Core>
#include <vector>
#include "tparam.h"

namespace ranav {

class MotionModel;
class SensorModel;

class EKF {
public:
  //! m needs to be initialized
  EKF(MotionModel *m);
  virtual ~EKF();
  virtual void init(const TParam &p);

  //! prediction step given the motionModel
  //! predict with linearization around mean and control
  void predict(const Eigen::VectorXd &control);

  //! correction step given the sensorModel
  //! correct with linearization around mean
  void correct(const Eigen::VectorXd &measurement, const SensorModel &sensorModel);

  const Eigen::VectorXd& getMean() const { return mean; }
  const Eigen::MatrixXd& getCovariance() const { return covariance; }
  Eigen::VectorXd& getMean() { return mean; }
  Eigen::MatrixXd& getCovariance() { return covariance; }


protected:
  MotionModel *motionModel;
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
};

} /* namespace ranav */

#endif /* EKF_H_ */
