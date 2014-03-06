#ifndef SENSORMODEL_H_
#define SENSORMODEL_H_

#include <Eigen/Core>
#include <vector>
#include "tparam.h"

namespace ranav {

class SensorModel {
public:
  //! ATTENTION: make sure to set stateDim, measurementDim, and noiseDim in every subclass
  SensorModel() : stateDim(0), measurementDim(0), noiseDim(0) {}
  virtual ~SensorModel() {}
  virtual void init(const TParam &p) = 0;

  //! returns whether a measurement is available in the given state
  virtual bool measurementAvailable(const Eigen::VectorXd &state) const { return true; }

  //! returns the dimensions of the measurement that contain angles (and need to be normalized in the EKF)
  const std::vector<unsigned int>& getAngleDimensions() const { return angleDimensions; }

  //! returns the resulting expected measurement
  //! returns empty vector in case the model doesn't expect to get a measurement in this state
  virtual Eigen::VectorXd sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const = 0;
  virtual Eigen::VectorXd sense(const Eigen::VectorXd &state) const;

  //! default implementation ignores state and measurement and samples from noiseCov
  virtual Eigen::VectorXd sampleNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;
  //! default implementation ignores state and measurement and returns noiseCov
  virtual Eigen::MatrixXd getNoiseCov(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;

  virtual Eigen::MatrixXd jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const = 0;
  virtual Eigen::MatrixXd jacobianNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const = 0;

  virtual Eigen::MatrixXd jacobianState(const Eigen::VectorXd &state) const;
  virtual Eigen::MatrixXd jacobianNoise(const Eigen::VectorXd &state) const;

  int getStateDim() const { return stateDim; }
  int getMeasurementDim() const { return measurementDim; }
  int getNoiseDim() const { return noiseDim; }

protected:
  int stateDim, measurementDim, noiseDim;
  Eigen::MatrixXd noiseCov;
  Eigen::MatrixXd noiseCovSqrt; // cached
  std::vector<unsigned int> angleDimensions; //!< the dimensions of the measurements that contain angles
};

} /* namespace ranav */

#endif /* SENSORMODEL_H_ */
