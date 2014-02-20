#ifndef ROTOR2DMOTIONMODEL_H_
#define ROTOR2DMOTIONMODEL_H_

#include "multi_drone_ekf/motionmodel.h"

namespace ranav {

class Rotor2dMotionModel : public MotionModel {
public:
  Rotor2dMotionModel();
  virtual ~Rotor2dMotionModel();
//  virtual void init(const TParam &p);


  virtual Eigen::VectorXd move(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const;
  using MotionModel::move;

  virtual Eigen::MatrixXd jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const;
  virtual Eigen::MatrixXd jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const;
  virtual Eigen::MatrixXd jacobianNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const;
  using MotionModel::jacobianState;
  using MotionModel::jacobianControl;
  using MotionModel::jacobianNoise;

protected:
  Eigen::MatrixXd A, B, V;
  double dt;
  double gravity;
};

} /* namespace ranav */

#endif /* ROTOR2DMOTIONMODEL_H_ */

