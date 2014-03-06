#ifndef POINT2DMOTIONMODEL_H_
#define POINT2DMOTIONMODEL_H_

#include "multiagentmotionmodel.h"

namespace ranav {

class Point2dMotionModel : public MultiAgentMotionModel {
public:
  Point2dMotionModel();
  virtual ~Point2dMotionModel();
  virtual void init(const TParam &p);

  //! returns the resulting state [x_1, y_1, ..., x_N, y_N, x_N+1, y_N+1, v_x_N+1, v_y_N+1] (in global system)
  //! given the applied control [u_x_1, u_y_1, ..., u_x_N, u_y_N] (in global system)
  //! and the noise [n_v_x_1, n_v_y_1, ..., n_v_x_N, n_v_y_N, n_a_x_N+1, n_a_y_N+1] (in global system)
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
};

} /* namespace ranav */

#endif /* POINT2DMOTIONMODEL_H_ */
