#ifndef ROTOR3DMOTIONMODEL_H_
#define ROTOR3DMOTIONMODEL_H_

#include "multi_drone_ekf/rotor2Dmotionmodel.h"

namespace ranav {

class Rotor3dMotionModel : public Rotor2dMotionModel {
public:
    Rotor3dMotionModel();
    virtual ~Rotor3dMotionModel();


    Eigen::VectorXd downProjectControl(const Eigen::VectorXd &control);

protected:
  Eigen::MatrixXd A, B, V;
  double dt;
  double gravity;

};

} /* namespace ranav */

#endif /* ROTOR3DMOTIONMODEL_H_ */
