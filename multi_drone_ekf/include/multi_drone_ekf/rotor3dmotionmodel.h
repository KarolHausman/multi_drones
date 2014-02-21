#ifndef ROTOR3DMOTIONMODEL_H_
#define ROTOR3DMOTIONMODEL_H_

#include <ranav/rotor2dmotionmodel.h>

namespace ranav {

class Rotor3dMotionModel : public Rotor2dMotionModel {
public:
    Rotor3dMotionModel();
    virtual ~Rotor3dMotionModel();

    Eigen::VectorXd downProjectControl(const Eigen::VectorXd &control);

protected:
};

} /* namespace ranav */

#endif /* ROTOR3DMOTIONMODEL_H_ */
