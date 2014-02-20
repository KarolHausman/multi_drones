#ifndef MARKER3DSENSORMODEL_H_
#define MARKER3DSENSORMODEL_H_

#include "multi_drone_ekf/marker2dsensormodel.h"

namespace ranav {

class Marker3dSensorModel : public Marker2dSensorModel {
public:
    Marker3dSensorModel(const tf::Transform& cam_to_world, const tf::Transform& drone_to_marker);
    virtual ~Marker3dSensorModel();

    Eigen::VectorXd downProjectMeasurement(const Eigen::VectorXd& measurement);
    void setNoiseCov(const Eigen::MatrixXd& noise_6dog);



protected:
    Eigen::MatrixXd H, W;
    //  TParam params;
    //  unsigned int nA; //!< num agents
    //  unsigned int nT; //!< num targets
    double measurementNoise;
    double distanceNoiseFactor;
    double visibilityRadius;
};

} /* namespace ranav */

#endif /* MARKER3DSENSORMODEL_H_ */
