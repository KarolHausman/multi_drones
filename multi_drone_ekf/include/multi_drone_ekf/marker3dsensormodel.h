#ifndef MARKER3DSENSORMODEL_H_
#define MARKER3DSENSORMODEL_H_

#include "multi_drone_ekf/marker2dsensormodel.h"

namespace ranav {

class Marker3dSensorModel : public Marker2dSensorModel {
public:
    Marker3dSensorModel(const tf::Transform& cam_to_world, const tf::Transform& drone_to_marker);
    virtual ~Marker3dSensorModel();

    Eigen::VectorXd downProjectMeasurement(const Eigen::VectorXd& measurement, const tf::Transform& world_to_cam);
    void setNoiseCov(const tf::Transform& world_to_cam, const Eigen::VectorXd& measurement);



protected:
    Eigen::MatrixXd H, W, noiseCovPrime;

    double measurementNoise;
    double distanceNoiseFactor;
    double visibilityRadius;
};

} /* namespace ranav */

#endif /* MARKER3DSENSORMODEL_H_ */
