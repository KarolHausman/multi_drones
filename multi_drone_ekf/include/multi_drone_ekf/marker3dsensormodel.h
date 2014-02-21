#ifndef MARKER3DSENSORMODEL_H_
#define MARKER3DSENSORMODEL_H_

#include <ranav/cam2dsensormodel.h>
#include <tf/transform_broadcaster.h>

namespace ranav {

class Marker3dSensorModel : public Cam2dSensorModel {
public:
    Marker3dSensorModel(int from, int to);
    virtual ~Marker3dSensorModel();

    Eigen::VectorXd downProjectMeasurement(const Eigen::VectorXd& measurement, const tf::Transform& world_to_cam) const;
    void setNoiseCov(const tf::Transform& world_to_cam, const Eigen::VectorXd& measurement);

protected:
    Eigen::MatrixXd noiseCovPrime;

    double measurementNoise;
    double distanceNoiseFactor;
    double visibilityRadius;
};

} /* namespace ranav */

#endif /* MARKER3DSENSORMODEL_H_ */
