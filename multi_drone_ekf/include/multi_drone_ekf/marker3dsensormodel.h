#ifndef MARKER3DSENSORMODEL_H_
#define MARKER3DSENSORMODEL_H_

#include <ranav/cam2dsensormodel.h>
#include <tf/transform_broadcaster.h>

namespace ranav {

class Marker3dSensorModel : public Cam2dSensorModel {
public:
    Marker3dSensorModel(int from, int to);
    virtual ~Marker3dSensorModel();

    //! world_to_cam is the camera position on its agent or the GPS-camera position wrt. to the world frame of reference
    Eigen::VectorXd downProjectMeasurement(const tf::Transform& measurement, const tf::Transform& world_to_cam) const;
    void setNoiseCov(const tf::Transform& world_to_cam, const tf::Transform& measurement);

protected:
    Eigen::MatrixXd noiseCovPrime;

    double measurementNoise;
    double distanceNoiseFactor;
    double visibilityRadius;
};

} /* namespace ranav */

#endif /* MARKER3DSENSORMODEL_H_ */
