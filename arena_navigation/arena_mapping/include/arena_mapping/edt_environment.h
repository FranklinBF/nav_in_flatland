
#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_


#include <Eigen/Eigen>
//#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>


//#include <plan_env/obj_predictor.h>
#include <arena_mapping/mapping.h>

class EDTEnvironment {
private:

    double resolution_inv_;
    double distToBox(int idx, const Eigen::Vector2d& pos, const double& time);
    double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
    EDTEnvironment(/* args */) {};
    ~EDTEnvironment() {};

    // define EDTEnvironment Ptr
    typedef std::shared_ptr<EDTEnvironment> Ptr;

    SDFMap::Ptr sdf_map_;
    
    /* Init env*/
    void init();

    void setMap(SDFMap::Ptr map);

    void getMapRegion(Eigen::Vector2d& ori, Eigen::Vector2d& size) {sdf_map_->getRegion(ori, size);}

    /* Get distance*/
    double evaluateCoarseEDT(Eigen::Vector2d& pos);

    /* Get distance gradient*/
    void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);

    std::pair<double, Eigen::Vector2d> interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff,
                                                     double& value, Eigen::Vector2d& grad);

    std::pair<double, Eigen::Vector3d> evaluateEDTWithGrad(const Eigen::Vector2d& pos,double& dist, Eigen::Vector2d& grad);

};














#endif