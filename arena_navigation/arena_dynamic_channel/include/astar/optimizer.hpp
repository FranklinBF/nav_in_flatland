/*
    Copyright (c) 2020, Delong Zhu <zhudelong@link.cuhk.edu.hk>
*/

#ifndef _OPTIMIZER_HPP_
#define _OPTIMIZER_HPP_

#pragma once

// system header
#include <vector>
#include <iostream>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// GTSAM headers
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/PriorFactor.h>


#include "dataset/odom_list.h"
#include "optimizer/PlanarSDF.h"
#include "optimizer/ObstacleDynamicSDFFactor.h"

// my unairy factor
#include "optimizer/GaussianProcessPriorLinear.h"
#include "optimizer/GaussianProcessInterpolatorLinear.h"

// obstacle factor
#include "optimizer/ObstacleCost.h"
#include "optimizer/ObstaclePlanarSDFFactor.h"
#include "optimizer/ObstaclePlanarSDFFactorGP.h"
#include "optimizer/PoseLimitFactor.h"
#include "optimizer/VelocityLimitFactor.h"

#include "astar/data.hpp"

//#define DEBUG

namespace astar
{

using namespace cv;
using namespace std;
using namespace gtsam;
using namespace gpmp2;


class PointRobotModel
{
private:
    size_t body_spheres_; // Number of body spheres for cost evaluation
    size_t dof_; // Number of variables that participate obstacle cost
public:
    typedef gtsam::Vector Pose;
    typedef gtsam::Vector Velocity;
    PointRobotModel(): body_spheres_(1), dof_(2) {}
    size_t nr_body_spheres() const {return body_spheres_;}
    size_t dof() const {return dof_;}
};


typedef GaussianProcessPriorLinear GPPrior;
typedef GaussianProcessInterpolatorLinear Interplotation;
typedef ObstaclePlanarSDFFactor<PointRobotModel> ObstFactor;
typedef ObstacleDynamicSDFFactor<PointRobotModel> DynObstFactor;
typedef ObstaclePlanarSDFFactorGP<PointRobotModel, Interplotation> ObstFactorGP;
typedef JointLimitFactorVector PLFactor;
typedef VelocityLimitFactorVector VLFactor;


/*
 * How to determin delta_t
 * max_speed (2m/s) / cell_size (0.2m) = (10 pixel/s)
 * SDFS_STEP = ceil(10 / 3 (pixels) ) = 0.4 (s)
 *
**/
void genTimedSDF(const std::vector<double>& coords,
                 const std::vector<double>& speeds,
                 std::vector<PlanarSDF>& sdfs,
                 Vec2d& bound,
                 Vec2d& offset) {

    sdfs.clear();
    double delta_t_ = SDF_INTVL;

    // locate boundary
    auto x_max = coords[PHASE1_INDEX*2];
    auto y_max = coords[PHASE1_INDEX*2 + 1];
    auto x_min = coords[PHASE3_INDEX*2];
    auto y_min = coords[PHASE3_INDEX*2 + 1];
    auto x_offset = coords[PHASE2_INDEX*2];
    auto y_offset = coords[PHASE2_INDEX*2 + 1];
    offset.x = x_offset;
    offset.y = y_offset;

    // image size
    auto height = y_max - y_min;
    auto width  = x_max - x_min;
    auto rows = static_cast<int>(ceil(height / CELL_SIZE));
    auto cols = static_cast<int>(ceil(width  / CELL_SIZE));
    bound.x = width;
    bound.y = height;

    // perform dynamic sdf
    size_t ped_num = coords.size() >> 1;
    for (size_t t = 0; t < SDF_NUM; ++t) {

        // distance map generation
        cv::Mat dst_img, src_img(rows, cols, CV_8UC1, cv::Scalar(255));
        for (size_t i = PEDS_START; i < ped_num; ++i) {
            double px = coords[2*i], py = coords[2*i+1];
            double vx = speeds[2*i], vy = speeds[2*i+1];

            double pxt = px + vx * t * delta_t_;
            double pyt = py + vy * t * delta_t_;

            double px_next = pxt + vx * delta_t_;
            double py_next = pyt + vy * delta_t_;

            double px_mean = (pxt + px_next) * 0.5;
            double py_mean = (pyt + py_next) * 0.5;

            // must be int type for boundary testing
            // xi = x - x_tl, yi = - (y - y_tl)
            auto c = static_cast<int> (floor((pxt - x_offset) / CELL_SIZE));
            auto r = static_cast<int> (floor((y_offset - pyt) / CELL_SIZE));
            if (c < 0 || c >= cols || r < 0 || r >= rows){
                continue;
            }
            src_img.at<char>(r, c) = 0;

            // the next point
            c = static_cast<int> (floor((px_next - x_offset) / CELL_SIZE));
            r = static_cast<int> (floor((y_offset - py_next) / CELL_SIZE));
            if (c < 0 || c >= cols || r < 0 || r >= rows){
                continue;
            }
            src_img.at<char>(r, c) = 0;

            // the center point
            c = static_cast<int> (floor((px_mean - x_offset) / CELL_SIZE));
            r = static_cast<int> (floor((y_offset - py_mean) / CELL_SIZE));
            if (c < 0 || c >= cols || r < 0 || r >= rows){
                continue;
            }
            src_img.at<char>(r, c) = 0;
        }

        // initialize single plannar sdf
        Matrix sdf_data;
        distanceTransform(src_img, dst_img, CV_DIST_L2, CV_DIST_MASK_3);
        cv::cv2eigen(dst_img, sdf_data);
        sdfs.push_back(PlanarSDF(Point2(-0.0001, -0.0001), CELL_SIZE, sdf_data*CELL_SIZE));
        //        imshow(to_string(t), src_img);
        //        waitKey(1);
    }

#ifdef DEBUG
    // visualization
    for (size_t t = 0; t < SDF_NUM; ++t) {
        Mat dst_img;
        cv::eigen2cv(sdfs[t].raw_data(), dst_img);
        normalize(dst_img, dst_img, 0, 1., cv::NORM_MINMAX);
        imshow(to_string(t), dst_img);
        waitKey(1);
    }
#endif

}



/*
 * waypoints and traj should in x-y frame not u-v
 *
 * TODO: replace Vec2d with Eigen type
 *
**/
void optimizeAstar(const std::vector<Vec2d>& waypoints,
                   const std::vector<PlanarSDF>& sdfs,
                   const Vec2d& bound,
                   const Vec2d& offset,
                   const Vec2d& vel_init,
                   const Vec2d& vel_goal,
                   std::vector<Vec2d>& traj,
                   std::vector<Vec2d>& vels) {

    // check waypoints
    double delta_t = SDF_INTVL;
    double STEP_DIST = MAX_SPEED * delta_t;
    traj.clear();
    vels.clear();


    // ensure at least three way points
    auto& wps = waypoints;
    auto n = wps.size() - 1;
    if ((wps[n] - wps[0]).length() < STEP_DIST + 0.01) {
        traj.push_back(wps[n]);
        traj.push_back(wps[0]);
        vels.push_back(vel_goal);
        vels.push_back(vel_goal);
        return;
    }

    // prepare the init values
    std::vector<Vec2d> anchors;
    for (int i = n; i > 0; i -= 1) { // when decreasing using int!!
        // forward distance in a single time step
        auto diff = wps[i-1] - wps[i];
        auto ford_vec = diff / diff.length() * STEP_DIST;

        // total forward steps
        size_t seg_num = static_cast<size_t>(round(diff.length() / STEP_DIST));
        for (size_t t = 0; t < seg_num; ++t) {
            auto anchor = wps[i] + ford_vec * t;
            anchors.push_back(anchor);
        }
    }
    anchors.push_back(wps[0]);

    // velocities
    std::vector<Vec2d> velocities;
    for (size_t i = 0; i < anchors.size() - 1; ++i) {
        velocities.push_back((anchors[i+1] - anchors[i]) / delta_t);
    }
    velocities.push_back(velocities.back());



    // frame transform and cal velocity
    std::vector<Vec2d> p_vals;
    std::vector<Vec2d> v_vals;
    for (size_t i = 0; i < anchors.size() - 1; ++i) {
        Vec2d cur(anchors[i].x - offset.x, offset.y - anchors[i].y);
        Vec2d next(anchors[i+1].x - offset.x, offset.y - anchors[i+1].y);
        p_vals.push_back(cur);
        v_vals.push_back((next - cur) / delta_t);
    }
    p_vals.push_back(Vec2d(wps[0].x-offset.x, offset.y-wps[0].y));
    v_vals.push_back(v_vals.back());



    // kinodynamic constraints
    auto pos_lmt_cov = noiseModel::Isotropic::Sigma(2, 0.01);
    Vector2 p_dlimit(0.0, 0.0);
    Vector2 p_ulimit(bound.x, bound.y);
    Vector2 p_thresh(0.01, 0.01);
    auto vel_lmt_cov = noiseModel::Isotropic::Sigma(2, 0.01);
    Vector2 v_limit(MAX_SPEED, MAX_SPEED);
    Vector2 v_thresh(0.01, 0.01);

    // prior factor parameters
    auto pos_cov = noiseModel::Isotropic::Sigma(2, 0.01);
    auto vel_cov = noiseModel::Isotropic::Sigma(2, 0.1);

    // unary obstacle factor parameter
    double eps_p = 1.0;  // meter
    double eps_v = 1.0;  // meter
    double obst_sigma = 0.01;

    // covatiance model
    Matrix Qc = 0.01 * Matrix::Identity(2, 2);
    auto Qc_model = noiseModel::Gaussian::Covariance(Qc);


    // define a graph
    Values init_values;
    NonlinearFactorGraph graph;
    auto robot_model = PointRobotModel();
    Vector vel_ref_i = (Vector(2) << vel_init.x, -vel_init.y).finished();
    Vector vel_ref_g = (Vector(2) << vel_goal.x, -vel_goal.y).finished();

    size_t total_steps = min(SDF_NUM, p_vals.size());
    for (size_t i = 0; i < total_steps; ++i) {

        Vector pos_val = (Vector(2) << p_vals[i].x, p_vals[i].y).finished();
        Vector vel_val = (Vector(2) << p_vals[i].x, p_vals[i].y).finished();

        // unary obstacle factor
        Key pos_key = Symbol('x', i);
        Key vel_key = Symbol('v', i);
        graph.add(DynObstFactor(pos_key, vel_key, robot_model, sdfs,
                                obst_sigma, eps_p, eps_v, delta_t, i));
        // graph.add(ObstFactor(pos_key, robot_model_, sdfs[0], obst_sigma, eps_p));

        // unary kinodynamic factor
        graph.add(PLFactor(pos_key, pos_lmt_cov, p_dlimit, p_ulimit, p_thresh));
        graph.add(VLFactor(vel_key, vel_lmt_cov, v_limit, v_thresh));

        // start factor
        if (i == 0) {
            graph.add(PriorFactor<Vector> (pos_key, pos_val, pos_cov));
            graph.add(PriorFactor<Vector> (vel_key, vel_ref_i, vel_cov));
            init_values.insert(pos_key, pos_val);
            init_values.insert(vel_key, vel_ref_i);
        } else if (i == total_steps - 1) {
            Key last_pos_key = Symbol('x', i-1);
            Key last_vel_key = Symbol('v', i-1);
            graph.add(GPPrior(last_pos_key, last_vel_key, pos_key, vel_key, delta_t, Qc_model));
            graph.add(PriorFactor<Vector> (pos_key, pos_val, pos_cov));
            graph.add(PriorFactor<Vector> (vel_key, vel_ref_g, vel_cov));
            init_values.insert(pos_key, pos_val);
            init_values.insert(vel_key, vel_ref_g);
        } else {
            Key last_pos_key = Symbol('x', i-1);
            Key last_vel_key = Symbol('v', i-1);
            graph.add(GPPrior(last_pos_key, last_vel_key, pos_key, vel_key, delta_t, Qc_model));
            init_values.insert(pos_key, (Vector(2) << p_vals[i].x, p_vals[i].y).finished());
            init_values.insert(vel_key, vel_val);
        }

    } // end graph buliding


    // start optimization
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values);
    Values values = optimizer.optimize();
//    auto pre_error = optimizer.error();
//    auto cur_error = optimizer.error();
//    auto success = checkConvergence(optimizer.params(), pre_error, cur_error);
//    std::cout << success << std::endl;
//    std::cout << optimizer.iterations() << std::endl;
//    std::cout << optimizer.error() << std::endl;


    for (size_t i = 0; i < total_steps; ++i) {
        Key pos_key = Symbol('x', i);
        auto pose = values.at<gtsam::Vector2>(pos_key);
        auto px = pose.x() + offset.x;
        auto py = offset.y - pose.y();
        anchors[i] = Vec2d(px, py);

        Key vel_key = Symbol('v', i);
        auto vel = values.at<gtsam::Vector2>(vel_key);
        velocities[i] = Vec2d(vel.x(), -vel.y());
    }


    traj.assign(anchors.begin(), anchors.end());
    vels.assign(velocities.begin(), velocities.end());


#ifdef DEBUG
    // test the relation between acc, vel, and pose
    std::vector<double> acc;
    for (size_t i = 0; i < vels.size()-2; ++i) {
        auto acc1 = (vels[i+1] - vels[i]) / delta_t;
        auto nextp = traj[i] + vels[i] * delta_t + acc1 * delta_t * delta_t * 0.5;
        acc.push_back((nextp - traj[i+1]).length());
    }
    std::cout << "end" << std::endl;
#endif

#ifdef DEBUG
    // visualization
    for (size_t i = 0; i < total_steps; i++) {
        Key pos_key = Symbol('x', i);
        auto pose = values.at<gtsam::Vector2>(pos_key);
        std::cout << pose.x() << ", " << pose.y() << std::endl;
    }
#endif

} // end optimization



} // end namespace
#endif
