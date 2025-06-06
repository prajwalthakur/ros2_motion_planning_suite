
#ifndef QP_MPC_PLANNER_GET_OBS_POS_HPP_
#define QP_MPC_PLANNER_GET_OBS_POS_HPP_
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
/**
 * @brief Retrieve all obstacle poses within a given distance of the ego vehicle.
 *
 * This function computes the Euclidean distance from the ego pose to each obstacle 
 * stored in the member array `obs_pose`. Any obstacle whose distance is strictly 
 * less than `dist_threshold` is included in the returned list. Each obstacle pose 
 * is represented as a 3‐element row [x, y, θ].
 *
 * @param ego_pose
 *   A 3×1 array containing the ego vehicle’s pose in the form [x, y, θ].
 * @param dist_threshold
 *   A positive scalar. Only obstacles whose Euclidean distance to `ego_pose` is
 *   less than this value will be returned.
 *
 * @return
 *   An N×3 array, where N is the number of obstacles closer than `dist_threshold`. 
 *   Each row of the result corresponds to [x, y, θ] of one nearby obstacle. If no 
 *   obstacles lie within the threshold, an empty (0×3) array is returned.
 */
Eigen::ArrayXXf extract_near_by_obs(Eigen::Array3f& , float );
#endif