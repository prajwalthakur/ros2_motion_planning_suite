#include "qp_mpc_planner/get_obs_pose.hpp"
Eigen::ArrayXXf extract_near_by_obs(Eigen::Array3f& ego_pose, float dist_threshold, float horizon_length) {
    // get the dist to ego 
    Eigen::ArrayXXf obs_pose  = {{2.0,2.0,0.2},
                                {10.0,0.0,0.0},
                                {0.1,0.02,0.3},
                                {20,0.0,0.0},
                                {10,3.0,0.0},
                                {13.0,0.0,0.0},
                                {16.0,0.0,0.0},
                                };    
    Eigen::Array2f temp_ego = ego_pose.block(0, 0, 2, 1);
    Eigen::ArrayXXf temp_obs = obs_pose.block(0, 0, obs_pose.rows(), 2);

    Eigen::ArrayXf dist_to_ego =  (temp_obs.rowwise()  - temp_ego.transpose()).rowwise().norm();

    // compute dist - threshold for each obstacle
    Eigen::ArrayXf dist_under_threshold = dist_to_ego
        .binaryExpr(dist_threshold * Eigen::ArrayXf::Ones(dist_to_ego.rows()),
                    [](float d, float t) { return d - t; });
    
    // collect all obstacle rows whose (distance - threshold) < 0
    std::vector<std::vector<float>> filt_obs;
    for (int i = 0; i < dist_under_threshold.rows(); ++i) {
        if (dist_under_threshold(i) <= 0.05f) {
            filt_obs.push_back({ obs_pose(i, 0), obs_pose(i, 1), obs_pose(i, 2) });
        }
    }
    
    // copy into an ArrayXXf (N×3) result
    Eigen::ArrayXXf filtered_obs(filt_obs.size(), 3);
    int num_obs = static_cast<int>(filt_obs.size());
    for (int i = 0; i < num_obs; ++i) {
        filtered_obs(i, 0) = filt_obs[i][0];
        filtered_obs(i, 1) = filt_obs[i][1];
        filtered_obs(i, 2) = filt_obs[i][2];
    }
    Eigen::ArrayXXf pred_filtered_obs_traj = filtered_obs.replicate(1, static_cast<int>(horizon_length));
    return pred_filtered_obs_traj;
}