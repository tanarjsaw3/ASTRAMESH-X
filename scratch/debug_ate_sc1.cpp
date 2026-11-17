#include "astromesh/simulation/pipeline_astromesh_x.hpp"
#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/simulation/trajectory_generator.hpp"
#include "astromesh/simulation/observation_model.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include <iostream>

using namespace astromesh;

int main() {
    uint64_t seed = 1042ULL;
    simulation::WorldConfig wcfg;
    wcfg.num_landmarks = 100;
    wcfg.sensor_range = 18.0;
    wcfg.arena_extent = 30.0;
    wcfg.seed = seed ^ 0x9e3779b97f4a7c15ULL;
    simulation::SyntheticWorld2D world(wcfg);

    simulation::TrajectoryConfig tcfg;
    tcfg.num_rovers = 2;
    tcfg.duration_ns = 8'000'000'000ULL;
    tcfg.step_dt_ns = 200'000'000ULL;
    tcfg.speed = 2.0;
    tcfg.arena_extent = 30.0;
    tcfg.seed = seed ^ 0x517cc1b727220a95ULL;
    auto trajectories = simulation::SwarmTrajectoryGenerator::generate_concentric_loops(tcfg);

    const size_t num_steps = static_cast<size_t>(tcfg.duration_ns / tcfg.step_dt_ns) + 1;

    simulation::ObservationNoiseConfig obs_noise;
    obs_noise.range_std_dev = 0.05;
    obs_noise.bearing_std_dev = 0.01;
    obs_noise.seed = seed ^ 0x3141592653589793ULL;
    simulation::ObservationModel2D obs_model(obs_noise);

    std::vector<simulation::SimulationStepInput> step_inputs;
    for (size_t s = 0; s < num_steps; ++s) {
        simulation::SimulationStepInput step;
        step.timestamp_ns = s * tcfg.step_dt_ns;
        for (size_t r = 0; r < 2; ++r) {
            simulation::RoverStepData rdata;
            rdata.rover_id = static_cast<uint16_t>(r + 1);
            rdata.timestamp_ns = step.timestamp_ns;
            if (s == 0) {
                rdata.odometry_step = geometry::SE2::identity();
            } else {
                rdata.odometry_step = trajectories[r].waypoints[s - 1].pose.inverse().compose(trajectories[r].waypoints[s].pose);
            }
            rdata.observations = obs_model.observe(world, trajectories[r].waypoints[s].pose, rdata.rover_id);
            step.rover_steps.push_back(rdata);
        }
        for (size_t i = 0; i < 2; ++i) {
            for (size_t j = i + 1; j < 2; ++j) {
                simulation::ChannelLinkState link;
                link.rover_a = static_cast<uint16_t>(i + 1);
                link.rover_b = static_cast<uint16_t>(j + 1);
                link.in_communication_range = true;
                link.is_partitioned = false;
                link.link_packet_loss_rate = 0.0;
                step.channel_states.push_back(link);
            }
        }
        step_inputs.push_back(step);
    }

    std::unordered_map<graph::NodeId, geometry::SE2> gt_pose_map;
    for (size_t r = 0; r < 2; ++r) {
        uint16_t rid = static_cast<uint16_t>(r + 1);
        for (size_t s = 0; s < num_steps; ++s) {
            gt_pose_map[graph::NodeId(rid, static_cast<uint32_t>(s))] = trajectories[r].waypoints[s].pose;
        }
    }

    simulation::AstroMeshXConfig cfg;
    cfg.seed = seed;
    cfg.custom_pipeline_name = "AstroMesh_X_RawExchange";
    cfg.discovery_mechanism = simulation::AstroMeshXConfig::DiscoveryMechanism::RawExchange;
    cfg.enable_cord = true;
    cfg.cord_options.max_steps = 40;
    cfg.cord_options.timestep = 0.02;
    simulation::AstroMeshXPipeline pipeline(cfg);

    for (const auto& step : step_inputs) {
        pipeline.ingest_step(step);
    }

    auto est_trajs_before = pipeline.get_estimated_trajectories();
    std::unordered_map<graph::NodeId, geometry::SE2> map_before;
    for (const auto& [rid, poses] : est_trajs_before) {
        for (size_t k = 0; k < poses.size(); ++k) {
            map_before[graph::NodeId(rid, static_cast<uint32_t>(k))] = poses[k];
        }
    }
    auto ate_before = optimization::compute_ate(map_before, gt_pose_map, true);

    std::cout << "Verified loop closures in RawExchange: " << pipeline.collect_metrics().loop_closures_verified << std::endl;
    std::cout << "ATE before optimize(): " << ate_before.translation.rmse << " m\n";

    bool opt_res = pipeline.optimize();
    std::cout << "optimize() return: " << (opt_res ? "true" : "false") << std::endl;
    std::cout << "CORD steps performed: " << pipeline.collect_metrics().optimization_steps_performed << std::endl;

    auto est_trajs_after = pipeline.get_estimated_trajectories();
    std::unordered_map<graph::NodeId, geometry::SE2> map_after;
    for (const auto& [rid, poses] : est_trajs_after) {
        for (size_t k = 0; k < poses.size(); ++k) {
            map_after[graph::NodeId(rid, static_cast<uint32_t>(k))] = poses[k];
        }
    }
    auto ate_after = optimization::compute_ate(map_after, gt_pose_map, true);
    std::cout << "ATE after optimize(): " << ate_after.translation.rmse << " m\n";

    double max_pose_diff = 0.0;
    for (const auto& [nid, p_bef] : map_before) {
        const auto& p_aft = map_after[nid];
        double diff = std::hypot(p_bef.x() - p_aft.x(), p_bef.y() - p_aft.y());
        if (diff > max_pose_diff) max_pose_diff = diff;
    }
    std::cout << "Max pose movement during CORD: " << max_pose_diff << " m\n";

    return 0;
}

