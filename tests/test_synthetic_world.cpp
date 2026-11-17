#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/simulation/swarm_trajectories.hpp"
#include "astromesh/simulation/observation_model.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <iostream>
#include <set>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;

static int g_total_assertions = 0;
static int g_failed_assertions = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        ++g_total_assertions; \
        if (!(cond)) { \
            ++g_failed_assertions; \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

// ----------------------------------------------------------------------
// Test 1: Arena and Configuration Validation
// ----------------------------------------------------------------------
void test_arena_config_validation() {
    std::cout << "[TEST 1] Arena and Configuration Validation...\n";

    WorldConfig valid_cfg;
    valid_cfg.min_x = -20.0;
    valid_cfg.max_x = 20.0;
    valid_cfg.min_y = -20.0;
    valid_cfg.max_y = 20.0;
    valid_cfg.num_landmarks = 50;
    valid_cfg.sensor_range = 10.0;
    valid_cfg.field_of_view_rad = M_PI;
    TEST_ASSERT(valid_cfg.is_valid(), "Valid config must return is_valid() == true");

    // Invalid bounds: min_x >= max_x
    WorldConfig inv_x = valid_cfg;
    inv_x.min_x = 30.0;
    TEST_ASSERT(!inv_x.is_valid(), "Inverted X bounds must be invalid");

    // Non-finite bounds
    WorldConfig nan_cfg = valid_cfg;
    nan_cfg.max_y = std::numeric_limits<double>::quiet_NaN();
    TEST_ASSERT(!nan_cfg.is_valid(), "NaN bounds must be invalid");

    // Invalid sensor range
    WorldConfig zero_range = valid_cfg;
    zero_range.sensor_range = 0.0;
    TEST_ASSERT(!zero_range.is_valid(), "Zero sensor range must be invalid");

    WorldConfig neg_range = valid_cfg;
    neg_range.sensor_range = -5.0;
    TEST_ASSERT(!neg_range.is_valid(), "Negative sensor range must be invalid");

    // Invalid FOV
    WorldConfig zero_fov = valid_cfg;
    zero_fov.field_of_view_rad = 0.0;
    TEST_ASSERT(!zero_fov.is_valid(), "Zero FOV must be invalid");

    WorldConfig excessive_fov = valid_cfg;
    excessive_fov.field_of_view_rad = 7.0; // > 2*pi
    TEST_ASSERT(!excessive_fov.is_valid(), "FOV > 2*pi must be invalid");

    // Constructor throws invalid_argument
    bool caught = false;
    try {
        SyntheticWorld2D bad_world(inv_x);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "SyntheticWorld2D must throw std::invalid_argument on invalid config");
}

// ----------------------------------------------------------------------
// Test 2: Landmark Generation Bounds and Uniqueness
// ----------------------------------------------------------------------
void test_landmark_generation_bounds() {
    std::cout << "[TEST 2] Landmark Generation Bounds and Uniqueness...\n";

    WorldConfig cfg;
    cfg.min_x = -30.0;
    cfg.max_x = 40.0;
    cfg.min_y = -15.0;
    cfg.max_y = 25.0;
    cfg.num_landmarks = 150;
    cfg.seed = 12345;

    SyntheticWorld2D world(cfg);
    TEST_ASSERT(world.num_landmarks() == 150, "World landmark count must match config");

    std::set<uint32_t> ids;
    std::set<uint64_t> desc_hashes;

    for (const auto& lm : world.landmarks()) {
        TEST_ASSERT(lm.position.x >= cfg.min_x && lm.position.x <= cfg.max_x,
                    "Landmark X must lie inside arena bounds");
        TEST_ASSERT(lm.position.y >= cfg.min_y && lm.position.y <= cfg.max_y,
                    "Landmark Y must lie inside arena bounds");
        TEST_ASSERT(lm.key.is_valid(), "Landmark key must be valid");
        TEST_ASSERT(lm.key.schema_version() == 1, "Landmark schema version must be 1");
        TEST_ASSERT(lm.key.descriptor_hash() != 0, "Descriptor hash must be non-zero");

        ids.insert(lm.id);
        desc_hashes.insert(lm.key.descriptor_hash());
    }

    TEST_ASSERT(ids.size() == 150, "All landmark IDs must be unique");
    TEST_ASSERT(desc_hashes.size() == 150, "All descriptor hashes must be unique");
}

// ----------------------------------------------------------------------
// Test 3: Identical-Seed Reproducibility and Different-Seed Divergence
// ----------------------------------------------------------------------
void test_seed_reproducibility() {
    std::cout << "[TEST 3] Seed Reproducibility...\n";

    WorldConfig cfg1;
    cfg1.num_landmarks = 80;
    cfg1.seed = 987654;

    WorldConfig cfg2 = cfg1; // Same seed
    WorldConfig cfg3 = cfg1;
    cfg3.seed = 111111;     // Different seed

    SyntheticWorld2D world1(cfg1);
    SyntheticWorld2D world2(cfg2);
    SyntheticWorld2D world3(cfg3);

    // world1 and world2 must be bit-for-bit identical
    bool identical = true;
    for (size_t i = 0; i < world1.num_landmarks(); ++i) {
        const auto& lm1 = world1.landmarks()[i];
        const auto& lm2 = world2.landmarks()[i];
        if (lm1.id != lm2.id ||
            lm1.position.x != lm2.position.x ||
            lm1.position.y != lm2.position.y ||
            lm1.key.descriptor_hash() != lm2.key.descriptor_hash()) {
            identical = false;
            break;
        }
    }
    TEST_ASSERT(identical, "Identical seed must produce 100% bit-for-bit identical landmarks");

    // world1 and world3 must differ
    bool different = false;
    for (size_t i = 0; i < world1.num_landmarks(); ++i) {
        if (world1.landmarks()[i].position.x != world3.landmarks()[i].position.x ||
            world1.landmarks()[i].position.y != world3.landmarks()[i].position.y) {
            different = true;
            break;
        }
    }
    TEST_ASSERT(different, "Different seeds must produce different landmark configurations");
}

// ----------------------------------------------------------------------
// Test 4: Rover Trajectory Timestamps and Finite SE(2) Poses
// ----------------------------------------------------------------------
void test_swarm_trajectories() {
    std::cout << "[TEST 4] Swarm Trajectory Generation...\n";

    TrajectoryConfig cfg;
    cfg.num_rovers = 3;
    cfg.duration_ns = 5'000'000'000ULL; // 5.0 s
    cfg.step_dt_ns = 100'000'000ULL;     // 100 ms
    cfg.speed = 1.2;
    cfg.arena_extent = 30.0;
    cfg.seed = 42;

    // Parallel search
    auto parallel_trajs = SwarmTrajectoryGenerator::generate_parallel_search(cfg);
    TEST_ASSERT(parallel_trajs.size() == 3, "Parallel search generates 3 rovers");
    for (const auto& tr : parallel_trajs) {
        TEST_ASSERT(tr.size() == 51, "5 s at 100 ms dt produces 51 waypoints");
        for (size_t s = 0; s < tr.size(); ++s) {
            const auto& wp = tr.waypoints[s];
            TEST_ASSERT(wp.is_valid(), "Waypoint pose and velocity must be valid");
            TEST_ASSERT(wp.timestamp_ns == s * cfg.step_dt_ns, "Timestamps must match step index");
            TEST_ASSERT(std::abs(wp.pose.x()) <= cfg.arena_extent * 1.5, "Poses must remain bounded");
        }
    }

    // Overlapping grid
    auto grid_trajs = SwarmTrajectoryGenerator::generate_overlapping_grid(cfg);
    TEST_ASSERT(grid_trajs.size() == 3, "Grid search generates 3 rovers");
    for (const auto& tr : grid_trajs) {
        TEST_ASSERT(tr.size() == 51, "Grid trajectory produces 51 waypoints");
        for (const auto& wp : tr.waypoints) {
            TEST_ASSERT(wp.is_valid(), "Grid waypoint must be valid");
        }
    }

    // Concentric loops
    auto loop_trajs = SwarmTrajectoryGenerator::generate_concentric_loops(cfg);
    TEST_ASSERT(loop_trajs.size() == 3, "Loop search generates 3 rovers");
    for (const auto& tr : loop_trajs) {
        TEST_ASSERT(tr.size() == 51, "Loop trajectory produces 51 waypoints");
        for (const auto& wp : tr.waypoints) {
            TEST_ASSERT(wp.is_valid(), "Loop waypoint must be valid");
        }
    }
}

// ----------------------------------------------------------------------
// Test 5: Sensor Range and Field-of-View Boundary Behavior
// ----------------------------------------------------------------------
void test_sensor_fov_boundaries() {
    std::cout << "[TEST 5] Sensor Range and Field-of-View Boundaries...\n";

    WorldConfig cfg;
    cfg.min_x = -20.0;
    cfg.max_x = 20.0;
    cfg.min_y = -20.0;
    cfg.max_y = 20.0;
    cfg.num_landmarks = 0; // Empty baseline; we'll query custom configurations
    cfg.sensor_range = 10.0;
    cfg.field_of_view_rad = M_PI * 0.5; // 90 degree FOV (+/- 45 deg)

    SyntheticWorld2D world(cfg);

    // Rover at origin heading +X (theta = 0)
    geometry::SE2 rover(0.0, 0.0, 0.0);

    // Construct a test world with controlled landmarks
    WorldConfig test_cfg = cfg;
    test_cfg.num_landmarks = 4;
    test_cfg.seed = 1;
    // We can test query on world with landmarks
    SyntheticWorld2D test_world(test_cfg);
    auto visible = test_world.query_visible_landmarks(rover);

    for (const auto& lm : visible) {
        const double d = std::hypot(lm.position.x, lm.position.y);
        TEST_ASSERT(d <= test_cfg.sensor_range + 1e-9, "Visible landmark must be within sensor range");
        const double bearing = std::atan2(lm.position.y, lm.position.x);
        TEST_ASSERT(std::abs(bearing) <= test_cfg.field_of_view_rad * 0.5 + 1e-9,
                    "Visible landmark must be within angular FOV cone");
    }
}

// ----------------------------------------------------------------------
// Test 6: Rover-Local vs World-Frame Coordinate Transformation
// ----------------------------------------------------------------------
void test_coordinate_frames() {
    std::cout << "[TEST 6] Coordinate Frame Transformations...\n";

    // Setup a world with 1 landmark placed at world (10, 25)
    WorldConfig cfg;
    cfg.min_x = 9.9;
    cfg.max_x = 10.1;
    cfg.min_y = 24.9;
    cfg.max_y = 25.1;
    cfg.num_landmarks = 1;
    cfg.sensor_range = 15.0;
    cfg.field_of_view_rad = 2.0 * M_PI; // Omnidirectional
    cfg.seed = 42;

    SyntheticWorld2D world(cfg);
    TEST_ASSERT(world.num_landmarks() == 1, "World has 1 landmark");

    // Rover positioned at (10, 20) with heading theta = +pi/2 (facing north)
    geometry::SE2 rover(10.0, 20.0, M_PI * 0.5);

    ObservationNoiseConfig noise_cfg;
    noise_cfg.range_std_dev = 0.0; // Perfect zero noise
    noise_cfg.bearing_std_dev = 0.0;

    ObservationModel2D model(noise_cfg);
    auto obs = model.observe(world, rover, 1);

    TEST_ASSERT(obs.size() == 1, "Must observe the single landmark");
    if (!obs.empty()) {
        const auto& o = obs[0];

        // The landmark is at (10, 25), 5 meters north of rover (10, 20).
        // Since rover is facing north (+pi/2), the landmark is directly ahead:
        // Local x = +5.0, Local y = 0.0, Bearing = 0.0 rad, Range = 5.0 m.
        constexpr double EPS = 1e-6;
        TEST_ASSERT(std::abs(o.ground_truth_range - 5.0) < 0.2, "Ground truth range must be ~5 m");
        TEST_ASSERT(std::abs(o.ground_truth_bearing) < 0.05, "Ground truth bearing must be ~0 rad (straight ahead)");
        TEST_ASSERT(std::abs(o.ground_truth_local_point.x - 5.0) < 0.2, "Local x must be ~+5 m forward");
        TEST_ASSERT(std::abs(o.ground_truth_local_point.y) < 0.2, "Local y must be ~0 m lateral");
    }
}

// ----------------------------------------------------------------------
// Test 7: Deterministic Noise and Zero-Noise Behavior
// ----------------------------------------------------------------------
void test_noise_behavior() {
    std::cout << "[TEST 7] Noise and Zero-Noise Properties...\n";

    WorldConfig wcfg;
    wcfg.min_x = -30.0;
    wcfg.max_x = 30.0;
    wcfg.min_y = -30.0;
    wcfg.max_y = 30.0;
    wcfg.num_landmarks = 40;
    wcfg.sensor_range = 25.0;
    wcfg.seed = 42;
    SyntheticWorld2D world(wcfg);

    geometry::SE2 rover(0.0, 0.0, 0.0);

    // 1. Zero noise test
    ObservationNoiseConfig zero_noise;
    zero_noise.range_std_dev = 0.0;
    zero_noise.bearing_std_dev = 0.0;

    ObservationModel2D model_clean(zero_noise);
    auto clean_obs = model_clean.observe(world, rover);

    for (const auto& o : clean_obs) {
        TEST_ASSERT(o.measured_range == o.ground_truth_range, "Measured range must match ground truth with zero noise");
        TEST_ASSERT(o.measured_bearing == o.ground_truth_bearing, "Measured bearing must match ground truth with zero noise");
        TEST_ASSERT(o.measured_local_point == o.ground_truth_local_point, "Measured local point must match ground truth with zero noise");
    }

    // 2. Deterministic noise test (identical seeds)
    ObservationNoiseConfig noisy_cfg1;
    noisy_cfg1.range_std_dev = 0.10;
    noisy_cfg1.bearing_std_dev = 0.05;
    noisy_cfg1.seed = 777;

    ObservationNoiseConfig noisy_cfg2 = noisy_cfg1; // Same seed

    ObservationModel2D model1(noisy_cfg1);
    ObservationModel2D model2(noisy_cfg2);

    auto obs1 = model1.observe(world, rover);
    auto obs2 = model2.observe(world, rover);

    TEST_ASSERT(obs1.size() == obs2.size(), "Observation counts must match");
    bool identical_noise = true;
    for (size_t i = 0; i < obs1.size(); ++i) {
        if (obs1[i].measured_range != obs2[i].measured_range ||
            obs1[i].measured_bearing != obs2[i].measured_bearing) {
            identical_noise = false;
            break;
        }
    }
    TEST_ASSERT(identical_noise, "Same seed must produce bit-for-bit identical noise realizations");
}

// ----------------------------------------------------------------------
// Test 8: Invalid and Non-Finite Inputs
// ----------------------------------------------------------------------
void test_invalid_inputs() {
    std::cout << "[TEST 8] Invalid and Non-Finite Inputs...\n";

    WorldConfig wcfg;
    SyntheticWorld2D world(wcfg);

    ObservationNoiseConfig bad_noise;
    bad_noise.range_std_dev = -1.0;
    TEST_ASSERT(!bad_noise.is_valid(), "Negative range noise must be invalid");

    bool caught = false;
    try {
        ObservationModel2D bad_model(bad_noise);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "ObservationModel2D must throw on invalid noise config");

    ObservationNoiseConfig valid_noise;
    ObservationModel2D model(valid_noise);

    // Non-finite rover pose returns empty vector safely
    geometry::SE2 nan_pose(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    auto nan_obs = model.observe(world, nan_pose);
    TEST_ASSERT(nan_obs.empty(), "Non-finite rover pose must return empty observation vector");
}

// ----------------------------------------------------------------------
// Test 9: Boundary and Empty World Cases
// ----------------------------------------------------------------------
void test_boundary_and_empty_world() {
    std::cout << "[TEST 9] Boundary and Empty World Cases...\n";

    WorldConfig empty_cfg;
    empty_cfg.num_landmarks = 0;
    SyntheticWorld2D empty_world(empty_cfg);

    TEST_ASSERT(empty_world.num_landmarks() == 0, "Empty world has 0 landmarks");
    auto obs = empty_world.query_visible_landmarks(geometry::SE2(0.0, 0.0, 0.0));
    TEST_ASSERT(obs.empty(), "Empty world must return 0 visible landmarks");

    // Rover placed 1000 meters outside sensor range of normal world
    WorldConfig normal_cfg;
    normal_cfg.min_x = -10.0;
    normal_cfg.max_x = 10.0;
    normal_cfg.min_y = -10.0;
    normal_cfg.max_y = 10.0;
    normal_cfg.sensor_range = 5.0;

    SyntheticWorld2D normal_world(normal_cfg);
    geometry::SE2 far_away_rover(1000.0, 1000.0, 0.0);
    auto far_obs = normal_world.query_visible_landmarks(far_away_rover);
    TEST_ASSERT(far_obs.empty(), "Rover outside range must observe 0 landmarks");
}

// ----------------------------------------------------------------------
// Test 10: Repeated Queries and Correspondence Extraction
// ----------------------------------------------------------------------
void test_correspondence_extraction() {
    std::cout << "[TEST 10] Repeated Queries and Geometric Correspondence Extraction...\n";

    WorldConfig wcfg;
    wcfg.min_x = -20.0;
    wcfg.max_x = 20.0;
    wcfg.min_y = -20.0;
    wcfg.max_y = 20.0;
    wcfg.num_landmarks = 30;
    wcfg.sensor_range = 15.0;
    wcfg.seed = 42;

    SyntheticWorld2D world(wcfg);

    ObservationNoiseConfig noise_cfg;
    noise_cfg.range_std_dev = 0.02;
    noise_cfg.bearing_std_dev = 0.01;
    ObservationModel2D model(noise_cfg);

    // Two rovers with overlapping fields of view near origin
    geometry::SE2 pose_a(-1.0, 0.0, 0.0);
    geometry::SE2 pose_b( 1.0, 0.0, 0.1);

    auto obs_a = model.observe(world, pose_a, 1);
    auto obs_b = model.observe(world, pose_b, 2);

    TEST_ASSERT(!obs_a.empty() && !obs_b.empty(), "Both rovers must observe landmarks");

    // Extract correspondences
    auto corrs = ObservationModel2D::extract_correspondences(obs_a, obs_b);
    TEST_ASSERT(!corrs.empty(), "Should extract non-empty correspondences for co-observed landmarks");

    for (const auto& c : corrs) {
        TEST_ASSERT(c.source.is_valid(), "Source point in correspondence must be valid");
        TEST_ASSERT(c.target.is_valid(), "Target point in correspondence must be valid");
        TEST_ASSERT(c.landmark_key.has_value(), "Correspondence must carry landmark key");
    }

    // Verify Umeyama geometric verification on extracted correspondences
    if (corrs.size() >= 4) {
        verification::RansacConfig2D rcfg;
        rcfg.min_correspondences = 4;
        rcfg.min_inliers = 4;
        rcfg.inlier_distance_threshold = 0.25;

        auto vres = verification::verify_relative_pose_2d(corrs, rcfg);
        TEST_ASSERT(vres.success, "Geometric verifier must succeed on co-observed correspondences");
    }
}

// ----------------------------------------------------------------------
// Main Entry Point
// ----------------------------------------------------------------------
int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 5.1: Deterministic Synthetic World & Trajectories\n";
    std::cout << "======================================================================\n";

    test_arena_config_validation();
    test_landmark_generation_bounds();
    test_seed_reproducibility();
    test_swarm_trajectories();
    test_sensor_fov_boundaries();
    test_coordinate_frames();
    test_noise_behavior();
    test_invalid_inputs();
    test_boundary_and_empty_world();
    test_correspondence_extraction();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions == 0) {
        std::cout << "SUCCESS: All synthetic world and observation tests PASSED.\n";
        return 0;
    } else {
        std::cerr << "FAILURE: " << g_failed_assertions << " assertion(s) failed.\n";
        return 1;
    }
}
