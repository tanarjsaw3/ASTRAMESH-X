#include "astromesh/coordination/rendezvous_protocol.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::verification;
using namespace astromesh::coordination;

static bool approx_equal(double a, double b, double eps = 1e-3) {
    return std::abs(a - b) <= eps;
}

void test_wire_serialization_and_bounds() {
    std::cout << "[RUN] test_wire_serialization_and_bounds..." << std::endl;

    // Verify bounded struct sizes strictly under 1024 bytes
    assert(sizeof(RendezvousInquiry) <= 1024);
    assert(sizeof(RendezvousResponse) <= 1024);
    assert(sizeof(RendezvousProposal) <= 1024);

    // Test RendezvousInquiry serialization
    RendezvousInquiry inq;
    inq.opportunity_id = 101;
    inq.rover_src = 1;
    inq.rover_dst = 2;
    inq.keyframe_src = NodeId(1, 10);
    inq.keyframe_dst = NodeId(2, 20);
    inq.epoch = 5;
    inq.num_requested_landmarks = 2;
    inq.requested_landmarks[0] = LandmarkKey(1, 1, 1, 0xAAAA);
    inq.requested_landmarks[1] = LandmarkKey(1, 1, 2, 0xBBBB);

    std::vector<uint8_t> inq_buf;
    inq.serialize(inq_buf);
    assert(inq_buf.size() == sizeof(RendezvousInquiry));

    RendezvousInquiry inq_dec;
    assert(RendezvousInquiry::deserialize(inq_buf.data(), inq_buf.size(), inq_dec));
    assert(inq_dec.opportunity_id == 101);
    assert(inq_dec.rover_src == 1);
    assert(inq_dec.rover_dst == 2);
    assert(inq_dec.num_requested_landmarks == 2);
    assert(inq_dec.requested_landmarks[0] == LandmarkKey(1, 1, 1, 0xAAAA));

    // Test RendezvousProposal serialization
    RendezvousProposal prop;
    prop.opportunity_id = 101;
    prop.from_node = NodeId(2, 20);
    prop.to_node = NodeId(1, 10);
    prop.tx = 12.34;
    prop.ty = 56.78;
    prop.theta = 0.45;
    prop.inlier_count = 6;
    prop.inlier_rmse = 0.03;
    prop.confidence = 0.95;

    std::vector<uint8_t> prop_buf;
    prop.serialize(prop_buf);
    RendezvousProposal prop_dec;
    assert(RendezvousProposal::deserialize(prop_buf.data(), prop_buf.size(), prop_dec));
    assert(prop_dec.opportunity_id == 101);
    assert(approx_equal(prop_dec.tx, 12.34));
    assert(approx_equal(prop_dec.ty, 56.78));
    assert(approx_equal(prop_dec.theta, 0.45));
    assert(prop_dec.inlier_count == 6);

    std::cout << "[PASS] test_wire_serialization_and_bounds" << std::endl;
}

void test_end_to_end_rendezvous_exchange() {
    std::cout << "[RUN] test_end_to_end_rendezvous_exchange..." << std::endl;

    RendezvousProtocolAgent agent1(1);
    RendezvousProtocolAgent agent2(2);

    PoseGraph pg2;
    pg2.add_node(KeyframeNode(NodeId(2, 10), Pose2D(10.0, 5.0, 0.2)));

    // 5 mutual landmarks in both rover databases
    // Remote responder positions are offset by T_12: translation (10, 5), theta = 0
    std::map<LandmarkKey, Point2D> db1, db2;
    std::vector<LandmarkKey> candidate_keys;
    for (uint32_t i = 1; i <= 5; ++i) {
        LandmarkKey key(1, 100, i, 0x1000 + i);
        candidate_keys.push_back(key);

        double gx = static_cast<double>(i) * 3.0;
        double gy = static_cast<double>(i) * 2.0;

        // Rover 1 local position: (gx, gy)
        db1[key] = Point2D(gx, gy);

        // Rover 2 local position: (gx - 10.0, gy - 5.0)
        db2[key] = Point2D(gx - 10.0, gy - 5.0);
    }

    // Step 1: Rover 1 creates inquiry
    auto inq = agent1.create_inquiry(42, 2, NodeId(1, 5), NodeId(2, 10), candidate_keys);
    assert(agent1.metrics().inquiries_sent == 1);

    // Step 2: Rover 2 handles inquiry
    auto resp = agent2.handle_inquiry(inq, pg2, db2);
    assert(resp.status == 0); // Success
    assert(resp.num_landmarks == 5);
    assert(agent2.metrics().responses_sent == 1);

    // Step 3: Rover 1 processes response and runs local geometric verification
    auto prop_opt = agent1.process_response(resp, db1);
    assert(prop_opt.has_value());
    assert(prop_opt->inlier_count == 5);
    assert(prop_opt->inlier_rmse < 0.01);
    // Relative pose should be T_12 = (10, 5, 0)
    assert(approx_equal(prop_opt->tx, 10.0));
    assert(approx_equal(prop_opt->ty, 5.0));
    assert(approx_equal(prop_opt->theta, 0.0));

    // Verify wire byte accounting: inquiry + response + proposal
    assert(agent1.metrics().total_wire_bytes == sizeof(RendezvousInquiry) + sizeof(RendezvousProposal));
    assert(agent2.metrics().total_wire_bytes == sizeof(RendezvousResponse));

    std::cout << "[PASS] test_end_to_end_rendezvous_exchange" << std::endl;
}

void test_missing_keyframe_and_insufficient_landmarks() {
    std::cout << "[RUN] test_missing_keyframe_and_insufficient_landmarks..." << std::endl;

    RendezvousProtocolAgent agent1(1);
    RendezvousProtocolAgent agent2(2);

    PoseGraph empty_pg;
    std::map<LandmarkKey, Point2D> empty_db;

    // Target keyframe does not exist
    RendezvousInquiry inq;
    inq.opportunity_id = 99;
    inq.rover_src = 1;
    inq.rover_dst = 2;
    inq.keyframe_dst = NodeId(2, 999);

    auto resp = agent2.handle_inquiry(inq, empty_pg, empty_db);
    assert(resp.status == 1); // KeyframeNotFound

    auto prop = agent1.process_response(resp, empty_db);
    assert(!prop.has_value());

    std::cout << "[PASS] test_missing_keyframe_and_insufficient_landmarks" << std::endl;
}

int main() {
    std::cout << "=================================================" << std::endl;
    std::cout << " AstroMesh-X: Rendezvous Protocol Unit Tests" << std::endl;
    std::cout << "=================================================" << std::endl;

    test_wire_serialization_and_bounds();
    test_end_to_end_rendezvous_exchange();
    test_missing_keyframe_and_insufficient_landmarks();

    std::cout << "All Rendezvous Protocol tests PASSED." << std::endl;
    return 0;
}

