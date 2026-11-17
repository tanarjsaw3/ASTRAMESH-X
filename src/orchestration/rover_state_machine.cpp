#include "astromesh/orchestration/rover_state_machine.hpp"

#include <iostream>

namespace astromesh::orchestration {

const char* to_string(RoverState state) noexcept {
    switch (state) {
        case RoverState::Initializing: return "Initializing";
        case RoverState::Exploring: return "Exploring";
        case RoverState::CommunicationDegraded: return "CommunicationDegraded";
        case RoverState::Disconnected: return "Disconnected";
        case RoverState::Reconnecting: return "Reconnecting";
        case RoverState::Synchronizing: return "Synchronizing";
        case RoverState::Optimizing: return "Optimizing";
        case RoverState::ResourceConstrained: return "ResourceConstrained";
        case RoverState::Stopped: return "Stopped";
    }
    return "Unknown";
}

std::ostream& operator<<(std::ostream& os, RoverState state) {
    return os << to_string(state);
}

const char* to_string(TransitionReason reason) noexcept {
    switch (reason) {
        case TransitionReason::InitializationComplete: return "InitializationComplete";
        case TransitionReason::OdometryAdvancement: return "OdometryAdvancement";
        case TransitionReason::HeartbeatTimeout: return "HeartbeatTimeout";
        case TransitionReason::LinkLossDetected: return "LinkLossDetected";
        case TransitionReason::LinkQualityDegraded: return "LinkQualityDegraded";
        case TransitionReason::LinkQualityRestored: return "LinkQualityRestored";
        case TransitionReason::PacketReceivedInDisconnected: return "PacketReceivedInDisconnected";
        case TransitionReason::ReconnectionHandshakeStarted: return "ReconnectionHandshakeStarted";
        case TransitionReason::SyncInitiated: return "SyncInitiated";
        case TransitionReason::SyncCompletedSuccessfully: return "SyncCompletedSuccessfully";
        case TransitionReason::SyncFailedTimeout: return "SyncFailedTimeout";
        case TransitionReason::SyncFailedDecode: return "SyncFailedDecode";
        case TransitionReason::OptimizationTriggered: return "OptimizationTriggered";
        case TransitionReason::OptimizationCompleted: return "OptimizationCompleted";
        case TransitionReason::QueueCapacityExceeded: return "QueueCapacityExceeded";
        case TransitionReason::ByteQuotaExhausted: return "ByteQuotaExhausted";
        case TransitionReason::ResourceConstraintCleared: return "ResourceConstraintCleared";
        case TransitionReason::ManualHalt: return "ManualHalt";
        case TransitionReason::SystemReboot: return "SystemReboot";
        case TransitionReason::GenericEvent: return "GenericEvent";
    }
    return "Unknown";
}

std::ostream& operator<<(std::ostream& os, TransitionReason reason) {
    return os << to_string(reason);
}

bool RoverStateMachine::is_transition_legal(RoverState from, RoverState to) noexcept {
    // Self-transition is always legal (handled idempotently).
    if (from == to) {
        return true;
    }

    // Any state can transition to Stopped upon manual halt or shutdown.
    if (to == RoverState::Stopped) {
        return true;
    }

    // Stopped rover can only restart into Initializing.
    if (from == RoverState::Stopped) {
        return to == RoverState::Initializing;
    }

    switch (from) {
        case RoverState::Initializing:
            return to == RoverState::Exploring;

        case RoverState::Exploring:
            return to == RoverState::CommunicationDegraded ||
                   to == RoverState::Disconnected ||
                   to == RoverState::Optimizing ||
                   to == RoverState::ResourceConstrained;

        case RoverState::CommunicationDegraded:
            return to == RoverState::Exploring ||
                   to == RoverState::Disconnected ||
                   to == RoverState::Optimizing ||
                   to == RoverState::ResourceConstrained;

        case RoverState::Disconnected:
            return to == RoverState::Reconnecting ||
                   to == RoverState::ResourceConstrained;

        case RoverState::Reconnecting:
            return to == RoverState::Synchronizing ||
                   to == RoverState::Disconnected ||
                   to == RoverState::Exploring ||
                   to == RoverState::ResourceConstrained;

        case RoverState::Synchronizing:
            return to == RoverState::Exploring ||
                   to == RoverState::Optimizing ||
                   to == RoverState::Disconnected ||
                   to == RoverState::ResourceConstrained;

        case RoverState::Optimizing:
            return to == RoverState::Exploring ||
                   to == RoverState::CommunicationDegraded ||
                   to == RoverState::Disconnected ||
                   to == RoverState::ResourceConstrained;

        case RoverState::ResourceConstrained:
            return to == RoverState::Exploring ||
                   to == RoverState::CommunicationDegraded ||
                   to == RoverState::Disconnected;

        case RoverState::Stopped:
            return to == RoverState::Initializing;
    }

    return false;
}

RoverStateMachine::RoverStateMachine(uint16_t rover_id, const RoverStateMachineConfig& config) noexcept
    : rover_id_(rover_id),
      config_(config),
      current_state_(RoverState::Initializing),
      previous_state_(RoverState::Initializing),
      state_entered_time_ns_(0) {}

bool RoverStateMachine::transition_to(
    RoverState target_state,
    TransitionReason reason,
    uint64_t timestamp_ns,
    const std::string& details) {

    // Idempotent no-op case:
    if (target_state == current_state_) {
        idempotent_transitions_++;
        record_transition(current_state_, target_state, reason, details, timestamp_ns, true, true);
        return true;
    }

    // Validate legal transition policy:
    if (!is_transition_legal(current_state_, target_state)) {
        rejected_transitions_++;
        record_transition(current_state_, target_state, reason, details, timestamp_ns, false, false);
        return false;
    }

    // Apply state transition:
    previous_state_ = current_state_;
    current_state_ = target_state;
    state_entered_time_ns_ = timestamp_ns;
    total_transitions_++;

    record_transition(previous_state_, current_state_, reason, details, timestamp_ns, true, false);
    return true;
}

void RoverStateMachine::evaluate_timeouts(
    uint64_t current_time_ns,
    uint64_t last_peer_contact_ns,
    bool is_resource_constrained) {

    // Stopped or Initializing rovers do not process communication timeouts.
    if (current_state_ == RoverState::Stopped || current_state_ == RoverState::Initializing) {
        return;
    }

    // Resource constraint check:
    if (is_resource_constrained && current_state_ != RoverState::ResourceConstrained) {
        transition_to(RoverState::ResourceConstrained, TransitionReason::ByteQuotaExhausted, current_time_ns,
                      "Resource budget or buffer capacity exceeded");
        return;
    }

    if (!is_resource_constrained && current_state_ == RoverState::ResourceConstrained) {
        transition_to(RoverState::Exploring, TransitionReason::ResourceConstraintCleared, current_time_ns,
                      "Resource constraint resolved");
        return;
    }

    // Synchronizing timeout guard:
    if (current_state_ == RoverState::Synchronizing) {
        if (time_in_current_state_ns(current_time_ns) >= config_.sync_timeout_ns) {
            transition_to(RoverState::Disconnected, TransitionReason::SyncFailedTimeout, current_time_ns,
                          "Synchronization duration exceeded timeout limit");
            return;
        }
    }

    // Peer communication timeout evaluation:
    uint64_t silence_duration = (last_peer_contact_ns > 0 && current_time_ns >= last_peer_contact_ns)
        ? (current_time_ns - last_peer_contact_ns)
        : current_time_ns;

    if (silence_duration >= config_.disconnection_timeout_ns) {
        if (current_state_ != RoverState::Disconnected) {
            transition_to(RoverState::Disconnected, TransitionReason::HeartbeatTimeout, current_time_ns,
                          "Silence duration exceeded disconnection threshold");
        }
    } else if (silence_duration >= config_.degradation_timeout_ns) {
        if (current_state_ == RoverState::Exploring) {
            transition_to(RoverState::CommunicationDegraded, TransitionReason::LinkQualityDegraded, current_time_ns,
                          "Silence duration exceeded degradation threshold");
        }
    } else {
        // Quality restored:
        if (current_state_ == RoverState::CommunicationDegraded) {
            transition_to(RoverState::Exploring, TransitionReason::LinkQualityRestored, current_time_ns,
                          "Peer communication restored within normal limits");
        }
    }
}

void RoverStateMachine::record_transition(
    RoverState from, RoverState to, TransitionReason reason,
    const std::string& details, uint64_t timestamp_ns, bool success, bool idempotent) {

    if (history_.size() >= config_.max_history_records) {
        history_.erase(history_.begin());
    }

    history_.push_back({from, to, reason, details, timestamp_ns, success, idempotent});
}

} // namespace astromesh::orchestration
