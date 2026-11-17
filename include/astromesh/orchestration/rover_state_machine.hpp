#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace astromesh::orchestration {

/**
 * @brief Operational states of an exploration rover in the collective.
 *
 * NOTE: These states represent operational and communication conditions,
 * NOT metric localization accuracy or convergence guarantees.
 */
enum class RoverState : uint8_t {
    Initializing = 0,            ///< Rover booting, initializing sensors and initial keyframe.
    Exploring = 1,               ///< Actively traversing, taking keyframes, nominal peer exchange.
    CommunicationDegraded = 2,   ///< Elevated latency, packet loss, or missed heartbeats detected.
    Disconnected = 3,            ///< Sustained communication timeout; peer links severed.
    Reconnecting = 4,            ///< Inbound packet or link recovery detected; beginning handshake.
    Synchronizing = 5,           ///< Exchanging IBLT differences, verifying candidate loop closures.
    Optimizing = 6,              ///< Executing distributed CORD optimization iterations.
    ResourceConstrained = 7,     ///< DTN buffer saturation or byte quota exhausted.
    Stopped = 8                  ///< Rover halted safely (operator halt, mission complete, or shutdown).
};

const char* to_string(RoverState state) noexcept;
std::ostream& operator<<(std::ostream& os, RoverState state);

/**
 * @brief Cause or trigger for a rover state transition.
 */
enum class TransitionReason : uint8_t {
    InitializationComplete,
    OdometryAdvancement,
    HeartbeatTimeout,
    LinkLossDetected,
    LinkQualityDegraded,
    LinkQualityRestored,
    PacketReceivedInDisconnected,
    ReconnectionHandshakeStarted,
    SyncInitiated,
    SyncCompletedSuccessfully,
    SyncFailedTimeout,
    SyncFailedDecode,
    OptimizationTriggered,
    OptimizationCompleted,
    QueueCapacityExceeded,
    ByteQuotaExhausted,
    ResourceConstraintCleared,
    ManualHalt,
    SystemReboot,
    GenericEvent
};

const char* to_string(TransitionReason reason) noexcept;
std::ostream& operator<<(std::ostream& os, TransitionReason reason);

/**
 * @brief Diagnostic telemetry record of a state transition attempt.
 */
struct StateTransitionRecord {
    RoverState from_state{RoverState::Initializing};
    RoverState to_state{RoverState::Initializing};
    TransitionReason reason{TransitionReason::InitializationComplete};
    std::string details;
    uint64_t timestamp_ns{0};
    bool success{false};
    bool idempotent{false};
};

/**
 * @brief Configuration parameters for the rover lifecycle state machine.
 */
struct RoverStateMachineConfig {
    uint64_t degradation_timeout_ns{500'000'000};  ///< 500 ms without peer traffic -> Degraded.
    uint64_t disconnection_timeout_ns{1'500'000'000}; ///< 1500 ms without peer traffic -> Disconnected.
    uint64_t sync_timeout_ns{2'000'000'000};       ///< 2000 ms max in Synchronizing.
    size_t max_history_records{1000};              ///< Maximum transition records to retain.
};

/**
 * @brief Deterministic, event-driven lifecycle state machine for a simulated rover.
 */
class RoverStateMachine {
public:
    explicit RoverStateMachine(uint16_t rover_id, const RoverStateMachineConfig& config = {}) noexcept;

    uint16_t rover_id() const noexcept { return rover_id_; }
    RoverState current_state() const noexcept { return current_state_; }
    RoverState previous_state() const noexcept { return previous_state_; }
    uint64_t state_entered_time_ns() const noexcept { return state_entered_time_ns_; }

    uint64_t time_in_current_state_ns(uint64_t current_time_ns) const noexcept {
        return (current_time_ns >= state_entered_time_ns_) ? (current_time_ns - state_entered_time_ns_) : 0;
    }

    const RoverStateMachineConfig& config() const noexcept { return config_; }

    /**
     * @brief Checks if a transition between two states is permitted by policy.
     */
    static bool is_transition_legal(RoverState from, RoverState to) noexcept;

    /**
     * @brief Attempts a state transition.
     * @param target_state Target state to enter.
     * @param reason The trigger or event causing the transition.
     * @param timestamp_ns Simulated monotonic timestamp of the event.
     * @param details Optional diagnostic context string.
     * @return true if transition was accepted (or was an idempotent no-op), false if rejected.
     */
    bool transition_to(
        RoverState target_state,
        TransitionReason reason,
        uint64_t timestamp_ns,
        const std::string& details = "");

    /**
     * @brief Inspects peer traffic activity and enforces timeout-driven transitions.
     * @param current_time_ns Current simulated timestamp.
     * @param last_peer_contact_ns Monotonic timestamp of most recent inbound packet from any peer.
     * @param is_resource_constrained true if queue or byte quotas are exhausted.
     */
    void evaluate_timeouts(
        uint64_t current_time_ns,
        uint64_t last_peer_contact_ns,
        bool is_resource_constrained);

    // --- History & Telemetry ---

    const std::vector<StateTransitionRecord>& transition_history() const noexcept {
        return history_;
    }

    size_t total_transitions() const noexcept { return total_transitions_; }
    size_t rejected_transitions() const noexcept { return rejected_transitions_; }
    size_t idempotent_transitions() const noexcept { return idempotent_transitions_; }

    void clear_history() noexcept { history_.clear(); }

private:
    uint16_t rover_id_{0};
    RoverStateMachineConfig config_;
    RoverState current_state_{RoverState::Initializing};
    RoverState previous_state_{RoverState::Initializing};
    uint64_t state_entered_time_ns_{0};

    std::vector<StateTransitionRecord> history_;
    size_t total_transitions_{0};
    size_t rejected_transitions_{0};
    size_t idempotent_transitions_{0};

    void record_transition(
        RoverState from, RoverState to, TransitionReason reason,
        const std::string& details, uint64_t timestamp_ns, bool success, bool idempotent);
};

} // namespace astromesh::orchestration
