#pragma once

#include "ddk/aliro/UwbRangingChannel.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

// Forward declarations from uwb_core
namespace uwb::hal {
class ISpiDevice;
class IGpioPin;
class IClock;
class IAsyncExecutor;
class ILogger;
}  // namespace uwb::hal

namespace uwb::crypto {
class ICryptoProvider;
class CccKeyDerivationEngine;
class Sp0SecurityEngine;
}  // namespace uwb::crypto

namespace uwb::ranging {
class RangeConsensusFilter;
}

namespace uwb::transceiver {
class DW3000Controller;
}

namespace uwb::session {
class RangingSession;
struct RangingResult;
}  // namespace uwb::session

namespace aliro_uwb {

class UwbDw3000Channel : public ddk::aliro::UwbRangingChannel {
public:
    // AoA is 1/100 deg, valid only when AoA is enabled (Kconfig UWB_AOA_ENABLE) and the
    // transceiver is a dual-antenna part with STS PDoA locked for the measurement.
    // nlos: the Final frame's first-path-power diagnostics classified this sample as
    // NLOS (Kconfig UWB_NLOS_ENABLE); such samples carry a positively-biased distance.
    using RangeCallback = std::function<void(int32_t distance_cm, int64_t age_ms,
                                             int32_t aoa_centi_degrees, bool aoa_valid,
                                             bool nlos)>;

    UwbDw3000Channel();
    ~UwbDw3000Channel() override;

    void set_range_callback(RangeCallback cb) { range_cb_ = std::move(cb); }

    // Unilaterally suspend the live ranging session: stops the
    // responder and tells the phone via the Ranging Session Suspended notification.
    // The phone re-initiates (ranging attribute 0 -> M1) when it returns.
    void suspend_and_notify();

    void set_sender(std::function<bool(ddk::span<const uint8_t> frame)> send) override;
    void arm(uint32_t session_id, ddk::span<const uint8_t> ursk,
             const std::array<uint8_t, 2>& selected_version) override;
    void handle_frame(ddk::span<const uint8_t> frame) override;
    void poll() override;
    bool ranging_active() const override;
    void stop() override;

private:
    RangeCallback range_cb_;
    std::function<bool(ddk::span<const uint8_t> frame)> send_;

    bool armed_ = false;
    uint32_t session_id_ = 0;
    std::array<std::byte, 32> ursk_{};
    std::array<uint8_t, 2> selected_version_{0x01, 0x00};

    mutable std::mutex range_mutex_;
    int32_t latest_distance_cm_ = 0;
    int32_t latest_aoa_centi_degrees_ = 0;
    bool latest_aoa_valid_ = false;
    bool latest_nlos_ = false;
    uint64_t latest_range_timestamp_ms_ = 0;
    uint32_t latest_generation_ = 0;
    uint32_t last_reported_generation_ = 0;
    std::atomic<bool> is_ranging_active_{false};

    void handle_ranging_service_frame(uint8_t msg_id, std::span<const std::byte> frame);
    void handle_notification_frame(uint8_t msg_id, std::span<const std::byte> frame);
    bool transmit_frame(std::span<const std::byte> frame);

    void on_ranging_measurement(const uwb::session::RangingResult& result);
};

bool init();

}  // namespace aliro_uwb
