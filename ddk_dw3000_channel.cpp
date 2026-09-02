#include "include/ddk_dw3000_channel.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <sdkconfig.h>

#include <uwb/core/StatusCode.hpp>
#include <uwb/core/Types.hpp>
#include <uwb/crypto/CccKeyDerivationEngine.hpp>
#include <uwb/crypto/MbedTlsCryptoProvider.hpp>
#include <uwb/crypto/Sp0SecurityEngine.hpp>
#include <uwb/hal/IClock.hpp>
#include <uwb/hal/IGpioPin.hpp>
#include <uwb/hal/ILogger.hpp>
#include <uwb/hal/ISpiDevice.hpp>
#include <uwb/protocol/SetupMessageCodec.hpp>
#include <uwb/ranging/RangeConsensusFilter.hpp>
#include <uwb/session/RangingSession.hpp>
#include <uwb/transceiver/DW3000Controller.hpp>

extern "C" {
#include "deca_device_api.h"
}

#include <algorithm>
#include <cstring>
#include <memory>
#include <mutex>

static const char* TAG = "aliro_uwb";

namespace aliro_uwb {

namespace {

constexpr gpio_num_t PIN_DW3000_SCLK   = static_cast<gpio_num_t>(CONFIG_DW3000_SPI_CLK);
    static_assert(PIN_DW3000_SCLK != -1, "DW3000 SPI CLK not configured!");
constexpr gpio_num_t PIN_DW3000_MOSI   = static_cast<gpio_num_t>(CONFIG_DW3000_SPI_MOSI);
    static_assert(PIN_DW3000_MOSI != -1, "DW3000 SPI MOSI not configured!");
constexpr gpio_num_t PIN_DW3000_MISO   = static_cast<gpio_num_t>(CONFIG_DW3000_SPI_MISO);
    static_assert(PIN_DW3000_MISO != -1, "DW3000 SPI MISO not configured!");
constexpr gpio_num_t PIN_DW3000_CS     = static_cast<gpio_num_t>(CONFIG_DW3000_SPI_CS);
    static_assert(PIN_DW3000_CS != -1, "DW3000 SPI CS not configured!");
constexpr gpio_num_t PIN_DW3000_RST    = static_cast<gpio_num_t>(CONFIG_DW3000_GPIO_RESET);
    static_assert(PIN_DW3000_RST != -1, "DW3000 GPIO reset not configured!");
constexpr gpio_num_t PIN_DW3000_IRQ    = static_cast<gpio_num_t>(CONFIG_DW3000_GPIO_IRQ);
    static_assert(PIN_DW3000_IRQ != -1, "DW3000 GPIO IRQ not configured!");
constexpr gpio_num_t PIN_DW3000_WAKEUP = static_cast<gpio_num_t>(CONFIG_DW3000_GPIO_WAKEUP);
    static_assert(PIN_DW3000_WAKEUP != -1, "DW3000 GPIO wakeup not configured!");

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
constexpr BaseType_t DW3000_TASK_CORE = 1;
#else
constexpr BaseType_t DW3000_TASK_CORE = 0;
#endif

class EspClock final : public uwb::hal::IClock {
public:
    uint64_t getMonotonicTimeUs() const noexcept override {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

    uint64_t getMonotonicTimeMs() const noexcept override {
        return getMonotonicTimeUs() / 1000ULL;
    }

    uint32_t getCycleCount() const noexcept override {
        return static_cast<uint32_t>(esp_timer_get_time() * 240);
    }

    void sleepMs(uint32_t milliseconds) override {
        if (milliseconds > 0) {
            vTaskDelay(pdMS_TO_TICKS(milliseconds));
        }
    }

    void busyWaitUs(uint32_t microseconds) override {
        if (microseconds > 0) {
            esp_rom_delay_us(microseconds);
        }
    }
};

class EspLogger final : public uwb::hal::ILogger {
public:
    void log(uwb::hal::LogLevel level, std::string_view component, std::string_view message) override {
        switch (level) {
            case uwb::hal::LogLevel::Trace:
            case uwb::hal::LogLevel::Debug:
                ESP_LOGD(TAG, "[%.*s] %.*s", static_cast<int>(component.size()), component.data(),
                         static_cast<int>(message.size()), message.data());
                break;
            case uwb::hal::LogLevel::Info:
                ESP_LOGI(TAG, "[%.*s] %.*s", static_cast<int>(component.size()), component.data(),
                         static_cast<int>(message.size()), message.data());
                break;
            case uwb::hal::LogLevel::Warn:
                ESP_LOGW(TAG, "[%.*s] %.*s", static_cast<int>(component.size()), component.data(),
                         static_cast<int>(message.size()), message.data());
                break;
            case uwb::hal::LogLevel::Error:
                ESP_LOGE(TAG, "[%.*s] %.*s", static_cast<int>(component.size()), component.data(),
                         static_cast<int>(message.size()), message.data());
                break;
        }
    }
};

static WORD_ALIGNED_ATTR DRAM_ATTR uint8_t s_spiTxBuf[2048];
static WORD_ALIGNED_ATTR DRAM_ATTR uint8_t s_spiRxBuf[2048];

class EspSpiDevice final : public uwb::hal::ISpiDevice {
public:
    EspSpiDevice() {
        spi_bus_config_t buscfg{};
        buscfg.mosi_io_num = PIN_DW3000_MOSI;
        buscfg.miso_io_num = PIN_DW3000_MISO;
        buscfg.sclk_io_num = PIN_DW3000_SCLK;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 2048;

        spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

        gpio_config_t cs_cfg{};
        cs_cfg.pin_bit_mask = (1ULL << PIN_DW3000_CS);
        cs_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&cs_cfg);
        gpio_set_level(PIN_DW3000_CS, 1);

        spi_device_interface_config_t devcfg{};
        devcfg.mode = 0;
        devcfg.spics_io_num = -1;
        devcfg.queue_size = 1;
        devcfg.clock_speed_hz = 2000000;

        spi_bus_add_device(SPI2_HOST, &devcfg, &dev_slow_);
        devcfg.clock_speed_hz = CONFIG_DW3000_SPI_MAX_MHZ * 1000 * 1000;
        spi_bus_add_device(SPI2_HOST, &devcfg, &dev_fast_);
        active_handle_ = dev_slow_;
    }

    uwb::core::Result<void> setSpeed(uwb::hal::SpiSpeed speed) override {
        active_handle_ = (speed == uwb::hal::SpiSpeed::Slow2MHz) ? dev_slow_ : dev_fast_;
        return {};
    }

    uwb::core::Result<void> transfer(
        std::span<const std::byte> txHeader,
        std::span<const std::byte> txBody,
        std::span<std::byte> rxBody) override {

        std::lock_guard<std::mutex> lock(m_mutex);

        const size_t bodyLen = !txBody.empty() ? txBody.size() : rxBody.size();
        const size_t total = txHeader.size() + bodyLen;
        if (total == 0 || total > sizeof(s_spiTxBuf)) {
            return std::unexpected(uwb::core::StatusCode::BufferOverflow);
        }

        std::memcpy(s_spiTxBuf, txHeader.data(), txHeader.size());
        if (!txBody.empty()) {
            std::memcpy(s_spiTxBuf + txHeader.size(), txBody.data(), txBody.size());
        } else if (!rxBody.empty()) {
            std::memset(s_spiTxBuf + txHeader.size(), 0, rxBody.size());
        }

        gpio_set_level(PIN_DW3000_CS, 0);

        spi_transaction_t t{};
        t.length = total * 8;
        t.tx_buffer = s_spiTxBuf;
        t.rx_buffer = !rxBody.empty() ? s_spiRxBuf : nullptr;

        esp_err_t err = spi_device_polling_transmit(active_handle_, &t);
        gpio_set_level(PIN_DW3000_CS, 1);

        if (err != ESP_OK) {
            return std::unexpected(uwb::core::StatusCode::TransceiverError);
        }

        if (!rxBody.empty()) {
            std::memcpy(rxBody.data(), s_spiRxBuf + txHeader.size(), rxBody.size());
        }
        return {};
    }

    void setChipSelect(bool active) override {
        gpio_set_level(PIN_DW3000_CS, active ? 0 : 1);
    }

    void wakeupPulse() override {
        setChipSelect(true);
        esp_rom_delay_us(500);
        setChipSelect(false);
    }

private:
    std::mutex m_mutex;
    spi_device_handle_t dev_slow_{nullptr};
    spi_device_handle_t dev_fast_{nullptr};
    spi_device_handle_t active_handle_{nullptr};
};

class EspGpioPin final : public uwb::hal::IGpioPin {
public:
    explicit EspGpioPin(gpio_num_t pin) : pin_(pin) {}

    uwb::core::Result<void> setMode(uwb::hal::PinMode mode) override {
        gpio_config_t cfg{};
        cfg.pin_bit_mask = (1ULL << pin_);
        cfg.mode = (mode == uwb::hal::PinMode::Input) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;
        gpio_config(&cfg);
        return {};
    }

    void write(bool high) override {
        gpio_set_level(pin_, high ? 1 : 0);
    }

    bool read() const override {
        return gpio_get_level(pin_) != 0;
    }

    uwb::core::Result<void> attachInterrupt(uwb::hal::InterruptTrigger trigger, InterruptCallback callback) override {
        (void)trigger;
        (void)callback;
        return {};
    }

    void enableInterrupt() override {
        gpio_intr_enable(pin_);
    }

    void disableInterrupt() override {
        gpio_intr_disable(pin_);
    }

private:
    gpio_num_t pin_;
};

static SemaphoreHandle_t s_dw3000_irq_sem = nullptr;
static TaskHandle_t s_dw3000_irq_task_handle = nullptr;

static void IRAM_ATTR dw3000_gpio_isr(void* arg) {
    (void)arg;
    BaseType_t hpw = pdFALSE;
    if (s_dw3000_irq_sem) {
        xSemaphoreGiveFromISR(s_dw3000_irq_sem, &hpw);
        if (hpw) {
            portYIELD_FROM_ISR();
        }
    }
}

static void dw3000_irq_worker(void* arg) {
    (void)arg;
    while (true) {
        if (xSemaphoreTake(s_dw3000_irq_sem, portMAX_DELAY) == pdTRUE) {
            uint32_t spins = 0;
            while (gpio_get_level(PIN_DW3000_IRQ) != 0 && spins++ < 30) {
                dwt_isr();
            }
            if (spins >= 30 && gpio_get_level(PIN_DW3000_IRQ) != 0) {
                // Lingered IRQ: clear only the bits still pending — a blind full clear
                // can wipe an RXFCG/RXFR that arrived between the last dwt_isr spin and
                // this check.
                const uint32_t pending = dwt_readsysstatuslo();
                if (pending != 0) {
                    dwt_writesysstatuslo(pending);
                }
            }
        }
    }
}

struct GlobalHardwareContext {
    EspClock clock;
    EspLogger logger;
    EspSpiDevice spi;
    EspGpioPin irqPin{PIN_DW3000_IRQ};
    EspGpioPin resetPin{PIN_DW3000_RST};
    uwb::crypto::MbedTlsCryptoProvider cryptoProvider;
    uwb::crypto::CccKeyDerivationEngine kdf{cryptoProvider};
    uwb::crypto::Sp0SecurityEngine sp0{cryptoProvider};
    uwb::ranging::RangeConsensusFilter filter{};
    std::unique_ptr<uwb::transceiver::DW3000Controller> transceiver;
    std::unique_ptr<uwb::session::RangingSession> session;
    uwb::protocol::setup::DeviceCapabilities capabilities{};
    uwb::protocol::setup::RangingSessionParameters parameters{};
    bool initialized = false;
};

GlobalHardwareContext* s_hw = nullptr;
UwbDw3000Channel* s_active_channel = nullptr;

}  // namespace

bool init()
{
    if (s_hw != nullptr && s_hw->initialized) {
        return true;
    }

    if (s_hw == nullptr) {
        s_hw = new GlobalHardwareContext();
    }

    gpio_config_t wk_cfg{};
    wk_cfg.pin_bit_mask = (1ULL << PIN_DW3000_WAKEUP);
    wk_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&wk_cfg);
    gpio_set_level(PIN_DW3000_WAKEUP, 1);
    esp_rom_delay_us(2000);

    s_hw->capabilities.supportedConfigIds = {0x0000};
    s_hw->capabilities.supportedPulseShapes = {0x00};
    s_hw->capabilities.channelBitmask = 0x03;               // Ch5 + Ch9
    s_hw->capabilities.slotBitmask = 0xFF;                  // All chaps configurations supported
    s_hw->capabilities.syncCodeBitmask = 0x00000F00;        // SYNC codes 9..12
    s_hw->capabilities.hoppingConfigBitmask = 0x1E;         // NoHopping + Continuous + Adaptive (all default-seq)
    s_hw->capabilities.minRanMultiplier = 1;
    // MAC Mode: 0x00 single round, or 0x40|Ok two rounds.
    // Ok=0 means "decide at M3 time as N_Round/2"; the exact Ok value sent to the
    // phone is finalized in handle_ranging_service_frame once N_Round is known.
#ifdef CONFIG_UWB_TWO_ROUNDS_ENABLE
    s_hw->capabilities.macMode = uwb::protocol::setup::buildMacMode(true, 0);
#else
    s_hw->capabilities.macMode = uwb::protocol::setup::buildMacMode(false, 0);
#endif
    s_hw->capabilities.responderCount = 1;

    s_hw->transceiver = std::make_unique<uwb::transceiver::DW3000Controller>(
        s_hw->spi, s_hw->irqPin, s_hw->resetPin, s_hw->clock
    );

    s_hw->session = std::make_unique<uwb::session::RangingSession>(
        *s_hw->transceiver, s_hw->kdf, s_hw->sp0, s_hw->filter,
        s_hw->clock, &s_hw->logger
    );

    auto initRes = s_hw->transceiver->initialize();
    if (!initRes) {
        ESP_LOGE(TAG, "DW3000 transceiver initialization failed: %s",
                 uwb::core::statusToString(initRes.error()).data());
        return false;
    }

    dwt_writesysstatuslo(0xFFFFFFFF);

    if (s_dw3000_irq_sem == nullptr) {
        s_dw3000_irq_sem = xSemaphoreCreateBinary();
    }

    if (s_dw3000_irq_task_handle == nullptr) {
        xTaskCreatePinnedToCore(
            dw3000_irq_worker,
            "dw3000_irq",
            4096,
            nullptr,
            // Highest priority: the arm sequences inside dwt_isr handling must never be
            // preempted by app tasks (WiFi/BLE stay on core 0 anyway; this guards the
            // time-slice against any future same-core work).
            configMAX_PRIORITIES - 1,
            &s_dw3000_irq_task_handle,
            DW3000_TASK_CORE
        );
    }

    gpio_config_t irq_cfg{};
    irq_cfg.pin_bit_mask = (1ULL << PIN_DW3000_IRQ);
    irq_cfg.mode = GPIO_MODE_INPUT;
    irq_cfg.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&irq_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_DW3000_IRQ, dw3000_gpio_isr, nullptr);

    s_hw->initialized = true;
    ESP_LOGI(TAG, "UWB initialized successfully (DW3000 on Core %d)", static_cast<int>(DW3000_TASK_CORE));
    return true;
}

UwbDw3000Channel::UwbDw3000Channel() = default;

UwbDw3000Channel::~UwbDw3000Channel()
{
    stop();
    if (s_active_channel == this) {
        s_active_channel = nullptr;
    }
}

void UwbDw3000Channel::set_sender(std::function<bool(ddk::span<const uint8_t> frame)> send)
{
    send_ = std::move(send);
}

void UwbDw3000Channel::arm(uint32_t session_id, ddk::span<const uint8_t> ursk,
                           const std::array<uint8_t, 2>& selected_version)
{
    if (ursk.size() != 32) {
        ESP_LOGE(TAG, "URSK must be 32 bytes, got %zu", ursk.size());
        return;
    }

    if (!init()) {
        ESP_LOGE(TAG, "Cannot arm: hardware initialization failed");
        return;
    }

    stop();

    s_active_channel = this;
    session_id_ = session_id;
    std::memcpy(ursk_.data(), ursk.data(), 32);
    selected_version_ = selected_version;
    armed_ = true;
    is_ranging_active_ = false;

    {
        std::lock_guard<std::mutex> lock(range_mutex_);
        latest_generation_ = 0;
        last_reported_generation_ = 0;
        latest_distance_cm_ = 0;
    }

    ESP_LOGI(TAG, "UWB channel armed for session %08" PRIX32, session_id);
}

bool UwbDw3000Channel::transmit_frame(std::span<const std::byte> frame)
{
    if (!send_) {
        ESP_LOGE(TAG, "Out-of-band sender callback not set");
        return false;
    }
    return send_(ddk::span<const uint8_t>(reinterpret_cast<const uint8_t*>(frame.data()), frame.size()));
}

void UwbDw3000Channel::handle_frame(ddk::span<const uint8_t> frame)
{
    if (!armed_ || frame.size() < 4) {
        return;
    }

    std::span<const std::byte> frameSpan(reinterpret_cast<const std::byte*>(frame.data()), frame.size());
    const auto category = static_cast<uwb::protocol::setup::ProtocolCategory>(frameSpan[0]);
    const uint8_t msgId = static_cast<uint8_t>(frameSpan[1]);

    switch (category) {
        case uwb::protocol::setup::ProtocolCategory::UwbRangingService:
            handle_ranging_service_frame(msgId, frameSpan);
            break;
        case uwb::protocol::setup::ProtocolCategory::Notification:
            handle_notification_frame(msgId, frameSpan);
            break;
        default:
            ESP_LOGW(TAG, "Ignored unsupported setup frame category 0x%02x", static_cast<uint8_t>(category));
            break;
    }
}

void UwbDw3000Channel::handle_ranging_service_frame(uint8_t msg_id, std::span<const std::byte> frame)
{
    const auto msgType = static_cast<uwb::protocol::setup::MessageType>(msg_id);

    if (msgType == uwb::protocol::setup::MessageType::RangingSetupM2) {
        ESP_LOGI(TAG, "Received Ranging Setup M2 from peer");
        s_hw->parameters.sessionId = uwb::core::SessionId{session_id_};
        s_hw->parameters.slotsPerRound = 12;
        // The phone-selected version feeds the SaltedHash RangingConfiguration
        // it must be set before the session keys are derived.
        s_hw->parameters.protocolVersion = selected_version_;

        auto parseRes = uwb::protocol::setup::SetupMessageCodec::parseM2(
            frame, s_hw->parameters.sessionId, s_hw->capabilities, s_hw->parameters
        );
        if (!parseRes) {
            ESP_LOGE(TAG, "Failed to parse Setup M2: %s", uwb::core::statusToString(parseRes.error()).data());
            // tell the phone we cannot set up right now instead
            // of going silent (it would otherwise wait for M3 until its own timeout).
            if (auto later = uwb::protocol::setup::SetupMessageCodec::buildRangingNotification(0x02)) {
                transmit_frame(*later);
            }
            return;
        }

        auto m3Bytes = uwb::protocol::setup::SetupMessageCodec::buildM3(s_hw->parameters, s_hw->capabilities);
        if (!m3Bytes) {
            ESP_LOGE(TAG, "Failed to build Setup M3");
            return;
        }

        if (s_hw->parameters.hoppingConfigBitmask == 0x40 ||  // Continuous AES
            s_hw->parameters.hoppingConfigBitmask == 0x20) {  // Adaptive AES
            ESP_LOGW(TAG, "Negotiated AES-based hopping mode 0x%02x — not supported "
                          "(Aliro forbids AES hopping; accepting is unexpected)",
                     s_hw->parameters.hoppingConfigBitmask);
        }

        // Finalize the MAC Mode round offset now that the negotiated parameters
        // determine N_Round: Ok defaults to N_Round/2, clamped to 1..N_Round-1
        // Kconfig UWB_ROUND_OFFSET overrides when non-zero.
        if ((s_hw->capabilities.macMode & 0xC0) == 0x40) {
            const uint32_t nran = s_hw->parameters.durationMs / 96;
            const uint64_t roundDurUs = static_cast<uint64_t>(s_hw->parameters.slotsPerRound) *
                                        s_hw->parameters.slotDurationRstu * 5ULL / 6ULL;
            uint32_t nRound = roundDurUs ? static_cast<uint32_t>(288ULL * nran / roundDurUs) : 0;
            if (nRound < 2) {
                ESP_LOGW(TAG, "N_Round=%" PRIu32 " too small for two rounds — falling back to single", nRound);
                s_hw->capabilities.macMode = uwb::protocol::setup::buildMacMode(false, 0);
            } else {
                uint8_t ok = 0;
#ifdef CONFIG_UWB_ROUND_OFFSET
                ok = static_cast<uint8_t>(CONFIG_UWB_ROUND_OFFSET);
#endif
                if (ok == 0 || ok >= nRound) {
                    ok = static_cast<uint8_t>(nRound / 2);
                }
                s_hw->capabilities.macMode = uwb::protocol::setup::buildMacMode(true, ok);
                ESP_LOGI(TAG, "Two-round MAC Mode: N_Round=%" PRIu32 ", Ok=%u", nRound, ok);
            }
        }

        ESP_LOGI(TAG, "Sending Ranging Setup M3 to peer");
        transmit_frame(*m3Bytes);

    } else if (msgType == uwb::protocol::setup::MessageType::RangingSetupM4) {
        ESP_LOGI(TAG, "Received Ranging Setup M4. Starting CCC UWB session...");

        auto parseRes = uwb::protocol::setup::SetupMessageCodec::parseM4(frame, s_hw->parameters);
        if (!parseRes) {
            ESP_LOGE(TAG, "Failed to parse Setup M4: %s", uwb::core::statusToString(parseRes.error()).data());
            return;
        }

        uwb::session::SessionNodeConfig nodeConfig{
            .responderIndex = 0,
            .blockParityFilter = -1,
            .antennaDelayBias = uwb::core::DistanceMm{0},
#ifdef CONFIG_UWB_AOA_ENABLE
            .enableAoA = true,
            // Kconfig has no float type here: spacing is configured in 0.1 mm units
            .aoaAntennaSpacingMm = static_cast<float>(CONFIG_UWB_AOA_ANT_SPACING_TENTHS_MM) / 10.0f
#endif
        };

        auto startRes = s_hw->session->start(
            ursk_,
            s_hw->parameters,
            nodeConfig,
            [this](const uwb::session::RangingResult& result) {
                this->on_ranging_measurement(result);
            }
        );

        if (!startRes) {
            ESP_LOGE(TAG, "Failed to start UWB session: %s", uwb::core::statusToString(startRes.error()).data());
            if (auto failed = uwb::protocol::setup::SetupMessageCodec::buildRangingNotification(0x04)) {
                transmit_frame(*failed);
            }
            return;
        }

        is_ranging_active_ = true;
        ESP_LOGI(TAG, "UWB ranging session is now ACTIVE (channel %d, sync %d)",
                 static_cast<int>(s_hw->parameters.channel), s_hw->parameters.syncCodeIndex);

    } else if (msgType == uwb::protocol::setup::MessageType::SuspendRequest) {
        ESP_LOGI(TAG, "Received Suspend Request");
        s_hw->session->suspend();
        is_ranging_active_ = false;

        auto resp = uwb::protocol::setup::SetupMessageCodec::buildSuspendResponse(true);
        if (resp) {
            transmit_frame(*resp);
        }
    } else if (msgType == uwb::protocol::setup::MessageType::ResumeRequest) {
        ESP_LOGW(TAG, "Received Resume Request from the phone — ignored");
    } else if (msgType == uwb::protocol::setup::MessageType::ResumeResponse) {
        uwb::core::StsIndex newSts0{0};
        uint64_t newTime0 = 0;
        auto parseRes = uwb::protocol::setup::SetupMessageCodec::parseResumeResponse(
            frame.subspan(4), newSts0, newTime0);
        if (!parseRes) {
            ESP_LOGE(TAG, "Failed to parse Resume Response: %s",
                     uwb::core::statusToString(parseRes.error()).data());
            return;
        }
        ESP_LOGI(TAG, "Resume Response: sts0=0x%08" PRIX32 " — resuming session", newSts0.get());
        auto resumeRes = s_hw->session->resumeWithAnchor(newSts0, newTime0);
        if (!resumeRes) {
            ESP_LOGE(TAG, "Resume with anchor failed: %s",
                     uwb::core::statusToString(resumeRes.error()).data());
            return;
        }
        is_ranging_active_ = true;
    }
}

void UwbDw3000Channel::handle_notification_frame(uint8_t msg_id, std::span<const std::byte> frame)
{
    if (msg_id != 0x01) {
        return;
    }

    size_t offset = 4; // Skip the 4-byte header (Category, MsgID, LenHi, LenLo)
    while (offset + 2 <= frame.size()) {
        uint8_t tag = static_cast<uint8_t>(frame[offset]);
        uint8_t len = static_cast<uint8_t>(frame[offset + 1]);

        if (offset + 2 + len > frame.size()) break;

        if (tag == 0x00) {
            ESP_LOGI(TAG, "Received Initiate-Ranging notification. Sending Setup M1...");
            auto m1Bytes = uwb::protocol::setup::SetupMessageCodec::buildM1(
                uwb::core::SessionId{session_id_}, s_hw->capabilities
            );
            if (m1Bytes) {
                transmit_frame(*m1Bytes);
            }
            return;
        }

        // Attribute 0x01 = Initiate Ranging Session Resume:
        // the phone triggers us to send the Resume Request (only meaningful for
        // a previously suspended session of this transaction).
        if (tag == 0x01) {
            if (s_hw->session->getState() != uwb::session::SessionState::Suspended) {
                ESP_LOGW(TAG, "Resume requested but no suspended session — ignored");
                return;
            }
            ESP_LOGI(TAG, "Received Initiate-Resume notification. Sending Resume Request...");
            if (auto req = uwb::protocol::setup::SetupMessageCodec::buildResumeRequest(
                    uwb::core::SessionId{session_id_})) {
                transmit_frame(*req);
            }
            return;
        }
        offset += 2 + len;
    }
}

void UwbDw3000Channel::on_ranging_measurement(const uwb::session::RangingResult& result)
{
    std::lock_guard<std::mutex> lock(range_mutex_);
    latest_distance_cm_ = result.distance.get() / 10;
    latest_aoa_centi_degrees_ = result.aoaCentiDegrees;
    latest_aoa_valid_ = result.aoaValid;
    latest_nlos_ = result.integrity.nlosValid && result.integrity.nlosDetected;
    latest_range_timestamp_ms_ = s_hw->clock.getMonotonicTimeMs();
    latest_generation_++;
}

void UwbDw3000Channel::poll()
{
    if (!is_ranging_active_) {
        return;
    }

    int32_t distance_cm = 0;
    int64_t age_ms = 0;
    int32_t aoa_centi_degrees = 0;
    bool aoa_valid = false;
    bool nlos = false;
    uint32_t generation = 0;

    {
        std::lock_guard<std::mutex> lock(range_mutex_);
        generation = latest_generation_;
        if (generation == 0 || generation == last_reported_generation_) {
            return;
        }

        distance_cm = latest_distance_cm_;
        aoa_centi_degrees = latest_aoa_centi_degrees_;
        aoa_valid = latest_aoa_valid_;
        nlos = latest_nlos_;
        age_ms = static_cast<int64_t>(s_hw->clock.getMonotonicTimeMs() - latest_range_timestamp_ms_);
        last_reported_generation_ = generation;
    }

    if (aoa_valid) {
        ESP_LOGI(TAG, "Trusted range: %ld cm (age %lld ms, aoa %ld.%02ld deg, nlos %d, session %08" PRIX32 ")",
                 static_cast<long>(distance_cm), static_cast<long long>(age_ms),
                 static_cast<long>(aoa_centi_degrees / 100),
                 static_cast<long>((aoa_centi_degrees < 0 ? -aoa_centi_degrees : aoa_centi_degrees) % 100),
                 nlos ? 1 : 0,
                 session_id_);
    } else {
        ESP_LOGI(TAG, "Trusted range: %ld cm (age %lld ms, nlos %d, session %08" PRIX32 ")",
                 static_cast<long>(distance_cm), static_cast<long long>(age_ms),
                 nlos ? 1 : 0, session_id_);
    }

    if (range_cb_) {
        range_cb_(distance_cm, age_ms, aoa_centi_degrees, aoa_valid, nlos);
    }
}

void UwbDw3000Channel::suspend_and_notify()
{
    if (!armed_) {
        return;
    }
    if (s_hw && s_hw->session &&
        s_hw->session->getState() != uwb::session::SessionState::Suspended) {
        s_hw->session->suspend();
    }
    is_ranging_active_ = false;
    ESP_LOGI(TAG, "Suspending UWB session unilaterally (Ranging Session Suspended)");
    if (auto notify = uwb::protocol::setup::SetupMessageCodec::buildRangingNotification(0x05)) {
        transmit_frame(*notify);
    }
}

void UwbDw3000Channel::stop()
{
    if (s_hw && s_hw->session) {
        s_hw->session->stop();
    }
    is_ranging_active_ = false;
    armed_ = false;
    ESP_LOGI(TAG, "UWB ranging channel stopped");
}

bool UwbDw3000Channel::ranging_active() const
{
    return is_ranging_active_.load();
}

}  // namespace aliro_uwb
