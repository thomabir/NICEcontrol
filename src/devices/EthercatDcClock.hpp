#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "ess.h"

// Reads the EtherCAT distributed clock (DC) of the esd ECS-PCIe/FPGA card.
//
//     ecat::DcClock clock;                     // throws std::runtime_error if the card does not start
//     if (auto s = clock.get_DC_sample())      // the DC time and the PC time of the same instant
//         use(s->dc_ns, s->pc);
//
// The card is a subdevice on the bus. An EtherCAT maindevice, for example TwinCAT, distributes the clock to it.
// The constructor opens the card and starts a background thread that samples the clock one time each millisecond.
// The destructor releases the card.
//
// The sample comes from the system time register (0x0910) of the EtherCAT chip, in two 32-bit accesses. The card also
// sends each sample on the bus as one 64-bit input (object 0x2010), thus the maindevice can show the same value.
//
// get_DC_sample() is thread-safe and gives the most recent sample. It gives an empty value if the clock is 0, if it
// does not advance, or if the last sample is older than kMaxSampleAge. Thus an invalid or stale clock never reaches
// the caller.
//
// The esd stack opens the card one time in each process. Thus a second object, also after the destruction of the
// first one, throws std::runtime_error.
//
// The build needs -DESS_ESD_LIBRARY, the headers in lib/esd/include and the library lib/esd/lib/libess.a. The card
// also needs the pexesc_uio driver, the ESI file on the maindevice, and the DC operation mode on the box.

namespace ecat {

namespace detail {

// The identity of the card. It must agree with the EEPROM of the card and with the ESI file of the maindevice.
inline constexpr uint32_t kVendorId = 0xe0000017;
inline constexpr uint32_t kProductCode = 0x0000000e;
inline constexpr uint32_t kRevisionNo = 0x01000001;
inline constexpr uint32_t kSerialNo = 0;
inline constexpr uint32_t kDeviceType = 5001;

inline constexpr const char *kDeviceName = "ECS-PCIe/FPGA";
inline constexpr const char *kHardwareVersion = "1";
inline constexpr const char *kSoftwareVersion = "EtherCAT DC clock";

// The process image: one 64-bit input at 0x2010, in the TxPDO 0x1a00 on the input SyncManager.
inline constexpr uint16_t kDcTimeIndex = 0x2010;
inline constexpr uint16_t kTxPdoIndex = 0x1a00;
inline constexpr const char *kDcTimeName = "DcTime";
inline constexpr ESS_SM kInputSm = ESS_SM_3;
inline constexpr uint16_t kPdoAssignment[] = {kTxPdoIndex};

// The period of the cyclic callback of the stack, in microseconds.
inline constexpr uint32_t kTimerIntervalUs = 1000;

// The period between two calls of essStop() while the destructor waits for the thread of the stack.
inline constexpr std::chrono::milliseconds kStopRetryPeriod{50};

// The maximum age of a sample. An older sample is not given to the caller.
// The cyclic callback makes a new sample each millisecond, thus this limit accepts a delay of 100 cycles.
inline constexpr std::chrono::milliseconds kMaxSampleAge{100};

// The SyncManagers: mailbox out, mailbox in, outputs (not in use), inputs.
// The values must agree with the ESI file of the maindevice.
inline constexpr ESS_SM_CONFIGURATION kSmConfigurations[] = {
    {46, 522, 1024, 0x1000, SM_TYPE_MBXOUT | REG_MASK_SMCONTROL_PDIINT},
    {46, 522, 1024, 0x1400, SM_TYPE_MBXIN | REG_MASK_SMCONTROL_PDIINT},
    {0, 4, 0, 0x1800, SM_TYPE_OUTPUTS | REG_MASK_SMCONTROL_PDIINT},
    {0, 8, 0, 0x2400, SM_TYPE_INPUTS | REG_MASK_SMCONTROL_PDIINT},
};

inline constexpr uint8_t kSmCount = 4;

}  // namespace detail

// One instant, in the two clocks. dc_ns counts nanoseconds from 2000-01-01 00:00, the epoch of the distributed clock.
//
// The read of the low word latches the value in the chip. pc is the monotonic PC clock immediately before that read
// and read_span is its length, thus the latch is inside [pc, pc + read_span] and read_span is the uncertainty of the
// pair. A constant part of read_span is the transfer over PCIe, thus the pair has a constant bias of a few
// microseconds that only an external reference can measure.
struct DcSample {
  uint64_t dc_ns = 0;
  std::chrono::steady_clock::time_point pc;
  std::chrono::nanoseconds read_span{0};
};

class DcClock {
 public:
  // Open the card and start to operate EtherCAT in the background.
  // Throws std::runtime_error if the driver is not available, if the stack refuses the configuration, or if the
  // process has an open card.
  explicit DcClock(unsigned device_index = 0) {
    DcClock *unclaimed = nullptr;
    if (!s_instance.compare_exchange_strong(unclaimed, this)) {
      throw std::runtime_error(
          "ecat::DcClock: this process has an open card already.\n"
          "The esd stack permits one open card in each process, also after the destruction of an earlier DcClock.\n"
          "Repair: keep one DcClock object, or start the program again.");
    }

    setup_configuration();
    const ESS_RESULT open_result = essOpen(static_cast<ESS_DEVICE_INDEX>(device_index), &configuration_, &hdev_);
    if (open_result != ESS_RESULT_SUCCESS) {
      s_instance.store(nullptr);
      throw std::runtime_error("ecat::DcClock: essOpen() for card " + std::to_string(device_index) + " gave " +
                               essFormatResult(open_result) +
                               ".\n"
                               "The card is not available. Examine these four points:\n"
                               "  1. the driver is loaded:            lsmod | grep pexesc_uio\n"
                               "  2. the device file is present:      ls -l /dev/uio0\n"
                               "  3. your user is in the group:       id -nG    (the list must contain netdev)\n"
                               "  4. no other process has the card:   sudo lsof /dev/uio0");
    }

    // essOpen() gave the card, thus the claim stays from this point, also if the setup fails.
    // A second essOpen() in this process cannot succeed.
    const std::string setup_error = build_object_dictionary();
    if (!setup_error.empty()) {
      essClose(hdev_);
      throw std::runtime_error("ecat::DcClock: the subdevice stack refused the configuration: " + setup_error +
                               ".\n"
                               "The object dictionary or the PDO does not agree with the stack build.\n"
                               "The card is closed again, thus a new DcClock is not possible in this process.");
    }

    mask_input_interrupt();
    active_.store(true);
    worker_ = std::thread([this] {
      record_background_error(failure("essStart()", essStart(hdev_)));
      {
        std::lock_guard<std::mutex> lock(stop_mutex_);
        stack_stopped_ = true;
      }
      stopped_condition_.notify_all();
    });
  }

  ~DcClock() {
    if (!active_.exchange(false)) {
      return;
    }
    // essStop() has no effect while essStart() has not reached its main loop. Thus the call repeats until the thread
    // of the stack ends; one call only can block the destructor for an unlimited time.
    for (;;) {
      essStop(hdev_);
      std::unique_lock<std::mutex> lock(stop_mutex_);
      if (stopped_condition_.wait_for(lock, detail::kStopRetryPeriod, [this] { return stack_stopped_; })) {
        break;
      }
    }
    worker_.join();
    essClose(hdev_);
  }

  DcClock(const DcClock &) = delete;
  DcClock &operator=(const DcClock &) = delete;

  // The most recent pair of the two clocks. The value is empty while the clock is not present: the sample is 0, it
  // does not advance, or it is older than detail::kMaxSampleAge.
  [[nodiscard]] std::optional<DcSample> get_DC_sample() const {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (sample_.dc_ns == 0 || std::chrono::steady_clock::now() - sample_.pc > detail::kMaxSampleAge) {
      return std::nullopt;
    }
    return sample_;
  }

  // The distributed clock alone, in nanoseconds from 2000-01-01 00:00.
  [[nodiscard]] std::optional<uint64_t> get_DC_clock() const {
    if (const std::optional<DcSample> sample = get_DC_sample()) {
      return sample->dc_ns;
    }
    return std::nullopt;
  }

  // The AL state of the subdevice: 1 INIT, 2 PREOP, 4 SAFEOP, 8 OP.
  [[nodiscard]] int state() const { return current_state_.load(); }

  // The first error of the background thread, or an empty value if no call has failed.
  // Such an error does not stop the clock; it stops the copy of the sample to the bus.
  [[nodiscard]] std::optional<std::string> background_error() const {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (background_error_.empty()) {
      return std::nullopt;
    }
    return background_error_;
  }

 private:
  // The stack calls C functions without an argument for the context, thus the callbacks find the object through this
  // pointer. The constructor claims the pointer and keeps the claim, also after the destruction, because a second
  // essOpen() in the same process cannot succeed.
  static inline std::atomic<DcClock *> s_instance{nullptr};

  static void cb_cyclic(ESS_CBDATA_CYCLIC *) {
    if (DcClock *self = s_instance.load()) {
      self->on_cyclic();
    }
  }

  static void cb_state_request(ESS_CBDATA_STATE_REQUEST *data) {
    if (DcClock *self = s_instance.load()) {
      self->on_state_request(data->newState);
    }
  }

  // These callbacks are mandatory, because the libess build has CoE support and DC support.
  static void cb_inoutputs_activate(ESS_CBDATA_INOUTPUTS_ACTIVATE *) {}
  static void cb_outputs_updated(ESS_CBDATA_SM_EVENT *) {}
  static void cb_coe_read_write(ESS_CBDATA_COE_READWRITE *) {}
  static void cb_coe_event(ESS_CBDATA_COE_EVENT *) {}
  static void cb_dc_event(ESS_CBDATA_DC *) {}
  static void cb_voe(ESS_CBDATA_VOE *) {}
  static void cb_soe(ESS_CBDATA_SOE *) {}

  // Sample the clock. Only this thread writes pdo_dc_time_ and previous_sample_.
  //
  // The value comes from the system time register 0x0910 of the ESC, in two 32-bit accesses. essGetDcTime() and a
  // 64-bit access both give all ones on this card, thus the code reads the register directly.
  //
  // The two PC times bracket the read of the low word, which is the access that latches the value. The read of the
  // high word stays outside the bracket, because it gives the latched value and not a new one.
  void on_cyclic() {
    if (!active_.load()) {
      return;
    }
    const std::chrono::steady_clock::time_point before = std::chrono::steady_clock::now();
    const uint32_t low = essESCRead32(hdev_, ESC_REG_DCSYSTEMTIME);
    const std::chrono::steady_clock::time_point after = std::chrono::steady_clock::now();
    const uint32_t high = essESCRead32(hdev_, ESC_REG_DCSYSTEMTIMEHI);
    const uint64_t sample = (static_cast<uint64_t>(high) << 32) | low;

    const bool valid = sample != 0 && sample != previous_sample_;
    previous_sample_ = sample;
    pdo_dc_time_ = sample;

    if (valid) {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      sample_.dc_ns = sample;
      sample_.pc = before;
      sample_.read_span = after - before;
    }

    const int state = current_state_.load();
    if (state == ESC_STATE_SAFEOP || state == ESC_STATE_OP) {
      essSyncInputs(hdev_, static_cast<ESS_ISYN_FLAGS>(1u << detail::kInputSm));
    }
  }

  void on_state_request(int new_state) {
    current_state_.store(new_state);
    // The stack removes the dynamic PDO configuration at each Init, thus the configuration goes to the card again.
    // The constructor sets active_ when the configuration is available; before that point there is nothing to apply.
    if (new_state == ESC_STATE_INIT && active_.load()) {
      const std::string error = apply_pdo();
      if (!error.empty()) {
        record_background_error("the PDO configuration after the INIT transition failed: " + error);
      }
    }
  }

  // Keep the first error, because it explains the failure.
  void record_background_error(const std::string &error) {
    if (error.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (background_error_.empty()) {
      background_error_ = error;
    }
  }

  // essOpen() keeps the configuration by reference, thus the members below must live as long as the open handle.
  void setup_configuration() {
    callbacks_.cbCyclic = &cb_cyclic;
    callbacks_.cbStateRequest = &cb_state_request;
    callbacks_.cbInOutputsActivate = &cb_inoutputs_activate;
    callbacks_.cbOutputsUpdated = &cb_outputs_updated;
    callbacks_.cbCoEReadWrite = &cb_coe_read_write;
    callbacks_.cbCoEEvent = &cb_coe_event;
    callbacks_.cbDCEvent = &cb_dc_event;
    callbacks_.cbVoE = &cb_voe;
    callbacks_.cbSoE = &cb_soe;

    configuration_.essABIVersion = ESS_ABI_VERSION;
    configuration_.flags = ESS_CONFIG_FLAGS_USE_ISR;
    configuration_.timerInterval = detail::kTimerIntervalUs;
    configuration_.cb = &callbacks_;
    configuration_.smConfigs = detail::kSmConfigurations;
    configuration_.smConfigCount = detail::kSmCount;
    configuration_.stats = &statistics_;
  }

  // The name of the call that failed and its result, or an empty text if the call succeeded.
  static std::string failure(const char *call, ESS_RESULT result) {
    if (result == ESS_RESULT_SUCCESS) {
      return std::string();
    }
    return std::string(call) + " gave " + essFormatResult(result);
  }

  // The stack keeps object_info_, entry_info_ and the storage by reference, thus they are members.
  // Gives an empty text on success, or the name of the call that failed.
  std::string build_object_dictionary() {
    std::string error = failure("essODCreate()", essODCreate(hdev_, ESS_OD_FLAGS_HANDLE_SM_TYPES));
    if (!error.empty()) {
      return error;
    }

    error = failure("essODAddGenericObjects()",
                    essODAddGenericObjects(hdev_, &detail::kDeviceType, &detail::kVendorId, &detail::kProductCode,
                                           &detail::kRevisionNo, &detail::kSerialNo, detail::kDeviceName,
                                           detail::kHardwareVersion, detail::kSoftwareVersion));
    if (!error.empty()) {
      return error;
    }

    object_info_ = ESS_OD_OBJECT_INFOS{detail::kDcTimeName, COE_DATATYPE_ULINT, COE_CODE_VARIABLE};
    entry_info_ = ESS_OD_ENTRY_INFOS{detail::kDcTimeName, nullptr, nullptr, nullptr, 0, COE_DATATYPE_ULINT};

    error = failure("essODObjectAdd(0x2010)",
                    essODObjectAdd(hdev_, detail::kDcTimeIndex, ESS_OD_OBJECT_FLAGS_NONE, &object_info_));
    if (!error.empty()) {
      return error;
    }

    // The input is read-only: an SDO write from the maindevice would change the sample.
    error = failure("essODEntryAdd(0x2010)",
                    essODEntryAdd(hdev_, detail::kDcTimeIndex, 0x00, 64, static_cast<volatile void *>(&pdo_dc_time_),
                                  COE_ACCESS_R | COE_ACCESS_TXMAPPABLE, ESS_OD_ENTRY_FLAGS_NONE, &entry_info_));
    if (!error.empty()) {
      return error;
    }

    pdo_entry_ = ESS_MAP_ENTRY(detail::kDcTimeIndex, 0, 64);
    return apply_pdo();
  }

  std::string apply_pdo() {
    const std::string error = failure(
        "essODUpdatePDOConfiguration(0x1a00)",
        essODUpdatePDOConfiguration(hdev_, detail::kTxPdoIndex, detail::kDcTimeName, &pdo_entry_, 1, ESS_FALSE));
    if (!error.empty()) {
      return error;
    }
    return failure("essODUpdatePDOAssignment()",
                   essODUpdatePDOAssignment(hdev_, detail::kInputSm, detail::kPdoAssignment, 1, ESS_TRUE));
  }

  // The interrupt of the input SM fires again in each cycle if no function services it.
  // This application has no handler for updated inputs, thus the interrupt is masked to keep the stack responsive.
  void mask_input_interrupt() {
    uint32_t mask = essESCRead32(hdev_, ESC_REG_ALEVENTMASK);
    mask &= ~REG_MASK_ALEVENT_PDWDEXPIRED;
    mask &= ~REG_MASK_ALEVENT_SM(detail::kInputSm);
    essESCWrite32(hdev_, ESC_REG_ALEVENTMASK, mask);
  }

  ESS_HANDLE hdev_{};
  ESS_CONFIGURATION configuration_{};
  ESS_CALLBACKS callbacks_{};
  ESS_STATISTICS statistics_{};
  ESS_OD_OBJECT_INFOS object_info_{};
  ESS_OD_ENTRY_INFOS entry_info_{};
  ESS_PDO_ENTRY pdo_entry_{};

  uint64_t pdo_dc_time_{};      // the storage that the stack maps into the input PDO
  uint64_t previous_sample_{};  // the cyclic callback only

  // The last valid sample and the first error of the background thread. sample_mutex_ guards both.
  mutable std::mutex sample_mutex_;
  DcSample sample_;
  std::string background_error_;

  std::thread worker_;
  std::atomic<bool> active_{false};
  std::atomic<int> current_state_{ESC_STATE_INIT};

  // The end of essStart() in the thread of the stack. stop_mutex_ guards stack_stopped_.
  std::mutex stop_mutex_;
  std::condition_variable stopped_condition_;
  bool stack_stopped_{false};
};

}  // namespace ecat
