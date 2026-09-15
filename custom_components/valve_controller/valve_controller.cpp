#include "valve_controller.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::valve_controller {

using namespace esphome::valve;

static const char *const TAG = "valve_controller";
static const char *const VALVE_CONTROLLER_VERSION = "0.1.0";

static constexpr uint8_t INA219_REG_CONFIG = 0x00;
static constexpr uint8_t INA219_REG_CURRENT = 0x04;
static constexpr uint8_t INA219_REG_CALIBRATION = 0x05;
static constexpr uint16_t INA219_CONFIG_DEFAULT = 0x399F;

void ValveController::dump_config() {
  ESP_LOGCONFIG(TAG, "Valve Controller:");
  LOG_VALVE("", "Valve Controller", this);
  ESP_LOGCONFIG(TAG, "  Open output: %s", this->open_output_ != nullptr ? "configured" : "missing");
  ESP_LOGCONFIG(TAG, "  Close output: %s", this->close_output_ != nullptr ? "configured" : "missing");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Current threshold: %.3f A", this->current_threshold_amps_);
  ESP_LOGCONFIG(TAG, "  INA219 shunt resistance: %.4f ohm", this->shunt_resistance_ohms_);
  ESP_LOGCONFIG(TAG, "  INA219 max expected current: %.3f A", this->max_expected_current_amps_);
  ESP_LOGCONFIG(TAG, "  Movement timeout: %lu ms", this->movement_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Running current check interval: %lu ms", this->running_current_check_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Idle current check interval: %lu ms", this->idle_current_check_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Minimum running time: %lu ms", this->minimum_running_time_ms_);
}

void ValveController::setup() {
  ESP_LOGI(TAG, "Starting Valve Controller v%s", VALVE_CONTROLLER_VERSION);
  this->publish_version_text_();
  this->publish_status_text_("initializing");
  if (!this->setup_ina219_()) {
    this->set_error_("INA219 initialization failed");
    return;
  }

  this->all_outputs_off_();
  this->startup_stage_ = StartupStage::OPEN_TEST;
  this->stage_started_at_ = millis();
  this->startup_open_current_ = false;
  this->startup_close_current_ = false;
  this->has_cached_current_sample_ = false;
  this->last_current_sample_at_ = 0;
  this->state_ = ValveState::ERROR;
  this->set_state_(ValveState::UNKNOWN);
  this->set_open_output_(true);
}

void ValveController::publish_version_text_() {
  if (this->version_text_sensor_ != nullptr) {
    this->version_text_sensor_->publish_state(VALVE_CONTROLLER_VERSION);
  }
}

void ValveController::publish_status_text_(const char *status_text) {
  if (this->status_text_sensor_ != nullptr) {
    this->status_text_sensor_->publish_state(status_text);
  }
}

bool ValveController::setup_ina219_() {
  if (this->shunt_resistance_ohms_ <= 0.0f || this->max_expected_current_amps_ <= 0.0f) {
    ESP_LOGE(TAG, "Invalid INA219 calibration inputs: shunt=%.6f, max_current=%.6f", this->shunt_resistance_ohms_,
             this->max_expected_current_amps_);
    return false;
  }

  const float current_lsb = this->max_expected_current_amps_ / 32768.0f;
  float calibration_f = 0.04096f / (current_lsb * this->shunt_resistance_ohms_);
  uint16_t calibration = static_cast<uint16_t>(calibration_f);
  if (calibration == 0) {
    calibration = 1;
  }

  if (!this->write_byte_16(INA219_REG_CONFIG, INA219_CONFIG_DEFAULT)) {
    ESP_LOGE(TAG, "Failed to write INA219 config register");
    return false;
  }

  if (!this->write_byte_16(INA219_REG_CALIBRATION, calibration)) {
    ESP_LOGE(TAG, "Failed to write INA219 calibration register");
    return false;
  }

  this->ina219_current_lsb_amps_ = 0.04096f / (static_cast<float>(calibration) * this->shunt_resistance_ohms_);
  this->ina219_configured_ = true;
  ESP_LOGI(TAG, "INA219 ready at 0x%02X, current_lsb=%.9f A/bit", this->get_i2c_address(),
           this->ina219_current_lsb_amps_);
  return true;
}

bool ValveController::read_current_amps_(float *current_amps) {
  if (!this->ina219_configured_) {
    return false;
  }

  uint16_t raw = 0;
  if (!this->read_byte_16(INA219_REG_CURRENT, &raw)) {
    ESP_LOGW(TAG, "INA219 current register read failed");
    return false;
  }

  const int16_t signed_raw = static_cast<int16_t>(raw);
  *current_amps = static_cast<float>(signed_raw) * this->ina219_current_lsb_amps_;
  return true;
}

void ValveController::loop() {
  if (this->startup_stage_ != StartupStage::DONE) {
    this->process_startup_();
    return;
  }

  this->process_motion_();

  if (this->state_ != ValveState::OPENING && this->state_ != ValveState::CLOSING) {
    const uint32_t now = millis();
    (void) this->current_above_threshold_throttled_(now, this->idle_current_check_interval_ms_);
  }
}

void ValveController::request_open() {
  ESP_LOGI(TAG, "Received request: OPEN");
  if (this->state_ == ValveState::OPEN) {
    ESP_LOGI(TAG, "Ignoring OPEN request: valve already open");
    return;
  }

  this->startup_stage_ = StartupStage::DONE;
  this->start_opening_();
}

void ValveController::request_close() {
  ESP_LOGI(TAG, "Received request: CLOSE");
  if (this->state_ == ValveState::CLOSED) {
    ESP_LOGI(TAG, "Ignoring CLOSE request: valve already closed");
    return;
  }

  this->startup_stage_ = StartupStage::DONE;
  this->start_closing_();
}

void ValveController::set_state_(ValveState state) {
  if (this->state_ == state) {
    return;
  }

  this->state_ = state;

  switch (this->state_) {
    case ValveState::OPEN:
      this->position = VALVE_OPEN;
      this->current_operation = VALVE_OPERATION_IDLE;
      break;
    case ValveState::CLOSED:
      this->position = VALVE_CLOSED;
      this->current_operation = VALVE_OPERATION_IDLE;
      break;
    case ValveState::OPENING:
      this->current_operation = VALVE_OPERATION_OPENING;
      break;
    case ValveState::CLOSING:
      this->current_operation = VALVE_OPERATION_CLOSING;
      break;
    case ValveState::UNKNOWN:
    case ValveState::ERROR:
      this->position = 0.5f;
      this->current_operation = VALVE_OPERATION_IDLE;
      break;
  }

  this->publish_state();
  this->publish_status_text_(this->state_name_(this->state_));
}

void ValveController::all_outputs_off_() {
  if (this->open_output_ != nullptr) {
    this->open_output_->turn_off();
  }
  if (this->close_output_ != nullptr) {
    this->close_output_->turn_off();
  }
}

void ValveController::set_open_output_(bool enabled) {
  if (this->close_output_ != nullptr) {
    this->close_output_->turn_off();
  }
  if (this->open_output_ != nullptr) {
    if (enabled) {
      this->open_output_->turn_on();
    } else {
      this->open_output_->turn_off();
    }
  }
}

void ValveController::set_close_output_(bool enabled) {
  if (this->open_output_ != nullptr) {
    this->open_output_->turn_off();
  }
  if (this->close_output_ != nullptr) {
    if (enabled) {
      this->close_output_->turn_on();
    } else {
      this->close_output_->turn_off();
    }
  }
}

bool ValveController::current_above_threshold_() {
  float current_amps = 0.0f;
  if (!this->read_current_amps_(&current_amps)) {
    return false;
  }
  return std::fabs(current_amps) >= this->current_threshold_amps_;
}

bool ValveController::current_above_threshold_throttled_(uint32_t now, uint32_t interval_ms, bool force) {
  if (interval_ms == 0) {
    interval_ms = 1;
  }

  if (force || !this->has_cached_current_sample_ || now - this->last_current_sample_at_ >= interval_ms) {
    this->cached_current_above_threshold_ = this->current_above_threshold_();
    this->last_current_sample_at_ = now;
    this->has_cached_current_sample_ = true;
  }

  return this->cached_current_above_threshold_;
}

void ValveController::start_opening_() {
  this->motion_started_at_ = millis();
  this->motion_current_check_pending_ = true;
  this->has_cached_current_sample_ = false;
  this->set_close_output_(false);
  this->set_open_output_(true);
  this->set_state_(ValveState::OPENING);
}

void ValveController::start_closing_() {
  this->motion_started_at_ = millis();
  this->motion_current_check_pending_ = true;
  this->has_cached_current_sample_ = false;
  this->set_open_output_(false);
  this->set_close_output_(true);
  this->set_state_(ValveState::CLOSING);
}

void ValveController::set_error_(const char *reason) {
  ESP_LOGE(TAG, "Valve state changed to error: %s", reason);
  this->all_outputs_off_();
  this->set_state_(ValveState::ERROR);
  this->publish_status_text_(reason);
  this->status_set_error();
}

const char *ValveController::state_name_(ValveState state) const {
  switch (state) {
    case ValveState::UNKNOWN:
      return "unknown";
    case ValveState::OPEN:
      return "open";
    case ValveState::CLOSED:
      return "closed";
    case ValveState::OPENING:
      return "opening";
    case ValveState::CLOSING:
      return "closing";
    case ValveState::ERROR:
      return "error";
  }

  return "error";
}

void ValveController::complete_startup_() {
  this->startup_stage_ = StartupStage::DONE;

  if (this->startup_open_current_ == this->startup_close_current_) {
    if (this->startup_open_current_) {
      this->set_error_("startup position detection saw current in both tests");
    } else {
      this->set_error_("startup position detection saw no current in either test");
    }
    return;
  }

  if (this->startup_open_current_) {
    this->all_outputs_off_();
    this->set_state_(ValveState::CLOSED);
    return;
  }

  this->all_outputs_off_();
  this->set_state_(ValveState::OPEN);
}

void ValveController::process_startup_() {
  const uint32_t now = millis();

  switch (this->startup_stage_) {
    case StartupStage::OPEN_TEST:
      if (now - this->stage_started_at_ < 50) {
        return;
      }

        this->startup_open_current_ =
          this->current_above_threshold_throttled_(now, this->running_current_check_interval_ms_, true);
      this->set_open_output_(false);
      this->set_close_output_(this->startup_open_current_);
      this->stage_started_at_ = now;
      this->startup_stage_ = this->startup_open_current_ ? StartupStage::CLOSE_NUDGE : StartupStage::CLOSE_TEST;
      if (!this->startup_open_current_) {
        return;
      }
      return;

    case StartupStage::CLOSE_NUDGE:
      if (now - this->stage_started_at_ < 100) {
        return;
      }

      this->stage_started_at_ = now;
      this->startup_stage_ = StartupStage::CLOSE_TEST;
      return;

    case StartupStage::CLOSE_TEST:
      if (now - this->stage_started_at_ < 50) {
        return;
      }

      this->startup_close_current_ =
          this->current_above_threshold_throttled_(now, this->running_current_check_interval_ms_, true);
      this->set_close_output_(false);
      if (this->startup_close_current_) {
        this->set_open_output_(true);
        this->stage_started_at_ = now;
        this->startup_stage_ = StartupStage::OPEN_NUDGE;
        return;
      }

      this->complete_startup_();
      return;

    case StartupStage::OPEN_NUDGE:
      if (now - this->stage_started_at_ < 100) {
        return;
      }

      this->set_open_output_(false);
      this->complete_startup_();
      return;

    case StartupStage::DONE:
      return;
  }
}

void ValveController::process_motion_() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->motion_started_at_;

  if (this->state_ == ValveState::OPENING) {
    if (this->motion_current_check_pending_) {
      if (elapsed < this->running_current_check_interval_ms_) {
        return;
      }

      if (!this->current_above_threshold_throttled_(now, this->running_current_check_interval_ms_, true)) {
        this->set_error_("opening current was not detected in the startup check window");
        return;
      }

      this->motion_current_check_pending_ = false;
    }

    if (!this->current_above_threshold_throttled_(now, this->running_current_check_interval_ms_)) {
      if (elapsed < this->minimum_running_time_ms_) {
        this->set_error_("opening current dropped below threshold too early");
        return;
      }

      this->set_open_output_(false);
      this->set_state_(ValveState::OPEN);
      return;
    }

    if (elapsed >= this->movement_timeout_ms_) {
      this->set_error_("opening current did not stop before the timeout expired");
    }
    return;
  }

  if (this->state_ == ValveState::CLOSING) {
    if (this->motion_current_check_pending_) {
      if (elapsed < this->running_current_check_interval_ms_) {
        return;
      }

      if (!this->current_above_threshold_throttled_(now, this->running_current_check_interval_ms_, true)) {
        this->set_error_("closing current was not detected in the startup check window");
        return;
      }

      this->motion_current_check_pending_ = false;
    }

    if (!this->current_above_threshold_throttled_(now, this->running_current_check_interval_ms_)) {
      if (elapsed < this->minimum_running_time_ms_) {
        this->set_error_("closing current dropped below threshold too early");
        return;
      }

      this->set_close_output_(false);
      this->set_state_(ValveState::CLOSED);
      return;
    }

    if (elapsed >= this->movement_timeout_ms_) {
      this->set_error_("closing current did not stop before the timeout expired");
    }
  }
}

ValveTraits ValveController::get_traits() {
  auto traits = ValveTraits();
  traits.set_is_assumed_state(true);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  traits.set_supports_position(false);
  return traits;
}

void ValveController::control(const ValveCall &call) {
  if (call.get_stop()) {
    this->all_outputs_off_();
    this->current_operation = VALVE_OPERATION_IDLE;
    this->publish_state();
    return;
  }

  auto toggle = call.get_toggle();
  if (toggle.has_value() && *toggle) {
    if (this->is_fully_open()) {
      this->request_close();
    } else {
      this->request_open();
    }
    return;
  }

  auto requested_position = call.get_position();
  if (requested_position.has_value()) {
    if (*requested_position >= 0.5f) {
      this->request_open();
    } else {
      this->request_close();
    }
  }
}

}  // namespace esphome::valve_controller
