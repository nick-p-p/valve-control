#include "valve_controller.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::valve_controller {

using namespace esphome::valve;

static const char *const TAG = "valve_controller";

void ValveButtonBase::dump_config() { LOG_BUTTON("", "Valve Button", this); }

void ValveOpenButton::press_action() { this->parent_->request_open(); }

void ValveCloseButton::press_action() { this->parent_->request_close(); }

void ValveController::dump_config() {
  ESP_LOGCONFIG(TAG, "Valve Controller:");
  LOG_VALVE("", "Valve Controller", this);
  ESP_LOGCONFIG(TAG, "  Open output: %s", this->open_output_ != nullptr ? "configured" : "missing");
  ESP_LOGCONFIG(TAG, "  Close output: %s", this->close_output_ != nullptr ? "configured" : "missing");
  if (this->current_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Current sensor: %s", this->current_sensor_->get_name().c_str());
  }
  if (this->state_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  State sensor: %s", this->state_sensor_->get_name().c_str());
  }
  ESP_LOGCONFIG(TAG, "  Current threshold: %.3f A", this->current_threshold_amps_);
  ESP_LOGCONFIG(TAG, "  Movement timeout: %u ms", this->movement_timeout_ms_);
}

void ValveController::setup() {
  this->all_outputs_off_();
  this->startup_stage_ = StartupStage::OPEN_TEST;
  this->stage_started_at_ = millis();
  this->startup_open_current_ = false;
  this->startup_close_current_ = false;
  this->state_ = ValveState::ERROR;
  this->set_state_(ValveState::UNKNOWN);
  this->set_open_output_(true);
}

void ValveController::loop() {
  if (this->startup_stage_ != StartupStage::DONE) {
    this->process_startup_();
    return;
  }

  this->process_motion_();
}

void ValveController::request_open() {
  if (this->state_ == ValveState::OPEN) {
    return;
  }

  this->startup_stage_ = StartupStage::DONE;
  this->start_opening_();
}

void ValveController::request_close() {
  if (this->state_ == ValveState::CLOSED) {
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
  this->publish_state_();
}

void ValveController::publish_state_() {
  if (this->state_sensor_ == nullptr) {
    return;
  }

  this->state_sensor_->publish_state(this->state_name_(this->state_));
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

bool ValveController::current_above_threshold_() const {
  return this->current_sensor_ != nullptr && this->current_sensor_->has_state() &&
         this->current_sensor_->state >= this->current_threshold_amps_;
}

void ValveController::start_opening_() {
  this->motion_started_at_ = millis();
  this->motion_current_check_pending_ = true;
  this->set_close_output_(false);
  this->set_open_output_(true);
  this->set_state_(ValveState::OPENING);
}

void ValveController::start_closing_() {
  this->motion_started_at_ = millis();
  this->motion_current_check_pending_ = true;
  this->set_open_output_(false);
  this->set_close_output_(true);
  this->set_state_(ValveState::CLOSING);
}

void ValveController::set_error_(const char *reason) {
  ESP_LOGE(TAG, "Valve state changed to error: %s", reason);
  this->all_outputs_off_();
  this->set_state_(ValveState::ERROR);
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

      this->startup_open_current_ = this->current_above_threshold_();
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

      this->startup_close_current_ = this->current_above_threshold_();
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

  if (this->state_ == ValveState::OPENING) {
    if (this->motion_current_check_pending_) {
      if (!this->current_above_threshold_()) {
        this->set_error_("opening current was not detected within one loop");
        return;
      }

      this->motion_current_check_pending_ = false;
    }

    if (!this->current_above_threshold_()) {
      this->set_open_output_(false);
      this->set_state_(ValveState::OPEN);
      return;
    }

    if (now - this->motion_started_at_ >= this->movement_timeout_ms_) {
      this->set_error_("opening current did not stop before the timeout expired");
    }
    return;
  }

  if (this->state_ == ValveState::CLOSING) {
    if (this->motion_current_check_pending_) {
      if (!this->current_above_threshold_()) {
        this->set_error_("closing current was not detected within one loop");
        return;
      }

      this->motion_current_check_pending_ = false;
    }

    if (!this->current_above_threshold_()) {
      this->set_close_output_(false);
      this->set_state_(ValveState::CLOSED);
      return;
    }

    if (now - this->motion_started_at_ >= this->movement_timeout_ms_) {
      this->set_error_("closing current did not stop before the timeout expired");
    }
  }
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
