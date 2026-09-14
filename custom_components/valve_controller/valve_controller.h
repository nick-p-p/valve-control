#pragma once

#include <cstdint>

#include "esphome/components/output/binary_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/valve/valve.h"
#include "esphome/core/component.h"

namespace esphome::valve_controller {

class ValveStateTextSensor final : public text_sensor::TextSensor, public Component {};

class ValveController final : public valve::Valve, public Component {
 public:
  void set_open_output(output::BinaryOutput *output) { this->open_output_ = output; }
  void set_close_output(output::BinaryOutput *output) { this->close_output_ = output; }
  void set_current_sensor(sensor::Sensor *sensor) { this->current_sensor_ = sensor; }
  void set_state_sensor(text_sensor::TextSensor *sensor) { this->state_sensor_ = sensor; }
  void set_current_threshold_amps(float threshold) { this->current_threshold_amps_ = threshold; }
  void set_movement_timeout_ms(uint32_t timeout_ms) { this->movement_timeout_ms_ = timeout_ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE - 1.0f; }

  void request_open();
  void request_close();

 protected:
  valve::ValveTraits get_traits() override;
  void control(const valve::ValveCall &call) override;

  enum class ValveState : uint8_t {
    UNKNOWN,
    OPEN,
    CLOSED,
    OPENING,
    CLOSING,
    ERROR,
  };

  enum class StartupStage : uint8_t {
    OPEN_TEST,
    CLOSE_NUDGE,
    CLOSE_TEST,
    OPEN_NUDGE,
    DONE,
  };

  void set_state_(ValveState state);
  void publish_state_();
  void all_outputs_off_();
  void set_open_output_(bool enabled);
  void set_close_output_(bool enabled);
  bool current_above_threshold_() const;
  void start_opening_();
  void start_closing_();
  void set_error_(const char *reason);
  void complete_startup_();
  void process_startup_();
  void process_motion_();
  const char *state_name_(ValveState state) const;

  output::BinaryOutput *open_output_{nullptr};
  output::BinaryOutput *close_output_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};

  ValveState state_{ValveState::UNKNOWN};
  StartupStage startup_stage_{StartupStage::OPEN_TEST};
  uint32_t stage_started_at_{0};
  bool startup_open_current_{false};
  bool startup_close_current_{false};
  bool motion_current_check_pending_{false};

  float current_threshold_amps_{0.05f};
  uint32_t movement_timeout_ms_{30000};
  uint32_t motion_started_at_{0};
};

}  // namespace esphome::valve_controller
