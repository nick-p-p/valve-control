#pragma once

#include <cstdint>

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/valve/valve.h"
#include "esphome/core/component.h"

namespace esphome::valve_controller {

class ValveVersionTextSensor final : public text_sensor::TextSensor {};
class ValveStatusTextSensor final : public text_sensor::TextSensor {};

class ValveController final : public valve::Valve, public Component, public i2c::I2CDevice {
 public:
  void set_open_output(output::BinaryOutput *output) { this->open_output_ = output; }
  void set_close_output(output::BinaryOutput *output) { this->close_output_ = output; }
  void set_current_threshold_amps(float threshold) { this->current_threshold_amps_ = threshold; }
  void set_movement_timeout_ms(uint32_t timeout_ms) { this->movement_timeout_ms_ = timeout_ms; }
  void set_running_current_check_interval_ms(uint32_t interval_ms) {
    this->running_current_check_interval_ms_ = interval_ms;
  }
  void set_idle_current_check_interval_ms(uint32_t interval_ms) { this->idle_current_check_interval_ms_ = interval_ms; }
  void set_minimum_running_time_ms(uint32_t min_time_ms) { this->minimum_running_time_ms_ = min_time_ms; }
  void set_shunt_resistance_ohms(float shunt_resistance_ohms) { this->shunt_resistance_ohms_ = shunt_resistance_ohms; }
  void set_max_expected_current_amps(float max_expected_current_amps) {
    this->max_expected_current_amps_ = max_expected_current_amps;
  }
  void set_version_text_sensor(text_sensor::TextSensor *sensor) { this->version_text_sensor_ = sensor; }
  void set_status_text_sensor(text_sensor::TextSensor *sensor) { this->status_text_sensor_ = sensor; }

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
  void all_outputs_off_();
  void set_open_output_(bool enabled);
  void set_close_output_(bool enabled);
  bool setup_ina219_();
  bool read_current_amps_(float *current_amps);
  bool current_above_threshold_();
  bool current_above_threshold_throttled_(uint32_t now, uint32_t interval_ms, bool force = false);
  void publish_version_text_();
  void publish_status_text_(const char *status_text);
  const char *state_name_(ValveState state) const;
  void start_opening_();
  void start_closing_();
  void set_error_(const char *reason);
  void complete_startup_();
  void process_startup_();
  void process_motion_();

  output::BinaryOutput *open_output_{nullptr};
  output::BinaryOutput *close_output_{nullptr};
  text_sensor::TextSensor *version_text_sensor_{nullptr};
  text_sensor::TextSensor *status_text_sensor_{nullptr};

  ValveState state_{ValveState::UNKNOWN};
  StartupStage startup_stage_{StartupStage::OPEN_TEST};
  uint32_t stage_started_at_{0};
  bool startup_open_current_{false};
  bool startup_close_current_{false};
  bool motion_current_check_pending_{false};
  bool has_cached_current_sample_{false};
  bool cached_current_above_threshold_{false};
  bool ina219_configured_{false};

  float current_threshold_amps_{0.05f};
  float shunt_resistance_ohms_{0.1f};
  float max_expected_current_amps_{3.2f};
  float ina219_current_lsb_amps_{0.0f};
  uint32_t movement_timeout_ms_{30000};
  uint32_t running_current_check_interval_ms_{20};
  uint32_t idle_current_check_interval_ms_{3000};
  uint32_t minimum_running_time_ms_{300};
  uint32_t last_current_sample_at_{0};
  uint32_t motion_started_at_{0};
};

}  // namespace esphome::valve_controller
