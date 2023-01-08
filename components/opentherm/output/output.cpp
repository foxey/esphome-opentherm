#include "esphome/core/log.h"
#include "output.h"
#include "../consts.h"

namespace esphome {
namespace opentherm {

void OpenThermFloatOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "Opentherm Output:");
  LOG_FLOAT_OUTPUT(this);
}

void OpenThermFloatOutput::write_state(float state) {
  if (this->gateway_enabled_switch_ == nullptr) {
    ESP_LOGD(TAG, "Output: Gateway Switch not present");
    return;
  }

  if (this->gateway_enabled_switch_->state) {
    return;
  }

  float ch_setpoint_temperature = 6.9;
  float ch_min_temperature = 18.0;
  float ch_max_temperature = 54.9;

  if (this->ch_min_temperature_sensor_) {
    if (this->ch_min_temperature_sensor_->has_state()) {
      ch_min_temperature = this->ch_min_temperature_sensor_->state;
    // } else {
    //   ESP_LOGW(TAG, "Output: minimum CH temperature sensor has no state");
    }
  } else {
    ESP_LOGW(TAG, "Output: using default minimum CH temperature (%f°C)", ch_min_temperature);
  }
  if (this->ch_max_temperature_sensor_) {
    if (this->ch_max_temperature_sensor_->has_state()) {
      ch_max_temperature = this->ch_max_temperature_sensor_->state;
    // } else {
    //   ESP_LOGW(TAG, "Output: maximum CH temperature sensor has no state");
    }
  // } else {
  //   ESP_LOGW(TAG, "Output: using default maximum CH temperature (%f°C)", ch_max_temperature);
  }

  if (state > 0.0) {
    ch_setpoint_temperature = ch_min_temperature + state * (ch_max_temperature - ch_min_temperature);
    if (this->ch_enabled_switch_) {
      this->ch_enabled_switch_->turn_on();
    } else {
      ESP_LOGW(TAG, "Output: cannot turn on heating.");
    }
  } else {
    if (this->ch_enabled_switch_) {
      this->ch_enabled_switch_->turn_off();
    } else {
      ESP_LOGW(TAG, "Output: cannot turn off heating.");
    }
  }

  if (this->control_setpoint_number_) {
    this->control_setpoint_number_->publish_state(ch_setpoint_temperature);
    ESP_LOGI(TAG, "Output set to %d%% (%f°C)", int(state*100.0), ch_setpoint_temperature);
  } else {
    ESP_LOGW(TAG, "Output: failed to set to %d%% (%f°C)", int(state*100.0), ch_setpoint_temperature);
  }
}

}  //  namespace opentherm
}  //  namespace esphome