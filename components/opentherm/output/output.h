#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/sensor/sensor.h"
#include "../switch/switch.h"
#include "../number/number.h"
#include "../consts.h"

namespace esphome {
namespace opentherm {

class OpenThermFloatOutput : public output::FloatOutput, public Component {
 public:
  void set_ch_enable_switch(opentherm::Switch *ch_enable_switch) { ch_enabled_switch_ = ch_enable_switch; }
  void set_gateway_enabled_switch(opentherm::Switch *gateway_enabled_switch) { gateway_enabled_switch_ = gateway_enabled_switch; }
  void set_control_setpoint_number(opentherm::Number *control_setpoint_number) { control_setpoint_number_ = control_setpoint_number; }
  void set_min_temperature_sensor_(sensor::Sensor *ch_min_temperature_sensor_) { ch_min_temperature_sensor_ = ch_min_temperature_sensor_; }
  void set_max_temperature_sensor_(sensor::Sensor *ch_max_temperature_sensor_) { ch_max_temperature_sensor_ = ch_max_temperature_sensor_; }
  void dump_config() override;

 protected:
  void write_state(float state) override;
  opentherm::Switch *gateway_enabled_switch_{nullptr};
  opentherm::Switch *ch_enabled_switch_{nullptr};
  opentherm::Number *control_setpoint_number_{nullptr};
  sensor::Sensor *ch_min_temperature_sensor_{nullptr};
  sensor::Sensor *ch_max_temperature_sensor_{nullptr};
    
};

}  // namespace opentherm
}  // namespace esphome
