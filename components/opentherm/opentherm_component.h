#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "switch/custom_switch.h"
#include "number/custom_number.h"
#include "consts.h"
#include "enums.h"
#include "libopentherm.h"
#include <queue>

namespace esphome {
namespace opentherm {

class OpenThermComponent : public PollingComponent {
 public:
  sensor::Sensor *ch_min_temperature_sensor_{nullptr};
  sensor::Sensor *ch_max_temperature_sensor_{nullptr};
  sensor::Sensor *dhw_min_temperature_sensor_{nullptr};
  sensor::Sensor *dhw_max_temperature_sensor_{nullptr};
  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *modulation_sensor_{nullptr};
  sensor::Sensor *boiler_temperature_sensor_{nullptr};
  sensor::Sensor *return_temperature_sensor_{nullptr};
  sensor::Sensor *room_temperature_sensor_{nullptr};
  binary_sensor::BinarySensor *ch_active_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *dhw_active_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *cooling_active_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *flame_active_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *fault_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *diagnostic_binary_sensor_{nullptr};
  opentherm::CustomSwitch *gateway_enabled_switch_{nullptr};
  opentherm::CustomSwitch *ch_enabled_switch_{nullptr};
  opentherm::CustomSwitch *dhw_enabled_switch_{nullptr};
  opentherm::CustomSwitch *cooling_enabled_switch_{nullptr};
  opentherm::CustomNumber *ch_setpoint_temperature_number_{nullptr};
  opentherm::CustomNumber *dhw_setpoint_temperature_number_{nullptr};
  opentherm::CustomNumber *room_setpoint_temperature_number_{nullptr};

  OpenThermComponent() = default;

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  void set_pins(InternalGPIOPin *responder_read_pin, InternalGPIOPin *responder_write_pin,
    InternalGPIOPin *controller_read_pin, InternalGPIOPin *controller_write_pin);

  void set_ch_min_temperature_sensor(sensor::Sensor *sensor) { ch_min_temperature_sensor_ = sensor; }
  void set_ch_max_temperature_sensor(sensor::Sensor *sensor) { ch_max_temperature_sensor_ = sensor; }
  void set_dhw_min_temperature_sensor(sensor::Sensor *sensor) { dhw_min_temperature_sensor_ = sensor; }
  void set_dhw_max_temperature_sensor(sensor::Sensor *sensor) { dhw_max_temperature_sensor_ = sensor; }
  void set_pressure_sensor(sensor::Sensor *sensor) { pressure_sensor_ = sensor; }
  void set_modulation_sensor(sensor::Sensor *sensor) { modulation_sensor_ = sensor; }
  void set_boiler_temperature_sensor(sensor::Sensor *sensor) { boiler_temperature_sensor_ = sensor; }
  void set_return_temperature_sensor(sensor::Sensor *sensor) { return_temperature_sensor_ = sensor; }
  void set_room_temperature_sensor(sensor::Sensor *sensor) { room_temperature_sensor_ = sensor; }
  void set_ch_active_binary_sensor(binary_sensor::BinarySensor *sensor) { ch_active_binary_sensor_ = sensor; }
  void set_dhw_active_binary_sensor(binary_sensor::BinarySensor *sensor) { dhw_active_binary_sensor_ = sensor; }
  void set_cooling_active_binary_sensor(binary_sensor::BinarySensor *sensor) { cooling_active_binary_sensor_ = sensor; }
  void set_flame_active_binary_sensor(binary_sensor::BinarySensor *sensor) { flame_active_binary_sensor_ = sensor; }
  void set_fault_binary_sensor(binary_sensor::BinarySensor *sensor) { fault_binary_sensor_ = sensor; }
  void set_diagnostic_binary_sensor(binary_sensor::BinarySensor *sensor) { diagnostic_binary_sensor_ = sensor; }
  void set_gateway_enabled_switch(opentherm::CustomSwitch *custom_switch) { gateway_enabled_switch_ = custom_switch; }
  void set_ch_enabled_switch(opentherm::CustomSwitch *custom_switch) { ch_enabled_switch_ = custom_switch; }
  void set_dhw_enabled_switch(opentherm::CustomSwitch *custom_switch) { dhw_enabled_switch_ = custom_switch; }
  void set_cooling_enabled_switch(opentherm::CustomSwitch *custom_switch) { cooling_enabled_switch_ = custom_switch; }
  void set_ch_setpoint_temperature_number(opentherm::CustomNumber *number) { ch_setpoint_temperature_number_ = number; }
  void set_dhw_setpoint_temperature_number(opentherm::CustomNumber *number) { dhw_setpoint_temperature_number_ = number; }
  void set_room_setpoint_temperature_number(opentherm::CustomNumber *number) { room_setpoint_temperature_number_ = number; }

 protected:
  InternalGPIOPin *responder_read_pin_;
  InternalGPIOPin *responder_write_pin_;

  OpenTherm responder_;

  InternalGPIOPin *controller_read_pin_;
  InternalGPIOPin *controller_write_pin_;

  OpenTherm controller_;

  std::queue<uint32_t> buffer_;
  bool ch_min_max_read_ = false;
  bool dhw_min_max_read_ = false;
  float confirmed_dhw_setpoint_ = 0;
  float confirmed_room_setpoint_ = 0;
  uint32_t last_millis_ = 0;
  bool gateway_enabled_ = false;
  bool wanted_ch_enabled_ = false;
  bool wanted_dhw_enabled_ = false;
  bool wanted_cooling_enabled_ = false;

  void set_boiler_status_();

  void enqueue_request_(uint32_t request);
  void process_responder_response_(uint32_t response, OpenThermResponseStatus response_status);
  void process_controller_response_(uint32_t response, OpenThermResponseStatus response_status);

  void log_message_(uint8_t level, const char *pre_message, uint32_t message);
  
  void publish_sensor_state_(sensor::Sensor *sensor, float state);
  void publish_binary_sensor_state_(binary_sensor::BinarySensor *sensor, bool state);
  void publish_number_state_(opentherm::CustomNumber *number, float state);
  void publish_switch_state_(opentherm::CustomSwitch *custom_switch, bool state);
};

}  // namespace opentherm
}  // namespace esphome
