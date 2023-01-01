/*
 * Notice:
 *   The code for actual communication with OpenTherm is heavily based on Ihor Melnyk's
 *   OpenTherm Library at https://github.com/ihormelnyk/opentherm_library, which is MIT licensed.
 */

#include "opentherm_component.h"
#include "libopentherm.h"
#include "esphome/core/log.h"
#include <vector>

namespace esphome {
namespace opentherm {

// All public method implementations

void OpenThermComponent::set_pins(InternalGPIOPin *responder_read_pin, InternalGPIOPin *responder_write_pin,
  InternalGPIOPin *controller_read_pin, InternalGPIOPin *controller_write_pin) {
  this->responder_read_pin_ = responder_read_pin;
  this->responder_write_pin_ = responder_write_pin;
  this->controller_read_pin_ = controller_read_pin;
  this->controller_write_pin_ = controller_write_pin;
}

void OpenThermComponent::setup() {
  this->responder_.setup(this->responder_read_pin_, this->responder_write_pin_, true);
  this->controller_.setup(this->controller_read_pin_, this->controller_write_pin_);

  if (this->ch_enabled_switch_) {
    this->ch_enabled_switch_->add_on_state_callback([this](bool enabled) {
      if (this->wanted_ch_enabled_ != enabled) {
        ESP_LOGI(TAG, "%s CH", (enabled ? "Enabled" : "Disabled"));
        this->wanted_ch_enabled_ = enabled;
        this->set_boiler_status_();
      }
    });
  }
  if (this->dhw_enabled_switch_) {
    this->dhw_enabled_switch_->add_on_state_callback([this](bool enabled) {
      if (this->wanted_dhw_enabled_ != enabled) {
        ESP_LOGI(TAG, "%s DHW", (enabled ? "Enabled" : "Disabled"));
        this->wanted_dhw_enabled_ = enabled;
        this->set_boiler_status_();
      }
    });
  }
  if (this->cooling_enabled_switch_) {
    this->cooling_enabled_switch_->add_on_state_callback([this](bool enabled) {
      if (this->wanted_cooling_enabled_ != enabled) {
        ESP_LOGI(TAG, "%s cooling", (enabled ? "Enabled" : "Disabled"));
        this->wanted_cooling_enabled_ = enabled;
        this->set_boiler_status_();
      }
    });
  }
  if (this->ch_setpoint_temperature_number_) {
    this->ch_setpoint_temperature_number_->setup();
    this->ch_setpoint_temperature_number_->add_on_state_callback(
        [](float temperature) { ESP_LOGI(TAG, "Request updating CH setpoint to %f", temperature); });
  }
  if (this->dhw_setpoint_temperature_number_) {
    this->dhw_setpoint_temperature_number_->setup();
    this->dhw_setpoint_temperature_number_->add_on_state_callback(
        [](float temperature) { ESP_LOGI(TAG, "Request updating DHW setpoint to %f", temperature); });
  }
}

void OpenThermComponent::loop() {
  if (this->controller_.is_ready() && !buffer_.empty()) {
    uint32_t request = buffer_.front();
    buffer_.pop();
    this->controller_.send_request_async(request);
    this->log_message_(0, "Request sent", request);
  }

  if (millis() - this->last_millis_ > 2000) {
    // The CH setpoint must be written at a fast interval or the boiler
    // might revert to a build-in default as a safety measure.
    this->last_millis_ = millis();
    this->enqueue_request_(
        OpenTherm::build_request(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::CH_SETPOINT,
                             OpenTherm::temperature_to_data(this->ch_setpoint_temperature_number_->state)));
    if (this->confirmed_dhw_setpoint_ != this->dhw_setpoint_temperature_number_->state) {
      this->enqueue_request_(
          OpenTherm::build_request(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::DHW_SETPOINT,
                               OpenTherm::temperature_to_data(this->dhw_setpoint_temperature_number_->state)));
    }
  }


  this->controller_.process();
  if (this->controller_.has_response_available()) {
    OpenThermResponseStatus response_status = this->controller_.get_response_status();
    uint32_t response = this->controller_.get_response();  
    this->process_controller_response_(response, response_status);
    this->controller_.mark_response_read();
  }
  yield();


  this->responder_.process();
  if (this->responder_.has_response_available()) {
    OpenThermResponseStatus response_status = this->responder_.get_response_status();
    uint32_t response = this->responder_.get_response();  
    this->process_responder_response_(response, response_status);
    this->responder_.mark_response_read();
  }
  yield();

}

void OpenThermComponent::update() {
  this->enqueue_request_(
      OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::RETURN_WATER_TEMP, 0));
  this->enqueue_request_(
      OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::BOILER_WATER_TEMP, 0));
  this->enqueue_request_(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::CH_PRESSURE, 0));
  this->enqueue_request_(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::REL_MOD_LEVEL, 0));
  this->enqueue_request_(
      OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::REMOTE_PARAM_FLAGS, 0));
  if (!this->dhw_min_max_read_) {
    this->enqueue_request_(
        OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::DHW_TEMP_MAX_MIN, 0));
  }
  if (!this->ch_min_max_read_) {
    this->enqueue_request_(
        OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::CH_TEMP_MAX_MIN, 0));
  }
  this->set_boiler_status_();
}

void OpenThermComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "OpenTherm:");
  LOG_PIN("  Controller Write Pin: ", this->controller_write_pin_);
  LOG_PIN("  Controller Read Pin: ", this->controller_read_pin_);
  LOG_SENSOR("  ", "CH min temperature:", this->ch_min_temperature_sensor_);
  LOG_SENSOR("  ", "CH max temperature:", this->ch_max_temperature_sensor_);
  LOG_SENSOR("  ", "DHW min temperature:", this->dhw_min_temperature_sensor_);
  LOG_SENSOR("  ", "DHW max temperature:", this->dhw_max_temperature_sensor_);
  LOG_SENSOR("  ", "Pressure:", this->pressure_sensor_);
  LOG_SENSOR("  ", "Modulation:", this->modulation_sensor_);
  LOG_SENSOR("  ", "Boiler temperature:", this->boiler_temperature_sensor_);
  LOG_SENSOR("  ", "Return temperature:", this->return_temperature_sensor_);
  LOG_BINARY_SENSOR("  ", "CH active:", this->ch_active_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "DHW active:", this->dhw_active_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Cooling active:", this->cooling_active_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Flame active:", this->flame_active_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Fault:", fault_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Diagnostic:", this->diagnostic_binary_sensor_);
  LOG_SWITCH("  ", "CH enabled:", this->ch_enabled_switch_);
  LOG_SWITCH("  ", "DHW enabled:", this->dhw_enabled_switch_);
  LOG_SWITCH("  ", "Cooling enabled:", this->cooling_enabled_switch_);
  if (this->ch_setpoint_temperature_number_) {
    this->ch_setpoint_temperature_number_->dump_custom_config("  ", "CH setpoint temperature:");
  }
  if (this->dhw_setpoint_temperature_number_) {
    this->dhw_setpoint_temperature_number_->dump_custom_config("  ", "DHW setpoint temperature:");
  }
}

// Private

void OpenThermComponent::log_message_(uint8_t level, const char *pre_message, uint32_t message) {
  switch (level) {
    case 0:
      ESP_LOGD(TAG, "%s: %s (%s, 0x%04hX)", pre_message, OpenTherm::get_message_type_string(message),
               OpenTherm::message_id_to_string(message), OpenTherm::get_uint16(message));
      break;
    default:
      ESP_LOGW(TAG, "%s: %s (%s, 0x%04hX)", pre_message, OpenTherm::get_message_type_string(message),
               OpenTherm::message_id_to_string(message), OpenTherm::get_uint16(message));
  }
}

void OpenThermComponent::process_responder_response_(uint32_t response, OpenThermResponseStatus response_status) {
  if (response_status == OpenThermResponseStatus::NONE) {
    ESP_LOGW(TAG, "Responder: OpenTherm is not initialized");
  } else if (response_status == OpenThermResponseStatus::TIMEOUT) {
    this->log_message_(2, "Responder: Request timeout", response);
  } else if (response_status == OpenThermResponseStatus::INVALID) {
    this->log_message_(2, "Responder: Received invalid response", response);
  } else if (response_status == OpenThermResponseStatus::SUCCESS) {
    this->log_message_(0, "Responder: Received response", response);
    // Following code is synchronous and adds a 30~60ms delay in the loop
    // I am happy to get help to make this (and the OpenTherm::send_bit_ call) asynchronous
    if (OpenTherm::is_valid_request(response)) {
      uint32_t controller_response = 0;
      switch (OpenTherm::get_data_id(response)) {
        case OpenThermMessageID::STATUS:
          this->publish_switch_state_(this->ch_enabled_switch_, OpenTherm::want_central_heating_active(response));
          this->publish_switch_state_(this->dhw_enabled_switch_, OpenTherm::want_hot_water_active(response));
          this->publish_switch_state_(this->cooling_enabled_switch_, OpenTherm::want_cooling_active(response));
          controller_response = this->controller_.send_request(response);
          if (!this->responder_.send_response(controller_response)) {
            this->log_message_(2, "Responder: Error sending response", controller_response);
          }
          break;
        case OpenThermMessageID::CH_SETPOINT:
          this->publish_number_state_(this->ch_setpoint_temperature_number_, OpenTherm::get_float(response));
          controller_response = OpenTherm::build_response(OpenThermMessageType::WRITE_ACK, OpenThermMessageID::CH_SETPOINT, OpenTherm::get_uint16(response));
          this->log_message_(0, "Responder: Acknowledge CH_SETPOINT request", controller_response);
          if (!this->responder_.send_response(controller_response)) {
            this->log_message_(2, "Responder: Error sending response", controller_response);
          }
          break;
        case OpenThermMessageID::DHW_SETPOINT:
          this->publish_number_state_(this->dhw_setpoint_temperature_number_, OpenTherm::get_float(response));
          controller_response = OpenTherm::build_response(OpenThermMessageType::WRITE_ACK, OpenThermMessageID::DHW_SETPOINT, OpenTherm::get_uint16(response));
          this->log_message_(0, "Responder: Acknowledge DHW_SETPOINT request", controller_response);
          if (!this->responder_.send_response(controller_response)) {
            this->log_message_(2, "Responder: Error sending response", controller_response);
          }
          break;
        default:
          controller_response = this->controller_.send_request(response);
          if (!this->responder_.send_response(controller_response)) {
            this->log_message_(2, "Responder: Error sending response", controller_response);
          }
          break;
      }
    }
  }
}

void OpenThermComponent::process_controller_response_(uint32_t response, OpenThermResponseStatus response_status) {
  if (response_status == OpenThermResponseStatus::SUCCESS) {
    this->log_message_(0, "Controller: Received response", response);
    switch (OpenTherm::get_data_id(response)) {
      case OpenThermMessageID::STATUS:
        this->publish_binary_sensor_state_(this->ch_active_binary_sensor_, OpenTherm::is_central_heating_active(response));
        this->publish_binary_sensor_state_(this->dhw_active_binary_sensor_, OpenTherm::is_hot_water_active(response));
        this->publish_binary_sensor_state_(this->cooling_active_binary_sensor_, OpenTherm::is_cooling_active(response));
        this->publish_binary_sensor_state_(this->flame_active_binary_sensor_, OpenTherm::is_flame_on(response));
        this->publish_binary_sensor_state_(this->fault_binary_sensor_, OpenTherm::is_fault(response));
        this->publish_binary_sensor_state_(this->diagnostic_binary_sensor_, OpenTherm::is_diagnostic(response));
        break;
      case OpenThermMessageID::RETURN_WATER_TEMP:
        this->publish_sensor_state_(this->return_temperature_sensor_, OpenTherm::get_float(response));
        break;
      case OpenThermMessageID::BOILER_WATER_TEMP:
        this->publish_sensor_state_(this->boiler_temperature_sensor_, OpenTherm::get_float(response));
        break;
      case OpenThermMessageID::CH_PRESSURE:
        this->publish_sensor_state_(this->pressure_sensor_, OpenTherm::get_float(response));
        break;
      case OpenThermMessageID::REL_MOD_LEVEL:
        this->publish_sensor_state_(this->modulation_sensor_, OpenTherm::get_float(response));
        break;
      case OpenThermMessageID::DHW_TEMP_MAX_MIN:
        this->dhw_min_max_read_ = true;
        this->publish_sensor_state_(this->dhw_max_temperature_sensor_, response >> 8 & 0xFF);
        this->publish_sensor_state_(this->dhw_min_temperature_sensor_, response & 0xFF);
        break;
      case OpenThermMessageID::CH_TEMP_MAX_MIN:
        this->ch_min_max_read_ = true;
        this->publish_sensor_state_(this->ch_max_temperature_sensor_, response >> 8 & 0xFF);
        this->publish_sensor_state_(this->ch_min_temperature_sensor_, response & 0xFF);
        break;
      case OpenThermMessageID::DHW_SETPOINT:
        if (OpenTherm::get_message_type(response) == OpenThermMessageType::WRITE_ACK) {
          this->confirmed_dhw_setpoint_ = OpenTherm::get_float(response);
        }
        break;
      default:
        ESP_LOGD(TAG, "Controller: Ignored response with id %s", OpenTherm::message_id_to_string(response));
        break;
    }
  } else if (response_status == OpenThermResponseStatus::NONE) {
    ESP_LOGW(TAG, "Controller: OpenTherm is not initialized");
  } else if (response_status == OpenThermResponseStatus::TIMEOUT) {
    ESP_LOGW(TAG, "Controller: Request timeout");
  } else if (response_status == OpenThermResponseStatus::INVALID) {
    this->log_message_(2, "Controller: Received invalid response", response);
  }
}

void OpenThermComponent::publish_sensor_state_(sensor::Sensor *sensor, float state) {
  if (sensor) {
    sensor->publish_state(state);
  }
}

void OpenThermComponent::publish_binary_sensor_state_(binary_sensor::BinarySensor *sensor, bool state) {
  if (sensor) {
    sensor->publish_state(state);
  }
}

void OpenThermComponent::publish_number_state_(opentherm::CustomNumber *number, float state) {
  if (number) {
    number->publish_state(state);
  }
}

void OpenThermComponent::publish_switch_state_(opentherm::CustomSwitch *custom_switch, bool state) {
  if (custom_switch) {
    custom_switch->publish_state(state);
  }
}


void OpenThermComponent::set_boiler_status_() {
  // Fields: CH enabled | DHW enabled | cooling | outside temperature compensation | central heating 2
  unsigned int status_request = OpenTherm::build_set_boiler_status_request(this->wanted_ch_enabled_,
                                                                  this->wanted_dhw_enabled_,
                                                                  this->wanted_cooling_enabled_);
  this->enqueue_request_(status_request);
}

void OpenThermComponent::enqueue_request_(uint32_t request) {
  if (this->buffer_.size() > 20) {
    this->log_message_(2, "Queue full. Discarded request", request);
  } else {
    this->buffer_.push(request);
    this->log_message_(0, "Enqueued request", request);
  }
}

}  // namespace opentherm
}  // namespace esphome
