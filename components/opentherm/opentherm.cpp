/*
 * Notice:
 *   The code for actual communication with OpenTherm is heavily based on Ihor Melnyk's
 *   OpenTherm Library at https://github.com/ihormelnyk/opentherm_library, which is MIT licensed.
 */

#include "opentherm.h"
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
  this->controller_.setup(this->controller_read_pin_, this->controller_write_pin_);
  this->controller_.begin();

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
        [](float temperature) { ESP_LOGI(TAG, "Request updating CH setpoint to %f", temperature); });
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
                             this->temperature_to_data_(this->ch_setpoint_temperature_number_->state)));
    if (this->confirmed_dhw_setpoint_ != this->dhw_setpoint_temperature_number_->state) {
      this->enqueue_request_(
          OpenTherm::build_request(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::DHW_SETPOINT,
                               this->temperature_to_data_(this->dhw_setpoint_temperature_number_->state)));
    }
  }
  this->process_();
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
      ESP_LOGD(TAG, "%s: %s(%s, 0x%04hX)", pre_message, this->format_message_type_(message),
               OpenTherm::message_id_to_string(message), this->get_uint16_(message));
      break;
    default:
      ESP_LOGW(TAG, "%s: %s(%s, 0x%04hX)", pre_message, this->format_message_type_(message),
               OpenTherm::message_id_to_string(message), this->get_uint16_(message));
  }
}

void OpenThermComponent::process_() {
  this->controller_.process();
  // OpenThermStatus status = this->controller_.get_status();
  OpenThermResponseStatus response_status = this->controller_.get_response_status();
  uint32_t response = this->controller_.get_response();

  // ESP_LOGD(TAG, "status: %s, response_status: %s", OpenTherm::status_to_string(status),
  //   OpenTherm::response_status_to_string(response_status));

  // if (status == OpenThermStatus::READY && response_status == OpenThermResponseStatus::TIMEOUT) {
  //   this->process_response_(response, response_status);
  // } else if (status == OpenThermStatus::DELAY && response_status != OpenThermResponseStatus::NONE) {
  //   this->process_response_(response, response_status);  
  // }
  
  if (response_status != OpenThermResponseStatus::NONE) {
    this->process_response_(response, response_status);
    this->controller_.reset_response_status();
  }

}

// void OpenThermComponent::process_() {
//   OpenThermStatus st = OpenThermStatus::NOT_INITIALIZED;
//   uint32_t ts = 0;
//   {
//     InterruptLock lock;
//     st = this->status_;
//     ts = this->response_timestamp_;
//   }

//   if (st == OpenThermStatus::READY) {
//     return;
//   }

//   uint32_t new_timestamp = micros();
//   if (st != OpenThermStatus::NOT_INITIALIZED && st != OpenThermStatus::DELAY && (new_timestamp - ts) > 1000000) {
//     this->status_ = OpenThermStatus::READY;
//     this->response_status_ = OpenThermResponseStatus::TIMEOUT;
//     this->process_response_(this->response_, this->response_status_);
//   } else if (st == OpenThermStatus::RESPONSE_INVALID) {
//     this->status_ = OpenThermStatus::DELAY;
//     this->response_status_ = OpenThermResponseStatus::INVALID;
//     this->process_response_(this->response_, this->response_status_);
//   } else if (st == OpenThermStatus::RESPONSE_READY) {
//     this->status_ = OpenThermStatus::DELAY;
//     this->response_status_ =
//         OpenTherm::is_valid_response(this->response_) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
//     this->process_response_(this->response_, this->response_status_);
//   } else if (st == OpenThermStatus::DELAY) {
//     if ((new_timestamp - ts) > 100000) {
//       this->status_ = OpenThermStatus::READY;
//     }
//   }
// }

OpenThermMessageType OpenThermComponent::get_message_type_(uint32_t message) {
  OpenThermMessageType messsage_type = static_cast<OpenThermMessageType>((message >> 28) & 7);
  return messsage_type;
}

OpenThermMessageID OpenThermComponent::get_data_id_(uint32_t frame) {
  return (OpenThermMessageID)((frame >> 16) & 0xFF);
}

const char *OpenThermComponent::message_type_to_string_(OpenThermMessageType message_type) {
  switch (message_type) {
    case READ_DATA:
      return "READ_DATA";
    case WRITE_DATA:
      return "WRITE_DATA";
    case INVALID_DATA:
      return "INVALID_DATA";
    case RESERVED:
      return "RESERVED";
    case READ_ACK:
      return "READ_ACK";
    case WRITE_ACK:
      return "WRITE_ACK";
    case DATA_INVALID:
      return "DATA_INVALID";
    case UNKNOWN_DATA_ID:
      return "UNKNOWN_DATA_ID";
    default:
      return "UNKNOWN";
  }
}

void OpenThermComponent::process_response_(uint32_t response, OpenThermResponseStatus response_status) {
  if (response_status == OpenThermResponseStatus::SUCCESS) {
    this->log_message_(0, "Received response", response);
    switch (this->get_data_id_(response)) {
      case OpenThermMessageID::STATUS:
        this->publish_binary_sensor_state_(this->ch_active_binary_sensor_, this->is_central_heating_active_(response));
        this->publish_binary_sensor_state_(this->dhw_active_binary_sensor_, this->is_hot_water_active_(response));
        this->publish_binary_sensor_state_(this->cooling_active_binary_sensor_, this->is_cooling_active_(response));
        this->publish_binary_sensor_state_(this->flame_active_binary_sensor_, this->is_flame_on_(response));
        this->publish_binary_sensor_state_(this->fault_binary_sensor_, this->is_fault_(response));
        this->publish_binary_sensor_state_(this->diagnostic_binary_sensor_, this->is_diagnostic_(response));
        break;
      case OpenThermMessageID::RETURN_WATER_TEMP:
        this->publish_sensor_state_(this->return_temperature_sensor_, this->get_float_(response));
        break;
      case OpenThermMessageID::BOILER_WATER_TEMP:
        this->publish_sensor_state_(this->boiler_temperature_sensor_, this->get_float_(response));
        break;
      case OpenThermMessageID::CH_PRESSURE:
        this->publish_sensor_state_(this->pressure_sensor_, this->get_float_(response));
        break;
      case OpenThermMessageID::REL_MOD_LEVEL:
        this->publish_sensor_state_(this->modulation_sensor_, this->get_float_(response));
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
        if (this->get_message_type_(response) == OpenThermMessageType::WRITE_ACK) {
          this->confirmed_dhw_setpoint_ = this->get_float_(response);
        }
        break;
      default:
        break;
    }
  } else if (response_status == OpenThermResponseStatus::NONE) {
    ESP_LOGW(TAG, "OpenTherm is not initialized");
  } else if (response_status == OpenThermResponseStatus::TIMEOUT) {
    ESP_LOGW(TAG, "Request timeout");
  } else if (response_status == OpenThermResponseStatus::INVALID) {
    this->log_message_(2, "Received invalid response", response);
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

void OpenThermComponent::set_boiler_status_() {
  // Fields: CH enabled | DHW enabled | cooling | outside temperature compensation | central heating 2
  unsigned int data = this->wanted_ch_enabled_ | (this->wanted_dhw_enabled_ << 1) |
                      (this->wanted_cooling_enabled_ << 2) | (false << 3) | (false << 4);
  data <<= 8;
  this->enqueue_request_(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::STATUS, data));
}

void OpenThermComponent::enqueue_request_(uint32_t request) {
  if (this->buffer_.size() > 20) {
    this->log_message_(2, "Queue full. Discarded request", request);
  } else {
    this->buffer_.push(request);
    this->log_message_(0, "Enqueued request", request);
  }
}

const char *OpenThermComponent::format_message_type_(uint32_t message) {
  return this->message_type_to_string_(this->get_message_type_(message));
}

}  // namespace opentherm
}  // namespace esphome
