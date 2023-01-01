/*
libopentherm.cpp - OpenTherm Communication Library For ESPHome
Based on https://github.com/ihormelnyk/OpenTherm
Licensed under MIT license
Copyright 2018, Ihor Melnyk
Copyright 2022, Michiel Fokke
*/

#include "libopentherm.h"
#include "consts.h"
#include "esphome/core/log.h"

namespace esphome {
namespace opentherm {

OpenTherm::OpenTherm():
is_responder_(false),
status_(OpenThermStatus::NOT_INITIALIZED),
response_(0),
response_status_(OpenThermResponseStatus::NONE),
response_timestamp_(0)
{
}

void OpenTherm::setup(InternalGPIOPin *read_pin, InternalGPIOPin *write_pin, bool is_responder) {
  this->read_pin_ = read_pin;
  this->write_pin_ = write_pin;
  this->is_responder_ = is_responder;
  this->read_pin_->setup();
  this->isr_read_pin_ = this->read_pin_->to_isr();
  this->read_pin_->attach_interrupt(OpenTherm::handle_interrupt, this, gpio::INTERRUPT_ANY_EDGE);    
  this->write_pin_->setup();
  this->activate_boiler_();
  this->status_ = OpenThermStatus::READY;
}

void OpenTherm::setup(InternalGPIOPin *read_pin, InternalGPIOPin *write_pin) {
  this->setup(read_pin, write_pin, false);
}

OpenThermStatus OpenTherm::get_status() {
  return this->status_;
}

bool IRAM_ATTR OpenTherm::is_ready() {
  return this->status_ == OpenThermStatus::READY;
}

int IRAM_ATTR OpenTherm::read_state_() {
  return this->isr_read_pin_.digital_read();
}

void OpenTherm::set_active_state_() {
  this->write_pin_->digital_write(false);
}

void OpenTherm::set_idle_state_() {
  this->write_pin_->digital_write(true);
}

void OpenTherm::activate_boiler_() {
  this->set_idle_state_();
  delay(1000);
}

void OpenTherm::send_bit_(bool high) {
  if (high) set_active_state_(); else set_idle_state_();
  delayMicroseconds(500);
  if (high) set_idle_state_(); else set_active_state_();
  delayMicroseconds(500);
}

bool OpenTherm::send_request_async(uint32_t request)
{
  bool ready = false;
  {
    InterruptLock lock;
    ready = this->status_ == OpenThermStatus::READY;
  }

  if (!ready) {
    return false;
  }

  this->status_ = OpenThermStatus::REQUEST_SENDING;
  this->response_ = 0;
  this->response_status_ = OpenThermResponseStatus::NONE;

  this->send_bit_(true);  // Start bit
  for (int i = 31; i >= 0; i--) {
    this->send_bit_(request >> i & 1);
  }
  this->send_bit_(true);  // Stop bit
  this->set_idle_state_();

  this->status_ = OpenThermStatus::RESPONSE_WAITING;
  this->response_timestamp_ = micros();
  return true;
}

uint32_t OpenTherm::send_request(uint32_t request)
{
  if (!this->send_request_async(request)) return 0;
  while (!this->is_ready()) {
    this->process();
    yield();
  }
  return this->response_;
}

bool OpenTherm::send_response(uint32_t request)
{
  this->status_ = OpenThermStatus::REQUEST_SENDING;
  this->response_ = 0;
  this->response_status_ = OpenThermResponseStatus::NONE;

  this->send_bit_(true); //start bit
  for (int i = 31; i >= 0; i--) {
    this->send_bit_(request >> i & 1);
  }
  this->send_bit_(true); //stop bit
  this->set_idle_state_();
  this->status_ = OpenThermStatus::READY;
  return true;
}

uint32_t OpenTherm::get_response() {
  return this->response_;
}

OpenThermResponseStatus OpenTherm::get_response_status() {
  return this->response_status_;
}

bool OpenTherm::has_response_available() {
  return this->response_status_ != OpenThermResponseStatus::NONE;
}

void OpenTherm::mark_response_read() {
  this->response_status_ = OpenThermResponseStatus::NONE;
}

void IRAM_ATTR OpenTherm::handle_interrupt(OpenTherm *opentherm)
{
  if (opentherm->is_ready())
  {
    if (opentherm->is_responder_ && opentherm->read_state_() == true) {
      opentherm->status_ = OpenThermStatus::RESPONSE_WAITING;
    }
    else {
      return;
    }
  }

  uint32_t new_timestamp = micros();

  if (opentherm->status_ == OpenThermStatus::RESPONSE_WAITING) {
    if (opentherm->read_state_() == true) {
      opentherm->status_ = OpenThermStatus::RESPONSE_START_BIT;
      opentherm->response_timestamp_ = new_timestamp;
    }
    else {
      opentherm->status_ = OpenThermStatus::RESPONSE_INVALID;
      opentherm->response_timestamp_ = new_timestamp;
    }
  }
  else if (opentherm->status_ == OpenThermStatus::RESPONSE_START_BIT) {
    if ((new_timestamp - opentherm->response_timestamp_ < 750) && opentherm->read_state_() == false) {
      opentherm->status_ = OpenThermStatus::RESPONSE_RECEIVING;
      opentherm->response_timestamp_ = new_timestamp;
      opentherm->response_bit_index_ = 0;
    }
    else {
      opentherm->status_ = OpenThermStatus::RESPONSE_INVALID;
      opentherm->response_timestamp_ = new_timestamp;
    }
  }
  else if (opentherm->status_ == OpenThermStatus::RESPONSE_RECEIVING) {
    if ((new_timestamp - opentherm->response_timestamp_) > 750) {
      if (opentherm->response_bit_index_ < 32) {
        opentherm->response_ = (opentherm->response_ << 1) | !opentherm->read_state_();
        opentherm->response_timestamp_ = new_timestamp;
        opentherm->response_bit_index_++;
      }
      else { //stop bit
        opentherm->status_ = OpenThermStatus::RESPONSE_READY;
        opentherm->response_timestamp_ = new_timestamp;
      }
    }
  }
}

void OpenTherm::process()
{
  OpenThermStatus status = OpenThermStatus::NOT_INITIALIZED;
  uint32_t timestamp = 0;
  {
    InterruptLock lock;
    status = this->status_;
    timestamp = this->response_timestamp_;
  }

  if (status == OpenThermStatus::READY) {
    return;
  }
  
  uint32_t new_timestamp = micros();
  if (status != OpenThermStatus::NOT_INITIALIZED && status != OpenThermStatus::DELAY && (new_timestamp - timestamp) > 1000000) {
    this->status_ = OpenThermStatus::READY;
    this->response_status_ = OpenThermResponseStatus::TIMEOUT;
  }
  else if (status == OpenThermStatus::RESPONSE_INVALID) {
    this->status_ = OpenThermStatus::DELAY;
    this->response_status_ = OpenThermResponseStatus::INVALID;
  }
  else if (status == OpenThermStatus::RESPONSE_READY) {
    this->status_ = OpenThermStatus::DELAY;
    this->response_status_ = (this->is_responder_ ? this->is_valid_request(this->response_) : this->is_valid_response(this->response_)) ?
      OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
  }
  else if (status == OpenThermStatus::DELAY) {
    if ((new_timestamp - timestamp) > 100000) {
      this->status_ = OpenThermStatus::READY;
    }
  }
}

bool OpenTherm::parity(uint32_t frame) //odd parity
{
  uint8_t p = 0;
  while (frame > 0) {
    if (frame & 1)
      p++;
    frame = frame >> 1;
  }
  return (p & 1);
}

OpenThermMessageType OpenTherm::get_message_type(uint32_t message)
{
  OpenThermMessageType msg_type = static_cast<OpenThermMessageType>((message >> 28) & 7);
  return msg_type;
}

const char *OpenTherm::message_type_to_string(OpenThermMessageType message_type)
{
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
      return "UNDEFINED";
  }
}

const char *OpenTherm::get_message_type_string(uint32_t message) {
  return OpenTherm::message_type_to_string(OpenTherm::get_message_type(message));
}

OpenThermMessageID OpenTherm::get_data_id(uint32_t frame)
{
  return (OpenThermMessageID)((frame >> 16) & 0xFF);
}

uint32_t OpenTherm::build_request(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
  uint32_t request = data;
  if (type == OpenThermMessageType::WRITE_DATA) {
    request |= 1ul << 28;
  }
  request |= ((uint32_t)id) << 16;
  if (OpenTherm::parity(request)) request |= (1ul << 31);
  return request;
}

uint32_t OpenTherm::build_response(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
  uint32_t response = data;
  response |= type << 28;
  response |= ((uint32_t)id) << 16;
  if (OpenTherm::parity(response)) response |= (1ul << 31);
  return response;
}

bool OpenTherm::is_valid_response(uint32_t response)
{
  if (OpenTherm::parity(response)) return false;
  uint8_t msg_type = (response << 1) >> 29;
  return msg_type == READ_ACK || msg_type == WRITE_ACK;
}

bool OpenTherm::is_valid_request(uint32_t request)
{
  if (OpenTherm::parity(request)) return false;
  uint8_t msg_type = (request << 1) >> 29;
  return msg_type == READ_DATA || msg_type == WRITE_DATA;
}

void OpenTherm::end() {
  if (this->handle_interrupt_callback_ != NULL) {
    this->read_pin_->detach_interrupt();
  }
}

const char *OpenTherm::response_status_to_string(OpenThermResponseStatus response_status)
{
  switch (response_status) {
    case NONE:  return "NONE";
    case SUCCESS: return "SUCCESS";
    case INVALID: return "INVALID";
    case TIMEOUT: return "TIMEOUT";
    default:    return "UNKNOWN";
  }
}

const char *OpenTherm::status_to_string(OpenThermStatus status) {
  switch (status) {
    case NOT_INITIALIZED: return "NOT_INITIALIZED";
    case READY: return "READY";
    case DELAY: return "DELAY";
    case REQUEST_SENDING: return "REQUEST_SENDING";
    case RESPONSE_WAITING: return "RESPONSE_WAITING";
    case RESPONSE_START_BIT: return "RESPONSE_START_BIT";
    case RESPONSE_RECEIVING: return "RESPONSE_RECEIVING";
    case RESPONSE_READY: return "RESPONSE_READY";
    case RESPONSE_INVALID: return "RESPONSE_INVALID";
    default:    return "UNKNOWN";
  }
}

const char *OpenTherm::message_id_to_string(uint32_t frame) {
    switch (OpenTherm::get_data_id(frame)) {
  // Class 1: Control and Status Information
        case OpenThermMessageID::STATUS: return "STATUS";
        case OpenThermMessageID::CH_SETPOINT: return "CH_SETPOINT";
        case OpenThermMessageID::APP_SPEC_FAULT_FLAGS: return "APP_SPEC_FAULT_FLAGS";
        case OpenThermMessageID::CH_SETPOINT_2: return "CH_SETPOINT_2";
        case OpenThermMessageID::OEM_DIAGNOSTIC_CODE: return "OEM_DIAGNOSTIC_CODE";
        // Class 2: Configuration Information
        case OpenThermMessageID::CONTROLLER_CONFIGURATION: return "CONTROLLER_CONFIGURATION";
        case OpenThermMessageID::RESPONDER_CONFIGURATION: return "RESPONDER_CONFIGURATION";
        case OpenThermMessageID::OT_VERSION_CONTROLLER: return "OT_VERSION_CONTROLLER";
        case OpenThermMessageID::OT_VERSION_RESPONDER: return "OT_VERSION_RESPONDER";
        case OpenThermMessageID::CONTROLLER_VERSION: return "CONTROLLER_VERSION";
        case OpenThermMessageID::RESPONDER_VERSION: return "RESPONDER_VERSION";
        // Class 3: Remote Commands
        case OpenThermMessageID::COMMAND: return "COMMAND";
        // Class 4: Sensor and Informational Data
        case OpenThermMessageID::ROOM_SETPOINT: return "ROOM_SETPOINT";
        case OpenThermMessageID::REL_MOD_LEVEL: return "REL_MOD_LEVEL";
        case OpenThermMessageID::CH_PRESSURE: return "CH_PRESSURE";
        case OpenThermMessageID::DHW_FLOW_RATE: return "DHW_FLOW_RATE";
        case OpenThermMessageID::DAY_TIME: return "DAY_TIME";
        case OpenThermMessageID::DATE: return "DATE";
        case OpenThermMessageID::YEAR: return "YEAR";
        case OpenThermMessageID::ROOM_SETPOINT_2: return "ROOM_SETPOINT_2";
        case OpenThermMessageID::ROOM_TEMP: return "ROOM_TEMP";
        case OpenThermMessageID::BOILER_WATER_TEMP: return "BOILER_WATER_TEMP";
        case OpenThermMessageID::DHW_TEMP: return "DHW_TEMP";
        case OpenThermMessageID::OUTSIDE_TEMP: return "OUTSIDE_TEMP";
        case OpenThermMessageID::RETURN_WATER_TEMP: return "RETURN_WATER_TEMP";
        case OpenThermMessageID::SOLAR_STORAGE_TEMP: return "SOLAR_STORAGE_TEMP";
        case OpenThermMessageID::SOLAR_COLL_TEMP: return "SOLAR_COLL_TEMP";
        case OpenThermMessageID::BOILER_WATER_TEMP_2: return "BOILER_WATER_TEMP_2";
        case OpenThermMessageID::DHW_TEMP_2: return "DHW_TEMP_2";
        case OpenThermMessageID::EXHAUST_TEMP: return "EXHAUST_TEMP";
        case OpenThermMessageID::BURNER_STARTS: return "BURNER_STARTS";
        case OpenThermMessageID::CH_PUMP_STARTS: return "CH_PUMP_STARTS";
        case OpenThermMessageID::DHW_PUMP_VALVE_STARTS: return "DHW_PUMP_VALVE_STARTS";
        case OpenThermMessageID::DHW_BURNER_STARTS: return "DHW_BURNER_STARTS";
        case OpenThermMessageID::BURNER_OPS_HOURS: return "BURNER_OPS_HOURS";
        case OpenThermMessageID::CH_PUMP_OPS_HOURS: return "CH_PUMP_OPS_HOURS";
        case OpenThermMessageID::DHW_PUMP_VALVE_OPS_HOURS: return "DHW_PUMP_VALVE_OPS_HOURS";
        case OpenThermMessageID::DHW_BURNER_OPS_HOURS: return "DHW_BURNER_OPS_HOURS";
        // Class 5: Pre-Defined Remote Boiler Parameters
        case OpenThermMessageID::REMOTE_PARAM_FLAGS: return "REMOTE_PARAM_FLAGS";
        case OpenThermMessageID::DHW_TEMP_MAX_MIN: return "DHW_TEMP_MAX_MIN";
        case OpenThermMessageID::CH_TEMP_MAX_MIN: return "CH_TEMP_MAX_MIN";
        case OpenThermMessageID::DHW_SETPOINT: return "DHW_SETPOINT";
        case OpenThermMessageID::MAX_CH_SETPOINT: return "MAX_CH_SETPOINT";
        // Class 6: Transparent Responder Parameters
        case OpenThermMessageID::NR_OF_TSPS: return "NR_OF_TSPS";
        case OpenThermMessageID::TSP_INDEX_VALUE: return "TSP_INDEX_VALUE";
        // Class 7: Fault History Data
        case OpenThermMessageID::FHB_SIZE: return "FHB_SIZE";
        case OpenThermMessageID::FHB_INDEX_VALUE: return "FHB_INDEX_VALUE";
        // Class 8: Control of Special Applications
        case OpenThermMessageID::COOLING_CONTROL: return "COOLING_CONTROL";
        case OpenThermMessageID::MAX_REL_MOD_LEVEL_SETTING: return "MAX_REL_MOD_LEVEL_SETTING";
        case OpenThermMessageID::MAX_CAPACITY_MIN_MOD_LEVEL: return "MAX_CAPACITY_MIN_MOD_LEVEL";
        case OpenThermMessageID::ROOM_TEMP_OVERRIDE: return "ROOM_TEMP_OVERRIDE";
        case OpenThermMessageID::REMOTE_FCT_OVERRIDE: return "REMOTE_FCT_OVERRIDE";
        default: return "UNDEFINED_MESSAGE_ID";
    }
};

//building requests

uint32_t OpenTherm::build_set_boiler_status_request(bool enable_central_heating, bool enable_hot_water, bool enable_cooling, bool enable_outside_temperature_compensation, bool enable_central_heating2) {
  unsigned int data = enable_central_heating | (enable_hot_water << 1) | (enable_cooling << 2) | (enable_outside_temperature_compensation << 3) | (enable_central_heating2 << 4);
  data <<= 8;
  return build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::STATUS, data);
}

uint32_t OpenTherm::build_set_boiler_temperature_request(float temperature) {
  unsigned int data = temperature_to_data(temperature);
  return build_request(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::CH_SETPOINT, data);
}

uint32_t OpenTherm::build_get_boiler_temperature_request() {
  return build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::BOILER_WATER_TEMP, 0);
}

//parsing responses
bool OpenTherm::is_fault(uint32_t response) {
  return response & 0x1;
}

bool OpenTherm::is_central_heating_active(uint32_t response) {
  return response & 0x2;
}

bool OpenTherm::is_hot_water_active(uint32_t response) {
  return response & 0x4;
}

bool OpenTherm::is_flame_on(uint32_t response) {
  return response & 0x8;
}

bool OpenTherm::is_cooling_active(uint32_t response) {
  return response & 0x10;
}

bool OpenTherm::is_diagnostic(uint32_t response) {
  return response & 0x40;
}

uint16_t OpenTherm::get_uint16(const uint32_t response) {
  const uint16_t u88 = response & 0xffff;
  return u88;
}

float OpenTherm::get_float(const uint32_t response) {
  const uint16_t u88 = get_uint16(response);
  const float f = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
  return f;
}

unsigned int OpenTherm::temperature_to_data(float temperature) {
  if (temperature < 0) temperature = 0;
  if (temperature > 100) temperature = 100;
  unsigned int data = (unsigned int)(temperature * 256);
  return data;
}

//basic requests

uint32_t OpenTherm::set_boiler_status(bool enable_central_heating,
                                        bool enable_hot_water,
                                        bool enable_cooling,
                                        bool enable_outside_temperature_compensation,
                                        bool enable_central_heating2) {
  return send_request(
    OpenTherm::build_set_boiler_status_request(enable_central_heating,
                                    enable_hot_water,
                                    enable_cooling,
                                    enable_outside_temperature_compensation,
                                    enable_central_heating2));
}

bool OpenTherm::set_boiler_temperature(float temperature) {
  uint32_t response = this->send_request(OpenTherm::build_set_boiler_temperature_request(temperature));
  return OpenTherm::is_valid_response(response);
}

float OpenTherm::get_boiler_temperature() {
  uint32_t response = this->send_request(OpenTherm::build_get_boiler_temperature_request());
  return OpenTherm::is_valid_response(response) ? OpenTherm::get_float(response) : 0;
}

float OpenTherm::get_return_temperature() {
    uint32_t response = this->send_request(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::RETURN_WATER_TEMP, 0));
    return OpenTherm::is_valid_response(response) ? OpenTherm::get_float(response) : 0;
}

bool OpenTherm::set_dhw_setpoint(float temperature) {
    unsigned int data = OpenTherm::temperature_to_data(temperature);
    uint32_t response = this->send_request(OpenTherm::build_request(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::DHW_SETPOINT, data));
    return OpenTherm::is_valid_response(response);
}
    
float OpenTherm::get_dhw_temperature() {
    uint32_t response = this->send_request(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::DHW_TEMP, 0));
    return OpenTherm::is_valid_response(response) ? OpenTherm::get_float(response) : 0;
}

float OpenTherm::get_modulation() {
    uint32_t response = this->send_request(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::REL_MOD_LEVEL, 0));
    return OpenTherm::is_valid_response(response) ? OpenTherm::get_float(response) : 0;
}

float OpenTherm::get_pressure() {
    uint32_t response = this->send_request(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::CH_PRESSURE, 0));
    return is_valid_response(response) ? get_float(response) : 0;
}

unsigned char OpenTherm::get_fault() {
    return ((send_request(OpenTherm::build_request(OpenThermMessageType::READ_DATA, OpenThermMessageID::APP_SPEC_FAULT_FLAGS, 0)) >> 8) & 0xff);
}

}  // namespace opentherm
}  // namespace esphome