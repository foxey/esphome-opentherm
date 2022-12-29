/*
libopentherm.h - OpenTherm Library for the ESPHome platform
Based on https://github.com/ihormelnyk/OpenTherm
Licensed under MIT license
Copyright 2018, Ihor Melnyk
Copyright 2022, Michiel Fokke

Frame Structure:
P MGS-TYPE SPARE DATA-ID  DATA-VALUE
0 000  0000  00000000 00000000 00000000

This library uses controller and responder instead of the OpenTherm master and slave concepts.

*/

#pragma once

#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "enums.h"

namespace esphome {
namespace opentherm {

class OpenTherm
{
public:
  OpenTherm(InternalGPIOPin *read_pin, InternalGPIOPin *write_pin, bool is_responder = false);
  void begin(void(*handle_interrupt_callback)(OpenTherm *opentherm));
  void begin(void(*handle_interrupt_callback)(OpenTherm *opentherm), void(*process_response_callback)(uint32_t response, OpenThermResponseStatus response_status));
  OpenThermStatus get_status(void);
  bool is_ready();
  uint32_t send_request(uint32_t request);
  bool send_response(uint32_t request);
  bool send_request_async(uint32_t request);
  uint32_t build_request(OpenThermMessageType type, OpenThermMessageID id, unsigned int data);
  uint32_t build_response(OpenThermMessageType type, OpenThermMessageID id, unsigned int data);
  uint32_t get_last_response();
  OpenThermResponseStatus get_last_response_status();
  const char *response_status_to_string(OpenThermResponseStatus response_status);
  void handle_interrupt(OpenTherm *);
  void process();
  void end();

  static bool parity(uint32_t frame);
  static OpenThermMessageType get_message_type(uint32_t message);
  static OpenThermMessageID get_data_id(uint32_t frame);
  static const char *message_type_to_string(OpenThermMessageType message_type);
  static const char *message_id_to_string(uint32_t frame);
  bool is_valid_request(uint32_t request);
  bool is_valid_response(uint32_t response);

  //requests
  uint32_t build_set_boiler_status_request(bool enable_central_heating, bool enable_hot_water = false, bool enable_cooling = false, bool enable_outside_temperature_compensation = false, bool enable_central_heating2 = false);
  uint32_t build_set_boiler_temperature_request(float temperature);
  uint32_t build_get_boiler_temperature_request();

  //responses
  bool is_fault(uint32_t response);
  bool is_central_heating_active(uint32_t response);
  bool is_hot_water_active(uint32_t response);
  bool is_flame_on(uint32_t response);
  bool is_cooling_active(uint32_t response);
  bool is_diagnostic(uint32_t response);
  uint16_t get_uint(const uint32_t response) const;
  float get_float(const uint32_t response) const;  
  unsigned int temperature_to_data(float temperature);

  //basic requests
  uint32_t set_boiler_status(bool enable_central_heating, bool enable_hot_water = false, bool enable_cooling = false, bool enable_outside_temperature_compensation = false, bool enable_central_heating2 = false);
  bool set_boiler_temperature(float temperature);
  float get_boiler_temperature();
  float get_return_temperature();
  bool set_dhw_setpoint(float temperature);
  float get_dhw_temperature();
  float get_modulation();
  float get_pressure();
  unsigned char get_fault();

protected:
  InternalGPIOPin *read_pin_;
  InternalGPIOPin *write_pin_;
  ISRInternalGPIOPin isr_read_pin_;
  const bool is_responder_;

  volatile OpenThermStatus status_;
  volatile uint32_t response_;
  volatile OpenThermResponseStatus response_status_;
  volatile uint32_t response_timestamp_;
  volatile uint8_t response_bit_index_;

  int read_state_();
  void set_active_state_();
  void set_idle_state_();
  void activate_boiler_();

  void send_bit_(bool high);
  void(*handle_interrupt_callback_)(OpenTherm *);
  void(*process_response_callback_)(uint32_t, OpenThermResponseStatus);
};

}  // namespace opentherm
}  // namespace esphome

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
