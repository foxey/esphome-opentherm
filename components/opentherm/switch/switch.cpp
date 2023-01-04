#include "switch.h"
#include "esphome/core/log.h"
#include "../consts.h"

namespace esphome {
namespace opentherm {


void Switch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Switch '%s'...", this->name_.c_str());

  bool initial_state = this->get_initial_state_with_restore_mode().value_or(false);
  ESP_LOGCONFIG(TAG, "Restoring state to %s", initial_state ? "ON" : "OFF");

  // write state before setup
  if (initial_state) {
    this->turn_on();
  } else {
    this->turn_off();
  }
  this->publish_state(initial_state);
}

void Switch::write_state(bool state) { this->publish_state(state); };

}  // namespace opentherm
}  // namespace esphome
