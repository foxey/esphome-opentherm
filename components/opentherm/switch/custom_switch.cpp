#include "custom_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace opentherm {

static const char *const TAG = "custom_switch";

void CustomSwitch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Custom Switch '%s'...", this->name_.c_str());

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

void CustomSwitch::write_state(bool state) { this->publish_state(state); };

}  // namespace opentherm
}  // namespace esphome
