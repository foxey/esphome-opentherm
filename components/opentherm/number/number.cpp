#include "esphome/core/log.h"
#include "number.h"
#include "../consts.h"

namespace esphome {
namespace opentherm {

void Number::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Number '%s'...", this->name_.c_str());

   float value{NAN};
  value = this->initial_value_;
  if (this->restore_value_) {
    this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash());
    if (this->pref_.load(&value)) {
      ESP_LOGCONFIG(TAG, "Restoring state to %f", value);
    } else {
      ESP_LOGCONFIG(TAG, "Restoring state failed");
    }
  }
  if (std::isnan(value)) {
    value = this->traits.get_min_value();
  }
  this->publish_state(value);
}

void Number::control(float value) {
  this->publish_state(value);

  if (this->restore_value_) {
    this->pref_.save(&value);
  }
};

void Number::dump_custom_config(const char *prefix, const char *type) {
  LOG_NUMBER(prefix, type, this);
  if (!std::isnan(this->initial_value_)) {
    ESP_LOGCONFIG(TAG, "%s  Initial value: '%f'", prefix, this->initial_value_);
  }
  if (!std::isnan(this->restore_value_)) {
    ESP_LOGCONFIG(TAG, "%s  Restore value: '%s'", prefix, YESNO(this->restore_value_));
  }
}

}  // namespace opentherm
}  // namespace esphome
