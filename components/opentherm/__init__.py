import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
)

CONF_RESPONDER_READ_PIN = "responder_read_pin"
CONF_RESPONDER_WRITE_PIN = "responder_write_pin"
CONF_CONTROLLER_READ_PIN = "controller_read_pin"
CONF_CONTROLLER_WRITE_PIN = "controller_write_pin"

CODEOWNERS = ["@foxey"]
DEPENDENCIES = []
AUTO_LOAD = ["sensor", "binary_sensor", "switch", "number", "output"]

CONF_OPENTHERM_ID = "opentherm_id"

opentherm = cg.esphome_ns.namespace("opentherm")
OpenThermComponent = opentherm.class_("OpenThermComponent", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OpenThermComponent),
        cv.Required(CONF_RESPONDER_READ_PIN): pins.gpio_input_pin_schema,
        cv.Required(CONF_RESPONDER_WRITE_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_CONTROLLER_READ_PIN): pins.gpio_input_pin_schema,
        cv.Required(CONF_CONTROLLER_WRITE_PIN): pins.gpio_output_pin_schema,
    }
).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    slave_read_pin = await cg.gpio_pin_expression(config[CONF_RESPONDER_READ_PIN])
    slave_write_pin = await cg.gpio_pin_expression(config[CONF_RESPONDER_WRITE_PIN])
    master_read_pin = await cg.gpio_pin_expression(config[CONF_CONTROLLER_READ_PIN])
    master_write_pin = await cg.gpio_pin_expression(config[CONF_CONTROLLER_WRITE_PIN])
    cg.add(var.set_pins(slave_read_pin, slave_write_pin, master_read_pin, master_write_pin))
