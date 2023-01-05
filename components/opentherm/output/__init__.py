import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import output
from esphome.const import CONF_ID, CONF_TYPE
from ...opentherm import (
    OpenThermComponent,
    CONF_OPENTHERM_ID,
)
from .. import opentherm

OpenThermFloatOutput = opentherm.class_("OpenThermFloatOutput", output.FloatOutput, cg.Component)

CONF_OPENTHERM_OUTPUT = "opentherm"

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
       cv.GenerateID(CONF_OPENTHERM_ID): cv.use_id(OpenThermComponent),
       cv.Required(CONF_ID): cv.declare_id(OpenThermFloatOutput),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_OPENTHERM_ID])
    key = CONF_OPENTHERM_OUTPUT
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await output.register_output(var, config)
    cg.add(getattr(hub, f"set_{key}_output")(var))