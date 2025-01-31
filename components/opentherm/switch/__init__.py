import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    ICON_RADIATOR,
)
from ...opentherm import (
    OpenThermComponent,
    CONF_OPENTHERM_ID,
)
from .. import opentherm

Switch = opentherm.class_("Switch", switch.Switch, cg.Component)

CONF_GATEWAY_ENABLED = "gateway_enabled"
CONF_CH_ENABLED = "ch_enabled"
CONF_DHW_ENABLED = "dhw_enabled"
CONF_COOLING_ENABLED = "cooling_enabled"

ICON_THERMOSTAT = "mdi:thermostat-auto"
ICON_WATER_BOILER = "mdi:water-boiler"
ICON_SNOWFLAKE = "mdi:snowflake"

TYPES = [
    CONF_GATEWAY_ENABLED,
    CONF_CH_ENABLED,
    CONF_DHW_ENABLED,
    CONF_COOLING_ENABLED,
]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_OPENTHERM_ID): cv.use_id(OpenThermComponent),
            cv.Optional(CONF_GATEWAY_ENABLED): switch.switch_schema(
                class_=Switch,
                icon=ICON_THERMOSTAT,
            ),
            cv.Required(CONF_CH_ENABLED): switch.switch_schema(
                class_=Switch,
                icon=ICON_RADIATOR,
            ),
            cv.Optional(CONF_DHW_ENABLED): switch.switch_schema(
                class_=Switch,
                icon=ICON_WATER_BOILER,
            ),
            cv.Optional(CONF_COOLING_ENABLED): switch.switch_schema(
                class_=Switch,
                icon=ICON_SNOWFLAKE,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        var = await switch.new_switch(conf)
        cg.add(getattr(hub, f"set_{key}_switch")(var))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_OPENTHERM_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
