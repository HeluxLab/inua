import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_CUBO_SYSTEM_ID, CuboSystem

DEPENDENCIES = ["cubo_system"]

CONF_WAKE_CAUSE = "wake_cause"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CUBO_SYSTEM_ID): cv.use_id(CuboSystem),
        cv.Optional(CONF_WAKE_CAUSE): text_sensor.text_sensor_schema(icon="mdi:power-sleep"),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_CUBO_SYSTEM_ID])
    if CONF_WAKE_CAUSE in config:
        ts = await text_sensor.new_text_sensor(config[CONF_WAKE_CAUSE])
        cg.add(hub.set_wake_cause_text_sensor(ts))
