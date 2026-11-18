import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_BQ25896_ID, BQ25896

DEPENDENCIES = ["bq25896"]

CONF_CHARGE_STATUS = "charge_status"
CONF_FAULT = "fault"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ25896_ID): cv.use_id(BQ25896),
        cv.Optional(CONF_CHARGE_STATUS): text_sensor.text_sensor_schema(
            icon="mdi:battery-charging"
        ),
        cv.Optional(CONF_FAULT): text_sensor.text_sensor_schema(icon="mdi:alert"),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BQ25896_ID])
    if CONF_CHARGE_STATUS in config:
        ts = await text_sensor.new_text_sensor(config[CONF_CHARGE_STATUS])
        cg.add(hub.set_charge_status_text_sensor(ts))
    if CONF_FAULT in config:
        ts = await text_sensor.new_text_sensor(config[CONF_FAULT])
        cg.add(hub.set_fault_text_sensor(ts))
