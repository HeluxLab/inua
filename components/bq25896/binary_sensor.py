import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_BATTERY_CHARGING, DEVICE_CLASS_PLUG

from . import CONF_BQ25896_ID, BQ25896

DEPENDENCIES = ["bq25896"]

CONF_POWER_GOOD = "power_good"
CONF_CHARGING = "charging"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ25896_ID): cv.use_id(BQ25896),
        cv.Optional(CONF_POWER_GOOD): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PLUG
        ),
        cv.Optional(CONF_CHARGING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY_CHARGING
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BQ25896_ID])
    if CONF_POWER_GOOD in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_POWER_GOOD])
        cg.add(hub.set_power_good_binary_sensor(bs))
    if CONF_CHARGING in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_CHARGING])
        cg.add(hub.set_charging_binary_sensor(bs))
