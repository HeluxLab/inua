import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREES,
)

from . import CONF_LSM6DSV_CUBO_ID, LSM6DSVCubo

DEPENDENCIES = ["lsm6dsv_cubo"]

CONF_KNOB_ANGLE = "knob_angle"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LSM6DSV_CUBO_ID): cv.use_id(LSM6DSVCubo),
        cv.Optional(CONF_KNOB_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon="mdi:rotate-right",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            device_class=DEVICE_CLASS_TEMPERATURE,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LSM6DSV_CUBO_ID])
    if CONF_KNOB_ANGLE in config:
        sens = await sensor.new_sensor(config[CONF_KNOB_ANGLE])
        cg.add(hub.set_knob_angle_sensor(sens))
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(hub.set_temperature_sensor(sens))
