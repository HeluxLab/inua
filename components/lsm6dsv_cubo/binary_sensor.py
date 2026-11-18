import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_MOTION

from . import CONF_LSM6DSV_CUBO_ID, LSM6DSVCubo

DEPENDENCIES = ["lsm6dsv_cubo"]

CONF_MOTION = "motion"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LSM6DSV_CUBO_ID): cv.use_id(LSM6DSVCubo),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOTION
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LSM6DSV_CUBO_ID])
    if CONF_MOTION in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_MOTION])
        cg.add(hub.set_motion_binary_sensor(bs))
