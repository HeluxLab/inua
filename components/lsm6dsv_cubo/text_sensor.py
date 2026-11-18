import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_LSM6DSV_CUBO_ID, LSM6DSVCubo

DEPENDENCIES = ["lsm6dsv_cubo"]

CONF_FACE = "face"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LSM6DSV_CUBO_ID): cv.use_id(LSM6DSVCubo),
        cv.Optional(CONF_FACE): text_sensor.text_sensor_schema(icon="mdi:cube-outline"),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LSM6DSV_CUBO_ID])
    if CONF_FACE in config:
        ts = await text_sensor.new_text_sensor(config[CONF_FACE])
        cg.add(hub.set_face_text_sensor(ts))
