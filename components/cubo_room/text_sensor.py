import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_BEACONS, CONF_CUBO_ROOM_ID, CONF_ROOM, CuboRoom

DEPENDENCIES = ["cubo_room"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CUBO_ROOM_ID): cv.use_id(CuboRoom),
        cv.Optional(CONF_ROOM): text_sensor.text_sensor_schema(icon="mdi:floor-plan"),
        # Cuántas balizas hay en cada habitación, y el resultado del último aprendizaje.
        cv.Optional(CONF_BEACONS): text_sensor.text_sensor_schema(icon="mdi:access-point"),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_CUBO_ROOM_ID])
    if CONF_ROOM in config:
        ts = await text_sensor.new_text_sensor(config[CONF_ROOM])
        cg.add(hub.set_room_text_sensor(ts))
    if CONF_BEACONS in config:
        ts = await text_sensor.new_text_sensor(config[CONF_BEACONS])
        cg.add(hub.set_beacons_text_sensor(ts))
