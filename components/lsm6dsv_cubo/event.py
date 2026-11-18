import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import event

from . import CONF_LSM6DSV_CUBO_ID, LSM6DSVCubo, lsm6dsv_cubo_ns

DEPENDENCIES = ["lsm6dsv_cubo"]

GestureEvent = lsm6dsv_cubo_ns.class_("GestureEvent", event.Event)

# Tipos de evento que el hub puede disparar. Los deslizamientos se nombran en el
# sistema de ejes del propio cubo (ver README); la asignación L/R/F/B se hace en HA.
GESTURE_TYPES = [
    "rotate_cw",
    "rotate_ccw",
    "slide_x_plus",
    "slide_x_minus",
    "slide_y_plus",
    "slide_y_minus",
    "slide_z_plus",
    "slide_z_minus",
    "tap",
    "face_changed",
    "shake",
]

CONFIG_SCHEMA = event.event_schema(GestureEvent, icon="mdi:gesture").extend(
    {cv.GenerateID(CONF_LSM6DSV_CUBO_ID): cv.use_id(LSM6DSVCubo)}
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LSM6DSV_CUBO_ID])
    var = await event.new_event(config, event_types=GESTURE_TYPES)
    cg.add(hub.set_gesture_event(var))
