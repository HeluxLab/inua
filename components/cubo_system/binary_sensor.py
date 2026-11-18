import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_CONNECTIVITY

from . import CONF_CUBO_SYSTEM_ID, CuboSystem

DEPENDENCIES = ["cubo_system"]

CONF_USB_HOST = "usb_host"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CUBO_SYSTEM_ID): cv.use_id(CuboSystem),
        cv.Optional(CONF_USB_HOST): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY, icon="mdi:usb-port"
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_CUBO_SYSTEM_ID])
    if CONF_USB_HOST in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_USB_HOST])
        cg.add(hub.set_usb_host_binary_sensor(bs))
