import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIAMP,
    UNIT_PERCENT,
    UNIT_VOLT,
)

from . import CONF_BQ25896_ID, BQ25896

DEPENDENCIES = ["bq25896"]

CONF_SYSTEM_VOLTAGE = "system_voltage"
CONF_VBUS_VOLTAGE = "vbus_voltage"
CONF_CHARGE_CURRENT = "charge_current"

VOLT_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    device_class=DEVICE_CLASS_VOLTAGE,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ25896_ID): cv.use_id(BQ25896),
        cv.Optional(CONF_BATTERY_VOLTAGE): VOLT_SCHEMA,
        cv.Optional(CONF_SYSTEM_VOLTAGE): VOLT_SCHEMA,
        cv.Optional(CONF_VBUS_VOLTAGE): VOLT_SCHEMA,
        cv.Optional(CONF_CHARGE_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIAMP,
            device_class=DEVICE_CLASS_CURRENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BQ25896_ID])
    for key, setter in (
        (CONF_BATTERY_VOLTAGE, hub.set_battery_voltage_sensor),
        (CONF_SYSTEM_VOLTAGE, hub.set_system_voltage_sensor),
        (CONF_VBUS_VOLTAGE, hub.set_vbus_voltage_sensor),
        (CONF_CHARGE_CURRENT, hub.set_charge_current_sensor),
        (CONF_BATTERY_LEVEL, hub.set_battery_level_sensor),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter(sens))
