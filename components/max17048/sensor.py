"""sensor: platform max17048 — porcentaje, tensión, tasa de carga y horas restantes."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_VOLT,
)

DEPENDENCIES = ["i2c"]

CONF_CHARGE_RATE = "charge_rate"
CONF_TIME_TO_EMPTY = "time_to_empty"
CONF_TIME_TO_FULL = "time_to_full"
CONF_ALERT_THRESHOLD = "alert_threshold"

max17048_ns = cg.esphome_ns.namespace("max17048")
MAX17048 = max17048_ns.class_("MAX17048", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MAX17048),
            cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # Positiva cargando, negativa descargando.
            cv.Optional(CONF_CHARGE_RATE): sensor.sensor_schema(
                unit_of_measurement="%/h",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # Solo válidas mientras descarga / carga; si no, quedan sin valor.
            cv.Optional(CONF_TIME_TO_EMPTY): sensor.sensor_schema(
                unit_of_measurement="h",
                accuracy_decimals=1,
                device_class="duration",
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TIME_TO_FULL): sensor.sensor_schema(
                unit_of_measurement="h",
                accuracy_decimals=1,
                device_class="duration",
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            # Umbral del pin ALRT (1–32 %). El pin va a IO17; por ahora solo queda configurado.
            cv.Optional(CONF_ALERT_THRESHOLD, default=10): cv.int_range(min=1, max=32),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x36))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_alert_threshold(config[CONF_ALERT_THRESHOLD]))
    for key, setter in (
        (CONF_BATTERY_LEVEL, var.set_battery_level_sensor),
        (CONF_BATTERY_VOLTAGE, var.set_battery_voltage_sensor),
        (CONF_CHARGE_RATE, var.set_charge_rate_sensor),
        (CONF_TIME_TO_EMPTY, var.set_time_to_empty_sensor),
        (CONF_TIME_TO_FULL, var.set_time_to_full_sensor),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter(sens))
