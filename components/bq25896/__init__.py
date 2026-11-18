"""BQ25896: cargador Li-ion con power path. Lectura de estado/ADC por I2C,
límite de corriente de entrada, ship mode y preparación para deep sleep."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import i2c
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]
MULTI_CONF = False

CONF_BQ25896_ID = "bq25896_id"
CONF_CHARGE_CURRENT = "charge_current"
CONF_CHARGE_VOLTAGE = "charge_voltage"
CONF_INPUT_CURRENT_LIMIT = "input_current_limit"
CONF_DISABLE_WATCHDOG = "disable_watchdog"
CONF_BATTERY_EMPTY_VOLTAGE = "battery_empty_voltage"
CONF_BATTERY_FULL_VOLTAGE = "battery_full_voltage"
CONF_CURRENT = "current"

bq25896_ns = cg.esphome_ns.namespace("bq25896")
BQ25896 = bq25896_ns.class_("BQ25896", cg.PollingComponent, i2c.I2CDevice)

SetInputCurrentLimitAction = bq25896_ns.class_("SetInputCurrentLimitAction", automation.Action)
ShipModeAction = bq25896_ns.class_("ShipModeAction", automation.Action)
PrepareSleepAction = bq25896_ns.class_("PrepareSleepAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BQ25896),
            # mA, 64 mA por paso. Default conservador; el ILIM del pin (1,36 A) manda igual.
            cv.Optional(CONF_CHARGE_CURRENT, default=1024): cv.int_range(min=0, max=3008),
            # mV, 16 mV por paso desde 3840.
            cv.Optional(CONF_CHARGE_VOLTAGE, default=4208): cv.int_range(min=3840, max=4608),
            # mA, 50 mA por paso desde 100. 3250 = dejar que mande el pin ILIM.
            cv.Optional(CONF_INPUT_CURRENT_LIMIT, default=3250): cv.int_range(min=100, max=3250),
            cv.Optional(CONF_DISABLE_WATCHDOG, default=True): cv.boolean,
            cv.Optional(CONF_BATTERY_EMPTY_VOLTAGE, default=3300): cv.int_range(min=2500, max=4000),
            cv.Optional(CONF_BATTERY_FULL_VOLTAGE, default=4150): cv.int_range(min=3500, max=4600),
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(i2c.i2c_device_schema(0x6B))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_charge_current(config[CONF_CHARGE_CURRENT]))
    cg.add(var.set_charge_voltage(config[CONF_CHARGE_VOLTAGE]))
    cg.add(var.set_input_current_limit_config(config[CONF_INPUT_CURRENT_LIMIT]))
    cg.add(var.set_disable_watchdog(config[CONF_DISABLE_WATCHDOG]))
    cg.add(var.set_battery_range(config[CONF_BATTERY_EMPTY_VOLTAGE], config[CONF_BATTERY_FULL_VOLTAGE]))


HUB_ACTION_SCHEMA = cv.maybe_simple_value(
    cv.Schema({cv.GenerateID(): cv.use_id(BQ25896)}), key=CONF_ID
)

SET_ILIM_SCHEMA = cv.maybe_simple_value(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(BQ25896),
            cv.Required(CONF_CURRENT): cv.templatable(cv.int_range(min=100, max=3250)),
        }
    ),
    key=CONF_CURRENT,
)


@automation.register_action(
    "bq25896.set_input_current_limit", SetInputCurrentLimitAction, SET_ILIM_SCHEMA, synchronous=True)
async def set_ilim_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_CURRENT], args, cg.uint16)
    cg.add(var.set_current(template_))
    return var


@automation.register_action("bq25896.ship_mode", ShipModeAction, HUB_ACTION_SCHEMA, synchronous=True)
async def ship_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("bq25896.prepare_sleep", PrepareSleepAction, HUB_ACTION_SCHEMA, synchronous=True)
async def prepare_sleep_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
