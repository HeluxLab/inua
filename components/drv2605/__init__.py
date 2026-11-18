"""DRV2605L: driver háptico (ERM / LRA / reactor Alps) con biblioteca de efectos, por I2C."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_FREQUENCY, CONF_TRIGGER_ID

DEPENDENCIES = ["i2c"]
MULTI_CONF = False

CONF_MOTOR_TYPE = "motor_type"
CONF_LIBRARY = "library"
CONF_EFFECT = "effect"
CONF_LRA_FREQUENCY = "lra_frequency"
CONF_RATED_VOLTAGE = "rated_voltage"
CONF_MAX_VOLTAGE = "max_voltage"
CONF_PULSES = "pulses"
CONF_AMPLITUDE = "amplitude"
CONF_ON_DRIVE = "on_drive"
CONF_INHIBIT = "inhibit"

drv2605_ns = cg.esphome_ns.namespace("drv2605")
DRV2605 = drv2605_ns.class_("DRV2605", cg.Component, i2c.I2CDevice)

DriveTrigger = drv2605_ns.class_("DriveTrigger", automation.Trigger.template())

PlayAction = drv2605_ns.class_("PlayAction", automation.Action)
PulseAction = drv2605_ns.class_("PulseAction", automation.Action)
StopAction = drv2605_ns.class_("StopAction", automation.Action)
StandbyAction = drv2605_ns.class_("StandbyAction", automation.Action)

# erm: motor de masa excéntrica (lazo cerrado por back-EMF).
# lra: resonador lineal clásico, lazo abierto a lra_frequency.
# reactor: Alps Haptic Reactor (AFT14*): bobina + imán con pulsos unipolares de 3 V;
#          se maneja como ERM en lazo abierto y se excita con `drv2605.pulse`.
MOTOR_TYPES = {"erm": 0, "lra": 1, "reactor": 2}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DRV2605),
        cv.Optional(CONF_MOTOR_TYPE, default="erm"): cv.enum(MOTOR_TYPES, lower=True),
        # 1–5 bibliotecas ERM (A–E), 6 = LRA. Si no se indica, se elige según el motor.
        cv.Optional(CONF_LIBRARY): cv.int_range(min=1, max=6),
        # Frecuencia de conmutación en lazo abierto (LRA) y por defecto de `pulse`.
        cv.Optional(CONF_LRA_FREQUENCY): cv.All(cv.frequency, cv.Range(min=50, max=400)),
        # Tensión nominal (RATED_VOLTAGE) y tope absoluto (OD_CLAMP). Si faltan, quedan los
        # valores de fábrica del chip (1,3 V / 3,0 V).
        cv.Optional(CONF_RATED_VOLTAGE): cv.All(cv.voltage, cv.Range(min=0.3, max=5.0)),
        cv.Optional(CONF_MAX_VOLTAGE): cv.All(cv.voltage, cv.Range(min=0.3, max=5.0)),
        # Se dispara justo antes de mover el motor, con `play` o con `pulse`.
        cv.Optional(CONF_ON_DRIVE): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(DriveTrigger)}
        ),
        # Bloqueo: lambda que devuelve true mientras el motor NO debe moverse. `play` y `pulse`
        # se omiten (no se encolan) mientras dure.
        cv.Optional(CONF_INHIBIT): cv.returning_lambda,
    }
).extend(i2c.i2c_device_schema(0x5A))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    motor = config[CONF_MOTOR_TYPE]
    cg.add(var.set_motor_type(motor))
    library = config.get(CONF_LIBRARY, 6 if motor == 1 else 1)
    cg.add(var.set_library(library))
    if CONF_LRA_FREQUENCY in config:
        cg.add(var.set_lra_frequency(config[CONF_LRA_FREQUENCY]))
    if CONF_RATED_VOLTAGE in config:
        cg.add(var.set_rated_voltage(config[CONF_RATED_VOLTAGE]))
    if CONF_MAX_VOLTAGE in config:
        cg.add(var.set_max_voltage(config[CONF_MAX_VOLTAGE]))
    for conf in config.get(CONF_ON_DRIVE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    if CONF_INHIBIT in config:
        inhibit = await cg.process_lambda(config[CONF_INHIBIT], [], return_type=cg.bool_)
        cg.add(var.set_inhibit(inhibit))


PLAY_SCHEMA = cv.maybe_simple_value(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(DRV2605),
            cv.Required(CONF_EFFECT): cv.templatable(cv.int_range(min=1, max=123)),
        }
    ),
    key=CONF_EFFECT,
)


@automation.register_action("drv2605.play", PlayAction, PLAY_SCHEMA, synchronous=True)
async def play_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_EFFECT], args, cg.uint8)
    cg.add(var.set_effect(template_))
    return var


# drv2605.pulse: N pulsos cuadrados a `frequency` (por defecto lra_frequency) con `amplitude` %.
# En erm/reactor son pulsos unipolares (la forma de onda que especifica Alps: 3 × 3,125 ms a
# 160 Hz); en lra es una ráfaga de N ciclos. Respeta 50 ms mínimos entre disparos.
PULSE_SCHEMA = cv.maybe_simple_value(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(DRV2605),
            cv.Optional(CONF_PULSES, default=3): cv.templatable(cv.int_range(min=1, max=20)),
            cv.Optional(CONF_FREQUENCY): cv.templatable(cv.All(cv.frequency, cv.Range(min=50, max=400))),
            cv.Optional(CONF_AMPLITUDE, default=100): cv.templatable(cv.int_range(min=0, max=100)),
        }
    ),
    key=CONF_PULSES,
)


@automation.register_action("drv2605.pulse", PulseAction, PULSE_SCHEMA, synchronous=True)
async def pulse_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_PULSES], args, cg.uint8)
    cg.add(var.set_pulses(template_))
    if CONF_FREQUENCY in config:
        template_ = await cg.templatable(config[CONF_FREQUENCY], args, cg.float_)
        cg.add(var.set_frequency(template_))
    template_ = await cg.templatable(config[CONF_AMPLITUDE], args, cg.uint8)
    cg.add(var.set_amplitude(template_))
    return var


HUB_ACTION_SCHEMA = cv.maybe_simple_value(
    cv.Schema({cv.GenerateID(): cv.use_id(DRV2605)}), key=CONF_ID
)


@automation.register_action("drv2605.stop", StopAction, HUB_ACTION_SCHEMA, synchronous=True)
async def stop_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("drv2605.standby", StandbyAction, HUB_ACTION_SCHEMA, synchronous=True)
async def standby_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
