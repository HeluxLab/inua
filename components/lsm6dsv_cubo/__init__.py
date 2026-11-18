"""LSM6DSV16X "Cubo" hub: gestos (perilla, deslizamientos, toque, cara arriba) y armado del
despertador wake-up para deep sleep. Componente externo de ESPHome.

Reemplaza al hub del BNO055 con el mismo contrato (opciones, triggers, acciones y
plataformas). La gravedad sale del bloque de fusión SFLP del propio chip (por FIFO),
la aceleración lineal es acelerómetro menos gravedad y el giróscopo se lee directo."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_TRIGGER_ID

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor", "event"]
MULTI_CONF = False

CONF_LSM6DSV_CUBO_ID = "lsm6dsv_cubo_id"
CONF_STEP_DEGREES = "step_degrees"
CONF_ROTATION_DEADBAND = "rotation_deadband"
CONF_SLIDE_THRESHOLD = "slide_threshold"
CONF_IDLE_TIMEOUT = "idle_timeout"
CONF_IDLE_TIMEOUT_TIMER_WAKE = "idle_timeout_timer_wake"
CONF_WAKE_THRESHOLD = "wake_threshold"
CONF_WAKE_DURATION = "wake_duration"
CONF_TAP_THRESHOLD = "tap_threshold"
CONF_FACE_NAMES = "face_names"
CONF_ON_ROTATE = "on_rotate"
CONF_ON_SLIDE = "on_slide"
CONF_ON_TAP = "on_tap"
CONF_ON_FACE = "on_face"
CONF_ON_IDLE = "on_idle"
CONF_ON_MOTION = "on_motion"
CONF_TIMEOUT = "timeout"

lsm6dsv_cubo_ns = cg.esphome_ns.namespace("lsm6dsv_cubo")
LSM6DSVCubo = lsm6dsv_cubo_ns.class_("LSM6DSVCubo", cg.PollingComponent, i2c.I2CDevice)

RotateTrigger = lsm6dsv_cubo_ns.class_(
    "RotateTrigger", automation.Trigger.template(cg.int_)
)
SlideTrigger = lsm6dsv_cubo_ns.class_(
    "SlideTrigger", automation.Trigger.template(cg.std_string)
)
TapTrigger = lsm6dsv_cubo_ns.class_("TapTrigger", automation.Trigger.template())
FaceTrigger = lsm6dsv_cubo_ns.class_(
    "FaceTrigger", automation.Trigger.template(cg.int_, cg.std_string)
)
IdleTrigger = lsm6dsv_cubo_ns.class_("IdleTrigger", automation.Trigger.template())
MotionTrigger = lsm6dsv_cubo_ns.class_("MotionTrigger", automation.Trigger.template())

ArmWakeAction = lsm6dsv_cubo_ns.class_("ArmWakeAction", automation.Action)
ClearInterruptAction = lsm6dsv_cubo_ns.class_("ClearInterruptAction", automation.Action)
SetIdleTimeoutAction = lsm6dsv_cubo_ns.class_("SetIdleTimeoutAction", automation.Action)
InhibitTapAction = lsm6dsv_cubo_ns.class_("InhibitTapAction", automation.Action)

# Orden fijo de caras: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z (ejes del propio LSM6DSV16X).
DEFAULT_FACE_NAMES = ["x+", "x-", "y+", "y-", "z+", "z-"]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LSM6DSVCubo),
            cv.Optional(CONF_STEP_DEGREES, default=15.0): cv.float_range(min=2.0, max=90.0),
            cv.Optional(CONF_ROTATION_DEADBAND, default=6.0): cv.float_range(min=0.5, max=60.0),
            cv.Optional(CONF_SLIDE_THRESHOLD, default=3.0): cv.float_range(min=0.5, max=20.0),
            cv.Optional(CONF_IDLE_TIMEOUT, default="20s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_IDLE_TIMEOUT_TIMER_WAKE, default="3s"): cv.positive_time_period_milliseconds,
            # mg de pendiente (diferencia entre muestras consecutivas a 15 Hz) que despierta.
            cv.Optional(CONF_WAKE_THRESHOLD, default=80): cv.int_range(min=8, max=1000),
            # muestras consecutivas por encima del umbral (1 LSB = 1/ODR = 67 ms).
            cv.Optional(CONF_WAKE_DURATION, default=2): cv.int_range(min=1, max=4),
            # mg de pendiente (media diferencia entre muestras consecutivas a 480 Hz) del golpe que
            # cuenta como toque, en pasos de 125 mg. 0 = sin toque: el chip queda a 60 Hz como antes.
            cv.Optional(CONF_TAP_THRESHOLD, default=500): cv.int_range(min=0, max=3875),
            cv.Optional(CONF_FACE_NAMES, default=DEFAULT_FACE_NAMES): cv.All(
                cv.ensure_list(cv.string), cv.Length(min=6, max=6)
            ),
            cv.Optional(CONF_ON_ROTATE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RotateTrigger)}
            ),
            cv.Optional(CONF_ON_SLIDE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SlideTrigger)}
            ),
            cv.Optional(CONF_ON_TAP): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(TapTrigger)}
            ),
            cv.Optional(CONF_ON_FACE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(FaceTrigger)}
            ),
            cv.Optional(CONF_ON_IDLE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(IdleTrigger)}
            ),
            cv.Optional(CONF_ON_MOTION): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(MotionTrigger)}
            ),
        }
    )
    .extend(cv.polling_component_schema("20ms"))
    .extend(i2c.i2c_device_schema(0x6A))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_step_degrees(config[CONF_STEP_DEGREES]))
    cg.add(var.set_rotation_deadband(config[CONF_ROTATION_DEADBAND]))
    cg.add(var.set_slide_threshold(config[CONF_SLIDE_THRESHOLD]))
    cg.add(var.set_idle_timeout(config[CONF_IDLE_TIMEOUT]))
    cg.add(var.set_idle_timeout_timer_wake(config[CONF_IDLE_TIMEOUT_TIMER_WAKE]))
    cg.add(var.set_wake_threshold(config[CONF_WAKE_THRESHOLD]))
    cg.add(var.set_wake_duration(config[CONF_WAKE_DURATION]))
    cg.add(var.set_tap_threshold(config[CONF_TAP_THRESHOLD]))
    for i, name in enumerate(config[CONF_FACE_NAMES]):
        cg.add(var.set_face_name(i, name))

    for conf in config.get(CONF_ON_ROTATE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.int_, "steps")], conf)
    for conf in config.get(CONF_ON_SLIDE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "direction")], conf)
    for conf in config.get(CONF_ON_TAP, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_FACE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger, [(cg.int_, "face"), (cg.std_string, "name")], conf
        )
    for conf in config.get(CONF_ON_IDLE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_MOTION, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


HUB_ACTION_SCHEMA = cv.maybe_simple_value(
    cv.Schema({cv.GenerateID(): cv.use_id(LSM6DSVCubo)}), key=CONF_ID
)


@automation.register_action("lsm6dsv_cubo.arm_wake", ArmWakeAction, HUB_ACTION_SCHEMA, synchronous=True)
async def arm_wake_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "lsm6dsv_cubo.clear_interrupt", ClearInterruptAction, HUB_ACTION_SCHEMA, synchronous=True)
async def clear_interrupt_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


SET_IDLE_TIMEOUT_SCHEMA = cv.maybe_simple_value(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(LSM6DSVCubo),
            cv.Required(CONF_TIMEOUT): cv.templatable(cv.positive_time_period_milliseconds),
        }
    ),
    key=CONF_TIMEOUT,
)


@automation.register_action(
    "lsm6dsv_cubo.set_idle_timeout", SetIdleTimeoutAction, SET_IDLE_TIMEOUT_SCHEMA, synchronous=True)
async def set_idle_timeout_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_TIMEOUT], args, cg.uint32)
    cg.add(var.set_timeout(template_))
    return var


# lsm6dsv_cubo.inhibit_tap: ignora los toques durante el tiempo indicado. Va en `on_drive` del
# drv2605: el golpe del motor háptico es, para el acelerómetro, igual a un toque.
@automation.register_action(
    "lsm6dsv_cubo.inhibit_tap", InhibitTapAction, SET_IDLE_TIMEOUT_SCHEMA, synchronous=True)
async def inhibit_tap_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_TIMEOUT], args, cg.uint32)
    cg.add(var.set_timeout(template_))
    return var
