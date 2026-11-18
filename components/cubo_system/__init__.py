"""Ayudas de sistema del cubo: detección de host USB (para no dormir mientras
está enchufado a la PC), causa del despertar, preparación de pines antes de
deep sleep (bajar y retener SD_MODE del amplificador, CLK de los micrófonos y
TRIG del háptico) y la tabla de particiones de Inua."""

from pathlib import Path
import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import esp32
from esphome.components.esp32.const import KEY_ESP32, KEY_FLASH_SIZE
from esphome.const import CONF_ID
from esphome.core import CORE

_LOGGER = logging.getLogger(__name__)

AUTO_LOAD = ["binary_sensor", "text_sensor"]
MULTI_CONF = False

CONF_CUBO_SYSTEM_ID = "cubo_system_id"
CONF_HOLD_LOW_PINS = "hold_low_pins"
CONF_LOW_BATTERY_PIN = "low_battery_pin"
CONF_PARTITION_TABLE = "partition_table"

# Tabla de particiones de Inua para 4 MB: 2 x 1984 kB de programa y 64 kB de datos. Viaja con el
# componente y no como archivo suelto al lado del YAML: cuando alguien adopta el cubo, ESPHome baja
# la configuración y los componentes desde GitHub, y `esp32: partitions:` buscaría el archivo en la
# carpeta del usuario, donde no está.
PARTITION_TABLE_4MB = Path(__file__).parent / "partitions-4mb.csv"

cubo_system_ns = cg.esphome_ns.namespace("cubo_system")
CuboSystem = cubo_system_ns.class_("CuboSystem", cg.PollingComponent)

PrepareSleepAction = cubo_system_ns.class_("PrepareSleepAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CuboSystem),
        # Números de GPIO sin registrar como "uso de pin": estos pines ya los usan
        # otros componentes (I2S, output); acá solo se tocan al ir a dormir.
        cv.Optional(CONF_HOLD_LOW_PINS, default=[]): cv.ensure_list(
            cv.int_range(min=0, max=48)
        ),
        # GPIO RTC (0-21) con la alerta de batería baja (open-drain, activo bajo). Al dormir se le
        # activa el pull-up interno y, si despierta por él, la causa es "batería baja".
        cv.Optional(CONF_LOW_BATTERY_PIN): cv.int_range(min=0, max=21),
        # true: usa la tabla de particiones de Inua (necesita flash de 4 MB y que `esp32:` no tenga
        # `partitions:`). false: queda la tabla por defecto de ESPHome.
        cv.Optional(CONF_PARTITION_TABLE, default=True): cv.boolean,
    }
).extend(cv.polling_component_schema("1s"))


def _final_validate(config):
    if not config[CONF_PARTITION_TABLE]:
        return config
    flash = CORE.data.get(KEY_ESP32, {}).get(KEY_FLASH_SIZE)
    if flash is not None and flash != "4MB":
        raise cv.Invalid(
            f"partition_table: la tabla de Inua es para 4 MB de flash y esta placa tiene {flash}. "
            "Poner partition_table: false."
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for p in config[CONF_HOLD_LOW_PINS]:
        cg.add(var.add_hold_low_pin(p))
    if CONF_LOW_BATTERY_PIN in config:
        cg.add(var.set_low_battery_pin(config[CONF_LOW_BATTERY_PIN]))
    if config[CONF_PARTITION_TABLE]:
        if not esp32.add_extra_build_file("partitions.csv", PARTITION_TABLE_4MB):
            _LOGGER.warning(
                "cubo_system: `esp32: partitions:` ya define una tabla; la de Inua no se usa. "
                "Para usarla, sacar `partitions:` de `esp32:`."
            )


@automation.register_action(
    "cubo_system.prepare_sleep",
    PrepareSleepAction,
    cv.maybe_simple_value(cv.Schema({cv.GenerateID(): cv.use_id(CuboSystem)}), key=CONF_ID),
    synchronous=True,
)
async def prepare_sleep_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
