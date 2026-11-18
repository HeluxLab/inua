"""cubo_room: en qué habitación está el cubo, según la baliza BLE que escucha más fuerte.

Cada habitación tiene una o más balizas fijas: una tecla HELUX que emite iBeacon, una etiqueta
iBeacon cualquiera o cualquier aparato BLE con dirección fija (un termómetro, por ejemplo).
El cubo escanea mientras está despierto, se queda con la más fuerte y publica la habitación.
Componente externo de ESPHome; se cuelga de `esp32_ble_tracker` como oyente normal y analiza el
anuncio crudo que viene adentro de cada dispositivo. No usa el canal de anuncios crudos: desde
ESPHome 2026.10 admite un solo oyente y lo toma el proxy Bluetooth si el usuario lo agrega."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import esp32_ble_tracker
from esphome.const import CONF_ID, CONF_MAC_ADDRESS, CONF_TRIGGER_ID

DEPENDENCIES = ["esp32_ble_tracker"]
AUTO_LOAD = ["sensor", "text_sensor"]
MULTI_CONF = False

CONF_CUBO_ROOM_ID = "cubo_room_id"
CONF_BEACONS = "beacons"
CONF_ROOM = "room"
CONF_IBEACON_UUID = "ibeacon_uuid"
CONF_IBEACON_MAJOR = "ibeacon_major"
CONF_IBEACON_MINOR = "ibeacon_minor"
CONF_OFFSET = "offset"
CONF_HYSTERESIS = "hysteresis"
CONF_MIN_RSSI = "min_rssi"
CONF_TIMEOUT = "timeout"
CONF_SETTLE_TIME = "settle_time"
CONF_UNKNOWN_ROOM = "unknown_room"
CONF_DISCOVERY = "discovery"
CONF_ON_ROOM = "on_room"
CONF_LEARN_MIN_RSSI = "learn_min_rssi"
CONF_ON_LEARNED = "on_learned"
CONF_ON_LEARN_FAILED = "on_learn_failed"

MAX_BEACONS = 24

cubo_room_ns = cg.esphome_ns.namespace("cubo_room")
CuboRoom = cubo_room_ns.class_(
    "CuboRoom", cg.Component, esp32_ble_tracker.ESPBTDeviceListener
)
RoomTrigger = cubo_room_ns.class_(
    "RoomTrigger", automation.Trigger.template(cg.std_string)
)
LearnedTrigger = cubo_room_ns.class_(
    "LearnedTrigger", automation.Trigger.template(cg.std_string, cg.std_string)
)
LearnFailedTrigger = cubo_room_ns.class_(
    "LearnFailedTrigger", automation.Trigger.template(cg.std_string)
)
LearnAction = cubo_room_ns.class_("LearnAction", automation.Action)
ForgetAction = cubo_room_ns.class_("ForgetAction", automation.Action)
ClearLearnedAction = cubo_room_ns.class_("ClearLearnedAction", automation.Action)


def _validate_beacon(config):
    if CONF_IBEACON_UUID not in config and (
        CONF_IBEACON_MAJOR in config or CONF_IBEACON_MINOR in config
    ):
        raise cv.Invalid("ibeacon_major e ibeacon_minor necesitan ibeacon_uuid")
    return config


BEACON_SCHEMA = cv.All(
    cv.Schema(
        {
            # Nombre de la habitación. Conviene que sea el nombre del área en Home Assistant.
            cv.Required(CONF_ROOM): cv.All(cv.string_strict, cv.Length(min=1, max=40)),
            cv.Optional(CONF_MAC_ADDRESS): cv.mac_address,
            cv.Optional(CONF_IBEACON_UUID): cv.uuid,
            cv.Optional(CONF_IBEACON_MAJOR): cv.uint16_t,
            cv.Optional(CONF_IBEACON_MINOR): cv.uint16_t,
            # dB que se suman a lo que se recibe de esta baliza: positivo si emite flojo o está
            # tapada, negativo si emite fuerte.
            cv.Optional(CONF_OFFSET, default=0): cv.int_range(min=-30, max=30),
        }
    ),
    cv.has_exactly_one_key(CONF_MAC_ADDRESS, CONF_IBEACON_UUID),
    _validate_beacon,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CuboRoom),
            cv.Optional(CONF_BEACONS, default=[]): cv.All(
                cv.ensure_list(BEACON_SCHEMA), cv.Length(max=MAX_BEACONS)
            ),
            # dB por los que la baliza de otra habitación tiene que ganarle a la de la actual.
            cv.Optional(CONF_HYSTERESIS, default=6.0): cv.float_range(min=0.0, max=30.0),
            # dBm: una baliza más floja que esto no alcanza para elegir habitación.
            cv.Optional(CONF_MIN_RSSI, default=-85): cv.int_range(min=-100, max=-30),
            # Tiempo de escaneo sin oír una baliza para darla por perdida.
            cv.Optional(CONF_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
            # Tiempo de escaneo al arrancar antes de decidir (junta lecturas de todas las balizas).
            cv.Optional(CONF_SETTLE_TIME, default="3s"): cv.positive_time_period_milliseconds,
            # Texto que se publica cuando no se oye ninguna baliza.
            cv.Optional(CONF_UNKNOWN_ROOM, default=""): cv.string,
            # Escribe en el log, cada 10 s, los aparatos BLE cercanos que no están en la lista.
            cv.Optional(CONF_DISCOVERY, default=False): cv.boolean,
            cv.Optional(CONF_ON_ROOM): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RoomTrigger)}
            ),
            # Balizas aprendidas con el cubo andando (`cubo_room.learn`): se guardan en la memoria
            # del cubo, hasta 12. dBm mínimos con que tiene que llegar el aparato para aceptarlo:
            # obliga a apoyar el cubo al lado de la baliza y deja afuera a los vecinos.
            cv.Optional(CONF_LEARN_MIN_RSSI, default=-65): cv.int_range(min=-90, max=-30),
            cv.Optional(CONF_ON_LEARNED): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LearnedTrigger)}
            ),
            cv.Optional(CONF_ON_LEARN_FAILED): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LearnFailedTrigger)}
            ),
        }
    )
    .extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # Oyente normal (no el canal de anuncios crudos, que en ESPHome 2026.10 es de un solo oyente).
    await esp32_ble_tracker.register_ble_device(var, config)

    cg.add(var.set_hysteresis(config[CONF_HYSTERESIS]))
    cg.add(var.set_min_rssi(config[CONF_MIN_RSSI]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_settle_time(config[CONF_SETTLE_TIME]))
    cg.add(var.set_unknown_room(config[CONF_UNKNOWN_ROOM]))
    cg.add(var.set_discovery(config[CONF_DISCOVERY]))

    for b in config[CONF_BEACONS]:
        if CONF_MAC_ADDRESS in b:
            cg.add(var.add_mac_beacon(b[CONF_ROOM], b[CONF_MAC_ADDRESS].as_hex, b[CONF_OFFSET]))
        else:
            value = b[CONF_IBEACON_UUID].int
            hi = cg.RawExpression("0x%016XULL" % (value >> 64))
            lo = cg.RawExpression("0x%016XULL" % (value & 0xFFFFFFFFFFFFFFFF))
            cg.add(
                var.add_ibeacon(
                    b[CONF_ROOM],
                    hi,
                    lo,
                    b.get(CONF_IBEACON_MAJOR, -1),
                    b.get(CONF_IBEACON_MINOR, -1),
                    b[CONF_OFFSET],
                )
            )

    for conf in config.get(CONF_ON_ROOM, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "room")], conf)

    cg.add(var.set_learn_min_rssi(config[CONF_LEARN_MIN_RSSI]))
    for conf in config.get(CONF_ON_LEARNED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger, [(cg.std_string, "room"), (cg.std_string, "detail")], conf
        )
    for conf in config.get(CONF_ON_LEARN_FAILED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "reason")], conf)


# cubo_room.learn: aprende la baliza que está al lado del cubo y la asigna a `room`.
# cubo_room.forget: olvida las balizas aprendidas de `room`.
ROOM_ACTION_SCHEMA = cv.maybe_simple_value(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(CuboRoom),
            cv.Required(CONF_ROOM): cv.templatable(cv.string),
        }
    ),
    key=CONF_ROOM,
)


@automation.register_action("cubo_room.learn", LearnAction, ROOM_ACTION_SCHEMA, synchronous=True)
async def learn_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_ROOM], args, cg.std_string)
    cg.add(var.set_room(template_))
    return var


@automation.register_action("cubo_room.forget", ForgetAction, ROOM_ACTION_SCHEMA, synchronous=True)
async def forget_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_ROOM], args, cg.std_string)
    cg.add(var.set_room(template_))
    return var


# cubo_room.clear: olvida todas las balizas aprendidas (las del YAML quedan).
CLEAR_ACTION_SCHEMA = cv.maybe_simple_value(
    cv.Schema({cv.GenerateID(): cv.use_id(CuboRoom)}), key=CONF_ID
)


@automation.register_action("cubo_room.clear", ClearLearnedAction, CLEAR_ACTION_SCHEMA, synchronous=True)
async def clear_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
