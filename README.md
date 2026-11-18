# Inua — ESPHome firmware

Inua is a gesture cube for Home Assistant, made by [HELUX LABS](https://heluxlabs.ar). The face
that is up picks what the cube controls. Turn it like a knob, slide it, tap it or shake it, and
feel a haptic click for every step. On its charging dock it also works as a voice assistant.

The firmware is [ESPHome](https://esphome.io) on ESP-IDF and talks to Home Assistant over the
native API, on your local network. No cloud and no account.

> **Status: prototype.** The firmware compiles with ESPHome 2026.6.5. It has **not been tested on
> the hardware yet**: the first boards are being made. Expect changes.

## Hardware

| Part | Chip |
|---|---|
| Microcontroller | ESP32-S3-MINI-1U-N4R2 (4 MB flash, 2 MB PSRAM, external antenna) |
| Motion | ST LSM6DSV16X, 6-axis IMU with on-chip sensor fusion |
| Haptics | TI DRV2605L driving an Alps Alpine AFT14A903A haptic reactor |
| Charger | TI BQ25896 |
| Fuel gauge | Analog Devices MAX17048 |
| Speaker amplifier | Analog Devices MAX98357A |
| Microphone | Infineon IM66D120A, PDM |

The six components in `components/` are written for the cube: gestures, tap detection and
wake-up on the IMU, haptic pulses for the Alps reactor, charger and fuel gauge, room detection,
and sleep handling.

## What the firmware does

- **Gestures.** The face that is up selects the function. Turning the cube on the table works
  as a knob, in steps of 15°. A short push is a slide, a light tap on the top face is a tap, and a
  strong shake is a shake. Every gesture is sent to Home Assistant as an event.
- **Haptics.** One pulse per knob step, two for a slide, a full click when the face changes.
- **Gestures are not lost while connecting.** If the cube has just woken up and Home Assistant is
  not connected yet, gestures are queued (up to 12, for 8 s) and sent as soon as it connects.
- **Voice assistant.** Put the assistant face up, or say the wake word while the cube is on its
  dock. Answers play on the speaker. Needs an Assist pipeline in Home Assistant.
- **Media player.** Home Assistant can send speech, announcements or audio to the cube.
- **Room awareness.** The cube listens to fixed Bluetooth beacons around the house and reports
  which room it is in, so the same gesture can control the lights of that room. Beacons are
  learned from Home Assistant, without recompiling.
- **Battery.** Real state of charge from the fuel gauge. Off the dock the cube goes to deep sleep
  after 20 s without movement and wakes up when touched. Estimated sleep current: about 80 µA,
  not measured yet. At 5 % it cuts the battery off to protect the cell, until it is docked again.
- **No vibration while audio plays.** The motor is blocked while the speaker is playing.

## Files

| File | What it is |
|---|---|
| `inua.yaml` | The device configuration. No credentials. This is what you adopt or include. |
| `inua.factory.yaml` | What the units ship with: `inua.yaml` plus Wi-Fi setup over Bluetooth (Improv), project info, adoption from the ESPHome dashboard and firmware updates. |
| `autoprueba.yaml` | Self-test firmware for the first power-up of a board and for production. Needs no Wi-Fi or Home Assistant. Its texts are in Spanish. |
| `components/` | The six custom ESPHome components. |
| `tools/sim_room.py` | Python copy of the room decision rule, to test it with synthetic signals. |

## Using it

### Adopting a unit

A unit that runs the factory firmware shows up in the ESPHome dashboard as a device to adopt.
Set up its Wi-Fi first, over Bluetooth with Improv or through the access point the cube opens
when it has no network. Adopting creates a small configuration in your ESPHome folder that pulls
`inua.yaml` from this repository.

### Your own configuration

```yaml
substitutions:
  name: inua
  friendly_name: Inua

packages:
  inua: github://HeluxLab/inua/inua.yaml@main

api:
  encryption:
    key: !secret api_key

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
```

### Partition table

Inua uses its own partition table: 2 × 1984 kB for the firmware and 64 kB for data, instead of
ESPHome's default 2 × 1792 kB and 448 kB. The firmware needs that space. The table is provided by
the `cubo_system` component, so it also applies when the configuration comes from this repository.
Do not set `partitions:` under `esp32:`.

A partition table can only be changed over USB, not over the air. Units ship with it already.

## In Home Assistant

### Gestures

All gestures arrive on the `Gesture` event entity. Its `event_type` is one of:

`face_changed`, `rotate_cw`, `rotate_ccw`, `slide_x_plus`, `slide_x_minus`, `slide_y_plus`,
`slide_y_minus`, `slide_z_plus`, `slide_z_minus`, `tap`, `shake`

Slides are reported on the cube's own axes. Which one is left or right depends on how the cube is
lying and where you are, so map them per face in your automations.

Example: volume with the knob while the Music face is up.

```yaml
automation:
  - alias: Inua - music volume
    trigger:
      - platform: state
        entity_id: event.inua_gesture
    condition:
      - condition: state
        entity_id: sensor.inua_face_up
        state: "Music"
      - condition: template
        value_template: "{{ trigger.to_state.attributes.event_type in ['rotate_cw', 'rotate_ccw'] }}"
    action:
      - service: >
          {{ 'media_player.volume_up' if trigger.to_state.attributes.event_type == 'rotate_cw'
             else 'media_player.volume_down' }}
        target:
          entity_id: media_player.living_room
```

Entity IDs depend on the device name. With `name: inua` they look like `event.inua_gesture`.

### Faces

The faces are numbered Face 1 to Face 6, in the fixed axis order of the sensor: +X, −X, +Y, −Y,
+Z, −Z. Their names are text entities, **Face 1** to **Face 6**, and are stored on the cube.
Default names: Music, Blinds, Climate, Lights, Assistant, Rest. To find out which physical face is
which, put it up and look at **Face Up**.

**Assistant Face** picks the face that starts the voice assistant, or None.

### Rooms

1. Put a Bluetooth beacon in each room. Any iBeacon tag works, or any Bluetooth device with a
   fixed address that advertises often. Phones do not work: they change their address.
2. With the cube awake, place it next to the beacon.
3. Type the room name in **Room to Learn**. Use the name of the area in Home Assistant.
4. Press **Learn Beacon**. The cube listens for 10 s, keeps the strongest device and stores it.
   Two pulses mean it was learned. **Beacons** shows the result.

Up to 12 beacons are stored on the cube. **Forget Room Beacons** and **Forget All Beacons** remove
them. The current room is in **Room**, so an automation can target that area.

### Entities

| Type | Entities |
|---|---|
| Sensors | Battery, Battery Voltage, Battery Charge Rate, Battery Time to Empty, Battery Time to Full, Charge Current, Input Voltage, Knob Angle, IMU Temperature, Beacon Signal |
| Binary sensors | Docked, Charging, USB Connected, Motion, Battery Alert, Update Mode (HA) |
| Text sensors | Face Up, Room, Beacons, Charge Status, Charger Fault, Wake Cause |
| Event | Gesture |
| Controls | Stay Awake, Volume, Media Player |
| Buttons | Listen, Sleep Now, Test Haptics, Ship Mode, Restart, Learn Beacon, Forget Room Beacons, Forget All Beacons |
| Settings stored on the cube | Face 1 to Face 6, Assistant Face, Room to Learn |

**Update Mode (HA)** follows a toggle helper you create in Home Assistant,
`input_boolean.inua_ota_mode`. While it is on, the cube does not go to sleep, so it can be
updated over the air. One helper serves all your cubes; to use another one, set the
`update_mode_helper` substitution.

## Notes

- Code comments and log messages are in Spanish for now. Entity names and values are in English.
- Hardware design files are published separately on OSHWLab under CC BY-NC-SA 4.0.

## License

The firmware in this repository is licensed under the GNU General Public License v3.0. See
[LICENSE](LICENSE).
