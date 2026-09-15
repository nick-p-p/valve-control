# Valve Controller External Component

This ESPHome external component controls a "3 wire" water valve with two independent GPIO outputs and an INA219 current sensor to detect when the motor is moving. This is compatible with the 3 wire valves from US solid that have individual wires for open adn close that only draw current when the valve is moving. This is essential as it is the drop in current that allows this addon to determine the motor has reached the desired position.

These valves draw far more current than an esp32 can handle so mosfets or relays are needed. The INA219 should be wired so that current for the motor flows through it; only for the motor. 

It exposes:

- a state text sensor with `open`, `closed`, `opening`, `closing`, `error`
- optional diagnostic text sensors:
  - `status_text` with the current controller status text
  - `health_text` with `Error` when valve state is `unknown` or `error`, otherwise `Ok`
- an `Open` button
- a `Close` button

The controller guarantees the open and close outputs are never driven high at the same time.

## Example

```yaml
external_components:
  - source:
      type: local
      path: ./custom_components

i2c:

sensor:
  - platform: ina219
    id: valve_current
    current:
      name: Valve Current
    update_interval: 20ms

output:
  - platform: gpio
    id: valve_open_output
    pin: GPIO12

  - platform: gpio
    id: valve_close_output
    pin: GPIO13

valve_controller:
  id: garden_valve
  open_output: valve_open_output
  close_output: valve_close_output
  current_sensor: valve_current
  movement_timeout: 20s
  current_threshold: 0.05
  status_text:
    name: Garden Valve Status
  health_text:
    name: Garden Valve Health
  state:
    name: Garden Valve State
  open_button:
    name: Open Garden Valve
  close_button:
    name: Close Garden Valve
```

## Notes

- Keep the INA219 update interval reasonably fast, or the 50 ms checks will be based on stale samples.
- The startup sequence follows the open-test / close-test behavior described in the request and reports `error` when the two tests are inconsistent or both fail.