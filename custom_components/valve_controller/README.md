# Valve Controller External Component

This ESPHome external component controls a water valve with two independent GPIO outputs and an INA219 current sensor.

It exposes:

- a state text sensor with `open`, `closed`, `opening`, `closing`, `error`
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