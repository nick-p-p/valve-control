import esphome.codegen as cg
from esphome.components import output, sensor, text_sensor, valve
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.types import ConfigType

CONF_OPEN_OUTPUT = "open_output"
CONF_CLOSE_OUTPUT = "close_output"
CONF_CURRENT_SENSOR = "current_sensor"
CONF_CURRENT_THRESHOLD = "current_threshold"
CONF_MOVEMENT_TIMEOUT = "movement_timeout"
CONF_STATE = "state"

valve_ns = cg.esphome_ns.namespace("valve_controller")
ValveController = valve_ns.class_("ValveController", valve.Valve, cg.Component)
ValveStateTextSensor = valve_ns.class_("ValveStateTextSensor", text_sensor.TextSensor)

CONFIG_SCHEMA = (
    valve.valve_schema(ValveController)
    .extend(
        {
            cv.Required(CONF_OPEN_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_CLOSE_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_CURRENT_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_STATE): text_sensor.text_sensor_schema(
                ValveStateTextSensor,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CURRENT_THRESHOLD, default=0.05): cv.positive_float,
            cv.Optional(CONF_MOVEMENT_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config: ConfigType) -> None:
    var = await valve.new_valve(config)
    await cg.register_component(var, config)

    open_output = await cg.get_variable(config[CONF_OPEN_OUTPUT])
    cg.add(var.set_open_output(open_output))

    close_output = await cg.get_variable(config[CONF_CLOSE_OUTPUT])
    cg.add(var.set_close_output(close_output))

    current_sensor = await cg.get_variable(config[CONF_CURRENT_SENSOR])
    cg.add(var.set_current_sensor(current_sensor))

    if state_config := config.get(CONF_STATE):
        state_sensor = await text_sensor.new_text_sensor(state_config)
        cg.add(var.set_state_sensor(state_sensor))

    cg.add(var.set_current_threshold_amps(config[CONF_CURRENT_THRESHOLD]))
    cg.add(var.set_movement_timeout_ms(config[CONF_MOVEMENT_TIMEOUT]))
