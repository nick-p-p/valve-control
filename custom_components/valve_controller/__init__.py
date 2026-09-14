import esphome.codegen as cg
from esphome.components import output, sensor, valve
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
)
from esphome.types import ConfigType

AUTO_LOAD = ["valve"]
DEPENDENCIES = ["valve"]

CONF_OPEN_OUTPUT = "open_output"
CONF_CLOSE_OUTPUT = "close_output"
CONF_CURRENT_SENSOR = "current_sensor"
CONF_CURRENT_THRESHOLD = "current_threshold"
CONF_MOVEMENT_TIMEOUT = "movement_timeout"
CONF_RUNNING_CURRENT_CHECK_INTERVAL = "running_current_check_interval"
CONF_IDLE_CURRENT_CHECK_INTERVAL = "idle_current_check_interval"
CONF_MINIMUM_RUNNING_TIME = "minimum_running_time"

valve_ns = cg.esphome_ns.namespace("valve_controller")
ValveController = valve_ns.class_("ValveController", valve.Valve, cg.Component)

CONFIG_SCHEMA = (
    valve.valve_schema(ValveController)
    .extend(
        {
            cv.Required(CONF_OPEN_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_CLOSE_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_CURRENT_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_CURRENT_THRESHOLD, default=0.05): cv.positive_float,
            cv.Optional(CONF_MOVEMENT_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_RUNNING_CURRENT_CHECK_INTERVAL, default="20ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_IDLE_CURRENT_CHECK_INTERVAL, default="3s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MINIMUM_RUNNING_TIME, default="300ms"): cv.positive_time_period_milliseconds,
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

    cg.add(var.set_current_threshold_amps(config[CONF_CURRENT_THRESHOLD]))
    cg.add(var.set_movement_timeout_ms(config[CONF_MOVEMENT_TIMEOUT]))
    cg.add(var.set_running_current_check_interval_ms(config[CONF_RUNNING_CURRENT_CHECK_INTERVAL]))
    cg.add(var.set_idle_current_check_interval_ms(config[CONF_IDLE_CURRENT_CHECK_INTERVAL]))
    cg.add(var.set_minimum_running_time_ms(config[CONF_MINIMUM_RUNNING_TIME]))
