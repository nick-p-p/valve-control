import esphome.codegen as cg
from esphome.components import i2c, output, text_sensor, valve
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.types import ConfigType

AUTO_LOAD = ["valve", "i2c", "text_sensor"]
DEPENDENCIES = ["valve", "i2c", "text_sensor"]

CONF_OPEN_OUTPUT = "open_output"
CONF_CLOSE_OUTPUT = "close_output"
CONF_CURRENT_THRESHOLD = "current_threshold"
CONF_MOVEMENT_TIMEOUT = "movement_timeout"
CONF_RUNNING_CURRENT_CHECK_INTERVAL = "running_current_check_interval"
CONF_IDLE_CURRENT_CHECK_INTERVAL = "idle_current_check_interval"
CONF_MINIMUM_RUNNING_TIME = "minimum_running_time"
CONF_SHUNT_RESISTANCE_OHMS = "shunt_resistance_ohms"
CONF_MAX_EXPECTED_CURRENT_AMPS = "max_expected_current_amps"
CONF_VERSION_TEXT = "version_text"
CONF_STATUS_TEXT = "status_text"
CONF_HEALTH_TEXT = "health_text"

valve_ns = cg.esphome_ns.namespace("valve_controller")
ValveController = valve_ns.class_("ValveController", valve.Valve, cg.Component, i2c.I2CDevice)
ValveVersionTextSensor = valve_ns.class_("ValveVersionTextSensor", text_sensor.TextSensor)
ValveStatusTextSensor = valve_ns.class_("ValveStatusTextSensor", text_sensor.TextSensor)
ValveHealthTextSensor = valve_ns.class_("ValveHealthTextSensor", text_sensor.TextSensor)

CONFIG_SCHEMA = (
    valve.valve_schema(ValveController)
    .extend(
        {
            cv.Required(CONF_OPEN_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_CLOSE_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_CURRENT_THRESHOLD, default=0.05): cv.positive_float,
            cv.Optional(CONF_MOVEMENT_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_RUNNING_CURRENT_CHECK_INTERVAL, default="20ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_IDLE_CURRENT_CHECK_INTERVAL, default="3s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MINIMUM_RUNNING_TIME, default="300ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SHUNT_RESISTANCE_OHMS, default=0.1): cv.positive_float,
            cv.Optional(CONF_MAX_EXPECTED_CURRENT_AMPS, default=3.2): cv.positive_float,
            cv.Optional(CONF_VERSION_TEXT): text_sensor.text_sensor_schema(
                ValveVersionTextSensor,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_STATUS_TEXT): text_sensor.text_sensor_schema(
                ValveStatusTextSensor,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_HEALTH_TEXT): text_sensor.text_sensor_schema(
                ValveHealthTextSensor,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(i2c.i2c_device_schema(0x40))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config: ConfigType) -> None:
    var = await valve.new_valve(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    open_output = await cg.get_variable(config[CONF_OPEN_OUTPUT])
    cg.add(var.set_open_output(open_output))

    close_output = await cg.get_variable(config[CONF_CLOSE_OUTPUT])
    cg.add(var.set_close_output(close_output))

    cg.add(var.set_current_threshold_amps(config[CONF_CURRENT_THRESHOLD]))
    cg.add(var.set_movement_timeout_ms(config[CONF_MOVEMENT_TIMEOUT]))
    cg.add(var.set_running_current_check_interval_ms(config[CONF_RUNNING_CURRENT_CHECK_INTERVAL]))
    cg.add(var.set_idle_current_check_interval_ms(config[CONF_IDLE_CURRENT_CHECK_INTERVAL]))
    cg.add(var.set_minimum_running_time_ms(config[CONF_MINIMUM_RUNNING_TIME]))
    cg.add(var.set_shunt_resistance_ohms(config[CONF_SHUNT_RESISTANCE_OHMS]))
    cg.add(var.set_max_expected_current_amps(config[CONF_MAX_EXPECTED_CURRENT_AMPS]))

    if version_text_config := config.get(CONF_VERSION_TEXT):
        version_text = await text_sensor.new_text_sensor(version_text_config)
        cg.add(var.set_version_text_sensor(version_text))

    if status_text_config := config.get(CONF_STATUS_TEXT):
        status_text = await text_sensor.new_text_sensor(status_text_config)
        cg.add(var.set_status_text_sensor(status_text))

    if health_text_config := config.get(CONF_HEALTH_TEXT):
        health_text = await text_sensor.new_text_sensor(health_text_config)
        cg.add(var.set_health_text_sensor(health_text))
