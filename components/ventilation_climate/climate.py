import esphome.codegen as cg
from esphome.components import climate
from esphome.components.modbus_controller import ModbusController
from esphome.components.modbus_controller.const import CONF_MODBUS_CONTROLLER_ID
import esphome.config_validation as cv

CONF_MIN_TEMP = "min_temp"
CONF_MAX_TEMP = "max_temp"
CONF_TARGET_TEMP_STEP = "target_temp_step"

DEPENDENCIES = ["climate", "modbus_controller"]

ventilation_climate_ns = cg.esphome_ns.namespace("ventilation_climate")
VentilationClimate = ventilation_climate_ns.class_(
    "VentilationClimate", climate.Climate, cg.Component
)

CONFIG_SCHEMA = climate.climate_schema(VentilationClimate).extend(
    {
        cv.Required(CONF_MODBUS_CONTROLLER_ID): cv.use_id(ModbusController),
        cv.Required(CONF_MIN_TEMP): cv.temperature,
        cv.Required(CONF_MAX_TEMP): cv.temperature,
        cv.Required(CONF_TARGET_TEMP_STEP): cv.positive_float,
    }
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_MODBUS_CONTROLLER_ID])

    cg.add(var.set_parent(parent))
    cg.add(var.set_min_temperature(config[CONF_MIN_TEMP]))
    cg.add(var.set_max_temperature(config[CONF_MAX_TEMP]))
    cg.add(var.set_target_temperature_step(config[CONF_TARGET_TEMP_STEP]))
    cg.add(var.register_modbus_items(parent))
