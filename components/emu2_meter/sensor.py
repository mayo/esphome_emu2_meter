import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor #, modbus
from esphome.const import (
    CONF_EXPORT_ACTIVE_ENERGY,
    CONF_ID,
    CONF_IMPORT_ACTIVE_ENERGY,
    CONF_POWER_FACTOR,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_WATT,
)

CODEOWNERS = ["@mayo"]

CONF_TOTAL_ACTIVE_ENERGY = "total_active_energy"
CONF_MAXIMUM_DEMAND_ACTIVE_POWER = "maximum_demand_active_power"

UNIT_KILOWATT_HOURS = "kWh"
UNIT_CURRENCY = "$"

emu2_meter_ns = cg.esphome_ns.namespace("emu2_meter")
Emu2Meter = emu2_meter_ns.class_("Emu2Meter", cg.Component)

SENSORS = {
    CONF_TOTAL_ACTIVE_ENERGY: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_IMPORT_ACTIVE_ENERGY: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),
    CONF_EXPORT_ACTIVE_ENERGY: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    ),

    # CONF_POWER_FACTOR: sensor.sensor_schema(
    #     accuracy_decimals=3,
    #     device_class=DEVICE_CLASS_POWER_FACTOR,
    #     state_class=STATE_CLASS_MEASUREMENT,
    # ),

    CONF_MAXIMUM_DEMAND_ACTIVE_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),

    #Price

    "price": sensor.sensor_schema(
        unit_of_measurement=UNIT_CURRENCY,
        accuracy_decimals=3,
        # device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),

    #status

    #RSSI
    "rssi": sensor.sensor_schema(
        unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(Emu2Meter)})
    .extend(
        {cv.Optional(sensor_name): schema for sensor_name, schema in SENSORS.items()}
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for name in SENSORS:
        if name in config:
            sens = await sensor.new_sensor(config[name])
            cg.add(getattr(var, f"set_{name}_sensor")(sens))
