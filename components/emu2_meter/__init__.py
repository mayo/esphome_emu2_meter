# stealing from esphome mdns component
from esphome.const import (
    CONF_ID,
    CONF_DISABLED,
)
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import add_idf_component

CODEOWNERS = ["@mayo"]
# DEPENDENCIES = ["network"]

emu2_ns = cg.esphome_ns.namespace("emu2_meter")
Emu2Meter = emu2_ns.class_("Emu2Meter", cg.Component)


def _remove_id_if_disabled(value):
    value = value.copy()
    if value[CONF_DISABLED]:
        value.pop(CONF_ID)
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Emu2Meter),
            cv.Optional(CONF_DISABLED, default=False): cv.boolean,
            # cv.Optional(CONF_SERVICES, default=[]): cv.ensure_list(SERVICE_SCHEMA),
        }
    ),
    _remove_id_if_disabled,
)

async def to_code(config):

    add_idf_component(
        name="usb_host_cdc_acm",
        repo="https://github.com/espressif/esp-usb.git",
        path="host/class/cdc/usb_host_cdc_acm",
    )

    cg.add_library(
        name="emu2-data-parser",
        repository="https://github.com/mayo/emu2-data-parser",
        version="main",
    )
