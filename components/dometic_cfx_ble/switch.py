import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TYPE, CONF_NAME

from esphome.components import switch

from . import DometicCfxBle, CONF_DOMETIC_CFX_BLE_ID

DometicCfxBleSwitch = dometic_cfx_ble_ns.class_("DometicCfxBleSwitch", esphome_switch.Switch, cg.PollingComponent)

CONF_SWITCH_TYPE = "type"  # or reuse CONF_TYPE directly

DometicCfxBleSwitch = dometic_cfx_ble_ns.class_("DometicCfxBleSwitch", switch.Switch)

CONFIG_SCHEMA = switch.switch_schema(DometicCfxBleSwitch).extend(
    {
        cv.GenerateID(CONF_DOMETIC_CFX_BLE_ID): cv.use_id(DometicCfxBle),
        cv.Required(CONF_SWITCH_TYPE): cv.string,  # or your own validator
    }
).extend(cv.polling_component_schema('60s'))

async def to_code(config):
    hub = await cg.get_variable(config[CONF_DOMETIC_CFX_BLE_ID])
    var = cg.new_Pvariable(config[CONF_ID])

    await switch.register_switch(var, config)
    parent = await cg.get_variable(config[CONF_DOMETIC_CFX_BLE_ID])
    cg.add(parent.add_entity(config[CONF_TYPE], var))
    cg.add(var.set_parent(parent))
    cg.add(var.set_topic(config[CONF_TYPE]))