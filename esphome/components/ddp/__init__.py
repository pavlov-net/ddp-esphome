# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, CONF_PORT
from esphome.components.light.effects import register_addressable_effect
from esphome.components.light.types import AddressableLightEffect
from esphome.components.lvgl.schemas import register_lvgl_widget
from esphome.components.lvgl.types import WidgetType, LvType
from esphome.components.lvgl import defines as lvgl_df

CODEOWNERS = ["@stuartparmenter"]
DEPENDENCIES = ["network"]

ddp_ns = cg.esphome_ns.namespace("ddp")
DdpComponent = ddp_ns.class_("DdpComponent", cg.Component)
DdpLightEffect = ddp_ns.class_("DdpLightEffect", AddressableLightEffect)

# Configuration keys
CONF_DDP_ID = "ddp_id"
CONF_STREAM = "stream"
CONF_METRICS = "metrics"

# Shared stream ID registry (used by ddp_canvas and ddp_light_effect)
_used_stream_ids = set()
_next_auto_stream_id = 2


def register_stream_id(stream_id):
    """Register an explicitly specified stream ID."""
    if stream_id in _used_stream_ids:
        raise cv.Invalid(
            f"Duplicate DDP stream ID {stream_id}! Each stream ID must be unique across all ddp_canvas and ddp_light_effect components."
        )
    _used_stream_ids.add(stream_id)


def allocate_stream_id():
    """Allocate the next available auto-generated stream ID."""
    global _next_auto_stream_id

    while _next_auto_stream_id in _used_stream_ids and _next_auto_stream_id < 250:
        _next_auto_stream_id += 1

    if _next_auto_stream_id > 249:
        raise cv.Invalid("Too many DDP streams: exceeded limit (2-249)")

    stream_id = _next_auto_stream_id
    _used_stream_ids.add(stream_id)
    _next_auto_stream_id += 1

    return stream_id


def _consume_ddp_sockets(config):
    """Register socket needs for DDP component (ESPHome 2025.11+)."""
    try:
        from esphome.components import socket

        # DDP needs 1 listening UDP socket
        socket.consume_sockets(1, "ddp")(config)
    except AttributeError:
        # Backwards compatibility: socket.consume_sockets not available in older ESPHome
        pass
    return config


# DDP component schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DdpComponent),
            cv.Optional(CONF_PORT, default=4048): cv.port,
            cv.Optional(CONF_METRICS, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_ddp_sockets,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    # Define DDP_METRICS build flag if metrics are enabled
    if config[CONF_METRICS]:
        cg.add_build_flag("-DDDP_METRICS")


# ========================================================================
# Register ddp_light_effect
# ========================================================================


@register_addressable_effect(
    "ddp_light_effect",
    DdpLightEffect,
    "DDP Stream",
    {
        cv.GenerateID(CONF_DDP_ID): cv.use_id(DdpComponent),
        cv.Optional(CONF_STREAM): cv.int_range(
            min=1, max=249
        ),  # Optional - auto-generates if not specified
    },
)
async def ddp_light_effect_to_code(config, effect_id):
    """Register a light effect renderer"""
    parent = await cg.get_variable(config[CONF_DDP_ID])

    # Auto-generate stream ID if not specified (using shared registry)
    if CONF_STREAM in config:
        stream_id = config[CONF_STREAM]
        register_stream_id(stream_id)
    else:
        stream_id = allocate_stream_id()

    effect = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(effect.set_parent(parent))
    cg.add(effect.set_stream_id(stream_id))

    # Register with parent DDP component
    cg.add(parent.register_renderer(effect))

    return effect


# ========================================================================
# Register ddpimg widget
# ========================================================================

# Define custom LVGL widget type
lv_ddpimg_t = LvType("lv_ddpimg_t")


# Custom WidgetType subclass (like CanvasType in canvas.py)
class DdpImgType(WidgetType):
    def __init__(self):
        super().__init__(
            name="ddpimg",
            w_type=lv_ddpimg_t,
            parts=(lvgl_df.CONF_MAIN,),
            schema={
                cv.GenerateID(CONF_DDP_ID): cv.use_id(DdpComponent),
                cv.Optional(CONF_STREAM): cv.int_range(min=1, max=249),
                cv.Optional("width"): cv.int_,
                cv.Optional("height"): cv.int_,
                cv.Optional("back_buffers", default=2): cv.one_of(0, 1, 2, int=True),
            },
            modify_schema={},
            lv_name="ddpimg",
        )

    async def to_code(self, w, config):
        """Generate code for ddpimg widget - called by LVGL framework"""
        # Get DDP parent component
        parent = await cg.get_variable(config[CONF_DDP_ID])

        # Set DDP parent
        cg.add(cg.RawExpression(f"lv_ddpimg_set_ddp_parent({w.obj}, {parent})"))

        # Stream ID (auto-generate or use explicit)
        if CONF_STREAM in config:
            stream_id = config[CONF_STREAM]
            register_stream_id(stream_id)
        else:
            stream_id = allocate_stream_id()
        cg.add(cg.RawExpression(f"lv_ddpimg_set_stream_id({w.obj}, {stream_id})"))

        # Size (optional)
        if "width" in config and "height" in config:
            cg.add(
                cg.RawExpression(
                    f"lv_ddpimg_set_size({w.obj}, {config['width']}, {config['height']})"
                )
            )

        # Back buffers
        cg.add(
            cg.RawExpression(
                f"lv_ddpimg_set_back_buffers({w.obj}, {config.get('back_buffers', 2)})"
            )
        )

        # Initialize (allocate buffers, register with DDP)
        cg.add(cg.RawExpression(f"lv_ddpimg_init({w.obj})"))


# Create widget spec instance
ddpimg_spec = DdpImgType()


@register_lvgl_widget(ddpimg_spec)
async def ddpimg_to_code(w, config):
    """Code generation for ddpimg widget - registered via decorator"""
    return await ddpimg_spec.to_code(w, config)
