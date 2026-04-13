# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import automation, codegen as cg, config_validation as cv
from esphome.const import CONF_ID, CONF_URL
from esphome.components.esp32 import add_idf_component

CODEOWNERS = ["@stuartparmenter"]
DEPENDENCIES = ["network", "ddp", "ddp_canvas"]

media_proxy_ns = cg.esphome_ns.namespace("media_proxy_control")
MediaProxyControl = media_proxy_ns.class_("MediaProxyControl", cg.Component)
MediaProxyOutput = media_proxy_ns.class_("MediaProxyOutput", cg.Component)

ddp_ns = cg.esphome_ns.namespace("ddp")
DdpComponent = ddp_ns.class_("DdpComponent", cg.Component)
DdpCanvas = ddp_ns.class_("DdpCanvas", cg.Component)

CONF_WS_HOST = "ws_host"
CONF_WS_PORT = "ws_port"
CONF_DEVICE_ID = "device_id"
CONF_DDP = "ddp"
CONF_OUTPUTS = "outputs"
CONF_STREAM = "stream"
CONF_SRC = "src"

OUTPUT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MediaProxyOutput),
        cv.Required(CONF_STREAM): cv.Any(
            cv.int_range(min=1, max=249),
            cv.use_id(DdpCanvas),
        ),
        cv.Required("src"): cv.string,
        cv.Optional("pace"): cv.int_range(min=0, max=240),
        cv.Optional("ema"): cv.float_range(min=0.0, max=1.0),
        cv.Optional("expand"): cv.Any(
            cv.int_range(min=0, max=2),
            cv.one_of("never", "auto", "force", "0", "1", "2", lower=True),
        ),
        cv.Optional("loop"): cv.boolean,
        cv.Optional("hw"): cv.one_of(
            "auto",
            "none",
            "cuda",
            "qsv",
            "vaapi",
            "videotoolbox",
            "d3d11va",
            lower=True,
        ),
        cv.Optional("format"): cv.one_of(
            "rgb888", "rgb565", "rgb565le", "rgb565be", "rgbw", lower=True
        ),
        cv.Optional("fit"): cv.one_of("auto", "pad", "cover", lower=True),
    }
)


def _consume_media_proxy_sockets(config):
    """Register socket needs for media_proxy_control component (ESPHome 2025.11+)."""
    try:
        from esphome.components import socket

        # media_proxy_control needs 1 WebSocket connection
        socket.consume_sockets(1, "media_proxy_control")(config)
    except AttributeError:
        # Backwards compatibility: socket.consume_sockets not available in older ESPHome
        pass
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(MediaProxyControl),
            # URL override (optional)
            cv.Optional(CONF_URL, default=""): cv.templatable(cv.string),
            cv.Optional(CONF_WS_HOST): cv.templatable(cv.string),
            cv.Optional(CONF_WS_PORT, default=8788): cv.int_range(min=1, max=65535),
            cv.Optional(CONF_DEVICE_ID, default="unknown"): cv.templatable(cv.string),
            # Orchestration
            cv.GenerateID(CONF_DDP): cv.use_id(DdpComponent),
            cv.Optional(CONF_OUTPUTS, default=[]): cv.ensure_list(OUTPUT_SCHEMA),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_media_proxy_sockets,
)

# Actions
SetSourceAction = media_proxy_ns.class_("SetSourceAction", automation.Action)
StartAction = media_proxy_ns.class_("StartAction", automation.Action)
StopAction = media_proxy_ns.class_("StopAction", automation.Action)


@automation.register_action(
    "media_proxy_control.set_source",
    SetSourceAction,
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.use_id(MediaProxyOutput),
            cv.Required(CONF_SRC): cv.templatable(cv.string),
        }
    ),
    synchronous=True,
)
async def set_source_action_to_code(config, action_id, template_arg, args):
    output = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, output)
    src_templ = await cg.templatable(config[CONF_SRC], args, cg.std_string)
    cg.add(var.set_src(src_templ))
    return var


@automation.register_action(
    "media_proxy_control.start",
    StartAction,
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.use_id(MediaProxyOutput),
        }
    ),
    synchronous=True,
)
async def start_action_to_code(config, action_id, template_arg, args):
    output = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, output)
    return var


@automation.register_action(
    "media_proxy_control.stop",
    StopAction,
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.use_id(MediaProxyOutput),
        }
    ),
    synchronous=True,
)
async def stop_action_to_code(config, action_id, template_arg, args):
    output = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, output)
    return var


def _templ(expr):
    return cg.templatable(expr, [], cg.std_string)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Networking / socket config
    cg.add(var.set_url(await _templ(config.get(CONF_URL, ""))))
    if CONF_WS_HOST in config:
        cg.add(var.set_ws_host(await _templ(config[CONF_WS_HOST])))
    cg.add(var.set_ws_port(config.get(CONF_WS_PORT, 8788)))
    cg.add(var.set_device_id(await _templ(config.get(CONF_DEVICE_ID, "unknown"))))

    # Bind ddp component
    ddp = await cg.get_variable(config[CONF_DDP])
    cg.add(var.set_ddp(ddp))

    # Create MediaProxyOutput components
    for s in config[CONF_OUTPUTS]:
        stream_param = s[CONF_STREAM]

        # Handle polymorphic stream parameter (int or DdpCanvas component)
        if isinstance(stream_param, int):
            # Direct integer stream ID
            stream_id = stream_param
        else:
            # Component reference - get the component and extract stream_id
            stream_component = await cg.get_variable(stream_param)
            stream_id = cg.RawExpression(f"{stream_component}->get_stream_id()")

        output = cg.new_Pvariable(s[CONF_ID])
        await cg.register_component(output, s)
        cg.add(output.set_stream_id(stream_id))

        # Handle expand enum/int
        exp = s.get("expand", "auto")
        if isinstance(exp, (int, float)):
            exp_i = max(0, min(2, int(exp)))
        else:
            e = str(exp).strip().lower()
            if e in ("never", "0"):
                exp_i = 0
            elif e in ("auto", "1"):
                exp_i = 1
            elif e in ("force", "2"):
                exp_i = 2
            else:
                exp_i = 1

        # Set all configuration before wiring up parent (avoids spurious updates during init)
        if "src" in s:
            cg.add(output.set_src(s["src"]))
        if "pace" in s:
            cg.add(output.set_pace(s["pace"]))
        if "ema" in s:
            cg.add(output.set_ema(s["ema"]))
        if exp_i is not None:
            cg.add(output.set_expand(exp_i))
        if "loop" in s:
            cg.add(output.set_loop(s["loop"]))
        if "hw" in s:
            cg.add(output.set_hw(s["hw"]))
        if "format" in s:
            cg.add(output.set_format(s["format"]))
        if "fit" in s:
            cg.add(output.set_fit(s["fit"]))

        # Wire up parent last, after all configuration is set
        cg.add(output.set_parent(var))
        cg.add(var.register_output(output, stream_id))

    add_idf_component(name="espressif/esp_websocket_client", ref="1.5.0")
