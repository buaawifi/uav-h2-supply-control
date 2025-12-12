// src/drivers/UartLink.cpp
#include "UartLink.h"
#include <string.h>

using namespace Proto;

void UartLink::begin(Stream &port)
{
    port_ = &port;
}

void UartLink::poll(ControlState &state)
{
    if (!port_) return;

    state_in_callback_ = &state;
    while (port_->available() > 0) {
        uint8_t b = (uint8_t)port_->read();
        codec_.processByte(b, &UartLink::onFrameStatic, this);
    }
    state_in_callback_ = nullptr;
}

void UartLink::onFrameStatic(const FrameCodec::Frame &frame, void *user)
{
    UartLink *self = static_cast<UartLink *>(user);
    if (!self || !self->state_in_callback_) return;
    self->onFrame(frame, *self->state_in_callback_);
}

static float readFloatLE(const uint8_t *buf, uint8_t offset)
{
    float v;
    uint32_t raw = (uint32_t)buf[offset] |
                   ((uint32_t)buf[offset + 1] << 8) |
                   ((uint32_t)buf[offset + 2] << 16) |
                   ((uint32_t)buf[offset + 3] << 24);
    memcpy(&v, &raw, sizeof(float));
    return v;
}

void UartLink::onFrame(const FrameCodec::Frame &frame, ControlState &state)
{
    switch (frame.msg_type) {
    case 0x10: { // 模式切换： payload[0] = wire_mode (0=SAFE,1=MANUAL,2=AUTO)
        if (frame.payload_len < 1) return;
        uint8_t wire_mode = frame.payload[0];
        switch (wire_mode) {
        case 0:
            state.mode = ControlMode::SAFE;
            break;
        case 1:
            state.mode = ControlMode::MANUAL;
            break;
        case 2:
            state.mode = ControlMode::AUTO;
            break;
        default:
            // 非法值：忽略或强制 SAFE，这里先忽略
            break;
        }
        break;
    }

    case 0x11: { // 自动模式温度设定值： payload = float target_temp_c
        if (frame.payload_len < 4) return;
        float Tsp = readFloatLE(frame.payload, 0);
        state.setpoints.target_temp_c = Tsp;
        // 如果你有“启用自动温控”标志，可在此一起置位
        state.setpoints.enable_temp_ctrl = true;
        break;
    }

    case 0x12: { // 手动指令： payload = float heater_pct, float valve_pct
        if (frame.payload_len < 8) return;
        float heater_pct = readFloatLE(frame.payload, 0);
        float valve_pct  = readFloatLE(frame.payload, 4);

        // 这里假定 ControlState::manual_cmd 中有如下字段
        state.manual_cmd.heater_power_pct   = heater_pct;
        state.manual_cmd.valve_opening_pct  = valve_pct;
        state.manual_cmd.has_heater_cmd     = true;
        state.manual_cmd.has_valve_cmd      = true;
        state.last_manual_ms                = millis();
        break;
    }

    default:
        // 其他类型暂不处理
        break;
    }
}

// 小端写入工具
static void writeU32LE(uint8_t *buf, uint8_t &idx, uint32_t v)
{
    buf[idx++] = (uint8_t)(v & 0xFF);
    buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
    buf[idx++] = (uint8_t)((v >> 16) & 0xFF);
    buf[idx++] = (uint8_t)((v >> 24) & 0xFF);
}

static void writeFloatLE(uint8_t *buf, uint8_t &idx, float v)
{
    uint32_t raw;
    memcpy(&raw, &v, sizeof(float));
    writeU32LE(buf, idx, raw);
}

void UartLink::sendTelemetry(const Telemetry &telem,
                             const Outputs   &out,
                             const ControlState &state)
{
    if (!port_) return;

    uint8_t payload[FrameCodec::MAX_PAYLOAD_SIZE];
    uint8_t idx = 0;

    // 1) 时间戳
    writeU32LE(payload, idx, telem.timestamp_ms);

    // 2) 模式（线上的编码）
    uint8_t wire_mode = 0;
    switch (state.mode) {
    case ControlMode::SAFE:   wire_mode = 0; break;
    case ControlMode::MANUAL: wire_mode = 1; break;
    case ControlMode::AUTO:   wire_mode = 2; break;
    default:                  wire_mode = 0; break;
    }
    payload[idx++] = wire_mode;

    // 3) 温度数量
    uint8_t temp_count = telem.temp_count;
    payload[idx++] = temp_count;

    float t1 = 0.0f;
    float t2 = 0.0f;
    if (temp_count > 0) t1 = telem.temp_c[0];
    if (temp_count > 1) t2 = telem.temp_c[1];

    writeFloatLE(payload, idx, t1);
    writeFloatLE(payload, idx, t2);

    // 4) 压力 Pa
    writeFloatLE(payload, idx, telem.pressure_pa);

    // 5) 执行器输出
    writeFloatLE(payload, idx, out.heater_power_pct);
    writeFloatLE(payload, idx, out.valve_opening_pct);

    const uint8_t payload_len = idx;
    const uint8_t seq = nextSeq();

    FrameCodec::sendFrame(*port_, 0x01, seq, payload, payload_len);
}
