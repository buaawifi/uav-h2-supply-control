// drivers/UartLink.cpp
#include "UartLink.h"

#include <string.h>

void UartLink::begin(uint32_t baud)
{
    serial_.begin(baud);
}

void UartLink::poll(ControlState &state, uint32_t now_ms)
{
    while (serial_.available() > 0) {
        uint8_t b = static_cast<uint8_t>(serial_.read());
        FrameCodec::FrameView f;
        if (parser_.feed(b, f)) {
            handleFrame(f, state, now_ms);
        }
    }
}

void UartLink::handleFrame(const FrameCodec::FrameView &f, ControlState &state, uint32_t now_ms)
{
    state.last_cmd_ms = now_ms;

    // 只要收到了“CRC 校验通过的有效帧”，就认为链路是活的。
    // 这样即使中继/上位机未额外发送独立心跳，只要持续有控制/交互帧，
    // 就不会被 SafetyManager 判定为断链从而强制 SAFE。
    state.link_alive = true;
    state.last_link_heartbeat_ms = now_ms;

    switch (f.msg_type) {
    case Proto::MSG_HEARTBEAT:
        // 心跳帧本身同样会刷新链路时间戳（上面已做，这里保留语义注释）
        // 心跳无需 ACK
        break;

    case Proto::MSG_MODE_SWITCH:
        if (f.payload_len == sizeof(Proto::PayloadModeSwitch)) {
            Proto::PayloadModeSwitch p;
            memcpy(&p, f.payload, sizeof(p));

            if (p.mode == Proto::MODE_SAFE) {
                state.mode = ControlMode::SAFE;
                sendAck(f.msg_type, f.seq, Proto::ACK_OK);
            } else if (p.mode == Proto::MODE_MANUAL) {
                state.mode = ControlMode::MANUAL;
                sendAck(f.msg_type, f.seq, Proto::ACK_OK);
            } else if (p.mode == Proto::MODE_AUTO) {
                state.mode = ControlMode::AUTO;
                sendAck(f.msg_type, f.seq, Proto::ACK_OK);
            } else {
                sendAck(f.msg_type, f.seq, Proto::ACK_ERR);
            }
        } else {
            sendAck(f.msg_type, f.seq, Proto::ACK_ERR);
        }
        break;

    case Proto::MSG_MANUAL_CMD_V1:
        if (f.payload_len == sizeof(Proto::PayloadManualCmdV1)) {
            Proto::PayloadManualCmdV1 p;
            memcpy(&p, f.payload, sizeof(p));

            state.manual_cmd.has_heater_cmd = (p.flags & Proto::MAN_FLAG_HEATER) != 0;
            state.manual_cmd.has_valve_cmd  = (p.flags & Proto::MAN_FLAG_VALVE) != 0;
            state.manual_cmd.has_pump_temp_cmd = (p.flags & Proto::MAN_FLAG_PUMP) != 0;

            state.manual_cmd.heater_power_pct   = p.heater_power_pct;
            state.manual_cmd.valve_opening_pct  = p.valve_opening_pct;
            state.manual_cmd.pump_target_temp_c = p.pump_target_temp_c;

            state.last_manual_ms = now_ms;
            sendAck(f.msg_type, f.seq, Proto::ACK_OK);
        } else {
            sendAck(f.msg_type, f.seq, Proto::ACK_ERR);
        }
        break;

    case Proto::MSG_SETPOINTS_V1:
        if (f.payload_len == sizeof(Proto::PayloadSetpointsV1)) {
            Proto::PayloadSetpointsV1 p;
            memcpy(&p, f.payload, sizeof(p));

            state.setpoints.target_temp_c         = p.target_temp_c;
            state.setpoints.target_pressure_pa    = p.target_pressure_pa;
            state.setpoints.target_valve_opening_pct = p.target_valve_opening_pct;
            state.setpoints.target_pump_temp_c    = p.target_pump_temp_c;

            state.setpoints.enable_temp_ctrl      = (p.enable_mask & Proto::SP_ENABLE_TEMP) != 0;
            state.setpoints.enable_pressure_ctrl  = (p.enable_mask & Proto::SP_ENABLE_PRESSURE) != 0;
            state.setpoints.enable_valve_ctrl     = (p.enable_mask & Proto::SP_ENABLE_VALVE) != 0;

            state.last_setpoint_ms = now_ms;
            sendAck(f.msg_type, f.seq, Proto::ACK_OK);
        } else {
            sendAck(f.msg_type, f.seq, Proto::ACK_ERR);
        }
        break;

    default:
        // 未识别消息：不回 ACK，避免误触发重发机制
        break;
    }
}

void UartLink::sendAck(uint8_t acked_msg_type, uint8_t seq, uint8_t status)
{
    Proto::PayloadAck p;
    p.acked_msg_type = acked_msg_type;
    p.status = status;

    uint8_t buf[32];
    const size_t n = FrameCodec::encode(Proto::MSG_ACK, seq,
                                        reinterpret_cast<uint8_t*>(&p),
                                        static_cast<uint8_t>(sizeof(p)),
                                        buf, sizeof(buf));
    if (n) {
        serial_.write(buf, n);
    }
}

void UartLink::sendTelemetry(const Proto::Telemetry &telem,
                            const Proto::Outputs &out,
                            uint32_t now_ms)
{
    Proto::PayloadTelemetryV1 p;
    p.timestamp_ms = now_ms;

    const uint8_t nT = (telem.temp_count > 4) ? 4 : telem.temp_count;
    p.temp_count = nT;
    for (uint8_t i = 0; i < 4; ++i) {
        p.temp_c[i] = (i < nT) ? telem.temp_c[i] : 0.0f;
    }

    p.pressure_pa = telem.pressure_pa;
    p.heater_power_pct  = out.heater_power_pct;
    p.valve_opening_pct = out.valve_opening_pct;

    uint8_t buf[256];
    const size_t n = FrameCodec::encode(Proto::MSG_TELEM_V1, tx_seq_++,
                                        reinterpret_cast<uint8_t*>(&p),
                                        static_cast<uint8_t>(sizeof(p)),
                                        buf, sizeof(buf));
    if (n) {
        serial_.write(buf, n);
    }
}
