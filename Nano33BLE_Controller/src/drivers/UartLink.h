// drivers/UartLink.h
#pragma once

#include <Arduino.h>

#include "../proto/FrameCodec.h"
#include "../proto/Protocol.h"
#include "../proto/Messages.h"
#include "../ctrl/ControlState.h"

// UartLink：
// - 负责 Serial1 的帧收发
// - poll() 内部解析帧并更新 ControlState
// - sendTelemetry() 周期发送遥测

class UartLink {
public:
    explicit UartLink(HardwareSerial &serial = Serial1) : serial_(serial) {}

    void begin(uint32_t baud);

    void poll(ControlState &state, uint32_t now_ms);

    void sendTelemetry(const Proto::Telemetry &telem,
                      const Proto::Outputs &out,
                      uint32_t now_ms);

private:
    HardwareSerial &serial_;
    FrameCodec::Parser parser_;
    uint8_t tx_seq_{0};

    void handleFrame(const FrameCodec::FrameView &f, ControlState &state, uint32_t now_ms);
    void sendAck(uint8_t acked_msg_type, uint8_t seq, uint8_t status);
};
