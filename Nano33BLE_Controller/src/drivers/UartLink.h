// src/drivers/UartLink.h
#pragma once

#include <Arduino.h>
#include "../ctrl/ControlState.h"
#include "../proto/Messages.h"
#include "../proto/FrameCodec.h"

class UartLink {
public:
    // 使用一个已初始化的串口（建议 Serial1）
    void begin(Stream &port);

    // 轮询串口接收缓存，解析帧并更新 ControlState
    void poll(ControlState &state);

    // 发送遥测帧（MsgType = 0x01）
    void sendTelemetry(const Proto::Telemetry &telem,
                       const Proto::Outputs   &out,
                       const ControlState     &state);

private:
    Stream *port_ = nullptr;
    Proto::FrameCodec codec_;
    uint8_t seq_counter_ = 0;

    // 为了在回调里访问当前 state，临时保存一个指针
    ControlState *state_in_callback_ = nullptr;

    static void onFrameStatic(const Proto::FrameCodec::Frame &frame, void *user);
    void        onFrame(const Proto::FrameCodec::Frame &frame, ControlState &state);

    uint8_t nextSeq() {
        ++seq_counter_;
        if (seq_counter_ == 0) {
            seq_counter_ = 1; // 避免 0 一直不变
        }
        return seq_counter_;
    }
};
