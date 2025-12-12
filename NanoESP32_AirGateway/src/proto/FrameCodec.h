// src/proto/FrameCodec.h
#pragma once

#include <Arduino.h>

namespace Proto {

class FrameCodec {
public:
    static const uint8_t MAX_PAYLOAD_SIZE = 48; // 足够当前 26 字节的遥测

    struct Frame {
        uint8_t msg_type;
        uint8_t seq_id;
        uint8_t payload[MAX_PAYLOAD_SIZE];
        uint8_t payload_len;
    };

    typedef void (*FrameHandler)(const Frame &frame, void *user);

    FrameCodec();

    void reset();
    void processByte(uint8_t b, FrameHandler handler, void *user);

    // 发送一帧：会自动加帧头、Len 和 CRC16
    static void sendFrame(Stream &port,
                          uint8_t msg_type,
                          uint8_t seq_id,
                          const uint8_t *payload,
                          uint8_t payload_len);

private:
    enum class State : uint8_t {
        WAIT_HEADER1,
        WAIT_HEADER2,
        READ_LEN,
        READ_BODY
    };

    State  state_;
    uint8_t len_byte_;
    uint8_t body_buf_[64];  // 存 MsgType..CRC 部分
    uint8_t body_idx_;
};

} // namespace Proto
