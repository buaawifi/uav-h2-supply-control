// proto/FrameCodec.h
#pragma once

#include <Arduino.h>

namespace FrameCodec {

static constexpr uint8_t SYNC1 = 0x55;
static constexpr uint8_t SYNC2 = 0xAA;
static constexpr size_t  MAX_PAYLOAD = 220; // payload 上限

struct FrameView {
    uint8_t msg_type = 0;
    uint8_t seq      = 0;
    const uint8_t *payload = nullptr;
    uint8_t payload_len = 0;
};

uint16_t crc16_modbus(const uint8_t *data, size_t len);

size_t encode(uint8_t msg_type, uint8_t seq,
              const uint8_t *payload, uint8_t payload_len,
              uint8_t *out_buf, size_t out_cap);

class Parser {
public:
    bool feed(uint8_t b, FrameView &out_frame);

private:
    enum class State : uint8_t {
        WAIT_SYNC1,
        WAIT_SYNC2,
        WAIT_LEN,
        WAIT_BODY
    };

    State state_{State::WAIT_SYNC1};
    uint8_t len_{0};
    uint8_t body_[MAX_PAYLOAD + 4] = {0};
    uint16_t body_pos_{0};

    void reset();
};

} // namespace FrameCodec
