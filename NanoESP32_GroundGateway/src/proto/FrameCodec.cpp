// proto/FrameCodec.cpp
#include "FrameCodec.h"

namespace FrameCodec {

uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

size_t encode(uint8_t msg_type, uint8_t seq,
              const uint8_t *payload, uint8_t payload_len,
              uint8_t *out_buf, size_t out_cap)
{
    const uint8_t len = static_cast<uint8_t>(payload_len + 4);
    const size_t total = 3 + len;
    if (out_cap < total) return 0;

    out_buf[0] = SYNC1;
    out_buf[1] = SYNC2;
    out_buf[2] = len;
    out_buf[3] = msg_type;
    out_buf[4] = seq;
    if (payload_len && payload) {
        memcpy(&out_buf[5], payload, payload_len);
    }

    const size_t crc_input_len = static_cast<size_t>(1 + 1 + 1 + payload_len);
    const uint16_t crc = crc16_modbus(&out_buf[2], crc_input_len);
    out_buf[5 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
    out_buf[6 + payload_len] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return total;
}

void Parser::reset()
{
    state_ = State::WAIT_SYNC1;
    len_ = 0;
    body_pos_ = 0;
}

bool Parser::feed(uint8_t b, FrameView &out_frame)
{
    switch (state_) {
    case State::WAIT_SYNC1:
        if (b == SYNC1) state_ = State::WAIT_SYNC2;
        return false;
    case State::WAIT_SYNC2:
        if (b == SYNC2) state_ = State::WAIT_LEN;
        else reset();
        return false;
    case State::WAIT_LEN:
        len_ = b;
        if (len_ < 4 || len_ > (MAX_PAYLOAD + 4)) {
            reset();
            return false;
        }
        body_pos_ = 0;
        state_ = State::WAIT_BODY;
        return false;
    case State::WAIT_BODY:
        body_[body_pos_++] = b;
        if (body_pos_ < len_) return false;

        {
            const uint8_t msg_type = body_[0];
            const uint8_t seq = body_[1];
            const uint8_t payload_len = static_cast<uint8_t>(len_ - 4);
            const uint16_t crc_rx = static_cast<uint16_t>(body_[len_ - 2]) |
                                    (static_cast<uint16_t>(body_[len_ - 1]) << 8);

            uint8_t temp[1 + 2 + MAX_PAYLOAD] = {0};
            temp[0] = len_;
            temp[1] = msg_type;
            temp[2] = seq;
            if (payload_len) memcpy(&temp[3], &body_[2], payload_len);
            const uint16_t crc_calc = crc16_modbus(temp, static_cast<size_t>(1 + 2 + payload_len));
            if (crc_calc != crc_rx) {
                reset();
                return false;
            }

            out_frame.msg_type = msg_type;
            out_frame.seq = seq;
            out_frame.payload = (payload_len ? &body_[2] : nullptr);
            out_frame.payload_len = payload_len;

            reset();
            return true;
        }

    default:
        reset();
        return false;
    }
}

} // namespace FrameCodec
