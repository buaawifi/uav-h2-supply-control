// src/proto/FrameCodec.cpp
#include "FrameCodec.h"
#include <string.h> // memcpy

namespace Proto {

namespace {

// Modbus CRC16 (多处通用)
uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

} // namespace

FrameCodec::FrameCodec()
    : state_(State::WAIT_HEADER1),
      len_byte_(0),
      body_idx_(0)
{
}

void FrameCodec::reset()
{
    state_    = State::WAIT_HEADER1;
    len_byte_ = 0;
    body_idx_ = 0;
}

void FrameCodec::processByte(uint8_t b, FrameHandler handler, void *user)
{
    switch (state_) {
    case State::WAIT_HEADER1:
        if (b == 0x55) {
            state_ = State::WAIT_HEADER2;
        }
        break;

    case State::WAIT_HEADER2:
        if (b == 0xAA) {
            state_ = State::READ_LEN;
        } else {
            state_ = State::WAIT_HEADER1;
        }
        break;

    case State::READ_LEN:
        len_byte_ = b;
        // Len 至少包含：MsgType(1)+SeqID(1)+CRC16(2)
        if (len_byte_ < 4 || len_byte_ > sizeof(body_buf_)) {
            reset();
        } else {
            body_idx_ = 0;
            state_    = State::READ_BODY;
        }
        break;

    case State::READ_BODY:
        body_buf_[body_idx_++] = b;
        if (body_idx_ >= len_byte_) {
            // body_buf_ = [MsgType, SeqID, payload..., CRC_lo, CRC_hi]
            // CRC 计算范围: [Len] + [MsgType..Payload]
            if (len_byte_ < 4) {
                reset();
                break;
            }

            const uint8_t crc_lo = body_buf_[len_byte_ - 2];
            const uint8_t crc_hi = body_buf_[len_byte_ - 1];
            const uint16_t crc_recv = (uint16_t)crc_lo | ((uint16_t)crc_hi << 8);

            // 准备 CRC 输入：Len + MsgType..Payload
            uint8_t crc_buf[1 + sizeof(body_buf_)];
            crc_buf[0] = len_byte_;
            const uint8_t data_len = len_byte_ - 2; // 不含 CRC 自身
            memcpy(&crc_buf[1], body_buf_, data_len);

            const uint16_t crc_calc = crc16_modbus(crc_buf, 1 + data_len);

            if (crc_calc == crc_recv) {
                Frame f;
                f.msg_type    = body_buf_[0];
                f.seq_id      = body_buf_[1];
                f.payload_len = len_byte_ - 4; // 去掉 MsgType, SeqID, CRC16
                if (f.payload_len > MAX_PAYLOAD_SIZE) {
                    f.payload_len = MAX_PAYLOAD_SIZE;
                }
                if (f.payload_len > 0) {
                    memcpy(f.payload, &body_buf_[2], f.payload_len);
                }
                if (handler) {
                    handler(f, user);
                }
            }

            reset();
        }
        break;
    }
}

void FrameCodec::sendFrame(Stream &port,
                           uint8_t msg_type,
                           uint8_t seq_id,
                           const uint8_t *payload,
                           uint8_t payload_len)
{
    if (payload_len > MAX_PAYLOAD_SIZE) {
        payload_len = MAX_PAYLOAD_SIZE;
    }

    const uint8_t len = (uint8_t)(1 + 1 + payload_len + 2); // MsgType+SeqID+Payload+CRC16

    // 准备 CRC 输入：Len + MsgType + SeqID + Payload
    uint8_t crc_buf[1 + 1 + 1 + MAX_PAYLOAD_SIZE];
    uint8_t idx = 0;
    crc_buf[idx++] = len;
    crc_buf[idx++] = msg_type;
    crc_buf[idx++] = seq_id;
    if (payload_len > 0) {
        memcpy(&crc_buf[idx], payload, payload_len);
        idx += payload_len;
    }
    const uint16_t crc = crc16_modbus(crc_buf, idx);
    const uint8_t  crc_lo = (uint8_t)(crc & 0xFF);
    const uint8_t  crc_hi = (uint8_t)((crc >> 8) & 0xFF);

    // 发送完整帧
    port.write(0x55);
    port.write(0xAA);
    port.write(len);
    port.write(msg_type);
    port.write(seq_id);
    if (payload_len > 0) {
        port.write(payload, payload_len);
    }
    port.write(crc_lo);
    port.write(crc_hi);
}

} // namespace Proto
