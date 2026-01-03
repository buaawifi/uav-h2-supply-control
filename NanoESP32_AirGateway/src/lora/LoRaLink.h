// lora/LoRaLink.h
#pragma once

#include <Arduino.h>

namespace LoRaLink {

struct RxPacket {
    int len = 0;
    int rssi = 0;
    float snr = 0.0f;
};

enum class TxResult : uint8_t {
    OK = 0,
    BUSY = 1,  // 发送保护间隔或底层通道忙（本次不发）
    FAIL = 2   // 参数错误或底层发送失败
};

// 初始化 LoRa（SX127x）。返回 true 表示初始化成功。
bool begin();

// 扩展发送接口：区分 BUSY/FAIL。
TxResult sendEx(const uint8_t *payload, size_t len);

// 兼容旧接口：仅返回是否发送成功。
inline bool send(const uint8_t *payload, size_t len) {
    return sendEx(payload, len) == TxResult::OK;
}

// 轮询接收（非阻塞）。若收到包则写入 buf 并返回 true。
bool pollReceive(uint8_t *buf, size_t cap, RxPacket &out);

} // namespace LoRaLink
