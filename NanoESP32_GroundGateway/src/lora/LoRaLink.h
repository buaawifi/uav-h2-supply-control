// lora/LoRaLink.h
#pragma once

#include <Arduino.h>

namespace LoRaLink {

// 运行期诊断信息（用于定位“长时间运行后不再收发 / 频繁自愈”的根因）。
//
// 说明：
// - reinit_* 计数只统计 LoRaLink.cpp 内部触发的重初始化；
// - 上层（例如 GroundGateway 的 RX watchdog）直接调用 begin() 的次数不计入这里。
enum class ReinitReason : uint8_t {
    NONE = 0,
    REG_VERSION_BAD = 1, // 读到 0x00/0xFF 或前后不一致（更像 SPI/供电/EMI 问题）
    TX_TIMEOUT = 2,      // 发送等待 TxDone 超时
    OP_MODE_BAD = 3      // OpMode 读写异常（更像 SPI/射频内部异常）
};

struct Diag {
    uint32_t reinit_total = 0;
    uint32_t reinit_regver_bad = 0;
    uint32_t reinit_tx_timeout = 0;
    uint32_t reinit_opmode_bad = 0;

    ReinitReason last_reason = ReinitReason::NONE;
    uint32_t last_reinit_ms = 0;

    // 最近一次健康检查采样（用于快速判断“是 SPI 读错还是只是收不到包”）
    uint8_t last_regver = 0;
    uint8_t last_opmode = 0;
    uint8_t last_irqflags = 0;
};

// 只读诊断快照
const Diag& diag();

// 清零诊断计数（不影响 LoRaLink 的工作状态）
void clearDiag();

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
