// util/BoardConfig.h (AirGateway)
#pragma once

#include <Arduino.h>

namespace BoardConfig {

static constexpr uint32_t USB_BAUD  = 115200;
static constexpr uint32_t UART_BAUD = 115200;

// Nano ESP32 (ABX00083) UART 默认分配在 D0/D1（见官方 datasheet）：
//   D1 / TX
//   D0 / RX
// 注意：这与很多“D0=TX, D1=RX”的板子习惯相反。
// 连接到 Nano 33 BLE（Serial1: D0=RX, D1=TX）时，推荐交叉接线：
//   ESP32 D1(TX)  -> Nano33BLE D0(RX)
//   ESP32 D0(RX)  <- Nano33BLE D1(TX)
static constexpr int UART_RX_PIN = D0;
static constexpr int UART_TX_PIN = D1;

// 心跳建议比 33BLE 的 LINK_TIMEOUT_MS 更保守，避免串口偶发阻塞导致误判
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 500;

// LoRa 上行遥测转发周期（空中->地面）。
// Nano33BLE 端遥测可能更高频，但空口半双工，过高频会导致空中端在 TX 时错过地面下行控制。
// 建议从 500~1000ms 起步，稳定后再逐步加快。
static constexpr uint32_t LORA_TELEM_PERIOD_MS = 500;

// =======================
// LoRa (SX1278 / RA-01)
// =======================
// 端口约定（按你提供的实际接线表）：
//   SCK  -> D13
//   MISO -> D12 (CIPO)
//   MOSI -> D11 (COPI)
//   CS   -> D10 (NSS)
//   RST  -> D6
//   DIO0 -> D2

// 建议先用 433MHz（RA-01 / SX1278 常用频点），确保两端完全一致。
// 如需更精确的 433.92MHz，可改为 433920000。
static constexpr long LORA_FREQ_HZ = 433000000;

static constexpr int LORA_SCK  = D13;
static constexpr int LORA_MISO = D12;
static constexpr int LORA_MOSI = D11;
static constexpr int LORA_SS   = D10;
static constexpr int LORA_RST  = D6;
static constexpr int LORA_DIO0 = D2;

static constexpr int LORA_TX_POWER_DBM = 17;     // 2..20，视模块/法规调整
static constexpr int LORA_SPREADING_FACTOR = 7;  // 6..12
static constexpr long LORA_SIGNAL_BW = 125E3;    // 7.8E3..500E3
static constexpr int LORA_CODING_RATE_DENOM = 5; // 5..8, 对应 4/5..4/8
static constexpr bool LORA_ENABLE_CRC = true;
// 建议使用“非 LoRaWAN 公网”的私有 SyncWord，以减少接收到其他网络的包。
// 两端必须一致。
// 先使用 LoRa 默认 SyncWord 0x12（兼容性最好）。
// 若现场同频干扰严重，再两端同时改为私有值（如 0x42）。
static constexpr uint8_t LORA_SYNC_WORD = 0x12;

static constexpr uint32_t LORA_TX_GUARD_MS = 5;  // 简单防抖，避免极短间隔连续发包

} // namespace BoardConfig
