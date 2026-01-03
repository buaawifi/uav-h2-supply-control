// util/BoardConfig.h (GroundGateway)
#pragma once

#include <Arduino.h>

namespace BoardConfig {

static constexpr uint32_t USB_BAUD = 115200;

// =======================
// LoRa (SX1278 / RA-01)
// =======================
// 端口约定（按你提供的实际接线表，需与空中中继一致）：
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

static constexpr int LORA_TX_POWER_DBM = 17;
static constexpr int LORA_SPREADING_FACTOR = 7;
static constexpr long LORA_SIGNAL_BW = 125E3;
static constexpr int LORA_CODING_RATE_DENOM = 5;
static constexpr bool LORA_ENABLE_CRC = true;
// 与空中端保持一致，使用私有 SyncWord 减少外部干扰
// 先使用 LoRa 默认 SyncWord 0x12（兼容性最好）。
// 若现场同频干扰严重，再两端同时改为私有值（如 0x42）。
static constexpr uint8_t LORA_SYNC_WORD = 0x12;

static constexpr uint32_t LORA_TX_GUARD_MS = 5;

// 可靠下行：地面端发送控制帧后，等待来自 33BLE 的 ACK（经空中中继回传）。
// 若超时未收到，则自动重发。
static constexpr uint32_t CMD_ACK_TIMEOUT_MS = 400;
static constexpr uint8_t  CMD_MAX_RETRY      = 3;

} // namespace BoardConfig
