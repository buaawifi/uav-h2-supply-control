// lora/LoRaLink.cpp
//
// 目标：提高 Nano ESP32 + RA-01(SX1278) 的长时间运行鲁棒性。
//
// 关键策略：
// 1) 不依赖 Arduino-LoRa(LoRa.h) 的阻塞 endPacket() 以及库内部状态机。
// 2) 直接通过 SPI 访问 SX127x 寄存器：TX/RX 切换、IRQ 清理、TxDone/RxDone 轮询。
// 3) 所有操作都带硬超时与自愈（硬复位 + 重新初始化）。

#include "LoRaLink.h"

#include <SPI.h>

#include "../util/BoardConfig.h"

namespace LoRaLink {
namespace {

// ---- SX127x register map (LoRa mode) ----
constexpr uint8_t REG_FIFO                 = 0x00;
constexpr uint8_t REG_OP_MODE              = 0x01;
constexpr uint8_t REG_FRF_MSB              = 0x06;
constexpr uint8_t REG_FRF_MID              = 0x07;
constexpr uint8_t REG_FRF_LSB              = 0x08;
constexpr uint8_t REG_PA_CONFIG            = 0x09;
constexpr uint8_t REG_OCP                  = 0x0B;
constexpr uint8_t REG_LNA                  = 0x0C;
constexpr uint8_t REG_FIFO_ADDR_PTR        = 0x0D;
constexpr uint8_t REG_FIFO_TX_BASE_ADDR    = 0x0E;
constexpr uint8_t REG_FIFO_RX_BASE_ADDR    = 0x0F;
constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
constexpr uint8_t REG_IRQ_FLAGS_MASK       = 0x11;
constexpr uint8_t REG_IRQ_FLAGS            = 0x12;
constexpr uint8_t REG_RX_NB_BYTES          = 0x13;
constexpr uint8_t REG_PKT_SNR_VALUE        = 0x1B;
constexpr uint8_t REG_PKT_RSSI_VALUE       = 0x1A;
constexpr uint8_t REG_MODEM_CONFIG_1       = 0x1D;
constexpr uint8_t REG_MODEM_CONFIG_2       = 0x1E;
constexpr uint8_t REG_PREAMBLE_MSB         = 0x20;
constexpr uint8_t REG_PREAMBLE_LSB         = 0x21;
constexpr uint8_t REG_PAYLOAD_LENGTH       = 0x22;
constexpr uint8_t REG_MODEM_CONFIG_3       = 0x26;
constexpr uint8_t REG_SYNC_WORD            = 0x39;
constexpr uint8_t REG_PA_DAC               = 0x4D;
constexpr uint8_t REG_VERSION              = 0x42;

// ---- OpMode ----
constexpr uint8_t LONG_RANGE_MODE = 0x80; // LoRa
constexpr uint8_t MODE_SLEEP      = 0x00;
constexpr uint8_t MODE_STDBY      = 0x01;
constexpr uint8_t MODE_TX         = 0x03;
constexpr uint8_t MODE_RX_CONT    = 0x05;

// ---- IRQ flags ----
constexpr uint8_t IRQ_RX_DONE            = 0x40;
constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR  = 0x20;
constexpr uint8_t IRQ_TX_DONE            = 0x08;

// SPI settings：杜邦线 + 外置 DC-DC + SX127x，建议保守。
constexpr uint32_t SPI_HZ = 1000000;
static SPISettings g_spi(SPI_HZ, MSBFIRST, SPI_MODE0);

static uint32_t g_last_tx_ms = 0;
static uint32_t g_last_rx_ms = 0;
static uint32_t g_last_force_rx_ms = 0;

static inline void csSelect()   { digitalWrite(BoardConfig::LORA_SS, LOW); }
static inline void csDeselect() { digitalWrite(BoardConfig::LORA_SS, HIGH); }

static uint8_t readReg(uint8_t addr)
{
    SPI.beginTransaction(g_spi);
    csSelect();
    SPI.transfer(addr & 0x7F);
    const uint8_t v = SPI.transfer(0x00);
    csDeselect();
    SPI.endTransaction();
    return v;
}

static void writeReg(uint8_t addr, uint8_t val)
{
    SPI.beginTransaction(g_spi);
    csSelect();
    SPI.transfer(addr | 0x80);
    SPI.transfer(val);
    csDeselect();
    SPI.endTransaction();
}

static void writeFifo(const uint8_t* data, size_t len)
{
    SPI.beginTransaction(g_spi);
    csSelect();
    SPI.transfer(REG_FIFO | 0x80);
    for (size_t i = 0; i < len; ++i) {
        SPI.transfer(data[i]);
    }
    csDeselect();
    SPI.endTransaction();
}

static void readFifo(uint8_t* data, size_t len)
{
    SPI.beginTransaction(g_spi);
    csSelect();
    SPI.transfer(REG_FIFO & 0x7F);
    for (size_t i = 0; i < len; ++i) {
        data[i] = SPI.transfer(0x00);
    }
    csDeselect();
    SPI.endTransaction();
}

static void hardResetRadio()
{
    pinMode(BoardConfig::LORA_RST, OUTPUT);
    digitalWrite(BoardConfig::LORA_RST, HIGH);
    delay(5);
    digitalWrite(BoardConfig::LORA_RST, LOW);
    delay(2);
    digitalWrite(BoardConfig::LORA_RST, HIGH);
    delay(10);
}

static void setOpMode(uint8_t mode)
{
    writeReg(REG_OP_MODE, LONG_RANGE_MODE | (mode & 0x07));
}

static void clearIrq(uint8_t flags = 0xFF)
{
    // 写 1 清除对应位
    writeReg(REG_IRQ_FLAGS, flags);
}

static uint8_t bwToReg(long bw_hz)
{
    // LoRa BW code: 7.8k..500k
    if (bw_hz <= 7800) return 0;
    if (bw_hz <= 10400) return 1;
    if (bw_hz <= 15600) return 2;
    if (bw_hz <= 20800) return 3;
    if (bw_hz <= 31250) return 4;
    if (bw_hz <= 41700) return 5;
    if (bw_hz <= 62500) return 6;
    if (bw_hz <= 125000) return 7;
    if (bw_hz <= 250000) return 8;
    return 9;
}

static void applyConfig()
{
    // sleep -> standby
    setOpMode(MODE_SLEEP);
    delay(2);
    setOpMode(MODE_STDBY);
    delay(2);

    // Frequency
    // FRF = freq * 2^19 / 32e6
    const uint64_t frf = ((uint64_t)BoardConfig::LORA_FREQ_HZ << 19) / 32000000ULL;
    writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));

    // FIFO base
    writeReg(REG_FIFO_TX_BASE_ADDR, 0x00);
    writeReg(REG_FIFO_RX_BASE_ADDR, 0x00);
    writeReg(REG_FIFO_ADDR_PTR, 0x00);

    // LNA: boost on (bits 1:0 = 0b11)
    writeReg(REG_LNA, (readReg(REG_LNA) & 0xFC) | 0x03);

    // OCP: 跟你当前 dump 保持一致（0x2B）
    writeReg(REG_OCP, 0x2B);

    // SyncWord
    writeReg(REG_SYNC_WORD, BoardConfig::LORA_SYNC_WORD);

    // Preamble = 8
    writeReg(REG_PREAMBLE_MSB, 0x00);
    writeReg(REG_PREAMBLE_LSB, 0x08);

    // ModemConfig1: BW + CR + explicit header
    const uint8_t bw = bwToReg(BoardConfig::LORA_SIGNAL_BW);
    uint8_t cr = 1; // 4/5
    if (BoardConfig::LORA_CODING_RATE_DENOM <= 5) cr = 1;
    else if (BoardConfig::LORA_CODING_RATE_DENOM == 6) cr = 2;
    else if (BoardConfig::LORA_CODING_RATE_DENOM == 7) cr = 3;
    else cr = 4;
    const uint8_t mc1 = (bw << 4) | (cr << 1) | 0x00;
    writeReg(REG_MODEM_CONFIG_1, mc1);

    // ModemConfig2: SF + CRC
    const uint8_t sf = (uint8_t)BoardConfig::LORA_SPREADING_FACTOR;
    uint8_t mc2 = (sf << 4);
    if (BoardConfig::LORA_ENABLE_CRC) mc2 |= 0x04;
    writeReg(REG_MODEM_CONFIG_2, mc2);

    // ModemConfig3: AGC auto on + low data rate optimize if needed
    uint8_t mc3 = 0x04; // AGC auto
    const bool ldr = (sf >= 11) && (BoardConfig::LORA_SIGNAL_BW <= 125000);
    if (ldr) mc3 |= 0x08;
    writeReg(REG_MODEM_CONFIG_3, mc3);

    // Tx power: 默认使用 PA_BOOST（RA-01 常用）
    int p = BoardConfig::LORA_TX_POWER_DBM;
    if (p < 2) p = 2;
    if (p > 17) p = 17;
    writeReg(REG_PA_CONFIG, 0x80 | (uint8_t)(p - 2));
    writeReg(REG_PA_DAC, 0x84);

    // IRQ mask: 不屏蔽
    writeReg(REG_IRQ_FLAGS_MASK, 0x00);

    clearIrq();

    // back to RX continuous
    setOpMode(MODE_RX_CONT);
    g_last_force_rx_ms = millis();
}

static bool reinit()
{
    // 轻量自愈：RST + 重新写配置。
    hardResetRadio();

    const uint8_t ver = readReg(REG_VERSION);
    if (ver == 0x00 || ver == 0xFF) {
        return false;
    }

    applyConfig();
    g_last_tx_ms = 0;
    g_last_rx_ms = 0;
    g_last_force_rx_ms = millis();
    return true;
}

static void ensureRx(uint32_t now)
{
    // 兜底：周期性强制回到 RX_CONT（防止某次异常后停在 STDBY/其它模式）。
    if (now - g_last_force_rx_ms < 300) return;
    g_last_force_rx_ms = now;

    const uint8_t op = readReg(REG_OP_MODE) & 0x07;
    if (op != MODE_RX_CONT) {
        clearIrq();
        setOpMode(MODE_RX_CONT);
    }
}

static inline int computeRssiDbm(uint8_t rssiRaw)
{
    // SX127x 数据手册：LF 频段（<=525MHz）Packet RSSI = -164 + raw
    // 你用 433MHz，属于 LF。
    return -164 + (int)rssiRaw;
}

static inline float computeSnr(uint8_t snrRaw)
{
    // two's complement，单位 0.25dB
    const int8_t v = (int8_t)snrRaw;
    return ((float)v) / 4.0f;
}

} // namespace

bool begin()
{
    pinMode(BoardConfig::LORA_SS, OUTPUT);
    csDeselect();

    SPI.begin(BoardConfig::LORA_SCK, BoardConfig::LORA_MISO,
              BoardConfig::LORA_MOSI, BoardConfig::LORA_SS);

    hardResetRadio();

    const uint8_t ver = readReg(REG_VERSION);
    if (ver == 0x00 || ver == 0xFF) {
        return false;
    }

    applyConfig();
    g_last_tx_ms = 0;
    g_last_rx_ms = 0;
    g_last_force_rx_ms = millis();
    return true;
}

TxResult sendEx(const uint8_t *payload, size_t len)
{
    if (!payload || len == 0) return TxResult::FAIL;
    if (len > 255) return TxResult::FAIL;

    const uint32_t now = millis();
    if (now - g_last_tx_ms < BoardConfig::LORA_TX_GUARD_MS) {
        return TxResult::BUSY;
    }

    // 进入 STDBY
    setOpMode(MODE_STDBY);

    // FIFO ptr
    const uint8_t txBase = readReg(REG_FIFO_TX_BASE_ADDR);
    writeReg(REG_FIFO_ADDR_PTR, txBase);

    // clear IRQ
    clearIrq();

    // write payload
    writeFifo(payload, len);
    writeReg(REG_PAYLOAD_LENGTH, (uint8_t)len);

    // TX
    setOpMode(MODE_TX);

    // 等待 TxDone（带硬超时）
    constexpr uint32_t TX_TIMEOUT_MS = 800;
    const uint32_t t0 = millis();
    while (true) {
        const uint8_t irq = readReg(REG_IRQ_FLAGS);
        if (irq & IRQ_TX_DONE) {
            clearIrq(IRQ_TX_DONE);
            break;
        }
        if ((millis() - t0) > TX_TIMEOUT_MS) {
            // 自愈：radio 可能卡死或 SPI 读异常
            (void)reinit();
            setOpMode(MODE_RX_CONT);
            g_last_tx_ms = millis();
            return TxResult::FAIL;
        }
        delay(1);
    }

    // 回 RX
    setOpMode(MODE_RX_CONT);

    g_last_tx_ms = millis();
    return TxResult::OK;
}

bool pollReceive(uint8_t *buf, size_t cap, RxPacket &out)
{
    const uint32_t now = millis();
    ensureRx(now);

    const uint8_t irq = readReg(REG_IRQ_FLAGS);
    if (!(irq & IRQ_RX_DONE)) {
        return false;
    }

    // CRC error
    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        clearIrq(IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR);
        setOpMode(MODE_RX_CONT);
        return false;
    }

    const uint8_t rxBytes = readReg(REG_RX_NB_BYTES);
    const uint8_t curAddr = readReg(REG_FIFO_RX_CURRENT_ADDR);
    writeReg(REG_FIFO_ADDR_PTR, curAddr);

    const size_t n = (rxBytes <= cap) ? rxBytes : cap;
    if (n > 0 && buf) {
        readFifo(buf, n);
    }

    out.len = (int)n;
    out.rssi = computeRssiDbm(readReg(REG_PKT_RSSI_VALUE));
    out.snr  = computeSnr(readReg(REG_PKT_SNR_VALUE));

    g_last_rx_ms = now;

    // 清除所有 RX 相关 IRQ，保持状态机干净
    clearIrq(0xFF);
    setOpMode(MODE_RX_CONT);

    return (n > 0);
}

} // namespace LoRaLink
