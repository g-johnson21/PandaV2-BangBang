#pragma once
#include <Arduino.h>
#include "BoardConfig.h"

// RS-485 half-duplex communication handler with DE pin control.
// Receives line-delimited packets, transmits with automatic DE assertion.
// Only complete, newline-terminated lines that fit the buffer become packets;
// unterminated fragments (idle timeout) and over-long lines are discarded.
// Adds XOR checksum to outgoing telemetry for noise resilience.

class CommsHandler {
public:
    CommsHandler(HardwareSerialIMXRT& port, uint8_t dePin, size_t rxBufSize = RS485_RX_BUF);

    void begin(uint32_t baud = RS485_BAUD);
    void poll();

    bool isPacketReady() const { return _ready; }

    // Fragments and over-long lines discarded since boot.
    uint32_t droppedCount() const { return _dropped; }
    char* takePacket();

    // TX helpers — assert DE, write, deassert DE
    void send(const char* msg);
    void sendLine(const char* msg);

    // Free space in the UART TX buffer. Periodic telemetry checks this and is
    // skipped rather than blocking when the bus is backed up.
    int availableForWrite() { return _port.availableForWrite(); }

    // Format a float array as a CSV telemetry row: "<id><val>,<val>,...\n"
    static size_t toCSVRow(const float* data, char id, uint8_t count,
                           char* buf, size_t bufLen, uint8_t decimals = DATA_DECIMALS);

private:
    HardwareSerialIMXRT& _port;
    uint8_t _dePin;
    char _rxBuf[RS485_RX_BUF];
    uint8_t _serialRxMem[RS485_RX_BUF];
    uint8_t _serialTxMem[RS485_TX_BUF];
    size_t _rxPos = 0;
    bool _ready = false;
    bool _overflow = false;
    uint32_t _dropped = 0;
    elapsedMillis _idleTimer;

    void deAssert();
    void deRelease();
};
