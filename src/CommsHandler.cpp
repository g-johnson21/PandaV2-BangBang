#include "CommsHandler.h"
#include <cstdio>

CommsHandler::CommsHandler(HardwareSerialIMXRT& port, uint8_t dePin, size_t rxBufSize)
    : _port(port), _dePin(dePin)
{
    (void)rxBufSize;
}

void CommsHandler::begin(uint32_t baud) {
    _port.begin(baud);

    // Expand Teensy's default 64-byte serial buffers
    _port.addMemoryForRead(_serialRxMem, sizeof(_serialRxMem));
    _port.addMemoryForWrite(_serialTxMem, sizeof(_serialTxMem));

    if (_dePin != 0xFF) {
        pinMode(_dePin, OUTPUT);
        deRelease();
    }

    _rxPos = 0;
    _ready = false;
}

void CommsHandler::poll() {
    while (_port.available() && !_ready) {
        char c = _port.read();
        _idleTimer = 0;

        if (c == '\n' || c == '\r') {
            if (_overflow) {
                // Line was longer than the buffer: the tail is gone, so the
                // head must not run as a command either.
                _overflow = false;
                _rxPos = 0;
                _dropped++;
            } else if (_rxPos > 0) {
                _rxBuf[_rxPos] = '\0';
                _ready = true;
            }
            return;
        }

        if (_rxPos < sizeof(_rxBuf) - 1) {
            _rxBuf[_rxPos++] = c;
        } else {
            _overflow = true;
        }
    }

    // Idle timeout — bytes with no terminator are line noise or a cut-off
    // command. Discard them: executing a fragment could arm the board or fire
    // a sequence on noise, and keeping them would corrupt the next real line.
    if (!_ready && (_rxPos > 0 || _overflow) && _idleTimer >= PACKET_IDLE_MS) {
        _rxPos = 0;
        _overflow = false;
        _dropped++;
    }
}

char* CommsHandler::takePacket() {
    _ready = false;
    _rxPos = 0;
    _idleTimer = 0;
    return _rxBuf;
}

void CommsHandler::send(const char* msg) {
    if (!msg) return;
    deAssert();
    _port.print(msg);
    // flush only needed when DE is GPIO-controlled (to hold DE until last bit)
    if (_dePin != 0xFF) _port.flush();
    deRelease();
}

void CommsHandler::sendLine(const char* msg) {
    if (!msg) return;
    deAssert();
    _port.println(msg);
    if (_dePin != 0xFF) _port.flush();
    deRelease();
}

size_t CommsHandler::toCSVRow(const float* data, char id, uint8_t count,
                               char* buf, size_t bufLen, uint8_t decimals)
{
    if (bufLen == 0) return 0;

    size_t pos = 0;
    for (uint8_t i = 0; i < count && pos < bufLen - 2; i++) {
        int n;
        if (i == 0)
            n = snprintf(buf + pos, bufLen - pos, "%c%.*f", id, decimals, data[i]);
        else
            n = snprintf(buf + pos, bufLen - pos, ",%c%.*f", id, decimals, data[i]);

        if (n < 0 || pos + n >= bufLen - 1) break;
        pos += n;
    }

    if (pos < bufLen - 1) {
        buf[pos++] = '\n';
    }
    buf[pos] = '\0';
    return pos;
}

void CommsHandler::deAssert() {
    if (_dePin != 0xFF) {
        digitalWrite(_dePin, HIGH);
        delayMicroseconds(5); // transceiver enable time ~60ns, margin for bus settle
    }
}

void CommsHandler::deRelease() {
    if (_dePin != 0xFF) {
        digitalWrite(_dePin, LOW);
    }
}
