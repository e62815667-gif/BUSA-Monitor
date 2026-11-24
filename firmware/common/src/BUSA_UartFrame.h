#pragma once
#include <stddef.h>
#include <stdint.h>

namespace busa_uart {
constexpr uint8_t delimiter = 0x7e, escape = 0x7d, version = 1;
constexpr size_t headerSize = 8, crcSize = 4;
inline uint32_t crc32(const uint8_t* data, size_t size) {
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
inline uint32_t read32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline void write32(uint8_t* p, uint32_t v) {
    for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}
struct Counters { uint32_t rx = 0, crcErrors = 0, formatErrors = 0, overflow = 0; };
struct Frame { uint8_t kind; uint32_t seq; const uint8_t* data; size_t size; };

// Buffer capacity is a caller-supplied, bounded payload limit plus header and CRC.
inline size_t encode(uint8_t kind, uint32_t seq, const uint8_t* data, size_t size,
                     uint8_t* raw, size_t rawCap, uint8_t* wire, size_t wireCap) {
    if (!kind || size > 65535 || rawCap < headerSize + crcSize ||
        size > rawCap - headerSize - crcSize || (size && !data) || !raw || !wire) return 0;
    raw[0] = version; raw[1] = kind; write32(raw + 2, seq);
    raw[6] = uint8_t(size); raw[7] = uint8_t(size >> 8);
    for (size_t i = 0; i < size; ++i) raw[headerSize + i] = data[i];
    const size_t n = headerSize + size;
    write32(raw + n, crc32(raw, n));
    size_t required = 2;
    for (size_t i = 0; i < n + crcSize; ++i)
        required += (raw[i] == delimiter || raw[i] == escape) ? 2 : 1;
    if (required > wireCap) return 0;
    size_t out = 0; wire[out++] = delimiter;
    for (size_t i = 0; i < n + crcSize; ++i) {
        if (raw[i] == delimiter || raw[i] == escape) {
            wire[out++] = escape; wire[out++] = raw[i] ^ 0x20;
        } else wire[out++] = raw[i];
    }
    wire[out++] = delimiter;
    return out;
}

class Decoder {
public:
    Decoder(uint8_t* buffer, size_t capacity) : buffer_(buffer), capacity_(buffer ? capacity : 0) {}
    Counters counters;
    // Returned frame is valid until the next call to push().
    bool push(uint8_t value, Frame& frame) {
        if (value == delimiter) {
            bool valid = false;
            if (active_ && !discard_ && (used_ || escaped_)) valid = finish(frame);
            active_ = true; discard_ = false; escaped_ = false; used_ = 0;
            return valid;
        }
        if (!active_ || discard_) return false;
        if (escaped_) {
            escaped_ = false;
            if (value != (delimiter ^ 0x20) && value != (escape ^ 0x20)) {
                ++counters.formatErrors; discard_ = true; return false;
            }
            value ^= 0x20;
        } else if (value == escape) { escaped_ = true; return false; }
        if (used_ == capacity_) { ++counters.overflow; discard_ = true; return false; }
        buffer_[used_++] = value;
        return false;
    }
private:
    bool finish(Frame& frame) {
        if (escaped_ || used_ < headerSize + crcSize || buffer_[0] != version || !buffer_[1]) {
            ++counters.formatErrors; return false;
        }
        const size_t payload = size_t(buffer_[6]) | (size_t(buffer_[7]) << 8);
        if (payload != used_ - headerSize - crcSize) { ++counters.formatErrors; return false; }
        if (read32(buffer_ + used_ - crcSize) != crc32(buffer_, used_ - crcSize)) {
            ++counters.crcErrors; return false;
        }
        frame = Frame{buffer_[1], read32(buffer_ + 2), buffer_ + headerSize, payload};
        ++counters.rx; return true;
    }
    uint8_t* buffer_;
    size_t capacity_, used_ = 0;
    bool active_ = false, escaped_ = false, discard_ = false;
};
} // namespace busa_uart
