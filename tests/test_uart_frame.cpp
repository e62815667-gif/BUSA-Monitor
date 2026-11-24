#include "../firmware/common/src/BUSA_UartFrame.h"
#include <assert.h>
#include <string.h>
using namespace busa_uart;
static unsigned feed(Decoder& decoder, const uint8_t* data, size_t size, Frame& frame) {
    unsigned count = 0;
    for (size_t i = 0; i < size; ++i) if (decoder.push(data[i], frame)) ++count;
    return count;
}
int main() {
    assert(crc32(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xcbf43926u);
    uint8_t payload[] = {0, 0x7d, 0x7e, 255}, raw[64], wire[128], rx[64];
    const size_t n = encode(1, 0x12345678, payload, sizeof(payload), raw, sizeof(raw), wire, sizeof(wire));
    assert(n && wire[0] == delimiter && wire[n-1] == delimiter);
    assert(encode(1, 0, payload, sizeof(payload), raw, sizeof(raw), wire, 1) == 0);
    Decoder d(rx, sizeof(rx)); Frame f{};
    assert(feed(d, wire, n, f) == 1);
    assert(f.kind == 1 && f.seq == 0x12345678 && f.size == sizeof(payload));
    assert(memcmp(f.data, payload, sizeof(payload)) == 0);
    wire[3] ^= 1;
    assert(feed(d, wire, n, f) == 0 && d.counters.crcErrors == 1);
    wire[3] ^= 1;
    assert(feed(d, wire, n-3, f) == 0);
    assert(feed(d, wire, n, f) == 1 && d.counters.formatErrors >= 1);
    uint8_t bad[] = {delimiter, escape, 1, delimiter};
    assert(feed(d, bad, sizeof(bad), f) == 0);
    uint8_t overflow[100]{}; overflow[0] = delimiter; overflow[99] = delimiter;
    assert(feed(d, overflow, sizeof(overflow), f) == 0 && d.counters.overflow == 1);
    assert(feed(d, wire, n, f) == 1 && d.counters.rx == 3);
    assert(encode(0, 0, payload, sizeof(payload), raw, sizeof(raw), wire, sizeof(wire)) == 0);
    assert(encode(1, 0, nullptr, 1, raw, sizeof(raw), wire, sizeof(wire)) == 0);
    assert(encode(1, 0, payload, sizeof(payload), raw, 10, wire, sizeof(wire)) == 0);
    const size_t empty = encode(2, 0, nullptr, 0, raw, sizeof(raw), wire, sizeof(wire));
    assert(empty && feed(d, wire, empty, f) == 1 && f.size == 0 && f.kind == 2);
}
