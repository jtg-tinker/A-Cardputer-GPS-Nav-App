#pragma once
#include <stddef.h>
#include <stdint.h>

#include "streets.h"

namespace nav {
namespace pbf {

// Sequential protobuf reader over a ByteSource range, with a small read-ahead buffer.
class Cursor {
public:
    Cursor(ByteSource &s, size_t pos, size_t end) : s_(s), pos_(pos), end_(end) {}
    bool done() const { return pos_ >= end_; }

    bool byte(uint8_t &b) {
        if (pos_ >= end_) return false;
        if (pos_ < bufStart_ || pos_ >= bufStart_ + bufLen_) {
            size_t n = end_ - pos_;
            if (n > sizeof buf_) n = sizeof buf_;
            if (!s_.read(pos_, buf_, n)) return false;
            bufStart_ = pos_;
            bufLen_ = n;
        }
        b = buf_[pos_ - bufStart_];
        ++pos_;
        return true;
    }
    bool varint(uint64_t &v) {
        v = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            uint8_t b;
            if (!byte(b)) return false;
            v |= uint64_t(b & 0x7f) << shift;
            if (!(b & 0x80)) return true;
        }
        return false;
    }
    bool skip(size_t n) {
        if (n > end_ - pos_) return false;
        pos_ += n;
        return true;
    }
    bool field(uint32_t &num, uint32_t &wire) {
        uint64_t k;
        if (!varint(k)) return false;
        num = uint32_t(k >> 3);
        wire = uint32_t(k & 7);
        return true;
    }
    bool lenDelim(size_t &start, size_t &len) {
        uint64_t l;
        if (!varint(l) || l > end_ - pos_) return false;
        start = pos_;
        len = size_t(l);
        pos_ += len;
        return true;
    }
    bool skipValue(uint32_t wire) {
        uint64_t v;
        size_t s, l;
        switch (wire) {
            case 0: return varint(v);
            case 1: return skip(8);
            case 2: return lenDelim(s, l);
            case 5: return skip(4);
            default: return false;
        }
    }

private:
    ByteSource &s_;
    size_t pos_, end_;
    uint8_t buf_[96];
    size_t bufStart_ = 0, bufLen_ = 0;
};

}  // namespace pbf
}  // namespace nav
