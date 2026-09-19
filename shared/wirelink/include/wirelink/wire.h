#pragma once

#include <wirelink/framing.h>

#include <optional>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>

namespace wirelink::codec {
    struct Reader {
        const uint8_t* buf;   // first byte to read from
        size_t len;           // how many bytes are readable
        size_t pos = 0;
        bool ok = true;

        void operator()(uint8_t& v)  { v = static_cast<uint8_t>(get(1)); }
        void operator()(uint16_t& v) { v = static_cast<uint16_t>(get(2)); }
        void operator()(uint32_t& v) { v = static_cast<uint32_t>(get(4)); }
        void operator()(uint64_t& v) { v = get(8); }
        void operator()(int32_t& v)  { v = static_cast<int32_t>(static_cast<uint32_t>(get(4))); }
        void operator()(float& v)    { uint32_t u = static_cast<uint32_t>(get(4)); std::memcpy(&v, &u, sizeof u); }
        void operator()(bytes::Payload& p) {
            uint8_t n = 0;
            (*this)(n);
            if (n > p.data.size()) { ok = false; p.len = 0; return; }
            p.len = n;
            for (size_t i = 0; i < p.len; ++i) (*this)(p.data[i]);
        }

        template <class T, size_t N>
        void operator()(List<T, N>& list) {
            (*this)(list.count);
            if (list.count > N) { ok = false; list.count = 0; return; }
            for (size_t i = 0; i < list.count; ++i) list.items[i].fields(*this);
        }

        template <class T> void operator()(T&) = delete;

    private:
        uint64_t get(size_t n) {
            if (!ok || pos + n > len) { ok = false; return 0; }
            uint64_t v = 0;
            for (size_t i = 0; i < n; ++i) v |= static_cast<uint64_t>(buf[pos++]) << (8 * i);
            return v;
        }
    };

    struct Writer {
        uint8_t* buf;   // first byte to write to
        size_t cap;     // how many bytes fit
        size_t pos = 0;
        bool ok = true;

        void operator()(uint8_t v)  { put(v, 1); }
        void operator()(uint16_t v) { put(v, 2); }
        void operator()(uint32_t v) { put(v, 4); }
        void operator()(uint64_t v) { put(v, 8); }
        void operator()(int32_t v)  { put(static_cast<uint32_t>(v), 4); }
        void operator()(float v)    { uint32_t u; std::memcpy(&u, &v, sizeof u); put(u, 4); }
        void operator()(bytes::Payload& p) {
            if (p.len > p.data.size()) { ok = false; return; }
            (*this)(static_cast<uint8_t>(p.len));
            for (size_t i = 0; i < p.len; ++i) (*this)(p.data[i]);
        }

        template <class T, size_t N>
        void operator()(List<T, N>& list) {
            if (list.count > N) { ok = false; return; }
            (*this)(list.count);
            for (size_t i = 0; i < list.count; ++i) list.items[i].fields(*this);
        }

        template <class T> void operator()(T) = delete;

    private:
        void put(uint64_t v, size_t n) {
            if (!ok || pos + n > cap) { ok = false; return; }
            for (size_t i = 0; i < n; ++i) buf[pos++] = static_cast<uint8_t>(v >> (8 * i));
        }
    };

    template <class T>
    bool pack(T msg, wirelink::Frame& frame) {
        Writer w{frame.payload.data.data(), frame.payload.data.size()};
        msg.fields(w);
        if (!w.ok) return false;
        frame.type = static_cast<uint8_t>(T::TYPE);
        frame.len = static_cast<uint8_t>(w.pos);
        return true;
    }

    template <class T>
    bool unpack(const wirelink::Frame& frame, T& out) {
        if (frame.type != static_cast<uint8_t>(T::TYPE)) return false;
        Reader r{frame.payload.data.data(), frame.len};
        out.fields(r);
        return r.ok && r.pos == frame.len;
    }

    // Read a message straight out of a byte buffer (no Frame involved).
    template <class T>
    bool unpack_bytes(const uint8_t* buf, size_t len, T& out) {
        Reader r{buf, len};
        out.fields(r);
        return r.ok && r.pos == len;
    }
}