#pragma once
#include <cstdint>
#include <cmath>

using fix16 = int32_t;

#define FIX(x) ((fix16)((x) * 65536.0f))
#define TO_FLOAT(x) ((float)(x) / 65536.0f)


#define MUL(a,b) ((fix16)(((int64_t)(a) * (b)) >> 16))
#define DIV(a,b) ((fix16)((((int64_t)(a) << 16) / (b))))

inline float dbg(fix16 v) {
    return TO_FLOAT(v);
}

inline fix16 clamp(fix16 v, fix16 lo, fix16 hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline fix16 abs(fix16 v) {
    return v < 0 ? -v : v;
}

inline fix16 sign(fix16 v) {
    return (v > 0) ? FIX(1) : FIX(-1);
}
