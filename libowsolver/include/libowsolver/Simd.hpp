#pragma once
#include <cmath>
#include <algorithm>


namespace simd {
    struct v4f {
        float x, y, z, w;
        v4f() : x(0), y(0), z(0), w(0) {}
        v4f(float x, float y, float z, float w=0) : x(x), y(y), z(z), w(w) {}
    };

    inline v4f zerof() {
        return v4f(0,0,0,0);
    }

    inline v4f splat(float s) {
        return v4f(s,s,s,s);
    }

    template<int I> inline v4f splat(const v4f& v);
    template<> inline v4f splat<0>(const v4f& v) {
        return splat(v.x);
    }

    template<> inline v4f splat<1>(const v4f& v) {
        return splat(v.y);
    }
    template<> inline v4f splat<2>(const v4f& v) {
        return splat(v.z);
    }
    template<> inline v4f splat<3>(const v4f& v) {
        return splat(v.w);
    }

    inline v4f operator+(const v4f& a, const v4f& b) {
        return {a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w};
    }

    inline v4f operator-(const v4f& a, const v4f& b) {
        return {a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w};
    }

    inline v4f operator*(const v4f& a, const v4f& b) {
        return {a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w};
    }

    inline v4f operator/(const v4f& a, const v4f& b) {
        return {a.x/b.x, a.y/b.y, a.z/b.z, a.w/b.w};
    }

    inline v4f max(const v4f& a, const v4f& b) {
        return {std::max(a.x,b.x), std::max(a.y,b.y), std::max(a.z,b.z), std::max(a.w,b.w)};
    }
    inline v4f min(const v4f& a, const v4f& b) {
        return {std::min(a.x,b.x), std::min(a.y,b.y), std::min(a.z,b.z), std::min(a.w,b.w)};
    }

    inline void storeSingle(float* dst, const v4f& v) {
        *dst = v.x;
    }
    inline v4f  loadSingle(const float* src) {
        return splat(*src);
    }

    inline float extractSlow(const v4f& v, int i) {
        switch(i) {
            case 0: return v.x; case 1: return v.y;
            case 2: return v.z; default: return v.w;
        }
    }

    inline v4f mulAdd(const v4f& a, const v4f& b, const v4f& c) {
        return a + b * c;
    }

    inline v4f form(float x, float y, float z=0, float w=0) {
        return {x,y,z,w};
    }

    inline v4f gatherX(const v4f& a, const v4f& b, const v4f& c, const v4f& d) {
        return {a.x, b.x, c.x, d.x};
    }

    // v4f_pod for use in unions
    using v4f_pod = v4f;

    inline v4f sumAcross3(const v4f& a, const v4f& b) {
        float sa = a.x + a.y + a.z;
        float sb = b.x + b.y + b.z;
        return form(sa, sb);
    }
    inline v4f sumAcross3(const v4f& a, const v4f& b, const v4f& c) {
        return form(a.x+a.y+a.z, b.x+b.y+b.z, c.x+c.y+c.z);
    }

    // permute
    template<int X, int Y, int Z, int W>
    inline v4f permute(const v4f& v) {
        float arr[4] = {v.x, v.y, v.z, v.w};
        return {arr[X], arr[Y], arr[Z], arr[W]};
    }

    inline v4f zipLow(const v4f& a, const v4f& b) {
        return {a.x, b.x, a.y, b.y};
    }

    template<int X, int Y, int Z, int W>
    inline v4f shuffle(const v4f& a, const v4f& b) {
        float aa[4] = {a.x, a.y, a.z, a.w};
        float bb[4] = {b.x, b.y, b.z, b.w};
        return {aa[X], aa[Y], bb[Z], bb[W]};
    }

    template<int A, int B, int C, int D>
    inline v4f select(const v4f& a, const v4f& b) {
        return {A >= 0 ? a.x : b.x, B >= 0 ? a.y : b.y, C >= 0 ? a.z : b.z, D >= 0 ? a.w : b.w};
    }

    template<int I>
    inline v4f replace(const v4f& target, const v4f& source) {
        v4f r = target;
        switch(I) {
            case 0: r.x = source.x; break;
            case 1: r.y = source.y; break;
            case 2: r.z = source.z; break;
            case 3: r.w = source.w; break;
        }
        return r;
    }

    inline void transpose(v4f& r0, v4f& r1, v4f& r2, v4f& r3, const v4f& a, const v4f& b, const v4f& c, const v4f& d) {
        r0 = {a.x, b.x, c.x, d.x};
        r1 = {a.y, b.y, c.y, d.y};
        r2 = {a.z, b.z, c.z, d.z};
        r3 = {a.w, b.w, c.w, d.w};
    }
    inline void transpose2x4(v4f& r0, v4f& r1, v4f& r2, v4f& r3, const v4f& a, const v4f& b) {
        r0 = splat<0>(a); r1 = splat<1>(a);
        r2 = splat<2>(a); r3 = splat<3>(a);
        // overwrite with b
        r0.y = b.x; r1.y = b.y; r2.y = b.z; r3.y = b.w;
    }
    inline void transpose3x4(v4f& r0, v4f& r1, v4f& r2, v4f& r3, const v4f& a, const v4f& b, const v4f& c) {
        r0 = {a.x, b.x, c.x, 0};
        r1 = {a.y, b.y, c.y, 0};
        r2 = {a.z, b.z, c.z, 0};
        r3 = {a.w, b.w, c.w, 0};
    }

} // namespace simd
