#ifndef VECMATH_H
#define VECMATH_H

#include <cmath>

template <typename ctype>
struct gvec2
{
    ctype X, Y;

    constexpr explicit gvec2() = default;
    constexpr explicit gvec2(ctype Uniform) : X(Uniform), Y(Uniform) {}
    constexpr explicit gvec2(ctype _X, ctype _Y) : X(_X), Y(_Y) {}
};


template <typename ctype>
struct gvec3
{
    ctype X, Y, Z;

    constexpr explicit gvec3() = default;
    constexpr explicit gvec3(ctype All) : X(All), Y(All), Z(All) {}
    constexpr explicit gvec3(ctype _X, ctype _Y, ctype _Z) : X(_X), Y(_Y), Z(_Z) {}
};


template <typename ctype>
struct gvec4
{
    ctype X, Y, Z, W;

    constexpr explicit gvec4() = default;
    constexpr explicit gvec4(ctype All) : X(All), Y(All), Z(All), W(All) {}
    constexpr explicit gvec4(ctype _X, ctype _Y, ctype _Z, ctype _W) : X(_X), Y(_Y), Z(_Z), W(_W) {}
};

using vec2 = gvec2<float>;
using vec3 = gvec3<float>;
using vec4 = gvec4<float>;
using ivec2 = gvec2<int>;
using ivec3 = gvec3<int>;
using ivec4 = gvec4<int>;

using uvec2 = gvec2<unsigned int>;
using uvec3 = gvec3<unsigned int>;
using uvec4 = gvec4<unsigned int>;

using bvec2 = gvec2<bool>;
using bvec3 = gvec3<bool>;
using bvec4 = gvec4<bool>;

// {{{ Boolean horizontal functions (vec2)

constexpr bool
All(bvec2 A)
{
    return A.X && A.Y;
}

constexpr bool
Any(bvec2 A)
{
    return A.X || A.Y;
}

// }}}


// {{{ Boolean horizontal functions (vec3)

constexpr bool
All(bvec3 A)
{
    return A.X && A.Y && A.Z;
}

constexpr bool
Any(bvec3 A)
{
    return A.X || A.Y || A.Z;
}

// }}}


// {{{ Boolean horizontal functions (vec4)

constexpr bool
All(bvec4 A)
{
    return A.X && A.Y && A.Z && A.W;
}

constexpr bool
Any(bvec4 A)
{
    return A.X || A.Y || A.Z || A.W;
}

// }}}


// {{{ Special boolean functions (vec2)
constexpr bvec2
operator&&(bvec2 L, bvec2 R)
{
    return bvec2{ L.X && R.X, L.Y && R.Y };
}

constexpr bvec2
operator||(bvec2 L, bvec2 R)
{
    return bvec2{ L.X || R.X, L.Y || R.Y };
}

constexpr bvec2
operator!(bvec2 L)
{
    return bvec2{ !L.X, !L.Y};
}
// }}}


// {{{ Special boolean functions (vec3)
constexpr bvec3
operator&&(bvec3 L, bvec3 R)
{
    return bvec3{ L.X && R.X, L.Y && R.Y, L.Z && R.Z };
}

constexpr bvec3
operator||(bvec3 L, bvec3 R)
{
    return bvec3{ L.X || R.X, L.Y || R.Y, L.Z || R.Z };
}

constexpr bvec3
operator!(bvec3 L)
{
    return bvec3{ !L.X, !L.Y, !L.Z};
}
// }}}


// {{{ Special boolean functions (vec3)
constexpr bvec4
operator&&(bvec4 L, bvec4 R)
{
    return bvec4{ L.X && R.X, L.Y && R.Y, L.Z && R.Z, L.W && R.W };
}

constexpr bvec4
operator||(bvec4 L, bvec4 R)
{
    return bvec4{ L.X || R.X, L.Y || R.Y, L.Z || R.Z, L.W || R.W };
}

constexpr bvec4
operator!(bvec4 L)
{
    return bvec4{ !L.X, !L.Y, !L.Z, !L.W};
}
// }}}


// {{{ Comparison operators (vec2)
template<typename c>
constexpr bvec2 Equal(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X == R.X, L.Y == R.Y };
}

template<typename c>
constexpr bvec2 NotEqual(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X != R.X, L.Y != R.Y };
}

template<typename c>
constexpr bvec2 LessThan(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X < R.X, L.Y < R.Y };
}

template<typename c>
constexpr bvec2 LessThanEqual(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X <= R.X, L.Y <= R.Y };
}

template<typename c>
constexpr bvec2 GreaterThan(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X > R.X, L.Y > R.Y };
}

template<typename c>
constexpr bvec2 GreaterThanEqual(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X >= R.X, L.Y >= R.Y };
}

// }}}


// {{{ Comparison operators (vec3)
template<typename c>
constexpr bvec3 Equal(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X == R.X, L.Y == R.Y, L.Z == R.Z };
}

template<typename c>
constexpr bvec3 NotEqual(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X != R.X, L.Y != R.Y, L.Z != R.Z };
}

template<typename c>
constexpr bvec3 LessThan(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X < R.X, L.Y < R.Y, L.Z < R.Z };
}

template<typename c>
constexpr bvec3 LessThanEqual(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X <= R.X, L.Y <= R.Y, L.Z <= R.Z };
}

template<typename c>
constexpr bvec3 GreaterThan(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X > R.X, L.Y > R.Y, L.Z > R.Z };
}

template<typename c>
constexpr bvec3 GreaterThanEqual(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X >= R.X, L.Y >= R.Y, L.Z >= R.Z };
}

// }}}


// {{{ Comparison operators (vec4)
template<typename c>
constexpr bvec4 Equal(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X == R.X, L.Y == R.Y, L.Z == R.Z, L.W == R.W };
}

template<typename c>
constexpr bvec4 NotEqual(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X != R.X, L.Y != R.Y, L.Z != R.Z, L.W != R.W };
}

template<typename c>
constexpr bvec4 LessThan(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X < R.X, L.Y < R.Y, L.Z < R.Z, L.W < R.W };
}

template<typename c>
constexpr bvec4 LessThanEqual(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X <= R.X, L.Y <= R.Y, L.Z <= R.Z, L.W <= R.W };
}

template<typename c>
constexpr bvec4 GreaterThan(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X > R.X, L.Y > R.Y, L.Z > R.Z, L.W > R.W };
}

template<typename c>
constexpr bvec4 GreaterThanEqual(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X >= R.X, L.Y >= R.Y, L.Z >= R.Z, L.W >= R.W };
}

// }}}


// {{{ Vector/Linear Algebra functions
template <typename c> constexpr c
Dot(gvec2<c> L, gvec2<c> R)
{
    return L.X*R.X + L.Y*R.Y;
}

template <typename c> constexpr c
Dot(gvec3<c> L, gvec3<c> R)
{
    return L.X*R.X + L.Y*R.Y + L.Z*R.Z;
}

template <typename c> constexpr c
Dot(gvec4<c> L, gvec4<c> R)
{
    return L.X*R.X + L.Y*R.Y + L.Z*R.Z + L.W*R.W;
}

template <typename c> constexpr c
Length(gvec2<c> V)
{
    return std::sqrt(V.X*V.X, V.Y*V.Y, V.Z*V.Z);
}

template <typename c> constexpr c
Normalize(gvec2<c> L, gvec2<c> R)
{

}

// }}}



#endif

