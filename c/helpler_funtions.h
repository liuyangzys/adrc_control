#pragma once

#include <float.h>
#include <stdbool.h>

#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

static inline int sign(float val)
{
    return (FLT_EPSILON < val) - (val < FLT_EPSILON);
}

static inline float wrap_float(float x, float low, float high)
{
    if (low <= x && x < high)
    {
        return x;
    }

    const float range = high - low;
    const float inv_range = 1.0f / range;
    const float num_wraps = floorf((x - low) * inv_range);
    return x - range * num_wraps;
}

static inline float wrap_pi(float x)
{
    return wrap_float(x, -M_PI, M_PI);
}

static inline float wrap_180(float x)
{
    return wrap_float(x, -180.0f, 180.0f);
}

static inline float constrain_float(float x, float low, float high)
{
    if (x < low)
        return low;
    else if (x > high)
        return high;
    else
        return x;
}

static inline float rad_to_deg(float rad)
{
    return (rad * 180.0f / M_PI);
}

static inline float deg_to_rad(float deg)
{
    return (deg * M_PI / 180.0f);
}
