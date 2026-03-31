#ifndef __CQC_H__
#define __CQC_H__

#include "Trajectory.h"
#include <stdint.h>
#include <cmath>

struct SegmentParameters
{
    float q;
    float cubic_a[2];
    float cubic_b[2];
    float cubic_c[2];
    float cubic_d[2];
    float quintic_a;
    float quintic_b;
    float quintic_c;
    float quintic_d;
    float quintic_e;
    float quintic_f;
};

template <uint16_t capacity>
class CQC_ : public TrajectoryPlanner
{
private:
    TrajectoryPointArray points;
    SegmentParameters params[capacity];
    float clamp[capacity];
    static constexpr float quintic_proportion = 0.7;
    uint16_t size;
    bool computed;
    uint32_t start_tick;
    uint8_t segment_index;
    uint8_t local_index;
    float local_t;
    static constexpr uint32_t epsilon = 1;

public:
    explicit CQC_() : points(capacity), computed(false), segment_index(0), local_index(0), local_t(0) { free = true; }
    ~CQC_() override {}
    void reset() override
    {
        computed = false;
        free = true;
        segment_index = 0;
        local_index = 0;
        local_t = 0;
        points.clear();
    }
    void receive(TrajectoryPointArray _points, float a) override
    {
        if (_points.size() < 1)
            return;
        points.clear();
        points.add(0, a);
        points.add(_points);
        computed = false;
        size = points.size();
    }
    void compute(uint32_t tick) override
    {
        size = points.size();
        if (size - 1 > capacity)
            return;
        free = false;
        computed = true;
        segment_index = 0;
        local_index = 0;
        local_t = 0;
        start_tick = tick;
        clamp[0] = clamp[size - 1] = 0;
        bool ski = points[0].a < points[1].a;
        for (int i = 0; i < size - 1; i++)
        {
            params[i].q = (points[i + 1].a - points[i].a) / ((points[i + 1].t - points[i].t) * 1e-3);
            if (i != 0)
            {
                if (ski != (points[i].a <= points[i + 1].a))
                {
                    clamp[i] = 0;
                    ski = !ski;
                }
                else
                {
                    if (fabs(params[i - 1].q + params[i].q) < 1e-5)
                        clamp[i] = 0;
                    else
                        clamp[i] = 2 * (params[i - 1].q * params[i].q) / (params[i - 1].q + params[i].q);
                }
            }
        }
        for (int i = 0; i < size - 1; i++)
        {
            float cubic_h = ((points[i + 1].t - points[i].t) * 1e-3) * (1 - quintic_proportion) / 2;
            float quintic_h = ((points[i + 1].t - points[i].t) * 1e-3) * quintic_proportion;
            float q1 = (2 * clamp[i] + params[i].q) / 3;
            params[i].cubic_a[0] = points[i].a;
            params[i].cubic_b[0] = clamp[i];
            params[i].cubic_c[0] = 0;
            params[i].cubic_d[0] = (q1 - clamp[i]) / (3 * pow(cubic_h, 2));
            float q2 = (2 * clamp[i + 1] + params[i].q) / 3;
            params[i].cubic_a[1] = points[i + 1].a - 2 * clamp[i + 1] * cubic_h / 3 - q2 * cubic_h / 3;
            params[i].cubic_b[1] = q2;
            params[i].cubic_c[1] = (clamp[i + 1] - q2) / cubic_h;
            params[i].cubic_d[1] = -(clamp[i + 1] - q2) / (3 * pow(cubic_h, 2));
            float quintic_m[2] = {2 * (q1 - clamp[i]) / cubic_h, 2 * (clamp[i + 1] - q2) / cubic_h};
            float quintic_y[2] = {(q1 - clamp[i]) * cubic_h / 3 + clamp[i] * cubic_h + points[i].a, params[i].cubic_a[1]};
            params[i].quintic_a = quintic_y[0];
            params[i].quintic_b = q1;
            params[i].quintic_c = quintic_m[0] / 2;
            params[i].quintic_d = (20 * quintic_y[1] - 20 * quintic_y[0] - (8 * q2 + 12 * q1) * quintic_h - (3 * quintic_m[0] - quintic_m[1]) * pow(quintic_h, 2)) / (2 * pow(quintic_h, 3));
            params[i].quintic_e = (30 * quintic_y[0] - 30 * quintic_y[1] + (14 * q2 + 16 * q1) * quintic_h + (3 * quintic_m[0] - 2 * quintic_m[1]) * pow(quintic_h, 2)) / (2 * pow(quintic_h, 4));
            params[i].quintic_f = (12 * quintic_y[1] - 12 * quintic_y[0] - (6 * q2 + 6 * q1) * quintic_h - (quintic_m[0] - quintic_m[1]) * pow(quintic_h, 2)) / (2 * pow(quintic_h, 5));
        }
    }
    bool get(uint32_t t, float *result) override
    {
        if (computed)
        {
            if ((t - start_tick) > points[size - 1].t)
            {
                free = true;
            }
            if (!free)
            {
                while (((t - start_tick) > points[segment_index + 1].t))
                    segment_index++;
                float cubic_h = (points[segment_index + 1].t - points[segment_index].t) * (1 - quintic_proportion) / 2;
                float quintic_h = (points[segment_index + 1].t - points[segment_index].t) * quintic_proportion;
                if ((t - start_tick) > points[segment_index].t + cubic_h + quintic_h)
                {
                    local_t = ((t - start_tick) - (points[segment_index].t + cubic_h + quintic_h)) * 1e-3;
                    local_index = 3;
                }
                else if ((t - start_tick) > points[segment_index].t + cubic_h)
                {
                    local_t = ((t - start_tick) - (points[segment_index].t + cubic_h)) * 1e-3;
                    local_index = 2;
                }
                else if ((t - start_tick) >= 0)
                {
                    local_t = ((t - start_tick) - (points[segment_index].t)) * 1e-3;
                    local_index = 1;
                }
                else
                {
                    local_index = 0;
                }
                float r;
                switch (local_index)
                {
                case 0:
                    r = points[0].a;
                    break;
                case 1:
                    r = params[segment_index].cubic_d[0] * local_t + params[segment_index].cubic_c[0];
                    r = r * local_t + params[segment_index].cubic_b[0];
                    r = r * local_t + params[segment_index].cubic_a[0];
                    break;
                case 2:
                    r = params[segment_index].quintic_f * local_t + params[segment_index].quintic_e;
                    r = r * local_t + params[segment_index].quintic_d;
                    r = r * local_t + params[segment_index].quintic_c;
                    r = r * local_t + params[segment_index].quintic_b;
                    r = r * local_t + params[segment_index].quintic_a;
                    break;
                case 3:
                    r = params[segment_index].cubic_d[1] * local_t + params[segment_index].cubic_c[1];
                    r = r * local_t + params[segment_index].cubic_b[1];
                    r = r * local_t + params[segment_index].cubic_a[1];
                    break;
                default:
                    break;
                }
                *result = r;
            }
            else
            {
                *result = points[size - 1].a;
            }
            return true;
        }
        return false;
    }
};

using CQC = CQC_<20>;

#endif
