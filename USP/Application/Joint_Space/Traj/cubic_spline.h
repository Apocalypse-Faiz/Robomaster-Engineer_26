#ifndef __CUBICSPLINE_H__
#define __CUBICSPLINE_H__

#include "Trajectory.h"

#define CUBICSPLINE_NORMAL // 自然边界条件
// #define CUBICSPLINE_CLAMP    // 夹持边界条件

/*
    CUBICSPLINE_NORMAL：自然边界条件，要求轨迹点数（包含第一个轨迹点）≥ 2
    CUBICSPLINE_CLAMP：夹持边界条件（伪），要求轨迹点数（包含第一个轨迹点）≥4
*/
class CubicSpline : public TrajectoryPlanner
{
private:
    float *a;
    float *b;
    float *c;
    float *d;
    float *h;
    float *alpha;
    float *l;
    float *mu;
    float *z;
    TrajectoryPointArray *trajectory_point_array; // 内部存储
    uint16_t size;
    uint16_t capacity;
    bool computed;
    uint32_t start_tick;
    uint8_t curve_index;
    const uint32_t epsilon = 1;

public:
    explicit CubicSpline(uint16_t capacity)
    {
        this->capacity = capacity;
        computed = false;
        free = true;
        curve_index = 0;
        a = new float[capacity];
        b = new float[capacity];
        c = new float[capacity];
        d = new float[capacity];
        h = new float[capacity];
        l = new float[capacity];
        mu = new float[capacity];
        z = new float[capacity];
        alpha = new float[capacity];
        trajectory_point_array = new TrajectoryPointArray(capacity);
    }
    ~CubicSpline() override
    {
        delete[] a;
        delete[] b;
        delete[] c;
        delete[] d;
        delete[] h;
        delete[] l;
        delete[] mu;
        delete[] z;
        delete[] alpha;
        delete trajectory_point_array;
    }
    void reset() override
    {
        computed = false;
        free = true;
        curve_index = 0;
        trajectory_point_array->clear();
    }
    void receive(TrajectoryPointArray trajectory_point_array_, float a) override
    {
        if (trajectory_point_array_.size() < 1)
            return;
        trajectory_point_array->clear();
        trajectory_point_array->add(0, a);
#ifdef CUBICSPLINE_CLAMP
        trajectory_point_array->add(epsilon, a);
#endif
        trajectory_point_array->add(trajectory_point_array_);
#ifdef CUBICSPLINE_CLAMP
        trajectory_point_array->add(epsilon, trajectory_point_array_[trajectory_point_array_.size() - 1].a);
#endif
        computed = false;
        size = trajectory_point_array->size();
    }
    void compute(uint32_t tick) override
    {
#ifdef CUBICSPLINE_CLAMP
        if (trajectory_point_array->size() > 3)
#endif
#ifdef CUBICSPLINE_NORMAL
            if (trajectory_point_array->size() > 1)
#endif
            {
                size = trajectory_point_array->size();
                free = false;
                computed = true;
                curve_index = 0;
                start_tick = tick;
                for (int i = 0; i < trajectory_point_array->size(); i++)
                {
                    a[i] = (*trajectory_point_array)[i].a;
                }
                for (int i = 0; i < trajectory_point_array->size() - 1; i++)
                {
                    h[i] = (*trajectory_point_array)[i + 1].t - (*trajectory_point_array)[i].t;
                }
                for (int i = 1; i < trajectory_point_array->size() - 1; i++)
                {
                    alpha[i] = 3 * (a[i + 1] - a[i]) / h[i] - 3.0f * (a[i] - a[i - 1]) / h[i - 1];
                }
                l[0] = 1;
                mu[0] = 0;
                z[0] = 0;
                for (int i = 1; i < trajectory_point_array->size() - 1; i++)
                {
                    l[i] = 2.0f * ((*trajectory_point_array)[i + 1].t - (*trajectory_point_array)[i - 1].t) - h[i - 1] * mu[i - 1];
                    mu[i] = h[i] / l[i];
                    z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
                }
                l[trajectory_point_array->size() - 1] = 1;
                z[trajectory_point_array->size() - 1] = 0;
                c[trajectory_point_array->size() - 1] = 0;
                for (int j = trajectory_point_array->size() - 2; j >= 0; j--)
                {
                    c[j] = z[j] - mu[j] * c[j + 1];
                    b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0f * c[j]) / 3.0f;
                    d[j] = (c[j + 1] - c[j]) / (3.0f * h[j]);
                }
                return;
            }
    }
    bool get(uint32_t t, float *result) override
    {
        if (computed)
        {
            if ((t - start_tick) > (*trajectory_point_array)[size - 1].t)
            {
                free = true;
            }
            if (!free)
            {
                while (((t - start_tick) > (*trajectory_point_array)[curve_index + 1].t))
                    curve_index++;
                *result = a[curve_index] + b[curve_index] * (t - (*trajectory_point_array)[curve_index].t - start_tick) + c[curve_index] * (t - (*trajectory_point_array)[curve_index].t - start_tick) * (t - (*trajectory_point_array)[curve_index].t - start_tick) + d[curve_index] * (t - (*trajectory_point_array)[curve_index].t - start_tick) * (t - (*trajectory_point_array)[curve_index].t - start_tick) * (t - (*trajectory_point_array)[curve_index].t - start_tick);
            }
            else
            {
                *result = (*trajectory_point_array)[size - 1].a;
            }
            return true;
        }
        return false;
    }
};

#endif
