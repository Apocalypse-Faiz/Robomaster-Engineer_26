#ifndef __LINEAR_H__
#define __LINEAR_H__

#include "Trajectory.h"
#include "main.h"

/**
 * @brief  线性插值 轨迹规划器
 * @details 独立运行的实体，绑定一个 Trajectory
 *
 */
class Linear : public TrajectoryPlanner
{
private:
    float *k;                                     // 直线系数
    float *b;                                     // 直线系数
    TrajectoryPointArray *trajectory_point_array; // 传入轨迹规划器的轨迹点组
    uint16_t capacity;                            // 轨迹点数量上限
    uint16_t size;                                // 轨迹点数量
    bool computed;                                // 轨迹规划器里的轨迹点是否已经进行规划计算
    uint32_t start_tick;                          // 记录轨迹规划开始的时间戳
    uint8_t curve_index;                          // 当前执行规划的曲线引索
public:
    /**
     * @brief 构造函数
     *
     * @param capacity 轨迹点数量上限
     */
    explicit Linear(uint16_t capacity)
    {
        this->capacity = capacity;
        computed = false;
        free = true;
        curve_index = 0;
        k = new float[capacity];
        b = new float[capacity];
        trajectory_point_array = new TrajectoryPointArray(capacity);
    }
    /**
     * @brief 构析函数
     * @details 因为有动态存储的成员，所以是必需的
     *
     */
    ~Linear() override
    {
        delete[] k;
        delete[] b;
        delete trajectory_point_array;
    }

    /**
     * @brief 重置轨迹规划器
     *
     */
    void reset() override
    {
        computed = false;
        free = true;
        curve_index = 0;
        trajectory_point_array->clear();
        for (int i = 0; i < capacity; i++)
        {
            k[i] = 0;
            b[i] = 0;
        }
    }

    /**
     * @brief 传入轨迹点组
     *
     * @param trajectory_point_array_ 传入的轨迹点组类类型参数
     * @param a 上一帧的轨迹点，即上一帧的规划的目标关节角
     */
    void receive(TrajectoryPointArray trajectory_point_array_, float a) override
    {
        trajectory_point_array->clear();
        trajectory_point_array->add(0, a); // 左边界条件
        trajectory_point_array->add(trajectory_point_array_);
        computed = false;
        size = trajectory_point_array->size();
    }
    /**
     * @brief 一次规划计算，并将 tick 作为本次轨迹规划的起始时间
     *
     * @param tick 时间戳，一般为调用时刻的时间戳
     */
    void compute(uint32_t tick) override
    {
        if (trajectory_point_array->size() > 1)
        {
            size = trajectory_point_array->size();
            free = false;
            computed = true;
            curve_index = 0;
            start_tick = tick;
            for (int i = 0; i < size - 1; i++)
            {
                float y1 = (*trajectory_point_array)[i].a,
                      y2 = (*trajectory_point_array)[i + 1].a,
                      x1 = (*trajectory_point_array)[i].t,
                      x2 = (*trajectory_point_array)[i + 1].t;
                k[i] = (y2 - y1) / (x2 - x1);
                b[i] = y1;
            }
            return;
        }
    }
    /**
     * @brief 获取 t 时刻的插值，即规划的目标关节角
     *
     * @param t 时间戳
     * @param result 指向插值结果存放变量的指针
     * @return true 正常
     * @return false 此时无法插值
     */
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
                while (((t - start_tick) > (*trajectory_point_array)[curve_index + 1].t)) // 一直执行到相应的函数区间
                    curve_index++;
                *result = k[curve_index] * (t - (*trajectory_point_array)[curve_index].t - start_tick) + b[curve_index];
            }
            else // 最后一次做完后保持
            {
                *result = (*trajectory_point_array)[size - 1].a;
            }
            return true;
        }
        return false;
    }
};

#endif