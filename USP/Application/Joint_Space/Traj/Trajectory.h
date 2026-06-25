#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "main.h"
#include <vector>

/**
 * @brief �켣��
 */
struct TrajectoryPoint
{
    uint32_t t; // �Ǵӿ�ʼִ��
    float a;
};

/**
 * @brief Ŀ��켣���飬�������㣬��������и���������
 */
class TrajectoryPointArray
{
public:
    explicit TrajectoryPointArray(uint16_t capacity_) : capacity(capacity_)
    {
        trajectory_points.reserve(capacity);
        tick_counter = 0;
    }

    // ����
    TrajectoryPointArray(const TrajectoryPointArray &other)
    {
        this->capacity = other.capacity;
        this->trajectory_points = other.trajectory_points;
    }

    ~TrajectoryPointArray()
    {
    }

    // �������캯��
    TrajectoryPointArray &operator=(const TrajectoryPointArray &other)
    {
        if (this != &other)
        {
            this->capacity = other.capacity;
            this->trajectory_points = other.trajectory_points;
        }
        return *this;
    }

    // ����һ����
    /**
     * @brief
     * @param dt deta��t������freertos��ʵ��ʱ�䣬�ǹ켣���м�����ʱ��
     * @param Ŀ��Ƕ�
     *
     */
    void add(uint32_t dt, float a)
    {
        if (dt == 0 && trajectory_points.size() > 0) // ��һ������ò�ҪΪ0
            return;
        if (trajectory_points.size() < capacity)
        {
            tick_counter += dt;
            TrajectoryPoint tmp;
            tmp.t = tick_counter;
            tmp.a = a;
            trajectory_points.push_back(tmp);
        }
        // �������capacity�Ǿ��޷������ˣ�
    }

    // ����һ��켣��
    void add(const TrajectoryPointArray &_trajectory_point_array)
    {
        TrajectoryPoint tmp;
        if ((this->capacity >= this->size() + _trajectory_point_array.size()) && (_trajectory_point_array.size() > 0))
        {
            uint32_t last_tick = (trajectory_points.size() == 0) ? (0) : (trajectory_points[trajectory_points.size() - 1].t); // ����ԭ�켣����û�й켣�㣬��Ҫ��֤��һ���켣���ʱ�����Ϊ0
            for (int i = 0; i < _trajectory_point_array.size(); i++)
            {
                tmp.t = _trajectory_point_array[i].t + last_tick;
                tmp.a = _trajectory_point_array[i].a;
                trajectory_points.push_back(tmp);
            }
            tick_counter = last_tick + _trajectory_point_array[_trajectory_point_array.size() - 1].t;
        }
    }

    uint16_t size() const
    {
        return trajectory_points.size();
    }

    const TrajectoryPoint &operator[](uint16_t index) const
    {
        if (index < trajectory_points.size())
            return trajectory_points[index];
        else
            for (;;)
                ;
    }

    void clear()
    {
        trajectory_points.clear();
        tick_counter = 0;
    }

private:
    uint32_t tick_counter; // ֻ��Ϊ��add���ã�
    uint16_t capacity;
    std::vector<TrajectoryPoint> trajectory_points;
};

class TrajectoryPlanning;

/**
 * @brief <freertos> ����Ĺ滮���ӿڣ��������Բ�ֵ�����β�ֵ��
 */
struct TrajectoryPlanner
{
    friend TrajectoryPlanning;

public:
    virtual void reset() = 0;
    virtual void receive(TrajectoryPointArray, float a) = 0;
    virtual void compute(uint32_t t) = 0;
    virtual bool get(uint32_t t, float *result) = 0;
    // 获取 t 时刻的目标速度/加速度 (多项式求导). 默认实现返回 false (未支持的规划器输出 0).
    virtual bool getDerivative(uint32_t t, float *vel, float *acc)
    {
        (void)t;
        if (vel) *vel = 0.0f;
        if (acc) *acc = 0.0f;
        return false;
    }
    virtual ~TrajectoryPlanner() {}

protected:
    bool free; // �Ƿ����
};

/**
 * @brief <freertos> ����켣�滮��
 */
class TrajectoryPlanning
{
private:
    TrajectoryPlanner *trajectory_planner = nullptr;
    TrajectoryPointArray *trajectory_point_array;
    float target;

public:
    explicit TrajectoryPlanning(TrajectoryPlanner *trajectory_planner_) : trajectory_planner(trajectory_planner_)
    {
    }

    ~TrajectoryPlanning()
    {
        delete trajectory_planner;
    }

    /**
     * @brief �󶨼��ع켣����
     */
    bool attach(TrajectoryPointArray *_p)
    {
        if (_p)
        {
            trajectory_point_array = _p;
            return true;
        }
        return false;
    }

    /**
     * @brief ����
     *
     */
    bool compute(uint32_t tick, float a)
    {
        if (trajectory_point_array && trajectory_planner)
        {
            if (trajectory_planner->free)
            {
                trajectory_planner->reset(); // �����һ��
                trajectory_planner->receive(*trajectory_point_array, a);
                trajectory_planner->compute(tick);
                return true;
            }
            else
            {
            }
        }
        return false;
    }

    void reset(float last_target)
    {
        this->target = last_target;
        if (trajectory_planner)
        {
            trajectory_planner->reset();
            return;
        }
    }

    bool get(uint32_t t, float *result)
    {
        bool ok = false;
        if (trajectory_point_array && trajectory_planner)
        {
            ok = trajectory_planner->get(t, &target);
            *result = target;
        }
        return ok;
    }

    /**
     * @brief 获取 t 时刻轨迹的目标速度与目标加速度
     * @param vel 输出目标速度 (单位/秒)
     * @param acc 输出目标加速度 (单位/秒²)
     */
    bool getDerivative(uint32_t t, float *vel, float *acc)
    {
        if (trajectory_point_array && trajectory_planner)
            return trajectory_planner->getDerivative(t, vel, acc);
        return false;
    }

    bool free()
    {
        return trajectory_planner->free;
    }
};

#endif
