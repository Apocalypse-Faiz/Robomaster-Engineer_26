#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "main.h"
#include <vector>

/**
 * @brief 轨迹点
 */
struct TrajectoryPoint
{
    uint32_t t; // 是从开始执行
    float a;
};

/**
 * @brief 目标轨迹点组，这就是描点，拟合曲线中给的条件点
 */
class TrajectoryPointArray
{
public:
    explicit TrajectoryPointArray(uint16_t capacity_) : capacity(capacity_)
    {
        trajectory_points.reserve(capacity);
        tick_counter = 0;
    }

    // 拷贝
    TrajectoryPointArray(const TrajectoryPointArray &other)
    {
        this->capacity = other.capacity;
        this->trajectory_points = other.trajectory_points;
    }

    ~TrajectoryPointArray()
    {
    }

    // 拷贝构造函数
    TrajectoryPointArray &operator=(const TrajectoryPointArray &other)
    {
        if (this != &other)
        {
            this->capacity = other.capacity;
            this->trajectory_points = other.trajectory_points;
        }
        return *this;
    }

    // 加入一个点
    /**
     * @brief
     * @param dt deta（t）不是freertos的实际时间，是轨迹点中间的相对时间
     * @param 目标角度
     *
     */
    void add(uint32_t dt, float a)
    {
        if (dt == 0 && trajectory_points.size() > 0) // 第一个点最好不要为0
            return;
        if (trajectory_points.size() < capacity)
        {
            tick_counter += dt;
            TrajectoryPoint tmp;
            tmp.t = tick_counter;
            tmp.a = a;
            trajectory_points.push_back(tmp);
        }
        // 如果超过capacity那就无法加入了？
    }

    // 加入一组轨迹点
    void add(const TrajectoryPointArray &_trajectory_point_array)
    {
        TrajectoryPoint tmp;
        if ((this->capacity >= this->size() + _trajectory_point_array.size()) && (_trajectory_point_array.size() > 0))
        {
            uint32_t last_tick = (trajectory_points.size() == 0) ? (0) : (trajectory_points[trajectory_points.size() - 1].t); // 假如原轨迹点组没有轨迹点，需要保证第一个轨迹点的时间戳不为0
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
    uint32_t tick_counter; // 只是为了add有用，
    uint16_t capacity;
    std::vector<TrajectoryPoint> trajectory_points;
};

class TrajectoryPlanning;

/**
 * @brief <freertos> 具体的规划器接口，比如线性插值，三次插值等
 */
struct TrajectoryPlanner
{
    friend TrajectoryPlanning;

public:
    virtual void reset() = 0;
    virtual void receive(TrajectoryPointArray, float a) = 0;
    virtual void compute(uint32_t t) = 0;
    virtual bool get(uint32_t t, float *result) = 0;
    virtual ~TrajectoryPlanner() {}

protected:
    bool free; // 是否空闲
};

/**
 * @brief <freertos> 抽象轨迹规划器
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
     * @brief 绑定加载轨迹点组
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
     * @brief 进行
     *
     */
    bool compute(uint32_t tick, float a)
    {
        if (trajectory_point_array && trajectory_planner)
        {
            if (trajectory_planner->free)
            {
                trajectory_planner->reset(); // 清空上一次
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

    bool free()
    {
        return trajectory_planner->free;
    }
};

#endif
