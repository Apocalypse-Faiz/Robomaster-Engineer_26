#pragma once
#include "internal.h"

/**
 * @brief 驱动器通用组件的命名空间
 * @details 各种类元件的通用组件，直接面向驱动器的搭建
 *
 */
namespace Actuator_GeneralComponents
{

    /**
     * @brief Encoder 类的前置声明
     * @details 使得在向下接口 Encoder_ 里可以设置 Encoder& 类型的参数
     *          这样 Encoder_ 能够向上调用 Encoder 类的公有成员（有时候
     *          是必要的）
     *
     */
    class Encoder;

    /**
     * @brief 编码器类型枚举
     *
     */
    enum class EncoderType
    {
        Absolute = 0, // 绝对式
        Incremental   // 增量式
    };

    /**
     * @brief 编码器多圈角度跟踪方式
     *
     */
    enum class EncoderAngleTrackingPattern
    {
        Physical = 0, // 物理实现
        Software      // 软件实现
    };

    /**
     * @brief 编码器状态
     *
     */
    enum class EncoderStatus
    {
        Ready = 0, // 就绪
        Error      // 错误报警
    };

    /**
     * @brief Encoder 类的向下接口，面向元件驱动
     * @details 用纯虚函数定义函数接口，用 protected 的成员变量充当数据
     *          接口（友元使得在 Encoder 类中可以通过 protected 提供的
     *          数据接口向下访问）
     *
     */
    class Encoder_
    {
        friend Encoder;

    public:
        virtual void reset() = 0; // 编码器复位（注：不是初始化接口，初始化应该在构造函数内完成）
        virtual void read() = 0;  // 读取编码器数据
        virtual ~Encoder_() {}    // 构析函数
    protected:
        EncoderType type;                                   // 编码器类型
        EncoderAngleTrackingPattern angle_tracking_pattern; // 编码器多圈角度跟踪方式
        EncoderStatus status;                               // 编码器运行状态
        float min_angle;                                    // 编码器支持的最小角度
        float max_angle;                                    // 编码器支持的最大角度
        float angle;                                        // 编码器原始角度
        float total_angle;                                  // 编码器总角度
    };

    /**
     * @brief 编码器通用类
     * @details 通过向下接口来向下访问和调用，并提供向上接口和用户接口，
     *          数据的实体在继承了 Encoder_ 的驱动类中
     *
     */
    class Encoder
    {
    private:
        Encoder_ *encoder_ = nullptr; // 向下接口
    public:
        Encoder(Encoder_ *encoder_) // 构造函数，绑定驱动类
        {
            this->encoder_ = encoder_;
            // std::cout << "Encoder attached at\"" << encoder_ << "\"" << std::endl;
            this->encoder_->reset();
        }
        void update() // 更新编码器数据
        {
            encoder_->read();
            // std::cout << "Encoder updated" << std::endl;
        }
        float getAngle() // 读取编码器角度
        {
            return encoder_->angle;
        }
        float getTotalAngle() // 读取编码器总角度
        {
            return encoder_->total_angle;
        }
        EncoderStatus getStatus() // 读取编码器状态
        {
            return encoder_->status;
        }
        ~Encoder()
        {
        }
    };

}
