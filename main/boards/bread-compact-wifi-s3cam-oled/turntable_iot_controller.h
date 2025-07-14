/*
 * 转盘IoT控制器 - 基于Thing接口实现
 * 参考Arduino代码实现双轴转盘控制
 * 支持水平旋转和垂直升降的IoT控制
 */

#pragma once

#include "../stepper_motor.h"
#include <memory>

#ifdef __cplusplus

// 前向声明IoT相关类（避免包含具体IoT头文件）
namespace iot {
    class Thing;
    class ParameterList;
}

/**
 * @brief 转盘IoT控制器配置
 */
struct TurntableIotConfig {
    // 水平电机配置（电机1）
    stepper_motor_config_t horizontal_motor_config;
    
    // 垂直电机配置（电机2）
    stepper_motor_config_t vertical_motor_config;
    
    // 步进电机减速比（参考Arduino代码中的stepper_i[]）
    float horizontal_gear_ratio = 2.2973f;  // 水平方向减速比
    float vertical_gear_ratio = 2.2973f;    // 垂直方向减速比
    
    // 特殊动作参数（参考Arduino PAN函数）
    float vertical_action_angle = -140.0f;  // PAN动作时垂直电机的动作角度
    uint32_t action_delay_ms = 1000;        // PAN动作延时
    
    // 小幅移动参数
    float small_move_angle = 10.0f;         // 小幅移动角度
    float relative_move_angle = 90.0f;      // 相对移动角度
    float vertical_move_angle = 130.0f;     // 垂直移动角度
};

/**
 * @brief 转盘IoT控制器类
 * 实现Thing接口，提供IoT控制功能
 */
class TurntableIotController {
public:
    /**
     * @brief 构造函数
     * @param config 转盘配置
     */
    explicit TurntableIotController(const TurntableIotConfig& config);
    
    /**
     * @brief 析构函数
     */
    ~TurntableIotController();
    
    /**
     * @brief 初始化转盘
     * @return bool 初始化是否成功
     */
    bool initialize();
    
    /**
     * @brief 获取Thing实例（用于IoT注册）
     * @return iot::Thing* Thing实例指针
     */
    iot::Thing* getThing() { return thing_; }
    
    // ========== Arduino串口指令对应的IoT方法 ==========
    
    /**
     * @brief 转台水平归零（对应串口指令'0'）
     * 转台水平方向转动位置设为0位置
     */
    bool resetHorizontalPosition();
    
    /**
     * @brief 转台垂直归零（对应串口指令'E'）
     * 转台竖直方向高度设为0位置
     */
    bool resetVerticalPosition();
    
    /**
     * @brief 转台水平相对正转（对应串口指令'+'）
     * 转台相对正转90°
     */
    bool rotateHorizontalPositive();
    
    /**
     * @brief 转台水平相对反转（对应串口指令'-'）
     * 转台相对反转90°
     */
    bool rotateHorizontalNegative();
    
    /**
     * @brief 转台上升（对应串口指令'z'）
     */
    bool moveVerticalUp();
    
    /**
     * @brief 转台下降（对应串口指令'f'）
     */
    bool moveVerticalDown();
    
    /**
     * @brief 转台垂直小幅下降（对应串口指令'F'）
     * 转台下降一小段
     */
    bool moveVerticalSmallDown();
    
    /**
     * @brief PAN到1号盘（对应串口指令'A'）
     * 转台归零 - 0度位置
     */
    bool panToPosition1();
    
    /**
     * @brief PAN到2号盘（对应串口指令'B'）
     * 转台转到坐标+90°
     */
    bool panToPosition2();
    
    /**
     * @brief PAN到3号盘（对应串口指令'C'）
     * 转台转到坐标+180°
     */
    bool panToPosition3();
    
    /**
     * @brief PAN到4号盘（新增）
     * 转台转到坐标+270°
     */
    bool panToPosition4();
    
    /**
     * @brief 通用PAN动作（支持自定义角度）
     * @param horizontal_angle 水平目标角度
     */
    bool performPanAction(float horizontal_angle);
    
    /**
     * @brief 获取水平电机当前角度
     * @return float 当前角度
     */
    float getHorizontalAngle();
    
    /**
     * @brief 获取垂直电机当前角度
     * @return float 当前角度
     */
    float getVerticalAngle();
    
    /**
     * @brief 检查是否正在运动
     * @return bool 是否在运动
     */
    bool isMoving();
    
    /**
     * @brief 停止所有运动
     * @return bool 操作是否成功
     */
    bool stopAll();

private:
    TurntableIotConfig config_;
    std::unique_ptr<StepperMotor> horizontal_motor_;  // 水平电机（电机1）
    std::unique_ptr<StepperMotor> vertical_motor_;    // 垂直电机（电机2）
    iot::Thing* thing_;                               // IoT Thing实例
    bool initialized_;
    
    /**
     * @brief 初始化IoT Thing并注册属性和方法
     */
    void initializeThing();
    
    /**
     * @brief 角度转换为步数（考虑减速比）
     * @param angle 角度
     * @param gear_ratio 减速比
     * @param steps_per_rev 每转步数
     * @return 步数
     */
    uint32_t angleToSteps(float angle, float gear_ratio, uint32_t steps_per_rev);
    
    /**
     * @brief 步数转换为角度（考虑减速比）
     * @param steps 步数
     * @param gear_ratio 减速比
     * @param steps_per_rev 每转步数
     * @return 角度
     */
    float stepsToAngle(uint32_t steps, float gear_ratio, uint32_t steps_per_rev);
    
    /**
     * @brief 等待电机停止运动
     * @param timeout_ms 超时时间（毫秒）
     */
    void waitForMotorsStop(uint32_t timeout_ms = 10000);
};

#endif /* __cplusplus */
