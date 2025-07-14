/*
 * 转盘IoT控制器实现
 * 基于Arduino代码逻辑，实现Thing接口
 */

#include "turntable_iot_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 假设的IoT框架头文件包含（实际项目中需要根据框架调整）
// #include "iot/thing.h"
// #include "iot/parameter_list.h"

static const char *TAG = "turntable_iot";

// 临时IoT接口模拟（实际项目中应使用真实的IoT框架）
namespace iot {
    class Thing {
    public:
        Thing(const char* name, const char* description) {}
        virtual ~Thing() = default;
        
        // 模拟属性和方法注册接口
        void addProperty(const char* name, const char* desc, std::function<float()> getter) {}
        void addBoolProperty(const char* name, const char* desc, std::function<bool()> getter) {}
        void addMethod(const char* name, const char* desc, std::function<bool()> handler) {}
    };
    
    class ParameterList {
    public:
        // 参数列表接口模拟
    };
}

/**
 * @brief 构造函数
 */
TurntableIotController::TurntableIotController(const TurntableIotConfig& config)
    : config_(config), thing_(nullptr), initialized_(false)
{
    ESP_LOGI(TAG, "创建转盘IoT控制器");
    
    // 创建水平电机
    horizontal_motor_ = std::make_unique<StepperMotor>(config_.horizontal_motor_config);
    
    // 创建垂直电机
    vertical_motor_ = std::make_unique<StepperMotor>(config_.vertical_motor_config);
    
    // 初始化IoT Thing
    initializeThing();
}

/**
 * @brief 析构函数
 */
TurntableIotController::~TurntableIotController()
{
    if (thing_) {
        delete thing_;
    }
    ESP_LOGI(TAG, "销毁转盘IoT控制器");
}

/**
 * @brief 初始化转盘
 */
bool TurntableIotController::initialize()
{
    ESP_LOGI(TAG, "初始化转盘IoT控制器");
    
    // 启用电机
    if (horizontal_motor_->enable() != ESP_OK) {
        ESP_LOGE(TAG, "启用水平电机失败");
        return false;
    }
    
    if (vertical_motor_->enable() != ESP_OK) {
        ESP_LOGE(TAG, "启用垂直电机失败");
        return false;
    }
    
    // 设置电机速度（参考Arduino代码中的速度设置）
    float horizontal_rpm = 90.0f;  // 1.5转/秒 * 60 = 90 RPM
    float vertical_rpm = 90.0f;
    
    horizontal_motor_->setSpeed(horizontal_rpm);
    vertical_motor_->setSpeed(vertical_rpm);
    
    initialized_ = true;
    ESP_LOGI(TAG, "转盘IoT控制器初始化完成");
    return true;
}

/**
 * @brief 初始化IoT Thing并注册属性和方法
 */
void TurntableIotController::initializeThing()
{
    // 创建Thing实例
    thing_ = new iot::Thing("Turntable", "双轴转盘控制器，支持水平旋转和垂直升降");
    
    // 注册属性
    thing_->addProperty("horizontal_angle", "水平电机当前角度", [this]() -> float {
        return getHorizontalAngle();
    });
    
    thing_->addProperty("vertical_angle", "垂直电机当前角度", [this]() -> float {
        return getVerticalAngle();
    });
    
    thing_->addBoolProperty("is_moving", "转盘是否正在运动", [this]() -> bool {
        return isMoving();
    });
    
    // 注册基础控制方法
    thing_->addMethod("resetHorizontalPosition", "转台水平归零 - 水平方向转动位置设为0位置", [this]() -> bool {
        return resetHorizontalPosition();
    });
    
    thing_->addMethod("resetVerticalPosition", "转台垂直归零 - 竖直方向高度设为0位置", [this]() -> bool {
        return resetVerticalPosition();
    });
    
    // 注册相对移动方法
    thing_->addMethod("rotateHorizontalPositive", "转台水平相对正转 - 相对正转90度", [this]() -> bool {
        return rotateHorizontalPositive();
    });
    
    thing_->addMethod("rotateHorizontalNegative", "转台水平相对反转 - 相对反转90度", [this]() -> bool {
        return rotateHorizontalNegative();
    });
    
    thing_->addMethod("moveVerticalUp", "转台上升 - 垂直方向上升", [this]() -> bool {
        return moveVerticalUp();
    });
    
    thing_->addMethod("moveVerticalDown", "转台下降 - 垂直方向下降", [this]() -> bool {
        return moveVerticalDown();
    });
    
    thing_->addMethod("moveVerticalSmallDown", "转台小幅下降 - 垂直方向小幅下降", [this]() -> bool {
        return moveVerticalSmallDown();
    });
    
    // 注册PAN位置方法（1-4号盘）
    thing_->addMethod("panToPosition1", "转到1号盘 - 转到1号盘0度位置", [this]() -> bool {
        return panToPosition1();
    });
    
    thing_->addMethod("panToPosition2", "转到2号盘 - 转到2号盘90度位置", [this]() -> bool {
        return panToPosition2();
    });
    
    thing_->addMethod("panToPosition3", "转到3号盘 - 转到3号盘180度位置", [this]() -> bool {
        return panToPosition3();
    });
    
    thing_->addMethod("panToPosition4", "转到4号盘 - 转到4号盘270度位置", [this]() -> bool {
        return panToPosition4();
    });
    
    // 注册停止方法
    thing_->addMethod("stopAll", "停止所有运动 - 立即停止水平和垂直电机运动", [this]() -> bool {
        return stopAll();
    });
    
    ESP_LOGI(TAG, "IoT Thing初始化完成，已注册所有属性和方法");
}

/**
 * @brief 转台水平归零（对应串口指令'0'）
 */
bool TurntableIotController::resetHorizontalPosition()
{
    ESP_LOGI(TAG, "执行水平归零");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_NULL(1) -> stepper1.setCurrentPosition(0)
    // 在ESP32实现中，我们将当前位置重置为0
    float current_angle = 0.0f;
    horizontal_motor_->getCurrentAngle(current_angle);
    
    // 由于stepper_motor库没有直接的setCurrentPosition，我们可以通过moveToAngle(0)实现
    esp_err_t ret = horizontal_motor_->moveToAngle(0.0f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "水平归零失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "水平归零完成");
    return true;
}

/**
 * @brief 转台垂直归零（对应串口指令'E'）
 */
bool TurntableIotController::resetVerticalPosition()
{
    ESP_LOGI(TAG, "执行垂直归零");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_NULL(2) -> stepper2.setCurrentPosition(0)
    esp_err_t ret = vertical_motor_->moveToAngle(0.0f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "垂直归零失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "垂直归零完成");
    return true;
}

/**
 * @brief 转台水平相对正转（对应串口指令'+'）
 */
bool TurntableIotController::rotateHorizontalPositive()
{
    ESP_LOGI(TAG, "执行水平相对正转90度");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_Control(1,-90) -> move(-90度)
    // 注意：Arduino代码中是-90度，表示相对负向移动90度
    esp_err_t ret = horizontal_motor_->moveAngle(config_.relative_move_angle, STEPPER_MOTOR_DIR_COUNTER_CLOCKWISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "水平相对正转失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "水平相对正转完成");
    return true;
}

/**
 * @brief 转台水平相对反转（对应串口指令'-'）
 */
bool TurntableIotController::rotateHorizontalNegative()
{
    ESP_LOGI(TAG, "执行水平相对反转90度");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_Control(1,+90) -> move(+90度)
    esp_err_t ret = horizontal_motor_->moveAngle(config_.relative_move_angle, STEPPER_MOTOR_DIR_CLOCKWISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "水平相对反转失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "水平相对反转完成");
    return true;
}

/**
 * @brief 转台上升（对应串口指令'z'）
 */
bool TurntableIotController::moveVerticalUp()
{
    ESP_LOGI(TAG, "执行转台上升");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_Control(2,-130) -> move(-130度)
    esp_err_t ret = vertical_motor_->moveAngle(config_.vertical_move_angle, STEPPER_MOTOR_DIR_COUNTER_CLOCKWISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "转台上升失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "转台上升完成");
    return true;
}

/**
 * @brief 转台下降（对应串口指令'f'）
 */
bool TurntableIotController::moveVerticalDown()
{
    ESP_LOGI(TAG, "执行转台下降");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_Control(2,+130) -> move(+130度)
    esp_err_t ret = vertical_motor_->moveAngle(config_.vertical_move_angle, STEPPER_MOTOR_DIR_CLOCKWISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "转台下降失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "转台下降完成");
    return true;
}

/**
 * @brief 转台垂直小幅下降（对应串口指令'F'）
 */
bool TurntableIotController::moveVerticalSmallDown()
{
    ESP_LOGI(TAG, "执行转台小幅下降");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino代码: Stepper_Control(2,+10) -> move(+10度)
    esp_err_t ret = vertical_motor_->moveAngle(config_.small_move_angle, STEPPER_MOTOR_DIR_CLOCKWISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "转台小幅下降失败");
        return false;
    }
    
    waitForMotorsStop();
    ESP_LOGI(TAG, "转台小幅下降完成");
    return true;
}

/**
 * @brief PAN到1号盘（对应串口指令'A'）
 */
bool TurntableIotController::panToPosition1()
{
    ESP_LOGI(TAG, "执行PAN到1号盘（0度）");
    return performPanAction(0.0f);
}

/**
 * @brief PAN到2号盘（对应串口指令'B'）
 */
bool TurntableIotController::panToPosition2()
{
    ESP_LOGI(TAG, "执行PAN到2号盘（90度）");
    return performPanAction(90.0f);
}

/**
 * @brief PAN到3号盘（对应串口指令'C'）
 */
bool TurntableIotController::panToPosition3()
{
    ESP_LOGI(TAG, "执行PAN到3号盘（180度）");
    return performPanAction(180.0f);
}

/**
 * @brief PAN到4号盘（新增）
 */
bool TurntableIotController::panToPosition4()
{
    ESP_LOGI(TAG, "执行PAN到4号盘（270度）");
    return performPanAction(270.0f);
}

/**
 * @brief 通用PAN动作（支持自定义角度）
 */
bool TurntableIotController::performPanAction(float horizontal_angle)
{
    ESP_LOGI(TAG, "执行PAN动作到%.1f度", horizontal_angle);
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    // 参考Arduino PAN函数：
    // stepper1.runToNewPosition(angle*stepper_i[0]*CIRCLE_Puls/360.0);
    // stepper2.runToNewPosition((-140)*stepper_i[1]*CIRCLE_Puls/360.0);
    // delay(1000);
    // stepper2.runToNewPosition(0);
    
    // 1. 水平电机移动到目标角度
    esp_err_t ret = horizontal_motor_->moveToAngle(horizontal_angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "水平移动到%.1f度失败", horizontal_angle);
        return false;
    }
    waitForMotorsStop();
    
    // 2. 垂直电机执行特殊动作（向下移动140度）
    ret = vertical_motor_->moveAngle(abs(config_.vertical_action_angle), 
                                   config_.vertical_action_angle < 0 ? 
                                   STEPPER_MOTOR_DIR_COUNTER_CLOCKWISE : 
                                   STEPPER_MOTOR_DIR_CLOCKWISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "垂直动作失败");
        return false;
    }
    waitForMotorsStop();
    
    // 3. 延时
    vTaskDelay(pdMS_TO_TICKS(config_.action_delay_ms));
    
    // 4. 垂直电机复位到0位置
    ret = vertical_motor_->moveToAngle(0.0f);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "垂直复位失败");
        return false;
    }
    waitForMotorsStop();
    
    ESP_LOGI(TAG, "PAN动作到%.1f度完成", horizontal_angle);
    return true;
}

/**
 * @brief 获取水平电机当前角度
 */
float TurntableIotController::getHorizontalAngle()
{
    if (!initialized_) {
        return 0.0f;
    }
    
    float angle = 0.0f;
    horizontal_motor_->getCurrentAngle(angle);
    return angle;
}

/**
 * @brief 获取垂直电机当前角度
 */
float TurntableIotController::getVerticalAngle()
{
    if (!initialized_) {
        return 0.0f;
    }
    
    float angle = 0.0f;
    vertical_motor_->getCurrentAngle(angle);
    return angle;
}

/**
 * @brief 检查是否正在运动
 */
bool TurntableIotController::isMoving()
{
    if (!initialized_) {
        return false;
    }
    
    bool h_running = false, v_running = false;
    horizontal_motor_->isRunning(h_running);
    vertical_motor_->isRunning(v_running);
    
    return h_running || v_running;
}

/**
 * @brief 停止所有运动
 */
bool TurntableIotController::stopAll()
{
    ESP_LOGI(TAG, "停止所有电机运动");
    
    if (!initialized_) {
        ESP_LOGE(TAG, "转盘未初始化");
        return false;
    }
    
    esp_err_t h_ret = horizontal_motor_->stop();
    esp_err_t v_ret = vertical_motor_->stop();
    
    if (h_ret != ESP_OK || v_ret != ESP_OK) {
        ESP_LOGE(TAG, "停止电机失败");
        return false;
    }
    
    ESP_LOGI(TAG, "所有电机已停止");
    return true;
}

/**
 * @brief 角度转换为步数（考虑减速比）
 */
uint32_t TurntableIotController::angleToSteps(float angle, float gear_ratio, uint32_t steps_per_rev)
{
    // 参考Arduino代码: angle*stepper_i[motor]*CIRCLE_Puls/360.0
    return (uint32_t)(angle * gear_ratio * steps_per_rev / 360.0f);
}

/**
 * @brief 步数转换为角度（考虑减速比）
 */
float TurntableIotController::stepsToAngle(uint32_t steps, float gear_ratio, uint32_t steps_per_rev)
{
    return (float)steps * 360.0f / (gear_ratio * steps_per_rev);
}

/**
 * @brief 等待电机停止运动
 */
void TurntableIotController::waitForMotorsStop(uint32_t timeout_ms)
{
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while (isMoving()) {
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "等待电机停止超时");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms检查间隔
    }
}
