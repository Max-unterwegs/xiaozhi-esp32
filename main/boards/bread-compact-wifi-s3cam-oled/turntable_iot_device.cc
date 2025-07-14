/*
 * 转盘IoT设备 - Thing包装器
 * 用于IoT框架注册和管理
 */

#include "turntable_iot_device.h"
#include "turntable_iot_controller.h"
#include "esp_log.h"

// #include "iot/thing_manager.h"  // 实际项目中包含IoT框架头文件

static const char *TAG = "turntable_iot_device";

/**
 * @brief 创建默认配置的转盘IoT控制器
 * @return TurntableIotController* 控制器实例
 */
TurntableIotController* CreateTurntableIotController() {
    // 默认配置（基于Arduino代码和硬件连接）
    TurntableIotConfig config = {};
    
    // 水平电机配置（电机1）
    config.horizontal_motor_config = {
        .type = STEPPER_MOTOR_TYPE_BIPOLAR,
        .drive_mode = STEPPER_MOTOR_DRIVE_FULL,
        .steps_per_revolution = 1600,  // Arduino代码中的CIRCLE_Puls
        .pulse_gpio_num = 5,           // STEPPER1_STEP_PIN
        .direction_gpio_num = 6,       // STEPPER1_DIR_PIN
        .enable_gpio_num = 7,         // Ena_PIN
        .rmt_channel = 0,
        .resolution = 1000000,         // 1MHz
        .max_frequency_hz = 2400,      // 1.5转/秒 * 1600步 = 2400Hz
        .min_frequency_hz = 100,
        .acceleration_steps = 400      // 1600/4
    };
    
    // 垂直电机配置（电机2）
    config.vertical_motor_config = {
        .type = STEPPER_MOTOR_TYPE_BIPOLAR,
        .drive_mode = STEPPER_MOTOR_DRIVE_FULL,
        .steps_per_revolution = 1600,  // Arduino代码中的CIRCLE_Puls
        .pulse_gpio_num = 15,           // STEPPER2_STEP_PIN
        .direction_gpio_num = 16,       // STEPPER2_DIR_PIN
        .enable_gpio_num = 17,         // Ena_PIN（共用使能）
        .rmt_channel = 1,
        .resolution = 1000000,         // 1MHz
        .max_frequency_hz = 2400,      // 1.5转/秒 * 1600步 = 2400Hz
        .min_frequency_hz = 100,
        .acceleration_steps = 400      // 1600/4
    };
    
    // 减速比和动作参数（来自Arduino代码）
    config.horizontal_gear_ratio = 2.2973f;
    config.vertical_gear_ratio = 2.2973f;
    config.vertical_action_angle = -140.0f;
    config.action_delay_ms = 1000;
    config.small_move_angle = 10.0f;
    config.relative_move_angle = 90.0f;
    config.vertical_move_angle = 130.0f;
    
    return new TurntableIotController(config);
}

/**
 * @brief 注册转盘设备到IoT管理器
 * @param thing_manager IoT设备管理器
 * @return bool 注册是否成功
 */
bool RegisterTurntableDevice(iot::ThingManager& thing_manager) {
    auto* controller = CreateTurntableIotController();
    if (!controller) {
        return false;
    }
    
    if (!controller->initialize()) {
        delete controller;
        return false;
    }
    
    // 注册到IoT管理器
    thing_manager.AddThing(controller->getThing());
    
    return true;
}

/**
 * @brief 在板级初始化中使用的便捷函数
 * 参考CompactWifiBoard的InitializeIot方法
 */
void InitializeTurntableIot() {
#if CONFIG_IOT_PROTOCOL_XIAOZHI
    auto& thing_manager = iot::ThingManager::GetInstance();
    
    // 注册现有设备
    thing_manager.AddThing(iot::CreateThing("Speaker"));
    thing_manager.AddThing(iot::CreateThing("Lamp"));
    
    // 注册转盘设备
    if (RegisterTurntableDevice(thing_manager)) {
        ESP_LOGI("TURNTABLE_IOT", "转盘设备注册成功");
    } else {
        ESP_LOGE("TURNTABLE_IOT", "转盘设备注册失败");
    }
    
#elif CONFIG_IOT_PROTOCOL_MCP
    // MCP协议模式下使用传统的控制器
    // static LampController lamp(LAMP_GPIO);
    // 转盘可以继续使用MCP方式控制
    ESP_LOGI("TURNTABLE_IOT", "MCP模式：转盘使用传统MCP协议控制");
#endif
}

// 如果使用DECLARE_THING宏方式注册（适合简单场景）
// DECLARE_THING(TurntableIotController);
