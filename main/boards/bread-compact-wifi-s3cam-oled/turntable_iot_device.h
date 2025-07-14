/*
 * 转盘IoT设备声明头文件
 * 便于在板级初始化中包含和使用
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 前向声明
namespace iot {
    class ThingManager;
}

class TurntableIotController;

/**
 * @brief 创建默认配置的转盘IoT控制器
 * @return TurntableIotController* 控制器实例
 */
TurntableIotController* CreateTurntableIotController();

/**
 * @brief 注册转盘设备到IoT管理器
 * @param thing_manager IoT设备管理器
 * @return bool 注册是否成功
 */
bool RegisterTurntableDevice(iot::ThingManager& thing_manager);

/**
 * @brief 在板级初始化中使用的便捷函数
 * 自动处理协议选择和设备注册
 */
void InitializeTurntableIot();

#ifdef __cplusplus
}
#endif
