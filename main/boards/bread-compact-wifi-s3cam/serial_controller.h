#ifndef __SERIAL_CONTROLLER_H__
#define __SERIAL_CONTROLLER_H__
// 引入必要的头文件
#include "board.h"            // 开发板相关定义头文件
#include <driver/gpio.h>      // ESP32 GPIO驱动头文件
#include <esp_log.h>          // ESP32日志功能头文件
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "mcp_server.h"      // MCP服务器头文件
// 配置参数
#define UART_PORT_NUM      UART_NUM_1    // 使用UART
#define TXD_PIN            GPIO_NUM_43   // 发送引脚
#define RXD_PIN            GPIO_NUM_44   // 接收引脚
#define BUF_SIZE           1024          // 缓冲区大小
#define UART_BAUD_RATE     9600        // 波特率

static char buffer[256];  // 接收缓冲区

#define TAG "CompactWifiBoardS3Cam"
 

class SerialController
{
private:
        void uart_init() {
            // UART配置结构体
            uart_config_t uart_config = {
                .baud_rate = UART_BAUD_RATE,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
            };
 
            // 配置UART参数
            ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
 
            // 设置UART引脚
            ESP_ERROR_CHECK(uart_set_pin(
                UART_PORT_NUM,
                TXD_PIN,
                RXD_PIN,
                UART_PIN_NO_CHANGE, // RTS引脚(不使用)
                UART_PIN_NO_CHANGE  // CTS引脚(不使用)
            ));
 
            // 安装UART驱动程序
            ESP_ERROR_CHECK(uart_driver_install(
                UART_PORT_NUM,
                BUF_SIZE * 2,  // 接收缓冲区大小
                BUF_SIZE * 2,   // 发送缓冲区大小
                0,              // 事件队列大小
                NULL,           // 事件队列句柄
                0               // 中断分配标志
            ));
            ESP_LOGI(TAG, "UART initialized with baud rate %d", UART_BAUD_RATE);
        }
        
        
        // 发送字符串
        void uart_send_str(const char* str) {
            uart_write_bytes(UART_PORT_NUM, str, strlen(str));
        }
        
        // 发送格式化数据（类似printf）
        void uart_printf(const char* format, ...) {
            char buffer[256];
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
        }

        //接收返回数据
        char* uart_receive_str() {
            uart_read_bytes(UART_PORT_NUM, buffer, 256-1, 20 / portTICK_PERIOD_MS);
            return buffer;
        }

public:
    SerialController()
    {
        auto &mcp_server = McpServer::GetInstance();
        // 水平位置归零
        mcp_server.AddTool("self.turntable.reset_horizontal",
                           "转台水平坐标设置为零",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                this->uart_send_str("reset_horizontal");
                                ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                return true;

                           });

        // 垂直位置归零
        mcp_server.AddTool("self.turntable.reset_vertical",
                           "转台垂直坐标设置为零",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                this->uart_send_str("reset_vertical");
                                ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                return true;
                           });

        // 水平相对旋转
        mcp_server.AddTool("self.turntable.rotate_horizontal",
                           "转台水平方向相对旋转，正数顺时针，负数逆时针",
                           PropertyList({Property("angle", kPropertyTypeInteger, -360, 360)}),
                           [this](const PropertyList &properties) -> ReturnValue
                           {
                                 int angle = properties["angle"].value<int>();
                                 this->uart_printf("H%d", static_cast<int>(angle));
                                 ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                 return true;   
                           });

        // 垂直相对移动
        mcp_server.AddTool("self.turntable.move_vertical",
                           "转台垂直方向相对移动，正数向上，负数向下",
                           PropertyList({Property("angle", kPropertyTypeInteger, -180, 180)}),
                           [this](const PropertyList &properties) -> ReturnValue
                           {
                                 int angle = properties["angle"].value<int>();
                                 this->uart_printf("V%d", static_cast<int>(angle));
                                 ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                 return true;
                           });

        // 水平绝对位置移动
        mcp_server.AddTool("self.turntable.move_to_horizontal",
                           "转台水平方向移动到绝对位置",
                           PropertyList({Property("target_angle", kPropertyTypeInteger, 0, 360)}),
                           [this](const PropertyList &properties) -> ReturnValue
                           {
                                 int target_angle = properties["target_angle"].value<int>();
                                 this->uart_printf("H%d", target_angle);
                                 ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                 return true;
                           });

        // PAN复合动作
        mcp_server.AddTool("self.turntable.pan_action",
                           "转台移动到指定角度下料",
                           PropertyList({Property("horizontal_angle", kPropertyTypeInteger, 0, 360)}),
                           [this](const PropertyList &properties) -> ReturnValue
                           {
                                 int horizontal_angle = properties["horizontal_angle"].value<int>();
                                 this->uart_printf("H%d", horizontal_angle);
                                 ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                 return true;
                           });

        // 垂直小幅下降
        mcp_server.AddTool("self.turntable.move_vertical_small_down",
                           "转台小幅下降",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                 this->uart_send_str("move_vertical_small_down");
                                 ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                 return true;
                           });

        // 快捷垂直移动指令
        mcp_server.AddTool("self.turntable.move_up",
                           "转台上升",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                 this->uart_send_str("move_up");
                                 ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                 return true;
                           });

        mcp_server.AddTool("self.turntable.move_down",
                           "转台下降",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                    this->uart_send_str("move_down");
                                    ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                    return true;
                           });

        // 快捷PAN位置指令
        mcp_server.AddTool("self.turntable.pan_to_position_1",
                           "转台转到1号食盒",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                this->uart_send_str("pan_to_position_1");
                                ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                return true;
                           });

        mcp_server.AddTool("self.turntable.pan_to_position_2",
                           "转台转到2号食盒",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                this->uart_send_str("pan_to_position_2");
                                ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                return true;
                           });

        mcp_server.AddTool("self.turntable.pan_to_position_3",
                           "转台转到3号食盒",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                this->uart_send_str("pan_to_position_3");
                                ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                return true;
                           });

        mcp_server.AddTool("self.turntable.pan_to_position_4",
                           "转台转到4号食盒",
                           PropertyList(),
                           [this](const PropertyList &) -> ReturnValue
                           {
                                this->uart_send_str("pan_to_position_4");
                                ESP_LOGI(TAG, "UART response: %s", this->uart_receive_str());
                                return true;
                           });
    }
};

#endif // __SERIAL_CONTROLLER_H__

