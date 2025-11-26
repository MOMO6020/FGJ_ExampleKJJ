#pragma once
#include "app/app_def.hpp"
#include "bsp/log/log.hpp"
#include "bsp/system/mutex.hpp"
#include "bsp/uart/stm32_uart.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"
#include "modules/umt/Message.hpp"
#include "stm32f4xx_hal_uart.h"
#include <format>
#include <memory>

// For Test:
UARTHandle_t              uart6 = nullptr;
extern UART_HandleTypeDef huart6;

class Gimbal
{
public:
    Gimbal(attitude_t* imu_data, std::shared_ptr<float> gimbal_yaw_motor_angle_ptr) : imu_data_(imu_data)
    {
        uart6 = STM32UART_Init(&huart6);
        if (gimbal_yaw_motor_angle_ptr == nullptr)
        {
            while (true)
            {
                LOGERROR("Gimbal", "gimbal_yaw_motor_angle_ptr is nullptr");
                STM32TimeDWT::Delay(1000);
            }
        }
        gimbal_yaw_motor_angle_ptr_ = gimbal_yaw_motor_angle_ptr;
        extern CAN_HandleTypeDef hcan1;

        // -------------------------- 只保留一个 GM6020 电机（示例：Pitch 轴）--------------------------
        // 初始化 Pitch 轴 GM6020（ID=2，根据实际拨码调整）
        motor_pitch_ = std::make_shared<DJIMotor>(
            &hcan1, 2, MotorType::GM6020,
            MotorPIDSetting{
                .outer_loop = CloseloopType::ANGLE_LOOP,
                .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
                .reverse = false,  // 安装方向反了就改成 true
                .external_angle_feedback = FeedbackType::INTERNAL,  // 用电机内置编码器（单独测试更稳定）
                .external_speed_feedback = FeedbackType::INTERNAL
            },
            MotorPID{
                // 角度环 PID（单个电机测试建议参数，可直接用）
                .pid_angle_ = PIDController(PIDConfig{30.0f, 0.8f, 1.0f, 3000.0f, 600.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
                // 速度环 PID（单个电机测试建议参数）
                .pid_speed_ = PIDController(PIDConfig{22.0f, 1.2f, 0.2f, 16384.0f, 1200.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
                // 注释外部反馈，避免 IMU 干扰单独电机测试
            });

        // -------------------------- 注释掉另一个电机（避免干扰）--------------------------
        // motor_yaw_ = std::make_shared<DMMotor>(...);  // 注释 Yaw 轴电机初始化
        // gimbal_cmd_sub_.bind("gimbal_cmd");  // 注释消息订阅（无需外部指令）

        LOGINFO("Gimbal", "Single GM6020 (Pitch) Init Success");
    }

    void GimbalTask()
    {
        try
        {
            // -------------------------- 单个电机固定目标角度控制 --------------------------
            motor_pitch_->enable();  // 一直使能电机（测试用）

            // ********** 这里修改目标角度！单位：度 **********
            float target_angle = 45.0f;  // 示例：转 45 度（可改成 0/90/-30 等）
            // ************************************************

            // 角度限位（根据机械结构调整，防止电机过转）
            const float PITCH_MIN = -45.0f;  // 最小角度（下俯45度）
            const float PITCH_MAX = 45.0f;   // 最大角度（上仰45度）
            target_angle = std::clamp(target_angle, PITCH_MIN, PITCH_MAX);

            // 设置目标角度，驱动电机转动
            motor_pitch_->setRef(target_angle);

            // -------------------------- 调试日志（查看电机状态）--------------------------
            static uint32_t log_cnt = 0;
            if (++log_cnt >= 50)  // 50*2ms=100ms 输出一次日志
            {
                log_cnt = 0;
                LOGINFO("GM6020_Test", std::format(
                    "Target: {:.1f}° | Current: {:.1f}° | Speed: {:.1f}°/s | Current: {}mA",
                    target_angle,
                    motor_pitch_->measure_.total_angle,  // 电机当前角度
                    motor_pitch_->measure_.speed_dps,    // 电机当前速度
                    motor_pitch_->measure_.torque_current // 电机输出电流
                ));
            }

            // 无需处理 Yaw 轴（已注释）
            *gimbal_yaw_motor_angle_ptr_ = 0.0f;  // 占位，避免空指针问题
        }
        catch (umt::MessageError& e)
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<gimbal_cmd> gimbal_cmd_sub_;
    std::shared_ptr<DMMotor>    motor_yaw_                  = nullptr;  // 未使用
    std::shared_ptr<DJIMotor>   motor_pitch_                = nullptr;  // 唯一的 GM6020 电机
    std::shared_ptr<float>      gimbal_yaw_motor_angle_ptr_ = nullptr;
    attitude_t*                 imu_data_                   = nullptr;
};