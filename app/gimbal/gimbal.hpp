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
        // 启用GM6020作为Yaw轴电机（修改部分）
        motor_yaw_ = std::make_shared<DJIMotor>(
            &hcan1, 1, MotorType::GM6020,  // 电机ID设为1（根据实际拨码调整），类型指定为GM6020
            MotorPIDSetting{
                .outer_loop = CloseloopType::ANGLE_LOOP,  // 外环为角度环
                .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,  // 角度+速度双环控制
                .reverse = false,  // 根据安装方向调整是否反转
                .external_angle_feedback = FeedbackType::INTERNAL,  // 使用电机内置编码器反馈角度
                .external_speed_feedback = FeedbackType::INTERNAL   // 使用电机内置编码器反馈速度
            },
            MotorPID{
                .pid_angle_ = PIDController(  // 角度环PID参数（需根据实际调试优化）
                    PIDConfig{35.0f, 0.5f, 1.2f, 3000.0f, 500.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
                .pid_speed_ = PIDController(  // 速度环PID参数（需根据实际调试优化）
                    PIDConfig{25.0f, 1.0f, 0.3f, 16384.0f, 1000.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
                // 注释掉外部反馈，使用电机内部反馈
                // .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->YawTotalAngle),
                // .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[2]),
            });
        
        // 保持Pitch轴GM6020配置不变（如需调整可参考Yaw轴修改）
        motor_pitch_ = std::make_shared<DJIMotor>(
            &hcan1, 2, MotorType::GM6020,
            MotorPIDSetting{
                CloseloopType::ANGLE_LOOP,
                static_cast<CloseloopType>(CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP),
                false,
                FeedbackType::INTERNAL,  // 改为内部反馈（推荐）
                FeedbackType::INTERNAL   // 改为内部反馈（推荐）
            },
            MotorPID{
                .pid_angle_ = PIDController(
                    PIDConfig{28.0f, 22.5f, 0.55f, 500.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
                .pid_speed_ = PIDController(
                    PIDConfig{30.0f, 22.0f, 0.0f, 16384.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
                // 注释掉外部反馈，使用电机内部反馈
                // .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Pitch),
                // .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[0]),
            });
        gimbal_cmd_sub_.bind("gimbal_cmd");
    }
    void GimbalTask()
    {
        try
        {
            gimbal_cmd cmd = gimbal_cmd_sub_.pop();
            if (cmd.force_stop)
            {
                motor_yaw_->disable();
                motor_pitch_->disable();
            }
            else
            {
                motor_yaw_->enable();
                motor_pitch_->enable();
                // 限制Yaw轴角度范围（根据机械结构调整，防止过转）
                const float YAW_MIN = -180.0f;  // 最小角度
                const float YAW_MAX = 180.0f;   // 最大角度
                cmd.yaw = std::clamp(cmd.yaw, YAW_MIN, YAW_MAX);
                
                // 限制Pitch轴角度范围
                const float PITCH_MIN = -30.0f;  // 最小角度（例如下俯30度）
                const float PITCH_MAX = 30.0f;   // 最大角度（例如上仰30度）
                cmd.pitch = std::clamp(cmd.pitch, PITCH_MIN, PITCH_MAX);
                
                motor_yaw_->setRef(cmd.yaw);
                motor_pitch_->setRef(cmd.pitch);
            }
            // 更新Yaw轴电机角度（使用内部编码器值）
            *gimbal_yaw_motor_angle_ptr_ = motor_yaw_->measure_.total_angle;
            
            // 调试日志（取消注释可查看电机状态）
            // LOGINFO("Gimbal", std::format("Yaw: {:.2f}, Target: {:.2f}; Pitch: {:.2f}, Target: {:.2f}",
            //                             motor_yaw_->measure_.total_angle, cmd.yaw,
            //                             motor_pitch_->measure_.total_angle, cmd.pitch));
        }
        catch (umt::MessageError& e)
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<gimbal_cmd> gimbal_cmd_sub_;
    std::shared_ptr<DJIMotor>   motor_yaw_                  = nullptr;  // 改为DJIMotor类型（GM6020）
    std::shared_ptr<DJIMotor>   motor_pitch_                = nullptr;
    std::shared_ptr<float>      gimbal_yaw_motor_angle_ptr_ = nullptr;
    attitude_t*                 imu_data_                   = nullptr;
};