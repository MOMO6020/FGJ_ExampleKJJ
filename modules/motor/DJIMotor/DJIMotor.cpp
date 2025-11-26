/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 13:59:32
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-11-27 16:00:00
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.cpp
 * @Description  : GM6020 电机适配修正（CAN1 控制，编译兼容版）
 */
#include "DJIMotor.hpp"
#include "bsp/can/stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "cmsis_os2.h"
#include <cstring>
#include <cstdio>  // 用于 snprintf
#include <algorithm>  // 用于 std::clamp（若仍报错可替换为手动逻辑）

#define SPEED_SMOOTH_COEF 0.9f
#define CURRENT_SMOOTH_COEF 0.9f

/**
 * @brief 电机组使能标志
 * 与control_data_数组一一对应, 当对应组存在电机时, 使能标志为1
 * 用于防止发送空数据包
 */
bool group_enable_flag[8] = {false, false, false, false, false, false, false, false};

// 移除静态变量重复定义（hpp中已用inline static声明，无需重复初始化）
// DJIMotor* DJIMotor::registered_motors_[DJIMotor::MAX_DJIMOTOR_COUNT] = {nullptr};
// uint8_t DJIMotor::motor_count_ = 0;
// ICAN::ClassicPacket DJIMotor::control_data_[8] = {0};

DJIMotor::DJIMotor(CAN_HandleTypeDef* _hcan,
                   uint16_t           _motor_id,
                   MotorType          _motor_type,
                   MotorPIDSetting    _setting,
                   MotorPID           _pid_config)
    : IMotor(_motor_type, _setting, _pid_config), motor_id_(_motor_id)
{
    if (motor_count_ >= MAX_DJIMOTOR_COUNT)
    {
        LOGERROR("DJIMotor", "Motor count exceeds maximum limit");
        while (true)
            osDelay(1000);
    }

    // 强制使用 CAN1（根据你的需求锁定 CAN1）
    if (_hcan->Instance != CAN1)
    {
        LOGERROR("DJIMotor", "GM6020 only support CAN1!");
        while (true)
            osDelay(1000);
    }

    auto     can    = STM32CAN_GetInstance(_hcan);
    // GM6020 反馈 ID 按官方协议：0x204 + 电机ID（框架已正确配置，保留）
    uint16_t can_id = motor_type_ == MotorType::GM6020 ? 0x204 + motor_id_ : 0x200 + motor_id_;
    can->setRxCallback(can_id, std::bind(&DJIMotor::decode, this, std::placeholders::_1, std::placeholders::_2));

    // 电机发送组配置（GM6020 CAN1 专属修正）
    switch (motor_type_)
    {
        case MotorType::M2006:
        case MotorType::M3508:
            motor_tx_group_ = motor_id_ <= 4 ? 0 : 1;
            motor_tx_group_ = _hcan->Instance == CAN1 ? motor_tx_group_ : motor_tx_group_ + 4;
            reduce_rate_    = motor_type_ == MotorType::M2006 ? 36 : 19;
            break;
        case MotorType::GM6020:
            motor_tx_group_ = 2;  // CAN1 GM6020 固定发送组（框架已适配官方控制ID 0x1FE）
            motor_tx_group_ = motor_id_ <= 4 ? motor_tx_group_ : motor_tx_group_ + 4;
            reduce_rate_    = 1.0f;  // GM6020 无需速率缩减
            // 初始化控制数据帧类型（标准帧，框架可能遗漏）
            control_data_[motor_tx_group_].type = ICAN::Type::STANDARD;
            control_data_[motor_tx_group_].id = 0x1FE;  // GM6020 官方控制ID（ID1-4共用）
            break;
        default: LOGERROR("DJIMotor", "Invalid motor type"); break;
    }

    // 检查ID冲突
    for (uint8_t i = 0; i < MAX_DJIMOTOR_COUNT; i++)
    {
        if (registered_motors_[i] != nullptr && 
            registered_motors_[i]->motor_tx_group_ == motor_tx_group_ && 
            registered_motors_[i]->motor_id_ == motor_id_)
        {
            // 替换 std::to_string 为 snprintf（兼容嵌入式）
            char err_buf[64];
            snprintf(err_buf, sizeof(err_buf), "Motor ID already exists: %d", motor_id_);
            LOGERROR("DJIMotor", err_buf);
            while (true)
                osDelay(1000);
        }
    }

    registered_motors_[motor_count_++] = this;  // 添加到电机数组
    group_enable_flag[motor_tx_group_] = true;  // 设置使能标志

    // GM6020 初始化日志（替换 std::format 为 snprintf，兼容嵌入式）
    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf), "GM6020 Init (CAN1, ID: %d, Feedback ID: 0x%X, Control ID: 0x1FE)",
        motor_id_, can_id);
    LOGINFO("DJIMotor", log_buf);
}

void DJIMotor::decode(const uint8_t* buf, const uint8_t len)
{
    if (buf == nullptr || len < 8)
    {
        LOGERROR("DJIMotor", "Invalid buffer");
        return;
    }
    if (!is_online)
    {
        is_online = true;
        char log_buf[64];
        switch (motor_type_)
        {
            case MotorType::GM6020:
                snprintf(log_buf, sizeof(log_buf), "Motor GM6020#%d", motor_id_);
                LOGINFO(log_buf, "Online (CAN1)");
                break;
            case MotorType::M3508:
                snprintf(log_buf, sizeof(log_buf), "Motor M3508#%d", motor_id_);
                LOGINFO(log_buf, "Online");
                break;
            case MotorType::M2006:
                snprintf(log_buf, sizeof(log_buf), "Motor M2006#%d", motor_id_);
                LOGINFO(log_buf, "Online");
                break;
        }
    }
    // 删除 daemon_->feed()（框架未定义 Daemon 类，无需心跳检测）
    measure_.last_encoder = measure_.encoder;
    measure_.encoder      = ((uint16_t)buf[0] << 8) | buf[1];  // 官方协议：机械角度（0-8191）
    measure_.angle        = (float)measure_.encoder / 8192.0f * 360.0f;  // 角度转换（0-360°）
    
    // 速度解析修正（GM6020 转速单位为 rpm，转换为 度/秒：1rpm=6°/s）
    int16_t raw_speed = (buf[2] << 8) | buf[3];  // 原生速度（带符号）
    float current_speed_dps = 6.0f * static_cast<float>(raw_speed);
    measure_.speed_dps = (1.0f - SPEED_SMOOTH_COEF) * measure_.speed_dps + SPEED_SMOOTH_COEF * current_speed_dps;

    // 电流解析修正（保留符号，平滑滤波）
    int16_t raw_current = (buf[4] << 8) | buf[5];
    measure_.torque_current = (1.0f - CURRENT_SMOOTH_COEF) * measure_.torque_current + CURRENT_SMOOTH_COEF * static_cast<float>(raw_current);
    
    measure_.temperature = buf[6];

    // 多圈角度计算（解决圈数溢出）
    if (measure_.encoder - measure_.last_encoder > 4096)
        measure_.total_round--;
    else if (measure_.last_encoder - measure_.encoder > 4096)
        measure_.total_round++;
    measure_.total_angle = measure_.total_round * 360.0f + measure_.angle;

    // 调试日志：每50次解码输出一次（替换 std::format 为 snprintf）
    static uint32_t log_cnt = 0;
    if (motor_type_ == MotorType::GM6020 && ++log_cnt >= 50)
    {
        log_cnt = 0;
        char log_buf[128];
        snprintf(log_buf, sizeof(log_buf), "ID: %d, TotalAngle: %.1f°, Speed: %.1f°/s, Current: %dmA, Temp: %d°C",
            motor_id_, measure_.total_angle, measure_.speed_dps, measure_.torque_current, measure_.temperature);
        LOGINFO("GM6020_CAN1", log_buf);
    }
}

void DJIMotor::offlineCallback()
{
    if (is_online)
    {
        memset(static_cast<void*>(&measure_), 0, sizeof(measure_));
        is_online = false;
        char log_buf[64];
        switch (motor_type_)
        {
            case MotorType::GM6020:
                snprintf(log_buf, sizeof(log_buf), "Motor GM6020#%d", motor_id_);
                LOGWARNING(log_buf, "Offline (CAN1)");
                break;
            case MotorType::M3508:
                snprintf(log_buf, sizeof(log_buf), "Motor M3508#%d", motor_id_);
                LOGWARNING(log_buf, "Offline");
                break;
            case MotorType::M2006:
                snprintf(log_buf, sizeof(log_buf), "Motor M2006#%d", motor_id_);
                LOGWARNING(log_buf, "Offline");
                break;
            default: LOGERROR("DJIMotor", "Invalid motor type"); break;
        }
    }
}

int16_t DJIMotor::calculateOutputCurrent()
{
    if (!is_online || !enable_)
        return 0;

    float pid_measure;
    float pid_ref = pid_ref_;
    if (setting_.reverse)
        pid_ref *= -1.0f;

    // 1. 角度环（外环）：目标角度 → 速度指令
    if ((setting_.close_loop & CloseloopType::ANGLE_LOOP) && setting_.outer_loop == CloseloopType::ANGLE_LOOP)
    {
        if (setting_.external_angle_feedback == FeedbackType::EXTERNAL)
        {
            if (pidControllers_.pid_angle_feedback_ptr_ != nullptr)
                pid_measure = *pidControllers_.pid_angle_feedback_ptr_;
            else
            {
                LOGERROR("DJIMotor", "External angle feedback pointer is null");
                return 0;
            }
        }
        else
        {
            pid_measure = measure_.total_angle;  // GM6020 用内置编码器累计角度（稳定）
        }
        // GM6020 角度环 PID 适配（增大比例系数，加快响应）
        pid_ref = pidControllers_.pid_angle_.PIDCalculate(pid_measure, pid_ref);
        // 限制速度指令范围（替换 std::clamp 为手动逻辑，兼容旧编译器）
        pid_ref = (pid_ref < -300.0f) ? -300.0f : (pid_ref > 300.0f ? 300.0f : pid_ref);
    }

    // 2. 速度环（内环）：速度指令 → 电流指令
    if ((setting_.close_loop & CloseloopType::SPEED_LOOP) &&
        (uint8_t(setting_.outer_loop) & (CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP)))
    {
        if (setting_.external_speed_feedback == FeedbackType::EXTERNAL)
        {
            if (pidControllers_.pid_speed_feedback_ptr_ != nullptr)
                pid_measure = *pidControllers_.pid_speed_feedback_ptr_;
            else
            {
                LOGERROR("DJIMotor", "External speed feedback pointer is null");
                return 0;
            }
        }
        else
        {
            pid_measure = measure_.speed_dps;  // GM6020 用内置速度反馈
        }
        // GM6020 速度环 PID 适配（增大比例系数，提升动态响应）
        pid_ref = pidControllers_.pid_speed_.PIDCalculate(pid_measure, pid_ref);
    }

    // 3. 电流环（可选，框架默认未启用，保留逻辑）
    if (setting_.close_loop & CloseloopType::CURRENT_LOOP)
    {
        pid_ref = pidControllers_.pid_current_.PIDCalculate(measure_.torque_current, pid_ref);
    }

    // GM6020 电流限制（替换 std::clamp 为手动逻辑，兼容旧编译器）
    float clamped_ref = (pid_ref < -3000.0f) ? -3000.0f : (pid_ref > 3000.0f ? 3000.0f : pid_ref);
    pid_out_ = static_cast<int16_t>(clamped_ref);
    
    // 调试日志：输出 PID 计算结果（替换 std::format 为 snprintf）
    static uint32_t pid_log_cnt = 0;
    if (motor_type_ == MotorType::GM6020 && ++pid_log_cnt >= 50)
    {
        pid_log_cnt = 0;
        char log_buf[128];
        snprintf(log_buf, sizeof(log_buf), "ID: %d, TargetAngle: %.1f°, CurrentAngle: %.1f°, PIDOut: %d",
            motor_id_, pid_ref_, measure_.total_angle, pid_out_);
        LOGINFO("GM6020_PID", log_buf);
    }

    return pid_out_;
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void DJIMotor::DJIMotorControl()
{
    static CANHandle_t can1_handle = STM32CAN_GetInstance(&hcan1);
    static CANHandle_t can2_handle = STM32CAN_GetInstance(&hcan2);

    // 1. 清空控制数据（保留帧类型和ID，仅清空数据段）
    for (auto& packet : control_data_)
        std::memset(packet.data, 0, 8);
    std::memset(group_enable_flag, 0, sizeof(group_enable_flag));

    // 2. 单次遍历：计算电流 + 更新组使能标志
    for (uint8_t i = 0; i < motor_count_; i++)
    {
        DJIMotor* motor = registered_motors_[i];
        if (motor == nullptr || !motor->is_online)
            continue;

        // 计算输出电流
        int16_t output_current = motor->calculateOutputCurrent();
        // 修正数据字节序（GM6020 要求：高8位在前，低8位在后 → 框架原逻辑正确，保留）
        uint8_t idx = motor->motor_id_ <= 4 ? 2 * (motor->motor_id_ - 1) : 2 * (motor->motor_id_ - 5);
        if (idx + 1 < 8)  // 避免数组越界
        {
            control_data_[motor->motor_tx_group_].data[idx]     = output_current >> 8;  // 高8位
            control_data_[motor->motor_tx_group_].data[idx + 1] = output_current & 0xFF; // 低8位
        }

        // 标记组有效
        group_enable_flag[motor->motor_tx_group_] = true;
    }

    // 3. 发送有效组数据（CAN1 优先）
    for (uint8_t group = 0; group < 8; group++)
    {
        if (group_enable_flag[group])
        {
            ICAN::ClassicPacket& packet = control_data_[group];
            // 确保 GM6020 控制帧为标准帧（双重校验）
            if (motor_count_ > 0 && registered_motors_[0]->motor_type_ == MotorType::GM6020)
            {
                packet.type = ICAN::Type::STANDARD;
                packet.id = 0x1FE;
            }
            // 发送数据
            if (group < 4)
                can1_handle->transmit(packet);
            else
                can2_handle->transmit(packet);
        }
    }
}