#pragma once
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "xuc.h"
#include "arm_math.h"

// 前馈双环云台控制器
class PID_FeedForward {
private:
    
    PID yaw_angle_pid;    // Yaw角度环PID
    PID yaw_speed_pid;    // Yaw速度环PID 
    PID pitch_angle_pid;  // Pitch角度环PID
    PID pitch_speed_pid;  // Pitch速度环PID

    // 前馈参数
    float yaw_vel_ffd_gain;
    float yaw_acc_ffd_gain;
    float pitch_vel_ffd_gain;
    float pitch_acc_ffd_gain;

    // 当前状态
    float current_yaw_angle;
    float current_yaw_speed;
    float current_pitch_angle;
    float current_pitch_speed;

    // 控制模式
    bool yaw_angle_control_mode;

public:
    PID_FeedForward()
        : yaw_angle_pid(2.5f, 0.001f, 0.08f, 0.1f)    // Kp, Ti, Td, alpha
        , yaw_speed_pid(0.6f, 0.0005f, 0.025f, 0.1f)  // Kp, Ti, Td, alpha
        , pitch_angle_pid(3.0f, 0.001f, 0.08f, 0.1f)  // Kp, Ti, Td, alpha  
        , pitch_speed_pid(0.8f, 0.0005f, 0.03f, 0.1f) // Kp, Ti, Td, alpha
        , yaw_vel_ffd_gain(0.8f)
        , yaw_acc_ffd_gain(0.1f)
        , pitch_vel_ffd_gain(0.7f)
        , pitch_acc_ffd_gain(0.08f)
        , current_yaw_angle(0)
        , current_yaw_speed(0)
        , current_pitch_angle(0)
        , current_pitch_speed(0)
        , yaw_angle_control_mode(true) {

        // 设置积分限幅
        yaw_angle_pid.max_limit = 600.0f;
        yaw_speed_pid.max_limit = 3500.0f;
        pitch_angle_pid.max_limit = 400.0f;
        pitch_speed_pid.max_limit = 2500.0f;
    }

    void readMotorState(float yaw_angle, float yaw_speed, float pitch_angle, float pitch_speed); // 读取当前电机状态
    void setYawControlMode(bool angle_control);// 设置Yaw控制模式
    void PID_FeedForwardControl(const RxPacket_TJ& vision_data,
        float& yaw_current_output, float& pitch_current_output);// 前馈双环PID控制
    void resetPIDs();// 重置所有PID控制器
    void setFeedforwardGains(float yaw_vel_ff, float yaw_acc_ff, float pitch_vel_ff, float pitch_acc_ff);// 设置前馈参数
    bool getYawControlMode() const;// 获取当前控制模式
};



