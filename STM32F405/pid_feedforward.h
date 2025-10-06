#pragma once
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "xuc.h"
#include "arm_math.h"

// ʹ�����е�PID��
enum { INTEGRATE = 0, LLAST = 0, LAST = 1, NOW = 2 };
#define FILTER 2

// ǰ��˫����̨������
class PID_FeedForward {
private:
    
    PID yaw_angle_pid;    // Yaw�ǶȻ�PID
    PID yaw_speed_pid;    // Yaw�ٶȻ�PID 
    PID pitch_angle_pid;  // Pitch�ǶȻ�PID
    PID pitch_speed_pid;  // Pitch�ٶȻ�PID

    // ǰ������
    float yaw_vel_ffd_gain;
    float yaw_acc_ffd_gain;
    float pitch_vel_ffd_gain;
    float pitch_acc_ffd_gain;

    // ��ǰ״̬
    float current_yaw_angle;
    float current_yaw_speed;
    float current_pitch_angle;
    float current_pitch_speed;

    // ����ģʽ
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

        // ���û����޷�
        yaw_angle_pid.max_limit = 600.0f;
        yaw_speed_pid.max_limit = 3500.0f;
        pitch_angle_pid.max_limit = 400.0f;
        pitch_speed_pid.max_limit = 2500.0f;
    }

    void readMotorState(float yaw_angle, float yaw_speed, float pitch_angle, float pitch_speed); // ��ȡ��ǰ���״̬
    void setYawControlMode(bool angle_control);// ����Yaw����ģʽ
    void PID_FeedForwardControl(const RxPacket_TJ& vision_data,
        float& yaw_current_output, float& pitch_current_output);// ǰ��˫��PID����
    void resetPIDs();// ��������PID������
    void setFeedforwardGains(float yaw_vel_ff, float yaw_acc_ff, float pitch_vel_ff, float pitch_acc_ff);// ����ǰ������
    bool getYawControlMode() const;// ��ȡ��ǰ����ģʽ
};



