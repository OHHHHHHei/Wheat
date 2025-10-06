#include "stm32f4xx_hal.h"
#include "pid.h"
#include "xuc.h"
#include "arm_math.h"
#include "pid_feedforward.h"

void PID_FeedForward::readMotorState(float yaw_angle, float yaw_speed, float pitch_angle, float pitch_speed) {
    current_yaw_angle = yaw_angle;
    current_yaw_speed = yaw_speed;
    current_pitch_angle = pitch_angle;
    current_pitch_speed = pitch_speed;
}

// ����Yaw����ģʽ
void PID_FeedForward::setYawControlMode(bool angle_control) {
    yaw_angle_control_mode = angle_control;
    // �л�ģʽʱ����PID�������
    resetPIDs();
}

// ǰ��˫��PID����
void PID_FeedForward::PID_FeedForwardControl(const RxPacket_TJ& vision_data,
    float& yaw_current_output, float& pitch_current_output) {

    // Yaw�����
    float yaw_speed_target;
    if (yaw_angle_control_mode) {
        // �ǶȻ�����
        float yaw_angle_error = vision_data.yaw_TJ - current_yaw_angle;
        yaw_speed_target = yaw_angle_pid.Position(yaw_angle_error, yaw_angle_pid.max_limit);
    }
    else {
        // �ٶȻ����ƣ�ֱ��ʹ���Ӿ��������ٶ�
        yaw_speed_target = vision_data.yaw_vel_TJ;
    }

    // Yaw�ٶȻ� + ǰ��
    float yaw_speed_error = yaw_speed_target - current_yaw_speed;
    float yaw_pid_output = yaw_speed_pid.Position(yaw_speed_error, yaw_speed_pid.max_limit);

    // ǰ������
    float yaw_velocity_ff = yaw_vel_ffd_gain * vision_data.yaw_vel_TJ;
    float yaw_acceleration_ff = yaw_acc_ffd_gain * vision_data.yaw_acc_TJ;

    yaw_current_output = yaw_pid_output + yaw_velocity_ff + yaw_acceleration_ff;

    // Pitch����ƣ��ǶȻ� + �ٶȻ� + ǰ��
    float pitch_angle_error = vision_data.pitch_TJ - current_pitch_angle;
    float pitch_speed_target = pitch_angle_pid.Position(pitch_angle_error, pitch_angle_pid.max_limit);

    float pitch_speed_error = pitch_speed_target - current_pitch_speed;
    float pitch_pid_output = pitch_speed_pid.Position(pitch_speed_error, pitch_speed_pid.max_limit);

    // ǰ������
    float pitch_velocity_ff = pitch_vel_ffd_gain * vision_data.pitch_vel_TJ;
    float pitch_acceleration_ff = pitch_acc_ffd_gain * vision_data.pitch_acc_TJ;

    pitch_current_output = pitch_pid_output + pitch_velocity_ff + pitch_acceleration_ff;
}

// ��������PID������
void PID_FeedForward::resetPIDs() {
    // �����������
    for (int i = 0; i < 3; i++) {
        yaw_angle_pid.m_error[i] = 0;
        yaw_speed_pid.m_error[i] = 0;
        pitch_angle_pid.m_error[i] = 0;
        pitch_speed_pid.m_error[i] = 0;
    }
}

// ����ǰ������
void PID_FeedForward::setFeedforwardGains(float yaw_vel_ff, float yaw_acc_ff, float pitch_vel_ff, float pitch_acc_ff) {
    yaw_vel_ffd_gain = yaw_vel_ff;
    yaw_acc_ffd_gain = yaw_acc_ff;
    pitch_vel_ffd_gain = pitch_vel_ff;
    pitch_acc_ffd_gain = pitch_acc_ff;
}

// ��ȡ��ǰ����ģʽ
bool PID_FeedForward::getYawControlMode() const {
    return yaw_angle_control_mode;
}


