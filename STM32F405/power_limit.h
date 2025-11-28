#pragma once
#include <cmath>
#include <cstdint>
#include "motor.h"

/**
 * @file power_limit.h
 * @brief 底盘功率控制模块
 * @note 电机功率模型: P = τω + k1|ω| + k2τ? + k3/4
 *       - τ: 电机输出力矩 (N·m)
 *       - ω: 电机角速度 (rad/s)
 *       - k1: 转速损耗系数
 *       - k2: 力矩平方损耗系数
 *       - k3: 静态损耗
 */

// M3508电机参数
#define M3508_TORQUE_CONSTANT   0.3f      // 力矩常数 (N·m/A)
#define M3508_REDUCTION_RATIO   19.0f     // 减速比
#define M3508_CURRENT_TO_AMPERE (20.0f / 16384.0f)  // 电流值转安培

// 功率分配阈值
#define ERROR_DISTRIBUTION_UPPER  20.0f   // error大P分配上阈值
#define ERROR_DISTRIBUTION_LOWER  15.0f   // error大P分配下阈值

/**
 * @brief 单个电机的功率控制对象
 */
struct PowerObj
{
    float pidOutput;     // PID原始输出电流 (电机current单位)
    float curAv;         // 当前角速度 (rad/s)
    float setAv;         // 目标角速度 (rad/s)
    float maxOutput;     // 最大输出限幅
};

/**
 * @brief 功率控制器类
 */
class PowerLimiter
{
public:
    /**
     * @brief 初始化功率控制器
     * @param motors 底盘电机指针数组 (4个)
     * @param maxPower 功率上限 (W)
     * @param k1 转速损耗系数 (默认0.22)
     * @param k2 力矩平方损耗系数 (默认1.2)
     * @param k3 静态损耗 (默认2.78)
     */
    void Init(Motor** motors, float maxPower, float k1 = 0.22f, float k2 = 1.2f, float k3 = 2.78f);
    
    /**
     * @brief 功率限制核心函数
     * @param pidOutputs 4个电机的PID原始输出
     * @param limitedOutputs 输出：限幅后的电流值
     */
    void LimitOutput(float pidOutputs[4], float limitedOutputs[4]);
    
    /**
     * @brief 应用功率限制到底盘电机 (高层封装)
     * @param can 用于更新CAN发送数据的CAN对象
     * @note 在MotorUpdateTask中调用，自动完成：
     *       1. 读取电机PID输出
     *       2. 功率限制计算
     *       3. 写回电机电流
     *       4. 更新CAN发送缓冲区
     */
    void ApplyToMotors(class CAN& can);
    
    /**
     * @brief 设置功率上限
     * @param maxPower 功率上限 (W)
     */
    void SetMaxPower(float maxPower);
    
    /**
     * @brief 获取当前功率上限
     */
    float GetMaxPower() const { return m_maxPower; }
    
    /**
     * @brief 获取预测功率
     */
    float GetEstimatedPower() const { return m_estimatedPower; }
    
    /**
     * @brief 获取实际功率 (来自功率计)
     */
    float GetMeasuredPower() const { return m_measuredPower; }
    
    /**
     * @brief 更新实测功率 (从功率计读取)
     * @param measuredPower 实测功率 (W)
     */
    void UpdateMeasuredPower(float measuredPower);
    
    /**
     * @brief 手动设置功率模型参数
     */
    void SetModelParams(float k1, float k2, float k3);
    
    /**
     * @brief 检查电机是否连接
     */
    bool IsMotorConnected(int index) const;

private:
    // RPM转角速度
    static float Rpm2Av(float rpm) { return rpm * 3.14159265f / 30.0f; }
    
    // 角速度转RPM
    static float Av2Rpm(float av) { return av * 30.0f / 3.14159265f; }

    // 电机轴RPM转输出轴角速度 (考虑减速比)
    // motor.curspeed 是电机轴转速，需要除以减速比得到输出轴转速
    static float MotorRpmToOutputAv(float motorRpm) {
        return (motorRpm / M3508_REDUCTION_RATIO) * 3.14159265f / 30.0f;
    }
    
    // 电流值转力矩 (输出轴)
    static float CurrentToTorque(float current);
    
    // 力矩转电流值
    static float TorqueToCurrent(float torque);
    
    // 计算单个电机功率
    float CalcMotorPower(float torque, float av) const;
    
    // 限幅函数
    static float Clamp(float value, float minVal, float maxVal);

private:
    Motor** m_motors = nullptr;     // 底盘电机指针数组
    
    float m_maxPower = 80.0f;       // 功率上限 (W)
    float m_estimatedPower = 0.0f;  // 预测功率 (W)
    float m_measuredPower = 0.0f;   // 实测功率 (W)
    
    // 功率模型参数
    float m_k1 = 0.22f;             // 转速损耗系数
    float m_k2 = 1.2f;              // 力矩平方损耗系数
    float m_k3 = 2.78f;             // 静态损耗
    
    // 力矩常数 (输出轴)
    float m_torqueConst = M3508_TORQUE_CONSTANT * M3508_REDUCTION_RATIO * M3508_CURRENT_TO_AMPERE;
    
    bool m_initialized = false;
};

// 全局功率控制器对象
extern PowerLimiter powerLimiter;
