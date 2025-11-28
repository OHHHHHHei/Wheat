#include "power_limit.h"
#include "Power_read.h"
#include "can.h"
#include "control.h"
#include <cmath>

// 全局功率控制器对象
PowerLimiter powerLimiter;

/**
 * @brief 电流值转力矩 (输出轴)
 * @param current 电机电流值 (-16384 ~ 16384)
 * @return 输出轴力矩 (N·m)
 */
float PowerLimiter::CurrentToTorque(float current)
{
    // current → 安培 → 输出轴力矩
    // M3508: 20A满电流对应约 6 N·m 输出力矩
    float ampere = current * M3508_CURRENT_TO_AMPERE;
    return ampere * M3508_TORQUE_CONSTANT;  // 直接用0.3，不乘减速比
}


/**
 * @brief 力矩转电流值
 * @param torque 输出轴力矩 (N·m)
 * @return 电机电流值
 */
float PowerLimiter::TorqueToCurrent(float torque)
{
    float ampere = torque / M3508_TORQUE_CONSTANT;  // 直接用0.3，不乘减速比
    return ampere / M3508_CURRENT_TO_AMPERE;
}

/**
 * @brief 限幅函数
 */
float PowerLimiter::Clamp(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/**
 * @brief 计算单个电机功率
 * @param torque 力矩 (N·m)
 * @param av 角速度 (rad/s)
 * @return 功率 (W)
 */
float PowerLimiter::CalcMotorPower(float torque, float av) const
{
    // P = τω + k1|ω| + k2τ? + k3/4
    return torque * av + m_k1 * fabsf(av) + m_k2 * torque * torque + m_k3 / 4.0f;
}

/**
 * @brief 检查电机是否连接
 */
bool PowerLimiter::IsMotorConnected(int index) const
{
    if (m_motors == nullptr || m_motors[index] == nullptr)
        return false;
    return m_motors[index]->getStatus() == FINE;
}

/**
 * @brief 初始化功率控制器
 */
void PowerLimiter::Init(Motor** motors, float maxPower, float k1, float k2, float k3)
{
    m_motors = motors;
    m_maxPower = maxPower;
    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;
    m_initialized = true;
}

/**
 * @brief 设置功率上限
 */
void PowerLimiter::SetMaxPower(float maxPower)
{
    m_maxPower = maxPower;
}

/**
 * @brief 设置功率模型参数
 */
void PowerLimiter::SetModelParams(float k1, float k2, float k3)
{
    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;
}

/**
 * @brief 更新实测功率
 */
void PowerLimiter::UpdateMeasuredPower(float measuredPower)
{
    m_measuredPower = measuredPower;
}

/**
 * @brief 应用功率限制到底盘电机
 */
void PowerLimiter::ApplyToMotors(CAN& can)
{
    if (!m_initialized || m_motors == nullptr)
        return;
    
    // 获取底盘电机PID原始输出
    float pidOutputs[4] = {
        static_cast<float>(m_motors[0]->current),
        static_cast<float>(m_motors[1]->current),
        static_cast<float>(m_motors[2]->current),
        static_cast<float>(m_motors[3]->current)
    };
    
    // 功率限制
    float limitedOutputs[4];
    LimitOutput(pidOutputs, limitedOutputs);
    
    // 应用限幅后的输出
    for (int i = 0; i < 4; i++)
    {
        m_motors[i]->current = static_cast<int32_t>(limitedOutputs[i]);
    }
    
    // 更新CAN发送数据 (底盘电机在can1_motor[0~3])
    for (int i = 0; i < 4; i++)
    {
        uint32_t idx = can1_motor[i].ID - 0x205;
        can.temp_data[idx * 2] = (m_motors[i]->current & 0xff00) >> 8;
        can.temp_data[idx * 2 + 1] = m_motors[i]->current & 0x00ff;
    }
}

/**
 * @brief 功率限制核心函数
 * @param pidOutputs 4个电机的PID原始输出 (电流值)
 * @param limitedOutputs 输出：限幅后的电流值
 */
void PowerLimiter::LimitOutput(float pidOutputs[4], float limitedOutputs[4])
{
    if (!m_initialized || m_motors == nullptr)
    {
        // 未初始化，直接透传
        for (int i = 0; i < 4; i++)
            limitedOutputs[i] = pidOutputs[i];
        return;
    }
    
    // ========== 第一步：计算预测功率 ==========
    float cmdPower[4] = {0};     // 每个电机预测功率
    float sumCmdPower = 0.0f;    // 总预测功率
    float error[4] = {0};        // 转速误差 |目标-实际|
    float sumError = 0.0f;       // 总误差
    float curAv[4] = {0};        // 当前角速度
    float setAv[4] = {0};        // 目标角速度
    
    for (int i = 0; i < 4; i++)
    {
        if (IsMotorConnected(i))
        {
            // 获取当前角速度 (电机轴rpm → 输出轴rad/s，需除以减速比19)
            curAv[i] = MotorRpmToOutputAv(static_cast<float>(m_motors[i]->curspeed));
            // 获取目标角速度 (setspeed也是电机轴转速)
            setAv[i] = MotorRpmToOutputAv(static_cast<float>(m_motors[i]->setspeed));
            
            // 计算力矩
            float torque = CurrentToTorque(pidOutputs[i]);
            
            // 计算预测功率
            cmdPower[i] = CalcMotorPower(torque, curAv[i]);
            
            // 只累加正功率
            if (cmdPower[i] > 0)
            {
                sumCmdPower += cmdPower[i];
            }
            
            // 计算转速误差
            error[i] = fabsf(setAv[i] - curAv[i]);
            sumError += error[i];
        }
        else
        {
            cmdPower[i] = 0;
            error[i] = 0;
            curAv[i] = 0;
            setAv[i] = 0;
        }
    }
    
    // 更新预测功率
    m_estimatedPower = sumCmdPower;
    
    // ========== 第二步：判断是否需要限制 ==========
    if (sumCmdPower <= m_maxPower)
    {
        // 不超功率，直接使用原始PID输出
        for (int i = 0; i < 4; i++)
        {
            if (IsMotorConnected(i))
                limitedOutputs[i] = pidOutputs[i];
            else
                limitedOutputs[i] = 0;
        }
        return;
    }
    
    // ========== 第三步：功率分配 ==========
    // 计算error置信度
    float errorConfidence = 0.0f;
    if (sumError > ERROR_DISTRIBUTION_UPPER)
    {
        errorConfidence = 1.0f;
    }
    else if (sumError > ERROR_DISTRIBUTION_LOWER)
    {
        errorConfidence = (sumError - ERROR_DISTRIBUTION_LOWER) / 
                          (ERROR_DISTRIBUTION_UPPER - ERROR_DISTRIBUTION_LOWER);
    }
    
    // 计算可分配功率
    float allocatablePower = m_maxPower;
    float sumPowerRequired = 0.0f;
    
    // 处理负功率电机（产生能量的电机不限制，并增加可分配功率）
    for (int i = 0; i < 4; i++)
    {
        if (cmdPower[i] <= 0)
        {
            allocatablePower += (-cmdPower[i]);
        }
        else
        {
            sumPowerRequired += cmdPower[i];
        }
    }
    
    // ========== 第四步：计算每个电机的最大允许力矩 ==========
    for (int i = 0; i < 4; i++)
    {
        if (!IsMotorConnected(i))
        {
            limitedOutputs[i] = 0;
            continue;
        }
        
        // 负功率电机不限制
        if (cmdPower[i] <= 0)
        {
            limitedOutputs[i] = pidOutputs[i];
            continue;
        }
        
        // 计算功率权重
        float powerWeight_Error = (sumError > 0.001f) ? (error[i] / sumError) : 0.25f;
        float powerWeight_Prop = (sumPowerRequired > 0.001f) ? (cmdPower[i] / sumPowerRequired) : 0.25f;
        float powerWeight = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
        
        // 分配给该电机的功率
        float allocatedPower = powerWeight * allocatablePower;
        
        // 求解一元二次方程: k2*τ? + ω*τ + (k1|ω| + k3/4 - P) = 0
        // A = k2, B = ω, C = k1|ω| + k3/4 - P
        float A = m_k2;
        float B = curAv[i];
        float C = m_k1 * fabsf(curAv[i]) + m_k3 / 4.0f - allocatedPower;
        
        float delta = B * B - 4.0f * A * C;
        float maxTorque;
        
        if (delta < 0)
        {
            // 无实数解，使用近似解
            maxTorque = -B / (2.0f * A);
        }
        else if (delta == 0)
        {
            // 重根
            maxTorque = -B / (2.0f * A);
        }
        else
        {
            // 两个实根，选择与原PID输出符号一致的解
            float sqrtDelta = sqrtf(delta);
            float torque1 = (-B + sqrtDelta) / (2.0f * A);
            float torque2 = (-B - sqrtDelta) / (2.0f * A);
            
            if (pidOutputs[i] > 0)
                maxTorque = torque1;  // 正输出取较大根
            else
                maxTorque = torque2;  // 负输出取较小根
        }
        
        // 转换回电流值
        float maxCurrent = TorqueToCurrent(maxTorque);
        
        // 限幅：取原始输出和最大允许输出的较小值（保持符号）
        if (pidOutputs[i] > 0)
        {
            limitedOutputs[i] = Clamp(pidOutputs[i], 0, fabsf(maxCurrent));
        }
        else
        {
            limitedOutputs[i] = Clamp(pidOutputs[i], -fabsf(maxCurrent), 0);
        }
        
        // 最终限幅到电机最大电流
        limitedOutputs[i] = Clamp(limitedOutputs[i], 
                                  -static_cast<float>(m_motors[i]->maxcurrent),
                                  static_cast<float>(m_motors[i]->maxcurrent));
    }
}
