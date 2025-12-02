#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"

class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};
	
	enum MODE { RESET, ROTATION, SEPARATE, FOLLOW, LOCK, TEST, AUTO, SHOOT, BLANK, SHENGSAI};
	MODE mode[2];
	float total_speed;

	struct CHASSIS
	{


		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};
		
		void Keep_Direction();

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
		float Ramp_plus(float setval, float curval, float Increase_Value, float Decrease_Value);
	};

	struct PANTILE
	{
		enum TYPE { YAW, PITCH };
		Kalman yawKalman{ 0.5f, 40.f };
		float mark_pitch{}, mark_yaw{}, markImuYaw{}, initialImuYaw{};//各个目标角度
		PID pantile_PID[3] = { {0.15f,0.f,0.f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} };//YAW PITCH
		const float sensitivity = 0.01f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		void Update();
		int Imucount;
		bool imu_err_flag = false;
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		bool manual_shoot = false;
		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6000;
		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;

	// 自瞄前馈参数
	struct AUTOAIM_FEEDFORWARD
	{
		float yaw_vel_ff = 1.48f;      // yaw速度前馈系数
		float yaw_acc_ff = 0.1f;      // yaw加速度前馈系数  
		float pitch_vel_ff = 0.05f;    // pitch速度前馈系数
		float pitch_acc_ff = 0.1f;    // pitch加速度前馈系数
	} autoaim_ff;
	
	// 小陀螺模式云台Yaw轴前馈补偿参数
	// 原理：根据底盘旋转角速度计算前馈电流，主动抵消底盘带动云台的干扰力矩
	struct ROTATION_FEEDFORWARD
	{
		float ff_gain = 800.0f;        // 前馈增益系数（需实际调试）
		float chassis_omega = 0.0f;    // 底盘当前旋转角速度 (rad/s)，由轮子转速反解
		float ff_current = 0.0f;       // 前馈电流输出值
	} rotation_ff;

	static int16_t Setrange(const int16_t original, const int16_t range);
	void manual_chassis(int32_t speedx, int32_t speedy, int32_t speedz);
	void Control_Pantile(float_t ch_yaw, float_t ch_pitch);
	void Control_AutoAim();  // 自瞄控制函数
	void CalcRotationFeedforward();  // 计算小陀螺前馈补偿
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();
private:

};

extern CONTROL ctrl;