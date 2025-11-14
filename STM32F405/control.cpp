#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "xuc.h"
#include "RC.h"

float cmd_pitch;
float cmd_yaw;

void CONTROL::Init(std::vector<Motor*> motor) //初始化
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	ctrl.mode[now] = SEPARATE; //初始化为分离模式

	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;  //pitch初始化
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;  //yaw初始化
	DMmotor->setSpeed = 4;
	DMmotor[0].setPos = para.initial_pitch;//达妙pitch初始化
}


void CONTROL::Control_Pantile(float_t ch_yaw, float_t ch_pitch)  //云台控制
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float pitch_adjangle = this->pantile.sensitivity / 2.f; //sensitivity是基础的灵敏度。
	float yaw_adjangle = this->pantile.sensitivity * 20.f;
	

	//小陀螺的云台控制
	if (ctrl.mode[now] == CONTROL::ROTATION)
	{
		if (ctrl.mode[pre] != CONTROL::ROTATION)
		{
			// 如果是，说明这是进入小陀螺模式的第一帧。
			// 此时，读取IMU的当前绝对角度，并将其设置为目标角度。
			pantile.markImuYaw = imu_pantile.GetAngleYaw();
		}

		// 机械角控制云台的小陀螺控制逻辑
		if (pantile_motor[0]->mode == POS)
		{
			// YAW轴：根据摇杆输入更新IMU目标角度，使用绝对角度控制（抵消底盘旋转）
			pantile.markImuYaw -= ch_yaw;
			pantile.Keep_Pantile(pantile.markImuYaw, CONTROL::PANTILE::TYPE::YAW, imu_pantile);
			// PITCH轴：不受底盘旋转影响，直接使用机械角度控制
			ctrl.pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);
		}

		// 陀螺仪控制云台的小陀螺控制逻辑
		if (pantile_motor[0]->mode == POS2)
		{
			// YAW轴：根据摇杆输入更新IMU目标角度
			pantile.markImuYaw = GetDelta(pantile.markImuYaw - ch_yaw * yaw_adjangle);
			// PITCH控制
			ctrl.pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);
		}

	}
	else {
		//陀螺仪控制云台控制逻辑
		if (pantile_motor[0]->mode == POS2)
		{
			// YAW轴：根据摇杆输入更新IMU目标角度
			pantile.markImuYaw = GetDelta(pantile.markImuYaw - ch_yaw * yaw_adjangle);
		}
		else if (pantile_motor[0]->mode == POS)//机械角控制
		{
			pantile.mark_yaw -= (float)(yaw_adjangle * ch_yaw);
		}

		//PITCH控制
		pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);

		//ctrl.pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);//改变pitch目标值
		//ctrl.pantile.mark_yaw -= (float)(yaw_adjangle * ch_yaw);//改变yaw目标值
	}
	//更新模式数据
	mode[pre] = mode[now];
}

//保持云台固定在绝对位置，机械角小陀螺时使用
void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)//控制YAW方向
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw())); //计算陀螺仪设定角度与现在角度误差并且归一化到最短路径
		if (delta <= -4096.f)//机械角归一化
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8291.f;
		if (abs(delta) >= 10.f) //死区设置，忽略小误差
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta); //增量式PID控制
	}
	else if (type==PITCH)// PITCH方向控制，方法同上
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));

		if (delta <= -4096.f)
		{
			delta += 8192.f;
		}
		else if (delta >= 4096.f)
		{
			delta -= 8192.f;
		}
			
		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);//增量式PID控制
		}
	}
}

// 使得底盘运动方向按照云台正方向修正，小陀螺时使用
void CONTROL::CHASSIS::Keep_Direction()
{
	double s_x = speedx, s_y = speedy;//记录原始的摇杆输入速度

	double theat = (-1.f) * ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])// 计算云台相对于底盘的旋转角度theta
					- mechanicalToDegree(para.initial_yaw)) * PI / 180.f;//initial_yaw是初始化时确定的，就是底盘正前方对应的云台的yaw值，转化为弧度制

	double st = sin(theat);//计算角度的正弦和余弦值
	double ct = cos(theat);

	//应用二维旋转矩阵公式，speedx 和 speedy 更新为“车体坐标系”下的正确速度
	speedx = s_x * ct - s_y * st;
	speedy = s_x * st + s_y * ct;
}

void CONTROL::manual_chassis(int32_t _speedx, int32_t _speedy, int32_t _speedz)//底盘控制，输出speedx, y, z,还需要经过运动学解算分配到各个电机
{
	_speedx *= 1;
	_speedy *= -1;
	_speedz *= -1;//方向相反在这里修改正负

	float setX, setY, setZ;
	//计算总速度
	float _total_speed = sqrt(_speedx * _speedx + _speedy * _speedy + _speedz * _speedz);
	//检查是否超速，9000为最大速度
	if (_total_speed > 9000)
	{
		float scale = 9000 / _total_speed;//缩放比例，必定小于1
		setX = _speedx * scale;
		setY = _speedy * scale;
		setZ = _speedz * scale;
	}
	else
	{
		setX = _speedx;
		setY = _speedy;
		setZ = _speedz;
	}
	//将处理后的值赋给对应speed
	total_speed = sqrt(setX * setX + setY * setY + setZ * setZ);
	this->chassis.speedx = setX;
	this->chassis.speedy = setY;
	this->chassis.speedz = setZ;
}

void CONTROL::CHASSIS::Update() 
{	
	//RESET时，底盘速度置零
	if (ctrl.mode[now] == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	//运动学解算
	ctrl.chassis_motor[0]->setspeed = Ramp_plus(+speedy + speedx - speedz, ctrl.chassis_motor[0]->setspeed, 25, 80);
	ctrl.chassis_motor[1]->setspeed = Ramp_plus(-speedy + speedx - speedz, ctrl.chassis_motor[1]->setspeed, 25, 80);
	ctrl.chassis_motor[2]->setspeed = Ramp_plus(-speedy - speedx - speedz, ctrl.chassis_motor[2]->setspeed, 25, 80);
	ctrl.chassis_motor[3]->setspeed = Ramp_plus(+speedy - speedx - speedz, ctrl.chassis_motor[3]->setspeed, 25, 80);
}

void CONTROL::PANTILE::Update()
{
	//错误处理，陀螺仪断电过久
	static float last_AngleYaw = 0;
	static float last_AnglePitch = 0;
	static float last_AngleRoll = 0;
	if (last_AngleYaw == imu_pantile.GetAngleYaw() && last_AnglePitch == imu_pantile.GetAnglePitch() && last_AngleRoll == imu_pantile.GetAngleRoll())
	{
		Imucount++;
		if (Imucount > 6000)
		{
			imu_err_flag = true;
			return;
		}
	}
	else
	{
		Imucount = 0;
		imu_err_flag = false;
	}
	last_AngleYaw = imu_pantile.GetAngleYaw();
	last_AnglePitch = imu_pantile.GetAnglePitch();
	last_AngleRoll = imu_pantile.GetAngleRoll();


	//更新云台电机对应IMU值
	ctrl.pantile_motor[0]->imuValue = imu_pantile.GetAngleYaw();

	//更新对正时候的陀螺仪角度
	if (fabs(ctrl.pantile_motor[PANTILE::YAW]->angle[now] - para.initial_yaw) < 5.f)
	{
		initialImuYaw = imu_pantile.GetAngleYaw();
	}

	// reset模式初始化yaw和pitch
	if (ctrl.mode[now] == RESET)
	{
		markImuYaw = initialImuYaw;
		mark_pitch = para.initial_pitch;
	}

	//处理环绕，归一化，使用机械角时
	if (mark_yaw > 8192.0)
	{
		mark_yaw -= 8192.0;
	}
	if (mark_yaw < 0.0)
	{
		mark_yaw += 8192.0;
	}

	//对pitch进行限位
	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

	//输出给电机YAW和PITCH
	ctrl.pantile_motor[0]->setImuValue = markImuYaw;

	/*ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw;*/

	ctrl.pantile_motor[PANTILE::PITCH]->setangle = mark_pitch;

	DMmotor[0].setPos = mark_pitch;
}

void CONTROL::SHOOTER::Update()
{
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
	if (ctrl.mode[now] == RESET)
	{
		openRub = false;
		supply_bullet = false;
		auto_shoot = false;
	}
	if (openRub)//开火控制摩擦轮
	{
		ctrl.shooter_motor[0]->setspeed = 6000;
		ctrl.shooter_motor[1]->setspeed = -6000;
	}
	else//停止摩擦轮
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
	}

	//if (supply_bullet && openRub)//如果开火和供弹
	//{
	//	if (auto_shoot && manual_shoot)//如果火控和操作手同时同意开火，则开火(双重火控)
	//	{
	//		//ctrl.supply_motor[0]->setspeed = 1000;
	//		//ctrl.supply_motor[0]->spinning = true;//spining一秒八发
	//	}
	//	else
	//	{
	//		ctrl.supply_motor[0]->setspeed = -1000;
	//		ctrl.supply_motor[0]->spinning = true;
	//	}
	//}
	//else 
	//{
	//	ctrl.supply_motor[0]->spinning = false;
	//	ctrl.supply_motor[1]->spinning = false;
	//}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::CHASSIS::Ramp_plus(float setval, float curval, float Increase_Value, float Decrease_Value)
{
	if (abs(setval) - abs(curval) >= 0)
	{
		if (setval - curval > 0)
		{
			curval += Increase_Value;
			curval = std::min(curval, setval);
		}
		else
		{
			curval -= Increase_Value;
			curval = std::max(curval, setval);
		}

	}
	else
	{
		if (setval - curval > 0)
		{
			curval += Decrease_Value;
			curval = std::min(curval, setval);
		}
		else
		{
			curval -= Decrease_Value;
			curval = std::max(curval, setval);
		}
	}

	return curval;
}

//计算角度最短路径,0-360度
float CONTROL::GetDelta(float delta) 
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

//限幅函数
int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

void CONTROL::Control_AutoAim()//自瞄控制函数
{
	// 检查视觉系统是否有目标数据
	if (xuc.RxNuc_TJ.mode_TJ != 0)  //表示是否检测到目标
	{
		// 读取视觉系统提供的目标信息（弧度制）
		float target_yaw = xuc.RxNuc_TJ.yaw_TJ;         // 目标yaw角度（弧度）
		float target_pitch = xuc.RxNuc_TJ.pitch_TJ;     // 目标pitch角度（弧度）
		float target_yaw_vel = xuc.RxNuc_TJ.yaw_vel_TJ;   // 目标yaw角速度（弧度/秒）
		float target_pitch_vel = xuc.RxNuc_TJ.pitch_vel_TJ; // 目标pitch角速度（弧度/秒）
		float target_yaw_acc = xuc.RxNuc_TJ.yaw_acc_TJ;   // 目标yaw角加速度（弧度/秒²）
		float target_pitch_acc = xuc.RxNuc_TJ.pitch_acc_TJ; // 目标pitch角加速度（弧度/秒²）

		//yaw用陀螺仪的值来控制
		cmd_yaw = target_yaw * 57.3 - 180;
		//pitch用弧度制来控制
		cmd_pitch = target_pitch;
		// 计算yaw轴前馈补偿
		float yaw_vel_feedforward = autoaim_ff.yaw_vel_ff * target_yaw_vel / (2 * PI) * 8192;
		float yaw_acc_feedforward = autoaim_ff.yaw_acc_ff * target_yaw_acc / (2 * PI) * 8192;

		// 计算pitch轴前馈补偿
		float pitch_vel_feedforward = autoaim_ff.pitch_vel_ff * target_pitch_vel / (2 * PI) * 8192;
		float pitch_acc_feedforward = autoaim_ff.pitch_acc_ff * target_pitch_acc / (2 * PI) * 8192;

		// 更新云台目标角度（目标角度 + 速度前馈 + 加速度前馈）
		pantile.markImuYaw = cmd_yaw;
		//+ yaw_vel_feedforward + yaw_acc_feedforward;
		
		pantile.mark_pitch = cmd_pitch;
		// +pitch_vel_feedforward + pitch_acc_feedforward;

		// 火控逻辑
		// mode_TJ: 0=不控制, 1=控制云台但不开火, 2=控制云台且开火
		if (xuc.RxNuc_TJ.mode_TJ == 2)
		{
			// 视觉系统请求开火
			shooter.openRub = true;        // 启动摩擦轮
			shooter.supply_bullet = true;  // 启动供弹
			shooter.auto_shoot = true;     // 火控同意射击

			// 双重火控模式，上位机和遥控器同时发出供弹指令再开始供弹
			if (abs(rc.rc.ch[3]) > 330)
			{
				shooter.manual_shoot = true;
			}
			else {
				shooter.manual_shoot = false;
			}

		}
		else if (xuc.RxNuc_TJ.mode_TJ == 1)
		{
			// 只控制云台，不拨弹但保持摩擦轮运转（快速响应）
			shooter.openRub = true;        // 保持摩擦轮运转
			shooter.supply_bullet = false; // 停止供弹
			shooter.auto_shoot = false;    // 关闭自动射击
			supply_motor[0]->setspeed = 0;   // 供弹停止

			
		}
	}
	else
	{
		// 没有检测到目标，停止射击, 手动控制云台
		shooter.openRub = false;
		shooter.supply_bullet = false;
		shooter.auto_shoot = false;
		supply_motor[0]->setspeed = 0;   // 供弹停止
		Control_Pantile(rc.rc.ch[2] * para.yaw_speed / 660.f, -rc.rc.ch[3] * para.pitch_speed / 660.f); // 云台控制
	}
}

extern uint8_t Power_stsRx[];
