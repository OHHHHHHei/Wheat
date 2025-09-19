#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
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


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)  //云台控制
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 2; //sensitivity是基础的灵敏度 这里云台的灵敏度乘2会更灵敏。


	//小陀螺的云台控制
	if (ctrl.mode[now] == CONTROL::ROTATION)
	{
		if (ctrl.mode[pre] != CONTROL::ROTATION)
		{
			// 如果是，说明这是进入小陀螺模式的第一帧。
			// 此时，读取IMU的当前绝对角度，并将其设置为目标角度。
			pantile.markImuYaw = imu_pantile.GetAngleYaw();
		}

		if (pantile_motor[0]->mode == POS)
		{
			pantile.Keep_Pantile(pantile.markImuYaw - ch_yaw, CONTROL::PANTILE::TYPE::YAW, imu_pantile);//保持云台稳定，包含pitch控制
		}
	}
	else {
		ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);//改变pitch目标值
		ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);//改变yaw目标值
	}

	//更新模式数据
	mode[pre] = mode[now];
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)//保持云台固定在绝对位置，小陀螺时使用
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
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 10.f)
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
	}
}

void CONTROL::CHASSIS::Keep_Direction() //使得底盘运动方向按照云台正方向修正，小陀螺时使用
{
	double s_x = speedx, s_y = speedy;//保存原始的摇杆输入速度
	double theat = (-1.f) * ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])// 计算云台相对于底盘的旋转角度theta
					- mechanicalToDegree(para.initial_yaw)) * PI / 180.f;//initial_yaw是初始化时确定的，就是底盘正前方对应的云台的yaw值，转化为弧度制
	double st = sin(theat);//计算角度的正弦和余弦值
	double ct = cos(theat);
	//应用二维旋转矩阵公式，speedx 和 speedy 更新为“车体坐标系”下的正确速度
	speedx = s_x * ct - s_y * st;
	speedy = s_x * st + s_y * ct;
}

void CONTROL::manual_chassis(int32_t _speedx, int32_t _speedy, int32_t _speedz)//底盘控制
{
	_speedx *= 1;
	_speedy *= -1;
	_speedz *= -1;//方向相反在这里修改正负

	float setX, setY, setZ;
	//计算总速度
	float _total_speed = sqrt(_speedx * _speedx + _speedy * _speedy + _speedz * _speedz);
	//检查是否超速
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
	if (ctrl.mode[now] == RESET) //reset状态置零
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	//运动学解算
	ctrl.chassis_motor[0]->setspeed = Ramp(speedy + speedx - speedz, ctrl.chassis_motor[0]->setspeed, 30);
	ctrl.chassis_motor[1]->setspeed = Ramp(-speedy + speedx - speedz, ctrl.chassis_motor[1]->setspeed, 30);
	ctrl.chassis_motor[2]->setspeed = Ramp(-speedy - speedx - speedz, ctrl.chassis_motor[2]->setspeed, 30);
	ctrl.chassis_motor[3]->setspeed = Ramp(speedy - speedx - speedz, ctrl.chassis_motor[3]->setspeed, 30);
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode[now] == RESET)// reset模式初始化yaw和pitch
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;
	}

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;//处理环绕，归一化
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	//对pitch进行限位
	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

	ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw;
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
	if (openRub)
	{
		ctrl.shooter_motor[0]->setspeed = 6000;
		ctrl.shooter_motor[1]->setspeed = -6000;
		ctrl.supply_motor[0]->setspeed = -2500;
	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
		ctrl.supply_motor[0]->setspeed = 0;
	}

	if (supply_bullet && openRub)
	{
		if (auto_shoot)
		{
			ctrl.supply_motor[0]->setspeed = 2160;
			ctrl.supply_motor[0]->spinning = true;
		}
		else
		{
			ctrl.supply_motor[0]->setspeed = 2160;
			ctrl.supply_motor[0]->spinning = true;
		}
	}
	else 
	{
		ctrl.supply_motor[0]->spinning = false;
		ctrl.supply_motor[1]->spinning = false;
	}
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

float CONTROL::GetDelta(float delta) //计算角度最短路径
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

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];
