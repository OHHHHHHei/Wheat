#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
void CONTROL::Init(std::vector<Motor*> motor)
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
	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 2;

	ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);
	ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8291.f;
		if (abs(delta) >= 10.f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);
	}
	else if (type==PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;

		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	double s_x = speedx, s_y = speedy;
	double theat = ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])
					- mechanicalToDegree(para.initial_yaw)) / 180.f;
	double st = sin(theat);
	double ct = cos(theat);
	speedx = s_x * ct - s_y * st;
	speedy = s_x * st + s_y * ct;
}

void CONTROL::CHASSIS::Update() // 运动学解算
{
	if (ctrl.mode == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	ctrl.chassis_motor[0]->setspeed = Ramp(speedy + speedx - speedz, ctrl.chassis_motor[0]->setspeed, 8);
	ctrl.chassis_motor[1]->setspeed = Ramp(-speedy + speedx - speedz, ctrl.chassis_motor[1]->setspeed, 8);
	ctrl.chassis_motor[2]->setspeed = Ramp(-speedy - speedx - speedz, ctrl.chassis_motor[2]->setspeed, 8);
	ctrl.chassis_motor[3]->setspeed = Ramp(speedy - speedx - speedz, ctrl.chassis_motor[3]->setspeed, 8);
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode == RESET)
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;
	}

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

	ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw;
	ctrl.pantile_motor[PANTILE::PITCH]->setangle = mark_pitch;
}

void CONTROL::SHOOTER::Update()
{
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
	if (ctrl.mode == RESET)
	{
		openRub = false;
		supply_bullet = false;
		auto_shoot = false;
	}
	if (openRub)
	{
		ctrl.shooter_motor[0]->setspeed = -speed ;
		ctrl.shooter_motor[1]->setspeed = speed ;
	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
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

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];
