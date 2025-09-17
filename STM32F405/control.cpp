#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
void CONTROL::Init(std::vector<Motor*> motor) //��ʼ��
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
	ctrl.mode = SEPARATE; //��ʼ��Ϊ����ģʽ

	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;  //pitch��ʼ��
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;  //yaw��ʼ��
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)  //��̨����
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//�����෴�޸���������
	float adjangle = this->pantile.sensitivity * 2; //sensitivity�ǻ����������� ������̨�������ȳ�2���������

	ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);//�ı�pitchĿ��ֵ
	ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);//�ı�yawĿ��ֵ
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)//������̨�̶��ھ���λ�ã�С����ʱʹ��
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)//����YAW����
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw())); //�����������趨�Ƕ������ڽǶ����ҹ�һ�������·��
		if (delta <= -4096.f)//��е�ǹ�һ��
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8291.f;
		if (abs(delta) >= 10.f) //�������ã�����С���
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta); //����ʽPID����
	}
	else if (type==PITCH)// PITCH������ƣ�����ͬ��
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

void CONTROL::CHASSIS::Keep_Direction() //ʹ�õ����˶���������̨������������С����ʱʹ��
{
	double s_x = speedx, s_y = speedy;//����ԭʼ��ҡ�������ٶ�
	double theat = ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])// ������̨����ڵ��̵���ת�Ƕ�theta
					- mechanicalToDegree(para.initial_yaw)) * PI / 180.f;//initial_yaw�ǳ�ʼ��ʱȷ���ģ����ǵ�����ǰ����Ӧ����̨��yawֵ��ת��Ϊ������
	double st = sin(theat);//����Ƕȵ����Һ�����ֵ
	double ct = cos(theat);
	//Ӧ�ö�ά��ת����ʽ��speedx �� speedy ����Ϊ����������ϵ���µ���ȷ�ٶ�
	speedx = s_x * ct - s_y * st;
	speedy = s_x * st + s_y * ct;
}

void CONTROL::manual_chassis(int32_t _speedx, int32_t _speedy, int32_t _speedz)//���̿���
{
	_speedx *= 1;
	_speedy *= -1;
	_speedz *= -1;//�����෴�������޸�����

	float setX, setY, setZ;
	//�������ٶ�
	float _total_speed = sqrt(_speedx * _speedx + _speedy * _speedy + _speedz * _speedz);
	//����Ƿ���
	if (_total_speed > 9000)
	{
		float scale = 9000 / _total_speed;//���ű������ض�С��1
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
	//��������ֵ������Ӧspeed
	total_speed = sqrt(setX * setX + setY * setY + setZ * setZ);
	this->chassis.speedx = setX;
	this->chassis.speedy = setY;
	this->chassis.speedz = setZ;
}

void CONTROL::CHASSIS::Update() 
{
	if (ctrl.mode == RESET) //reset״̬����
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	if (ctrl.mode == ROTATION)
	{
		Keep_Direction();
	}

	//�˶�ѧ����
	ctrl.chassis_motor[0]->setspeed = Ramp(speedy + speedx - speedz, ctrl.chassis_motor[0]->setspeed, 30);
	ctrl.chassis_motor[1]->setspeed = Ramp(-speedy + speedx - speedz, ctrl.chassis_motor[1]->setspeed, 30);
	ctrl.chassis_motor[2]->setspeed = Ramp(-speedy - speedx - speedz, ctrl.chassis_motor[2]->setspeed, 30);
	ctrl.chassis_motor[3]->setspeed = Ramp(speedy - speedx - speedz, ctrl.chassis_motor[3]->setspeed, 30);
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode == RESET)// resetģʽ��ʼ��yaw��pitch
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;
	}

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;//�����ƣ���һ��
	if (mark_yaw < 0.0)mark_yaw += 8192.0;
	//��pitch������λ
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

float CONTROL::GetDelta(float delta) //����Ƕ����·��
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
