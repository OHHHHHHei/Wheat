#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "xuc.h"

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
	ctrl.mode[now] = SEPARATE; //��ʼ��Ϊ����ģʽ

	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;  //pitch��ʼ��
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;  //yaw��ʼ��
	DMmotor->setSpeed = 4;
	DMmotor[0].setPos = para.initial_pitch;//����pitch��ʼ��
}


void CONTROL::Control_Pantile(float_t ch_yaw, float_t ch_pitch)  //��̨����
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//�����෴�޸���������
	float pitch_adjangle = this->pantile.sensitivity; //sensitivity�ǻ����������ȡ�
	float yaw_adjangle = this->pantile.sensitivity * 1000;


	//С���ݵ���̨����
	if (ctrl.mode[now] == CONTROL::ROTATION)
	{
		if (ctrl.mode[pre] != CONTROL::ROTATION)
		{
			// ����ǣ�˵�����ǽ���С����ģʽ�ĵ�һ֡��
			// ��ʱ����ȡIMU�ĵ�ǰ���ԽǶȣ�����������ΪĿ��Ƕȡ�
			pantile.markImuYaw = imu_pantile.GetAngleYaw();
		}

		if (pantile_motor[0]->mode == POS)
		{
			pantile.Keep_Pantile(pantile.markImuYaw - ch_yaw, CONTROL::PANTILE::TYPE::YAW, imu_pantile);//������̨�ȶ�������pitch����
		}
	}
	else {
		ctrl.pantile.mark_pitch -= (float)(pitch_adjangle * ch_pitch);//�ı�pitchĿ��ֵ
		ctrl.pantile.mark_yaw -= (float)(yaw_adjangle * ch_yaw);//�ı�yawĿ��ֵ
	}

	//����ģʽ����
	mode[pre] = mode[now];
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
		{
			delta += 8192.f;
		}
		else if (delta >= 4096.f)
		{
			delta -= 8192.f;
		}
			
		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);//����ʽPID����
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction() //ʹ�õ����˶���������̨������������С����ʱʹ��
{
	double s_x = speedx, s_y = speedy;//��¼ԭʼ��ҡ�������ٶ�

	double theat = (-1.f) * ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])// ������̨����ڵ��̵���ת�Ƕ�theta
					- mechanicalToDegree(para.initial_yaw)) * PI / 180.f;//initial_yaw�ǳ�ʼ��ʱȷ���ģ����ǵ�����ǰ����Ӧ����̨��yawֵ��ת��Ϊ������

	double st = sin(theat);//����Ƕȵ����Һ�����ֵ
	double ct = cos(theat);

	//Ӧ�ö�ά��ת����ʽ��speedx �� speedy ����Ϊ����������ϵ���µ���ȷ�ٶ�
	speedx = s_x * ct - s_y * st;
	speedy = s_x * st + s_y * ct;
}

void CONTROL::manual_chassis(int32_t _speedx, int32_t _speedy, int32_t _speedz)//���̿��ƣ����speedx, y, z,����Ҫ�����˶�ѧ������䵽�������
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
	if (ctrl.mode[now] == RESET) //reset״̬����
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	//�˶�ѧ����
	ctrl.chassis_motor[0]->setspeed = Ramp_plus(+speedy + speedx - speedz, ctrl.chassis_motor[0]->setspeed, 25, 80);
	ctrl.chassis_motor[1]->setspeed = Ramp_plus(-speedy + speedx - speedz, ctrl.chassis_motor[1]->setspeed, 25, 80);
	ctrl.chassis_motor[2]->setspeed = Ramp_plus(-speedy - speedx - speedz, ctrl.chassis_motor[2]->setspeed, 25, 80);
	ctrl.chassis_motor[3]->setspeed = Ramp_plus(+speedy - speedx - speedz, ctrl.chassis_motor[3]->setspeed, 25, 80);
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode[now] == RESET)// resetģʽ��ʼ��yaw��pitch
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;
	}

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;//�����ƣ���һ��
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	//��pitch������λ
	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

	//��������YAW��PITCH
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

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)//�޷�����
{
	return fmaxf(fminf(range, original), -range);
}

void CONTROL::Control_AutoAim()//������ƺ���
{
	// ����Ӿ�ϵͳ�Ƿ���Ŀ������
	if (xuc.track_flag == true)  // xuc.track_flag��ʾ�Ƿ��⵽Ŀ��
	{
		// ��ȡ�Ӿ�ϵͳ�ṩ��Ŀ����Ϣ
		float target_yaw = xuc.yaw;        // Ŀ��yaw�Ƕ�
		float target_pitch = xuc.pitch;    // Ŀ��pitch�Ƕ�
		float yaw_diff = xuc.yaw_diff;     // yaw�ǶȲ�
		float pitch_diff = xuc.pitch_diff; // pitch�ǶȲ�

		// �򵥵ı������� - ���ǶȲ�ת��Ϊ��̨��������
		float yaw_control_increment = yaw_diff * 0.5f;   // 0.5�Ǳ���ϵ�����ɵ���
		float pitch_control_increment = pitch_diff * 0.5f;

		// ������̨Ŀ��Ƕ�
		pantile.mark_yaw += yaw_control_increment;
		pantile.mark_pitch += pitch_control_increment;

		// �򵥵��޷�����
		pantile.mark_yaw = Setrange(static_cast<int16_t>(pantile.mark_yaw), 4096);
		pantile.mark_pitch = Setrange(static_cast<int16_t>(pantile.mark_pitch), 2000);
	}
	// ���û�м�⵽Ŀ�꣬���ֵ�ǰ��̨λ�ò���
}

extern uint8_t Power_stsRx[];
