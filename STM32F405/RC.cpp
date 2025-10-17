#include "label.h"
#include "RC.h"
#include "HTmotor.h"
#include "control.h"

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // ���߱���
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}

	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//�����ĸ�λ����R F G Z X C 
	pc.key_l = m_frame[14];//�����ĵ�8λ W S A D SHIFT CTRL Q E

	
}

void RC::OnRC()
{
	//��������ģʽ�ĵ�λ
	if (rc.s[0] == MID && rc.s[1] == MID)
	{

	}
	else if (rc.s[0] == UP && rc.s[1] == MID)//����ģʽ   ����
	{
		ctrl.mode[now] = CONTROL::SEPARATE;
		//DMmotor[0].CanComm_ControlCmd(can1, CMD_CLEAR_MODE, 1 + MOTOR_MODE);//�������
	}
	else if (rc.s[0] == UP && rc.s[1] == UP)//С����ģʽ    ����
	{
		ctrl.mode[now] = CONTROL::ROTATION;
	}
	else if (rc.s[0] == MID && rc.s[1] == UP)//����ģʽ    ����
	{
		ctrl.mode[now] = CONTROL::AUTO;
	}
	else if (rc.s[0] == DOWN && rc.s[1] == DOWN)//��������   ����
	{
		ctrl.mode[now] = CONTROL::SHOOT;
	}
	else if (rc.s[0] == DOWN && rc.s[1] == UP)
	{

	}
	else if (rc.s[0] == DOWN && rc.s[1] == MID)  //RESETģʽ    ����
	{
		ctrl.mode[now] = CONTROL::RESET;
	}

	else if (rc.s[0] == MID && rc.s[1] == DOWN)
	{

	}
	else if (rc.s[0] == UP && rc.s[1] == DOWN)
	{
		;
	}
	if (Shift_mode())
	{

	}

	//�������
	if (ctrl.mode[now] != CONTROL::RESET)
	{
		switch (ctrl.mode[now])
		{
		case CONTROL::SEPARATE: // ����ģʽ
		{
			ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, -rc.ch[3] * para.pitch_speed / 660.f); // ��̨����
			ctrl.manual_chassis(rc.ch[1] * para.max_speed / 660.f, 0, rc.ch[0] * para.max_speed / 660.f);   // ����ģʽ���Ƕ���Y�᷽�����
			// ��ȫ�ر����ϵͳ
			ctrl.shooter.openRub = false;
			ctrl.supply_motor[0]->setspeed = 0;
			break;
		}

		case CONTROL::ROTATION: // С����ģʽ
		{
			// ��С����ģʽ�������ٶ�������������ҡ���Ƴ��ľ����С����¼�Ƹ���������¼����
			float RCv_xy = sqrt(pow(rc.ch[1], 2.f) + pow(rc.ch[0], 2.f)) / 660.f;
			// ƽ�ƻή��ת�٣�������ǰ��������һ��ת�����ֲ������ʧ
			ctrl.manual_chassis(rc.ch[1] * MAXSPEED / 660, -rc.ch[0] * MAXSPEED / 660, para.rota_speed + RCv_xy);
			ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, -rc.ch[3] * para.pitch_speed / 660.f); // ��̨����
			ctrl.chassis.Keep_Direction(); // ����������
			// ��ȫ�ر����ϵͳ
			ctrl.shooter.openRub = false;
			ctrl.supply_motor[0]->setspeed = 0;
			break;
		}

		case CONTROL::SHOOT: // ����ģʽ
		{
			ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, -rc.ch[3] * para.pitch_speed / 660.f); // ��̨����
			ctrl.shooter.openRub = true;//��Ħ����
			//��������
			if (abs(rc.ch[0]) > 330)
			{
				ctrl.supply_motor[0]->setspeed = -2500;//����
			}
			else
			{
				ctrl.supply_motor[0]->setspeed = 0;
			}
			/*if (abs(rc.ch[0]) > 330)
			{
				ctrl.shooter.openRub = true;
			}
			else
			{
				ctrl.shooter.openRub = false;
			}*/
			break;
		}

		case CONTROL::AUTO:
		{
			ctrl.Control_AutoAim();  // ����������ƺ���
			//��������
			if (abs(rc.ch[2]) > 330)
			{
				ctrl.supply_motor[0]->setspeed = -2500;//����
			}
			else
			{
				ctrl.supply_motor[0]->setspeed = 0;
			}
			ctrl.manual_chassis(rc.ch[1] * para.max_speed / 660.f, 0, rc.ch[0] * para.max_speed / 660.f);   // ����Y�᷽�����
		}

		default:
		{
			// ��������δָ����ģʽ������ʲô������
			break;
		}
		}
	}


}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
