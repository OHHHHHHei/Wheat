#include "label.h"
#include "RC.h"
#include "HTmotor.h"
#include "control.h"

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
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

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

	
}

void RC::OnRC()
{
	//调整各种模式的档位
	if (rc.s[0] == MID && rc.s[1] == MID)
	{
		ctrl.mode[now] = CONTROL::RESET;
	}
	else if (rc.s[0] == UP && rc.s[1] == MID)//分离模式
	{
		ctrl.mode[now] = CONTROL::SEPARATE;
		//DMmotor[0].CanComm_ControlCmd(can1, CMD_CLEAR_MODE, 1 + MOTOR_MODE);//清除错误
	}
	else if (rc.s[0] == UP && rc.s[1] == UP)//小陀螺模式
	{
		ctrl.mode[now] = CONTROL::ROTATION;
	}
	else if (rc.s[0] == MID && rc.s[1] == UP)//底盘跟随云台
	{
		ctrl.mode[now] = CONTROL::FOLLOW;
	}
	else if (rc.s[0] == DOWN && rc.s[1] == DOWN)//单独开火
	{
		ctrl.mode[now] = CONTROL::FOLLOW;
		ctrl.shooter.openRub = true;
		if (abs(rc.ch[0]) > 330)
		{
			ctrl.supply_motor[0]->setspeed = -2500;//供弹
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
	}
	else if (rc.s[0] == DOWN && rc.s[1] == UP)
	{

	}
	else if (rc.s[0] == DOWN && rc.s[1] == MID)
	{

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

	//具体控制
	if (ctrl.mode[now] != CONTROL::RESET)
	{
		if (ctrl.mode[now] == CONTROL::SEPARATE)//分离模式需要修改遥控器通道
		{
			ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, -rc.ch[3] * para.pitch_speed / 660.f);//云台控制
			ctrl.manual_chassis(rc.ch[1] * para.max_speed / 660.f, 0, rc.ch[0] * para.max_speed / 660.f); //分离模式我们丢弃Y轴方向控制
		}
		else if(ctrl.mode[now] == CONTROL::ROTATION)//小陀螺模式的具体控制
		{
			float RCv_xy = sqrt(pow(rc.ch[1], 2.f) + pow(rc.ch[0], 2.f)) / 660.f;//给小陀螺模式的自旋速度做补偿，计算摇杆推出的距离大小，记录推杆力气不记录方向
			ctrl.manual_chassis(rc.ch[1] * MAXSPEED / 660, -rc.ch[0] * MAXSPEED / 660, para.rota_speed + RCv_xy);//平移会降低转速，于是提前主动增加一点转速来弥补这个损失
			ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, -rc.ch[3] * para.pitch_speed / 660.f);//云台控制
			ctrl.chassis.Keep_Direction();//控制正方向
		}
		else 
		{
			//ctrl.manual_chassis(rc.ch[1] * para.max_speed / 660.f, rc.ch[0] * para.max_speed / 660.f, rc.ch[2] * para.max_speed / 660.f);
		}

		//底盘控制，现在已经被封装到manual_chassis
		//ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.f;
		//ctrl.chassis.speedy = rc.ch[0] * para.max_speed / 660.f;
		//ctrl.chassis.speedz = rc.ch[2] * para.max_speed / 660.f;

		//云台控制
		//ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, rc.ch[3] * para.pitch_speed / 660.f);//控制yaw和pitch，para.yaw_speed是当摇杆推到底时，云台的最大转速，pitch同理

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
